"""Persistent, fail-closed semantic places for known-map Go2 navigation.

Only explicitly confirmed places are stored here. Perception may propose a
place in a later stage, but it must not silently become a navigation target.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from datetime import datetime, timezone
from enum import StrEnum
import json
import os
from pathlib import Path
import tempfile
import threading
import time
from typing import Annotated, Any, Literal, Protocol
import unicodedata
from uuid import uuid4

from dimos_lcm.std_msgs import String
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    field_validator,
)
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.spec.utils import Spec

from .mission_contracts import MissionKind, TaskSpec

_LABEL_MAX_LENGTH = 200
_DEFAULT_STORAGE_PATH = Path("~/.dimos/go2-studio/semantic-world.json")

NonEmptyLabel = Annotated[str, Field(min_length=1, max_length=_LABEL_MAX_LENGTH)]


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _clean_label(value: str) -> str:
    return " ".join(unicodedata.normalize("NFKC", value).strip().split())


def _label_key(value: str) -> str:
    return _clean_label(value).casefold()


class _WorldModel(BaseModel):
    model_config = ConfigDict(
        extra="forbid",
        frozen=True,
        str_strip_whitespace=True,
        validate_default=True,
        allow_inf_nan=False,
    )


class SemanticPlacePose(_WorldModel):
    """Serializable map-frame pose associated with a confirmed place."""

    frame_id: NonEmptyLabel
    ts: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @classmethod
    def from_pose_stamped(cls, pose: PoseStamped) -> SemanticPlacePose:
        """Create a persistent pose from a DimOS navigation pose."""

        return cls(
            frame_id=pose.frame_id,
            ts=float(pose.ts),
            x=float(pose.x),
            y=float(pose.y),
            z=float(pose.z),
            qx=float(pose.orientation.x),
            qy=float(pose.orientation.y),
            qz=float(pose.orientation.z),
            qw=float(pose.orientation.w),
        )

    def to_pose_stamped(self) -> PoseStamped:
        """Restore the DimOS pose passed to the navigation interface."""

        return PoseStamped(
            ts=self.ts,
            frame_id=self.frame_id,
            position=[self.x, self.y, self.z],
            orientation=[self.qx, self.qy, self.qz, self.qw],
        )


class SemanticPlaceDraft(_WorldModel):
    """Validated input accepted at the confirmation boundary."""

    name: NonEmptyLabel
    aliases: tuple[NonEmptyLabel, ...] = ()
    pose: SemanticPlacePose

    @field_validator("name", mode="before")
    @classmethod
    def _normalize_name(cls, value: Any) -> Any:
        return _clean_label(value) if isinstance(value, str) else value

    @field_validator("aliases", mode="before")
    @classmethod
    def _normalize_aliases(cls, value: Any) -> Any:
        if value is None:
            return ()
        if not isinstance(value, (list, tuple)):
            return value
        return tuple(_clean_label(item) if isinstance(item, str) else item for item in value)


class SemanticPlace(_WorldModel):
    """One stable, manually confirmed place in one concrete map version."""

    entity_id: Annotated[
        str,
        Field(
            min_length=8,
            max_length=128,
            pattern=r"^[A-Za-z0-9][A-Za-z0-9_.:-]*$",
        ),
    ]
    name: NonEmptyLabel
    aliases: tuple[NonEmptyLabel, ...] = ()
    map_id: NonEmptyLabel
    map_version: NonEmptyLabel
    pose: SemanticPlacePose
    confirmation: Literal["confirmed"] = "confirmed"
    confirmed_at: datetime
    updated_at: datetime

    @field_validator("confirmed_at", "updated_at")
    @classmethod
    def _timestamps_are_aware_utc(cls, value: datetime) -> datetime:
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("semantic place timestamps must include timezone")
        return value.astimezone(timezone.utc)


class SemanticWorldDocument(_WorldModel):
    """Versioned on-disk document so schema migrations can be explicit."""

    schema_version: Literal[1] = 1
    places: tuple[SemanticPlace, ...] = ()


class ResolutionStatus(StrEnum):
    """Diagnostic result of resolving a semantic destination."""

    RESOLVED = "resolved"
    UNRESOLVED = "unresolved"
    MAP_MISMATCH = "map_mismatch"
    MAP_UNAVAILABLE = "map_unavailable"
    UNSUPPORTED_TASK = "unsupported_task"


class DestinationResolution(_WorldModel):
    """Resolution outcome without issuing any navigation command."""

    status: ResolutionStatus
    place: SemanticPlace | None = None
    reason: str


class SemanticWorldStorageError(RuntimeError):
    """Raised when persistent semantic-place state cannot be trusted."""


class MapIdentityUnavailableError(RuntimeError):
    """Raised when a write has no current map ID and version."""


class SemanticFrameTransformError(RuntimeError):
    """Raised when a semantic pose cannot cross the live map/world TF boundary."""


class DestinationResolverSpec(Spec, Protocol):
    """DimOS module contract consumed by the mission executor."""

    def resolve(self, task: TaskSpec) -> PoseStamped | None: ...


class SemanticWorldConfig(ModuleConfig):
    """Runtime configuration for one persistent semantic world."""

    storage_path: Path = _DEFAULT_STORAGE_PATH
    map_id: str = ""
    map_version: str = ""
    canonical_frame_id: str = ""
    navigation_frame_id: str = ""
    transform_tolerance_s: float = 5.0

    @field_validator("storage_path", mode="before")
    @classmethod
    def _expand_storage_path(cls, value: str | Path) -> Path:
        return Path(value).expanduser()

    @field_validator(
        "map_id",
        "map_version",
        "canonical_frame_id",
        "navigation_frame_id",
        mode="before",
    )
    @classmethod
    def _normalize_map_identity(cls, value: Any) -> Any:
        return _clean_label(value) if isinstance(value, str) else value

    @field_validator("transform_tolerance_s")
    @classmethod
    def _positive_transform_tolerance(cls, value: float) -> float:
        if value <= 0:
            raise ValueError("transform_tolerance_s must be positive")
        return value


class SemanticWorld(Module):
    """Persist and resolve only manually confirmed semantic places."""

    config: SemanticWorldConfig
    semantic_place_candidate: In[String]
    semantic_place_confirmation: Out[String]
    semantic_places_snapshot: Out[String]

    def __init__(
        self,
        *,
        clock: Callable[[], datetime] = _utc_now,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._clock = clock
        self._lock = threading.RLock()
        try:
            self._document = self._load_document()
        except Exception:
            super().stop()
            raise

    @rpc
    def start(self) -> None:
        """Start the world and publish its current-map read model."""

        super().start()
        if self.semantic_place_candidate.transport is not None:
            self.register_disposable(
                Disposable(
                    self.semantic_place_candidate.subscribe(
                        self._confirm_map_candidate
                    )
                )
            )
        self._publish_semantic_places_snapshot()

    def _confirm_map_candidate(self, message: String) -> None:
        """Confirm one operator-clicked point through the canonical boundary."""

        request_id = ""
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("map marker payload must be an object")
            raw_request_id = payload.get("request_id")
            if not isinstance(raw_request_id, str) or not raw_request_id:
                raise ValueError("map marker request_id is required")
            request_id = raw_request_id
            draft = SemanticPlaceDraft.model_validate(payload.get("place"))
            place = self.confirm_place(
                name=draft.name,
                aliases=draft.aliases,
                pose=draft.pose.to_pose_stamped(),
            )
        except (
            MapIdentityUnavailableError,
            SemanticFrameTransformError,
            SemanticWorldStorageError,
            ValidationError,
            ValueError,
            json.JSONDecodeError,
        ) as exc:
            self.semantic_place_confirmation.publish(
                String(
                    self._json(
                        {
                            "request_id": request_id,
                            "accepted": False,
                            "reason": str(exc),
                        }
                    )
                )
            )
            return

        self.semantic_place_confirmation.publish(
            String(
                self._json(
                    {
                        "request_id": request_id,
                        "accepted": True,
                        "place": place.model_dump(mode="json"),
                    }
                )
            )
        )

    def confirm_place(
        self,
        *,
        name: str,
        aliases: Iterable[str],
        pose: PoseStamped,
    ) -> SemanticPlace:
        """Confirm or update one place on the current map.

        Args:
            name: Canonical human-readable place name.
            aliases: Alternative names accepted by the resolver.
            pose: Confirmed map-frame navigation pose.
        """

        map_id, map_version = self._required_map_identity()
        canonical_pose = self._pose_in_frame(
            pose,
            self.config.canonical_frame_id,
            use_pose_timestamp=True,
        )
        draft = SemanticPlaceDraft(
            name=name,
            aliases=tuple(aliases),
            pose=SemanticPlacePose.from_pose_stamped(canonical_pose),
        )
        aliases_clean = self._deduplicate_aliases(draft.name, draft.aliases)
        now = self._aware_utc_now()

        with self._lock:
            existing = self._find_by_canonical_name(
                draft.name,
                map_id=map_id,
                map_version=map_version,
            )
            self._reject_conflicts(
                draft.name,
                aliases_clean,
                map_id=map_id,
                map_version=map_version,
                exclude_entity_id=existing.entity_id if existing is not None else None,
            )
            place = SemanticPlace(
                entity_id=(
                    existing.entity_id
                    if existing is not None
                    else f"place-{uuid4().hex}"
                ),
                name=draft.name,
                aliases=aliases_clean,
                map_id=map_id,
                map_version=map_version,
                pose=draft.pose,
                confirmed_at=(
                    existing.confirmed_at if existing is not None else now
                ),
                updated_at=now,
            )
            places = [
                item
                for item in self._document.places
                if item.entity_id != place.entity_id
            ]
            places.append(place)
            document = SemanticWorldDocument(places=tuple(places))
            self._write_document(document)
            self._document = document
        self._publish_semantic_places_snapshot()
        return place

    @skill
    def confirm_semantic_place(self, place_json: str) -> str:
        """Confirm one semantic place from a strict JSON boundary.

        Args:
            place_json: JSON matching `SemanticPlaceDraft`.
        """

        try:
            draft = SemanticPlaceDraft.model_validate_json(place_json)
            place = self.confirm_place(
                name=draft.name,
                aliases=draft.aliases,
                pose=draft.pose.to_pose_stamped(),
            )
        except (
            MapIdentityUnavailableError,
            SemanticFrameTransformError,
            SemanticWorldStorageError,
            ValidationError,
            ValueError,
        ) as exc:
            return self._json({"accepted": False, "reason": str(exc)})
        return self._json(
            {
                "accepted": True,
                "place": place.model_dump(mode="json"),
            }
        )

    def list_places(self) -> tuple[SemanticPlace, ...]:
        """Return confirmed places for the current map in deterministic order."""

        with self._lock:
            places = tuple(
                place
                for place in self._document.places
                if self._is_current_map(place)
            )
        return tuple(
            sorted(
                places,
                key=lambda place: (_label_key(place.name), place.entity_id),
            )
        )

    @skill
    def list_semantic_places(self) -> str:
        """List confirmed semantic places available on the current map."""

        return self._json(self._semantic_places_payload())

    @rpc
    def resolve(self, task: TaskSpec) -> PoseStamped | None:
        """Resolve a supported task to a current-map pose without moving."""

        result = self.resolve_detail(task)
        if result.status is not ResolutionStatus.RESOLVED or result.place is None:
            return None
        try:
            return self._pose_in_frame(
                result.place.pose.to_pose_stamped(),
                self.config.navigation_frame_id,
                use_pose_timestamp=False,
            )
        except SemanticFrameTransformError:
            return None

    @rpc
    def resolve_detail(self, task: TaskSpec) -> DestinationResolution:
        """Explain semantic resolution while preserving fail-closed behavior."""

        if not self.config.map_id or not self.config.map_version:
            return DestinationResolution(
                status=ResolutionStatus.MAP_UNAVAILABLE,
                reason="current map identity is not configured",
            )
        if task.kind not in {
            MissionKind.GO_TO_PLACE,
            MissionKind.INSPECT_PLACE,
        }:
            return DestinationResolution(
                status=ResolutionStatus.UNSUPPORTED_TASK,
                reason=f"task kind {task.kind.value} is not a known-place mission",
            )
        if not task.destination:
            return DestinationResolution(
                status=ResolutionStatus.UNRESOLVED,
                reason="task has no semantic destination",
            )

        destination_key = _label_key(task.destination)
        with self._lock:
            matches = [
                place
                for place in self._document.places
                if destination_key in self._place_keys(place)
            ]
            current_matches = [
                place for place in matches if self._is_current_map(place)
            ]

        if len(current_matches) == 1:
            return DestinationResolution(
                status=ResolutionStatus.RESOLVED,
                place=current_matches[0],
                reason="confirmed place resolved on current map",
            )
        if matches:
            return DestinationResolution(
                status=ResolutionStatus.MAP_MISMATCH,
                reason="place exists, but not for the current map ID and version",
            )
        return DestinationResolution(
            status=ResolutionStatus.UNRESOLVED,
            reason="no confirmed place matches destination",
        )

    def _load_document(self) -> SemanticWorldDocument:
        path = self.config.storage_path
        if not path.exists():
            return SemanticWorldDocument()
        try:
            document = SemanticWorldDocument.model_validate_json(
                path.read_text(encoding="utf-8")
            )
            self._validate_document_uniqueness(document)
            return document
        except (OSError, ValidationError, ValueError) as exc:
            raise SemanticWorldStorageError(
                f"could not load semantic world from {path}: {exc}"
            ) from exc

    def _write_document(self, document: SemanticWorldDocument) -> None:
        path = self.config.storage_path
        temp_name: str | None = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=path.parent,
                prefix=f".{path.name}.",
                suffix=".tmp",
                delete=False,
            ) as handle:
                temp_name = handle.name
                handle.write(document.model_dump_json(indent=2))
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_name, path)
        except OSError as exc:
            if temp_name is not None:
                try:
                    Path(temp_name).unlink(missing_ok=True)
                except OSError:
                    pass
            raise SemanticWorldStorageError(
                f"could not persist semantic world to {path}: {exc}"
            ) from exc

    @classmethod
    def _validate_document_uniqueness(
        cls,
        document: SemanticWorldDocument,
    ) -> None:
        indexes: dict[tuple[str, str], dict[str, str]] = {}
        for place in document.places:
            index = indexes.setdefault((place.map_id, place.map_version), {})
            for key in cls._place_keys(place):
                other = index.get(key)
                if other is not None and other != place.entity_id:
                    raise ValueError(
                        "semantic world contains conflicting place names or aliases"
                    )
                index[key] = place.entity_id

    def _required_map_identity(self) -> tuple[str, str]:
        if not self.config.map_id or not self.config.map_version:
            raise MapIdentityUnavailableError(
                "current map_id and map_version must be configured"
            )
        return self.config.map_id, self.config.map_version

    def _pose_in_frame(
        self,
        pose: PoseStamped,
        target_frame_id: str,
        *,
        use_pose_timestamp: bool,
    ) -> PoseStamped:
        if not target_frame_id or pose.frame_id == target_frame_id:
            return pose

        transform = self.tf.get(
            target_frame_id,
            pose.frame_id,
            time_point=pose.ts if use_pose_timestamp else None,
            time_tolerance=self.config.transform_tolerance_s,
        )
        if transform is None:
            raise SemanticFrameTransformError(
                "could not transform semantic pose "
                f"from {pose.frame_id!r} to {target_frame_id!r}; "
                "relocalization TF is unavailable"
            )

        transformed = transform + Transform.from_pose("semantic_place", pose)
        return PoseStamped(
            ts=pose.ts if use_pose_timestamp else time.time(),
            frame_id=target_frame_id,
            position=transformed.translation,
            orientation=transformed.rotation,
        )

    def _aware_utc_now(self) -> datetime:
        value = self._clock()
        if value.tzinfo is None or value.utcoffset() is None:
            raise ValueError("semantic world clock must return a timezone-aware datetime")
        return value.astimezone(timezone.utc)

    def _semantic_places_payload(self) -> dict[str, Any]:
        return {
            "map_id": self.config.map_id,
            "map_version": self.config.map_version,
            "places": [
                place.model_dump(mode="json") for place in self.list_places()
            ],
        }

    def _publish_semantic_places_snapshot(self) -> None:
        self.semantic_places_snapshot.publish(
            String(self._json(self._semantic_places_payload()))
        )

    def _is_current_map(self, place: SemanticPlace) -> bool:
        return (
            place.map_id == self.config.map_id
            and place.map_version == self.config.map_version
        )

    def _find_by_canonical_name(
        self,
        name: str,
        *,
        map_id: str,
        map_version: str,
    ) -> SemanticPlace | None:
        name_key = _label_key(name)
        for place in self._document.places:
            if (
                place.map_id == map_id
                and place.map_version == map_version
                and _label_key(place.name) == name_key
            ):
                return place
        return None

    def _reject_conflicts(
        self,
        name: str,
        aliases: tuple[str, ...],
        *,
        map_id: str,
        map_version: str,
        exclude_entity_id: str | None,
    ) -> None:
        requested_keys = {_label_key(name), *(_label_key(alias) for alias in aliases)}
        for place in self._document.places:
            if (
                place.map_id != map_id
                or place.map_version != map_version
                or place.entity_id == exclude_entity_id
            ):
                continue
            conflicts = requested_keys.intersection(self._place_keys(place))
            if conflicts:
                raise ValueError(
                    f"semantic place name or alias conflicts with {place.name!r}"
                )

    @staticmethod
    def _deduplicate_aliases(
        name: str,
        aliases: tuple[str, ...],
    ) -> tuple[str, ...]:
        name_key = _label_key(name)
        seen = {name_key}
        result: list[str] = []
        for alias in aliases:
            key = _label_key(alias)
            if key in seen:
                continue
            seen.add(key)
            result.append(alias)
        return tuple(result)

    @staticmethod
    def _place_keys(place: SemanticPlace) -> set[str]:
        return {
            _label_key(place.name),
            *(_label_key(alias) for alias in place.aliases),
        }

    @staticmethod
    def _json(payload: dict[str, Any]) -> str:
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

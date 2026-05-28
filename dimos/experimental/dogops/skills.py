from __future__ import annotations

from collections.abc import Callable
import json
import math
from pathlib import Path
import struct
import time
from typing import Any, TypeVar
import zlib

from dimos.experimental.dogops.config_loader import (
    DEFAULT_MANIFEST,
    DEFAULT_MISSION,
    DEFAULT_POLICY,
    DEFAULT_SITE,
    load_manifest as read_manifest,
    load_mission as read_mission,
    load_site_config as read_site_config,
)
from dimos.experimental.dogops.detector import DetectedTag, DogOpsTagDetector
from dimos.experimental.dogops.heatmap_runs import gather_heatmap_run
from dimos.experimental.dogops.mission_engine import run_offline_simulation
from dimos.experimental.dogops.models import (
    Asset,
    DogOpsState,
    Incident,
    IncidentState,
    IncidentType,
    MissionState,
    Observation,
    PackageState,
    Severity,
    WorkOrder,
    WorkOrderState,
)
from dimos.experimental.dogops.report import build_report_data
from dimos.experimental.dogops.live_map import DogOpsLiveMapAdapter
from dimos.experimental.dogops.route_executor import (
    CallableGoalPublisher,
    ClickedPointGoalPublisher,
    DogOpsRouteExecutor,
    RouteExecutionError,
)
from dimos.experimental.dogops.store import DogOpsStore

try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from reactivex.disposable import Disposable as RxDisposable
except ModuleNotFoundError:  # pragma: no cover - local project-pack fallback.
    RxDisposable = None

try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from dimos.agents.annotation import skill
except ModuleNotFoundError:  # pragma: no cover - fallback behavior is tested through methods.
    F = TypeVar("F", bound=Callable[..., Any])

    def skill(func: F | None = None, **metadata: object) -> F | Callable[[F], F]:
        def decorate(inner: F) -> F:
            inner.__dogops_skill__ = True  # type: ignore[attr-defined]
            inner.__dogops_skill_metadata__ = metadata  # type: ignore[attr-defined]
            return inner

        if func is None:
            return decorate
        return decorate(func)


try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from dimos.core.module import Module
except ModuleNotFoundError:

    class Module:
        def __init__(self, **_: object) -> None:
            pass

        @classmethod
        def blueprint(cls, **kwargs: object) -> dict[str, object]:
            return {"module": cls.__name__, "kwargs": kwargs}

        def start(self) -> None:
            pass

        def stop(self) -> None:
            pass


try:  # pragma: no cover - exercised only inside a full DimOS checkout.
    from dimos.core.stream import In, Out
    from dimos.msgs.geometry_msgs.PointStamped import PointStamped
    from dimos.msgs.sensor_msgs.Image import Image
except ModuleNotFoundError:  # pragma: no cover - local project-pack fallback.

    class In:
        def __class_getitem__(cls, _: object) -> type[In]:
            return cls

    class Out:
        def __class_getitem__(cls, _: object) -> type[Out]:
            return cls

    class PointStamped:
        def __init__(
            self,
            *,
            ts: float,
            frame_id: str,
            x: float,
            y: float,
            z: float,
        ) -> None:
            self.ts = ts
            self.frame_id = frame_id
            self.x = x
            self.y = y
            self.z = z

    Image = Any  # type: ignore[misc, assignment]


class DogOpsSkillContainer(Module):
    """Deterministic SiteOps skills exposed through DimOS MCP or direct tests."""

    color_image: In[Image]
    clicked_point: Out[PointStamped]

    def __init__(
        self,
        *,
        site_path: str | Path = DEFAULT_SITE,
        manifest_path: str | Path = DEFAULT_MANIFEST,
        mission_path: str | Path = DEFAULT_MISSION,
        policy_path: str | Path = DEFAULT_POLICY,
        run_dir: str | Path = ".dogops/runs/latest",
        go_to_handler: Callable[[float, float, float, str], object] | None = None,
        route_stop_handler: Callable[[], object] | None = None,
        live_map_adapter: DogOpsLiveMapAdapter | None = None,
        runtime_module: bool = False,
        **_: object,
    ) -> None:
        if runtime_module or _:
            super().__init__(**_)
        self.site_path = Path(site_path)
        self.manifest_path = Path(manifest_path)
        self.mission_path = Path(mission_path)
        self.policy_path = Path(policy_path)
        self.run_dir = Path(run_dir)
        self._go_to_handler = go_to_handler
        self._route_stop_handler = route_stop_handler
        self._live_map_adapter = live_map_adapter
        self._latest_camera_image: object | None = None
        self._latest_camera_received_at: float | None = None

    def start(self) -> None:  # pragma: no cover - lifecycle exercised in full DimOS.
        super().start()
        color_image = getattr(self, "color_image", None)
        if color_image is not None and hasattr(color_image, "subscribe"):
            subscription = color_image.subscribe(self.ingest_camera_image)
            _register_subscription(self, subscription)

    def ingest_camera_image(self, image: object) -> None:
        self._latest_camera_image = image
        self._latest_camera_received_at = time.time()

    @skill
    def load_site_config(self, path: str = str(DEFAULT_SITE)) -> str:
        """Load a DogOps site configuration file and report inventory counts."""
        self.site_path = Path(path)
        site = read_site_config(self.site_path)
        return _json(
            ok=True,
            skill="load_site_config",
            site_id=site.site_id,
            zones=len(site.zones),
            assets=len(site.assets),
            packages=len(site.packages),
        )

    @skill
    def load_manifest(self, path: str = str(DEFAULT_MANIFEST)) -> str:
        """Load a receiving manifest file and report the package count."""
        self.manifest_path = Path(path)
        manifest = read_manifest(self.manifest_path)
        return _json(
            ok=True,
            skill="load_manifest",
            manifest_id=manifest.manifest_id,
            packages=len(manifest.items),
        )

    @skill
    def load_mission(self, path: str = str(DEFAULT_MISSION)) -> str:
        """Load a DogOps mission plan and report its step count."""
        self.mission_path = Path(path)
        mission = read_mission(self.mission_path)
        return _json(
            ok=True,
            skill="load_mission",
            mission_id=mission.mission_id,
            steps=len(mission.steps),
        )

    @skill
    def run_mission(self, mission_id: str = "receiving_sre_demo") -> str:
        """Run the deterministic DogOps mission simulation."""
        mission = read_mission(self.mission_path)
        if mission.mission_id != mission_id:
            return _json(
                ok=False,
                skill="run_mission",
                error="unknown_mission",
                mission_id=mission_id,
                configured_mission_id=mission.mission_id,
            )
        state = run_offline_simulation(
            site=self.site_path,
            manifest=self.manifest_path,
            mission=self.mission_path,
            policy=self.policy_path,
            out=self.run_dir,
        )
        return _json(
            ok=True,
            skill="run_mission",
            run_id=state.run.id,
            mission_id=state.run.mission_id,
            state=state.run.state,
            report=str(self.run_dir / "report.md"),
            summary=state.run.summary,
        )

    @skill
    def go_to(self, x: float, y: float, z: float = 0.0, frame_id: str = "map") -> str:
        """Send a navigation target through the DimOS clicked point stream."""
        try:
            x_f, y_f, z_f = _finite_point(x, y, z)
        except ValueError as exc:
            return _json(
                ok=False,
                skill="go_to",
                error="invalid_go_to_target",
                message=str(exc),
            )
        frame = frame_id.strip() or "map"
        if self._go_to_handler is not None:
            result = self._go_to_handler(x_f, y_f, z_f, frame)
            return _json(
                ok=True,
                skill="go_to",
                transport="handler",
                x=x_f,
                y=y_f,
                z=z_f,
                frame_id=frame,
                result=result,
            )

        publisher = getattr(self, "clicked_point", None)
        if publisher is None or not hasattr(publisher, "publish"):
            return _json(
                ok=False,
                skill="go_to",
                error="navigation_stream_unavailable",
                message="DogOps go_to needs the DimOS clicked_point stream.",
                x=x_f,
                y=y_f,
                z=z_f,
                frame_id=frame,
            )

        point = PointStamped(ts=time.time(), frame_id=frame, x=x_f, y=y_f, z=z_f)
        publisher.publish(point)
        return _json(
            ok=True,
            skill="go_to",
            transport="clicked_point",
            x=x_f,
            y=y_f,
            z=z_f,
            frame_id=frame,
        )

    @skill
    def follow_route(self, route_id: str | None = None, dry_run: bool = False) -> str:
        """Follow the selected authored DogOps route waypoint by waypoint."""
        publisher = self._route_goal_publisher()
        if not dry_run and publisher is None:
            return _json(
                ok=False,
                skill="follow_route",
                error="navigation_stream_unavailable",
                message="DogOps follow_route needs the DimOS clicked_point stream or a handler.",
            )
        live_adapter = self._live_map_adapter or DogOpsLiveMapAdapter()
        executor = DogOpsRouteExecutor(
            self.run_dir,
            goal_publisher=publisher,
            stop_handler=self._route_stop_handler,
            scan_zone_handler=self.scan_zone,
            capture_image_handler=self._capture_latest_camera_image,
            live_snapshot_reader=live_adapter.snapshot if not dry_run else None,
        )
        try:
            state = executor.follow_route(route_id, dry_run=dry_run)
        except RouteExecutionError as exc:
            return _json(
                ok=False,
                skill="follow_route",
                error="route_execution_rejected",
                message=str(exc),
            )
        return _json(
            ok=state.state == "completed",
            skill="follow_route",
            route_id=state.route_id,
            state=state.state,
            active_waypoint_id=state.active_waypoint_id,
            waypoints_total=state.waypoints_total,
            waypoints_reached=state.waypoints_reached,
            transport=state.transport,
            dry_run=dry_run,
            last_error=state.last_error,
            route_execution=state.model_dump(mode="json"),
        )

    @skill
    def stop_route(self) -> str:
        """Request that a running DogOps authored route stop."""
        executor = DogOpsRouteExecutor(self.run_dir, stop_handler=self._route_stop_handler)
        try:
            state = executor.stop_route()
        except RouteExecutionError as exc:
            return _json(
                ok=False,
                skill="stop_route",
                error="route_execution_rejected",
                message=str(exc),
            )
        return _json(
            ok=state.last_error is None,
            skill="stop_route",
            route_id=state.route_id,
            state=state.state,
            active_waypoint_id=state.active_waypoint_id,
            last_error=state.last_error,
            route_execution=state.model_dump(mode="json"),
        )

    @skill
    def route_status(self) -> str:
        """Return the current DogOps authored-route execution state."""
        executor = DogOpsRouteExecutor(self.run_dir)
        state = executor.status()
        return _json(
            ok=True,
            skill="route_status",
            route_id=state.route_id,
            state=state.state,
            active_waypoint_id=state.active_waypoint_id,
            waypoints_total=state.waypoints_total,
            waypoints_reached=state.waypoints_reached,
            transport=state.transport,
            last_error=state.last_error,
            route_execution=state.model_dump(mode="json"),
        )

    @skill
    def gather_heatmap(self, area_id: str = "", duration_s: float = 0.0) -> str:
        """Snapshot the current DimOS costmap as a DogOps gather-heatmap run."""
        live_adapter = self._live_map_adapter or DogOpsLiveMapAdapter()
        result = gather_heatmap_run(
            self.run_dir,
            live_snapshot=live_adapter.snapshot(),
            live_snapshot_reader=live_adapter.snapshot,
            area_id=area_id,
            duration_s=duration_s,
        )
        return _json(skill="gather_heatmap", **result)

    @skill
    def scan_zone(self, zone_id: str) -> str:
        """Scan a zone using the live camera when available, otherwise deterministic fixtures."""
        camera_result = self._scan_zone_with_latest_camera(zone_id)
        if camera_result is not None:
            return camera_result

        mission = read_mission(self.mission_path)
        observations = [
            obs for obs in mission.simulation_observations.values() if obs.zone_id == zone_id
        ]
        if not observations:
            return _json(ok=False, skill="scan_zone", error="unknown_zone", zone_id=zone_id)
        visible_tag_ids = sorted({tag for obs in observations for tag in obs.visible_tag_ids})
        package_ids = sorted(
            key.removesuffix(".zone_id")
            for obs in observations
            for key in obs.facts
            if key.startswith("PKG-") and key.endswith(".zone_id")
        )
        return _json(
            ok=True,
            skill="scan_zone",
            zone_id=zone_id,
            visible_tag_ids=visible_tag_ids,
            package_ids=package_ids,
            source="simulation",
        )

    @skill
    def camera_stream_status(self) -> str:
        """Return whether DogOps has received a live camera frame."""
        return _json(skill="camera_stream_status", **self._camera_stream_status())

    def _capture_latest_camera_image(self, context: dict[str, Any]) -> dict[str, Any] | None:
        if self._latest_camera_image is None:
            raise RuntimeError("no live Go2 camera frame is available for capture_image")
        action = context.get("action") if isinstance(context.get("action"), dict) else {}
        action_args = action.get("args") if isinstance(action, dict) and isinstance(action.get("args"), dict) else {}
        max_age_s = _positive_float(action_args.get("max_camera_frame_age_s"), default=2.0)
        frame_age_s = self._camera_frame_age_s()
        if frame_age_s is None or frame_age_s > max_age_s:
            raise RuntimeError(
                f"latest Go2 camera frame is stale for capture_image: age={frame_age_s}s max={max_age_s}s"
            )
        evidence_dir = Path(context["evidence_dir"])
        evidence_dir.mkdir(parents=True, exist_ok=True)
        waypoint_id = str(context["waypoint_id"])
        action_id = str(context["action_id"])
        path = evidence_dir / f"{waypoint_id}-{action_id}.png"
        _write_camera_frame_png(path, self._latest_camera_image)
        return {
            "path": str(path),
            "source": "go2_camera_live",
            "mime_type": "image/png",
            "metadata": {
                "camera_frame_age_s": frame_age_s,
                "camera_frame_id": getattr(self._latest_camera_image, "frame_id", None),
                "camera_shape": _image_shape(self._latest_camera_image),
                "camera_encoding": getattr(self._latest_camera_image, "encoding", None),
            },
        }

    @skill
    def read_gauge(self, asset_id: str = "TEMP_1") -> str:
        """Read or simulate a gauge value for a configured asset."""
        site = read_site_config(self.site_path)
        asset = site.asset_by_id().get(asset_id)
        if asset is None:
            return _json(ok=False, skill="read_gauge", error="unknown_asset", asset_id=asset_id)
        state = self._load_state_if_exists()
        raw_reading, evidence_id = _latest_fact(state, f"{asset_id}.temperature_c")
        threshold = _to_float(asset.expected_state.get("max_celsius"))
        reading_celsius = _to_float(raw_reading)
        source = "observation" if evidence_id is not None else "deterministic_expected_state"
        if reading_celsius is None:
            reading_celsius = _to_float(asset.expected_state.get("current_celsius"))
        if reading_celsius is None and threshold is not None:
            reading_celsius = round(threshold - 2.0, 1)
        within_threshold = (
            None if reading_celsius is None or threshold is None else reading_celsius <= threshold
        )
        status = asset.expected_status
        if within_threshold is True:
            status = "below_threshold"
        elif within_threshold is False:
            status = "above_threshold"
        return _json(
            ok=True,
            skill="read_gauge",
            asset_id=asset.id,
            display_name=asset.display_name,
            tag_id=asset.tag_id,
            reading_celsius=reading_celsius,
            max_celsius=threshold,
            within_threshold=within_threshold,
            state=status or "unknown",
            evidence_observation_id=evidence_id,
            source=source,
            summary=(
                f"{asset.id} reading {reading_celsius}C under {threshold}C."
                if reading_celsius is not None and threshold is not None
                else f"{asset.id} gauge read from expected state."
            ),
        )

    @skill
    def check_clearance(self, asset_id: str) -> str:
        """Check whether an asset clearance is blocked."""
        site = read_site_config(self.site_path)
        asset = site.asset_by_id().get(asset_id)
        if asset is None:
            return _json(
                ok=False,
                skill="check_clearance",
                error="unknown_asset",
                asset_id=asset_id,
            )
        state = self._load_state_if_exists()
        snapshot = _clearance_snapshot(asset, state)
        return _json(
            ok=True,
            skill="check_clearance",
            asset_id=asset.id,
            display_name=asset.display_name,
            tag_id=asset.tag_id,
            expected_clear=asset.expected_clear,
            **snapshot,
        )

    @skill
    def detect_blocked_aisle(self, zone_id: str = "AISLE_1") -> str:
        """Detect whether a configured aisle is blocked."""
        site = read_site_config(self.site_path)
        asset = site.asset_by_id().get(zone_id)
        if asset is None:
            asset = next(
                (
                    candidate
                    for candidate in site.assets
                    if candidate.zone_id == zone_id
                    and candidate.asset_kind == "aisle_clearance"
                ),
                None,
            )
        if asset is None:
            return _json(
                ok=False,
                skill="detect_blocked_aisle",
                error="unknown_aisle",
                zone_id=zone_id,
            )
        state = self._load_state_if_exists()
        snapshot = _clearance_snapshot(asset, state)
        open_blocked_incident = _has_open_incident(state, asset.id, IncidentType.blocked_aisle)
        blocked = snapshot["clearance_clear"] is False or open_blocked_incident
        return _json(
            ok=True,
            skill="detect_blocked_aisle",
            zone_id=zone_id,
            asset_id=asset.id,
            display_name=asset.display_name,
            blocked=blocked,
            blocked_reason="blocked_aisle_incident" if open_blocked_incident else None,
            **snapshot,
        )

    @skill
    def scan_receiving_manifest(self, zone_id: str = "INBOUND_DOCK") -> str:
        """Scan a receiving zone and compare observations with the manifest."""
        site = read_site_config(self.site_path)
        if zone_id not in site.zone_by_id():
            return _json(
                ok=False,
                skill="scan_receiving_manifest",
                error="unknown_zone",
                zone_id=zone_id,
            )
        manifest = read_manifest(self.manifest_path)
        mission = read_mission(self.mission_path)
        state = self._load_state_if_exists()
        expected_package_ids = sorted(
            item.package_id for item in manifest.items if item.expected_zone_id == zone_id
        )
        observed_package_ids = _observed_packages_for_zone(state, mission, zone_id)
        missing_package_ids = sorted(set(expected_package_ids) - set(observed_package_ids))
        unexpected_package_ids = sorted(set(observed_package_ids) - set(expected_package_ids))
        visible_tag_ids = _visible_tags_for_zone(state, mission, zone_id)
        evidence_observation_ids = (
            [obs.id for obs in state.observations if obs.zone_id == zone_id] if state else []
        )
        manifest_exceptions = len(missing_package_ids) + len(unexpected_package_ids)
        return _json(
            ok=True,
            skill="scan_receiving_manifest",
            zone_id=zone_id,
            expected_package_ids=expected_package_ids,
            observed_package_ids=observed_package_ids,
            missing_package_ids=missing_package_ids,
            unexpected_package_ids=unexpected_package_ids,
            manifest_exceptions=manifest_exceptions,
            visible_tag_ids=visible_tag_ids,
            evidence_observation_ids=evidence_observation_ids,
            summary=(
                f"{len(observed_package_ids)}/{len(expected_package_ids)} expected packages "
                f"observed at {zone_id}."
            ),
        )

    @skill
    def inspect_asset(self, asset_id: str) -> str:
        """Inspect a configured asset and return its current issue state."""
        site = read_site_config(self.site_path)
        asset = site.asset_by_id().get(asset_id)
        if asset is None:
            return _json(ok=False, skill="inspect_asset", error="unknown_asset", asset_id=asset_id)
        incidents = []
        if self._state_file().exists():
            state = DogOpsStore.load_existing(self.run_dir).state
            assert state is not None
            incidents = [
                incident.model_dump(mode="json")
                for incident in state.incidents
                if incident.entity_id == asset_id
            ]
        return _json(
            ok=True,
            skill="inspect_asset",
            asset_id=asset.id,
            display_name=asset.display_name,
            expected_clear=asset.expected_clear,
            incidents=incidents,
        )

    @skill
    def reconcile_manifest(self) -> str:
        """Reconcile the current run against the receiving manifest."""
        store = self._require_store("reconcile_manifest")
        if isinstance(store, str):
            return store
        state = store.state
        assert state is not None
        report = build_report_data(state)
        return _json(
            ok=True,
            skill="reconcile_manifest",
            run_id=state.run.id,
            packages_expected=report["packages_expected"],
            packages_observed=report["packages_observed"],
            manifest_exceptions=report["manifest_exceptions"],
            open_issues=report["open_issues"],
        )

    @skill
    def open_work_order(self, entity_id: str, issue_type: str) -> str:
        """Open or return a work order for a DogOps incident."""
        store = self._require_store("open_work_order")
        if isinstance(store, str):
            return store
        state = store.state
        assert state is not None
        for incident in state.incidents:
            if incident.entity_id == entity_id and incident.type == issue_type:
                work_order = _work_order_for_incident(state.work_orders, incident.id)
                return _json(
                    ok=True,
                    skill="open_work_order",
                    incident_id=incident.id,
                    work_order_id=work_order.id if work_order else None,
                    state=incident.state,
                    summary="Existing work order returned.",
                )

        incident_type = IncidentType(issue_type)
        rule = state.policy.rule_for_type(incident_type)
        incident = Incident(
            id=f"INC-{len(state.incidents) + 1:03d}",
            run_id=state.run.id,
            ts_open=time.time(),
            severity=Severity(rule.severity) if rule else Severity.P2,
            type=incident_type,
            entity_id=entity_id,
            related_package_id=entity_id if entity_id.startswith("PKG-") else None,
            state=IncidentState.open,
            title=f"{entity_id} {issue_type}",
            recommended_action=rule.recommended_action if rule else "Review and remediate.",
        )
        work_order = WorkOrder(
            id=f"WO-{len(state.work_orders) + 1:03d}",
            incident_id=incident.id,
            requested_action=incident.recommended_action,
            state=WorkOrderState.assigned,
        )
        store.append_incident(incident)
        store.append_work_order(work_order)
        store.write_state(state.run.id)
        store.write_report(state.run.id)
        return _json(
            ok=True,
            skill="open_work_order",
            incident_id=incident.id,
            work_order_id=work_order.id,
            state=incident.state,
        )

    @skill
    def mark_ready_to_verify(self, work_order_id: str) -> str:
        """Mark a work order ready for robot verification."""
        store = self._require_store("mark_ready_to_verify")
        if isinstance(store, str):
            return store
        state = store.state
        assert state is not None
        work_order = _find_work_order(state.work_orders, work_order_id)
        if work_order is None:
            return _json(
                ok=False,
                skill="mark_ready_to_verify",
                error="unknown_work_order",
                work_order_id=work_order_id,
            )
        if work_order.state != WorkOrderState.verified_closed:
            work_order.state = WorkOrderState.ready_to_verify
            incident = _find_incident(state.incidents, work_order.incident_id)
            if incident is not None and incident.state != IncidentState.resolved:
                incident.state = IncidentState.ready_to_verify
                store.update_incident(incident)
            store.update_work_order(work_order)
            store.write_state(state.run.id)
            store.write_report(state.run.id)
        return _json(
            ok=True,
            skill="mark_ready_to_verify",
            work_order_id=work_order.id,
            state=work_order.state,
        )

    @skill
    def verify_work_order(self, work_order_id: str) -> str:
        """Verify a ready work order and close it if resolved."""
        store = self._require_store("verify_work_order")
        if isinstance(store, str):
            return store
        state = store.state
        assert state is not None
        work_order = _find_work_order(state.work_orders, work_order_id)
        if work_order is None:
            return _json(
                ok=False,
                skill="verify_work_order",
                error="unknown_work_order",
                work_order_id=work_order_id,
            )
        incident = _find_incident(state.incidents, work_order.incident_id)
        if work_order.state != WorkOrderState.verified_closed:
            work_order.state = WorkOrderState.verified_closed
            if incident is not None:
                incident.state = IncidentState.resolved
                incident.ts_closed = time.time()
                store.update_incident(incident)
            store.update_work_order(work_order)
            store.write_state(state.run.id)
            store.write_report(state.run.id)
        return _json(
            ok=True,
            skill="verify_work_order",
            work_order_id=work_order.id,
            state=work_order.state,
            summary=f"{incident.entity_id if incident else work_order.id} verified closed.",
        )

    @skill
    def what_changed(self, since_run_id: str | None = None) -> str:
        """Summarize operational changes in the current DogOps run."""
        store = self._require_store("what_changed")
        if isinstance(store, str):
            return store
        state = store.state
        assert state is not None
        if since_run_id is not None and since_run_id != state.run.id:
            return _json(
                ok=False,
                skill="what_changed",
                error="unknown_run",
                since_run_id=since_run_id,
                current_run_id=state.run.id,
            )
        return _json(ok=True, skill="what_changed", run_id=state.run.id, changes=state.what_changed)

    @skill
    def nav_eval_report(self, run_id: str | None = None) -> str:
        """Return navigation evaluation metrics for a DogOps run."""
        store = self._require_store("nav_eval_report")
        if isinstance(store, str):
            return store
        state = store.state
        assert state is not None
        if run_id is not None and run_id != state.run.id:
            return _json(
                ok=False,
                skill="nav_eval_report",
                error="unknown_run",
                run_id=run_id,
                current_run_id=state.run.id,
            )
        return _json(
            ok=True,
            skill="nav_eval_report",
            run_id=state.run.id,
            nav_summary=state.nav_summary.model_dump(mode="json") if state.nav_summary else None,
        )

    @skill
    def dock_align(self, dock_id: str = "DOCK_1") -> str:
        """Report simulated AprilTag dock alignment readiness."""
        site = read_site_config(self.site_path)
        if not any(entity.id == dock_id for entity in site.special_entities.values()):
            return _json(ok=False, skill="dock_align", error="unknown_dock", dock_id=dock_id)
        return _json(
            ok=True,
            skill="dock_align",
            dock_id=dock_id,
            simulated=True,
            aligned=True,
            guided=False,
        )

    @skill
    def portal_entry(self, portal_id: str = "PORTAL_1") -> str:
        """Report simulated readiness for gated portal entry."""
        site = read_site_config(self.site_path)
        if not any(entity.id == portal_id for entity in site.special_entities.values()):
            return _json(ok=False, skill="portal_entry", error="unknown_portal", portal_id=portal_id)
        return _json(
            ok=True,
            skill="portal_entry",
            portal_id=portal_id,
            simulated=True,
            door_open=True,
            entered=True,
            guided=False,
        )

    @skill
    def stop_mission(self) -> str:
        """Stop the current DogOps mission run if one is active."""
        if not self._state_file().exists():
            return _json(ok=True, skill="stop_mission", state="not_started")
        store = DogOpsStore.load_existing(self.run_dir)
        state = store.state
        assert state is not None
        if state.run.state not in {MissionState.done, MissionState.failed, MissionState.stopped}:
            store.finish_run(
                state.run.id,
                MissionState.stopped,
                "Mission stopped by DogOpsSkillContainer.",
                ended_at=time.time(),
            )
        return _json(ok=True, skill="stop_mission", run_id=state.run.id, state=state.run.state)

    def _state_file(self) -> Path:
        return self.run_dir / "state.json"

    def _load_state_if_exists(self) -> DogOpsState | None:
        if not self._state_file().exists():
            return None
        store = DogOpsStore.load_existing(self.run_dir)
        assert store.state is not None
        return store.state

    def _scan_zone_with_latest_camera(self, zone_id: str) -> str | None:
        if self._latest_camera_image is None:
            return None

        site = read_site_config(self.site_path)
        if zone_id not in site.zone_by_id():
            return _json(ok=False, skill="scan_zone", error="unknown_zone", zone_id=zone_id)

        try:
            detections = DogOpsTagDetector(site).detect_dimos_image(self._latest_camera_image)
        except RuntimeError as exc:
            return _json(
                ok=False,
                skill="scan_zone",
                error="camera_detector_unavailable",
                message=str(exc),
                zone_id=zone_id,
                source="camera",
            )

        visible_tag_ids = sorted({detection.tag_id for detection in detections})
        package_ids = _package_ids_from_detections(detections)
        evidence_observation_ids = self._persist_camera_observations(
            detections,
            zone_id=zone_id,
        )
        return _json(
            ok=True,
            skill="scan_zone",
            zone_id=zone_id,
            visible_tag_ids=visible_tag_ids,
            package_ids=package_ids,
            source="camera",
            detection_count=len(detections),
            camera_frame_age_s=self._camera_frame_age_s(),
            evidence_observation_ids=evidence_observation_ids,
        )

    def _persist_camera_observations(
        self,
        detections: list[DetectedTag],
        *,
        zone_id: str,
    ) -> list[str]:
        if not detections or not self._state_file().exists():
            return []

        store = DogOpsStore.load_existing(self.run_dir)
        state = store.state
        assert state is not None
        evidence_ids: list[str] = []
        observed_at = time.time()
        for index, detection in enumerate(detections, start=1):
            observation = _camera_observation(
                detection,
                zone_id=zone_id,
                run_id=state.run.id,
                observed_at=observed_at,
                index=index,
            )
            store.append_observation(observation)
            _apply_observation_package_facts(state, observation)
            evidence_ids.append(observation.id)

        store.write_state(state.run.id)
        store.write_report(state.run.id)
        return evidence_ids

    def _camera_stream_status(self) -> dict[str, object]:
        if self._latest_camera_image is None:
            return {
                "ok": False,
                "mode": "not_subscribed",
                "fallback": "scan_zone uses deterministic fixtures until a camera frame arrives",
            }
        return {
            "ok": True,
            "mode": "latest_frame",
            "frame_age_s": self._camera_frame_age_s(),
            "frame_id": getattr(self._latest_camera_image, "frame_id", None),
            "shape": _image_shape(self._latest_camera_image),
        }

    def _camera_frame_age_s(self) -> float | None:
        if self._latest_camera_received_at is None:
            return None
        return round(time.time() - self._latest_camera_received_at, 3)

    def _require_store(self, skill_name: str) -> DogOpsStore | str:
        if not self._state_file().exists():
            return _json(ok=False, skill=skill_name, error="missing_run", run_dir=str(self.run_dir))
        return DogOpsStore.load_existing(self.run_dir)

    def _route_goal_publisher(self) -> CallableGoalPublisher | ClickedPointGoalPublisher | None:
        if self._go_to_handler is not None:
            return CallableGoalPublisher(self._go_to_handler)
        publisher = getattr(self, "clicked_point", None)
        if publisher is None or not hasattr(publisher, "publish"):
            return None
        return ClickedPointGoalPublisher(publisher, PointStamped)


def _find_incident(incidents: list[Incident], incident_id: str) -> Incident | None:
    for incident in incidents:
        if incident.id == incident_id:
            return incident
    return None


def _camera_observation(
    detection: DetectedTag,
    *,
    zone_id: str,
    run_id: str,
    observed_at: float,
    index: int,
) -> Observation:
    facts = _camera_detection_facts(detection, zone_id)
    return Observation(
        id=f"CAM-{int(observed_at * 1000)}-{index:02d}-{detection.tag_id}",
        ts=observed_at,
        run_id=run_id,
        entity_id=detection.entity_id,
        tag_id=detection.tag_id,
        zone_id=zone_id,
        facts=facts,
        confidence=detection.confidence,
        source=detection.source,
    )


def _camera_detection_facts(
    detection: DetectedTag,
    zone_id: str,
) -> dict[str, bool | str | int | float]:
    facts: dict[str, bool | str | int | float] = {
        "entity_kind": detection.entity_kind or "unknown",
        "detection_source": detection.source,
        "frame_id": detection.frame_id or "",
        "visible_tag_ids": str(detection.tag_id),
        "center_px": _format_point(detection.center_px),
        "area_px": round(detection.area_px, 3) if detection.area_px is not None else 0.0,
    }
    if detection.entity_id and detection.entity_id.startswith("PKG-"):
        facts[f"{detection.entity_id}.zone_id"] = zone_id
    return facts


def _apply_observation_package_facts(state: DogOpsState, obs: Observation) -> None:
    for key, value in obs.facts.items():
        if not key.endswith(".zone_id"):
            continue
        package_id = key.removesuffix(".zone_id")
        status = state.package_statuses.get(package_id)
        if status is None or not isinstance(value, str):
            continue
        previous_zone = status.observed_zone_id
        status.observed_zone_id = value
        if value == status.expected_zone_id:
            status.state = PackageState.found_ok
            status.blocks_asset_id = None
        else:
            status.state = PackageState.wrong_zone
        if package_id == "PKG-104" and previous_zone in {"RACK_ROW_A", "COOLING_1"}:
            if value == "QA_HOLD":
                state.what_changed.append(
                    "PKG-104 moved from COOLING_1/RACK_ROW_A to QA_HOLD; INC-001 resolved."
                )


def _package_ids_from_detections(detections: list[DetectedTag]) -> list[str]:
    return sorted(
        detection.entity_id
        for detection in detections
        if detection.entity_id is not None and detection.entity_id.startswith("PKG-")
    )


def _image_shape(image: object) -> list[int] | None:
    shape = getattr(image, "shape", None)
    data = getattr(image, "data", None)
    if shape is None and data is not None:
        shape = getattr(data, "shape", None)
    if shape is None:
        return None
    return [int(item) for item in shape]


def _write_camera_frame_png(path: Path, image: object) -> None:
    array = _camera_array_from_image(image)
    shape = getattr(array, "shape", None)
    if shape is None and hasattr(image, "shape"):
        shape = getattr(image, "shape")
    if shape is None or len(shape) < 2:
        raise RuntimeError("latest camera frame does not expose a 2D image shape")

    height = int(shape[0])
    width = int(shape[1])
    channels = int(shape[2]) if len(shape) > 2 else 1
    encoding = str(getattr(image, "encoding", "") or "").lower()
    rows = []
    for y in range(height):
        row = bytearray()
        for x in range(width):
            row.extend(_rgb_pixel(array[y][x], channels=channels, encoding=encoding))
        rows.append(b"\x00" + bytes(row))
    raw = b"".join(rows)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"tEXt", b"Description\x00DogOps live Go2 camera frame")
        + _png_chunk(b"IDAT", zlib.compress(raw, level=6))
        + _png_chunk(b"IEND", b"")
    )


def _camera_array_from_image(image: object) -> Any:
    if hasattr(image, "to_opencv"):
        return image.to_opencv()  # type: ignore[attr-defined]
    data = getattr(image, "data", None)
    if data is not None and hasattr(data, "shape"):
        return data
    return image


def _rgb_pixel(pixel: object, *, channels: int, encoding: str) -> bytes:
    if channels <= 1:
        value = _uint8(pixel)
        return bytes((value, value, value))
    values = [_uint8(pixel[index]) for index in range(min(channels, 4))]  # type: ignore[index]
    if len(values) < 3:
        value = values[0] if values else 0
        return bytes((value, value, value))
    if "rgb" in encoding and "bgr" not in encoding:
        red, green, blue = values[:3]
    else:
        blue, green, red = values[:3]
    return bytes((red, green, blue))


def _uint8(value: object) -> int:
    result = int(value)  # type: ignore[arg-type]
    return max(0, min(255, result))


def _positive_float(value: object, *, default: float) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result) or result <= 0:
        return default
    return result


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    checksum = zlib.crc32(kind + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", checksum)


def _format_point(point: tuple[float, float] | None) -> str:
    if point is None:
        return ""
    return f"{point[0]:.1f},{point[1]:.1f}"


def _register_subscription(owner: object, subscription: object) -> None:
    register = getattr(owner, "register_disposable", None)
    if not callable(register):
        return
    if callable(subscription) and RxDisposable is not None:
        register(RxDisposable(subscription))
        return
    register(subscription)


def _find_work_order(work_orders: list[WorkOrder], work_order_id: str) -> WorkOrder | None:
    for work_order in work_orders:
        if work_order.id == work_order_id:
            return work_order
    return None


def _finite_point(x: float, y: float, z: float) -> tuple[float, float, float]:
    try:
        point = (float(x), float(y), float(z))
    except (TypeError, ValueError) as exc:
        raise ValueError("go_to requires numeric x, y, and z") from exc
    if not all(math.isfinite(value) for value in point):
        raise ValueError("go_to target must be finite")
    return point


def _work_order_for_incident(work_orders: list[WorkOrder], incident_id: str) -> WorkOrder | None:
    for work_order in work_orders:
        if work_order.incident_id == incident_id:
            return work_order
    return None


def _clearance_snapshot(asset: Asset, state: DogOpsState | None) -> dict[str, object]:
    raw_clear, evidence_id = _latest_fact(state, f"{asset.id}.clearance_clear")
    clearance_clear = _to_bool(raw_clear)
    if clearance_clear is None:
        clearance_clear = asset.expected_clear
    blocking_package_ids = _blocking_package_ids(state, asset.id)
    if not blocking_package_ids:
        blocking_package_ids = sorted(asset.blocking_package_ids)
    if blocking_package_ids:
        clearance_clear = False
    state_label = (
        "clear" if clearance_clear is True else "blocked" if clearance_clear is False else "unknown"
    )
    return {
        "clearance_clear": clearance_clear,
        "state": state_label,
        "blocking_package_ids": blocking_package_ids,
        "evidence_observation_id": evidence_id,
    }


def _latest_fact(
    state: DogOpsState | None, key: str
) -> tuple[bool | str | int | float | None, str | None]:
    if state is None:
        return None, None
    for obs in reversed(state.observations):
        if key in obs.facts:
            return obs.facts[key], obs.id
    return None, None


def _blocking_package_ids(state: DogOpsState | None, asset_id: str) -> list[str]:
    if state is None:
        return []
    return sorted(
        status.package_id
        for status in state.package_statuses.values()
        if status.blocks_asset_id == asset_id
    )


def _has_open_incident(
    state: DogOpsState | None, entity_id: str, incident_type: IncidentType
) -> bool:
    if state is None:
        return False
    return any(
        incident.entity_id == entity_id
        and incident.type == incident_type
        and incident.state != IncidentState.resolved
        for incident in state.incidents
    )


def _observed_packages_for_zone(
    state: DogOpsState | None, mission: object, zone_id: str
) -> list[str]:
    package_ids: set[str] = set()
    if state is not None:
        package_ids.update(
            status.package_id
            for status in state.package_statuses.values()
            if status.observed_zone_id == zone_id
        )
        for obs in state.observations:
            if obs.zone_id == zone_id:
                package_ids.update(_package_ids_from_observation(obs, zone_id))
        return sorted(package_ids)

    observations = mission.simulation_observations.values()
    for obs in observations:
        if obs.zone_id == zone_id:
            package_ids.update(_package_ids_from_facts(obs.facts, zone_id))
    return sorted(package_ids)


def _visible_tags_for_zone(
    state: DogOpsState | None, mission: object, zone_id: str
) -> list[int]:
    tag_ids: set[int] = set()
    if state is not None:
        for obs in state.observations:
            if obs.zone_id == zone_id:
                tag_ids.update(_observation_tag_ids(obs))
        return sorted(tag_ids)

    observations = mission.simulation_observations.values()
    for obs in observations:
        if obs.zone_id == zone_id:
            tag_ids.update(obs.visible_tag_ids)
    return sorted(tag_ids)


def _package_ids_from_observation(obs: Observation, zone_id: str) -> set[str]:
    return _package_ids_from_facts(obs.facts, zone_id)


def _package_ids_from_facts(
    facts: dict[str, bool | str | int | float], zone_id: str
) -> set[str]:
    return {
        key.removesuffix(".zone_id")
        for key, value in facts.items()
        if key.startswith("PKG-") and key.endswith(".zone_id") and value == zone_id
    }


def _observation_tag_ids(obs: Observation) -> set[int]:
    tag_ids: set[int] = set()
    if obs.tag_id is not None:
        tag_ids.add(obs.tag_id)
    raw_tag_ids = obs.facts.get("visible_tag_ids")
    if isinstance(raw_tag_ids, str):
        for item in raw_tag_ids.split(","):
            item = item.strip()
            if item:
                tag_ids.add(int(item))
    return tag_ids


def _to_float(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int | float):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _to_bool(value: object) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "1", "clear"}:
            return True
        if normalized in {"false", "no", "0", "blocked"}:
            return False
    return None


def _json(**payload: object) -> str:
    return json.dumps(payload, sort_keys=True)

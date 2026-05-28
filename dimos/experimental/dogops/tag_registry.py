from __future__ import annotations

from dataclasses import dataclass

from dimos.experimental.dogops.models import SiteConfig, SiteEntity


@dataclass(frozen=True)
class TagRegistration:
    tag_id: int
    entity_id: str
    entity_kind: str
    display_name: str
    zone_id: str | None


class DogOpsTagRegistry:
    def __init__(self, site: SiteConfig) -> None:
        self.site = site
        self._by_tag_id = {
            tag_id: _registration_for_entity(tag_id, entity)
            for tag_id, entity in site.entity_for_tag().items()
        }

    def get(self, tag_id: int) -> TagRegistration | None:
        return self._by_tag_id.get(tag_id)

    def require(self, tag_id: int) -> TagRegistration:
        registration = self.get(tag_id)
        if registration is None:
            raise KeyError(f"unknown DogOps tag id: {tag_id}")
        return registration

    def all(self) -> list[TagRegistration]:
        return [self._by_tag_id[tag_id] for tag_id in sorted(self._by_tag_id)]


def _registration_for_entity(tag_id: int, entity: SiteEntity) -> TagRegistration:
    return TagRegistration(
        tag_id=tag_id,
        entity_id=entity.id,
        entity_kind=str(entity.kind),
        display_name=entity.display_name,
        zone_id=entity.zone_id,
    )

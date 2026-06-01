from __future__ import annotations
from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class SourceConfig:
    topic: str


@dataclass(frozen=True)
class OptiBridgeConfig:
    frame_id: str
    topic_prefix: str
    axis_mode: str
    invert_y: bool
    sources: Dict[str, SourceConfig]


def load_config_from_dict(d: dict) -> OptiBridgeConfig:
    frame_id = d.get("frame_id", "world")
    topic_prefix = d.get("topic_prefix", "/optitrack")
    axis_mode = d.get("axis_mode", "identity")
    invert_y = d.get("invert_y", False)

    sources_raw = d.get("sources", {})
    if not sources_raw:
        raise ValueError("'sources' must not be empty")

    sources = {
        drone_id: SourceConfig(topic=str(topic))
        for drone_id, topic in sources_raw.items()
    }

    return OptiBridgeConfig(
        frame_id=frame_id,
        topic_prefix=topic_prefix,
        axis_mode=axis_mode,
        sources=sources,
        invert_y=invert_y,

    )
















from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Literal, Optional, Final


InputType = Literal[
    "pose_stamped",
    "transform_stamped",
    "odom",
    "pose",
    "twist_xyz",
]

SUPPORTED_AXIS_MODES: Final[set[str]] = {
    "identity",
    "optitrack_to_enu",
}


@dataclass(frozen=True)
class SourceConfig:
    topic: str
    input_type: InputType
    frame_id: Optional[str] = None


@dataclass(frozen=True)
class MocapNormalizerConfig:
    output_frame_id: str
    output_topic_prefix: str
    axis_mode: str
    sources: Dict[str, SourceConfig]


def _require(d: dict, key: str):
    if key not in d:
        raise ValueError(f"Missing required config key: '{key}'")
    return d[key]


def _require_nonempty_string(value, field_name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"'{field_name}' must be a string")
    out = value.strip()
    if not out:
        raise ValueError(f"'{field_name}' must not be empty")
    return out


def _normalize_topic_prefix(value: str) -> str:
    value = value.strip()

    if not value:
        raise ValueError("'topic_prefix' must not be empty")

    if not value.startswith("/"):
        value = "/" + value

    if value != "/":
        value = value.rstrip("/")

    return value


def _validate_id(id: str) -> str:
    id = id.strip()
    if not id:
        raise ValueError("Id must not be empty")

    if "/" in id:
        raise ValueError(f"Invalid  id '{id}': must not contain '/'")

    return id


def load_config_from_dict(d: dict) -> MocapNormalizerConfig:
    if not isinstance(d, dict):
        raise ValueError("Top-level YAML config must be a mapping")

    output_frame_id = _require_nonempty_string(
        d.get("frame_id", "map"), "frame_id"
    )

    output_topic_prefix = _normalize_topic_prefix(
        str(d.get("topic_prefix", "/mocap"))
    )

    axis_mode = _require_nonempty_string(
        d.get("axis_mode", "identity"), "axis_mode"
    )

    if axis_mode not in SUPPORTED_AXIS_MODES:
        raise ValueError(
            f"Invalid axis_mode '{axis_mode}'. "
            f"Supported: {sorted(SUPPORTED_AXIS_MODES)}"
        )

    sources_raw = _require(d, "sources")
    if not isinstance(sources_raw, dict):
        raise ValueError("'sources' must be a mapping")

    if not sources_raw:
        raise ValueError("'sources' must not be empty")

    valid_types = set(InputType.__args__)  # type: ignore[attr-defined]

    sources: Dict[str, SourceConfig] = {}
    seen_topics: set[str] = set()

    for raw_drone_id, item in sources_raw.items():
        drone_id = _validate_id(str(raw_drone_id))

        if not isinstance(item, dict):
            raise ValueError(f"'sources.{drone_id}' must be a mapping")

        topic = _require_nonempty_string(
            _require(item, "topic"),
            f"sources.{drone_id}.topic",
        )

        input_type = _require_nonempty_string(
            _require(item, "type"),
            f"sources.{drone_id}.type",
        )

        if input_type not in valid_types:
            raise ValueError(
                f"Invalid sources.{drone_id}.type='{input_type}'. "
                f"Valid: {sorted(valid_types)}"
            )

        if topic in seen_topics:
            raise ValueError(f"Duplicate topic detected: '{topic}'")
        seen_topics.add(topic)

        frame_override = item.get("frame_id")
        if frame_override is not None:
            frame_override = _require_nonempty_string(
                frame_override,
                f"sources.{drone_id}.frame_id",
            )

        sources[drone_id] = SourceConfig(
            topic=topic,
            input_type=input_type,  # type: ignore[arg-type]
            frame_id=frame_override,
        )

    return MocapNormalizerConfig(
        output_frame_id=output_frame_id,
        output_topic_prefix=output_topic_prefix,
        axis_mode=axis_mode,
        sources=sources,
    )
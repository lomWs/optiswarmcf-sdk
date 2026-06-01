from __future__ import annotations

from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class DroneConfig:
    drone_id: str
    uri: str
    ns: str
    mocap_topic: str  # e.g. /optitrack/<id>/pose


@dataclass(frozen=True)
class CfBridgeConfig:
    drones: List[DroneConfig]

    with_orient: bool = False
    start_hl: bool = True
    hl_only: bool = True

    estimator: int = 2
    controller: int = 2
    en_high_level: bool = True

    speed: float = 0.30
    min_abs_duration: float = 1.0
    min_rel_duration: float = 0.5
    abs_duration_margin: float = 0.5
    rel_duration_margin: float = 0.3
    fallback_abs_duration: float = 1.5

    diag_period_sec: float = 0.5
    pose_stale_after_sec: float = 0.5
    extpos_max_rate_hz: float = 50.0
    max_pose_jump_m: float = 0.25
    min_valid_z_m: float = -0.02

    cache_dir: str = "./.cf_cache"


def _require(d: dict, key: str):
    if key not in d:
        raise ValueError(f"Missing required config key: {key}")
    return d[key]


def _require_nonempty_str(d: dict, key: str) -> str:
    value = str(_require(d, key)).strip()
    if not value:
        raise ValueError(f"Config key '{key}' must be a non-empty string")
    return value


def _normalize_ns(value: str) -> str:
    ns = value.strip()
    if not ns:
        raise ValueError("Drone namespace must be non-empty")

    if not ns.startswith("/"):
        ns = f"/{ns}"

    ns = ns.rstrip("/")
    if ns == "":
        raise ValueError("Drone namespace must not resolve to empty")

    return ns


def _normalize_topic(value: str) -> str:
    topic = value.strip()
    if not topic:
        raise ValueError("mocap_topic must be non-empty")

    if not topic.startswith("/"):
        topic = f"/{topic}"

    return topic


def _positive_float(d: dict, key: str, default: float) -> float:
    value = float(d.get(key, default))
    if value <= 0.0:
        raise ValueError(f"'{key}' must be > 0")
    return value


def _float(d: dict, key: str, default: float) -> float:
    return float(d.get(key, default))


def _nonnegative_float(d: dict, key: str, default: float) -> float:
    value = float(d.get(key, default))
    if value < 0.0:
        raise ValueError(f"'{key}' must be >= 0")
    return value


def load_config_from_dict(d: dict) -> CfBridgeConfig:
    drones_raw = _require(d, "drones")
    if not isinstance(drones_raw, list) or not drones_raw:
        raise ValueError("'drones' must be a non-empty list")

    drones: List[DroneConfig] = []
    for item in drones_raw:
        if not isinstance(item, dict):
            raise ValueError("Each drones[] entry must be a dict")

        drone_id = _require_nonempty_str(item, "id")
        uri = _require_nonempty_str(item, "uri")
        ns = _normalize_ns(_require_nonempty_str(item, "ns"))
        mocap_topic = _normalize_topic(_require_nonempty_str(item, "mocap_topic"))

        drones.append(
            DroneConfig(
                drone_id=drone_id,
                uri=uri,
                ns=ns,
                mocap_topic=mocap_topic,
            )
        )

    cfg = CfBridgeConfig(
        drones=drones,
        with_orient=bool(d.get("with_orient", False)),
        start_hl=bool(d.get("start_hl", True)),
        hl_only=bool(d.get("hl_only", True)),
        estimator=int(d.get("estimator", 2)),
        controller=int(d.get("controller", 2)),
        en_high_level=bool(d.get("en_high_level", True)),
        speed=_positive_float(d, "speed", 0.30),
        min_abs_duration=_positive_float(d, "min_abs_duration", 1.0),
        min_rel_duration=_positive_float(d, "min_rel_duration", 0.5),
        abs_duration_margin=_nonnegative_float(d, "abs_duration_margin", 0.5),
        rel_duration_margin=_nonnegative_float(d, "rel_duration_margin", 0.3),
        fallback_abs_duration=_positive_float(d, "fallback_abs_duration", 1.5),
        diag_period_sec=_positive_float(d, "diag_period_sec", 0.5),
        pose_stale_after_sec=_positive_float(d, "pose_stale_after_sec", 0.5),
        extpos_max_rate_hz=_positive_float(d, "extpos_max_rate_hz", 50.0),
        max_pose_jump_m=_positive_float(d, "max_pose_jump_m", 0.25),
        min_valid_z_m=_float(d, "min_valid_z_m", -0.02),
        cache_dir=str(d.get("cache_dir", "./.cf_cache")).strip() or "./.cf_cache",
    )

    return cfg
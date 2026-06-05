from __future__ import annotations

import os
import time
import math
from typing import Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander

from cflib.utils.reset_estimator import reset_estimator
from .config import CfBridgeConfig, DroneConfig


PARAM_SET_SLEEP_SEC = 0.1
EKF_RESET_WAIT_SEC = 2.0
DEFAULT_TAKEOFF_DELTA_Z_M = 0.15
MIN_TAKEOFF_DURATION_SEC = 2
TAKEOFF_DURATION_MARGIN_SEC = 1
LAND_HEIGHT_M = 0.0
LAND_DURATION_SEC = 3.0


def make_sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


def make_cmd_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


class CfClient:
    def __init__(self, node: Node, dcfg: DroneConfig, cfg: CfBridgeConfig) -> None:
        self.n = node
        self.dcfg = dcfg
        self.cfg = cfg

        self.last_pose: Optional[PoseStamped] = None
        self._pose_rate = 0.0
        self._pose_last_t = 0.0
        self._last_cmd: Optional[str] = None

        self._connected = False
        self._last_extpos_t = 0.0
        self._last_extpos_xyz: Optional[tuple[float, float, float]] = None

        self.cf: Optional[Crazyflie] = None
        self.scf: Optional[SyncCrazyflie] = None
        self.hlc: Optional[HighLevelCommander] = None

        self._connect_crazyflie()
        self._configure_crazyflie_firmware()

        self._setup_ros_publishers()
        self._setup_ros_subscriptions()
        self._setup_ros_services()
        self._start_ros_timers()

    def _setup_ros_subscriptions(self) -> None:
        self.sub_pose = self.n.create_subscription(
            PoseStamped, self.dcfg.mocap_topic, self.cb_pose, make_sensor_qos()
        )
        self.sub_abs = self.n.create_subscription(
            PoseStamped, f"{self.dcfg.ns}/cmd_pos", self.cb_pos_abs, make_cmd_qos()
        )
        self.sub_rel = self.n.create_subscription(
            Twist, f"{self.dcfg.ns}/cmd_pos_relative", self.cb_pos_rel, make_cmd_qos()
        )

    def _setup_ros_publishers(self) -> None:
        self.pub_diag = self.n.create_publisher(
            DiagnosticArray, f"{self.dcfg.ns}/diag", 10
        )

    def _setup_ros_services(self) -> None:
        self.srv_take = self.n.create_service(
            Trigger, f"{self.dcfg.ns}/takeoff", self.srv_takeoff
        )
        self.srv_land = self.n.create_service(
            Trigger, f"{self.dcfg.ns}/land", self.srv_land_fn
        )
        self.srv_rst = self.n.create_service(
            Trigger, f"{self.dcfg.ns}/ekf_reset", self.srv_ekf_reset
        )

    def _start_ros_timers(self) -> None:
        self._tmr = self.n.create_timer(self.cfg.diag_period_sec, self._tick_diag)

    def _connect_crazyflie(self) -> None:
        try:
            cache_dir = os.path.realpath(os.path.expanduser(self.cfg.cache_dir))
            os.makedirs(cache_dir, exist_ok=True)

            self.cf = Crazyflie(rw_cache=cache_dir)
            self.scf = SyncCrazyflie(self.dcfg.uri, cf=self.cf)
            self.scf.open_link()
            self.hlc = HighLevelCommander(self.cf)

            self._connected = True
            self.n.get_logger().info(
                f"[{self.dcfg.ns}] Connected to Crazyflie at {self.dcfg.uri}"
            )
        except Exception as e:
            self._connected = False
            self.n.get_logger().error(
                f"[{self.dcfg.ns}] Crazyflie connection failed: {e}"
            )
            raise

    def _configure_crazyflie_firmware(self) -> None:
        if not self._connected or self.cf is None:
            raise RuntimeError("Crazyflie not connected")

        self._ensure_param("stabilizer.estimator", str(self.cfg.estimator))
        self._ensure_param("stabilizer.controller", str(self.cfg.controller))
        self._ensure_param(
            "commander.enHighLevel",
            "1" if self.cfg.en_high_level else "0",
        )

        self.n.get_logger().info(
            f"[CF READY] {self.dcfg.ns} id={self.dcfg.drone_id} "
            f"uri={self.dcfg.uri} mocap={self.dcfg.mocap_topic} "
            f"estimator={self.cfg.estimator} controller={self.cfg.controller} "
            f"hl={self.cfg.en_high_level}"
        )

    def _ensure_param(self, name: str, val: str) -> None:
        if self.cf is None:
            raise RuntimeError("Crazyflie param interface is not available")

        try:
            self.cf.param.set_value(name, str(val))
            time.sleep(PARAM_SET_SLEEP_SEC)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] Param set failed {name}={val}: {e}"
            )

    def _extpos(self, x: float, y: float, z: float, q=None) -> None:
        if not self._connected or self.cf is None:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] extpos skipped: Crazyflie not connected"
            )
            return

        try:
            if q is not None and hasattr(self.cf.extpos, "send_extpose"):
                self.cf.extpos.send_extpose(x, y, z, q[0], q[1], q[2], q[3])
            else:
                self.cf.extpos.send_extpos(x, y, z)
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] extpos failed: {e}")

    def cb_pose(self, msg: PoseStamped) -> None:
        self.last_pose = msg

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._pose_last_t > 0.0 and t > self._pose_last_t:
            self._pose_rate = 1.0 / (t - self._pose_last_t)
        self._pose_last_t = t

        p = msg.pose.position
        x = float(p.x)
        y = float(p.y)
        z = float(p.z)

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] invalid mocap pose: ({x}, {y}, {z})"
            )
            return

        if z < self.cfg.min_valid_z_m:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] rejected mocap z={z:.3f} below min_valid_z_m={self.cfg.min_valid_z_m:.3f}"
            )
            return

        min_dt = 1.0 / self.cfg.extpos_max_rate_hz
        if self._last_extpos_t > 0.0 and (t - self._last_extpos_t) < min_dt:
            return

        if self._last_extpos_xyz is not None:
            lx, ly, lz = self._last_extpos_xyz
            jump = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
            if jump > self.cfg.max_pose_jump_m:
                self.n.get_logger().warning(
                    f"[{self.dcfg.ns}] rejected mocap jump={jump:.3f} m "
                    f"limit={self.cfg.max_pose_jump_m:.3f} m"
                )
                return

        self._last_extpos_t = t
        self._last_extpos_xyz = (x, y, z)

        if self.cfg.with_orient:
            q = msg.pose.orientation
            self._extpos(
                x,
                y,
                z,
                (float(q.x), float(q.y), float(q.z), float(q.w)),
            )
        else:
            self._extpos(x, y, z, None)

    
    '''def cb_pose(self, msg: PoseStamped) -> None:
        self.last_pose = msg

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._pose_last_t > 0.0 and t > self._pose_last_t:
            self._pose_rate = 1.0 / (t - self._pose_last_t)
        self._pose_last_t = t

        p = msg.pose.position
        x = float(p.x)
        y = float(p.y)
        z = float(p.z)

        #  NaN/infinity.
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] invalid mocap pose: ({x}, {y}, {z})"
            )
            return

        self._last_extpos_t = t
        self._last_extpos_xyz = (x, y, z)

        if self.cfg.with_orient:
            q = msg.pose.orientation
            self._extpos(
                x,
                y,
                z,
                (float(q.x), float(q.y), float(q.z), float(q.w)),
            )
        else:
            self._extpos(x, y, z, None)'''

    def cb_pos_abs(self, msg: PoseStamped) -> None:
        try:
            x, y, z = self._pose_xyz(msg)
            yaw = self._pose_yaw(msg)
            dur = self._compute_abs_duration(x, y, z)
            self._send_abs_position(x, y, z, yaw, dur)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] go_to abs failed: {e}"
            )

    def cb_pos_rel(self, msg: Twist) -> None:
        try:
            dx, dy, dz = self._relative_xyz(msg)
            yaw = self._relative_yaw(msg)
            dur = self._compute_rel_duration(dx, dy, dz)
            self._send_rel_position(dx, dy, dz, yaw, dur)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] go_to rel failed: {e}"
            )

    def srv_takeoff(self, req, resp):
        try:
            if not self._connected or self.hlc is None:
                raise RuntimeError("Crazyflie not connected or HLC not available")

            if self.last_pose is None:
                raise RuntimeError("No mocap pose received yet")

            self._last_cmd = "takeoff"
            current_z = float(self.last_pose.pose.position.z)
            height = DEFAULT_TAKEOFF_DELTA_Z_M
            dur = max(
                MIN_TAKEOFF_DURATION_SEC,
                height / max(0.10, self.cfg.speed) + TAKEOFF_DURATION_MARGIN_SEC,
            )
            self.hlc.takeoff(height, dur)
            resp.success = True
            resp.message = "OK"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp

    def srv_land_fn(self, req, resp):
        try:
            if not self._connected or self.hlc is None:
                raise RuntimeError("Crazyflie not connected or HLC not available")

            self._last_cmd = "land"
            self.hlc.land(LAND_HEIGHT_M, LAND_DURATION_SEC)
            resp.success = True
            resp.message = "Land sent"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp

    def srv_ekf_reset(self, req, resp):
        try:
            if not self._connected or self.cf is None:
                raise RuntimeError("Crazyflie not connected")

            self._last_cmd = "ekf_reset"

            if self.last_pose is None:
                resp.success = False
                resp.message = "No mocap pose received yet"
                return resp

            self.cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(PARAM_SET_SLEEP_SEC)
            self.cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(EKF_RESET_WAIT_SEC)

            resp.success = True
            resp.message = "EKF reset and stabilization wait completed"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp

    def _tick_diag(self) -> None:
        has_pose = self.last_pose is not None

        if has_pose:
            t_pose = (
                self.last_pose.header.stamp.sec
                + self.last_pose.header.stamp.nanosec * 1e-9
            )
            now = self.n.get_clock().now().nanoseconds * 1e-9
            pose_age = max(0.0, now - t_pose)
        else:
            pose_age = -1.0

        ready = self._connected and has_pose and (
            pose_age < self.cfg.pose_stale_after_sec if has_pose else False
        )

        st = DiagnosticStatus()
        st.name = f"cf_bridge[{self.dcfg.drone_id}]"

        if not self._connected:
            st.level = DiagnosticStatus.ERROR
            st.message = "disconnected"
        elif not has_pose:
            st.level = DiagnosticStatus.WARN
            st.message = "waiting_for_pose"
        elif pose_age > self.cfg.pose_stale_after_sec:
            st.level = DiagnosticStatus.WARN
            st.message = "pose_stale"
        else:
            st.level = DiagnosticStatus.OK
            st.message = "ready"

        st.values = [
            KeyValue(key="drone_id", value=self.dcfg.drone_id),
            KeyValue(key="uri", value=self.dcfg.uri),
            KeyValue(key="mocap_topic", value=self.dcfg.mocap_topic),
            KeyValue(key="connected", value=str(self._connected)),
            KeyValue(key="has_pose", value=str(has_pose)),
            KeyValue(key="ready", value=str(ready)),
            KeyValue(key="pose_rate_hz", value=f"{self._pose_rate:.1f}"),
            KeyValue(key="last_pose_age_sec", value=f"{pose_age:.3f}"),
            KeyValue(key="with_orient", value=str(self.cfg.with_orient)),
            KeyValue(key="speed", value=f"{self.cfg.speed:.2f}"),
            KeyValue(key="estimator", value=str(self.cfg.estimator)),
            KeyValue(key="controller", value=str(self.cfg.controller)),
            KeyValue(key="en_high_level", value=str(self.cfg.en_high_level)),
            KeyValue(key="last_cmd", value=str(self._last_cmd)),
        ]

        arr = DiagnosticArray()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def _pose_xyz(self, msg: PoseStamped) -> tuple[float, float, float]:
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise ValueError(f"Invalid absolute target position: ({x}, {y}, {z})")

        return x, y, z

    #transform quaternion to yaw angle in radians
    def _pose_yaw(self, msg: PoseStamped) -> float:
        q = msg.pose.orientation
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)

        if not (
            math.isfinite(qx)
            and math.isfinite(qy)
            and math.isfinite(qz)
            and math.isfinite(qw)
        ):
            raise ValueError("Invalid absolute target orientation")

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _current_pose_xyz(self) -> Optional[tuple[float, float, float]]:
        if self.last_pose is None:
            return None

        lx = float(self.last_pose.pose.position.x)
        ly = float(self.last_pose.pose.position.y)
        lz = float(self.last_pose.pose.position.z)

        if not (math.isfinite(lx) and math.isfinite(ly) and math.isfinite(lz)):
            return None

        return lx, ly, lz

    def _compute_abs_duration(self, x: float, y: float, z: float) -> float:
        cur = self._current_pose_xyz()
        speed = max(0.10, self.cfg.speed)

        if cur is None:
            return self.cfg.fallback_abs_duration

        lx, ly, lz = cur
        dist = max(0.01, math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2))
        return max(
            self.cfg.min_abs_duration,
            dist / speed + self.cfg.abs_duration_margin,
        )

    def _send_abs_position(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        dur: float,
    ) -> None:
        if not self._connected or self.hlc is None:
            raise RuntimeError("Crazyflie not connected or HLC not available")

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise ValueError(f"Invalid absolute command: ({x}, {y}, {z})")

        if not math.isfinite(yaw):
            raise ValueError(f"Invalid absolute yaw: {yaw}")

        if not math.isfinite(dur) or dur <= 0.0:
            raise ValueError(f"Invalid absolute duration: {dur}")

        try:
            self.hlc.go_to(x, y, z, yaw, dur, relative=False)
        except TypeError:
            self.hlc.go_to(x, y, z, yaw, dur)

        self._last_cmd = (
            f"goto_abs({x:.2f},{y:.2f},{z:.2f}, yaw={yaw:.2f}, T={dur:.2f})"
        )

    def _relative_xyz(self, msg: Twist) -> tuple[float, float, float]:
        dx = float(msg.linear.x)
        dy = float(msg.linear.y)
        dz = float(msg.linear.z)

        if not (math.isfinite(dx) and math.isfinite(dy) and math.isfinite(dz)):
            raise ValueError(
                f"Invalid relative target displacement: ({dx}, {dy}, {dz})"
            )

        return dx, dy, dz

    def _relative_yaw(self, msg: Twist) -> float:
        yaw = float(msg.angular.z)

        if not math.isfinite(yaw):
            raise ValueError(f"Invalid relative yaw: {yaw}")

        return yaw

    def _compute_rel_duration(self, dx: float, dy: float, dz: float) -> float:
        dist = max(0.01, math.sqrt(dx * dx + dy * dy + dz * dz))
        speed = max(0.10, self.cfg.speed)
        return max(self.cfg.min_rel_duration, dist / speed + self.cfg.rel_duration_margin)

    def _send_rel_position(
        self,
        dx: float,
        dy: float,
        dz: float,
        yaw: float,
        dur: float,
    ) -> None:
        if not self._connected or self.hlc is None:
            raise RuntimeError("Crazyflie not connected or HLC not available")

        if not (math.isfinite(dx) and math.isfinite(dy) and math.isfinite(dz)):
            raise ValueError(f"Invalid relative command: ({dx}, {dy}, {dz})")

        if not math.isfinite(yaw):
            raise ValueError(f"Invalid relative yaw: {yaw}")

        if not math.isfinite(dur) or dur <= 0.0:
            raise ValueError(f"Invalid relative duration: {dur}")

        try:
            self.hlc.go_to(dx, dy, dz, yaw, dur, relative=True)
        except TypeError:
            cur = self._current_pose_xyz()
            if cur is None:
                raise RuntimeError("Relative fallback requires a valid current pose")

            lx, ly, lz = cur
            self.hlc.go_to(lx + dx, ly + dy, lz + dz, yaw, dur)

        self._last_cmd = (
            f"goto_rel({dx:.2f},{dy:.2f},{dz:.2f}, yaw={yaw:.2f}, T={dur:.2f})"
        )

    def close(self) -> None:
        if self.hlc is not None:
            try:
                self.hlc.stop()
            except Exception as e:
                self.n.get_logger().warning(
                    f"[{self.dcfg.ns}] HLC stop failed: {e}"
                )

        if self.scf is not None:
            try:
                self.scf.close_link()
            except Exception as e:
                self.n.get_logger().warning(
                    f"[{self.dcfg.ns}] close_link failed: {e}"
                )

        self._connected = False
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

from .config import CfBridgeConfig, DroneConfig

def make_sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
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
        self._vel_mode: Optional[str] = None  # "world" or "body"

        self._setup_ros_subscriptions()
        self._setup_ros_publishers()
        self._setup_ros_services()

        self._connect_crazyflie()
        self._configure_crazyflie_firmware()
        self._start_ros_timers()



    #----- private methods for configuration and command sending -----

    def _setup_ros_subscriptions(self) -> None:
        self.sub_pose = self.n.create_subscription(
            PoseStamped, self.dcfg.mocap_topic, self.cb_pose, make_sensor_qos()
        )
        self.sub_abs = self.n.create_subscription(
            PoseStamped, f"{self.dcfg.ns}/cmd_pos", self.cb_pos_abs, 10
        )
        self.sub_rel = self.n.create_subscription(
            Twist, f"{self.dcfg.ns}/cmd_pos_relative", self.cb_pos_rel, 10
        )
        self.sub_vel = self.n.create_subscription(
            Twist, f"{self.dcfg.ns}/cmd_vel", self.cb_cmd_vel, 10
        )
        self.sub_vel_world = self.n.create_subscription(
            Twist, f"{self.dcfg.ns}/cmd_vel_world", self.cb_cmd_vel_world, 10
        )
        self.sub_vel_body = self.n.create_subscription(
            Twist, f"{self.dcfg.ns}/cmd_vel_body", self.cb_cmd_vel_body, 10
        )
    
    def _setup_ros_publishers(self) -> None:
        self.pub_diag = self.n.create_publisher(DiagnosticArray, f"{self.dcfg.ns}/diag", 10)
    
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
        #start periodic timer for publishing diagnostics
        self._tmr = self.n.create_timer(self.cfg.diag_period_sec, self._tick_diag)

    def _connect_crazyflie(self) -> None: 
        # Crazyflie link
        cache_dir = os.path.realpath(os.path.expanduser(self.cfg.cache_dir))# ensure cache dir exists for storing radio calibration and other data
        os.makedirs(cache_dir, exist_ok=True)

        self.cf = Crazyflie(rw_cache=cache_dir)
        self.scf = SyncCrazyflie(self.dcfg.uri, cf=self.cf)
        self.scf.open_link()
        self.hlc = HighLevelCommander(self.cf)
        
    def _configure_crazyflie_firmware(self) -> None:
        # Estimator / controller / HL
        self._ensure_param("stabilizer.estimator", "2")   # Kalman
        self._ensure_param("stabilizer.controller", "2")  # Mellinger

        if self.cfg.start_hl or self.cfg.hl_only:
            self._ensure_param("commander.enHighLevel", "1")
        else:
            self._ensure_param("commander.enHighLevel", "0")
        self.n.get_logger().info(
            f"[CF READY] {self.dcfg.ns} id={self.dcfg.drone_id} uri={self.dcfg.uri} mocap={self.dcfg.mocap_topic}"
        )

    #used in configuration_firmware_Crazyflie()
    def _ensure_param(self, name: str, val: str) -> None:
        try:
            self.cf.param.set_value(name, str(val))
            time.sleep(0.02)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] Param set failed {name}={val}: {e}"
            )

    #----- private methods for configuration and command sending -----

    #----- private methods for command sending (called from callbacks) -----
    def _extpos(self, x: float, y: float, z: float, q=None) -> None:
        try:
            if q is not None and hasattr(self.cf.extpos, "send_extpose"):
                self.cf.extpos.send_extpose(x, y, z, q[0], q[1], q[2], q[3])
            else:
                self.cf.extpos.send_extpos(x, y, z)
        except Exception as e:
            self.n.get_logger().warning(f"[{self.dcfg.ns}] extpos failed: {e}")

    # private helper to stop any active HL command before sending a new one, to avoid conflicts and ensure responsiveness
    def _release_low_level_priority(self) -> None:
        try:
            cmdr = self.cf.commander
            if hasattr(cmdr, "send_notify_setpoint_stop"):
                cmdr.send_notify_setpoint_stop()
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] notify_setpoint_stop failed: {e}"
            )

    # --- callbacks ---

    def cb_pose(self, msg: PoseStamped) -> None:
        # Callback from mocap pose updates. 
        # We use this to track the pose rate and for computing durations for position commands.
        t = time.time()
        if self._pose_last_t > 0:
            dt = t - self._pose_last_t
            if dt > 1e-6:
                self._pose_rate = 0.9 * self._pose_rate + 0.1 * (1.0 / dt)

        self._pose_last_t = t
        self.last_pose = msg

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)

        if self.cfg.invert_y:
            y = -y

        if self.cfg.with_orient:
            q = msg.pose.orientation
            self._extpos(
                x, y, z,
                (float(q.x), float(q.y), float(q.z), float(q.w))
            )
        else:
            self._extpos(x, y, z, None)
    
    def cb_pos_abs(self, msg: PoseStamped) -> None:
        try:
            x, y, z = self._pose_xyz(msg)
            dur = self._compute_abs_duration(x, y, z)
            self._send_abs_position(x, y, z, dur)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] go_to abs failed: {e}"
            )
    
    def cb_pos_rel(self, msg: Twist) -> None:
        try:
            dx, dy, dz = self._relative_xyz(msg)
            dur = self._compute_rel_duration(dx, dy, dz)
            self._send_rel_position(dx, dy, dz, dur)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] go_to rel failed: {e}"
            )

    # alias of cmd_vel_world
    def cb_cmd_vel(self, msg: Twist) -> None:
        try:
            self._forward_vel("world", msg)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] cmd_vel (alias world) failed: {e}"
            )
    
    def cb_cmd_vel_world(self, msg: Twist) -> None:
        try:
            self._forward_vel("world", msg)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] cmd_vel_world failed: {e}"
            )

    def cb_cmd_vel_body(self, msg: Twist) -> None:
        try:
            self._forward_vel("body", msg)
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] cmd_vel_body failed: {e}"
            )
    
    # --- callbacks ---
    # --- services ---

    def srv_takeoff(self, req, resp):
        try:
            self._last_cmd = "takeoff"
            self._release_low_level_priority()
            self.hlc.takeoff(0.5, 2.0)
            resp.success = True
            resp.message = "OK"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp

    def srv_land_fn(self, req, resp):
        try:
            self._last_cmd = "land"
            self._release_low_level_priority()
            self.hlc.land(0.0, 2.0)
            resp.success = True
            resp.message = "OK"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp

    def srv_ekf_reset(self, req, resp):
        try:
            self._last_cmd = "ekf_reset"
            self.cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.05)
            self.cf.param.set_value("kalman.resetEstimation", "0")
            resp.success = True
            resp.message = "EKF reset"
        except Exception as e:
            resp.success = False
            resp.message = f"ERROR: {e}"
        return resp
    
    # --- services ---
    # --- internals ---

    def _tick_diag(self) -> None:
        st = DiagnosticStatus()
        st.level = DiagnosticStatus.OK
        st.name = f"{self.dcfg.ns}/cf_bridge"
        st.message = "running"
        st.values = [
            KeyValue(key="drone_id", value=self.dcfg.drone_id),
            KeyValue(key="uri", value=self.dcfg.uri),
            KeyValue(key="mocap_topic", value=self.dcfg.mocap_topic),
            KeyValue(key="pose_rate_hz", value=f"{self._pose_rate:.1f}"),
            KeyValue(key="last_cmd", value=str(self._last_cmd)),
        ]

        arr = DiagnosticArray()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def _forward_vel(self, mode: str, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vz = float(msg.linear.z)
        yaw_rate_rad_s = float(msg.angular.z)

        if self.cfg.invert_y:
            vy = -vy

        yaw_rate_deg_s = yaw_rate_rad_s * (180.0 / math.pi)

        self._vel_mode = mode
        self._last_cmd = (
            f"cmd_vel_{mode}(vx={vx:.2f},vy={vy:.2f},vz={vz:.2f},yaw={yaw_rate_rad_s:.2f})"
        )

        cmdr = self.cf.commander
        if mode == "world":
            if not hasattr(cmdr, "send_velocity_world_setpoint"):
                raise RuntimeError("Commander missing send_velocity_world_setpoint()")
            cmdr.send_velocity_world_setpoint(vx, vy, vz, yaw_rate_deg_s)
        elif mode == "body":
            if not hasattr(cmdr, "send_velocity_setpoint"):
                raise RuntimeError("Commander missing send_velocity_setpoint()")
            cmdr.send_velocity_setpoint(vx, vy, vz, yaw_rate_deg_s)
        else:
            raise ValueError(f"Unknown vel mode: {mode}")
    
    #define some helpers functions for cb_pos_abs()
    def _pose_xyz(self,msg: PoseStamped) -> tuple[float, float, float]:
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)

        if self.cfg.invert_y:
            y = -y
        
        return x, y, z    
    def _current_pose_xyz(self) -> Optional[tuple[float, float, float]]:
        if self.last_pose is None:
            return None

        lx = float(self.last_pose.pose.position.x)
        ly = float(self.last_pose.pose.position.y)
        lz = float(self.last_pose.pose.position.z)

        if self.cfg.invert_y:
            ly = -ly

        return lx, ly, lz   
    def _compute_abs_duration(self, x: float, y: float, z: float) -> float:
        cur = self._current_pose_xyz()
        if cur is None:
            return 1.0

        lx, ly, lz = cur
        dist = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
        return max(0.5, dist / max(0.10, self.cfg.speed))  
    def _send_abs_position(self, x: float, y: float, z: float, dur: float) -> None:
        self._last_cmd = f"abs({x:.2f},{y:.2f},{z:.2f}) dur={dur:.2f}"
        self._release_low_level_priority()
        self.hlc.go_to(x, y, z, 0.0, dur, relative=False)
    
    #define some helpers functions for cb_pos_rel()
    def _relative_xyz(self, msg: Twist) -> tuple[float, float, float]:
        dx = float(msg.linear.x)
        dy = float(msg.linear.y)
        dz = float(msg.linear.z)

        if self.cfg.invert_y:
            dy = -dy

        return dx, dy, dz   
    def _compute_rel_duration(self, dx: float, dy: float, dz: float) -> float:
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return max(0.5, dist / max(0.10, self.cfg.speed))
    def _send_rel_position(self, dx: float, dy: float, dz: float, dur: float) -> None:
        self._last_cmd = f"rel({dx:.2f},{dy:.2f},{dz:.2f}) dur={dur:.2f}"
        self._release_low_level_priority()
        self.hlc.go_to(dx, dy, dz, 0.0, dur, relative=True)
    
    
    # --- internals ---

    def close(self) -> None:
        try:
            self.hlc.stop()
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] HLC stop failed: {e}"
            )

        try:
            self.scf.close_link()
        except Exception as e:
            self.n.get_logger().warning(
                f"[{self.dcfg.ns}] close_link failed: {e}"
            )

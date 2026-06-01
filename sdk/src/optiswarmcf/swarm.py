from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable

from rclpy.node import Node

from .crazyflie import CrazyflieAgent, CrazyflieAgentConfig
from .optitrack import OptiTrack
from .context import RosContext

@dataclass(frozen=True)
class SwarmConfig:
    mocap_pose_topics: Dict[str, str]         # drone_id -> /mocap/<id>/pose
    agents: Dict[str, CrazyflieAgentConfig]            # drone_id -> agent cfg


class Swarm:
    def __init__(self, ctx: RosContext, cfg: SwarmConfig) -> None:
        self._cfg = cfg
        self._agents: Dict[str, CrazyflieAgent] = {
            did: CrazyflieAgent(ctx, acfg)
            for did, acfg in cfg.agents.items()
        }

    @property
    def agents(self) -> Dict[str, CrazyflieAgent]:
        return self._agents

    def ids(self) -> list[str]:
        return list(self._agents.keys())

    def agent(self, drone_id: str) -> CrazyflieAgent:
        try:
            return self._agents[drone_id]
        except KeyError as e:
            raise ValueError(
                f"Unknown agent with ID '{drone_id}'. Available agents: {list(self._agents.keys())}"
            ) from e

    def get_agent(self, drone_id: str) -> CrazyflieAgent:
        return self.agent(drone_id)

    def __getitem__(self, drone_id: str) -> CrazyflieAgent:
        return self.agent(drone_id)

    def values(self) -> Iterable[CrazyflieAgent]:
        return self._agents.values()

    def items(self):
        return self._agents.items()

    def wait_all_ready(
        self,
        mocap: OptiTrack,
        tmax: float = 10.0,
        sleep_dt: float = 0.02,
    ) -> bool:
        return mocap.wait_all_ready(
            drone_ids=self.ids(),
            tmax=tmax,
            sleep_dt=sleep_dt,
        )

    def takeoff_all(self, timeout_sec: float = 1.0) -> None:
        for agent in self._agents.values():
            agent.takeoff(timeout_sec=timeout_sec)

    def land_all(self, timeout_sec: float = 1.0) -> None:
        for agent in self._agents.values():
            agent.land(timeout_sec=timeout_sec)
    
    def reset_estimator_all(self, timeout_sec: float = 1.0) -> None:
        for agent in self._agents.values():
            agent.ekf_reset(timeout_sec=timeout_sec)

    # ---------------------------------------------------------
    # Single-agent position commands
    # ---------------------------------------------------------

    def go_to_abs(
        self,
        drone_id: str,
        x: float,
        y: float,
        z: float,
        frame_id: str = "map",
    ) -> None:
        """
        Send an absolute position reference to one drone.

        Example:
            swarm.go_to_abs("cf3", 0.0, 0.5, 0.7)
        """
        self.agent(drone_id).go_to_abs(
            x=x,
            y=y,
            z=z,
            frame_id=frame_id,
        )

    def go_to_rel(
        self,
        drone_id: str,
        dx: float,
        dy: float,
        dz: float,
    ) -> None:
        """
        Send a relative position reference to one drone.

        Example:
            swarm.go_to_rel("cf3", 0.1, 0.0, 0.0)
        """
        self.agent(drone_id).go_to_rel(
            dx=dx,
            dy=dy,
            dz=dz,
        )
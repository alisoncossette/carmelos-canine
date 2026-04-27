"""Mock-friendly Go2 navigation driver with connection lifecycle support."""

from __future__ import annotations

import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from hal.base_driver import BaseDriver
from hal.navigation import TargetNavigationBackend
from hal.ros2 import ROS2Bridge

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


class Go2Driver(BaseDriver):
    """Navigation-oriented driver with a dependency-free mock runtime."""

    def __init__(
        self,
        gui: bool = False,
        bridge: ROS2Bridge | None = None,
        *,
        robot_id: str | None = None,
        **kwargs: Any,
    ):
        self._gui = gui
        self._bridge = bridge or ROS2Bridge(enabled=False)
        self.robot_id = str(robot_id or kwargs.get("robot_id") or "go2_edu_001").strip() or "go2_edu_001"
        self._objects: dict[str, dict] = {}
        self._target_navigation_backend = TargetNavigationBackend(
            backend_mode=kwargs.get("target_navigation_backend", "mock"),
            **kwargs,
        )
        self._connection_config = {
            "transport": kwargs.get("transport", "ssh"),
            "host": kwargs.get("host", "192.168.1.23"),
            "port": int(kwargs.get("port", 22)),
            "user": kwargs.get("user", "robot"),
            "auth": kwargs.get("auth", "key"),
            "reconnect_policy": kwargs.get("reconnect_policy", "auto"),
        }
        self._runtime_state = {"robots": {self.robot_id: self._make_robot_state()}}

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / "go2_edu.md"

    def load_scene(self, scene: dict[str, dict]) -> None:
        self._objects = dict(scene)

    def connect(self) -> bool:
        state = self._robot_state(self.robot_id)
        conn = dict(state["connection_state"])
        backend_ok = self._target_navigation_backend.connect()
        conn.update(
            {
                "status": "connected" if backend_ok else "degraded",
                "transport": self._connection_config["transport"],
                "host": self._connection_config["host"],
                "port": self._connection_config["port"],
                "last_heartbeat": self._stamp(),
                "last_error": None,
            }
        )
        state["connection_state"] = conn
        self._refresh_target_navigation_runtime(self.robot_id)
        return backend_ok

    def disconnect(self) -> None:
        state = self._robot_state(self.robot_id)
        conn = dict(state["connection_state"])
        self._target_navigation_backend.disconnect()
        conn.update(
            {
                "status": "disconnected",
                "last_error": None,
            }
        )
        state["connection_state"] = conn

    def is_connected(self) -> bool:
        return self._robot_state(self.robot_id)["connection_state"].get("status") == "connected"

    def health_check(self) -> bool:
        state = self._robot_state(self.robot_id)
        conn = dict(state["connection_state"])
        backend_health = self._target_navigation_backend.health_check()
        if self.is_connected():
            conn["last_heartbeat"] = self._stamp()
            if backend_health.get("status") == "degraded":
                conn["last_error"] = "backend_degraded"
            else:
                conn["last_error"] = None
            state["connection_state"] = conn
            self._refresh_target_navigation_runtime(self.robot_id)
            return True

        if self._connection_config.get("reconnect_policy") == "auto":
            conn["status"] = "reconnecting"
            conn["reconnect_attempts"] = conn.get("reconnect_attempts", 0) + 1
            conn["last_error"] = "connection_lost"
            state["connection_state"] = conn
            return self.connect()
        return False

    def execute_action(self, action_type: str, params: dict) -> str:
        try:
            self._validate_robot_id(params)
        except ValueError as exc:
            return f"Error: {exc}"

        if action_type == "connect_robot":
            self.connect()
            return "Robot connection established."
        if action_type == "disconnect_robot":
            self.disconnect()
            return "Robot connection closed."
        if action_type == "reconnect_robot":
            self.disconnect()
            self.connect()
            return "Robot reconnected."
        if action_type == "check_connection":
            return "connected" if self.health_check() else "disconnected"

        if not self.is_connected() and not self.connect():
            self._update_nav_state(
                params.get("robot_id", self.robot_id),
                mode="idle",
                status="failed",
                last_error="disconnected",
            )
            return "Connection error: robot is not connected."

        if action_type == "semantic_navigate":
            return self._semantic_navigate(params)
        if action_type == "target_navigation":
            return self._target_navigation(params)
        if action_type == "localize":
            return self._localize(params)
        if action_type == "stop":
            robot_id = params.get("robot_id", self.robot_id)
            self._target_navigation_backend.stop()
            self._update_nav_state(
                robot_id,
                mode="idle",
                status="stopped",
                last_error=None,
            )
            return "Navigation stopped."
        return f"Unknown action: {action_type}"

    def get_scene(self) -> dict[str, dict]:
        return dict(self._objects)

    def get_runtime_state(self) -> dict[str, Any]:
        return self._runtime_state

    def _validate_robot_id(self, params: dict[str, Any]) -> None:
        requested = str(params.get("robot_id", "")).strip()
        if requested and requested != self.robot_id:
            raise ValueError(
                f"robot_id mismatch: requested={requested}, configured={self.robot_id}"
            )

    def _semantic_navigate(self, params: dict[str, Any]) -> str:
        robot_id = params.get("robot_id", self.robot_id)
        goal_pose = params.get("goal_pose") or {}
        target_ref = params.get("target_ref") or {}
        mock_status = params.get("mock_status")

        if not target_ref:
            self._update_nav_state(
                robot_id,
                mode="navigating",
                status="failed",
                target_ref={},
                goal=None,
                path_progress=0.0,
                last_error="target_not_found",
            )
            return "Navigation failed: target reference missing."

        if "x" not in goal_pose or "y" not in goal_pose:
            self._update_nav_state(
                robot_id,
                mode="navigating",
                status="failed",
                target_ref=target_ref,
                goal=None,
                path_progress=0.0,
                last_error="planner_timeout",
            )
            return "Navigation failed: goal pose missing."

        if mock_status == "blocked":
            self._update_nav_state(
                robot_id,
                mode="navigating",
                status="blocked",
                target_ref=target_ref,
                goal={
                    "x": float(goal_pose["x"]),
                    "y": float(goal_pose["y"]),
                    "yaw": float(goal_pose.get("yaw", 0.0)),
                },
                path_progress=0.5,
                last_error="recoverable_obstacle",
                recovery_count=1,
            )
            return f"Navigation blocked near {target_ref.get('label', 'target')}."

        state = self._robot_state(robot_id)
        dx = float(goal_pose["x"]) - float(state["robot_pose"]["x"])
        dy = float(goal_pose["y"]) - float(state["robot_pose"]["y"])
        yaw = float(goal_pose.get("yaw", math.atan2(dy, dx) if dx or dy else 0.0))
        state["robot_pose"] = {
            "frame": goal_pose.get("frame", "map"),
            "x": float(goal_pose["x"]),
            "y": float(goal_pose["y"]),
            "z": float(goal_pose.get("z", 0.0)),
            "yaw": yaw,
            "stamp": self._stamp(),
        }
        self._update_nav_state(
            robot_id,
            mode="navigating",
            status="arrived",
            target_ref=target_ref,
            goal={
                "x": float(goal_pose["x"]),
                "y": float(goal_pose["y"]),
                "yaw": yaw,
            },
            path_progress=1.0,
            last_error=None,
        )
        self._bridge.publish("/navigate_to_pose", goal_pose)
        return f"Navigation success: arrived near {target_ref.get('label', 'target')}."

    def _target_navigation(self, params: dict[str, Any]) -> str:
        robot_id = params.get("robot_id", self.robot_id)
        target_label = str(params.get("target_label", "")).strip()
        if not target_label:
            self._update_nav_state(
                robot_id,
                mode="navigating",
                status="failed",
                target_ref=None,
                goal=None,
                path_progress=0.0,
                last_error="target_not_found",
            )
            return "Navigation failed: target_label is required."

        status = self._target_navigation_backend.run_navigation(params)
        self._refresh_target_navigation_runtime(robot_id)
        final_status = self._robot_state(robot_id).get("nav_state", {}).get("status", "idle")
        if final_status == "arrived":
            return f"Target navigation success: arrived near {target_label}."
        if final_status == "blocked":
            return f"Target navigation blocked near {target_label}."
        if final_status == "stopped":
            return f"Target navigation cancelled for {target_label}."
        return f"Target navigation failed for {target_label}: {status.get('message', 'unknown error')}."

    def _localize(self, params: dict[str, Any]) -> str:
        robot_id = params.get("robot_id", self.robot_id)
        state = self._robot_state(robot_id)
        state["robot_pose"]["stamp"] = self._stamp()
        self._update_nav_state(
            robot_id,
            mode="localizing",
            status="localized",
            last_error=None,
            relocalization_confidence=0.82,
        )
        return f"Localization success for {robot_id}."

    def _update_nav_state(
        self,
        robot_id: str,
        *,
        mode: str,
        status: str,
        target_ref: dict[str, Any] | None = None,
        goal: dict[str, Any] | None = None,
        path_progress: float | None = None,
        last_error: str | None = None,
        recovery_count: int | None = None,
        relocalization_confidence: float | None = None,
    ) -> None:
        state = self._robot_state(robot_id)
        current = dict(state.get("nav_state", {}))
        state["nav_state"] = {
            "mode": mode,
            "status": status,
            "goal_id": (target_ref or {}).get("id"),
            "target_ref": target_ref,
            "goal": goal,
            "path_progress": path_progress,
            "recovery_count": current.get("recovery_count", 0) if recovery_count is None else recovery_count,
            "last_error": last_error,
            "relocalization_confidence": relocalization_confidence,
        }

    def _robot_state(self, robot_id: str | None) -> dict[str, Any]:
        robot_id = robot_id or self.robot_id
        robots = self._runtime_state.setdefault("robots", {})
        if robot_id not in robots:
            robots[robot_id] = self._make_robot_state()
        return robots[robot_id]

    def _make_robot_state(self) -> dict[str, Any]:
        return {
            "connection_state": {
                "status": "disconnected",
                "transport": self._connection_config["transport"],
                "host": self._connection_config["host"],
                "port": self._connection_config["port"],
                "last_heartbeat": None,
                "last_error": None,
                "reconnect_attempts": 0,
            },
            "robot_pose": {
                "frame": "map",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "stamp": self._stamp(),
            },
            "nav_state": {
                "mode": "idle",
                "status": "idle",
                "goal_id": None,
                "target_ref": None,
                "goal": None,
                "path_progress": None,
                "recovery_count": 0,
                "last_error": None,
                "relocalization_confidence": None,
                "target_label": None,
                "active_horizon_target": None,
                "history_tail": [],
            },
        }

    def _refresh_target_navigation_runtime(self, robot_id: str) -> None:
        runtime = self._target_navigation_backend.snapshot_runtime(
            robot_id,
            current_state=self._robot_state(robot_id),
        )
        self._runtime_state.setdefault("robots", {}).update(runtime)

    @staticmethod
    def _stamp() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

"""HAL driver for the Seeed reBot Arm B601-DM.

The reBot Arm is a 6+1 DoF open-source manipulator built around Damiao
brushless servos (DM4310 / DM4340P) communicating over CAN via a USB-to-CAN
adapter. The bundle ships a parallel-jaw gripper and is advertised as
LeRobot / ROS2 / Isaac Sim compatible.

This driver mirrors the structure of :class:`SO101Driver` so that the action
vocabulary and Critic-facing surface stay consistent across LeRobot-family
arms. Mock mode requires no extra dependencies. Real-hardware mode lazily
imports the Damiao Python SDK; the exact package is selected at connect
time so that frameworks shipping different bindings can plug in.
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from hal.base_driver import BaseDriver

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"

# Default joint targets for the home pose (radians). Conservative — arm folded
# into a compact, table-safe configuration.
DEFAULT_HOME = [0.0, -0.5, 1.0, 0.0, 0.0, 0.0]

# Reach envelope: 767 mm per Seeed spec sheet. Used as a soft software check
# in mock mode; real hardware reach is enforced by the QP solver / firmware.
REACH_ENVELOPE_M = 0.767

# Maximum advertised payload (kg). Critic uses this as a hard upper bound
# when validating grasp action parameters.
MAX_PAYLOAD_KG = 1.5

# Shoulder rotation point relative to the base mount (metres). Used by the
# mock reach check; replace with values from the URDF when one is published.
SHOULDER_OFFSET_LOCAL = (0.0, 0.0, 0.40)

# 6-DoF arm joints. The gripper is exposed as a separate state variable
# rather than a 7th joint so the action surface matches the SO-101 driver.
JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_pitch",
    "wrist_yaw",
    "wrist_roll",
)

# Default Damiao motor model assumption. Override at instantiation time when
# the user knows their bundle ships DM4340P (higher-torque variant).
# TODO: hardware verify — confirm DM4310 vs DM4340P for the B601-DM bundle.
DEFAULT_MOTOR_MODEL = "DM4310"

# Default calibration cache path. Damiao SDKs vary; this location mirrors the
# LeRobot calibration convention so future LeRobot integration drops in
# cleanly.
DEFAULT_CALIBRATION_PATH = (
    Path.home()
    / ".cache"
    / "huggingface"
    / "lerobot"
    / "calibration"
    / "robots"
    / "rebot_arm"
    / "rebot_arm.json"
)


class ReBotArmDriver(BaseDriver):
    """Mock-first driver for the Seeed reBot Arm B601-DM.

    Parameters
    ----------
    port:
        Serial path to the USB-to-CAN adapter (Damiao 43 USB2CAN board).
        On Linux typically ``/dev/ttyUSB0``; on macOS ``/dev/tty.usbmodem*``.
    mock:
        When ``True`` (default), no hardware I/O is performed and joint state
        is simulated. When ``False``, the Damiao SDK is imported lazily and
        a CAN bus session is opened on ``port``.
    robot_id:
        Stable identifier for ENVIRONMENT.md scene-graph keys.
    base_pose_world:
        Pose of the arm base in world coordinates (``(x, y, z)`` metres).
    calibration_path:
        Override for the calibration JSON. Falls back to
        :data:`DEFAULT_CALIBRATION_PATH`.
    motor_model:
        ``"DM4310"`` (default, lighter) or ``"DM4340P"`` (higher-torque).
        Affects torque limits the driver advertises but not control logic.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        mock: bool = True,
        robot_id: str = "rebot_arm_001",
        base_pose_world: tuple[float, float, float] = (0.0, 0.0, 0.0),
        calibration_path: str | Path | None = None,
        motor_model: str = DEFAULT_MOTOR_MODEL,
        **_kwargs: Any,
    ) -> None:
        self._port = port
        self._mock = mock
        self.robot_id = robot_id
        self._base_pose_world = base_pose_world
        self._motor_model = motor_model
        self._objects: dict[str, dict] = {}
        self._joint_angles = list(DEFAULT_HOME)
        self._gripper_state = "open"
        self._gripper_position = 0.0  # 0.0 = open, 1.0 = closed (Damiao position-controlled)
        self._holding: str | None = None
        self._end_effector_world = self._compute_end_effector_world()
        self._bus: Any = None
        self._connected = False
        self._calibration_path = (
            Path(calibration_path) if calibration_path else DEFAULT_CALIBRATION_PATH
        )
        if not mock:
            self._connect_hardware()

    # ------------------------------------------------------------------ #
    # Hardware bring-up                                                  #
    # ------------------------------------------------------------------ #

    def _connect_hardware(self) -> None:
        """Open the Damiao CAN bus and read the present joint positions.

        The Damiao Python SDK is imported lazily so mock-mode users do not
        need it installed. If the user's bundle ships a different binding
        (the LeRobot integration is one such option), set
        ``REBOT_ARM_BACKEND=lerobot`` in the environment to switch.
        """
        backend = self._select_backend()
        if backend == "damiao":
            self._connect_damiao()
        elif backend == "lerobot":
            self._connect_lerobot()
        else:  # pragma: no cover — guarded by _select_backend
            raise ValueError(f"unknown reBot Arm backend: {backend!r}")

    def _select_backend(self) -> str:
        import os

        # TODO: hardware verify — once we confirm which SDK Seeed ships with
        # the B601-DM, harden this default rather than env-flag.
        return os.environ.get("REBOT_ARM_BACKEND", "damiao").lower()

    def _connect_damiao(self) -> None:
        try:
            # Damiao publishes a Python SDK alongside their motor firmware.
            # The exact import path is verified against the B601-DM bundle.
            # TODO: hardware verify — confirm package name and import path.
            from damiao_python_sdk import DamiaoMotorBus  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "Real reBot Arm hardware mode requires the Damiao Python SDK. "
                "Install via: pip install damiao_python_sdk\n"
                "If your bundle ships a different binding, set "
                "REBOT_ARM_BACKEND=lerobot to use the LeRobot path instead."
            ) from exc

        if not self._calibration_path.exists():
            raise FileNotFoundError(
                f"reBot Arm calibration not found at {self._calibration_path}. "
                "Run the Damiao calibration tool (or LeRobot calibrate) first."
            )

        with open(self._calibration_path) as f:
            calibration = json.load(f)

        self._bus = DamiaoMotorBus(
            port=self._port,
            motor_model=self._motor_model,
            joint_names=list(JOINT_NAMES) + ["gripper"],
            calibration=calibration,
        )
        self._bus.connect()
        self._bus.enable_torque()
        self._connected = True

        # Read the live joint state so mock-vs-real first contact is consistent.
        positions = self._bus.read_positions()
        self._joint_angles = [float(positions[name]) for name in JOINT_NAMES]

    def _connect_lerobot(self) -> None:
        # The LeRobot integration claim from Seeed's product page is unverified
        # at the time of writing. When confirmed, this branch should mirror the
        # SO-101 driver's lerobot wiring (see ``so101_driver.py``).
        # TODO: hardware verify — flesh out once LeRobot integration is tested.
        raise NotImplementedError(
            "LeRobot backend for reBot Arm is not yet implemented. "
            "Set REBOT_ARM_BACKEND=damiao or run in mock mode."
        )

    # ------------------------------------------------------------------ #
    # BaseDriver interface                                               #
    # ------------------------------------------------------------------ #

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / "rebot_arm.md"

    def load_scene(self, scene: dict[str, dict]) -> None:
        self._objects = {k: dict(v) for k, v in scene.items()}

    def execute_action(self, action_type: str, params: dict) -> str:
        handler = self._handlers.get(action_type)
        if handler is None:
            return f"unknown action: {action_type}"
        try:
            return handler(self, params or {})
        except (KeyError, TypeError, ValueError) as exc:
            return f"error: {exc}"

    def get_scene(self) -> dict[str, dict]:
        scene = {k: dict(v) for k, v in self._objects.items()}
        if self._holding and self._holding in scene:
            scene[self._holding]["carried_by"] = self.robot_id
            scene[self._holding]["position"] = {
                "x": self._end_effector_world[0],
                "y": self._end_effector_world[1],
                "z": self._end_effector_world[2],
            }
        return scene

    def get_runtime_state(self) -> dict[str, Any]:
        if self._bus is not None and self._connected:
            try:
                live = self._bus.read_positions()
                self._joint_angles = [float(live[name]) for name in JOINT_NAMES]
                self._gripper_position = float(live.get("gripper", self._gripper_position))
            except Exception:
                # Read failures are tolerated — the runtime state will reflect
                # the last known good values until the next successful read.
                pass
        return {
            "robots": {
                self.robot_id: {
                    "arm": {
                        "joint_angles_rad": list(self._joint_angles),
                        "gripper_state": self._gripper_state,
                        "gripper_position": self._gripper_position,
                        "holding": self._holding,
                        "end_effector_world": list(self._end_effector_world),
                        "motor_model": self._motor_model,
                        "stamp": _now_iso(),
                    }
                }
            }
        }

    def is_connected(self) -> bool:
        return self._mock or self._connected

    def close(self) -> None:
        if self._bus is not None and self._connected:
            try:
                self._bus.disconnect()
            finally:
                self._bus = None
                self._connected = False

    # ------------------------------------------------------------------ #
    # Action handlers                                                    #
    # ------------------------------------------------------------------ #

    def _do_home(self, _params: dict) -> str:
        self._joint_angles = list(DEFAULT_HOME)
        self._end_effector_world = self._compute_end_effector_world()
        if not self._mock:
            self._write_motor_targets(self._joint_angles)
        return f"home: joint_angles={self._joint_angles}"

    def _do_move_to_pose(self, params: dict) -> str:
        if not self._mock:
            return (
                "error: move_to_pose not supported on real reBot Arm hardware "
                "(no IK available); use 'move_to_joints' with explicit joint targets"
            )
        pose = params.get("pose")
        if not isinstance(pose, (list, tuple)) or len(pose) != 3:
            return "error: 'pose' must be a 3-element list [x, y, z]"
        if not self._within_reach(pose):
            return f"error: pose {list(pose)} is outside reach envelope ({REACH_ENVELOPE_M} m)"
        self._joint_angles = self._inverse_kinematics(pose)
        self._end_effector_world = tuple(float(v) for v in pose)
        return f"moved to {list(pose)}"

    def _do_move_to_joints(self, params: dict) -> str:
        joints = params.get("joints")
        if not isinstance(joints, (list, tuple)) or len(joints) != 6:
            return "error: 'joints' must be a 6-element list [j1..j6]"
        try:
            joints = [float(v) for v in joints]
        except (TypeError, ValueError):
            return "error: 'joints' values must be numeric"
        self._joint_angles = list(joints)
        if not self._mock:
            self._write_motor_targets(joints)
        return f"moved to joints {joints}"

    def _do_set_gripper(self, params: dict) -> str:
        """Position-controlled gripper command.

        Damiao motors run in position mode by default, so the gripper exposes
        a normalised closure parameter (0.0 fully open .. 1.0 fully closed)
        rather than a force command.
        """
        value = params.get("value")
        try:
            value = float(value)
        except (TypeError, ValueError):
            return "error: 'value' must be numeric in range [0.0, 1.0]"
        if not 0.0 <= value <= 1.0:
            return f"error: gripper value {value} outside [0.0, 1.0]"
        self._gripper_position = value
        self._gripper_state = "closed" if value > 0.5 else "open"
        if not self._mock:
            self._write_gripper_target(value)
        return f"gripper set to {value:.2f} ({self._gripper_state})"

    def _do_grasp(self, params: dict) -> str:
        if not self._mock:
            return (
                "error: grasp not supported on real reBot Arm hardware "
                "(no IK available); use 'move_to_joints' to position, then 'set_gripper'"
            )
        target_id = params.get("target_id")
        if not target_id:
            return "error: 'target_id' is required"
        if target_id not in self._objects:
            return f"error: target_id {target_id!r} not found in scene"
        if self._holding is not None:
            return f"error: gripper already holding {self._holding!r}"
        # Conservative payload check against advertised maximum.
        target_mass = float(self._objects[target_id].get("mass_kg") or 0.0)
        if target_mass > MAX_PAYLOAD_KG:
            return (
                f"error: target {target_id!r} mass {target_mass} kg exceeds "
                f"max payload {MAX_PAYLOAD_KG} kg"
            )
        pose = params.get("pose") or self._object_pose(target_id)
        if pose is None:
            return f"error: no pose available for {target_id!r}"
        if not self._within_reach(pose):
            return f"error: target {target_id!r} at {pose} is outside reach envelope"
        move_result = self._do_move_to_pose({"pose": pose})
        if move_result.startswith("error:"):
            return move_result
        self._gripper_state = "closed"
        self._gripper_position = 1.0
        self._holding = target_id
        return f"grasped {target_id} at {list(pose)}"

    def _do_release(self, _params: dict) -> str:
        if not self._mock:
            return (
                "error: release not supported on real reBot Arm hardware; "
                "use 'move_to_joints' to position, then 'set_gripper' with value=0.0"
            )
        if self._holding is None:
            return "error: nothing to release"
        released = self._holding
        self._holding = None
        self._gripper_state = "open"
        self._gripper_position = 0.0
        return f"released {released}"

    _handlers: dict = {}

    # ------------------------------------------------------------------ #
    # Utilities                                                          #
    # ------------------------------------------------------------------ #

    def _within_reach(self, pose) -> bool:
        sx = self._base_pose_world[0] + SHOULDER_OFFSET_LOCAL[0]
        sy = self._base_pose_world[1] + SHOULDER_OFFSET_LOCAL[1]
        sz = self._base_pose_world[2] + SHOULDER_OFFSET_LOCAL[2]
        dx, dy, dz = pose[0] - sx, pose[1] - sy, pose[2] - sz
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= REACH_ENVELOPE_M

    def _inverse_kinematics(self, _pose) -> list[float]:
        # Mock-only: real IK belongs in a constraint solver downstream of this
        # driver, not in the HAL layer.
        return list(DEFAULT_HOME)

    def _compute_end_effector_world(self) -> tuple[float, float, float]:
        bx, by, bz = self._base_pose_world
        # Approximate forward extension at the home pose. Replace with a
        # forward-kinematics computation once a URDF is wired in.
        return (bx + 0.45, by, bz + 0.55)

    def _object_pose(self, target_id: str) -> tuple[float, float, float] | None:
        obj = self._objects.get(target_id, {})
        pos = obj.get("position")
        if isinstance(pos, dict) and {"x", "y", "z"} <= pos.keys():
            return float(pos["x"]), float(pos["y"]), float(pos["z"])
        if isinstance(pos, (list, tuple)) and len(pos) == 3:
            return float(pos[0]), float(pos[1]), float(pos[2])
        return None

    def _write_motor_targets(self, joints: list[float]) -> None:
        if self._bus is None or not self._connected:
            raise RuntimeError("reBot Arm hardware bus is not connected")
        targets = {name: float(value) for name, value in zip(JOINT_NAMES, joints)}
        self._bus.write_positions(targets)

    def _write_gripper_target(self, value: float) -> None:
        if self._bus is None or not self._connected:
            raise RuntimeError("reBot Arm hardware bus is not connected")
        self._bus.write_positions({"gripper": float(value)})


ReBotArmDriver._handlers = {
    "home": ReBotArmDriver._do_home,
    "move_to_pose": ReBotArmDriver._do_move_to_pose,
    "move_to_joints": ReBotArmDriver._do_move_to_joints,
    "set_gripper": ReBotArmDriver._do_set_gripper,
    "grasp": ReBotArmDriver._do_grasp,
    "release": ReBotArmDriver._do_release,
}


def _now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")

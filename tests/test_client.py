import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from artemis_vicon.client import ArtemisViconClient
from artemis_vicon.config import EngineConfig, RunConfig, load_run_config
from artemis_vicon.engine import create_engine_client
from artemis_vicon.engine.mudri_zmq import (
    MudriObservationAdapter,
    MudriZmqEngineClient,
    command_to_step_request,
)
from artemis_vicon.schemas import ControlCommand, MotorMode


class ConfigTest(unittest.TestCase):
    def test_load_run_config_requires_yaml_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(
                """
engine:
  kind: mudri_zmq
  endpoint: tcp://127.0.0.1:5556
controller:
  task_path: examples/m0/task1.json
""",
                encoding="utf-8",
            )

            config = load_run_config(path)

        self.assertEqual(config.engine.kind, "mudri_zmq")
        self.assertEqual(config.engine.endpoint, "tcp://127.0.0.1:5556")
        self.assertEqual(config.controller.task_path, Path("examples/m0/task1.json"))
        self.assertEqual(config.start.control_period_s, 0.02)

    def test_load_run_config_resolves_task_path_from_environment(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(
                """
engine:
  kind: mudri_zmq
  endpoint: tcp://127.0.0.1:5556
controller:
  task_path: ${oc.env:ARTEMIS_M0_TASK_PATH}
  line_tracking_pid:
    ki: 0.1
    kp: 2.0
    kd: 0.3
  yaw_hold_pid:
    ki: 0.0
    kp: 0.4
    kd: 0.02
""",
                encoding="utf-8",
            )

            with patch.dict(os.environ, {"ARTEMIS_M0_TASK_PATH": "examples/m0/task3.json"}):
                config = load_run_config(path)

        self.assertEqual(config.controller.task_path, Path("examples/m0/task3.json"))
        self.assertEqual(config.controller.line_tracking_pid.ki, 0.1)
        self.assertEqual(config.controller.line_tracking_pid.kp, 2.0)
        self.assertEqual(config.controller.line_tracking_pid.kd, 0.3)
        self.assertEqual(config.controller.yaw_hold_pid.kp, 0.4)

    def test_load_run_config_rejects_missing_task_path_environment(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(
                """
engine:
  kind: mudri_zmq
  endpoint: tcp://127.0.0.1:5556
controller:
  task_path: ${oc.env:ARTEMIS_M0_TASK_PATH}
""",
                encoding="utf-8",
            )

            with patch.dict(os.environ, {}, clear=False):
                os.environ.pop("ARTEMIS_M0_TASK_PATH", None)
                with self.assertRaises(Exception):
                    load_run_config(path)

    def test_grpc_engine_is_explicit_placeholder(self) -> None:
        with self.assertRaisesRegex(NotImplementedError, "gRPC external engine ABI is not defined yet"):
            create_engine_client(EngineConfig(kind="grpc", endpoint="127.0.0.1:50051"))


class MudriObservationAdapterTest(unittest.TestCase):
    def test_adapter_prefers_digital_line_sensor_values(self) -> None:
        adapter = MudriObservationAdapter(line_sensor_darkness_threshold=0.55)

        observation = adapter.from_wire(
            _wire_observation(
                line_sensor={
                    "digital": [True, False, True],
                    "darkness": [0.0, 1.0, 0.0],
                },
                line_sensor_darkness=[0.0, 1.0, 0.0],
            )
        )

        self.assertEqual(observation.digital_values.tolist(), [1, 0, 1])

    def test_adapter_falls_back_to_darkness_threshold(self) -> None:
        adapter = MudriObservationAdapter(line_sensor_darkness_threshold=0.55)

        observation = adapter.from_wire(
            _wire_observation(
                line_sensor={"darkness": [0.54, 0.55, 0.9]},
                line_sensor_darkness=[0.54, 0.55, 0.9],
            )
        )

        self.assertEqual(observation.digital_values.tolist(), [0, 1, 1])

    def test_adapter_uses_encoder_distance_without_kinematics_integration(self) -> None:
        adapter = MudriObservationAdapter()

        observation = adapter.from_wire(
            _wire_observation(
                sequence_id=7,
                sim_time_s=1.5,
                yaw_deg=12.0,
                forward_distance_cm=42.5,
            )
        )

        self.assertEqual(observation.sequence_id, 7)
        self.assertEqual(float(observation.sim_time_s), np.float32(1.5))
        self.assertEqual(float(observation.yaw_deg), 12.0)
        self.assertEqual(float(observation.forward_distance_cm), 42.5)


class MudriZmqEngineClientTest(unittest.TestCase):
    def test_command_encodes_only_step_wheel_targets(self) -> None:
        command = ControlCommand(
            sequence_id=7,
            velocity=1.0,
            turn=2.0,
            rear_left_target_speed=3.0,
            rear_right_target_speed=4.0,
            rear_left_mode=MotorMode.TRACK,
            rear_right_mode=MotorMode.TRACK,
        )

        request = command_to_step_request(command)

        self.assertEqual(
            request,
            {
                "type": "step",
                "sequence_id": 7,
                "rear_left_target_speed": 3.0,
                "rear_right_target_speed": 4.0,
            },
        )

    def test_zmq_client_runs_start_step_stop_over_request_socket(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            task_path = Path(directory) / "task.json"
            task_path.write_text(
                """
{
  "actions": [
    {"kind": "drive_for", "duration_s": 0.01},
    {"kind": "finish"}
  ]
}
""",
                encoding="utf-8",
            )
            socket = FakeSocket(
                [
                    {
                        "type": "started",
                        "started": {"time_limit_s": 6.0, "control_period_s": 0.02},
                        "observation": _wire_observation(sequence_id=0),
                    },
                    {"type": "observation", "observation": _wire_observation(sequence_id=1, sim_time_s=0.02)},
                    {
                        "type": "finished",
                        "finished": {
                            "reason": "task_completed",
                            "summary": {"reached_goal": True, "elapsed_time_s": 0.02},
                        },
                    },
                ]
            )
            client = ArtemisViconClient(
                config=_run_config(task_path=task_path),
                engine_client=MudriZmqEngineClient("inproc://test", socket_factory=lambda: socket),
            )

            result = client.run()

        self.assertEqual(result.reason, "task_completed")
        self.assertTrue(result.reached_goal)
        self.assertEqual([request["type"] for request in socket.requests], ["start", "step", "stop"])
        self.assertEqual(socket.requests[1]["sequence_id"], 0)

    def test_zmq_client_returns_finished_response_from_step(self) -> None:
        socket = FakeSocket(
            [
                {
                    "type": "started",
                    "started": {"time_limit_s": 6.0, "control_period_s": 0.02},
                    "observation": _wire_observation(sequence_id=0),
                },
                {
                    "type": "finished",
                    "finished": {
                        "reason": "time_limit",
                        "summary": {"reached_goal": False, "elapsed_time_s": 6.0},
                    },
                },
            ]
        )
        client = ArtemisViconClient(
            config=_run_config(),
            engine_client=MudriZmqEngineClient("inproc://test", socket_factory=lambda: socket),
        )

        result = client.run()

        self.assertEqual(result.reason, "time_limit")
        self.assertFalse(result.reached_goal)
        self.assertEqual([request["type"] for request in socket.requests], ["start", "step"])

    def test_zmq_client_raises_engine_error_response(self) -> None:
        socket = FakeSocket([{"type": "error", "error": "boom"}])
        client = MudriZmqEngineClient("inproc://test", socket_factory=lambda: socket)

        with self.assertRaisesRegex(RuntimeError, "boom"):
            client.start(_run_config().start)


class FakeSocket:
    def __init__(self, responses: list[dict]) -> None:
        self.responses = responses
        self.requests: list[dict] = []
        self.connected_endpoint = ""
        self.closed = False

    def connect(self, endpoint: str) -> None:
        self.connected_endpoint = endpoint

    def send_json(self, request: dict) -> None:
        self.requests.append(request)

    def recv_json(self) -> dict:
        return self.responses.pop(0)

    def close(self, *, linger: int = 0) -> None:
        del linger
        self.closed = True


def _run_config(*, task_path: Path = Path("examples/m0/task3.json")) -> RunConfig:
    return RunConfig.model_validate(
        {
            "engine": {"kind": "mudri_zmq", "endpoint": "tcp://127.0.0.1:5556"},
            "start": {"max_time_s": 6.0, "control_period_s": 0.02},
            "controller": {
                "task_path": str(task_path),
                "line_sensor_darkness_threshold": 0.55,
            },
        }
    )


def _wire_observation(
    *,
    sequence_id: int = 1,
    sim_time_s: float = 0.0,
    yaw_deg: float = 0.0,
    forward_distance_cm: float = 0.0,
    line_sensor: dict | None = None,
    line_sensor_darkness: list[float] | None = None,
) -> dict:
    if line_sensor_darkness is None:
        line_sensor_darkness = [0.0] * 8
    if line_sensor is None:
        line_sensor = {"digital": [False] * len(line_sensor_darkness), "darkness": line_sensor_darkness}
    return {
        "sequence_id": sequence_id,
        "sim_time_s": sim_time_s,
        "line_sensor_darkness": line_sensor_darkness,
        "line_sensor": line_sensor,
        "imu": {"yaw_deg": yaw_deg, "yaw_rate_deg_s": 0.0},
        "encoder": {"forward_distance_cm": forward_distance_cm},
        "kinematics": {"longitudinal_velocity_m_s": 999.0},
    }


if __name__ == "__main__":
    unittest.main()

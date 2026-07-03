import unittest

from artemis_vicon.bridge import (
    SerialMudriBridge,
    SerialProtocolError,
    SerialStartCommand,
    SerialStepCommand,
    SerialStopCommand,
    parse_serial_command,
)
from artemis_vicon.config import StartConfig
from artemis_vicon.engine.base import EngineFinished, EngineObservation, EngineStarted
from artemis_vicon.schemas import ControlCommand


class SerialProtocolTest(unittest.TestCase):
    def test_parse_start_uses_mudri_start_defaults(self) -> None:
        command = parse_serial_command("START max_time_s=12 random_seed=3")

        self.assertEqual(
            command,
            SerialStartCommand(max_time_s=12.0, control_period_s=0.02, initial_progress_index=0, random_seed=3),
        )

    def test_parse_step_accepts_positional_compact_form(self) -> None:
        command = parse_serial_command("STEP 7 3.5 4.5")

        self.assertEqual(command, SerialStepCommand(sequence_id=7, rear_left_target_speed=3.5, rear_right_target_speed=4.5))

    def test_parse_step_accepts_key_value_form(self) -> None:
        command = parse_serial_command("T seq=8 left=-1.0 right=2.0")

        self.assertEqual(command, SerialStepCommand(sequence_id=8, rear_left_target_speed=-1.0, rear_right_target_speed=2.0))

    def test_parse_stop_defaults_reason(self) -> None:
        command = parse_serial_command("STOP")

        self.assertEqual(command, SerialStopCommand(reason="mcu_stop"))

    def test_parse_rejects_unknown_command(self) -> None:
        with self.assertRaisesRegex(SerialProtocolError, "unknown command"):
            parse_serial_command("PING")


class SerialMudriBridgeTest(unittest.TestCase):
    def test_bridge_forwards_start_step_stop_and_writes_compact_responses(self) -> None:
        serial = FakeSerial(
            [
                b"START max_time_s=6\n",
                b"STEP 0 7.0 8.0\n",
                b"STOP task_completed\n",
            ]
        )
        engine = FakeEngine()
        bridge = SerialMudriBridge(engine_client=engine, serial_stream=serial)

        self.assertTrue(bridge.run_once())
        self.assertTrue(bridge.run_once())
        self.assertTrue(bridge.run_once())

        self.assertEqual(engine.started_configs[0].max_time_s, 6.0)
        self.assertEqual(engine.commands[0].sequence_id, 0)
        self.assertEqual(float(engine.commands[0].rear_left_target_speed), 7.0)
        self.assertEqual(float(engine.commands[0].rear_right_target_speed), 8.0)
        self.assertEqual(engine.stop_reasons, ["task_completed"])
        self.assertEqual(
            serial.writes,
            [
                b"STARTED time_limit_s=6 control_period_s=0.02 seq=0 t=0 yaw=1.5 distance_cm=2.5 digital=1010\n",
                b"OBS seq=1 t=0.02 yaw=2.5 distance_cm=3.5 digital=0101\n",
                b"FINISHED reason=task_completed reached_goal=1 elapsed_time_s=0.02\n",
            ],
        )

    def test_bridge_returns_error_line_for_invalid_command(self) -> None:
        serial = FakeSerial([b"BAD\n"])
        bridge = SerialMudriBridge(engine_client=FakeEngine(), serial_stream=serial)

        self.assertTrue(bridge.run_once())

        self.assertEqual(serial.writes, [b"ERR message=unknown_command:_BAD\n"])


class FakeSerial:
    def __init__(self, lines: list[bytes]) -> None:
        self.lines = lines
        self.writes: list[bytes] = []
        self.closed = False

    def readline(self) -> bytes:
        if not self.lines:
            return b""
        return self.lines.pop(0)

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def close(self) -> None:
        self.closed = True


class FakeEngine:
    def __init__(self) -> None:
        self.started_configs: list[StartConfig] = []
        self.commands: list[ControlCommand] = []
        self.stop_reasons: list[str] = []
        self.closed = False

    def start(self, config: StartConfig) -> EngineStarted:
        self.started_configs.append(config)
        return EngineStarted(
            time_limit_s=6.0,
            control_period_s=0.02,
            observation=_wire_observation(
                sequence_id=0,
                sim_time_s=0.0,
                yaw_deg=1.5,
                forward_distance_cm=2.5,
                digital=[1, 0, 1, 0],
            ),
        )

    def step(self, command: ControlCommand) -> EngineObservation | EngineFinished:
        self.commands.append(command)
        return EngineObservation(
            observation=_wire_observation(
                sequence_id=1,
                sim_time_s=0.02,
                yaw_deg=2.5,
                forward_distance_cm=3.5,
                digital=[0, 1, 0, 1],
            )
        )

    def stop(self, reason: str) -> EngineFinished:
        self.stop_reasons.append(reason)
        return EngineFinished(reason=reason, summary={"reached_goal": True, "elapsed_time_s": 0.02})

    def close(self) -> None:
        self.closed = True


def _wire_observation(
    *,
    sequence_id: int,
    sim_time_s: float,
    yaw_deg: float,
    forward_distance_cm: float,
    digital: list[int],
) -> dict:
    return {
        "sequence_id": sequence_id,
        "sim_time_s": sim_time_s,
        "line_sensor": {"digital": digital},
        "imu": {"yaw_deg": yaw_deg},
        "encoder": {"forward_distance_cm": forward_distance_cm},
    }


if __name__ == "__main__":
    unittest.main()

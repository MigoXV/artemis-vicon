from __future__ import annotations
"""Serial-to-Mudri bridge service."""

import logging
from typing import Protocol

from artemis_vicon.bridge.serial_protocol import (
    SerialProtocolError,
    SerialStartCommand,
    SerialStepCommand,
    SerialStopCommand,
    format_error,
    format_finished,
    format_observation,
    format_started,
    parse_serial_command,
)
from artemis_vicon.engine.base import EngineClient, EngineFinished
from artemis_vicon.engine.mudri_zmq import MudriObservationAdapter
from artemis_vicon.schemas import ControlCommand, MotorMode

logger = logging.getLogger(__name__)


class SerialStream(Protocol):
    def readline(self) -> bytes | str:
        """Read one newline-delimited MCU command."""

    def write(self, data: bytes) -> int | None:
        """Write one newline-delimited response."""

    def close(self) -> None:
        """Close the serial device."""


class SerialMudriBridge:
    """Bridge MCU serial commands to the artemis-mudri ZMQ JSON ABI."""

    def __init__(
        self,
        *,
        engine_client: EngineClient,
        serial_stream: SerialStream,
        adapter: MudriObservationAdapter | None = None,
        encoding: str = "ascii",
    ) -> None:
        self.engine_client = engine_client
        self.serial_stream = serial_stream
        self.adapter = adapter or MudriObservationAdapter()
        self.encoding = encoding

    def serve_forever(self) -> None:
        """Run until interrupted or the serial stream raises an exception."""

        while True:
            self.run_once()

    def run_once(self) -> bool:
        """Read, handle, and respond to a single serial line.

        Returns False when the read timed out or produced an empty line.
        """

        raw_line = self.serial_stream.readline()
        if raw_line in {b"", ""}:
            return False

        line = _decode_line(raw_line, self.encoding)
        response = self.handle_line(line)
        if response is None:
            return False
        self._write_line(response)
        return True

    def handle_line(self, line: str) -> str | None:
        line = line.strip()
        if not line:
            return None

        try:
            command = parse_serial_command(line)
            if isinstance(command, SerialStartCommand):
                started = self.engine_client.start(command.to_start_config())
                return format_started(started, self.adapter)
            if isinstance(command, SerialStepCommand):
                result = self.engine_client.step(_control_command(command))
                if isinstance(result, EngineFinished):
                    return format_finished(result)
                return format_observation("OBS", result.observation, self.adapter)
            if isinstance(command, SerialStopCommand):
                return format_finished(self.engine_client.stop(command.reason))
        except (SerialProtocolError, KeyError, RuntimeError, ValueError) as exc:
            logger.warning("Serial bridge command failed line=%r error=%s", line, exc)
            return format_error(str(exc))

        return format_error("unsupported command")

    def close(self) -> None:
        self.engine_client.close()
        self.serial_stream.close()

    def _write_line(self, line: str) -> None:
        self.serial_stream.write((line + "\n").encode(self.encoding))


def _control_command(command: SerialStepCommand) -> ControlCommand:
    return ControlCommand(
        sequence_id=command.sequence_id,
        velocity=0.0,
        turn=0.0,
        rear_left_target_speed=command.rear_left_target_speed,
        rear_right_target_speed=command.rear_right_target_speed,
        rear_left_mode=MotorMode.SPEED,
        rear_right_mode=MotorMode.SPEED,
    )


def _decode_line(raw_line: bytes | str, encoding: str) -> str:
    if isinstance(raw_line, bytes):
        return raw_line.decode(encoding, errors="replace")
    return raw_line

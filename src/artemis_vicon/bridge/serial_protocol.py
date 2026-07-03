from __future__ import annotations
"""Line-oriented MCU serial protocol for the Mudri bridge."""

from dataclasses import dataclass
import shlex

from artemis_vicon.config import StartConfig
from artemis_vicon.engine.base import EngineFinished, EngineStarted, JsonObject
from artemis_vicon.engine.mudri_zmq import MudriObservationAdapter


class SerialProtocolError(ValueError):
    """Raised when an MCU serial line cannot be decoded."""


@dataclass(frozen=True)
class SerialStartCommand:
    max_time_s: float | None = None
    control_period_s: float | None = 0.02
    initial_progress_index: int = 0
    random_seed: int | None = None

    def to_start_config(self) -> StartConfig:
        return StartConfig(
            max_time_s=self.max_time_s,
            control_period_s=self.control_period_s,
            initial_progress_index=self.initial_progress_index,
            random_seed=self.random_seed,
        )


@dataclass(frozen=True)
class SerialStepCommand:
    sequence_id: int
    rear_left_target_speed: float
    rear_right_target_speed: float


@dataclass(frozen=True)
class SerialStopCommand:
    reason: str = "mcu_stop"


SerialCommand = SerialStartCommand | SerialStepCommand | SerialStopCommand


def parse_serial_command(line: str) -> SerialCommand:
    """Parse one MCU command line.

    Supported commands:
    START [max_time_s=120] [control_period_s=0.02] [initial_progress_index=0] [random_seed=1]
    STEP <sequence_id> <rear_left_target_speed> <rear_right_target_speed>
    STEP sequence_id=1 left=7.0 right=7.0
    STOP [reason]
    """

    tokens = shlex.split(line.strip())
    if not tokens:
        raise SerialProtocolError("empty command")

    command = tokens[0].upper()
    if command in {"S", "START"}:
        return _parse_start(tokens[1:])
    if command in {"T", "STEP"}:
        return _parse_step(tokens[1:])
    if command in {"X", "STOP"}:
        return _parse_stop(tokens[1:])
    raise SerialProtocolError(f"unknown command: {tokens[0]}")


def format_started(started: EngineStarted, adapter: MudriObservationAdapter) -> str:
    observation = _observation_fields(adapter, started.observation)
    return (
        f"STARTED time_limit_s={started.time_limit_s:g} "
        f"control_period_s={started.control_period_s:g} {observation}"
    )


def format_observation(prefix: str, observation: JsonObject, adapter: MudriObservationAdapter) -> str:
    return f"{prefix} {_observation_fields(adapter, observation)}"


def format_finished(finished: EngineFinished) -> str:
    reached_goal = int(bool(finished.summary.get("reached_goal", False)))
    elapsed_time_s = float(finished.summary.get("elapsed_time_s", 0.0))
    return (
        f"FINISHED reason={_safe_token(finished.reason)} "
        f"reached_goal={reached_goal} elapsed_time_s={elapsed_time_s:g}"
    )


def format_error(message: str) -> str:
    return f"ERR message={_safe_token(message)}"


def _parse_start(tokens: list[str]) -> SerialStartCommand:
    options = _parse_key_values(tokens)
    return SerialStartCommand(
        max_time_s=_optional_float(options, "max_time_s"),
        control_period_s=_optional_float(options, "control_period_s", default=0.02),
        initial_progress_index=_optional_int(options, "initial_progress_index", default=0),
        random_seed=_optional_int(options, "random_seed"),
    )


def _parse_step(tokens: list[str]) -> SerialStepCommand:
    if len(tokens) == 3 and all("=" not in token for token in tokens):
        try:
            return SerialStepCommand(
                sequence_id=int(tokens[0]),
                rear_left_target_speed=float(tokens[1]),
                rear_right_target_speed=float(tokens[2]),
            )
        except ValueError as exc:
            raise SerialProtocolError("STEP expects: sequence_id left_speed right_speed") from exc

    options = _parse_key_values(tokens)
    return SerialStepCommand(
        sequence_id=_required_int(options, "sequence_id", aliases=("seq",)),
        rear_left_target_speed=_required_float(
            options,
            "rear_left_target_speed",
            aliases=("left", "l"),
        ),
        rear_right_target_speed=_required_float(
            options,
            "rear_right_target_speed",
            aliases=("right", "r"),
        ),
    )


def _parse_stop(tokens: list[str]) -> SerialStopCommand:
    if not tokens:
        return SerialStopCommand()
    if len(tokens) == 1 and tokens[0].startswith("reason="):
        return SerialStopCommand(reason=tokens[0].split("=", 1)[1] or "mcu_stop")
    return SerialStopCommand(reason="_".join(tokens))


def _parse_key_values(tokens: list[str]) -> dict[str, str]:
    options: dict[str, str] = {}
    for token in tokens:
        if "=" not in token:
            raise SerialProtocolError(f"expected key=value token: {token}")
        key, value = token.split("=", 1)
        if not key:
            raise SerialProtocolError("empty option key")
        options[key] = value
    return options


def _required_float(options: dict[str, str], key: str, *, aliases: tuple[str, ...] = ()) -> float:
    value = _lookup(options, key, aliases)
    if value is None:
        raise SerialProtocolError(f"missing option: {key}")
    try:
        return float(value)
    except ValueError as exc:
        raise SerialProtocolError(f"{key} must be a float") from exc


def _required_int(options: dict[str, str], key: str, *, aliases: tuple[str, ...] = ()) -> int:
    value = _lookup(options, key, aliases)
    if value is None:
        raise SerialProtocolError(f"missing option: {key}")
    try:
        return int(value)
    except ValueError as exc:
        raise SerialProtocolError(f"{key} must be an integer") from exc


def _optional_float(options: dict[str, str], key: str, *, default: float | None = None) -> float | None:
    value = options.get(key)
    if value is None or value == "null":
        return default
    try:
        return float(value)
    except ValueError as exc:
        raise SerialProtocolError(f"{key} must be a float") from exc


def _optional_int(options: dict[str, str], key: str, *, default: int | None = None) -> int | None:
    value = options.get(key)
    if value is None or value == "null":
        return default
    try:
        return int(value)
    except ValueError as exc:
        raise SerialProtocolError(f"{key} must be an integer") from exc


def _lookup(options: dict[str, str], key: str, aliases: tuple[str, ...]) -> str | None:
    for candidate in (key, *aliases):
        if candidate in options:
            return options[candidate]
    return None


def _observation_fields(adapter: MudriObservationAdapter, observation: JsonObject) -> str:
    parsed = adapter.from_wire(observation)
    digital = "".join("1" if int(value) else "0" for value in parsed.digital_values)
    return (
        f"seq={parsed.sequence_id} "
        f"t={float(parsed.sim_time_s):g} "
        f"yaw={float(parsed.yaw_deg):g} "
        f"distance_cm={float(parsed.forward_distance_cm):g} "
        f"digital={digital}"
    )


def _safe_token(value: object) -> str:
    text = str(value).strip()
    if not text:
        return "none"
    return "_".join(text.split())

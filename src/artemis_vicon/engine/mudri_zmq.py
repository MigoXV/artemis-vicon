from __future__ import annotations
"""artemis-mudri ZMQ JSON ABI 客户端。"""

from collections.abc import Callable
from typing import Any

import numpy as np

from artemis_vicon.config import StartConfig
from artemis_vicon.engine.base import EngineFinished, EngineObservation, EngineStarted, JsonObject
from artemis_vicon.schemas import ControlCommand, Observation

SocketFactory = Callable[[], Any]


class MudriZmqEngineClient:
    """通过 ZMQ REQ/REP 访问 artemis-mudri ABI。"""

    def __init__(self, endpoint: str, *, socket_factory: SocketFactory | None = None) -> None:
        self.endpoint = endpoint
        if socket_factory is None:
            import zmq

            context = zmq.Context.instance()
            self._socket = context.socket(zmq.REQ)
        else:
            self._socket = socket_factory()
        self._socket.connect(endpoint)

    def start(self, config: StartConfig) -> EngineStarted:
        request: JsonObject = {"type": "start"}
        if config.max_time_s is not None:
            request["max_time_s"] = config.max_time_s
        if config.control_period_s is not None:
            request["control_period_s"] = config.control_period_s
        if config.initial_pose is not None:
            request["initial_pose"] = config.initial_pose.model_dump()
        request["initial_progress_index"] = config.initial_progress_index
        if config.random_seed is not None:
            request["random_seed"] = config.random_seed

        response = self._request(request)
        if response.get("type") != "started":
            raise RuntimeError(f"Unexpected start response: {response.get('type')!r}.")
        started = _require_object(response, "started")
        return EngineStarted(
            time_limit_s=float(started["time_limit_s"]),
            control_period_s=float(started["control_period_s"]),
            observation=_require_object(response, "observation"),
        )

    def step(self, command: ControlCommand) -> EngineObservation | EngineFinished:
        response = self._request(command_to_step_request(command))
        response_type = response.get("type")
        if response_type == "observation":
            return EngineObservation(observation=_require_object(response, "observation"))
        if response_type == "finished":
            return _finished_from_response(response)
        raise RuntimeError(f"Unexpected step response: {response_type!r}.")

    def stop(self, reason: str) -> EngineFinished:
        response = self._request({"type": "stop", "reason": reason})
        if response.get("type") != "finished":
            raise RuntimeError(f"Unexpected stop response: {response.get('type')!r}.")
        return _finished_from_response(response)

    def close(self) -> None:
        self._socket.close(linger=0)

    def _request(self, request: JsonObject) -> JsonObject:
        self._socket.send_json(request)
        response = self._socket.recv_json()
        if not isinstance(response, dict):
            raise RuntimeError("Engine response must be a JSON object.")
        if response.get("type") == "error":
            raise RuntimeError(str(response.get("error") or "Unknown engine error."))
        return response


class MudriObservationAdapter:
    """把 mudri ABI observation 适配成控制器输入。"""

    def __init__(self, *, line_sensor_darkness_threshold: float = 0.55) -> None:
        self.line_sensor_darkness_threshold = line_sensor_darkness_threshold

    def from_wire(self, observation: JsonObject) -> Observation:
        line_sensor = observation.get("line_sensor")
        if not isinstance(line_sensor, dict):
            line_sensor = {}
        imu = _require_object(observation, "imu")
        encoder = _require_object(observation, "encoder")
        return Observation(
            sequence_id=int(observation["sequence_id"]),
            sim_time_s=np.float32(float(observation["sim_time_s"])),
            yaw_deg=np.float32(float(imu["yaw_deg"])),
            digital_values=self._digital_values(observation, line_sensor),
            forward_distance_cm=np.float32(float(encoder["forward_distance_cm"])),
        )

    def _digital_values(self, observation: JsonObject, line_sensor: JsonObject) -> np.ndarray:
        digital = line_sensor.get("digital")
        if digital is not None:
            return np.asarray(digital, dtype=np.int_)
        darkness = observation.get("line_sensor_darkness")
        if darkness is None:
            darkness = line_sensor.get("darkness")
        if darkness is None:
            raise KeyError("line_sensor.digital or line_sensor_darkness")
        return (np.asarray(darkness, dtype=np.float32) >= self.line_sensor_darkness_threshold).astype(np.int_)


def command_to_step_request(command: ControlCommand) -> JsonObject:
    """把控制器命令编码成 mudri ABI step 请求。"""

    return {
        "type": "step",
        "sequence_id": int(command.sequence_id),
        "rear_left_target_speed": float(command.rear_left_target_speed),
        "rear_right_target_speed": float(command.rear_right_target_speed),
    }


def _finished_from_response(response: JsonObject) -> EngineFinished:
    finished = _require_object(response, "finished")
    return EngineFinished(
        reason=str(finished["reason"]),
        summary=_require_object(finished, "summary"),
    )


def _require_object(message: JsonObject, key: str) -> JsonObject:
    value = message[key]
    if not isinstance(value, dict):
        raise RuntimeError(f"{key} must be a JSON object.")
    return value

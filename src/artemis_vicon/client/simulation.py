from __future__ import annotations
"""仿真服务 gRPC 客户端封装。"""

import logging
import queue
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import grpc
import numpy as np

from artemis_vicon.protos.simulation.v1 import vehicle_simulation_pb2 as pb2
from artemis_vicon.protos.simulation.v1 import vehicle_simulation_pb2_grpc as pb2_grpc
from artemis_vicon.schemas import ControlCommand, MotorMode, Observation
from artemis_vicon.vehicle import MissionStateMachine, load_task_actions

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ClientResult:
    """客户端运行结果。"""

    reason: str
    reached_goal: bool
    elapsed_time_s: float


@dataclass(frozen=True)
class ArtemisViconClient:
    """连接仿真服务并运行 artemis-m0 风格任务控制器。"""

    target: str = "127.0.0.1:50051"
    task_path: Path = Path("examples/m0/task1.json")
    max_time_s: float | None = None
    control_period_s: float = 0.02
    random_seed: int | None = None

    def run(self) -> ClientResult:
        """运行一次仿真 episode。"""

        request_queue: queue.Queue[pb2.ClientMessage | None] = queue.Queue()
        start_request = pb2.ClientMessage(start=self._start_episode_request())
        controller = MissionStateMachine(load_task_actions(self.task_path))

        with grpc.insecure_channel(self.target) as channel:
            stub = pb2_grpc.VehicleSimulationServiceStub(channel)
            responses = stub.StreamEpisode(_request_iterator(start_request, request_queue))
            for response in responses:
                payload = response.WhichOneof("payload")
                if payload == "started":
                    logger.info(
                        "Connected to simulation task=%s time_limit=%s control_period=%s",
                        self.task_path,
                        response.started.time_limit_s,
                        response.started.control_period_s,
                    )
                    continue
                if payload == "observation":
                    command = controller.step(_observation_from_proto(response.observation))
                    if command.completed:
                        request_queue.put(pb2.ClientMessage(stop=pb2.StopEpisodeRequest(reason="task_completed")))
                    else:
                        request_queue.put(pb2.ClientMessage(control_command=_command_to_proto(command)))
                    continue
                if payload == "finished":
                    request_queue.put(None)
                    summary = response.finished.summary
                    return ClientResult(
                        reason=response.finished.reason,
                        reached_goal=summary.reached_goal,
                        elapsed_time_s=summary.elapsed_time_s,
                    )
                if payload == "error":
                    request_queue.put(None)
                    raise RuntimeError(response.error.message)

        return ClientResult(reason="stream_closed", reached_goal=False, elapsed_time_s=0.0)

    def _start_episode_request(self) -> pb2.StartEpisodeRequest:
        start = pb2.StartEpisodeRequest(
            max_time_s=self.max_time_s or 0.0,
            control_period_s=self.control_period_s,
        )
        if self.random_seed is not None:
            start.random_seed = self.random_seed
        return start


def _request_iterator(
    start_request: pb2.ClientMessage,
    request_queue: queue.Queue[pb2.ClientMessage | None],
) -> Iterator[pb2.ClientMessage]:
    yield start_request
    while True:
        message = request_queue.get()
        if message is None:
            return
        yield message


def _observation_from_proto(frame: pb2.ObservationFrame) -> Observation:
    return Observation(
        sequence_id=int(frame.sequence_id),
        sim_time_s=np.float32(frame.sim_time_s),
        yaw_deg=np.float32(frame.imu.yaw_deg),
        digital_values=np.fromiter(frame.line_sensor.digital_values, dtype=np.int_),
        forward_distance_cm=np.float32(frame.encoder.forward_distance_cm),
    )


def _command_to_proto(command: ControlCommand) -> pb2.VehicleControlCommand:
    return pb2.VehicleControlCommand(
        sequence_id=command.sequence_id,
        velocity=command.velocity,
        turn=command.turn,
        rear_left_target_speed=command.rear_left_target_speed,
        rear_right_target_speed=command.rear_right_target_speed,
        rear_left_mode=_mode_to_proto(command.rear_left_mode),
        rear_right_mode=_mode_to_proto(command.rear_right_mode),
    )


def _mode_to_proto(mode: MotorMode) -> int:
    return int(mode)

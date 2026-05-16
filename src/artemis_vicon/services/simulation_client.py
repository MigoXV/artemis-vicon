from __future__ import annotations
"""仿真服务 gRPC 客户端编排。"""

import logging
import queue
from collections.abc import Iterator
from dataclasses import dataclass

import grpc

from artemis_vicon.firmware import ArtemisM0FirmwareController, FirmwareCommand, FirmwareObservation, MotorMode
from artemis_vicon.simulation.v1 import vehicle_simulation_pb2 as pb2
from artemis_vicon.simulation.v1 import vehicle_simulation_pb2_grpc as pb2_grpc

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ClientResult:
    """客户端运行结果。"""

    reason: str
    reached_goal: bool
    elapsed_time_s: float


def run_client(
    target: str = "127.0.0.1:50051",
    task_id: str = "1",
    max_time_s: float | None = None,
    control_period_s: float = 0.02,
    random_seed: int | None = None,
    initial_yaw_noise_deg: float = 5.0,
) -> ClientResult:
    """连接仿真服务并运行 artemis-m0 风格固件控制器。"""

    request_queue: queue.Queue[pb2.ClientMessage | None] = queue.Queue()
    start = pb2.StartEpisodeRequest(
        task_id=task_id,
        max_time_s=max_time_s or 0.0,
        control_period_s=control_period_s,
        initial_yaw_noise_deg=initial_yaw_noise_deg,
    )
    if random_seed is not None:
        start.random_seed = random_seed
    start_request = pb2.ClientMessage(start=start)
    controller = ArtemisM0FirmwareController(task_id=task_id)

    with grpc.insecure_channel(target) as channel:
        stub = pb2_grpc.VehicleSimulationServiceStub(channel)
        responses = stub.StreamEpisode(_request_iterator(start_request, request_queue))
        for response in responses:
            payload = response.WhichOneof("payload")
            if payload == "started":
                controller = ArtemisM0FirmwareController(task_id=response.started.task_id)
                logger.info("Connected to simulation task=%s", response.started.task_id)
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


def _observation_from_proto(frame: pb2.ObservationFrame) -> FirmwareObservation:
    return FirmwareObservation(
        sequence_id=int(frame.sequence_id),
        sim_time_s=float(frame.sim_time_s),
        yaw_deg=float(frame.imu.yaw_deg),
        digital_values=tuple(int(value) for value in frame.line_sensor.digital_values),
        forward_distance_cm=float(frame.encoder.forward_distance_cm),
    )


def _command_to_proto(command: FirmwareCommand) -> pb2.VehicleControlCommand:
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

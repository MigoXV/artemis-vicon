from concurrent import futures
import unittest

import grpc

from artemis_vicon.services import run_client
from artemis_vicon.simulation.v1 import vehicle_simulation_pb2 as pb2
from artemis_vicon.simulation.v1 import vehicle_simulation_pb2_grpc as pb2_grpc


class FakeVehicleSimulationService(pb2_grpc.VehicleSimulationServiceServicer):
    def StreamEpisode(self, request_iterator, context):
        first_message = next(request_iterator)
        if first_message.WhichOneof("payload") != "start":
            yield pb2.ServerMessage(error=pb2.SimulationError(message="start required"))
            return

        task_id = first_message.start.task_id or "1"
        yield pb2.ServerMessage(
            started=pb2.EpisodeStarted(
                task_id=task_id,
                label="Task 1",
                description="Drive from A to B and stop.",
                time_limit_s=15.0,
                control_period_s=0.01,
            )
        )
        yield pb2.ServerMessage(
            observation=pb2.ObservationFrame(
                sequence_id=1,
                sim_time_s=0.0,
                line_sensor=pb2.LineSensorFrame(
                    digital_values=[0, 0, 0, 0, 0, 0, 0, 0],
                    darkness=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    line_detected=False,
                    weights=[-4, -3, -2, -1, 1, 2, 3, 4],
                    error=0.0,
                ),
                imu=pb2.ImuFrame(yaw_deg=0.0, yaw_rate_deg_s=0.0),
                task_progress=pb2.TaskProgressFrame(
                    task_id=task_id,
                    active_segment_index=0,
                    completed_event_count=0,
                    completed_events=[],
                    reached_goal=False,
                ),
                encoder=pb2.EncoderFrame(
                    rear_left_pulses=0.0,
                    rear_right_pulses=0.0,
                    rear_left_total_pulses=0.0,
                    rear_right_total_pulses=0.0,
                    rear_left_measure_speed=0.0,
                    rear_right_measure_speed=0.0,
                    forward_distance_cm=0.0,
                ),
                motor_debug=pb2.MotorDebugFrame(),
            )
        )
        command_message = next(request_iterator)
        if command_message.WhichOneof("payload") != "control_command":
            yield pb2.ServerMessage(error=pb2.SimulationError(message="control command required"))
            return
        yield pb2.ServerMessage(
            finished=pb2.EpisodeFinished(
                reason="goal_reached",
                summary=pb2.SimulationSummary(
                    task_id=task_id,
                    reached_goal=True,
                    elapsed_time_s=0.01,
                    route_length_m=1.0,
                    max_cross_track_error_m=0.0,
                    rms_cross_track_error_m=0.0,
                    final_pose=pb2.Pose2D(x_m=1.6, y_m=1.0, yaw_rad=0.0),
                    events=[],
                ),
            )
        )


def create_fake_grpc_server() -> grpc.Server:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    pb2_grpc.add_VehicleSimulationServiceServicer_to_server(
        FakeVehicleSimulationService(),
        server,
    )
    return server


class ArtemisViconClientTest(unittest.TestCase):
    def test_m0_firmware_client_finishes_task1_against_in_process_service(self) -> None:
        server = create_fake_grpc_server()
        port = server.add_insecure_port("127.0.0.1:0")
        server.start()
        try:
            result = run_client(
                target=f"127.0.0.1:{port}",
                task_id="1",
                max_time_s=6.0,
            )
        finally:
            server.stop(0)
        self.assertEqual(result.reason, "goal_reached")
        self.assertTrue(result.reached_goal)


if __name__ == "__main__":
    unittest.main()

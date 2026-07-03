from __future__ import annotations
"""CLI entry point for the MCU serial bridge."""

import logging
from typing import Annotated

import typer

from artemis_vicon.bridge import SerialMudriBridge
from artemis_vicon.engine.mudri_zmq import MudriObservationAdapter, MudriZmqEngineClient

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
logger = logging.getLogger(__name__)

app = typer.Typer(add_completion=False, help="Bridge MCU serial lines to artemis-mudri ZMQ JSON.")


@app.command()
def main(
    port: Annotated[
        str,
        typer.Option("--port", "-p", envvar="ARTEMIS_SERIAL_PORT", show_envvar=True, help="MCU serial port."),
    ],
    engine_endpoint: Annotated[
        str,
        typer.Option(
            "--engine-endpoint",
            "--endpoint",
            "-e",
            envvar="ARTEMIS_MUDRI_ENDPOINT",
            show_envvar=True,
            help="artemis-mudri ZMQ JSON endpoint.",
        ),
    ] = "tcp://127.0.0.1:5556",
    baudrate: Annotated[
        int,
        typer.Option("--baudrate", "-b", envvar="ARTEMIS_SERIAL_BAUDRATE", show_envvar=True),
    ] = 115200,
    timeout_s: Annotated[
        float,
        typer.Option("--timeout-s", min=0.0, help="Serial read/write timeout in seconds."),
    ] = 0.1,
    line_sensor_darkness_threshold: Annotated[
        float,
        typer.Option(
            "--line-sensor-darkness-threshold",
            envvar="ARTEMIS_LINE_SENSOR_DARKNESS_THRESHOLD",
            show_envvar=True,
            help="Fallback threshold for Mudri darkness arrays.",
        ),
    ] = 0.55,
) -> None:
    """Run the serial bridge."""

    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is required. Install dependencies with `poetry install`.") from exc

    logger.info("Starting serial bridge port=%s baudrate=%s engine_endpoint=%s", port, baudrate, engine_endpoint)
    serial_stream = serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=timeout_s,
        write_timeout=timeout_s,
    )
    bridge = SerialMudriBridge(
        engine_client=MudriZmqEngineClient(engine_endpoint),
        serial_stream=serial_stream,
        adapter=MudriObservationAdapter(line_sensor_darkness_threshold=line_sensor_darkness_threshold),
    )

    try:
        bridge.serve_forever()
    except KeyboardInterrupt:
        logger.info("Serial bridge stopped by user.")
    finally:
        bridge.close()


if __name__ == "__main__":
    app()

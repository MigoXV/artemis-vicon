from __future__ import annotations

import time

import numpy as np

from artemis_vicon.controllers.pid import PIDController


ITERATIONS = 1000


def error_series(iterations: int) -> np.ndarray:
    indexes = np.arange(iterations, dtype=np.float32)
    return ((indexes % np.float32(17.0)) - np.float32(8.0)) / np.float32(3.0)


def benchmark(name: str, controller: PIDController, errors: np.ndarray) -> tuple[np.float32, np.float32]:
    output = np.float32(0.0)
    setpoints = np.zeros_like(errors, dtype=np.float32)
    observations = -errors

    start = time.perf_counter()
    for output, _ in controller.infer_stream(iter(setpoints), iter(observations)):
        pass
    elapsed = np.float32(time.perf_counter() - start)

    print(
        f"{name}: {elapsed * 1_000_000.0:.2f} us total, "
        f"{elapsed / errors.size * 1_000_000.0:.4f} us/iter, last={output:.6f}"
    )
    return elapsed, output


def main() -> None:
    controller = PIDController(ki=0.0, kp=25.0, kd=3.5)
    errors = error_series(ITERATIONS)

    elapsed, _ = benchmark("numpy-stream", controller, errors)
    print(f"iterations: {errors.size}")
    print(f"elapsed_s: {elapsed:.9f}")


if __name__ == "__main__":
    main()

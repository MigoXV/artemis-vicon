from __future__ import annotations

import numpy as np

from artemis_vicon.controllers.base import ControlValue


def default_error_fn(setpoint: ControlValue, observation: ControlValue) -> ControlValue:
    error = np.asarray(setpoint, dtype=np.float32) - np.asarray(observation, dtype=np.float32)
    if error.shape == ():
        return np.float32(error)
    return error.astype(np.float32)

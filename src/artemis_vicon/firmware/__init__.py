"""artemis-m0 风格固件控制器。"""

from artemis_vicon.firmware.controller import ArtemisM0FirmwareController
from artemis_vicon.firmware.math_utils import clamp, normalize_360, wrap_deg
from artemis_vicon.firmware.track import TrackController
from artemis_vicon.firmware.types import ActionKind, FirmwareAction, FirmwareCommand, FirmwareObservation, MotorMode
from artemis_vicon.firmware.yaw import YawController

__all__ = [
    "ActionKind",
    "ArtemisM0FirmwareController",
    "FirmwareAction",
    "FirmwareCommand",
    "FirmwareObservation",
    "MotorMode",
    "TrackController",
    "YawController",
    "clamp",
    "normalize_360",
    "wrap_deg",
]

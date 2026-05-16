from __future__ import annotations
"""固件控制器使用的数学工具。"""


def wrap_deg(angle_deg: float) -> float:
    """将角度误差规整到 [-180, 180]。"""

    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def normalize_360(angle_deg: float) -> float:
    """将角度规整到 [0, 360)。"""

    return float(angle_deg % 360.0)


def clamp(value: float, lower: float, upper: float) -> float:
    """将数值限制到闭区间 [lower, upper]。"""

    return min(max(value, lower), upper)

"""Data exchanged by perception, decision, and control.

The existing camera geometry uses metres, x forward and y left. Decision
converts its centre curve to vehicle coordinates (x forward, y right) once.
Vehicle speed is m/s, requested/target speed is km/h, and dt is seconds.
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np
from numpy.polynomial import Polynomial


@dataclass(frozen=True)
class LaneObservation:
    """Camera-ground boundary curves y(x), in metres; None means not found.

    These come from the existing neural detector. The road-boundary preview's
    pixel coordinates are a different format and cannot be passed here as-is.
    """
    left: Optional[Polynomial]
    right: Optional[Polynomial]
    overlay: np.ndarray
    # Farthest CURRENT image evidence, not the extrapolated polynomial range.
    left_extent_m: float = 0.0
    right_extent_m: float = 0.0
    # Separate forbidden median; never treat its opposite side as our lane.
    centerline: Optional[Polynomial] = None
    centerline_extent_m: float = 0.0
    right_double_yellow: bool = False


@dataclass(frozen=True)
class DrivePlan:
    """Vehicle-frame Nx2 path and the decision to drive or stop."""
    trajectory: Optional[np.ndarray]
    target_kmh: float
    status: str
    drive_allowed: bool
    stop_brake: float
    lane_width_m: float
    left_valid: bool
    right_valid: bool


@dataclass(frozen=True)
class ControlCommand:
    """Normalized CARLA controls, with the effective target/status.

    Control can report NO STEERING TARGET - STOP even for a drivable plan.
    """
    throttle: float
    brake: float
    steer: float
    target_kmh: float
    status: str

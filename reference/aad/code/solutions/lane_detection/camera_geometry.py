"""Compatibility import; edit the implementation in perception/ instead."""
from ..perception.camera_geometry import (
    CameraGeometry,
    get_intrinsic_matrix,
    project_polyline
)

__all__ = ['CameraGeometry', 'get_intrinsic_matrix', 'project_polyline']

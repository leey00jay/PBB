"""Compatibility import; edit the implementation in perception/ instead."""
from ..perception.road_boundary_recovery import (
    BoundaryTracker,
    process_frame,
    draw_result,
    demo_frame,
    save_image,
    main,
    _fit_yellow,
    _fit_curb
)

__all__ = ['BoundaryTracker', 'process_frame', 'draw_result', 'demo_frame', 'save_image', 'main']

if __name__ == "__main__":
    main()

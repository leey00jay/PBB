"""Render the lane observations that are actually sent to the planner."""
import cv2
import numpy as np
from .camera_geometry import project_polyline


def draw_observation(rgb, observation, diagnostics, camera):
    output = rgb.copy()
    transform = np.linalg.inv(camera.trafo_cam_to_road)
    for index, (side, color) in enumerate((('left', (60, 140, 255)), ('right', (255, 90, 80)))):
        curve = getattr(observation, side)
        extent = getattr(observation, side + '_extent_m')
        info = diagnostics[side]
        if curve is not None:
            x = np.linspace(4., max(4.5, min(26., extent)), 90)
            ground = np.column_stack((-curve(x), np.zeros(len(x)), x))
            uv = project_polyline(ground, transform, camera.intrinsic_matrix)
            if np.isfinite(uv).all():
                cv2.polylines(output, [np.clip(uv, -10000, 10000).astype(np.int32)],
                              False, color, 2, cv2.LINE_AA)
            source = {'paint_tracking':'PAINT', 'double_yellow':'YELLOW'}.get(info.get('source'),'MODEL')
            label = f'{side.upper()}: {source}  seen to {extent:.1f} m'
        else:
            label = f'{side.upper()}: {info.get("reason", "lost")}'
        cv2.putText(output, label, (8, 19 + index*21), cv2.FONT_HERSHEY_SIMPLEX,
                    .43, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(output, label, (8, 19 + index*21), cv2.FONT_HERSHEY_SIMPLEX,
                    .43, color, 1, cv2.LINE_AA)
    if observation.centerline is not None:
        x = np.linspace(2., max(2.5,min(26.,observation.centerline_extent_m)),70)
        ground = np.column_stack((-observation.centerline(x),np.zeros(len(x)),x))
        uv = project_polyline(ground,transform,camera.intrinsic_matrix)
        if np.isfinite(uv).all():
            cv2.polylines(output,[np.clip(uv,-10000,10000).astype(np.int32)],
                          False,(255,210,30),2,cv2.LINE_AA)
    if observation.right_double_yellow:
        cv2.putText(output,'RIGHT DOUBLE YELLOW - STOP',(8,65),
                    cv2.FONT_HERSHEY_SIMPLEX,.5,(255,40,40),2,cv2.LINE_AA)
    return output

"""Current-image double yellow markings, kept separate from lane boundaries.

Row pairs must be two thin, separated stripes with a continuous longitudinal
ground trace. Yellow objects, a single line and transverse paint are rejected.
Coordinates are the same x-forward, y-left frame as lane observations.
"""
import cv2
import numpy as np
from numpy.polynomial import Polynomial


def double_yellow(rgb, grid, cut_v, side_view=False):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = ((hsv[:, :, 0] >= 15) & (hsv[:, :, 0] <= 40)
            & (hsv[:, :, 1] > 70) & (hsv[:, :, 2] > 65))
    ground = grid.reshape(rgb.shape[0] - cut_v, rgb.shape[1], 2)
    pairs = []
    for row, points in zip(mask[cut_v:], ground):
        pixels = np.flatnonzero(row)
        if len(pixels) < 4:
            continue
        runs = np.split(pixels, np.flatnonzero(np.diff(pixels) > 1) + 1)
        stripes = []
        for run in runs:
            if len(run) < 2:
                continue
            p = points[run]
            width = np.linalg.norm(p[-1] - p[0])
            middle = np.mean(p, axis=0)
            if .02 <= width <= .28 and 2 <= middle[0] <= 25 and abs(middle[1]) < 7:
                stripes.append(middle)
        for a, b in zip(stripes, stripes[1:]):
            separation = np.linalg.norm(a-b)
            if .18 <= separation <= .65:
                pairs.append((*(.5*(a+b)), separation))
    if len(pairs) < 8:
        return None, 0.
    p = np.asarray(pairs)
    # Aggregate in ground distance so many near image rows cannot hide a
    # disconnected far marking. Every fit needs current near evidence.
    minimum_span = 1.2 if side_view else 3.
    minimum_bins = 5 if side_view else 6
    bins = np.floor(p[:, 0]*(4 if side_view else 2)).astype(int)
    p = np.array([np.median(p[bins == b], axis=0) for b in np.unique(bins)])
    groups = np.split(p, np.flatnonzero(np.diff(p[:, 0]) > 2.5)+1)
    groups = [g for g in groups if len(g) >= minimum_bins and np.ptp(g[:, 0]) >= minimum_span and g[0, 0] <= 8]
    if not groups:
        return None, 0.
    p = max(groups, key=lambda g: np.ptp(g[:, 0]))
    for _ in range(3):
        if len(p) < minimum_bins or np.ptp(p[:, 0]) < minimum_span or p[:, 0].min() > 8:
            return None, 0.
        degree = 2 if np.ptp(p[:, 0]) >= 3 else 1
        curve = Polynomial.fit(p[:, 0], p[:, 1], degree, w=1/np.sqrt(p[:, 0])).convert()
        residual = np.abs(curve(p[:, 0])-p[:, 1])
        keep = residual < max(.10, 2.5*float(np.median(residual)))
        if keep.all():
            break
        p = p[keep]
    else:
        return None, 0.
    if (np.max(residual) > .22 or np.ptp(p[:, 2]) > .18
            or np.max(np.diff(p[:, 0])) > 2.5
            or np.max(np.abs(curve.deriv()(p[:, 0]))) > 1.0
            or abs(float(curve.deriv(2)(0))) > .10):
        return None, 0.
    return curve, float(p[:, 0].max())

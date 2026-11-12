"""Confirm distant straight paint without refitting the near steering curve."""
import numpy as np


def straight_extent(mask, grid, cut_v, curve, near_extent):
    if curve is None or near_extent < 18:
        return near_extent
    check = np.linspace(4., 25., 20)
    slope = curve.deriv()(check)
    curvature = np.abs(curve.deriv(2)(check))/(1+slope*slope)**1.5
    if np.max(np.abs(slope)) >= .10 or np.max(curvature) >= .0025:
        return near_extent
    x, y = grid.T
    # Require real, narrow paint close to the already measured near curve.
    use = mask[cut_v:].ravel() & (x >= 20) & (x <= 45)
    x, y = x[use], y[use]
    # Small quadratic fitting noise grows as x² far outside the steering fit.
    # Verify a straight continuation of the near tangent against CURRENT far
    # paint; a real bend leaving that corridor ends the visible straight.
    straight_y = curve(8.) + curve.deriv()(8.)*(x-8.)
    residual = y - straight_y
    support = []
    bins = np.floor(x).astype(int)
    for key in np.unique(bins):
        take = bins == key
        bx, by = x[take], residual[take]
        order = np.argsort(by)
        bx, by = bx[order], by[order]
        groups = np.split(np.arange(len(by)), np.flatnonzero(np.diff(by) > .12)+1)
        for group in groups:
            # At 35–45 m a stripe can occupy just one camera pixel. It is only
            # used as part of the continuous multi-bin support checked below.
            minimum_pixels = 1 if float(np.median(bx[group])) > 35 else 2
            if (len(group) >= minimum_pixels and np.ptp(by[group]) <= .35
                    and abs(float(np.median(by[group]))) <= .25):
                support.append(float(np.median(bx[group])))
                break
    # Do not jump a blank interval to unrelated far paint or crosswalks.
    previous = start = None
    count = 0
    extent = near_extent
    for distance in support:
        if previous is None:
            if distance > min(near_extent, 26.)+2.:
                break
            start = distance
        elif distance-previous > 4.:
            break
        count += 1
        previous = distance
        if count >= 5 and distance-start >= 8.:
            extent = max(extent, distance)
    return float(extent)

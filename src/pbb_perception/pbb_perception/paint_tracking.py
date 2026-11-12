"""Fit current white/yellow paint in ground coordinates, including bends.

No map or remembered pixels are used as observations. A prior only selects a
search corridor; every returned boundary needs support in the current image.
"""
import numpy as np
from numpy.polynomial import Polynomial


def paint_mask(rgb):
    import cv2
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    top = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT,
                         cv2.getStructuringElement(cv2.MORPH_RECT, (19, 3)))
    local = cv2.GaussianBlur(gray, (19, 19), 0).astype(float)
    contrast = top > np.maximum(10, local * .10)
    white = (gray > 55) & (hsv[:, :, 1] < 105) & contrast
    yellow = ((hsv[:, :, 0] >= 15) & (hsv[:, :, 0] <= 40)
              & (hsv[:, :, 1] > 70) & (hsv[:, :, 2] > 65) & (top > 8))
    return white | yellow


def fit_paint(mask, grid, cut_v, side, prior=None, extra_points=None, near_side=False):
    x, y = grid.T
    selected = (mask[cut_v:].ravel() & (x >= 4) & (x <= 26) & (np.abs(y) < 8))
    x, y = x[selected], y[selected]
    if extra_points is not None and len(extra_points):
        extra = np.asarray(extra_points)
        use = ((extra[:, 0] >= (1.5 if near_side else 4))
               & (extra[:, 0] <= (8 if near_side else 26)) & (np.abs(extra[:, 1]) < 8))
        x = np.concatenate((x, extra[use, 0]))
        y = np.concatenate((y, extra[use, 1]))
    if len(x) < 40:
        return None, {'reason': 'too_few_paint_pixels'}
    shape = prior if prior is not None else Polynomial([1.75 if side == 'left' else -1.75])
    if near_side and prior is None:
        # The +/-45 degree cameras see the adjacent stripe mainly at 1.5-5 m.
        # Seed a fresh track from that stripe, not an old center-camera curve.
        sign = 1 if side == 'left' else -1
        nearby = y[(x <= 4) & (sign*y >= .6) & (sign*y <= 3.5)]
        if len(nearby) < 20:
            return None, {'reason': 'no_near_side_stripe'}
        hist, edges = np.histogram(nearby, bins=np.arange(-3.6, 3.61, .2))
        peak = int(np.argmax(hist))
        shape = Polynomial([.5*(edges[peak]+edges[peak+1])])
    bins = np.floor(x * (4 if near_side else 2)).astype(int)
    points = []
    for key in np.unique(bins):
        take = bins == key
        bx, by = x[take], y[take]
        reference = float(np.median(bx))
        expected = float(shape(reference))
        if points and prior is None:
            px, py = np.array(points[-6:]).T
            offset = py - shape(px)
            slope = 0.0 if len(px) < 3 else float(np.polyfit(px, offset, 1)[0])
            expected += offset[-1] + np.clip(slope, -.5, .5) * (reference - px[-1])
        corridor = .55 + .025 * (reference - 4)
        keep = np.abs(by - expected) < corridor
        bx, by = bx[keep], by[keep]
        if len(by) < 3:
            continue
        # A bend traverses y within each x bin. Measure stripe thickness
        # perpendicular to its local tangent, not its total lateral span.
        # Otherwise a real curved solid line is rejected as broad paint.
        tangent = float(shape.deriv()(reference))
        cross = (by-tangent*(bx-reference))/np.sqrt(1+tangent*tangent)
        order = np.argsort(cross)
        bx, by, cross = bx[order], by[order], cross[order]
        groups = np.split(np.arange(len(by)), np.flatnonzero(np.diff(cross) > .10) + 1)
        candidates = []
        for group in groups:
            if len(group) < 3 or np.ptp(cross[group]) > .38:
                continue  # broad transverse paint/shadow is not a lane stripe
            mx, my = float(np.median(bx[group])), float(np.median(by[group]))
            candidates.append((abs(my - expected), mx, my))
        if candidates:
            _, mx, my = min(candidates)
            points.append((mx, my))
    if len(points) < 6:
        return None, {'reason': 'paint_support_too_short'}
    px, py = np.array(points).T
    span = float(np.ptp(px))
    minimum_span = 1.2 if near_side else 2.5 if prior is not None else 7.0
    if px.min() > (4 if near_side else 10) or span < minimum_span:
        return None, {'reason': 'paint_support_too_short'}
    degree = 2 if span >= 6.0 and len(px) >= 10 else 1
    base = shape if degree == 1 and prior is not None else Polynomial([0.])
    for _ in range(3):
        fit = Polynomial.fit(px, py - base(px), degree, w=1/np.sqrt(px)).convert() + base
        residual = np.abs(fit(px) - py)
        keep = residual < max(.10, 2.5 * float(np.median(residual)))
        if keep.all():
            break
        px, py = px[keep], py[keep]
        if len(px) < 6 or np.ptp(px) < minimum_span:
            return None, {'reason': 'paint_not_consistent'}
        degree = 2 if np.ptp(px) >= 6.0 and len(px) >= 10 else 1
        base = shape if degree == 1 and prior is not None else Polynomial([0.])
    fit = Polynomial.fit(px, py - base(px), degree, w=1/np.sqrt(px)).convert() + base
    residual = float(np.mean(np.abs(fit(px) - py)))
    check = np.linspace(4., min(18., px.max()), 12)
    if (not np.isfinite(fit.coef).all() or residual > .12
            or abs(float(fit(0))) > 4.5
            or np.max(np.abs(fit.deriv()(check))) > 1.2
            or np.max(np.abs(fit.deriv(2)(check))) > .12):
        return None, {'reason': 'paint_fit_rejected'}
    if prior is not None:
        near = np.array([4., 8., min(12., px.max())])
        if np.max(np.abs(fit(near) - prior(near))) > .65:
            return None, {'reason': 'paint_track_mismatch'}
    return fit, {'reason': 'ok', 'source': 'paint_tracking',
                 'support_min_m': float(px.min()), 'support_max_m': float(px.max()),
                 'mean_residual_m': residual, 'bins': len(px)}

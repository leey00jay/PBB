"""인지: fit usable lane boundaries from the existing detector's masks."""
import numpy as np
from ..interfaces import LaneObservation


class LanePerception:
    def __init__(self, detector):
        self.detector = detector

    def fit_boundary(self, probabilities):
        """Reject tiny/noisy detections and fit only the nearby visible road."""
        cg = self.detector
        weights = probabilities[cg.cut_v:, :].ravel()
        x, y = cg.grid.T
        selected = ((weights > 0.5) & (x >= 4.0) & (x <= 25.0)
                    & (np.abs(y) < 8.0))
        if selected.sum() < 30:
            return None
        x, y, weights = x[selected], y[selected], weights[selected]
        # One point per half-meter avoids over-weighting thick/nearby markings.
        bins = np.floor(x * 2).astype(int)
        points = []
        for key in np.unique(bins):
            use = bins == key
            points.append((np.average(x[use], weights=weights[use]),
                           np.average(y[use], weights=weights[use]),
                           float(weights[use].mean())))
        points = np.asarray(points)
        if len(points) < 6 or np.ptp(points[:, 0]) < 3.0 or points[:, 0].min() > 8:
            return None
        x, y, weights = points.T
        degree = 2 if np.ptp(x) >= 8.0 else 1
        for _ in range(3):
            fit = np.polynomial.Polynomial.fit(x, y, degree, w=weights).convert()
            residual = np.abs(fit(x) - y)
            keep = residual < max(0.2, 2.5 * float(np.median(residual)))
            if keep.all():
                break
            if keep.sum() < 6 or np.ptp(x[keep]) < 3.0:
                return None
            x, y, weights = x[keep], y[keep], weights[keep]
        fit = np.polynomial.Polynomial.fit(x, y, degree, w=weights).convert()
        check = np.array([5.0, 10.0, 15.0])
        if (not np.isfinite(fit.coef).all()
                or np.max(np.abs(fit(check))) > 6.0
                or np.max(np.abs(fit.deriv()(check))) > 0.7
                or np.mean(np.abs(fit(x) - y)) > 0.25):
            return None
        return fit

    def observe(self, rgb):
        _, left_mask, right_mask = self.detector.detect(rgb)
        overlay = rgb.copy()
        overlay[left_mask > 0.5] = [0, 0, 255]
        overlay[right_mask > 0.5] = [255, 0, 0]
        left = self.fit_boundary(left_mask)
        right = self.fit_boundary(right_mask)
        return LaneObservation(left, right, overlay)

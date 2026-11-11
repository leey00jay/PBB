"""차선 인식 및 인식 실패 원인 기록."""
import os
import numpy as np
from pbb_common.models import LaneObservation


class LanePerception:
    def __init__(self, detector):
        self.detector = detector
        self.last_diagnostics = {}
        self._debug_frames = 0
        self._debug_previous = None

    def fit_boundary(self, probabilities):
        cg = self.detector
        self._fit_info = {"reason": "unknown"}

        weights = probabilities[cg.cut_v:, :].ravel()
        x, y = cg.grid.T

        selected = (
            (weights > 0.5)
            & (x >= 4.0)
            & (x <= 25.0)
            & (np.abs(y) < 8.0)
        )

        self._fit_info.update(
            selected_pixels=int(selected.sum()),
            above_threshold_pixels=int((weights > 0.5).sum()),
        )

        if selected.sum() < 30:
            self._fit_info["reason"] = "too_few_nearby_pixels"
            return None

        x, y, weights = x[selected], y[selected], weights[selected]

        # 가까운 차선이나 두꺼운 차선에 점이 몰리지 않도록 묶기
        bins = np.floor(x * 2).astype(int)
        points = []

        for key in np.unique(bins):
            use = bins == key
            points.append((
                np.average(x[use], weights=weights[use]),
                np.average(y[use], weights=weights[use]),
                float(weights[use].mean()),
            ))

        points = np.asarray(points)

        self._fit_info.update(
            bins=int(len(points)),
            x_min_m=float(points[:, 0].min()),
            x_max_m=float(points[:, 0].max()),
        )

        if len(points) < 6:
            self._fit_info["reason"] = "too_few_distance_bins"
            return None

        if np.ptp(points[:, 0]) < 3.0:
            self._fit_info["reason"] = "visible_span_too_short"
            return None

        if points[:, 0].min() > 8:
            self._fit_info["reason"] = "no_nearby_evidence"
            return None

        x, y, weights = points.T
        degree = 2 if np.ptp(x) >= 8.0 else 1

        for _ in range(3):
            fit = np.polynomial.Polynomial.fit(
                x, y, degree, w=weights
            ).convert()

            residual = np.abs(fit(x) - y)
            keep = residual < max(
                0.2, 2.5 * float(np.median(residual))
            )

            if keep.all():
                break

            if keep.sum() < 6 or np.ptp(x[keep]) < 3.0:
                self._fit_info["reason"] = (
                    "too_little_evidence_after_outlier_filter"
                )
                return None

            x, y, weights = x[keep], y[keep], weights[keep]

        fit = np.polynomial.Polynomial.fit(
            x, y, degree, w=weights
        ).convert()

        check = np.array([5.0, 10.0, 15.0])

        if not np.isfinite(fit.coef).all():
            self._fit_info["reason"] = "nonfinite_fit"
            return None

        self._fit_info.update(
            max_abs_y_m=float(np.max(np.abs(fit(check)))),
            max_abs_slope=float(
                np.max(np.abs(fit.deriv()(check)))
            ),
            mean_residual_m=float(
                np.mean(np.abs(fit(x) - y))
            ),
        )

        if self._fit_info["max_abs_y_m"] > 6.0:
            self._fit_info["reason"] = "boundary_too_far_sideways"
            return None

        if self._fit_info["max_abs_slope"] > 0.7:
            self._fit_info["reason"] = "slope_too_large"
            return None

        if self._fit_info["mean_residual_m"] > 0.25:
            self._fit_info["reason"] = "fit_residual_too_large"
            return None

        self._fit_info["reason"] = "ok"
        return fit

    def observe(self, rgb):
        _, left_mask, right_mask = self.detector.detect(rgb)

        overlay = rgb.copy()
        overlay[left_mask > 0.5] = [0, 0, 255]
        overlay[right_mask > 0.5] = [255, 0, 0]

        left = self.fit_boundary(left_mask)
        left_info = dict(self._fit_info)

        right = self.fit_boundary(right_mask)
        right_info = dict(self._fit_info)

        self.last_diagnostics = {
            "left": left_info,
            "right": right_info,
        }

        self._debug_frames += 1
        state = (left_info["reason"], right_info["reason"])

        if os.environ.get("PBB_PERCEPTION_DEBUG") == "1":
            if (
                state != self._debug_previous
                or self._debug_frames % 30 == 0
            ):
                print(
                    f"[perception frame={self._debug_frames}] "
                    f"left={left_info} right={right_info}",
                    flush=True,
                )

        self._debug_previous = state
        return LaneObservation(left, right, overlay)
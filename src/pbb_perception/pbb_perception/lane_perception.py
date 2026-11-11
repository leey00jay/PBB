"""AI 차선 검출과 현재 영상의 점선을 이용한 보완 추적."""
import os
import numpy as np
from pbb_common.models import LaneObservation


class LanePerception:
    def __init__(self, detector):
        self.detector = detector
        self.last_diagnostics = {}
        self._debug_frames = 0
        self._debug_previous = None

        self._tracks = {"left": None, "right": None}
        self._ages = {"left": 0, "right": 0}
        self._anchors = {"left": None, "right": None}

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

        # 距離ごとの集約ではなく、道路上の0.5mごとに点をまとめる
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

    def _recover(self, white, side):
        prior = self._tracks[side]

        if prior is None or self._ages[side] > 15:
            return None, {"reason": "tracking_expired"}

        x, y = self.detector.grid.T

        selected = (
            white[self.detector.cut_v:].ravel()
            & (x >= 4)
            & (x <= 16)
            & (np.abs(y - prior(x)) < 0.45)
        )

        if selected.sum() < 60:
            return None, {"reason": "too_few_paint_pixels"}

        x, y = x[selected], y[selected]
        bins = np.floor(x * 2).astype(int)

        points = np.array([
            (
                np.median(x[bins == b]),
                np.median(y[bins == b]),
            )
            for b in np.unique(bins)
        ])

        x, y = points.T

        if len(x) < 4 or np.ptp(x) < 1.5 or x.min() > 12:
            return None, {"reason": "paint_support_too_short"}

        # 현재 영상의 점선으로 직전 차선을 작게 보정
        residual = y - prior(x)
        offset = float(np.median(residual))
        keep = np.abs(residual - offset) < 0.12

        if keep.sum() < 4 or np.ptp(x[keep]) < 1.5:
            return None, {"reason": "paint_not_consistent"}

        x, y = x[keep], y[keep]

        correction = np.polynomial.Polynomial.fit(
            x, y - prior(x), 1
        ).convert()

        slope = float(
            np.clip(correction.coef[1], -0.05, 0.05)
        )
        reference = float(np.median(x))
        shift = float(np.median(y - prior(x)))

        candidate = prior + np.polynomial.Polynomial([
            shift - slope * reference,
            slope,
        ])

        checks = np.array([5.0, 8.0, 12.0])
        anchor = self._anchors[side]

        if (
            not np.isfinite(candidate.coef).all()
            or np.max(
                np.abs(candidate(checks) - prior(checks))
            ) > 0.4
            or np.max(
                np.abs(candidate(checks) - anchor(checks))
            ) > 0.7
            or np.mean(
                np.abs(candidate(x) - y)
            ) > 0.08
        ):
            return None, {"reason": "paint_tracking_rejected"}

        return candidate, {
            "reason": "ok",
            "source": "paint_tracking",
            "support_min_m": float(x.min()),
            "support_max_m": float(x.max()),
            "frames_since_model": self._ages[side],
        }

    def observe(self, rgb):
        import cv2

        _, left_mask, right_mask = self.detector.detect(rgb)

        masks = {
            "left": left_mask,
            "right": right_mask,
        }
        fits = {}
        infos = {}

        for side in ("left", "right"):
            fits[side] = self.fit_boundary(masks[side])
            infos[side] = dict(
                self._fit_info,
                source="model",
            )

            if fits[side] is not None:
                self._ages[side] = 0
                self._anchors[side] = fits[side]
            else:
                self._ages[side] += 1

        white = None

        needs_recovery = any(
            fits[side] is None
            and self._tracks[side] is not None
            and self._ages[side] <= 15
            for side in fits
        )

        if needs_recovery:
            gray = cv2.cvtColor(
                rgb, cv2.COLOR_RGB2GRAY
            )
            saturation = cv2.cvtColor(
                rgb, cv2.COLOR_RGB2HSV
            )[:, :, 1]

            top = cv2.morphologyEx(
                gray,
                cv2.MORPH_TOPHAT,
                cv2.getStructuringElement(
                    cv2.MORPH_RECT, (19, 3)
                ),
            )

            white = (
                (top > 20)
                & (gray > 110)
                & (saturation < 90)
            )

            for side in fits:
                if fits[side] is None:
                    recovered, info = self._recover(
                        white, side
                    )
                    infos[side]["recovery_reason"] = (
                        info["reason"]
                    )

                    if recovered is not None:
                        fits[side] = recovered
                        infos[side] = info

        # 반대쪽 차선과 폭이 맞지 않는 보완 결과는 제외
        if (
            fits["left"] is not None
            and fits["right"] is not None
        ):
            check = np.array([5.0, 8.0, 12.0])
            widths = (
                fits["left"](check)
                - fits["right"](check)
            )

            valid_width = (
                np.all(
                    (widths >= 2.5)
                    & (widths <= 4.5)
                )
                and np.ptp(widths) < 0.8
            )

            if not valid_width:
                for side in fits:
                    if (
                        infos[side]["source"]
                        == "paint_tracking"
                    ):
                        fits[side] = None
                        infos[side]["reason"] = (
                            "recovery_width_invalid"
                        )

        for side in fits:
            if fits[side] is not None:
                self._tracks[side] = fits[side]

            elif self._ages[side] > 15:
                self._tracks[side] = None
                self._anchors[side] = None

        overlay = rgb.copy()
        overlay[left_mask > 0.5] = [0, 0, 255]
        overlay[right_mask > 0.5] = [255, 0, 0]

        if white is not None:
            x, y = self.detector.grid.T

            for side in fits:
                if (
                    fits[side] is not None
                    and infos[side]["source"]
                    == "paint_tracking"
                ):
                    used = (
                        white[self.detector.cut_v:].ravel()
                        & (
                            np.abs(y - fits[side](x))
                            < 0.15
                        )
                    )

                    region = overlay[self.detector.cut_v:]
                    region[
                        used.reshape(region.shape[:2])
                    ] = [0, 255, 0]

        self.last_diagnostics = infos
        self._debug_frames += 1

        state = tuple(
            (
                infos[side]["reason"],
                infos[side]["source"],
            )
            for side in fits
        )

        if os.environ.get("PBB_PERCEPTION_DEBUG") == "1":
            if (
                state != self._debug_previous
                or self._debug_frames % 30 == 0
            ):
                print(
                    f"[perception frame={self._debug_frames}] "
                    f"{infos}",
                    flush=True,
                )

        self._debug_previous = state

        return LaneObservation(
            fits["left"],
            fits["right"],
            overlay,
        )
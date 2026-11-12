"""AI 차선 검출과 현재 영상의 점선을 이용한 보완 추적."""
import os
import numpy as np
from pbb_common.models import LaneObservation
from pbb_common.motion import move_curve
from .paint_tracking import paint_mask, fit_paint
from .visibility import straight_extent
from .centerline import double_yellow


class LanePerception:
    def __init__(self, detector, long_straight_visibility=False):
        self.detector = detector
        self.long_straight_visibility = long_straight_visibility
        self.last_diagnostics = {}
        self._debug_frames = 0
        self._debug_previous = None

        self._tracks = {"left": None, "right": None}
        self._ages = {"left": 0, "right": 0}
        self._anchors = {"left": None, "right": None}
        self._previous_pose = None
        self._lane_width = 3.5
        self._extra_paint_points = None
        self._side_grids = {}
        for name, yaw, left_offset in (('left', -45., .6), ('right', 45., -.6)):
            c, s = np.cos(np.deg2rad(yaw)), np.sin(np.deg2rad(yaw))
            x, y = detector.grid.T
            # CARLA camera yaw is positive right; road y is positive LEFT.
            self._side_grids[name] = np.column_stack((c*x+s*y, -s*x+c*y+left_offset))

    def fit_boundary(self, probabilities, prior=None):
        cg = self.detector
        self._fit_info = {"reason": "unknown"}

        weights = probabilities[cg.cut_v:, :].ravel()
        x, y = cg.grid.T

        threshold = .35 if prior is not None else .5
        minimum_bins = 4 if prior is not None else 6
        minimum_span = 1.5 if prior is not None else 3.0
        selected = (
            (weights > threshold)
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

        if len(points) < minimum_bins:
            self._fit_info["reason"] = "too_few_distance_bins"
            return None

        if np.ptp(points[:, 0]) < minimum_span:
            self._fit_info["reason"] = "visible_span_too_short"
            return None

        distant = points[:, 0].min() > 8.0
        if distant and (prior is None or points[:, 0].min() > 14.0
                        or np.ptp(points[:, 0]) < 8.0):
            self._fit_info["reason"] = "no_nearby_evidence"
            return None

        x, y, weights = points.T
        # Short visible dashes in bends used to be forced into a straight line
        # (span < 8 m), discarding the curvature needed through the next gap.
        degree = 2 if np.ptp(x) >= 5.0 else 1
        shape = (prior if degree == 1 and prior is not None
                 else np.polynomial.Polynomial([0.0]))

        for _ in range(3):
            fit = np.polynomial.Polynomial.fit(
                x, y - shape(x), degree, w=weights
            ).convert() + shape

            residual = np.abs(fit(x) - y)
            keep = residual < max(
                0.2, 2.5 * float(np.median(residual))
            )

            if keep.all():
                break

            if keep.sum() < minimum_bins or np.ptp(x[keep]) < minimum_span:
                self._fit_info["reason"] = (
                    "too_little_evidence_after_outlier_filter"
                )
                return None

            x, y, weights = x[keep], y[keep], weights[keep]
            degree = 2 if np.ptp(x) >= 5.0 else 1
            shape = (prior if degree == 1 and prior is not None
                     else np.polynomial.Polynomial([0.0]))

        fit = np.polynomial.Polynomial.fit(
            x, y - shape(x), degree, w=weights
        ).convert() + shape

        # A tight bend may legitimately have a steep tangent farther away.
        # Judge slope where paint is actually observed, not at a fixed 15 m
        # extrapolation of a 2 m dashed fragment.
        check = np.linspace(max(4., x.min()), min(18., x.max()), 12)

        if not np.isfinite(fit.coef).all():
            self._fit_info["reason"] = "nonfinite_fit"
            return None

        self._fit_info.update(
            bins=int(len(x)), x_min_m=float(x.min()), x_max_m=float(x.max()),
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

        if self._fit_info["max_abs_slope"] > 1.2:
            self._fit_info["reason"] = "slope_too_large"
            return None

        if self._fit_info["mean_residual_m"] > 0.25:
            self._fit_info["reason"] = "fit_residual_too_large"
            return None

        # A dashed line can start beyond 8 m. Accept that real distant paint
        # only if its near-field extrapolation agrees with the moved track.
        if distant:
            nearby = np.array([4.0, 6.0, 8.0])
            if (np.max(np.abs(fit(nearby) - prior(nearby))) > 0.6
                    or np.max(np.abs(fit.deriv()(nearby) - prior.deriv()(nearby))) > 0.15):
                self._fit_info["reason"] = "distant_track_mismatch"
                return None
            self._fit_info["distant_support"] = True

        self._fit_info["reason"] = "ok"
        return fit

    def _recover(self, paint, side, prior=None, fresh=False):
        track = self._tracks[side] if prior is None else prior
        result = fit_paint(paint, self.detector.grid, self.detector.cut_v,
                           side, track, self._extra_paint_points)
        if result[0] is None and track is not None and fresh:
            # A failed old corridor must not prevent fresh side-camera paint
            # from reacquiring the lane after a blank intersection.
            result = fit_paint(paint, self.detector.grid, self.detector.cut_v,
                               side, None, self._extra_paint_points)
        if result[0] is None and self._extra_paint_points is not None and fresh:
            result = fit_paint(np.zeros_like(paint), self.detector.grid, self.detector.cut_v,
                               side, None, self._extra_paint_points, near_side=True)
        return result

    def observe(self, rgb, vehicle_pose=None, side_images=None, paint_rgb=None):
        import cv2

        # Brightened neural input can amplify wet-road reflections. Validate
        # physical paint in the original exposure, at identical pixel positions.
        paint_source = rgb if paint_rgb is None else paint_rgb
        if paint_source.shape != rgb.shape or paint_source.dtype != np.uint8:
            raise ValueError('Paint verification image must match the RGB input')

        if vehicle_pose is not None:
            if self._previous_pose is not None:
                for tracks in (self._tracks, self._anchors):
                    for side, curve in tracks.items():
                        if curve is not None:
                            tracks[side] = move_curve(curve, self._previous_pose, vehicle_pose)
            self._previous_pose = tuple(vehicle_pose)
        else:
            self._previous_pose = None

        _, left_mask, right_mask = self.detector.detect(rgb)

        masks = {
            "left": left_mask,
            "right": right_mask,
        }
        fits = {}
        infos = {}
        medians = []
        median, extent = double_yellow(paint_source, self.detector.grid, self.detector.cut_v)
        if median is not None:
            medians.append((median, extent))
        right_double_yellow = False
        self._extra_paint_points = None
        if side_images:
            evidence = []
            for side, image in side_images.items():
                if side not in self._side_grids or image.shape != rgb.shape:
                    raise ValueError('Side cameras must match the center image calibration')
                mask = paint_mask(image)[self.detector.cut_v:].ravel()
                evidence.append(self._side_grids[side][mask])
                median, extent = double_yellow(image, self._side_grids[side], self.detector.cut_v, side_view=True)
                if median is not None:
                    medians.append((median, extent))
                    if side == 'right':
                        right_double_yellow = True
            if evidence:
                self._extra_paint_points = np.concatenate(evidence)
        median, median_extent = (min(medians, key=lambda p: abs(float(p[0](4))))
                                 if medians else (None, 0.))
        if median is not None and float(median(0)) < 0:
            right_double_yellow = True

        for side in ("left", "right"):
            prior = self._tracks[side] if self._ages[side] < 15 else None
            fits[side] = self.fit_boundary(masks[side], prior=prior)
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

        # Current paint can reacquire a missing model output and extend short
        # model fragments into a curve. It is not limited to shifting an old line.
        needs_recovery = any(fits[side] is None or infos[side].get("x_max_m", 0) < 18
                             for side in fits)
        if needs_recovery:
            white = paint_mask(paint_source)
            model_fits = dict(fits)
            for side in fits:
                recovered, info = self._recover(white, side)
                opposite = 'right' if side == 'left' else 'left'
                reference = model_fits[opposite]
                if reference is not None:
                    reference = reference + (self._lane_width if side == 'left' else -self._lane_width)
                    near = np.array([5., 8.])
                    if recovered is None or np.max(np.abs(recovered(near)-reference(near))) > .45:
                        paired, paired_info = self._recover(white, side, prior=reference)
                        if paired is not None:
                            recovered, info = paired, paired_info
                infos[side]["recovery_reason"] = info["reason"]
                if recovered is None:
                    continue
                current = fits[side]
                agrees = (current is None or np.max(np.abs(
                    recovered(np.array([5., 8.])) - current(np.array([5., 8.])))) < .35)
                extends = info["support_max_m"] > infos[side].get("x_max_m", 0) + 3
                if current is None or (agrees and extends):
                    fits[side], infos[side] = recovered, info

            if all(curve is None for curve in fits.values()):
                for side in fits:
                    recovered, info = self._recover(white, side, fresh=True)
                    if recovered is not None:
                        fits[side], infos[side] = recovered, info

        # 반대쪽 차선과 폭이 맞지 않는 보완 결과는 제외
        if (
            fits["left"] is not None
            and fits["right"] is not None
        ):
            extent = min(infos[side].get('support_max_m', infos[side].get('x_max_m', 12.))
                         for side in fits)
            check = np.linspace(5., max(6., min(12., extent)), 3)
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
            if valid_width:
                self._lane_width = .95*self._lane_width + .05*float(np.median(widths))

            both_paint = all(infos[side]['source'] == 'paint_tracking' for side in fits)
            if not valid_width and both_paint:
                # Keep independent observations for the planner's temporal
                # adjudication; do not turn one bad pairing into total loss.
                for side in fits:
                    infos[side]['pair_width_valid'] = False
            elif not valid_width:
                for side in fits:
                    if (
                        infos[side]["source"]
                        == "paint_tracking"
                    ):
                        fits[side] = None
                        infos[side]["reason"] = (
                            "recovery_width_invalid"
                        )

        # Double yellow is the left limit of our direction of travel. Exclude
        # inferred lanes beyond it, and use its own-side stripe to reacquire
        # only when it is the nearby left boundary (not several lanes away).
        if median is not None and not right_double_yellow:
            check = np.array([4., min(8., median_extent)])
            for side, curve in fits.items():
                if curve is not None and np.any(curve(check) > median(check)+.2):
                    fits[side] = None
                    infos[side]['reason'] = 'beyond_yellow_centerline'
            if fits['left'] is None and .7 <= float(median(0)) <= 3.0:
                fits['left'] = median - .15
                infos['left'] = dict(reason='ok', source='double_yellow', support_max_m=median_extent)

        for side in fits:
            if fits[side] is not None:
                self._tracks[side] = fits[side]
                self._ages[side] = 0
                self._anchors[side] = fits[side]

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

        if self.long_straight_visibility:
            paint = white if white is not None else paint_mask(paint_source)
            for side, curve in fits.items():
                if curve is not None:
                    near_extent = infos[side].get('support_max_m', infos[side].get('x_max_m', 0.))
                    infos[side]['lookahead_max_m'] = straight_extent(
                        paint, self.detector.grid, self.detector.cut_v, curve, near_extent)

        self.last_diagnostics = infos
        self.last_diagnostics['centerline'] = dict(right_double_yellow=right_double_yellow,
                                                   extent_m=median_extent)
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
            left_extent_m=(float(infos["left"].get("lookahead_max_m", infos["left"].get("support_max_m", infos["left"].get("x_max_m", 0))))
                           if fits["left"] is not None else 0.0),
            right_extent_m=(float(infos["right"].get("lookahead_max_m", infos["right"].get("support_max_m", infos["right"].get("x_max_m", 0))))
                            if fits["right"] is not None else 0.0),
            centerline=median, centerline_extent_m=median_extent,
            right_double_yellow=right_double_yellow,
        )

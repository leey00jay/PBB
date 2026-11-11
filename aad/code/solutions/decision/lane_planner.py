"""판단: 차선 중심 경로, 목표 속도, 주행 상태 결정."""

import numpy as np
from ..interfaces import DrivePlan


class LanePlanner:
    def __init__(self):
        self.lane_width = 3.5
        self.previous_center = None
        self.previous_speed = 0.0
        self.valid_frames = 0
        self.frames = 0

        # 주행 속도 설정
        self.cruise_speed = 80.0
        self.single_lane_speed = 40.0

        # 주행 안정성 설정
        self.max_lateral_accel = 2.0
        self.max_accel = 1.5
        self.max_decel = 3.0
        self.max_path_jump = 0.8

        # 경로 안정화 비율
        self.smoothing = 0.5

    def calculate_curve_speed(self, center):
        """전방 곡률과 감속 거리를 고려한 속도 계산."""

        x = np.linspace(3.0, 25.0, 45)

        dy = center.deriv(1)(x)
        ddy = center.deriv(2)(x)

        curvature = np.abs(ddy) / (1.0 + dy ** 2) ** 1.5

        if not np.all(np.isfinite(curvature)):
            return 0.0

        curvature = np.maximum(curvature, 1e-6)

        # 각 지점에서 허용되는 속도
        curve_speed = np.sqrt(
            self.max_lateral_accel / curvature
        )

        # 커브에 도달하기 전 감속 가능한 거리 반영
        distance = np.maximum(x - 2.0, 0.0)

        allowed_speed = np.sqrt(
            curve_speed ** 2
            + 2.0 * self.max_decel * distance
        )

        return float(
            np.clip(
                np.min(allowed_speed) * 3.6,
                0.0,
                self.cruise_speed
            )
        )

    def limit_speed_change(self, target_kmh, dt):
        """목표 속도의 급격한 변화 제한."""

        max_increase = self.max_accel * dt * 3.6
        max_decrease = self.max_decel * dt * 3.6

        target_kmh = np.clip(
            target_kmh,
            self.previous_speed - max_decrease,
            self.previous_speed + max_increase
        )

        self.previous_speed = float(target_kmh)

        return float(target_kmh)

    def plan(self, observation, desired_speed_kmh, dt=0.05):

        self.frames += 1

        left = observation.left
        right = observation.right

        center = None
        traj = None

        status = "LANE LOST - STOP"
        target_kmh = 0.0

        drive_allowed = False
        stop_brake = 0.7

        # 외부 목표 속도 적용
        desired_speed_kmh = float(
            np.clip(
                desired_speed_kmh,
                0.0,
                self.cruise_speed
            )
        )

        # 양쪽 차선 인식
        if left is not None and right is not None:

            sample_x = np.array([5.0, 10.0, 15.0])
            widths = left(sample_x) - right(sample_x)

            if (
                np.all(np.isfinite(widths))
                and np.all((widths >= 2.5) & (widths <= 4.5))
                and np.ptp(widths) < 0.8
            ):
                self.lane_width = (
                    0.95 * self.lane_width
                    + 0.05 * float(np.median(widths))
                )

                center = (left + right) * 0.5

                status = "BOTH LANES"
                target_kmh = desired_speed_kmh

            else:
                status = "LANE WIDTH INVALID - STOP"

        # 오른쪽 차선만 인식
        elif right is not None:

            center = right + self.lane_width / 2.0

            status = "RIGHT ONLY"

            target_kmh = min(
                desired_speed_kmh,
                self.single_lane_speed
            )

        # 왼쪽 차선만 인식
        elif left is not None:

            center = left - self.lane_width / 2.0

            status = "LEFT ONLY"

            target_kmh = min(
                desired_speed_kmh,
                self.single_lane_speed
            )

        # 경로 유효성 검사
        if center is not None:

            check_x = np.array([0.0, 6.0, 15.0, 25.0])
            check_y = center(check_x)

            if not np.all(np.isfinite(check_y)):

                status = "INVALID PATH - STOP"
                center = None

            elif abs(float(center(0.0))) > 2.0:

                status = "PATH TOO FAR - STOP"
                center = None

            elif self.previous_center is not None:

                path_difference = abs(
                    float(
                        center(6.0)
                        - self.previous_center(6.0)
                    )
                )

                if path_difference > self.max_path_jump:

                    status = "PATH JUMP - STOP"
                    center = None

        # 경로 안정화 및 속도 결정
        if center is not None:

            if self.previous_center is not None:

                center = (
                    (1.0 - self.smoothing) * center
                    + self.smoothing * self.previous_center
                )

            self.previous_center = center

            # 커브 속도 제한
            curve_speed = self.calculate_curve_speed(center)

            # DEBUG: 속도 제한 원인 확인
            print(
                f"[SPEED DEBUG] "
                f"desired={desired_speed_kmh:.1f} | "
                f"curve={curve_speed:.1f} | "
                f"before={target_kmh:.1f} | "
                f"lane={status}"
            )

            target_kmh = min(
                target_kmh,
                curve_speed
            )

            # 주행 경로 생성
            x = np.arange(-2.0, 31.0)

            traj = np.column_stack(
                (x + 0.5, -center(x))
            )

            self.valid_frames += 1

        else:
            self.valid_frames = 0
            self.previous_center = None

            # DEBUG: 경로 인식 실패
            print(
                f"[SPEED DEBUG] "
                f"PATH INVALID | "
                f"status={status}"
            )

        # 주행 허용 판단
        if self.frames <= 30:

            status = "WARMUP - STOP"
            stop_brake = 1.0

        elif traj is not None and self.valid_frames >= 5:

            drive_allowed = True

        elif traj is not None:

            status = "CHECKING LANE - STOP"

        # 주행 불가 시 즉시 정지 명령
        if not drive_allowed:

            target_kmh = 0.0
            self.previous_speed = 0.0

        else:

            target_kmh = self.limit_speed_change(
                target_kmh,
                dt
            )

        # 판단 결과 출력
        print(
            f"[DECISION] "
            f"target={target_kmh:.1f} km/h | "
            f"drive={drive_allowed} | "
            f"status={status}"
        )

        return DrivePlan(
            trajectory=traj,
            target_kmh=float(target_kmh),
            status=status,
            drive_allowed=drive_allowed,
            stop_brake=stop_brake,
            lane_width_m=float(self.lane_width),
            left_valid=left is not None,
            right_valid=right is not None
        )
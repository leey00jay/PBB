"""Entry point for the official CARLA 0.9.16 PID controller.

The unmodified implementation is in third_party/carla_agents/agents/navigation/
controller.py. This module imports it and supplies the tested PBB configuration.
"""

from agents.navigation.controller import (
    PIDLateralController,
    PIDLongitudinalController,
    VehiclePIDController,
)

__all__ = [
    "VehiclePIDController",
    "PIDLateralController",
    "PIDLongitudinalController",
    "create_controller",
]


def create_controller(vehicle):
    """Create the official controller with the tested 20 Hz low-speed tuning.

    run_step(target_speed_kmh, carla_waypoint) returns a VehicleControl.
    The caller applies it to the vehicle. Signal decisions and emergency stops
    belong to BehaviorAgent. Settings match the 20261109-214402 driving run;
    max_steering=0.8 was the official LocalPlanner default in that run.
    """
    return VehiclePIDController(
        vehicle,
        args_lateral={"K_P": 1.2, "K_I": 0, "K_D": 0.05, "dt": 0.05},
        args_longitudinal={"K_P": 0.35, "K_I": 0.03, "K_D": 0, "dt": 0.05},
        max_throttle=0.55,
        max_brake=0.6,
        max_steering=0.8,
    )

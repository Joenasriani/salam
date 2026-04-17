# RoboSim Studio Package

"""
RoboSim Studio - A high-end consumer robotics simulation platform.

This package provides:
- Robot kinematics and dynamics
- Physics simulation
- 2D/3D visualization
- Interactive learning tools
"""

__version__ = "1.0.0"
__author__ = "RoboSim Team"

from robosim_studio.core.kinematics import (
    RobotKinematics,
    DHParameter,
    create_planar_arm,
    create_scara_robot,
    create_puma_like_robot
)

from robosim_studio.simulation.physics_engine import (
    PhysicsEngine,
    SimulationScenario
)

from robosim_studio.visualization.renderer import (
    RobotVisualizer2D,
    PointCloudVisualizer,
    InteractiveUI
)

from robosim_studio.main import RoboSimStudio

__all__ = [
    'RobotKinematics',
    'DHParameter',
    'PhysicsEngine',
    'SimulationScenario',
    'RobotVisualizer2D',
    'PointCloudVisualizer',
    'InteractiveUI',
    'RoboSimStudio',
    'create_planar_arm',
    'create_scara_robot',
    'create_puma_like_robot'
]
# RoboSim Studio - Quick Start Guide

## Installation

```bash
# Install required dependencies
pip install numpy

# Optional: Install visualization and physics libraries
pip install pygame pybullet pclpy
```

## Usage Examples

### 1. Basic Robot Kinematics

```python
from robosim_studio.core.kinematics import create_planar_arm, create_puma_like_robot
import numpy as np

# Create a simple 3-link planar robot
robot = create_planar_arm(n_links=3, link_length=0.5)

# Forward kinematics
joint_angles = [np.pi/4, np.pi/6, -np.pi/8]
end_pose = robot.forward_kinematics(joint_angles)
print(f"End-effector position: {end_pose[:3, 3]}")

# Compute manipulability
manipulability = robot.get_manipulability(joint_angles)
print(f"Manipulability: {manipulability:.4f}")

# Sample workspace
workspace = robot.compute_workspace(n_samples=1000)
```

### 2. Physics Simulation

```python
from robosim_studio.simulation.physics_engine import PhysicsEngine, SimulationScenario

# Initialize physics engine
engine = PhysicsEngine(gui=True)  # Set gui=False for headless mode
engine.initialize()

# Create objects
engine.create_box([0, 0, 0.5], [0.2, 0.2, 0.2], mass=1.0, name="box")
engine.create_sphere([0.5, 0, 1.0], 0.1, mass=0.5, name="sphere")

# Run simulation
for i in range(1000):
    engine.step_simulation()

# Clean up
engine.disconnect()
```

### 3. Interactive Visualization

```python
from robosim_studio.main import RoboSimStudio

# Create studio instance
studio = RoboSimStudio()

# Launch interactive designer (requires display)
studio.launch_designer()

# Or run in learning mode
studio.learn_mode()

# Run simulation demo
studio.demonstrate_pick_and_place()
```

### 4. Custom Robot Design

```python
from robosim_studio.core.kinematics import RobotKinematics, DHParameter

# Create custom robot
robot = RobotKinematics(name="custom_arm")

# Add links with DH parameters
# DHParameter(theta, d, a, alpha)
robot.add_link(DHParameter(0, 0.1, 0.3, 0), joint_limit=(-np.pi, np.pi))
robot.add_link(DHParameter(0, 0, 0.25, 0), joint_limit=(-np.pi/2, np.pi/2))
robot.add_link(DHParameter(0, 0, 0.2, np.pi/2), joint_limit=(-np.pi, np.pi))

# Export to URDF
urdf_content = robot.to_urdf()
with open("my_robot.urdf", "w") as f:
    f.write(urdf_content)

# Save configuration
robot.save_to_file("my_robot.json")

# Load configuration later
loaded_robot = RobotKinematics.load_from_file("my_robot.json")
```

### 5. Point Cloud Processing

```python
from robosim_studio.visualization.renderer import PointCloudVisualizer
import numpy as np

# Create visualizer
pc_viz = PointCloudVisualizer()

# Create point cloud from numpy array
points = np.random.randn(1000, 3).astype(np.float32)
cloud = pc_viz.create_point_cloud(points)

# Apply filters
filtered = pc_viz.filter_passthrough(cloud, axis='z', min_val=-1, max_val=1)
downsampled = pc_viz.filter_voxel_grid(filtered, leaf_size=0.05)

# Compute normals
cloud_with_normals = pc_viz.compute_normals(downsampled, search_radius=0.03)

# Segment plane
plane_model, inliers, outliers = pc_viz.segment_plane(cloud, distance_threshold=0.01)
```

## Command Line Interface

```bash
# Launch designer mode
python -m robosim_studio.main --mode design

# Run in learning mode
python -m robosim_studio.main --mode learn

# Run simulation demo
python -m robosim_studio.main --demo

# Load custom robot
python -m robosim_studio.main --robot my_robot.json

# Run without GUI (headless)
python -m robosim_studio.main --no-gui
```

## Learning Mode Topics

When running in learning mode, you can study:

1. **Introduction to Robot Kinematics** - Basic concepts
2. **Denavit-Hartenberg Parameters** - Robot representation
3. **Forward Kinematics** - Computing end-effector pose
4. **Inverse Kinematics** - Finding joint angles
5. **Jacobian and Velocity Kinematics** - Motion analysis
6. **Manipulability Analysis** - Performance metrics
7. **Path Planning Basics** - Navigation algorithms
8. **Collision Detection** - Safety systems

## Daily Use Cases

### Morning Planning Session
- Load your robot configuration
- Test different task scenarios
- Verify reachability of workspaces
- Optimize robot placement

### Design Iteration
- Modify DH parameters
- Visualize changes in real-time
- Check manipulability at key poses
- Export updated URDF

### Learning & Education
- Study robotics concepts interactively
- Visualize abstract mathematical concepts
- Practice inverse kinematics problems
- Understand singularities

### Rapid Prototyping
- Test path planning algorithms
- Validate collision avoidance
- Simulate pick-and-place tasks
- Verify system integration

## Troubleshooting

### Pygame not available
The application will run in headless mode. Install pygame for visualization:
```bash
pip install pygame
```

### PyBullet not available
Physics simulation will run in simulation-only mode. Install for full features:
```bash
pip install pybullet
```

### pclpy not available
Point cloud processing will use fallback mode. Install for advanced features:
```bash
pip install pclpy
```

## Support

For issues and feature requests, please check the documentation or community forums.

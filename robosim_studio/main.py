"""
RoboSim Studio - Main Application

Unifies all components into a cohesive, user-friendly application
for everyday robotics simulation and learning.
"""

import numpy as np
from typing import Dict, Optional, List
import time
import os

# Import core modules
from core.kinematics import (
    RobotKinematics, DHParameter,
    create_planar_arm, create_scara_robot, create_puma_like_robot
)
from simulation.physics_engine import PhysicsEngine, SimulationScenario
from visualization.renderer import (
    RobotVisualizer2D, PointCloudVisualizer, InteractiveUI
)


class RoboSimStudio:
    """
    Main application class for RoboSim Studio.
    
    Provides a unified interface for:
    - Robot design and kinematics
    - Physics simulation
    - Visualization and interaction
    - Learning and education
    """
    
    def __init__(self, config: Optional[Dict] = None):
        """
        Initialize RoboSim Studio.
        
        Args:
            config: Optional configuration dictionary
        """
        self.config = config or {}
        self.robot = None
        self.physics_engine = None
        self.visualizer = None
        self.ui = None
        self.point_cloud_viz = None
        self.current_mode = "design"
        self.trajectory = []
        self.is_running = False
        
        # Initialize components
        self._initialize_components()
    
    def _initialize_components(self):
        """Initialize all system components."""
        print("Initializing RoboSim Studio...")
        
        # Initialize physics engine
        gui_enabled = self.config.get('show_physics_gui', False)
        self.physics_engine = PhysicsEngine(gui=gui_enabled)
        
        # Initialize 2D visualizer
        width = self.config.get('window_width', 1024)
        height = self.config.get('window_height', 768)
        self.visualizer = RobotVisualizer2D(width=width, height=height)
        
        # Initialize UI
        self.ui = InteractiveUI(self.visualizer)
        
        # Initialize point cloud visualizer
        self.point_cloud_viz = PointCloudVisualizer()
        
        # Create default robot
        self.load_default_robot()
        
        print("Initialization complete!")
    
    def load_default_robot(self):
        """Load a default 3-link planar robot."""
        self.robot = create_planar_arm(n_links=3, link_length=0.5)
        print(f"Loaded default robot: {self.robot.name}")
    
    def load_robot_from_file(self, filepath: str):
        """
        Load robot configuration from file.
        
        Args:
            filepath: Path to JSON configuration file
        """
        self.robot = RobotKinematics.load_from_file(filepath)
        print(f"Loaded robot from file: {self.robot.name}")
    
    def create_robot_from_dh(self, name: str, dh_params: List[Dict]):
        """
        Create robot from DH parameters.
        
        Args:
            name: Robot name
            dh_params: List of DH parameter dictionaries
        """
        self.robot = RobotKinematics(name=name)
        
        for dh_data in dh_params:
            dh = DHParameter(
                theta=dh_data.get('theta', 0),
                d=dh_data.get('d', 0),
                a=dh_data.get('a', 0),
                alpha=dh_data.get('alpha', 0)
            )
            joint_limit = dh_data.get('joint_limit', (-np.pi, np.pi))
            self.robot.add_link(dh, joint_limit=joint_limit)
        
        print(f"Created custom robot: {name}")
    
    def set_mode(self, mode: str):
        """
        Set application mode.
        
        Args:
            mode: Mode name ("design", "simulate", "learn", "planning")
        """
        self.current_mode = mode
        self.ui.set_mode(mode)
        print(f"Switched to {mode} mode")
        
        if mode == "simulate" and not self.physics_engine.is_initialized:
            self.physics_engine.initialize()
    
    def launch_designer(self, blocking: bool = True):
        """
        Launch the robot designer interface.
        
        Args:
            blocking: Whether to block until window is closed
        """
        self.set_mode("design")
        self._setup_designer_ui()
        self._run_visualization_loop(blocking)
    
    def _setup_designer_ui(self):
        """Set up UI elements for designer mode."""
        if not self.robot:
            return
        
        # Add sliders for each joint
        for i in range(len(self.robot.dh_params)):
            self.ui.add_slider(
                name=f"joint_{i}",
                min_val=-np.pi,
                max_val=np.pi,
                initial_val=0.0,
                position=(20, 500 + i * 40),
                callback=self._on_joint_slider_change
            )
        
        # Add buttons
        self.ui.add_button(
            "compute_ik", "Compute IK",
            position=(20, 650),
            callback=self._on_compute_ik
        )
        
        self.ui.add_button(
            "show_workspace", "Show Workspace",
            position=(140, 650),
            callback=self._on_show_workspace
        )
        
        self.ui.add_button(
            "export_urdf", "Export URDF",
            position=(280, 650),
            callback=self._on_export_urdf
        )
        
        self.ui.add_button(
            "simulate", "Simulate",
            position=(420, 650),
            callback=lambda x: self.set_mode("simulate")
        )
    
    def _on_joint_slider_change(self, name: str, value: float):
        """Handle joint slider changes."""
        pass  # Handled in visualization loop
    
    def _on_compute_ik(self, button_name: str):
        """Handle inverse kinematics computation."""
        print("Computing inverse kinematics...")
        # Would need target position from user input
        pass
    
    def _on_show_workspace(self, button_name: str):
        """Handle workspace visualization."""
        if self.robot:
            workspace = self.robot.compute_workspace(n_samples=500)
            self.workspace_points = workspace
            print(f"Computed workspace with {len(workspace)} points")
    
    def _on_export_urdf(self, button_name: str):
        """Handle URDF export."""
        if self.robot:
            urdf_content = self.robot.to_urdf()
            filepath = f"data/{self.robot.name}.urdf"
            os.makedirs("data", exist_ok=True)
            with open(filepath, 'w') as f:
                f.write(urdf_content)
            print(f"Exported URDF to {filepath}")
    
    def run_simulation(self, duration: float = 10.0):
        """
        Run physics simulation.
        
        Args:
            duration: Simulation duration in seconds
        """
        self.set_mode("simulate")
        
        if not self.physics_engine.is_initialized:
            self.physics_engine.initialize()
        
        # Load robot into simulation
        if self.robot:
            # Export temporary URDF
            urdf_content = self.robot.to_urdf()
            temp_urdf = "data/temp_robot.urdf"
            os.makedirs("data", exist_ok=True)
            with open(temp_urdf, 'w') as f:
                f.write(urdf_content)
            
            self.physics_engine.load_robot(temp_urdf, name="robot")
        
        # Run simulation loop
        start_time = time.time()
        while time.time() - start_time < duration:
            self.physics_engine.step_simulation()
            time.sleep(self.physics_engine.time_step)
        
        print(f"Simulation completed: {duration}s")
    
    def demonstrate_pick_and_place(self):
        """Demonstrate a pick-and-place scenario."""
        self.set_mode("simulate")
        
        if not self.physics_engine.is_initialized:
            self.physics_engine.initialize()
        
        # Set up scenario
        SimulationScenario.pick_and_place(self.physics_engine)
        
        print("Pick-and-place scenario initialized")
        print("Use the physics GUI to interact with the scene")
    
    def visualize_trajectory(self, waypoints: List[List[float]]):
        """
        Visualize a trajectory.
        
        Args:
            waypoints: List of joint configurations
        """
        self.trajectory = waypoints
        
        if self.visualizer.pygame and self.visualizer.running:
            # Convert to end-effector positions for visualization
            ee_positions = []
            for config in waypoints:
                pose = self.robot.forward_kinematics(config)
                ee_positions.append((pose[0, 3], pose[1, 3]))
            
            self.visualizer.draw_trajectory(ee_positions)
    
    def _run_visualization_loop(self, blocking: bool = True):
        """
        Run the main visualization loop.
        
        Args:
            blocking: Whether to block execution
        """
        if not self.visualizer.pygame:
            print("Pygame not available. Running in headless mode.")
            return
        
        self.visualizer.start(title="RoboSim Studio")
        self.is_running = True
        
        clock = self.visualizer.clock
        pygame = self.visualizer.pygame
        
        frame_count = 0
        start_time = time.time()
        
        try:
            while self.is_running:
                # Handle events
                events = self.visualizer.handle_events()
                
                if events.get('quit'):
                    self.is_running = False
                    break
                
                # Handle keyboard shortcuts
                if events.get('key_pressed'):
                    key = events['key_pressed']
                    if key == pygame.K_ESCAPE:
                        self.is_running = False
                        break
                    elif key == pygame.K_s:
                        self.set_mode("simulate")
                    elif key == pygame.K_d:
                        self.set_mode("design")
                    elif key == pygame.K_l:
                        self.set_mode("learn")
                    elif key == pygame.K_w:
                        # Toggle workspace
                        if hasattr(self, 'workspace_points'):
                            delattr(self, 'workspace_points')
                        else:
                            self.workspace_points = self.robot.compute_workspace(500)
                
                # Get mouse state for UI
                mouse_pos = pygame.mouse.get_pos()
                self.ui.handle_mouse_motion(mouse_pos)
                
                # Clear screen
                self.visualizer.clear()
                
                # Draw grid
                self.visualizer.draw_grid()
                
                # Draw robot based on current joint angles
                if self.robot:
                    # Get joint angles from UI sliders
                    joint_angles = []
                    for i in range(len(self.robot.dh_params)):
                        slider_name = f"joint_{i}"
                        if slider_name in self.ui.sliders:
                            joint_angles.append(self.ui.sliders[slider_name]['value'])
                        else:
                            joint_angles.append(0.0)
                    
                    # Get link lengths
                    link_lengths = [dh.a for dh in self.robot.dh_params]
                    
                    # Draw robot
                    self.visualizer.draw_robot_arm(joint_angles, link_lengths)
                    
                    # Calculate and display end-effector position
                    if len(joint_angles) == len(link_lengths):
                        ee_pose = self.robot.forward_kinematics(joint_angles)
                        ee_pos = (ee_pose[0, 3], ee_pose[1, 3])
                        
                        # Draw end effector position indicator
                        self.visualizer.draw_target(ee_pos, color=(255, 0, 0), size=5)
                        
                        # Calculate manipulability
                        manipulability = self.robot.get_manipulability(joint_angles)
                        
                        # Display info panel
                        info = {
                            "Mode": self.current_mode,
                            "End-Effector X": f"{ee_pos[0]:.3f} m",
                            "End-Effector Y": f"{ee_pos[1]:.3f} m",
                            "Manipulability": f"{manipulability:.4f}",
                            "Joints": len(self.robot.dh_params),
                            "FPS": f"{frame_count / max(0.001, time.time() - start_time):.1f}"
                        }
                        self.visualizer.draw_info_panel(info)
                
                # Draw workspace if computed
                if hasattr(self, 'workspace_points'):
                    self.visualizer.draw_workspace(self.workspace_points)
                
                # Draw UI
                self.ui.draw()
                
                # Update display
                self.visualizer.update()
                
                frame_count += 1
                
                if not blocking and frame_count > 10:
                    break
                    
        except Exception as e:
            print(f"Visualization error: {e}")
        finally:
            self.visualizer.stop()
            self.is_running = False
    
    def learn_mode(self):
        """Launch interactive learning mode."""
        self.set_mode("learn")
        
        print("\n" + "="*60)
        print("ROBOSIM STUDIO - LEARNING MODE")
        print("="*60)
        print("""
Welcome to RoboSim Studio Learning Mode!

Available Lessons:
1. Introduction to Robot Kinematics
2. Denavit-Hartenberg Parameters
3. Forward Kinematics
4. Inverse Kinematics
5. Jacobian and Velocity Kinematics
6. Manipulability Analysis
7. Path Planning Basics
8. Collision Detection

Select a lesson number (1-8) or press Q to quit:
        """)
        
        # Simple text-based learning interface
        lessons = {
            1: {
                "title": "Introduction to Robot Kinematics",
                "content": """
Robot kinematics is the study of motion in robots without considering forces.

Key Concepts:
- Configuration: The position and orientation of all robot links
- Degrees of Freedom (DOF): Number of independent joint variables
- Workspace: All positions reachable by the end-effector
- Redundancy: More DOF than required for a task

Try adjusting the joint sliders to see how the robot moves!
                """
            },
            2: {
                "title": "Denavit-Hartenberg Parameters",
                "content": """
DH parameters provide a systematic way to describe robot kinematics.

Four Parameters per Joint:
- θ (theta): Joint angle - rotation about z-axis
- d: Link offset - distance along z-axis
- a: Link length - distance along x-axis
- α (alpha): Link twist - rotation about x-axis

Each joint's transformation is built from these four values!
                """
            },
            3: {
                "title": "Forward Kinematics",
                "content": """
Forward Kinematics: Given joint angles, find end-effector pose.

Process:
1. Start from base frame
2. Apply transformation for each joint
3. Multiply all transformations together
4. Result is end-effector pose relative to base

Formula: T_base_ee = T_0_1 × T_1_2 × ... × T_n-1_n

Watch how the end-effector position updates as you move joints!
                """
            },
            4: {
                "title": "Inverse Kinematics",
                "content": """
Inverse Kinematics: Given desired end-effector pose, find joint angles.

Challenges:
- Multiple solutions may exist
- Some poses may be unreachable
- Singularities cause problems
- Computationally more complex than forward kinematics

Methods:
- Analytical (closed-form) - fast but limited
- Numerical (iterative) - general but slower

Click 'Compute IK' to try inverse kinematics!
                """
            },
            5: {
                "title": "Jacobian and Velocity Kinematics",
                "content": """
The Jacobian matrix relates joint velocities to end-effector velocities.

v = J × q_dot

Where:
- v: End-effector velocity (linear + angular)
- J: Jacobian matrix (6×n for n-joint robot)
- q_dot: Joint velocity vector

Uses:
- Resolved-rate motion control
- Singularity analysis
- Force/velocity transformation
                """
            },
            6: {
                "title": "Manipulability Analysis",
                "content": """
Manipulability measures how well a robot can move in different directions.

Yoshikawa's Measure: w = sqrt(det(J × J^T))

High manipulability:
- Robot can move easily in all directions
- Far from singularities
- Good force transmission

Low manipulability:
- Limited motion directions
- Near singularities
- Poor force transmission

Watch the manipulability value as you move the robot!
                """
            },
            7: {
                "title": "Path Planning Basics",
                "content": """
Path planning finds a collision-free path from start to goal.

Common Algorithms:
- A*: Grid-based optimal search
- RRT (Rapidly-exploring Random Tree): Sampling-based
- PRM (Probabilistic Roadmap): Multi-query planning
- Potential Fields: Artificial forces

Considerations:
- Obstacle avoidance
- Smoothness
- Optimality (shortest/fastest)
- Computational efficiency
                """
            },
            8: {
                "title": "Collision Detection",
                "content": """
Collision detection checks if robot intersects with obstacles.

Approaches:
- Bounding volumes (spheres, boxes)
- Separating axis theorem
- Distance fields
- Continuous collision detection

Applications:
- Safe path planning
- Real-time monitoring
- Virtual fixtures
- Human-robot collaboration

Red warnings indicate potential collisions!
                """
            }
        }
        
        while True:
            try:
                choice = input("\nEnter lesson number: ").strip()
                
                if choice.upper() == 'Q':
                    print("Exiting learning mode.")
                    break
                
                lesson_num = int(choice)
                if 1 <= lesson_num <= 8:
                    lesson = lessons[lesson_num]
                    print(f"\n{'='*60}")
                    print(f"Lesson {lesson_num}: {lesson['title']}")
                    print('='*60)
                    print(lesson['content'])
                    print('='*60)
                else:
                    print("Please enter a number between 1 and 8.")
                    
            except ValueError:
                print("Invalid input. Please enter a number or Q.")
    
    def save_project(self, filepath: str):
        """
        Save current project to file.
        
        Args:
            filepath: Output file path
        """
        if self.robot:
            self.robot.save_to_file(filepath)
            print(f"Project saved to {filepath}")
    
    def load_project(self, filepath: str):
        """
        Load project from file.
        
        Args:
            filepath: Input file path
        """
        self.load_robot_from_file(filepath)
    
    def close(self):
        """Clean up and close the application."""
        self.is_running = False
        
        if self.physics_engine:
            self.physics_engine.disconnect()
        
        if self.visualizer:
            self.visualizer.stop()
        
        print("RoboSim Studio closed.")


def main():
    """Main entry point for RoboSim Studio."""
    import argparse
    
    parser = argparse.ArgumentParser(description="RoboSim Studio - Robotics Simulation Platform")
    parser.add_argument('--mode', choices=['design', 'simulate', 'learn'], 
                       default='design', help='Start mode')
    parser.add_argument('--robot', type=str, help='Load robot from file')
    parser.add_argument('--no-gui', action='store_true', help='Run without GUI')
    parser.add_argument('--demo', action='store_true', help='Run demo simulation')
    
    args = parser.parse_args()
    
    # Initialize studio
    config = {
        'show_physics_gui': not args.no_gui,
        'window_width': 1200,
        'window_height': 800
    }
    
    studio = RoboSimStudio(config=config)
    
    # Load custom robot if specified
    if args.robot:
        studio.load_robot_from_file(args.robot)
    
    try:
        if args.demo:
            print("Running demo...")
            studio.demonstrate_pick_and_place()
        elif args.mode == 'learn':
            studio.learn_mode()
        elif args.mode == 'simulate':
            studio.run_simulation(duration=5.0)
        else:
            studio.launch_designer(blocking=True)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        studio.close()


if __name__ == "__main__":
    main()

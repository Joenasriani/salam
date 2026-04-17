"""
RoboSim Studio - Physics Simulation Engine

Integrates PyBullet for physics simulation, URDF loading, and 
real-time robot simulation with collision detection.
"""

import numpy as np
from typing import List, Dict, Optional, Tuple
import os


class PhysicsEngine:
    """
    Physics simulation engine using PyBullet.
    
    Provides real-time physics simulation for robots including:
    - URDF/SDF/MJCF model loading
    - Gravity and friction modeling
    - Contact detection and response
    - Joint control and sensing
    """
    
    def __init__(self, gui: bool = True):
        """
        Initialize physics engine.
        
        Args:
            gui: Whether to show graphical interface
        """
        self.gui = gui
        self.client_id = None
        self.robot_ids = {}
        self.object_ids = {}
        self.gravity = -9.81
        self.time_step = 1.0 / 240.0
        self.is_initialized = False
    
    def initialize(self):
        """Initialize PyBullet physics server."""
        try:
            import pybullet as p
            
            if self.gui:
                self.client_id = p.connect(p.GUI)
            else:
                self.client_id = p.connect(p.DIRECT)
            
            # Configure physics parameters
            p.setGravity(0, 0, self.gravity)
            p.setTimeStep(self.time_step)
            p.setPhysicsEngineParameter(fixedTimeStep=self.time_step)
            
            # Load plane
            plane_id = p.loadURDF("plane.urdf")
            self.object_ids['plane'] = plane_id
            
            self.is_initialized = True
            
        except ImportError:
            print("PyBullet not installed. Running in simulation-only mode.")
            self.is_initialized = False
        except Exception as e:
            print(f"Warning: Could not initialize PyBullet: {e}")
            self.is_initialized = False
    
    def load_robot(self, urdf_path: str, name: str = "robot",
                   base_position: Optional[List[float]] = None,
                   base_orientation: Optional[List[float]] = None) -> int:
        """
        Load a robot from URDF file.
        
        Args:
            urdf_path: Path to URDF file
            name: Name identifier for the robot
            base_position: [x, y, z] base position
            base_orientation: [roll, pitch, yaw] or quaternion
            
        Returns:
            Robot ID in simulation
        """
        if not self.is_initialized:
            print("Physics engine not initialized. Returning dummy ID.")
            robot_id = len(self.robot_ids)
            self.robot_ids[name] = robot_id
            return robot_id
        
        import pybullet as p
        
        if base_position is None:
            base_position = [0, 0, 0]
        
        if base_orientation is None:
            base_orientation = [0, 0, 0, 1]  # Quaternion
        
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=base_position,
            baseOrientation=base_orientation,
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION
        )
        
        self.robot_ids[name] = robot_id
        return robot_id
    
    def create_box(self, position: List[float], size: List[float],
                   mass: float = 1.0, color: Optional[List[float]] = None,
                   name: str = "box") -> int:
        """
        Create a box object in simulation.
        
        Args:
            position: [x, y, z] position
            size: [length, width, height]
            mass: Object mass (0 for static)
            color: [R, G, B, A] color
            name: Object identifier
            
        Returns:
            Object ID
        """
        if not self.is_initialized:
            obj_id = len(self.object_ids)
            self.object_ids[name] = obj_id
            return obj_id
        
        import pybullet as p
        
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[s/2 for s in size]
        )
        
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[s/2 for s in size]
        )
        
        obj_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        
        if color:
            p.changeVisualShape(obj_id, -1, rgbaColor=color)
        
        self.object_ids[name] = obj_id
        return obj_id
    
    def create_sphere(self, position: List[float], radius: float,
                      mass: float = 1.0, color: Optional[List[float]] = None,
                      name: str = "sphere") -> int:
        """
        Create a sphere object in simulation.
        
        Args:
            position: [x, y, z] position
            radius: Sphere radius
            mass: Object mass
            color: [R, G, B, A] color
            name: Object identifier
            
        Returns:
            Object ID
        """
        if not self.is_initialized:
            obj_id = len(self.object_ids)
            self.object_ids[name] = obj_id
            return obj_id
        
        import pybullet as p
        
        visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=radius)
        collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
        
        obj_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        
        if color:
            p.changeVisualShape(obj_id, -1, rgbaColor=color)
        
        self.object_ids[name] = obj_id
        return obj_id
    
    def set_joint_position(self, robot_name: str, joint_index: int, 
                          target_position: float):
        """
        Set joint position for a robot.
        
        Args:
            robot_name: Robot identifier
            joint_index: Joint index
            target_position: Target joint angle
        """
        if not self.is_initialized:
            return
        
        import pybullet as p
        robot_id = self.robot_ids.get(robot_name)
        
        if robot_id is not None:
            p.setJointMotorControl2(
                robot_id,
                joint_index,
                p.POSITION_CONTROL,
                targetPosition=target_position
            )
    
    def set_joint_positions(self, robot_name: str, 
                           target_positions: List[float]):
        """
        Set multiple joint positions.
        
        Args:
            robot_name: Robot identifier
            target_positions: List of target joint angles
        """
        if not self.is_initialized:
            return
        
        import pybullet as p
        robot_id = self.robot_ids.get(robot_name)
        
        if robot_id is not None:
            num_joints = p.getNumJoints(robot_id)
            for i, pos in enumerate(target_positions):
                if i < num_joints:
                    p.setJointMotorControl2(
                        robot_id,
                        i,
                        p.POSITION_CONTROL,
                        targetPosition=pos
                    )
    
    def get_joint_states(self, robot_name: str) -> List[Tuple]:
        """
        Get current joint states.
        
        Args:
            robot_name: Robot identifier
            
        Returns:
            List of (position, velocity, reaction_force, reaction_torque)
        """
        if not self.is_initialized:
            return []
        
        import pybullet as p
        robot_id = self.robot_ids.get(robot_name)
        
        if robot_id is None:
            return []
        
        num_joints = p.getNumJoints(robot_id)
        states = []
        
        for i in range(num_joints):
            state = p.getJointState(robot_id, i)
            states.append(state)
        
        return states
    
    def get_link_state(self, robot_name: str, link_index: int) -> Dict:
        """
        Get state of a specific link.
        
        Args:
            robot_name: Robot identifier
            link_index: Link index
            
        Returns:
            Dictionary with position, orientation, velocity
        """
        if not self.is_initialized:
            return {}
        
        import pybullet as p
        robot_id = self.robot_ids.get(robot_name)
        
        if robot_id is None:
            return {}
        
        state = p.getLinkState(robot_id, link_index)
        
        return {
            'position': state[0],
            'orientation': state[1],
            'linear_velocity': state[6],
            'angular_velocity': state[7]
        }
    
    def step_simulation(self):
        """Advance simulation by one time step."""
        if not self.is_initialized:
            return
        
        import pybullet as p
        p.stepSimulation()
    
    def check_collision(self, body_a: int, body_b: int) -> bool:
        """
        Check if two bodies are colliding.
        
        Args:
            body_a: First body ID
            body_b: Second body ID
            
        Returns:
            True if colliding
        """
        if not self.is_initialized:
            return False
        
        import pybullet as p
        contacts = p.getContactPoints(bodyA=body_a, bodyB=body_b)
        return len(contacts) > 0
    
    def get_contact_points(self) -> List[Dict]:
        """
        Get all contact points in simulation.
        
        Returns:
            List of contact information dictionaries
        """
        if not self.is_initialized:
            return []
        
        import pybullet as p
        contacts = p.getContactPoints()
        
        contact_list = []
        for contact in contacts:
            contact_list.append({
                'body_a': contact[0],
                'body_b': contact[1],
                'link_a': contact[3],
                'link_b': contact[4],
                'position': contact[5],
                'normal': contact[7],
                'force': contact[9]
            })
        
        return contact_list
    
    def reset_simulation(self):
        """Reset the simulation to initial state."""
        if not self.is_initialized:
            return
        
        import pybullet as p
        p.resetSimulation()
        self.robot_ids.clear()
        self.object_ids.clear()
    
    def set_camera(self, distance: float, yaw: float, pitch: float,
                   target_position: List[float]):
        """
        Set camera view.
        
        Args:
            distance: Camera distance from target
            yaw: Camera yaw angle
            pitch: Camera pitch angle
            target_position: [x, y, z] point to look at
        """
        if not self.is_initialized:
            return
        
        import pybullet as p
        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=target_position
        )
    
    def capture_image(self, width: int = 640, height: int = 480) -> np.ndarray:
        """
        Capture image from simulation camera.
        
        Args:
            width: Image width
            height: Image height
            
        Returns:
            RGB image as numpy array
        """
        if not self.is_initialized:
            return np.zeros((height, width, 3), dtype=np.uint8)
        
        import pybullet as p
        
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0, 0, 0],
            distance=2,
            yaw=0,
            pitch=-30,
            roll=0,
            upAxisIndex=2
        )
        
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=width/height,
            nearVal=0.1,
            farVal=100
        )
        
        img_arr = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix
        )
        
        # Extract RGB channel
        rgb = np.array(img_arr[2]).reshape((height, width, 4))[:, :, :3]
        return rgb
    
    def disconnect(self):
        """Disconnect from physics server."""
        if not self.is_initialized:
            return
        
        import pybullet as p
        p.disconnect()
        self.is_initialized = False


class SimulationScenario:
    """
    Pre-defined simulation scenarios for common robotics tasks.
    """
    
    @staticmethod
    def pick_and_place(engine: PhysicsEngine):
        """
        Set up a pick-and-place scenario.
        
        Creates a table, objects to pick, and target locations.
        """
        # Create table
        engine.create_box(
            position=[0, 0, -0.5],
            size=[2, 2, 0.1],
            mass=0,
            color=[0.5, 0.3, 0.1, 1],
            name="table"
        )
        
        # Create object to pick
        engine.create_box(
            position=[0.5, 0.3, 0.05],
            size=[0.05, 0.05, 0.05],
            mass=0.1,
            color=[0, 0.8, 0, 1],
            name="target_object"
        )
        
        # Create target location
        engine.create_sphere(
            position=[-0.5, -0.3, 0.025],
            radius=0.03,
            mass=0,
            color=[0.8, 0, 0, 1],
            name="target_location"
        )
    
    @staticmethod
    def obstacle_course(engine: PhysicsEngine):
        """
        Set up an obstacle course for path planning practice.
        """
        # Create ground
        engine.create_box(
            position=[0, 0, -0.5],
            size=[4, 4, 0.1],
            mass=0,
            color=[0.3, 0.3, 0.3, 1],
            name="ground"
        )
        
        # Create obstacles
        obstacles = [
            ([0.5, 0.5, 0.1], [0.2, 0.2, 0.2]),
            ([-0.5, 0.5, 0.15], [0.3, 0.1, 0.3]),
            ([0.5, -0.5, 0.1], [0.15, 0.15, 0.2]),
            ([-0.5, -0.5, 0.2], [0.25, 0.25, 0.4]),
        ]
        
        for i, (pos, size) in enumerate(obstacles):
            engine.create_box(
                position=pos,
                size=size,
                mass=0,
                color=[0.8, 0.2, 0.2, 1],
                name=f"obstacle_{i}"
            )
    
    @staticmethod
    def conveyor_belt(engine: PhysicsEngine):
        """
        Set up a conveyor belt simulation.
        """
        # Create conveyor surface
        engine.create_box(
            position=[0, 0, 0],
            size=[2, 0.5, 0.05],
            mass=0,
            color=[0.4, 0.4, 0.4, 1],
            name="conveyor"
        )
        
        # Create objects on conveyor
        for i in range(5):
            engine.create_box(
                position=[-0.8 + i*0.3, 0, 0.1],
                size=[0.08, 0.08, 0.08],
                mass=0.05,
                color=[0.2, 0.6, 0.8, 1],
                name=f"package_{i}"
            )

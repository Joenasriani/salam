"""
RoboSim Studio - Visualization Module

Provides 2D/3D visualization using Pygame, point cloud rendering with pclpy,
and interactive user interface components.
"""

import numpy as np
from typing import List, Dict, Optional, Tuple
import math


class RobotVisualizer2D:
    """
    2D visualization of robot kinematics using Pygame.
    
    Features:
    - Planar robot arm visualization
    - Workspace plotting
    - Trajectory animation
    - Collision visualization
    """
    
    def __init__(self, width: int = 800, height: int = 600):
        """
        Initialize 2D visualizer.
        
        Args:
            width: Window width in pixels
            height: Window height in pixels
        """
        self.width = width
        self.height = height
        self.scale = 100  # pixels per meter
        self.offset_x = width // 2
        self.offset_y = height // 2
        self.running = False
        self.screen = None
        self.clock = None
        
        try:
            import pygame
            self.pygame = pygame
            pygame.init()
        except ImportError:
            print("Pygame not installed. Visualization disabled.")
            self.pygame = None
    
    def start(self, title: str = "RoboSim Studio - 2D View"):
        """Start the visualization window."""
        if not self.pygame:
            return
        
        self.screen = self.pygame.display.set_mode((self.width, self.height))
        self.pygame.display.set_caption(title)
        self.clock = self.pygame.time.Clock()
        self.running = True
    
    def stop(self):
        """Stop the visualization."""
        if self.pygame and self.running:
            self.pygame.quit()
            self.running = False
    
    def world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to screen coordinates."""
        screen_x = int(self.offset_x + x * self.scale)
        screen_y = int(self.offset_y - y * self.scale)  # Y is inverted in screen coords
        return (screen_x, screen_y)
    
    def draw_grid(self, spacing: float = 0.5, color: Tuple[int, int, int] = (200, 200, 200)):
        """Draw a reference grid."""
        if not self.screen:
            return
        
        # Vertical lines
        x = -self.offset_x / self.scale
        while x * self.scale < self.width:
            start = self.world_to_screen(x, -10)
            end = self.world_to_screen(x, 10)
            self.pygame.draw.line(self.screen, color, start, end)
            x += spacing
        
        # Horizontal lines
        y = -self.height / self.scale
        while y * self.scale < self.height:
            start = self.world_to_screen(-10, y)
            end = self.world_to_screen(10, y)
            self.pygame.draw.line(self.screen, color, start, end)
            y += spacing
    
    def draw_robot_arm(self, joint_angles: List[float], link_lengths: List[float],
                       base_pos: Tuple[float, float] = (0, 0),
                       color: Tuple[int, int, int] = (0, 100, 200)):
        """
        Draw a planar robot arm.
        
        Args:
            joint_angles: List of joint angles in radians
            link_lengths: List of link lengths
            base_pos: Base position (x, y)
            color: RGB color for the arm
        """
        if not self.screen or len(joint_angles) != len(link_lengths):
            return
        
        current_x, current_y = base_pos
        current_angle = 0
        
        # Draw base
        base_screen = self.world_to_screen(current_x, current_y)
        self.pygame.draw.circle(self.screen, (50, 50, 50), base_screen, 8)
        
        # Draw links and joints
        for i, (angle, length) in enumerate(zip(joint_angles, link_lengths)):
            current_angle += angle
            
            # Calculate end position
            end_x = current_x + length * math.cos(current_angle)
            end_y = current_y + length * math.sin(current_angle)
            
            # Draw link
            start_screen = self.world_to_screen(current_x, current_y)
            end_screen = self.world_to_screen(end_x, end_y)
            self.pygame.draw.line(self.screen, color, start_screen, end_screen, 5)
            
            # Draw joint
            self.pygame.draw.circle(self.screen, (100, 100, 255), end_screen, 6)
            
            current_x, current_y = end_x, end_y
        
        # Draw end effector
        end_screen = self.world_to_screen(current_x, current_y)
        self.pygame.draw.circle(self.screen, (255, 100, 100), end_screen, 8)
    
    def draw_workspace(self, positions: np.ndarray, 
                       color: Tuple[int, int, int] = (100, 200, 100),
                       alpha: int = 50):
        """
        Draw robot workspace from sampled positions.
        
        Args:
            positions: Nx2 or Nx3 array of positions
            color: Point color
            alpha: Transparency (not fully supported in basic pygame)
        """
        if not self.screen:
            return
        
        for pos in positions:
            x, y = pos[0], pos[1] if len(pos) > 1 else 0
            screen_pos = self.world_to_screen(x, y)
            
            # Only draw if within screen bounds
            if 0 <= screen_pos[0] < self.width and 0 <= screen_pos[1] < self.height:
                self.pygame.draw.circle(self.screen, color, screen_pos, 2)
    
    def draw_target(self, position: Tuple[float, float],
                    color: Tuple[int, int, int] = (255, 0, 0),
                    size: int = 10):
        """
        Draw a target position.
        
        Args:
            position: Target (x, y) position
            color: Target color
            size: Target size
        """
        if not self.screen:
            return
        
        screen_pos = self.world_to_screen(position[0], position[1])
        
        # Draw crosshair
        self.pygame.draw.line(self.screen, color, 
                             (screen_pos[0] - size, screen_pos[1]),
                             (screen_pos[0] + size, screen_pos[1]), 2)
        self.pygame.draw.line(self.screen, color,
                             (screen_pos[0], screen_pos[1] - size),
                             (screen_pos[0], screen_pos[1] + size), 2)
        
        # Draw circle
        self.pygame.draw.circle(self.screen, color, screen_pos, size, 1)
    
    def draw_trajectory(self, waypoints: List[Tuple[float, float]],
                        color: Tuple[int, int, int] = (255, 100, 0)):
        """
        Draw a trajectory path.
        
        Args:
            waypoints: List of (x, y) waypoints
            color: Path color
        """
        if not self.screen or len(waypoints) < 2:
            return
        
        points = [self.world_to_screen(x, y) for x, y in waypoints]
        self.pygame.draw.lines(self.screen, color, False, points, 3)
    
    def draw_collision_warning(self, position: Tuple[float, float]):
        """
        Draw a collision warning indicator.
        
        Args:
            position: Position of potential collision
        """
        if not self.screen:
            return
        
        screen_pos = self.world_to_screen(position[0], position[1])
        
        # Flashing warning circle
        time_val = self.pygame.time.get_ticks() / 200
        radius = 15 + int(5 * math.sin(time_val))
        
        self.pygame.draw.circle(self.screen, (255, 200, 0), screen_pos, radius, 3)
        self.pygame.draw.circle(self.screen, (255, 100, 0), screen_pos, radius - 5, 2)
    
    def draw_text(self, text: str, position: Tuple[int, int],
                  color: Tuple[int, int, int] = (0, 0, 0),
                  size: int = 24):
        """
        Draw text on screen.
        
        Args:
            text: Text to display
            position: Screen position (x, y)
            color: Text color
            size: Font size
        """
        if not self.screen:
            return
        
        font = self.pygame.font.Font(None, size)
        text_surface = font.render(text, True, color)
        self.screen.blit(text_surface, position)
    
    def draw_info_panel(self, info_dict: Dict):
        """
        Draw an information panel with key-value pairs.
        
        Args:
            info_dict: Dictionary of information to display
        """
        if not self.screen:
            return
        
        y_offset = 10
        for key, value in info_dict.items():
            text = f"{key}: {value}"
            self.draw_text(text, (10, y_offset), color=(0, 0, 0), size=20)
            y_offset += 25
    
    def handle_events(self) -> Dict:
        """
        Handle pygame events.
        
        Returns:
            Dictionary with event information
        """
        if not self.pygame or not self.running:
            return {}
        
        events = {'quit': False, 'key_pressed': None, 'mouse_click': None}
        
        for event in self.pygame.event.get():
            if event.type == self.pygame.QUIT:
                events['quit'] = True
            
            elif event.type == self.pygame.KEYDOWN:
                events['key_pressed'] = event.key
            
            elif event.type == self.pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    events['mouse_click'] = self.pygame.mouse.get_pos()
        
        return events
    
    def update(self):
        """Update the display."""
        if not self.screen:
            return
        
        self.pygame.display.flip()
        if self.clock:
            self.clock.tick(60)  # 60 FPS
    
    def clear(self):
        """Clear the screen."""
        if self.screen:
            self.screen.fill((255, 255, 255))


class PointCloudVisualizer:
    """
    Point cloud visualization using pclpy.
    
    Features:
    - Load and display point clouds
    - Filter and process point clouds
    - Extract features and normals
    """
    
    def __init__(self):
        """Initialize point cloud visualizer."""
        try:
            import pclpy
            self.pclpy = pclpy
            self.available = True
        except ImportError:
            print("pclpy not available. Using fallback visualization.")
            self.available = False
            self.pclpy = None
    
    def create_point_cloud(self, points: np.ndarray) -> object:
        """
        Create a point cloud from numpy array.
        
        Args:
            points: Nx3 array of points
            
        Returns:
            Point cloud object
        """
        if not self.available:
            return {'points': points, 'type': 'fallback'}
        
        xyz = points.astype(np.float32)
        cloud = self.pclpy.pointcloud.PointCloud(xyz)
        return cloud
    
    def filter_passthrough(self, cloud: object, axis: str = 'z',
                           min_val: float = 0, max_val: float = 2) -> object:
        """
        Apply passthrough filter to point cloud.
        
        Args:
            cloud: Input point cloud
            axis: Axis to filter on
            min_val: Minimum value
            max_val: Maximum value
            
        Returns:
            Filtered point cloud
        """
        if not self.available:
            return cloud
        
        filter_obj = self.pclpy.filter.PassThrough(axis)
        filter_obj.setInputCloud(cloud)
        filter_obj.setFilterLimits(min_val, max_val)
        
        filtered = self.pclpy.pointcloud.PointCloud()
        filter_obj.apply(filtered)
        return filtered
    
    def filter_voxel_grid(self, cloud: object, leaf_size: float = 0.01) -> object:
        """
        Downsample point cloud using voxel grid filter.
        
        Args:
            cloud: Input point cloud
            leaf_size: Voxel size
            
        Returns:
            Downsampled point cloud
        """
        if not self.available:
            return cloud
        
        filter_obj = self.pclpy.filter.VoxelGrid(leaf_size)
        filter_obj.setInputCloud(cloud)
        
        filtered = self.pclpy.pointcloud.PointCloud()
        filter_obj.apply(filtered)
        return filtered
    
    def compute_normals(self, cloud: object, search_radius: float = 0.03) -> object:
        """
        Compute surface normals for point cloud.
        
        Args:
            cloud: Input point cloud
            search_radius: Radius for normal estimation
            
        Returns:
            Point cloud with normals
        """
        if not self.available:
            return cloud
        
        normal_est = self.pclpy.surface.NormalEstimation(search_radius)
        normal_est.setInputCloud(cloud)
        
        cloud_with_normals = self.pclpy.pointcloud.PointCloud()
        normal_est.compute(cloud_with_normals)
        return cloud_with_normals
    
    def segment_plane(self, cloud: object, distance_threshold: float = 0.01) -> Tuple:
        """
        Segment plane from point cloud using RANSAC.
        
        Args:
            cloud: Input point cloud
            distance_threshold: Distance threshold for inliers
            
        Returns:
            Tuple of (plane_model, inlier_cloud, outlier_cloud)
        """
        if not self.available:
            return None, cloud, self.create_point_cloud(np.array([]))
        
        segmentation = self.pclpy.segment.SACSegmentation(
            model_type=self.pclpy.segment.SACModel.PLANE,
            method=self.pclpy.segment.SACMethod.RANSAC,
            distance_threshold=distance_threshold
        )
        
        segmentation.setInputCloud(cloud)
        
        indices = self.pclpy.pointcloud.PointIndices()
        coefficients = self.pclpy.pointcloud.ModelCoefficients()
        
        segmentation.segment(indices, coefficients)
        
        inlier_cloud = self.pclpy.pointcloud.PointCloud()
        outlier_cloud = self.pclpy.pointcloud.PointCloud()
        
        extract_inliers = self.pclpy.filter.ExtractIndices()
        extract_inliers.setInputCloud(cloud)
        extract_inliers.setIndices(indices)
        extract_inliers.setNegative(False)
        extract_inliers.apply(inlier_cloud)
        
        extract_inliers.setNegative(True)
        extract_inliers.apply(outlier_cloud)
        
        plane_model = coefficients.values
        
        return plane_model, inlier_cloud, outlier_cloud
    
    def get_points_array(self, cloud: object) -> np.ndarray:
        """
        Extract points from point cloud as numpy array.
        
        Args:
            cloud: Point cloud object
            
        Returns:
            Nx3 numpy array
        """
        if isinstance(cloud, dict) and cloud.get('type') == 'fallback':
            return cloud['points']
        
        if not self.available:
            return np.array([])
        
        return np.asarray(cloud.xyz())


class InteractiveUI:
    """
    Interactive user interface components for RoboSim Studio.
    
    Provides:
    - Slider controls for joint angles
    - Button interfaces
    - Real-time parameter adjustment
    - Mode selection
    """
    
    def __init__(self, visualizer: RobotVisualizer2D):
        """
        Initialize interactive UI.
        
        Args:
            visualizer: RobotVisualizer2D instance
        """
        self.visualizer = visualizer
        self.sliders = {}
        self.buttons = {}
        self.current_mode = "design"
        self.callbacks = {}
    
    def add_slider(self, name: str, min_val: float, max_val: float,
                   initial_val: float, position: Tuple[int, int],
                   callback=None):
        """
        Add a slider control.
        
        Args:
            name: Slider identifier
            min_val: Minimum value
            max_val: Maximum value
            initial_val: Initial value
            position: (x, y) screen position
            callback: Function to call on value change
        """
        self.sliders[name] = {
            'min': min_val,
            'max': max_val,
            'value': initial_val,
            'position': position,
            'width': 150,
            'height': 20,
            'dragging': False,
            'callback': callback
        }
    
    def add_button(self, name: str, label: str, position: Tuple[int, int],
                   size: Tuple[int, int] = (100, 40), callback=None):
        """
        Add a button.
        
        Args:
            name: Button identifier
            label: Button text
            position: (x, y) screen position
            size: (width, height)
            callback: Function to call on click
        """
        self.buttons[name] = {
            'label': label,
            'position': position,
            'size': size,
            'hovered': False,
            'callback': callback
        }
    
    def set_mode(self, mode: str):
        """
        Set UI mode.
        
        Args:
            mode: Mode name ("design", "simulate", "learn")
        """
        self.current_mode = mode
    
    def handle_mouse_event(self, event: Dict):
        """
        Handle mouse events for interactive elements.
        
        Args:
            event: Event dictionary from visualizer
        """
        if not event.get('mouse_click'):
            return
        
        mouse_x, mouse_y = event['mouse_click']
        
        # Check button clicks
        for name, button in self.buttons.items():
            bx, by = button['position']
            bw, bh = button['size']
            
            if bx <= mouse_x <= bx + bw and by <= mouse_y <= by + bh:
                if button['callback']:
                    button['callback'](name)
                break
        
        # Check slider interactions
        for name, slider in self.sliders.items():
            sx, sy = slider['position']
            sw, sh = slider['width'], slider['height']
            
            if sx <= mouse_x <= sx + sw and sy <= mouse_y <= sy + sh:
                slider['dragging'] = True
    
    def handle_mouse_release(self):
        """Handle mouse release events."""
        for slider in self.sliders.values():
            slider['dragging'] = False
    
    def handle_mouse_motion(self, mouse_pos: Tuple[int, int]):
        """
        Handle mouse motion for sliders.
        
        Args:
            mouse_pos: Current mouse position
        """
        mouse_x, mouse_y = mouse_pos
        
        for name, slider in self.sliders.items():
            if slider['dragging']:
                sx, sy = slider['position']
                sw = slider['width']
                
                # Calculate new value based on mouse position
                rel_x = (mouse_x - sx) / sw
                rel_x = max(0, min(1, rel_x))
                
                new_value = slider['min'] + rel_x * (slider['max'] - slider['min'])
                
                if abs(new_value - slider['value']) > 0.01:
                    slider['value'] = new_value
                    
                    if slider['callback']:
                        slider['callback'](name, new_value)
    
    def draw(self):
        """Draw all UI elements."""
        if not self.visualizer.screen:
            return
        
        pygame = self.visualizer.pygame
        
        # Draw sliders
        for name, slider in self.sliders.items():
            sx, sy = slider['position']
            sw, sh = slider['width'], slider['height']
            
            # Background
            pygame.draw.rect(self.visualizer.screen, (200, 200, 200),
                           (sx, sy, sw, sh))
            
            # Fill based on value
            fill_width = int((slider['value'] - slider['min']) / 
                            (slider['max'] - slider['min']) * sw)
            pygame.draw.rect(self.visualizer.screen, (0, 150, 255),
                           (sx, sy, fill_width, sh))
            
            # Border
            pygame.draw.rect(self.visualizer.screen, (100, 100, 100),
                           (sx, sy, sw, sh), 2)
            
            # Label
            self.visualizer.draw_text(f"{name}: {slider['value']:.2f}",
                                     (sx, sy - 20), size=18)
        
        # Draw buttons
        for name, button in self.buttons.items():
            bx, by = button['position']
            bw, bh = button['size']
            
            color = (100, 200, 100) if button['hovered'] else (50, 150, 50)
            pygame.draw.rect(self.visualizer.screen, color,
                           (bx, by, bw, bh), border_radius=5)
            pygame.draw.rect(self.visualizer.screen, (30, 100, 30),
                           (bx, by, bw, bh), 2, border_radius=5)
            
            # Button label
            self.visualizer.draw_text(button['label'],
                                     (bx + 10, by + 10),
                                     color=(255, 255, 255),
                                     size=24)
    
    def get_slider_values(self) -> Dict[str, float]:
        """Get all current slider values."""
        return {name: slider['value'] for name, slider in self.sliders.items()}
    
    def set_slider_value(self, name: str, value: float):
        """
        Set slider value programmatically.
        
        Args:
            name: Slider name
            value: New value
        """
        if name in self.sliders:
            slider = self.sliders[name]
            value = max(slider['min'], min(slider['max'], value))
            slider['value'] = value

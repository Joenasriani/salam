"""
RoboNet Behavior Runner - Executes RBP scenario files in PyBullet.

This is the core execution engine that reads JSON scenario definitions
and runs them in the physics simulation with real-time state management.
"""

import json
import time
from typing import Dict, List, Optional, Any, Callable
from enum import Enum
import numpy as np

from core.schema import (
    ScenarioFile, TaskNode, NodeType, EdgeCondition,
    TriggerType, NodeConfig
)
from simulation.physics_engine import PhysicsEngine


class NodeStatus(str, Enum):
    """Execution status of a task node."""
    PENDING = "pending"
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"
    SKIPPED = "skipped"


class ExecutionState:
    """Tracks the current state of scenario execution."""
    
    def __init__(self, scenario: ScenarioFile):
        self.scenario = scenario
        self.node_statuses: Dict[str, NodeStatus] = {}
        self.current_node_id: Optional[str] = None
        self.completed_nodes: List[str] = []
        self.failed_nodes: List[str] = []
        self.start_time: float = 0
        self.elapsed_time: float = 0
        self.telemetry_data: List[Dict] = []
        
        # Initialize all nodes as pending
        for node_data in scenario.task_graph['nodes']:
            self.node_statuses[node_data['id']] = NodeStatus.PENDING
    
    def get_node(self, node_id: str) -> Optional[Dict]:
        """Get node data by ID."""
        for node_data in self.scenario.task_graph['nodes']:
            if node_data['id'] == node_id:
                return node_data
        return None
    
    def get_outgoing_edges(self, node_id: str) -> List[Dict]:
        """Get all edges originating from a node."""
        return [
            edge for edge in self.scenario.task_graph['edges']
            if edge['source'] == node_id
        ]
    
    def find_next_node(self, current_node_id: str) -> Optional[str]:
        """Find the next node to execute based on edge conditions."""
        outgoing = self.get_outgoing_edges(current_node_id)
        current_status = self.node_statuses[current_node_id]
        
        for edge in outgoing:
            condition = edge.get('condition', {'type': 'always'})
            condition_type = condition.get('type', 'always')
            
            # Check if condition is satisfied
            should_transition = False
            if condition_type == 'always':
                should_transition = True
            elif condition_type == 'on_success' and current_status == NodeStatus.SUCCESS:
                should_transition = True
            elif condition_type == 'on_failure' and current_status == NodeStatus.FAILURE:
                should_transition = True
            elif condition_type == 'custom':
                # Custom expression evaluation (simplified)
                should_transition = True  # Would need safe eval in production
            
            if should_transition:
                # Check timeout
                timeout = condition.get('timeout')
                if timeout and self.elapsed_time > timeout:
                    continue
                
                return edge['target']
        
        return None


class ScenarioRunner:
    """
    Executes RBP scenario files in PyBullet.
    
    This class loads a scenario, initializes the physics engine,
    and executes the task graph node by node.
    """
    
    def __init__(self, scenario_path: Optional[str] = None, 
                 scenario_data: Optional[ScenarioFile] = None):
        """
        Initialize the runner.
        
        Args:
            scenario_path: Path to scenario JSON file
            scenario_data: Pre-loaded ScenarioFile object
        """
        if scenario_path:
            self.scenario = ScenarioFile.load(scenario_path)
        elif scenario_data:
            self.scenario = scenario_data
        else:
            raise ValueError("Must provide either scenario_path or scenario_data")
        
        self.physics_engine = PhysicsEngine(gui=True)
        self.state = ExecutionState(self.scenario)
        self.robot_id = None
        self.is_running = False
        self.current_node_executor: Optional[Callable] = None
        
        # Node type executors
        self.node_executors = {
            NodeType.TRIGGER: self._execute_trigger,
            NodeType.MOTION_PLAN: self._execute_motion_plan,
            NodeType.MANIPULATION: self._execute_manipulation,
            NodeType.WAIT: self._execute_wait,
            NodeType.CONDITION: self._execute_condition,
            NodeType.ACTION: self._execute_action,
        }
    
    def initialize(self):
        """Initialize physics engine and load scenario assets."""
        print(f"Initializing scenario: {self.scenario.metadata.name}")
        
        # Initialize physics
        self.physics_engine.initialize()
        
        # Load environment assets
        for asset in self.scenario.environment.assets:
            self._load_asset(asset)
        
        # Load robot if URDF specified
        if self.scenario.robot.urdf_path:
            self.robot_id = self.physics_engine.load_robot(
                self.scenario.robot.urdf_path,
                name="scenario_robot"
            )
        
        print("Scenario initialized successfully")
    
    def _load_asset(self, asset):
        """Load an environment asset into simulation."""
        asset_type = asset.type
        path = asset.path
        pose = asset.pose
        
        if asset_type == 'urdf':
            self.physics_engine.load_urdf(path, pose)
        elif asset_type == 'primitive':
            # Handle primitive shapes (box, sphere, cylinder)
            pass  # Implementation depends on physics engine capabilities
        else:
            print(f"Warning: Unsupported asset type: {asset_type}")
    
    def run(self, max_steps: int = 1000, realtime: bool = True):
        """
        Execute the scenario.
        
        Args:
            max_steps: Maximum number of execution steps
            realtime: Whether to run at realtime speed
        """
        self.initialize()
        self.is_running = True
        self.state.start_time = time.time()
        
        # Find starting node (first trigger node)
        start_node = None
        for node_data in self.scenario.task_graph['nodes']:
            if node_data['type'] == 'trigger':
                start_node = node_data['id']
                break
        
        if not start_node:
            print("Error: No trigger node found in scenario")
            return
        
        self.state.current_node_id = start_node
        step_count = 0
        
        try:
            while self.is_running and step_count < max_steps:
                # Execute current node
                success = self._execute_current_node()
                
                if not success:
                    print(f"Node execution failed: {self.state.current_node_id}")
                    self.state.node_statuses[self.state.current_node_id] = NodeStatus.FAILURE
                    self.state.failed_nodes.append(self.state.current_node_id)
                    
                    # Try to find next node on failure
                    next_node = self.state.find_next_node(self.state.current_node_id)
                    if next_node:
                        self.state.current_node_id = next_node
                        self.state.node_statuses[next_node] = NodeStatus.PENDING
                    else:
                        print("No recovery path found, stopping execution")
                        break
                else:
                    # Mark current node as success
                    current_id = self.state.current_node_id
                    self.state.node_statuses[current_id] = NodeStatus.SUCCESS
                    self.state.completed_nodes.append(current_id)
                    
                    # Find and transition to next node
                    next_node = self.state.find_next_node(current_id)
                    if next_node:
                        print(f"Transitioning: {current_id} -> {next_node}")
                        self.state.current_node_id = next_node
                        self.state.node_statuses[next_node] = NodeStatus.PENDING
                    else:
                        print("Scenario completed successfully!")
                        break
                
                # Physics step
                self.physics_engine.step_simulation()
                
                # Collect telemetry
                if self.scenario.telemetry.enabled:
                    self._collect_telemetry()
                
                # Realtime pacing
                if realtime:
                    time.sleep(self.physics_engine.time_step)
                
                step_count += 1
                
        except Exception as e:
            print(f"Execution error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.is_running = False
            self.state.elapsed_time = time.time() - self.state.start_time
            print(f"Execution finished. Elapsed time: {self.state.elapsed_time:.2f}s")
            print(f"Completed nodes: {len(self.state.completed_nodes)}")
            print(f"Failed nodes: {len(self.state.failed_nodes)}")
    
    def _execute_current_node(self) -> bool:
        """Execute the current node based on its type."""
        node_id = self.state.current_node_id
        node_data = self.state.get_node(node_id)
        
        if not node_data:
            print(f"Error: Node {node_id} not found")
            return False
        
        node_type = NodeType(node_data['type'])
        executor = self.node_executors.get(node_type)
        
        if not executor:
            print(f"Error: No executor for node type: {node_type}")
            return False
        
        # Execute the node
        return executor(node_data)
    
    def _execute_trigger(self, node_data: Dict) -> bool:
        """Execute a trigger node (immediate success)."""
        print(f"Executing trigger: {node_data['name']}")
        trigger_params = node_data.get('trigger_params', {})
        
        # Check for timer delay
        if 'delay' in trigger_params:
            time.sleep(trigger_params['delay'])
        
        return True
    
    def _execute_motion_plan(self, node_data: Dict) -> bool:
        """Execute a motion planning node."""
        print(f"Executing motion plan: {node_data['name']}")
        config = node_data.get('config', {}).get('params', {})
        
        target_position = config.get('target_position', [0, 0, 0])
        max_velocity = config.get('max_velocity', 0.5)
        
        if self.robot_id is None:
            print("Error: No robot loaded for motion planning")
            return False
        
        # Simple motion execution (in production, use OMPL or similar)
        # Get current robot state
        joint_states = self.physics_engine.get_joint_states(self.robot_id)
        
        # Move joints toward target (simplified)
        num_joints = len(joint_states)
        target_angles = [0.5] * num_joints  # Simplified target
        
        for i in range(50):  # Simulate motion over 50 steps
            if not self.is_running:
                return False
            
            # Interpolate toward target
            progress = i / 50.0
            current_angles = [(1-progress)*curr + progress*targ 
                            for curr, targ in zip(joint_states, target_angles)]
            
            # Apply joint positions
            for j, angle in enumerate(current_angles):
                self.physics_engine.set_joint_position(self.robot_id, j, angle)
            
            self.physics_engine.step_simulation()
            time.sleep(self.physics_engine.time_step)
        
        return True
    
    def _execute_manipulation(self, node_data: Dict) -> bool:
        """Execute a manipulation node (gripper control, etc.)."""
        print(f"Executing manipulation: {node_data['name']}")
        config = node_data.get('config', {}).get('params', {})
        
        action = config.get('action', 'grip')
        
        if action == 'close_gripper':
            print("Closing gripper")
            # Implement gripper control
        elif action == 'open_gripper':
            print("Opening gripper")
            # Implement gripper control
        
        time.sleep(0.5)  # Simulate action duration
        return True
    
    def _execute_wait(self, node_data: Dict) -> bool:
        """Execute a wait node."""
        print(f"Executing wait: {node_data['name']}")
        config = node_data.get('config', {}).get('params', {})
        
        duration = config.get('duration', 1.0)
        time.sleep(duration)
        return True
    
    def _execute_condition(self, node_data: Dict) -> bool:
        """Execute a condition check node."""
        print(f"Checking condition: {node_data['name']}")
        config = node_data.get('config', {}).get('params', {})
        
        condition_type = config.get('type', 'custom')
        
        # Implement condition checking logic
        # For now, always succeed
        return True
    
    def _execute_action(self, node_data: Dict) -> bool:
        """Execute a generic action node."""
        print(f"Executing action: {node_data['name']}")
        # Custom action implementation
        return True
    
    def _collect_telemetry(self):
        """Collect telemetry data from simulation."""
        telemetry = {
            'timestamp': time.time() - self.state.start_time,
            'current_node': self.state.current_node_id,
            'node_status': self.state.node_statuses.get(self.state.current_node_id, 'unknown'),
        }
        
        if self.robot_id is not None:
            joint_states = self.physics_engine.get_joint_states(self.robot_id)
            telemetry['joint_positions'] = joint_states
        
        self.state.telemetry_data.append(telemetry)
    
    def get_status(self) -> Dict:
        """Get current execution status."""
        return {
            'is_running': self.is_running,
            'current_node': self.state.current_node_id,
            'completed_nodes': self.state.completed_nodes,
            'failed_nodes': self.state.failed_nodes,
            'elapsed_time': self.state.elapsed_time,
            'node_statuses': {k: v.value for k, v in self.state.node_statuses.items()}
        }
    
    def stop(self):
        """Stop scenario execution."""
        self.is_running = False
        print("Stopping scenario execution...")


if __name__ == "__main__":
    # Example: Run a simple scenario
    from core.schema import create_simple_motion_scenario
    
    # Create a test scenario
    scenario = create_simple_motion_scenario(
        name="Test Motion Execution",
        target_position=[0.5, 0.3, 0.2],
        robot_dof=4
    )
    
    # Save it
    scenario.save("data/scenarios/test_execution.json")
    
    # Run it
    runner = ScenarioRunner(scenario_data=scenario)
    runner.run(max_steps=200, realtime=True)

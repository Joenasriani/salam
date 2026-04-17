"""
RoboNet Behavior Protocol (RBP) - Schema Definitions

Defines the JSON/YAML schema for shareable robot behavior packs.
This is the core specification that makes behaviors portable and versionable.
"""

from typing import List, Dict, Any, Optional, Literal
from pydantic import BaseModel, Field, field_validator
from enum import Enum
import json


class NodeType(str, Enum):
    """Types of task nodes in the behavior graph."""
    TRIGGER = "trigger"
    MOTION_PLAN = "motion_plan"
    MANIPULATION = "manipulation"
    PERCEPTION = "perception"
    CONDITION = "condition"
    ACTION = "action"
    WAIT = "wait"
    CUSTOM = "custom"


class TriggerType(str, Enum):
    """Trigger types to start node execution."""
    IMMEDIATE = "immediate"
    ON_COMPLETE = "on_complete"
    ON_CONDITION = "on_condition"
    TIMER = "timer"
    SENSOR_EVENT = "sensor_event"


class EdgeCondition(BaseModel):
    """Condition for transitioning between nodes."""
    type: Literal["always", "on_success", "on_failure", "custom"] = "always"
    expression: Optional[str] = None  # Python expression for custom conditions
    timeout: Optional[float] = None  # Timeout in seconds
    
    class Config:
        schema_extra = {
            "example": {
                "type": "on_success",
                "timeout": 5.0
            }
        }


class NodeConfig(BaseModel):
    """Configuration parameters for a node."""
    params: Dict[str, Any] = Field(default_factory=dict)
    
    class Config:
        schema_extra = {
            "example": {
                "params": {
                    "target_position": [0.5, 0.3, 0.2],
                    "max_velocity": 0.5,
                    "max_acceleration": 1.0
                }
            }
        }


class TaskNode(BaseModel):
    """
    A single node in the behavior graph.
    
    This is the atomic unit of robot behavior that can be composed
    into complex scenarios.
    """
    id: str = Field(..., description="Unique identifier for this node")
    name: str = Field(..., description="Human-readable name")
    type: NodeType = Field(..., description="Type of task this node performs")
    description: Optional[str] = None
    config: NodeConfig = Field(default_factory=NodeConfig)
    trigger: TriggerType = Field(default=TriggerType.IMMEDIATE)
    trigger_params: Dict[str, Any] = Field(default_factory=dict)
    metadata: Dict[str, Any] = Field(default_factory=dict)
    
    class Config:
        schema_extra = {
            "example": {
                "id": "move_to_pick",
                "name": "Move to Pick Position",
                "type": "motion_plan",
                "description": "Move end-effector to object pickup location",
                "config": {
                    "params": {
                        "target_position": [0.5, 0.0, 0.3],
                        "orientation": [0, 0, 0, 1],
                        "planner": "rrt_connect",
                        "max_velocity": 0.5
                    }
                },
                "trigger": "on_complete",
                "trigger_params": {"delay": 0.5}
            }
        }


class Edge(BaseModel):
    """Directed edge connecting two nodes in the behavior graph."""
    source: str = Field(..., description="Source node ID")
    target: str = Field(..., description="Target node ID")
    condition: EdgeCondition = Field(default_factory=EdgeCondition)
    label: Optional[str] = None
    
    class Config:
        schema_extra = {
            "example": {
                "source": "move_to_pick",
                "target": "close_gripper",
                "condition": {"type": "on_success"},
                "label": "reached"
            }
        }


class RobotSpec(BaseModel):
    """Specification of the robot required for this scenario."""
    urdf_path: Optional[str] = None
    dh_parameters: Optional[List[Dict[str, float]]] = None
    required_dof: int = Field(..., ge=1)
    required_sensors: List[str] = Field(default_factory=list)
    end_effector_type: Optional[str] = None
    
    class Config:
        schema_extra = {
            "example": {
                "urdf_path": "robots/scara_arm.urdf",
                "required_dof": 4,
                "required_sensors": ["depth_camera", "force_torque"],
                "end_effector_type": "parallel_gripper"
            }
        }


class EnvironmentAsset(BaseModel):
    """3D asset or object in the simulation environment."""
    id: str
    type: Literal["mesh", "primitive", "urdf", "sdf"]
    path: str
    pose: List[float] = Field([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                              description="[x, y, z, roll, pitch, yaw]")
    physical_properties: Dict[str, Any] = Field(default_factory=dict)
    
    class Config:
        schema_extra = {
            "example": {
                "id": "table_01",
                "type": "urdf",
                "path": "objects/table.urdf",
                "pose": [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
                "physical_properties": {
                    "mass": 5.0,
                    "friction": 0.8
                }
            }
        }


class EnvironmentSpec(BaseModel):
    """Specification of the simulation environment."""
    gravity: List[float] = Field([0.0, 0.0, -9.81])
    assets: List[EnvironmentAsset] = Field(default_factory=list)
    lighting: Dict[str, Any] = Field(default_factory=dict)
    collision_enabled: bool = True
    
    class Config:
        schema_extra = {
            "example": {
                "gravity": [0.0, 0.0, -9.81],
                "assets": [],
                "collision_enabled": True
            }
        }


class ValidationRule(BaseModel):
    """Rule for validating scenario feasibility."""
    type: Literal["collision_free", "reachable", "time_limit", "custom"]
    params: Dict[str, Any] = Field(default_factory=dict)
    critical: bool = True  # If False, warning only


class ScenarioMetadata(BaseModel):
    """Metadata about the scenario pack."""
    name: str
    version: str
    author: str
    description: str
    tags: List[str] = Field(default_factory=list)
    license: str = "MIT"
    created_at: Optional[str] = None
    updated_at: Optional[str] = None
    dependencies: List[str] = Field(default_factory=list)
    
    class Config:
        schema_extra = {
            "example": {
                "name": "Pick and Place Basic",
                "version": "1.0.0",
                "author": "John Doe",
                "description": "Basic pick and place operation for SCARA robots",
                "tags": ["pick-and-place", "beginner", "scara"],
                "license": "MIT",
                "dependencies": ["robosim_core>=1.0.0"]
            }
        }


class TelemetryConfig(BaseModel):
    """Configuration for telemetry data collection."""
    enabled: bool = True
    sample_rate: float = 60.0  # Hz
    metrics: List[str] = Field(
        default_factory=lambda: ["position", "velocity", "torque", "collision"]
    )
    output_format: Literal["json", "csv", "rosbag"] = "json"


class ScenarioFile(BaseModel):
    """
    Complete scenario file schema.
    
    This is the top-level structure that defines a complete,
    executable robot behavior pack.
    """
    metadata: ScenarioMetadata
    robot: RobotSpec
    environment: EnvironmentSpec = Field(default_factory=EnvironmentSpec)
    task_graph: Dict[str, Any]  # Contains nodes and edges
    validation_rules: List[ValidationRule] = Field(default_factory=list)
    telemetry: TelemetryConfig = Field(default_factory=TelemetryConfig)
    
    @field_validator('task_graph')
    @classmethod
    def validate_task_graph(cls, v):
        if 'nodes' not in v or 'edges' not in v:
            raise ValueError("task_graph must contain 'nodes' and 'edges'")
        if not isinstance(v['nodes'], list):
            raise ValueError("'nodes' must be a list")
        if not isinstance(v['edges'], list):
            raise ValueError("'edges' must be a list")
        return v
    
    class Config:
        schema_extra = {
            "example": {
                "metadata": {
                    "name": "Simple Pick and Place",
                    "version": "1.0.0",
                    "author": "RoboNet Team",
                    "description": "Demonstration scenario"
                },
                "robot": {
                    "required_dof": 4,
                    "required_sensors": []
                },
                "environment": {
                    "gravity": [0.0, 0.0, -9.81],
                    "assets": []
                },
                "task_graph": {
                    "nodes": [
                        {
                            "id": "start",
                            "name": "Start",
                            "type": "trigger",
                            "config": {"params": {}}
                        },
                        {
                            "id": "move_to_object",
                            "name": "Move to Object",
                            "type": "motion_plan",
                            "config": {
                                "params": {
                                    "target_position": [0.5, 0.0, 0.3]
                                }
                            }
                        }
                    ],
                    "edges": [
                        {
                            "source": "start",
                            "target": "move_to_object",
                            "condition": {"type": "always"}
                        }
                    ]
                }
            }
        }
    
    def to_json(self, indent: int = 2) -> str:
        """Export scenario to JSON string."""
        return json.dumps(self.model_dump(), indent=indent)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'ScenarioFile':
        """Load scenario from JSON string."""
        data = json.loads(json_str)
        return cls(**data)
    
    def save(self, filepath: str):
        """Save scenario to file."""
        with open(filepath, 'w') as f:
            f.write(self.to_json())
    
    @classmethod
    def load(cls, filepath: str) -> 'ScenarioFile':
        """Load scenario from file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls(**data)


# Convenience functions for creating common scenarios
def create_simple_motion_scenario(
    name: str,
    target_position: List[float],
    robot_dof: int = 4
) -> ScenarioFile:
    """Create a simple motion planning scenario."""
    
    nodes = [
        TaskNode(
            id="init",
            name="Initialize",
            type=NodeType.TRIGGER,
            config=NodeConfig(params={})
        ).model_dump(),
        TaskNode(
            id="move_to_target",
            name="Move to Target",
            type=NodeType.MOTION_PLAN,
            config=NodeConfig(params={
                "target_position": target_position,
                "max_velocity": 0.5
            })
        ).model_dump()
    ]
    
    edges = [
        Edge(
            source="init",
            target="move_to_target",
            condition=EdgeCondition(type="always")
        ).model_dump()
    ]
    
    return ScenarioFile(
        metadata=ScenarioMetadata(
            name=name,
            version="1.0.0",
            author="User",
            description=f"Simple motion to {target_position}"
        ),
        robot=RobotSpec(
            required_dof=robot_dof,
            required_sensors=[]
        ),
        environment=EnvironmentSpec(),
        task_graph={"nodes": nodes, "edges": edges}
    )


if __name__ == "__main__":
    # Example: Create and save a simple scenario
    scenario = create_simple_motion_scenario(
        name="Test Motion",
        target_position=[0.5, 0.3, 0.2],
        robot_dof=4
    )
    
    print("Generated Scenario Schema:")
    print(scenario.to_json())
    
    # Save to file
    scenario.save("data/scenarios/test_motion.json")
    print("\nSaved to data/scenarios/test_motion.json")

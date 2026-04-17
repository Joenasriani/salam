# RoboNet Behavior Protocol - Platform Ecosystem

A complete robotics behavior platform with visual editor, execution engine, and shareable scenario packs.

## What Was Built

### Core Components

1. **RBP Schema (`core/schema.py`)**
   - Complete JSON specification for robot behaviors
   - Task nodes: trigger, motion_plan, manipulation, perception, condition, wait, action
   - Directed graph edges with conditions (on_success, on_failure, timeout)
   - Robot/environment specifications
   - Telemetry configuration

2. **Scenario Runner (`core/runner.py`)**
   - Executes RBP scenarios in PyBullet
   - State machine for node execution
   - Real-time telemetry collection
   - Support for all node types

3. **WebSocket Server (`server/websocket_server.py`)**
   - Real-time bidirectional communication
   - Scenario validation
   - Live execution streaming
   - Multi-client support

4. **Visual Editor (`frontend/editor.html`)**
   - React Flow-based node editor
   - Drag-and-drop node creation
   - Property panel for configuration
   - Export to RBP JSON
   - Live connection status and logs

## File Structure

```
robosim_studio/
├── core/
│   ├── schema.py      # RBP JSON schema definitions
│   ├── runner.py      # Scenario execution engine
│   └── kinematics.py  # Robot kinematics (existing)
├── server/
│   └── websocket_server.py  # Real-time communication
├── frontend/
│   └── editor.html    # Visual node editor
├── data/
│   └── scenarios/     # Saved scenario packs
│       └── test_motion.json
└── main.py            # Main application entry
```

## Quick Start

### 1. Install Dependencies

```bash
pip install pydantic websockets pybullet numpy
```

### 2. Start the WebSocket Server

```bash
cd /workspace/robosim_studio
python server/websocket_server.py
```

Server runs on `ws://localhost:8765`

### 3. Open the Visual Editor

Open `frontend/editor.html` in a browser (or serve it):

```bash
python -m http.server 8080 --directory frontend
# Then open http://localhost:8080/editor.html
```

### 4. Create Your First Scenario

1. Drag nodes from sidebar (Trigger, Motion Plan, etc.)
2. Connect nodes by dragging from output to input handles
3. Configure properties in the right panel
4. Click "Validate" to check your scenario
5. Click "Run" to execute in simulation
6. Click "Export" to save as `.json` file

## RBP Schema Example

```json
{
  "metadata": {
    "name": "Pick and Place",
    "version": "1.0.0",
    "author": "You"
  },
  "robot": { "required_dof": 4, "required_sensors": [] },
  "environment": { "gravity": [0, 0, -9.81] },
  "task_graph": {
    "nodes": [
      { "id": "start", "name": "Start", "type": "trigger" },
      { "id": "move", "name": "Move to Object", "type": "motion_plan", 
        "config": { "params": { "target_position": [0.5, 0.3, 0.2] } } }
    ],
    "edges": [
      { "source": "start", "target": "move", "condition": { "type": "always" } }
    ]
  }
}
```

## Node Types

| Type | Icon | Description |
|------|------|-------------|
| Trigger | ⚡ | Starts execution or waits for event |
| Motion Plan | 🎯 | Plans and executes robot motion |
| Manipulation | 🤖 | Gripper control, grasping |
| Perception | 👁️ | Sensor processing, object detection |
| Condition | ❓ | Branching logic based on state |
| Wait | ⏱️ | Time delay |
| Action | 🔧 | Custom actions |

## API Reference

### WebSocket Messages

**Client → Server:**
- `validate_scenario` - Validate scenario JSON
- `load_scenario` - Load scenario for execution
- `start_execution` - Begin execution
- `stop_execution` - Stop running scenario
- `get_status` - Get current execution status

**Server → Client:**
- `connected` - Connection established
- `validation_result` - Validation success/failure
- `scenario_loaded` - Scenario loaded successfully
- `execution_started` - Execution began
- `execution_complete` - All nodes completed
- `telemetry` - Real-time execution data
- `error` - Error message

## Use Cases

### Education
- Teach robotics concepts visually
- Share lesson plans as scenario packs
- Student assignments with validation

### Prototyping
- Rapid behavior iteration
- Test before deploying to real robots
- Collaborative development

### Research
- Reproducible experiments
- Share algorithms as scenarios
- Benchmark comparisons

## Next Steps

1. **Add More Node Types**: perception, custom actions
2. **URDF Integration**: Load real robot models
3. **Physics Simulation**: Full PyBullet integration
4. **Marketplace**: Share and discover scenarios
5. **Version Control**: Git-like scenario versioning
6. **Cloud Execution**: Run scenarios on remote robots

## License

MIT License

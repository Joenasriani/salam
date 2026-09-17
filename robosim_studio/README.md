# RoboNet Behavior Protocol / RoboSim Studio

RoboNet Behavior Protocol (RBP) defines a structured JSON representation for robot-behavior scenarios. RoboSim Studio provides a browser editor and local execution path for composing and running those scenarios in simulation.

## Current components

### RBP schema — `core/schema.py`

Defines:

- task nodes such as trigger, motion plan, manipulation, perception, condition, wait, action, and custom;
- directed graph edges and transition conditions;
- robot requirements;
- simulation environment configuration;
- validation-rule structures;
- telemetry configuration;
- scenario metadata and serialization models.

### Scenario runner — `core/runner.py`

Maintains execution state, dispatches implemented node types, advances through directed edges, initializes the simulation layer, and records execution/telemetry state.

The current executor table includes trigger, motion plan, manipulation, wait, condition, and action nodes.

### Browser editor — `frontend/index.html`

Provides:

- React Flow node composition;
- directed node connections;
- node-property editing;
- RBP JSON export;
- Validate, Load, Run, and Stop controls;
- execution logs and connection state.

The editor currently connects to `ws://localhost:8765`; it therefore expects the local WebSocket service to be running.

### WebSocket service — `server/`

Provides the communication layer between the browser editor and local validation/execution code.

### Simulation and kinematics

`simulation/` and `core/kinematics.py` contain the physics/simulation and robot-motion support used by the local execution path.

## Scenario flow

**behavior graph → RBP scenario JSON → validation/load request → runner execution → simulation state → telemetry/log stream**

## Current implementation boundaries

The current repository does not implement every capability represented by its schema or earlier roadmap text.

- The schema includes `PERCEPTION`, but the runner currently has no perception executor in its node-dispatch table.
- Custom transition expressions are not evaluated; the custom-condition path is currently treated as satisfied.
- Primitive environment-asset loading contains a placeholder.
- The editor targets a local WebSocket service rather than a hosted execution backend.
- Remote-robot execution is not implemented here.
- A scenario marketplace is not implemented here.
- Git-style scenario versioning is not implemented here.

These boundaries distinguish the code that exists from possible extensions.

## Local setup

Install the Python dependencies required by the codebase, including Pydantic, websockets, PyBullet, and NumPy.

Start the WebSocket service from the project directory, then serve or open the browser editor from `frontend/`.

The exact commands and project paths should be taken from the repository structure in the environment where the project is run rather than from historical `/workspace/...` paths.

## Repository identity

Use **RoboNet Behavior Protocol (RBP)** for the behavior-scenario format and **RoboSim Studio** for the editor/simulation codebase in this repository. Avoid presenting unrelated deployments or generic “digital fabrication” descriptions as the identity of this codebase.

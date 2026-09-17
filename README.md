# RoboNet Behavior Protocol / RoboSim Studio

This repository contains a robotics behavior-authoring and simulation codebase centered on the **RoboNet Behavior Protocol (RBP)** and **RoboSim Studio**.

The implementation combines a typed behavior-graph schema, a browser node editor, a WebSocket execution service, a scenario runner, robot kinematics, and PyBullet-based simulation components.

## Implemented structure

- `robosim_studio/core/schema.py` — typed RBP scenario, robot, environment, task-node, edge, trigger, validation, and telemetry models.
- `robosim_studio/core/runner.py` — graph execution state and node dispatch for simulated scenarios.
- `robosim_studio/core/kinematics.py` — robot kinematics utilities.
- `robosim_studio/server/` — WebSocket communication between the editor and execution layer.
- `robosim_studio/frontend/index.html` — browser behavior-graph editor built with React Flow.
- `robosim_studio/simulation/` — physics/simulation components.
- `robosim_studio/data/` — scenario data.

## Interaction model

**compose behavior nodes → connect directed transitions → configure robot/action parameters → export RBP JSON → validate/load scenario → execute through the local simulation service → stream execution state and telemetry**

The browser editor currently connects to a local WebSocket service at `ws://localhost:8765`.

## Current implementation boundaries

The source should not be read as a finished robotics deployment platform.

- `PERCEPTION` exists in the schema, but the current runner dispatch table does not expose a perception executor.
- custom edge-condition evaluation is not implemented; the current runner treats the custom-condition branch as satisfied.
- primitive environment-asset loading contains an implementation placeholder.
- the browser editor is configured for a local execution server rather than a hosted execution backend.
- marketplace, cloud execution, real-robot deployment, and Git-style scenario versioning are not implemented in the current repository.

## Public deployment

No canonical live deployment of this robotics system is established by the repository contents. Public URLs should be verified against the RoboNet/RoboSim interface before being presented as this project's deployment.

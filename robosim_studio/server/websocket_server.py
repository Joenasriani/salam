"""
RoboNet WebSocket Server - Real-time communication between editor and runner.

Provides bidirectional streaming of:
- Scenario JSON from frontend to backend
- Execution state from backend to frontend
- Telemetry data for live visualization
"""

import asyncio
import json
from typing import Dict, Optional, Set
from datetime import datetime

try:
    import websockets
    from websockets.server import serve
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    print("Warning: websockets not installed. Run: pip install websockets")

from core.schema import ScenarioFile
from core.runner import ScenarioRunner


class ConnectionState:
    """Tracks state for a connected client."""
    
    def __init__(self, websocket):
        self.websocket = websocket
        self.scenario: Optional[ScenarioFile] = None
        self.runner: Optional[ScenarioRunner] = None
        self.is_running = False
        self.client_id = datetime.now().strftime("%Y%m%d_%H%M%S")


class RoboNetServer:
    """
    WebSocket server for RoboNet Behavior Protocol.
    
    Handles:
    - Client connections
    - Scenario validation
    - Execution control
    - Real-time telemetry streaming
    """
    
    def __init__(self, host: str = "localhost", port: int = 8765):
        self.host = host
        self.port = port
        self.clients: Set[ConnectionState] = set()
        self.running_tasks: Dict[str, asyncio.Task] = {}
    
    async def handle_client(self, websocket):
        """Handle a single client connection."""
        state = ConnectionState(websocket)
        self.clients.add(state)
        
        print(f"Client connected: {state.client_id}")
        
        # Send welcome message
        await self.send_message(state, {
            "type": "connected",
            "client_id": state.client_id,
            "message": "Connected to RoboNet Server"
        })
        
        try:
            async for message in websocket:
                await self.process_message(state, message)
        except websockets.exceptions.ConnectionClosed:
            print(f"Client disconnected: {state.client_id}")
        finally:
            self.clients.discard(state)
            if state.client_id in self.running_tasks:
                self.running_tasks[state.client_id].cancel()
    
    async def process_message(self, state: ConnectionState, message: str):
        """Process incoming message from client."""
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            
            if msg_type == "validate_scenario":
                await self.handle_validate(state, data)
            
            elif msg_type == "load_scenario":
                await self.handle_load_scenario(state, data)
            
            elif msg_type == "start_execution":
                await self.handle_start_execution(state, data)
            
            elif msg_type == "stop_execution":
                await self.handle_stop_execution(state)
            
            elif msg_type == "get_status":
                await self.handle_get_status(state)
            
            elif msg_type == "ping":
                await self.send_message(state, {"type": "pong"})
            
            else:
                await self.send_message(state, {
                    "type": "error",
                    "message": f"Unknown message type: {msg_type}"
                })
        
        except json.JSONDecodeError:
            await self.send_message(state, {
                "type": "error",
                "message": "Invalid JSON"
            })
        except Exception as e:
            await self.send_message(state, {
                "type": "error",
                "message": str(e)
            })
    
    async def handle_validate(self, state: ConnectionState, data: Dict):
        """Validate a scenario JSON."""
        try:
            scenario_json = data.get("scenario", "")
            scenario = ScenarioFile.from_json(scenario_json)
            
            # Basic validation
            nodes_count = len(scenario.task_graph['nodes'])
            edges_count = len(scenario.task_graph['edges'])
            
            await self.send_message(state, {
                "type": "validation_result",
                "valid": True,
                "nodes": nodes_count,
                "edges": edges_count,
                "message": "Scenario is valid"
            })
        
        except Exception as e:
            await self.send_message(state, {
                "type": "validation_result",
                "valid": False,
                "error": str(e)
            })
    
    async def handle_load_scenario(self, state: ConnectionState, data: Dict):
        """Load a scenario for execution."""
        try:
            scenario_json = data.get("scenario", "")
            state.scenario = ScenarioFile.from_json(scenario_json)
            
            await self.send_message(state, {
                "type": "scenario_loaded",
                "name": state.scenario.metadata.name,
                "version": state.scenario.metadata.version,
                "nodes": len(state.scenario.task_graph['nodes']),
                "message": "Scenario loaded successfully"
            })
        
        except Exception as e:
            await self.send_message(state, {
                "type": "error",
                "message": f"Failed to load scenario: {e}"
            })
    
    async def handle_start_execution(self, state: ConnectionState, data: Dict):
        """Start scenario execution."""
        if not state.scenario:
            await self.send_message(state, {
                "type": "error",
                "message": "No scenario loaded. Load a scenario first."
            })
            return
        
        if state.is_running:
            await self.send_message(state, {
                "type": "error",
                "message": "Execution already running"
            })
            return
        
        state.is_running = True
        
        # Create runner and start execution in background task
        state.runner = ScenarioRunner(scenario_data=state.scenario)
        
        # Start execution task
        task = asyncio.create_task(self.run_execution(state))
        self.running_tasks[state.client_id] = task
        
        await self.send_message(state, {
            "type": "execution_started",
            "timestamp": datetime.now().isoformat()
        })
    
    async def run_execution(self, state: ConnectionState):
        """Run scenario execution and stream telemetry."""
        try:
            # Initialize without GUI for server mode
            state.runner.physics_engine.gui = False
            state.runner.initialize()
            
            # Custom execution loop with telemetry streaming
            state.runner.is_running = True
            state.runner.state.start_time = asyncio.get_event_loop().time()
            
            # Find start node
            start_node = None
            for node_data in state.runner.scenario.task_graph['nodes']:
                if node_data['type'] == 'trigger':
                    start_node = node_data['id']
                    break
            
            if not start_node:
                await self.send_message(state, {
                    "type": "execution_error",
                    "message": "No trigger node found"
                })
                return
            
            state.runner.state.current_node_id = start_node
            step_count = 0
            
            while state.runner.is_running and step_count < 500:
                # Execute current node (simplified for async)
                success = await self.execute_node_async(state)
                
                # Update status
                current_id = state.runner.state.current_node_id
                status = state.runner.state.node_statuses.get(current_id, "unknown")
                
                # Stream telemetry
                await self.send_message(state, {
                    "type": "telemetry",
                    "step": step_count,
                    "current_node": current_id,
                    "node_status": status.value if hasattr(status, 'value') else status,
                    "completed": len(state.runner.state.completed_nodes),
                    "elapsed": asyncio.get_event_loop().time() - state.runner.state.start_time
                })
                
                # Check for completion
                if not state.runner.state.find_next_node(current_id):
                    await self.send_message(state, {
                        "type": "execution_complete",
                        "completed_nodes": state.runner.state.completed_nodes,
                        "total_steps": step_count
                    })
                    break
                
                step_count += 1
                await asyncio.sleep(0.1)  # Simulate time step
            
        except Exception as e:
            await self.send_message(state, {
                "type": "execution_error",
                "message": str(e)
            })
        finally:
            state.is_running = False
            if state.client_id in self.running_tasks:
                del self.running_tasks[state.client_id]
    
    async def execute_node_async(self, state: ConnectionState) -> bool:
        """Execute current node asynchronously."""
        from core.runner import NodeStatus
        
        node_id = state.runner.state.current_node_id
        node_data = state.runner.state.get_node(node_id)
        
        if not node_data:
            return False
        
        # Simulate node execution
        node_type = node_data['type']
        config = node_data.get('config', {}).get('params', {})
        
        # Simulate different node types
        if node_type == 'trigger':
            delay = node_data.get('trigger_params', {}).get('delay', 0)
            if delay:
                await asyncio.sleep(delay)
        
        elif node_type == 'motion_plan':
            # Simulate motion
            await asyncio.sleep(0.5)
        
        elif node_type == 'wait':
            duration = config.get('duration', 1.0)
            await asyncio.sleep(min(duration, 2.0))  # Cap at 2s
        
        elif node_type == 'manipulation':
            await asyncio.sleep(0.3)
        
        # Mark as success
        state.runner.state.node_statuses[node_id] = NodeStatus.SUCCESS
        state.runner.state.completed_nodes.append(node_id)
        
        # Find next node
        next_node = state.runner.state.find_next_node(node_id)
        if next_node:
            state.runner.state.current_node_id = next_node
            state.runner.state.node_statuses[next_node] = NodeStatus.PENDING
        
        return True
    
    async def handle_stop_execution(self, state: ConnectionState):
        """Stop scenario execution."""
        if state.runner:
            state.runner.is_running = False
            state.is_running = False
        
        await self.send_message(state, {
            "type": "execution_stopped",
            "timestamp": datetime.now().isoformat()
        })
    
    async def handle_get_status(self, state: ConnectionState):
        """Get current execution status."""
        if state.runner:
            status = state.runner.get_status()
        else:
            status = {"is_running": False}
        
        await self.send_message(state, {
            "type": "status",
            **status
        })
    
    async def send_message(self, state: ConnectionState, data: Dict):
        """Send message to client."""
        try:
            await state.websocket.send(json.dumps(data))
        except Exception as e:
            print(f"Error sending message: {e}")
    
    async def broadcast(self, data: Dict):
        """Broadcast message to all clients."""
        message = json.dumps(data)
        for state in self.clients:
            try:
                await state.websocket.send(message)
            except:
                pass
    
    async def run(self):
        """Start the WebSocket server."""
        if not WEBSOCKETS_AVAILABLE:
            print("Cannot start server: websockets library not installed")
            print("Install with: pip install websockets")
            return
        
        print(f"Starting RoboNet Server on ws://{self.host}:{self.port}")
        
        async with serve(self.handle_client, self.host, self.port):
            await asyncio.Future()  # Run forever


async def main():
    """Run the server."""
    server = RoboNetServer(host="0.0.0.0", port=8765)
    await server.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped")

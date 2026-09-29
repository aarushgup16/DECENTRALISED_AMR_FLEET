"""Passive Fleet Monitoring Dashboard server (FastAPI + WebSocket). Zero SPOF.

Refactored for High-Frequency, Non-Blocking Asynchronous Telemetry Pipeline.
Decouples simulation physics tick from WebSocket network broadcasting with
frame-dropping under backpressure and MessagePack / Packed-JSON serialization.
"""

import asyncio
from contextlib import asynccontextmanager
import json
import os
from typing import Any, Dict, List, Optional, Set, Tuple
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
import msgpack
from pydantic import BaseModel

from decentralized_amr.simulation.fleet_coordinator import FleetCoordinator
from decentralized_amr.simulation.baseline_runner import BaselineSimulator
from decentralized_amr.simulation.warehouse_map import WarehouseMap
from decentralized_amr.task_allocation.task import WarehouseTask


class TaskCreateRequest(BaseModel):
    pickup_x: float
    pickup_y: float
    dropoff_x: float
    dropoff_y: float
    urgency: int = 1

class ObstacleCreateRequest(BaseModel):
    obstacle_id: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float

class SimControlRequest(BaseModel):
    action: str
    value: Optional[float] = None

class ChaosSeverLinkRequest(BaseModel):
    node_a: int
    node_b: int

class ChaosPacketLossRequest(BaseModel):
    rate: float

class ChaosKillNodeRequest(BaseModel):
    node_id: int
    action: str = "kill"


# Global simulation state & decoupled telemetry pipeline
coordinator: Optional[FleetCoordinator] = None
is_paused: bool = False
speed_multiplier: float = 1.0
sim_task: Optional[asyncio.Task] = None
broadcaster_task: Optional[asyncio.Task] = None

# Asynchronous Telemetry Queue (maxsize protects memory under backpressure)
telemetry_queue: asyncio.Queue = asyncio.Queue(maxsize=15)
active_connections: List[WebSocket] = []
client_formats: Dict[WebSocket, str] = {}

STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")


@asynccontextmanager
async def lifespan(app: FastAPI):
    global coordinator, sim_task, broadcaster_task
    coordinator = FleetCoordinator(num_robots=4)
    # Inject initial demo task set
    tasks = coordinator.map.generate_random_tasks(num_tasks=12, seed=42)
    coordinator.inject_tasks(tasks)

    # Spawn decoupled producer and consumer tasks
    sim_task = asyncio.create_task(simulation_background_loop())
    broadcaster_task = asyncio.create_task(websocket_broadcaster_loop())
    
    yield
    
    if sim_task:
        sim_task.cancel()
    if broadcaster_task:
        broadcaster_task.cancel()


app = FastAPI(title="Decentralized AMR Fleet Dashboard", version="2.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


async def simulation_background_loop():
    """
    Simulation Physics & Coordination Loop.
    Steps autonomous AMRs and pushes state snapshots to the async queue.
    If the queue is full (network backpressure / slow clients), the oldest frame
    is dropped to ensure the physics simulation is never blocked.
    """
    global coordinator, is_paused, speed_multiplier, telemetry_queue
    while True:
        try:
            if coordinator and not is_paused:
                steps_to_run = max(1, int(speed_multiplier))
                for _ in range(steps_to_run):
                    coordinator.step()

                # Generate high-frequency packed telemetry packet
                packet = coordinator.get_telemetry_packet()

                # Non-blocking enqueue with oldest-frame eviction
                if telemetry_queue.full():
                    try:
                        telemetry_queue.get_nowait()
                        telemetry_queue.task_done()
                    except (asyncio.QueueEmpty, ValueError):
                        pass

                try:
                    telemetry_queue.put_nowait(packet)
                except asyncio.QueueFull:
                    pass

            await asyncio.sleep(0.05 / max(1.0, min(speed_multiplier, 10.0)))
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"[Simulation Loop Error] {e}")
            await asyncio.sleep(0.05)


async def websocket_broadcaster_loop():
    """
    Dedicated WebSocket Broadcaster Loop.
    Consumes from telemetry queue at a fixed 30 Hz rate, pre-serializes
    once with MessagePack or packed JSON, and broadcasts asynchronously.
    """
    global telemetry_queue, active_connections, client_formats
    target_interval = 1.0 / 30.0  # 30 Hz broadcast rate
    while True:
        try:
            # Drain queue to immediately grab the latest available frame
            latest_packet = None
            while not telemetry_queue.empty():
                try:
                    latest_packet = telemetry_queue.get_nowait()
                    telemetry_queue.task_done()
                except asyncio.QueueEmpty:
                    break

            if latest_packet is not None and active_connections:
                json_bytes: Optional[str] = None
                msgpack_bytes: Optional[bytes] = None
                disconnected = []

                for ws in list(active_connections):
                    fmt = client_formats.get(ws, "json")
                    try:
                        if fmt == "msgpack":
                            if msgpack_bytes is None:
                                msgpack_bytes = msgpack.packb(latest_packet, use_bin_type=True)
                            await ws.send_bytes(msgpack_bytes)
                        else:
                            if json_bytes is None:
                                json_bytes = json.dumps(latest_packet)
                            await ws.send_text(json_bytes)
                    except Exception:
                        disconnected.append(ws)

                for ws in disconnected:
                    if ws in active_connections:
                        active_connections.remove(ws)
                    client_formats.pop(ws, None)

            await asyncio.sleep(target_interval)
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"[Broadcaster Loop Error] {e}")
            await asyncio.sleep(0.05)


class TaskCreateRequest(BaseModel):
    pickup_x: float
    pickup_y: float
    dropoff_x: float
    dropoff_y: float
    urgency: int = 1


class ObstacleCreateRequest(BaseModel):
    obstacle_id: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float


class SimControlRequest(BaseModel):
    action: str  # "play", "pause", "reset", "speed"
    value: Optional[float] = None


@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket, format: str = Query("json")):
    """
    Passive, read-only WebSocket connection for high-frequency fleet monitoring.
    Supports query parameter `format=msgpack` or `format=json`.
    """
    await websocket.accept()
    active_connections.append(websocket)
    client_formats[websocket] = format.lower()

    try:
        # Send initial snapshot immediately upon connect
        if coordinator:
            init_packet = coordinator.get_telemetry_packet()
            if format.lower() == "msgpack":
                await websocket.send_bytes(msgpack.packb(init_packet, use_bin_type=True))
            else:
                await websocket.send_text(json.dumps(init_packet))

        while True:
            # Keepalive listening loop
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        if websocket in active_connections:
            active_connections.remove(websocket)
        client_formats.pop(websocket, None)


@app.get("/api/status")
async def get_status():
    if not coordinator:
        return {"status": "uninitialized"}
    return coordinator.get_snapshot()


@app.post("/api/task")
async def create_task(req: TaskCreateRequest):
    if not coordinator:
        return {"error": "uninitialized"}
    task_id = f"TASK-{len(coordinator.task_pool)+1:03d}"
    new_task = WarehouseTask(
        task_id=task_id,
        pickup_pos=(req.pickup_x, req.pickup_y),
        dropoff_pos=(req.dropoff_x, req.dropoff_y),
        urgency=req.urgency,
        base_reward=100.0 + req.urgency * 25.0
    )
    coordinator.add_task(new_task)
    return {"status": "created", "task": task_id}


@app.post("/api/obstacle")
async def create_obstacle(req: ObstacleCreateRequest):
    if not coordinator:
        return {"error": "uninitialized"}
    coordinator.inject_blocked_aisle(
        req.obstacle_id,
        (req.x_min, req.y_min, req.x_max, req.y_max)
    )
    return {"status": "obstacle_injected", "id": req.obstacle_id}


@app.delete("/api/obstacle/{obs_id}")
async def delete_obstacle(obs_id: str):
    if not coordinator:
        return {"error": "uninitialized"}
    coordinator.map.remove_dynamic_obstacle(obs_id)
    for r in coordinator.robots:
        r.planner.remove_dynamic_obstacle(obs_id)
    return {"status": "obstacle_removed", "id": obs_id}


@app.post("/api/sim/control")
async def control_sim(req: SimControlRequest):
    global is_paused, speed_multiplier, coordinator
    if req.action == "pause":
        is_paused = True
    elif req.action == "play":
        is_paused = False
    elif req.action == "speed" and req.value is not None:
        speed_multiplier = max(0.5, min(10.0, req.value))
    elif req.action == "reset":
        coordinator = FleetCoordinator(num_robots=4)
        tasks = coordinator.map.generate_random_tasks(num_tasks=12, seed=42)
        coordinator.inject_tasks(tasks)
        is_paused = False
    return {"status": "ok", "paused": is_paused, "speed": speed_multiplier}


@app.post("/api/chaos/sever_link")
async def sever_mesh_link(req: ChaosSeverLinkRequest):
    if not coordinator:
        return {"error": "uninitialized"}
    coordinator.sever_mesh_link(req.node_a, req.node_b)
    return {"status": "link_severed", "node_a": req.node_a, "node_b": req.node_b}


@app.post("/api/chaos/heal_link")
async def heal_mesh_link(req: ChaosSeverLinkRequest):
    if not coordinator:
        return {"error": "uninitialized"}
    coordinator.heal_mesh_link(req.node_a, req.node_b)
    return {"status": "link_healed", "node_a": req.node_a, "node_b": req.node_b}


@app.post("/api/chaos/heal_all")
async def heal_all_links():
    if not coordinator:
        return {"error": "uninitialized"}
    coordinator.heal_all_mesh_links()
    return {"status": "all_links_healed"}


@app.post("/api/chaos/packet_loss")
async def set_packet_loss(req: ChaosPacketLossRequest):
    if not coordinator:
        return {"error": "uninitialized"}
    coordinator.set_packet_loss_spike(req.rate)
    return {"status": "packet_loss_updated", "rate": req.rate}


@app.post("/api/chaos/kill_node")
async def kill_node(req: ChaosKillNodeRequest):
    if not coordinator:
        return {"error": "uninitialized"}
    if req.action == "kill":
        coordinator.kill_node(req.node_id)
    else:
        coordinator.revive_node(req.node_id)
    return {"status": f"node_{req.action}ed", "node_id": req.node_id}


@app.post("/api/benchmark")
async def run_benchmark_comparison(num_robots: int = 4, num_tasks: int = 12):
    """Run head-to-head comparison between Baseline and Solution."""
    # 1. Generate identical tasks
    ref_map = WarehouseMap()
    tasks_solution = ref_map.generate_random_tasks(num_tasks=num_tasks, seed=99)
    tasks_baseline = ref_map.generate_random_tasks(num_tasks=num_tasks, seed=99)

    # 2. Run Baseline
    baseline_sim = BaselineSimulator(num_robots=num_robots)
    baseline_sim.inject_tasks(tasks_baseline)
    baseline_metrics = baseline_sim.run_until_complete(max_seconds=200.0)

    # 3. Run Solution
    sol_coord = FleetCoordinator(num_robots=num_robots)
    sol_coord.inject_tasks(tasks_solution)
    while sol_coord.sim_time < 200.0:
        still_running = sol_coord.step()
        if not still_running:
            break
    solution_metrics = sol_coord.get_metrics_summary()

    # 4. Compare
    t_base = max(0.1, baseline_metrics.total_time_seconds)
    t_sol = max(0.1, solution_metrics.total_time_seconds)
    time_reduction_pct = round(((t_base - t_sol) / t_base) * 100.0, 2)

    return {
        "baseline": {
            "total_time": baseline_metrics.total_time_seconds,
            "collisions": baseline_metrics.collision_count,
            "tasks_completed": baseline_metrics.tasks_completed,
            "throughput": baseline_metrics.throughput_tasks_per_min,
            "distance": baseline_metrics.total_fleet_distance_m,
            "battery_used": baseline_metrics.total_battery_consumed_pct
        },
        "solution": {
            "total_time": solution_metrics.total_time_seconds,
            "collisions": solution_metrics.collision_count,
            "tasks_completed": solution_metrics.tasks_completed,
            "throughput": solution_metrics.throughput_tasks_per_min,
            "distance": solution_metrics.total_fleet_distance_m,
            "battery_used": solution_metrics.total_battery_consumed_pct,
            "priority_tie_breaks": solution_metrics.priority_tie_breaks
        },
        "comparison": {
            "time_reduction_percentage": time_reduction_pct,
            "meets_speed_criteria": time_reduction_pct >= 20.0,
            "meets_zero_collision_criteria": solution_metrics.collision_count == 0
        }
    }


# Mount Static Files and Root Route
if os.path.exists(STATIC_DIR):
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
async def get_index():
    index_path = os.path.join(STATIC_DIR, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return HTMLResponse("<h1>Decentralized AMR Fleet Server</h1><p>Static files loading...</p>")


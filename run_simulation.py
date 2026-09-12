"""Main simulation runner for Decentralized AMR Fleet Coordination & Collision Avoidance."""

import argparse
import asyncio
import sys
import time
import uvicorn

from decentralized_amr.simulation.fleet_coordinator import FleetCoordinator
from decentralized_amr.simulation.warehouse_map import WarehouseMap
from decentralized_amr.network.network_chaos import NetworkChaosSimulator


def run_cli_simulation(num_robots: int = 4, num_tasks: int = 12, enable_chaos: bool = False):
    """Run headless simulation in terminal with live text telemetry."""
    print("=" * 75)
    print("  DECENTRALIZED AMR FLEET COORDINATION — HEADLESS SIMULATION")
    print("  Core: NH-ORCA + CBBA + Lamport Logical Clocks + Bounded Priority")
    print("=" * 75)

    coordinator = FleetCoordinator(num_robots=num_robots)
    if enable_chaos:
        print("[Network Chaos] Active: 5% packet loss + [50ms, 150ms] latency jitter + Wi-Fi dead-zone.")
        coordinator.mesh.chaos = NetworkChaosSimulator(
            packet_loss_rate=0.05,
            min_latency_ms=50.0,
            max_latency_ms=150.0,
            dead_zones=[(10.0, 5.0, 13.0, 8.0)]
        )

    tasks = coordinator.map.generate_random_tasks(num_tasks=num_tasks, seed=42)
    coordinator.inject_tasks(tasks)
    print(f"Spawned {num_robots} AMRs across P2P Mesh.")
    print(f"Injected {num_tasks} warehouse transport jobs into decentralized task pool.\n")

    start_wall = time.time()
    last_print = 0.0

    while True:
        still_running = coordinator.step()
        
        if coordinator.sim_time - last_print >= 2.0 or not still_running:
            last_print = coordinator.sim_time
            m = coordinator.metrics
            tasks_done = len(m.task_end_times)
            
            states_str = " | ".join(
                f"AMR-{r.robot_id}: {r.state.value[:4]} (L:{r.clock.get_time()}, Bat:{int(r.battery)}%)"
                for r in coordinator.robots
            )
            print(f"[T={coordinator.sim_time:5.1f}s] Tasks: {tasks_done}/{num_tasks} | Collisions: {len(m.collision_events)} | {states_str}")

        if not still_running:
            break
        
        # Accelerated tick rate for CLI
        time.sleep(0.002)

    wall_duration = time.time() - start_wall
    summary = coordinator.get_metrics_summary()

    print("\n" + "=" * 75)
    print("  SIMULATION COMPLETED SUCCESSFULLY")
    print("=" * 75)
    print(f"  • Total Simulated Time    : {summary.total_time_seconds:.2f} s (Wall clock: {wall_duration:.2f} s)")
    print(f"  • Tasks Completed         : {summary.tasks_completed} / {summary.total_tasks}")
    print(f"  • Inter-Robot Collisions  : {summary.collision_count} (Zero-collision criteria: {'PASSED ✓' if summary.collision_count == 0 else 'FAILED ✗'})")
    print(f"  • Fleet Throughput        : {summary.throughput_tasks_per_min:.2f} tasks/min")
    print(f"  • Fleet Distance Traveled : {summary.total_fleet_distance_m:.2f} m")
    print(f"  • Priority Tie-Breaks Won : {summary.priority_tie_breaks}")
    print("=" * 75 + "\n")


def run_dashboard_server(host: str = "127.0.0.1", port: int = 8080):
    """Launch FastAPI + WebSocket passive dashboard server."""
    import socket
    target_port = port
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        if s.connect_ex((host, target_port)) == 0:
            # Port is occupied, find next available port
            for candidate in range(target_port + 1, target_port + 20):
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
                    if s2.connect_ex((host, candidate)) != 0:
                        print(f"[Port Warning] Port {target_port} is currently busy. Auto-switching to port {candidate}.")
                        target_port = candidate
                        break

    print(f"\n==========================================================================")
    print(f"  PASSIVE FLEET MONITORING DASHBOARD (Zero SPOF)")
    print(f"  Open in Browser -> http://{host}:{target_port}")
    print(f"==========================================================================\n")
    uvicorn.run("decentralized_amr.dashboard.server:app", host=host, port=target_port, log_level="info")


def main():
    parser = argparse.ArgumentParser(description="Decentralized AMR Fleet Coordinator")
    parser.add_argument("--headless", action="store_true", help="Run in headless terminal mode with live text telemetry")
    parser.add_argument("--dashboard", action="store_true", help="Launch interactive web dashboard (default)")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Dashboard host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080, help="Dashboard port (default: 8080)")
    parser.add_argument("--robots", type=int, default=4, help="Number of AMRs in fleet (default: 4)")
    parser.add_argument("--tasks", type=int, default=12, help="Number of warehouse tasks (default: 12)")
    parser.add_argument("--chaos", action="store_true", help="Enable network degradation and dead-zones")

    args, _ = parser.parse_known_args()

    if args.headless:
        run_cli_simulation(num_robots=args.robots, num_tasks=args.tasks, enable_chaos=args.chaos)
    else:
        run_dashboard_server(host=args.host, port=args.port)


if __name__ == "__main__":
    main()


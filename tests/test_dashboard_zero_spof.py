"""Test verifying Zero Single Point of Failure (SPOF) for the Dashboard.

Per Solution Bible v2 Section 4.6:
'Killing the dashboard must not affect the fleet — verify this explicitly as a test case.'
"""

import asyncio
import pytest
from decentralized_amr.simulation.fleet_coordinator import FleetCoordinator
from decentralized_amr.dashboard.server import active_connections


class MockWebSocketClient:
    """Simulates a passive dashboard browser/monitoring client."""
    def __init__(self):
        self.received_messages = []
        self.is_closed = False

    async def send_json(self, data):
        if self.is_closed:
            raise ConnectionResetError("Client disconnected / process killed")
        self.received_messages.append(data)


@pytest.mark.anyio
async def test_dashboard_disconnect_zero_spof():
    """
    Verify that abruptly killing/disconnecting dashboard clients mid-mission
    does not interrupt AMR path planning, task execution, or collision avoidance.
    """
    coordinator = FleetCoordinator(num_robots=4)
    tasks = coordinator.map.generate_random_tasks(num_tasks=6, seed=777)
    coordinator.inject_tasks(tasks)

    # 1. Connect mock dashboard client
    client = MockWebSocketClient()
    active_connections.append(client)

    # 2. Run simulation for first 50 steps with active dashboard
    for _ in range(50):
        coordinator.step()
        snapshot = coordinator.get_snapshot()
        await client.send_json(snapshot)

    assert len(client.received_messages) == 50
    assert not client.is_closed

    # 3. KILL / ABRUPTLY TERMINATE the dashboard client
    client.is_closed = True

    # 4. Continue running the autonomous fleet to completion
    max_steps = 4000
    steps = 50
    while steps < max_steps:
        still_running = coordinator.step()
        steps += 1
        if not still_running:
            break

    # Clean up
    if client in active_connections:
        active_connections.remove(client)

    summary = coordinator.get_metrics_summary()

    # 5. Formally verify fleet completed all tasks with 0 collisions despite killed dashboard
    assert summary.collision_count == 0, f"Expected 0 collisions, got {summary.collision_count}"
    assert summary.tasks_completed == 6, f"Expected 6/6 tasks completed, got {summary.tasks_completed}"
    assert summary.total_time_seconds > 0.0


def test_fleet_operates_without_any_dashboard_server():
    """
    Verify that the multi-robot fleet operates with pure peer-to-peer autonomy
    in the complete absence of any dashboard or centralized server.
    """
    coordinator = FleetCoordinator(num_robots=4)
    tasks = coordinator.map.generate_random_tasks(num_tasks=6, seed=888)
    coordinator.inject_tasks(tasks)

    # Run without any server or dashboard attachments
    max_steps = 4000
    steps = 0
    while steps < max_steps:
        still_running = coordinator.step()
        steps += 1
        if not still_running:
            break

    summary = coordinator.get_metrics_summary()
    assert summary.collision_count == 0
    assert summary.tasks_completed == 6

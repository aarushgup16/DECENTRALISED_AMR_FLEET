"""Unit tests for Phase 3: Visual Hierarchy & Custom Shaders Telemetry."""

from decentralized_amr.simulation.fleet_coordinator import FleetCoordinator


def test_phase3_telemetry_bidding_links_and_locks():
    """Verify get_telemetry_packet contains active bidding links and spatial locks for shaders."""
    coordinator = FleetCoordinator(num_robots=4)
    tasks = coordinator.map.generate_random_tasks(num_tasks=4, seed=404)
    coordinator.inject_tasks(tasks)

    # Step simulation
    for _ in range(5):
        coordinator.step()

    packet = coordinator.get_telemetry_packet()

    # Verify bidding_links list exists
    assert "bidding_links" in packet
    assert isinstance(packet["bidding_links"], list)
    if len(packet["bidding_links"]) > 0:
        b = packet["bidding_links"][0]
        assert "task_id" in b
        assert "robot_id" in b
        assert "task_x" in b and "task_y" in b
        assert "robot_x" in b and "robot_y" in b
        assert "bid_score" in b
        assert "is_winner" in b

    # Verify spatial_locks list exists
    assert "spatial_locks" in packet
    assert isinstance(packet["spatial_locks"], list)
    if len(packet["spatial_locks"]) > 0:
        lk = packet["spatial_locks"][0]
        assert "x" in lk and "y" in lk
        assert "duration_rem" in lk
        assert "duration_total" in lk
        assert "robot_id" in lk

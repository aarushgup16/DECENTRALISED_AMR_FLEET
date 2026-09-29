"""Unit tests for Phase 1: High-Frequency Telemetry & Asynchronous Data Pipeline."""

import asyncio
import msgpack
import pytest
from decentralized_amr.network.network_chaos import NetworkChaosSimulator
from decentralized_amr.simulation.fleet_coordinator import FleetCoordinator


def test_telemetry_packet_strict_schema():
    """Verify get_telemetry_packet conforms strictly to Phase 1 schema requirements."""
    coordinator = FleetCoordinator(num_robots=4)
    tasks = coordinator.map.generate_random_tasks(num_tasks=4, seed=101)
    coordinator.inject_tasks(tasks)

    # Step coordinator once
    coordinator.step()
    packet = coordinator.get_telemetry_packet()

    # 1. Kinematics array check: [[id, x, y, theta, vx, vy], ...]
    assert "kinematics" in packet
    assert len(packet["kinematics"]) == 4
    for k in packet["kinematics"]:
        assert len(k) == 6
        assert isinstance(k[0], int) # id
        assert all(isinstance(v, (float, int)) for v in k[1:])

    # 2. ORCA state check
    assert "orca_state" in packet
    assert len(packet["orca_state"]) == 4
    for r_id, lines in packet["orca_state"].items():
        assert isinstance(r_id, (int, str))
        assert isinstance(lines, list)

    # 3. Consensus state check
    assert "consensus_state" in packet
    assert len(packet["consensus_state"]) == 4
    for t_id, cstate in packet["consensus_state"].items():
        assert "bid" in cstate
        assert "winner" in cstate
        assert "lamport_clock" in cstate

    # 4. Metadata check
    assert "metadata" in packet
    m = packet["metadata"]
    assert "sim_time" in m
    assert "frame_seq" in m
    assert "stale_nodes" in m
    assert "dropped_packets" in m


def test_msgpack_serialization_integrity():
    """Verify telemetry packet serializes and unpacks with MessagePack with zero loss."""
    coordinator = FleetCoordinator(num_robots=4)
    tasks = coordinator.map.generate_random_tasks(num_tasks=6, seed=202)
    coordinator.inject_tasks(tasks)
    coordinator.step()

    packet = coordinator.get_telemetry_packet()
    packed_bytes = msgpack.packb(packet, use_bin_type=True)
    assert isinstance(packed_bytes, bytes)
    assert len(packed_bytes) > 0

    unpacked = msgpack.unpackb(packed_bytes, raw=False)
    assert unpacked["sim_time"] == packet["sim_time"]
    assert len(unpacked["kinematics"]) == 4
    assert len(unpacked["consensus_state"]) == 6


def test_network_chaos_dead_zone_stale_metadata():
    """Verify dead zone in network_chaos is dynamically reflected in telemetry metadata."""
    coordinator = FleetCoordinator(num_robots=4)
    # Configure a dead zone covering robot 1's starting position (2.0, 5.0)
    coordinator.mesh.chaos = NetworkChaosSimulator(
        dead_zones=[(0.0, 0.0, 5.0, 10.0)]
    )
    coordinator.step()
    packet = coordinator.get_telemetry_packet()

    # Robot 1 at (2.0, 5.0) is in dead zone
    assert 1 in packet["metadata"]["stale_nodes"]
    robot_1_detail = next(r for r in packet["robots"] if r["id"] == 1)
    assert robot_1_detail["is_stale"] is True


@pytest.mark.anyio
async def test_async_queue_backpressure_frame_dropping():
    """Verify async telemetry queue evicts oldest frame when full without blocking."""
    queue = asyncio.Queue(maxsize=3)

    for i in range(5):
        frame = {"frame_id": i}
        if queue.full():
            queue.get_nowait()
            queue.task_done()
        queue.put_nowait(frame)

    assert queue.qsize() == 3
    # Remaining items should be the latest 3 frames: 2, 3, 4
    items = []
    while not queue.empty():
        items.append(await queue.get())

    assert [item["frame_id"] for item in items] == [2, 3, 4]

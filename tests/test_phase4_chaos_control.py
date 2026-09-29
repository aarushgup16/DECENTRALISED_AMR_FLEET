"""Unit tests for Phase 4: Chaos Control Surface & Live State Injection."""

from decentralized_amr.network.network_chaos import NetworkChaosSimulator
from decentralized_amr.simulation.fleet_coordinator import FleetCoordinator


def test_sever_and_heal_mesh_link():
    """Verify live link severing and healing in NetworkChaosSimulator."""
    chaos = NetworkChaosSimulator()
    assert len(chaos.severed_links) == 0

    # Sever link between Node 1 and Node 2
    chaos.sever_link(1, 2)
    assert chaos.is_link_severed(1, 2) is True
    assert chaos.is_link_severed(2, 1) is True # Bidirectional
    assert chaos.is_link_severed(1, 3) is False

    # Should drop packets between 1 and 2
    assert chaos.should_drop(sender_id=1, receiver_id=2) is True
    assert chaos.should_drop(sender_id=2, receiver_id=1) is True
    assert chaos.should_drop(sender_id=1, receiver_id=3) is False

    # Heal link
    chaos.heal_link(1, 2)
    assert chaos.is_link_severed(1, 2) is False
    assert chaos.should_drop(sender_id=1, receiver_id=2) is False


def test_kill_and_revive_node():
    """Verify live node killing and reviving."""
    chaos = NetworkChaosSimulator()

    chaos.kill_node(3)
    assert 3 in chaos.killed_nodes
    assert chaos.should_drop(sender_id=3, receiver_id=1) is True
    assert chaos.should_drop(sender_id=1, receiver_id=3) is True
    assert chaos.should_drop(sender_id=1, receiver_id=2) is False

    chaos.revive_node(3)
    assert 3 not in chaos.killed_nodes
    assert chaos.should_drop(sender_id=3, receiver_id=1) is False


def test_packet_loss_spike():
    """Verify packet loss rate configuration."""
    chaos = NetworkChaosSimulator()
    chaos.set_packet_loss_spike(0.5)
    assert chaos.packet_loss_rate == 0.5

    chaos.set_packet_loss_spike(0.0)
    assert chaos.packet_loss_rate == 0.0


def test_fleet_coordinator_chaos_methods_and_telemetry():
    """Verify FleetCoordinator chaos methods propagate to telemetry packet."""
    coordinator = FleetCoordinator(num_robots=4)
    coordinator.sever_mesh_link(1, 3)
    coordinator.kill_node(4)
    coordinator.set_packet_loss_spike(0.5)

    packet = coordinator.get_telemetry_packet()
    assert "chaos_state" in packet
    cstate = packet["chaos_state"]
    assert [1, 3] in cstate["severed_links"]
    assert 4 in cstate["killed_nodes"]
    assert cstate["packet_loss_rate"] == 0.5

    # Heal all
    coordinator.heal_all_mesh_links()
    coordinator.revive_node(4)
    coordinator.set_packet_loss_spike(0.0)

    packet2 = coordinator.get_telemetry_packet()
    assert len(packet2["chaos_state"]["severed_links"]) == 0
    assert len(packet2["chaos_state"]["killed_nodes"]) == 0
    assert packet2["chaos_state"]["packet_loss_rate"] == 0.0

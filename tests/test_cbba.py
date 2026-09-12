"""Unit tests for Consensus-Based Bundle Algorithm (CBBA) and dynamic re-auctioning."""

import pytest
from decentralized_amr.network.lamport_clock import LamportClock
from decentralized_amr.network.messages import CBBABidMessage
from decentralized_amr.task_allocation.cbba_agent import CBBAAgent
from decentralized_amr.task_allocation.routing import GridPlanner
from decentralized_amr.task_allocation.task import WarehouseTask


def test_cbba_bundle_construction():
    """Verify an agent builds a bundle greedily based on task rewards."""
    clock = LamportClock(node_id=1)
    planner = GridPlanner()
    agent = CBBAAgent(robot_id=1, lamport_clock=clock, planner=planner, max_bundle_size=2)

    # Add two tasks
    t1 = WarehouseTask(task_id="T1", pickup_pos=(2.0, 2.0), dropoff_pos=(2.0, 18.0), urgency=1)
    t2 = WarehouseTask(task_id="T2", pickup_pos=(8.0, 2.0), dropoff_pos=(8.0, 18.0), urgency=3) # Higher urgency

    agent.add_known_task(t1)
    agent.add_known_task(t2)

    changed = agent.build_bundle(current_robot_pos=(8.0, 1.0))
    assert changed is True
    assert len(agent.bundle) > 0
    # Higher urgency task close to robot should be first in bundle
    assert agent.bundle[0] == "T2"


def test_cbba_consensus_higher_bid_wins():
    """Verify peer with higher bid score wins the task in consensus."""
    clock1 = LamportClock(node_id=1)
    clock2 = LamportClock(node_id=2)
    planner = GridPlanner()

    agent1 = CBBAAgent(robot_id=1, lamport_clock=clock1, planner=planner)
    agent2 = CBBAAgent(robot_id=2, lamport_clock=clock2, planner=planner)

    t1 = WarehouseTask(task_id="T1", pickup_pos=(2.0, 2.0), dropoff_pos=(2.0, 18.0), urgency=1)
    agent1.add_known_task(t1)
    agent2.add_known_task(t1)

    # Agent 1 builds bundle (robot at distance 15m)
    agent1.build_bundle(current_robot_pos=(15.0, 15.0))
    # Agent 2 builds bundle (robot right next to pickup at 2.0, 1.5m -> higher score!)
    agent2.build_bundle(current_robot_pos=(2.0, 1.5))

    bid_msg_2 = agent2.create_bid_message()

    # Agent 1 processes Agent 2's bid
    pruned = agent1.process_peer_bid_message(bid_msg_2, current_robot_pos=(15.0, 15.0))
    assert pruned is True
    assert "T1" not in agent1.bundle, "Agent 1 should have released T1 because Agent 2 had higher bid"


def test_cbba_lamport_clock_tiebreaker():
    """Verify that when bids tie in score, higher Lamport clock wins."""
    clock1 = LamportClock(initial_value=5, node_id=1)
    planner = GridPlanner()
    agent1 = CBBAAgent(robot_id=1, lamport_clock=clock1, planner=planner)

    t1 = WarehouseTask(task_id="T1", pickup_pos=(5.0, 5.0), dropoff_pos=(5.0, 15.0))
    agent1.add_known_task(t1)
    agent1.winning_bids["T1"] = 150.0
    agent1.winning_agents["T1"] = 1
    agent1.bid_clocks["T1"] = 5
    agent1.bundle = ["T1"]

    # Incoming peer bid with identical score 150.0, but causally newer Lamport clock 12
    peer_msg = CBBABidMessage(
        sender_id=2,
        lamport_clock=12,
        winning_bids={"T1": 150.0},
        winning_agents={"T1": 2},
        bid_clocks={"T1": 12}
    )

    pruned = agent1.process_peer_bid_message(peer_msg, current_robot_pos=(5.0, 5.0))
    assert pruned is True
    assert agent1.winning_agents["T1"] == 2
    assert "T1" not in agent1.bundle


def test_cbba_dynamic_task_release():
    """Verify task release on blocked aisle clears bundle and generates release broadcast."""
    clock = LamportClock(node_id=1)
    planner = GridPlanner()
    agent = CBBAAgent(robot_id=1, lamport_clock=clock, planner=planner)

    t1 = WarehouseTask(task_id="T1", pickup_pos=(5.0, 5.0), dropoff_pos=(5.0, 15.0))
    agent.add_known_task(t1)
    agent.bundle = ["T1"]

    release_msg = agent.release_task("T1", current_robot_pos=(2.0, 2.0))
    assert release_msg is not None
    assert release_msg.task_id == "T1"
    assert "T1" not in agent.bundle
    assert agent.winning_agents["T1"] == -1


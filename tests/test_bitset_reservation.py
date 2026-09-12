"""Unit tests for O(1) Bitset Reservation Table."""

import pytest
from decentralized_amr.collision_avoidance.bitset_reservation import BitsetReservationTable


def test_bitset_coordinate_conversion():
    """Verify conversion between continuous world coordinates and grid cells."""
    table = BitsetReservationTable(grid_width=60, grid_height=40, resolution=0.5)
    cell_idx = table.coords_to_cell(10.2, 5.4)
    assert cell_idx is not None
    # col = 20, row = 10 -> index = 10 * 60 + 20 = 620
    assert cell_idx == 620

    wx, wy = table.cell_to_coords(cell_idx)
    assert abs(wx - 10.25) < 0.01
    assert abs(wy - 5.25) < 0.01


def test_bitset_conflict_detection():
    """Verify O(1) bitwise AND detection for overlapping spatiotemporal paths."""
    table_a = BitsetReservationTable()
    table_b = BitsetReservationTable()

    # Robot A path through (5.0, 5.0) at step 0
    table_a.set_cell(step=0, x=5.0, y=5.0, footprint_radius=0.4)

    # Robot B at a distant position (20.0, 15.0) at step 0
    table_b.set_cell(step=0, x=20.0, y=15.0, footprint_radius=0.4)
    assert table_a.has_conflict(table_b) is False

    # Robot B also reserved at (5.2, 5.1) at step 0 (overlapping within radius)
    table_b.set_cell(step=0, x=5.2, y=5.1, footprint_radius=0.4)
    assert table_a.has_conflict(table_b) is True
    assert 0 in table_a.get_conflict_steps(table_b)


def test_bitset_trajectory_projection():
    """Verify setting waypoints projects across time steps."""
    table = BitsetReservationTable(horizon_steps=10)
    waypoints = [(0.0, 0.0), (10.0, 0.0)]
    table.set_trajectory(waypoints, nominal_speed=1.0)

    # At step 0, cell around (0,0) must be occupied
    c0 = table.coords_to_cell(0.2, 0.0)
    word0 = c0 // 64
    bit0 = c0 % 64
    assert (table.bitset[0, word0] & (1 << bit0)) != 0


import math

import numpy as np
import pytest

from bvi_core.bvi import (
    queue_bvi, run_and_crosscheck, run_bvi, synchronous_bvi, value_mismatches,
)
from bvi_core.grid import empty_grid, generate_map, generate_reachable_map
from bvi_core.metrics import manhattan

ALL = ["sync", "queue", "queue-relax"]


@pytest.mark.parametrize("algo", ALL)
def test_lecture_chain_S_A_B_G(algo):
    # S -> A -> B -> G on a 4x1 corridor: J = 3, 2, 1, 0
    grid = empty_grid(4, 1, start=(0, 0), goal=(3, 0))
    res = run_bvi(grid, algo)
    assert [res.J((x, 0)) for x in range(4)] == [3, 2, 1, 0]


@pytest.mark.parametrize("algo", ALL)
def test_empty_grid_equals_manhattan(algo):
    grid = empty_grid(7, 5, start=(0, 0), goal=(4, 2))
    res = run_bvi(grid, algo)
    for x in range(7):
        for y in range(5):
            assert res.J((x, y)) == manhattan((x, y), (4, 2))


@pytest.mark.parametrize("algo", ALL)
def test_single_obstacle_detour(algo):
    # 3x3, goal (1,2) top middle, obstacle (1,1) centre, start (1,0).
    #   y=2:  2  G  2        Moving straight up from (1,0) is blocked,
    #   y=1:  1  #  1        so J(1,0) = 4 instead of Manhattan 2.
    #   y=0:  3  S4 3
    grid = empty_grid(3, 3, start=(1, 0), goal=(1, 2), obstacles=((1, 1),))
    res = run_bvi(grid, algo)
    expected = {(0, 2): 1, (2, 2): 1, (1, 2): 0,
                (0, 1): 2, (2, 1): 2,
                (0, 0): 3, (2, 0): 3, (1, 0): 4}
    for cell, j in expected.items():
        assert res.J(cell) == j, cell
    assert math.isinf(res.J((1, 1)))


@pytest.mark.parametrize("algo", ALL)
def test_unreachable_region_stays_infinite(algo):
    # Wall at x=2 splits the 5x3 grid; right side unreachable from goal (0,0).
    wall = tuple((2, y) for y in range(3))
    grid = empty_grid(5, 3, start=(0, 2), goal=(0, 0), obstacles=wall)
    res = run_bvi(grid, algo)
    for x in (3, 4):
        for y in range(3):
            assert math.isinf(res.J((x, y)))
    assert res.J((1, 2)) == 3
    assert all((x, y) not in res.policy for x in (2, 3, 4) for y in range(3))


@pytest.mark.parametrize("algo", ALL)
def test_start_equals_goal(algo):
    grid = empty_grid(3, 3, start=(1, 1), goal=(1, 1))
    assert run_bvi(grid, algo).J((1, 1)) == 0


@pytest.mark.parametrize("seed", range(60))
def test_sync_and_queue_values_identical_on_random_maps(seed):
    grid = generate_map(seed)  # includes disconnected maps on purpose
    sync, queue = run_and_crosscheck(grid)
    relax = queue_bvi(grid, full_backup=False)
    assert value_mismatches(sync.values, queue.values) == []
    assert value_mismatches(sync.values, relax.values) == []
    # Obstacles are always infinite.
    assert np.all(np.isinf(sync.values[grid.occupied]))


def test_mismatch_detector_catches_differences():
    a = np.array([[0.0, 1.0, np.inf]])
    b = np.array([[0.0, 2.0, np.inf]])
    assert value_mismatches(a, b) == [(1, 0)]
    assert value_mismatches(a, a.copy()) == []


def test_bellman_equation_holds_at_fixed_point():
    grid, _ = generate_reachable_map(3)
    res = queue_bvi(grid)
    for s in grid.free_cells():
        if s == grid.goal:
            continue
        rhs = min((1 + res.J(sp) for sp in grid.successors(s)), default=math.inf)
        assert res.J(s) == rhs


def test_statistics_are_consistent():
    grid, _ = generate_reachable_map(5)
    sync, queue = run_and_crosscheck(grid)
    n_states = len(grid.free_cells()) - 1
    reachable_non_goal = int(np.isfinite(sync.values).sum()) - 1
    assert sync.stats["state_updates"] == sync.stats["sweeps"] * n_states
    # Last sweep is the "no change" sweep; values settle after max finite J sweeps.
    assert sync.stats["sweeps"] == np.max(sync.values[np.isfinite(sync.values)]) + 1
    assert sync.stats["value_changes"] == reachable_non_goal
    # FIFO + unit cost: every reachable state improves exactly once, is popped once.
    assert queue.stats["value_changes"] == reachable_non_goal
    assert queue.stats["queue_pops"] == reachable_non_goal + 1
    assert queue.stats["state_updates"] < sync.stats["state_updates"]


def test_peak_queue_is_the_widest_wavefront():
    grid, _ = generate_reachable_map(5)
    queue = queue_bvi(grid)
    finite = queue.values[np.isfinite(queue.values)].astype(int)
    # FIFO + unit cost: the queue holds at most two adjacent J levels.
    widest_two_levels = max(np.sum(finite == d) + np.sum(finite == d + 1) for d in range(finite.max()))
    assert 1 <= queue.stats["peak_queue"] <= widest_two_levels
    assert queue.stats["peak_queue"] < len(grid.free_cells())


def test_unknown_algorithm_rejected():
    with pytest.raises(ValueError):
        run_bvi(empty_grid(2, 2, (0, 0), (1, 1)), "astar")

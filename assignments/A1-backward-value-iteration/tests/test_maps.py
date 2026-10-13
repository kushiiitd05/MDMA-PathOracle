import numpy as np
import pytest

from bvi_core.grid import (
    DEFAULT_GOAL, DEFAULT_START, GRID_SIZE, NUM_OBSTACLES, GridMap,
    connected_to_goal, empty_grid, generate_map, generate_reachable_map,
)


@pytest.mark.parametrize("seed", range(25))
def test_exactly_180_distinct_obstacles(seed):
    grid = generate_map(seed)
    assert grid.occupied.shape == (GRID_SIZE, GRID_SIZE) == (30, 30)
    assert NUM_OBSTACLES == 180
    assert grid.num_obstacles == 180
    assert grid.num_obstacles / grid.occupied.size == pytest.approx(0.20)


@pytest.mark.parametrize("seed", range(200))
def test_obstacles_never_overwrite_start_or_goal(seed):
    grid = generate_map(seed)
    assert grid.is_free(grid.start) and grid.is_free(grid.goal)


def test_custom_start_goal_kept_free():
    for seed in range(50):
        grid = generate_map(seed, start=(5, 20), goal=(22, 3))
        assert grid.is_free((5, 20)) and grid.is_free((22, 3))
        assert grid.num_obstacles == 180


def test_same_seed_same_map_different_seed_different_map():
    assert np.array_equal(generate_map(7).occupied, generate_map(7).occupied)
    assert not np.array_equal(generate_map(7).occupied, generate_map(8).occupied)


def test_default_start_goal_far_apart_with_margin():
    assert DEFAULT_START == (1, 1)
    assert DEFAULT_GOAL == (28, 28)


def test_reachable_generator_returns_connected_map_and_logs_rejections():
    for seed in range(30):
        grid, rejected = generate_reachable_map(seed)
        assert grid.start in connected_to_goal(grid)
        assert grid.seed == seed + len(rejected)
        for bad in rejected:
            bad_grid = generate_map(bad)
            assert bad_grid.start not in connected_to_goal(bad_grid)


def test_motion_model_is_4_connected_and_blocks_walls_and_obstacles():
    grid = empty_grid(3, 3, (0, 0), (2, 2), obstacles=((1, 1),))
    assert set(grid.successors((0, 0))) == {(0, 1), (1, 0)}
    assert set(grid.successors((1, 0))) == {(0, 0), (2, 0)}  # (1,1) blocked
    assert grid.successors((1, 1)) == ()  # obstacle has no actions
    assert grid.predecessors((0, 1)) == grid.successors((0, 1))


def test_invalid_start_or_goal_rejected():
    occ = np.zeros((3, 3), dtype=bool)
    occ[0, 0] = True
    with pytest.raises(ValueError):
        GridMap(occ, (0, 0), (2, 2))
    with pytest.raises(ValueError):
        GridMap(np.zeros((3, 3), bool), (0, 0), (5, 5))


def test_grid_is_immutable():
    grid = generate_map(0)
    with pytest.raises(ValueError):
        grid.occupied[0, 0] = True


# --- difficulty selection -------------------------------------------------
from bvi_core.difficulty import (  # noqa: E402
    MapStats, build_pool, select_by_quantile, select_difficulty_seeds,
)


def test_quantile_selection_is_deterministic_and_ordered():
    pool = [MapStats(seed=i, d_opt=54 + i, d_manhattan=54,
                     stretch=(54 + i) / 54, turns=10) for i in range(11)]
    chosen = select_by_quantile(list(reversed(pool)))
    assert [chosen[k].seed for k in ("easy", "medium", "hard")] == [2, 5, 9]


def test_difficulty_maps_reachable_and_exactly_20_percent():
    report = select_difficulty_seeds(size=120)
    assert report["accepted"] + report["rejected_disconnected"] == 120
    sel = report["selected"]
    assert sel["easy"]["stretch"] <= sel["medium"]["stretch"] <= sel["hard"]["stretch"]
    for stats in sel.values():
        grid = generate_map(stats["seed"])
        assert grid.num_obstacles == 180
        assert grid.start in connected_to_goal(grid)
    assert report == select_difficulty_seeds(size=120)  # reproducible


def test_pool_rejects_only_disconnected_maps():
    accepted, rejected = build_pool(size=80)
    for seed in rejected:
        g = generate_map(seed)
        assert g.start not in connected_to_goal(g)
    assert all(m.d_opt >= m.d_manhattan for m in accepted)

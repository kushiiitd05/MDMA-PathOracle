import pytest

from bvi_core.bvi import queue_bvi, run_bvi, synchronous_bvi
from bvi_core.grid import empty_grid, generate_map, generate_reachable_map
from bvi_core.metrics import count_turns, path_stretch
from bvi_core.path import NoPathError, extract_path, validate_path


@pytest.mark.parametrize("seed", range(40))
@pytest.mark.parametrize("algo", ["sync", "queue"])
def test_random_paths_are_legal_and_optimal(seed, algo):
    grid, _ = generate_reachable_map(seed)
    res = run_bvi(grid, algo)
    for policy in (res.policy, None):
        path = extract_path(grid, res.values, policy)
        validate_path(grid, res.values, path)          # all checks at once
        # Explicit versions of the individual acceptance criteria:
        assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(path, path[1:]))
        assert all(res.J(a) - res.J(b) == 1 for a, b in zip(path, path[1:]))
        assert len(path) - 1 == res.J(grid.start)


def test_start_equals_goal_path_length_zero():
    grid = empty_grid(3, 3, start=(2, 2), goal=(2, 2))
    res = queue_bvi(grid)
    path = extract_path(grid, res.values, res.policy)
    assert path == [(2, 2)]
    assert len(path) - 1 == 0
    validate_path(grid, res.values, path)


def test_no_path_is_reported():
    wall = tuple((1, y) for y in range(3))
    grid = empty_grid(3, 3, start=(0, 0), goal=(2, 2), obstacles=wall)
    for res in (queue_bvi(grid), synchronous_bvi(grid)):
        with pytest.raises(NoPathError):
            extract_path(grid, res.values, res.policy)


def test_unconnected_random_seed_reports_no_path():
    for seed in range(500):
        grid = generate_map(seed)
        res = queue_bvi(grid)
        if res.J(grid.start) == float("inf"):
            with pytest.raises(NoPathError):
                extract_path(grid, res.values)
            return
    pytest.skip("no disconnected map among first 500 seeds")


def test_validate_rejects_bad_paths():
    grid = empty_grid(3, 1, start=(0, 0), goal=(2, 0))
    res = queue_bvi(grid)
    with pytest.raises(ValueError, match="N/S/E/W"):
        validate_path(grid, res.values, [(0, 0), (2, 0)])
    with pytest.raises(ValueError, match="end at goal"):
        validate_path(grid, res.values, [(0, 0), (1, 0)])


def test_turns_and_stretch():
    assert count_turns([(0, 0), (1, 0), (2, 0)]) == 0
    assert count_turns([(0, 0), (1, 0), (1, 1), (2, 1)]) == 2
    assert path_stretch(60, (1, 1), (28, 28)) == pytest.approx(60 / 54)
    assert path_stretch(0, (1, 1), (1, 1)) == 1.0

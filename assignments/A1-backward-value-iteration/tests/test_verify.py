import numpy as np
import pytest

from bvi_core.bvi import queue_bvi, synchronous_bvi
from bvi_core.grid import empty_grid, generate_map, generate_reachable_map
from bvi_core.path import NoPathError, extract_path
from bvi_core.verify import certify


@pytest.mark.parametrize("seed", range(30))
def test_certificate_passes_for_both_algorithms(seed):
    grid, _ = generate_reachable_map(seed)
    for res in (synchronous_bvi(grid), queue_bvi(grid)):
        path = extract_path(grid, res.values, res.policy)
        cert = certify(grid, res.values, path)
        assert cert.passed, cert.lines()
        assert len(cert.checks) == 5


def test_certificate_detects_wrong_value():
    grid, _ = generate_reachable_map(1)
    res = queue_bvi(grid)
    path = extract_path(grid, res.values, res.policy)
    bad = res.values.copy()
    x, y = path[len(path) // 2]
    bad[y, x] -= 1                      # pretend a shortcut exists
    cert = certify(grid, bad, path)
    assert not cert.passed
    assert not {c.name: c.passed for c in cert.checks}["C2 Bellman fixed point"]


def test_certificate_detects_suboptimal_path():
    grid = empty_grid(3, 3, start=(0, 0), goal=(2, 0))
    res = queue_bvi(grid)
    detour = [(0, 0), (0, 1), (1, 1), (2, 1), (2, 0)]   # 4 moves, J(start) = 2
    cert = certify(grid, res.values, detour)
    assert not cert.passed
    assert not {c.name: c.passed for c in cert.checks}["C5 path optimal"]


def test_certificate_detects_finite_value_on_unreachable_cell():
    wall = tuple((1, y) for y in range(3))
    grid = empty_grid(3, 3, start=(0, 0), goal=(2, 2), obstacles=wall)
    res = queue_bvi(grid)
    bad = res.values.copy()
    bad[0, 0] = 5.0
    assert not certify(grid, bad, None).passed
    assert certify(grid, res.values, None).passed  # correct "no path" case


def test_certificate_on_disconnected_random_map():
    for seed in range(500):
        grid = generate_map(seed)
        res = synchronous_bvi(grid)
        try:
            extract_path(grid, res.values)
        except NoPathError:
            assert certify(grid, res.values, None).passed
            assert np.isinf(res.J(grid.start))
            return
    pytest.skip("no disconnected map found")

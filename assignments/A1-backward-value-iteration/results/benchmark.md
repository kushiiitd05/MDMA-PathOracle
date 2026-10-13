# BVI benchmark (200 reachable 30x30 maps, 180 obstacles each)

Seeds 10000..10207; 8 disconnected maps skipped. Runtime = best of 3 runs (CPython, single thread). All maps: J_sync == J_queue == J_queue-relax on every cell; path validated; optimality certificate (bvi_core/verify.py) passed.

| metric (mean / median) | synchronous BVI | queue BVI (Bellman backup) | queue BVI (edge relax) |
|---|---|---|---|
| runtime [ms] | 30.19 / 30.05 | 2.03 / 2.02 | 0.84 / 0.84 |
| state updates attempted | 41041 / 40983 | 2222 / 2224 | 2222 / 2224 |
| successor evaluations | 126920 / 126682 | 7370 / 7370 | 2222 / 2224 |
| successful value changes | 716 / 717 | 716 / 717 | 716 / 717 |
| sweeps / queue pops | 57.1 / 57.0 sweeps | 717 / 718 pops | same pops |
| optimal path length | 54.3 / 54.0 | same | same |

Update-count ratio sync/queue: **18.5x** state updates, **17.2x** successor evaluations.

Timings are sub-40 ms on a 900-cell grid and include Python overhead; the
update counts are the hardware-independent comparison.

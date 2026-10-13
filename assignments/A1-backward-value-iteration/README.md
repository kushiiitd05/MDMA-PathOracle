# BVI Planner — Backward Value Iteration on a 30×30 grid

**Assignment 1 · Algorithmic Techniques for Robot Planning (CSE634)**
Full write-up: [`REPORT.md`](REPORT.md) · 4-page PDF: [`report/ARP_A1_BVI_Report.pdf`](report/ARP_A1_BVI_Report.pdf)

## What it does

* Builds a 30×30 map with exactly **180 random obstacles (20 %)** from a seed. Start (1,1), goal (28,28); both configurable.
* Solves **J(goal)=0, J(s) = min over s' ∈ Succ(s) of [1 + J(s')]** with two hand-written schedules:
  * **synchronous BVI** — full Jacobi sweeps until nothing changes;
  * **queue ABVI** — a FIFO wavefront: for each changed state y, back up every predecessor p with min over Succ(p).
* Asserts the two J maps are identical, extracts and validates the optimal path, and **proves optimality** with a 5-part certificate (`bvi_core/verify.py`).
* ROS 2 publishes the map, J values, both paths, policy arrows and a stats panel. Two robots animate the paths in RViz2, and an independent node re-verifies optimality from `/map`.
* No A*, Dijkstra, NetworkX or Nav2 anywhere.

| | synchronous BVI | queue ABVI |
|---|---|---|
| backups (easy map) | 40,983 | **2,224** (≈ 18× fewer) |
| runtime, 200-map mean | 30.2 ms | **2.0 ms** |
| J, path cost | identical | identical |

## Layout

```
bvi_core/            pure Python (numpy only), shared by everything
  grid.py            GridMap, N/S/E/W model, seeded generation, reachability
  bvi.py             synchronous_bvi, queue_bvi, cross-check
  path.py            extract_path, validate_path
  verify.py          optimality certificate C1..C5
  metrics.py         manhattan, turns, stretch
  difficulty.py      easy/medium/hard quantile selection
standalone/          no-ROS tools (macOS / Linux / Windows)
  run_demo.py        figure of J, path, stats
  benchmark.py       200-map sync vs queue comparison
  tradeoff.py        6-run table + convergence plot
tests/               pytest (434 tests)
ros2_ws/src/bvi_planner/
  bvi_planner/       planner_node, robot_animation_node, optimality_checker_node, markers, runner
  bvi_core -> ../../../bvi_core   (symlink: one copy of the algorithm)
  launch/bvi.launch.py   config/params.yaml   rviz/bvi.rviz, bvi_policy.rviz
scripts/             ros_env.sh, build.sh, verify.sh, demo.sh
report/              4-page PDF + the scripts that build it
results/             PNGs, tables, difficulty_seeds.json, rviz/ screenshots
```

## Run it without ROS (any OS)

```bash
python3 -m venv .venv && .venv/bin/pip install numpy matplotlib pytest
.venv/bin/pytest -q                                              # 434 passed
.venv/bin/python standalone/run_demo.py --difficulty easy
.venv/bin/python standalone/run_demo.py --difficulty hard --compare-bvi --show-policy --save
.venv/bin/python standalone/run_demo.py --seed 42 --start 2,3 --goal 27,25 --algorithm sync
.venv/bin/python standalone/benchmark.py --maps 200 --repeats 3
.venv/bin/python standalone/tradeoff.py
```

| `run_demo.py` flag | effect |
|---|---|
| `--difficulty easy\|medium\|hard` | use the quantile-selected map |
| `--seed N` | use an explicit map seed instead |
| `--start X,Y --goal X,Y` | set start and goal cells |
| `--algorithm queue\|sync\|queue-relax` | choose the BVI schedule |
| `--show-values / --no-show-values` | integer J labels on or off |
| `--show-policy` | draw policy arrows |
| `--compare-bvi` | run both algorithms and assert equal J |
| `--save` | write the PNG to `results/` |
| `--no-show` | don't open a window |

## Run it with ROS 2 Humble (Ubuntu 22.04)

Install ROS 2 Humble (`sudo apt install ros-humble-desktop`, or a RoboStack conda env), then:

```bash
./scripts/build.sh          # colcon build
./scripts/verify.sh         # tests + certificates + trade-off table
./scripts/demo.sh hard      # map, values, both paths, 2 robots, live checker, RViz2
```

Or the raw launch:

```bash
source scripts/ros_env.sh
ros2 launch bvi_planner bvi.launch.py difficulty:=easy            # rviz:=false for headless
ros2 launch bvi_planner bvi.launch.py seed:=42 rviz_config:=bvi_policy.rviz
```

| topic | type | content |
|---|---|---|
| `/map` | nav_msgs/OccupancyGrid | 0 free, 100 obstacle, 30×30, resolution 1.0 |
| `/bvi_values` | visualization_msgs/MarkerArray | J heat-map (CUBE_LIST), TEXT labels, START/GOAL |
| `/bvi_path`, `/abvi_path` | nav_msgs/Path | optimal path from sync / queue BVI |
| `/bvi_policy`, `/abvi_policy` | MarkerArray | π(s) arrows for every reachable state |
| `/bvi_stats` | Marker | comparison panel + certificates |
| `/bvi_robot`, `/abvi_robot` | MarkerArray | animated robots (2.5 Hz) |
| `/optimality_report` | std_msgs/String | independent checker verdict |

All static topics use transient-local (latched) QoS, frame `map`. Parameters live in `config/params.yaml`.

**Environment notes**

* `colcon build --symlink-install` fails with some setuptools versions, so `build.sh` uses a plain `colcon build`.
* ROS's `launch_testing` pytest plugin breaks pytest 9, so `verify.sh` sets `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
* `ros_env.sh` sets `ROS_DOMAIN_ID=73` and `ROS_LOCALHOST_ONLY=1` so the demo never collides with other ROS nodes on the network.

## Rebuild the report (optional)

```bash
.venv/bin/python standalone/report_data.py && .venv/bin/python report/make_figures.py
.venv/bin/python report/build_report.py      # needs Google Chrome for PDF printing
```

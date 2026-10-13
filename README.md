<div align="center">

# 🤖 MDMA · PathOracle

### Algorithmic Techniques for Robot Planning — CSE634 · IIIT Delhi · Monsoon 2026

*Bellman equations, proven optimal, driving robots live in ROS 2.*

![Python](https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![RViz2](https://img.shields.io/badge/RViz2-live%20demo-6f42c1)
![Tests](https://img.shields.io/badge/pytest-434%20passed-2ea44f?logo=pytest&logoColor=white)

<img src="assignments/A1-backward-value-iteration/results/rviz/hard_mid.png" width="720" alt="RViz2: J heat-map, BVI and ABVI paths, two robots mid-run"/>

</div>

---

## ✨ Assignment 1 — Backward Value Iteration on a 30×30 grid

**[→ assignments/A1-backward-value-iteration](assignments/A1-backward-value-iteration)**

- 30×30 grid, exactly **180 random obstacles (20 %)**, start (1,1), goal (28,28), N/S/E/W moves with cost 1.
- Two schedules of one Bellman equation, both hand-written:
  - **synchronous BVI**: Jacobi sweeps;
  - **queue ABVI**: a FIFO wavefront over predecessors.
- A **5-part optimality certificate** (C1–C5) proves the path is optimal. It runs in the tests, the demo, the planner node **and** an independent ROS checker node that rebuilds J from `/map` alone.
- Live ROS 2 scene:
  - J heat-map;
  - both optimal paths;
  - policy arrows;
  - two animated robots;
  - a stats panel.
- Easy / medium / hard maps picked by quantile from a 500-seed pool.
- No A*, Dijkstra, NetworkX or Nav2.

| 200 random maps (mean) | synchronous BVI | queue ABVI | queue (edge relax) |
|---|---:|---:|---:|
| state updates | 41,041 | **2,222** | 2,222 |
| successor evaluations | 126,920 | 7,370 | **2,222** |
| runtime | 30.19 ms | 2.03 ms | **0.84 ms** |
| optimal path length | 54.3 | same | same |

Same J on every cell, ≈ **18× fewer backups**. Sync work grows like |X|^1.5; queue work like 4|X|.

### Quick start

```bash
cd assignments/A1-backward-value-iteration
python3 -m venv .venv && .venv/bin/pip install numpy matplotlib pytest
.venv/bin/pytest -q                                                       # 434 passed
.venv/bin/python standalone/run_demo.py --difficulty hard --compare-bvi --show-policy
```

With ROS 2 Humble: `./scripts/build.sh && ./scripts/demo.sh hard`. Full guide in the [assignment README](assignments/A1-backward-value-iteration/README.md); results and proofs in [REPORT.md](assignments/A1-backward-value-iteration/REPORT.md).

---

## 📚 Lectures

| # | Topic |
|---|---|
| 01 | Introduction |
| 02–03 | Forward and backward search |
| 04–05 | Discrete optimal planning (value iteration) |
| L5 | LPA* and D* Lite |
| 07 | Asynchronous BVI |
| 08–010 | Transition systems and an application |
| 010–011 | LTL and its application |
| 012–013 | Art gallery problem |

All slides are in [`lectures/`](lectures).

Reading list:
- S. Koenig & M. Likhachev, *D* Lite*, AAAI 2002.
- H. Kress-Gazit, G. Fainekos & G. Pappas, *Where's Waldo? Sensor-based temporal logic motion planning*, ICRA 2007.
- T. Kundu & I. Saha, *Charging Station Placement for Indoor Robotic Applications*, ICRA 2018.

<div align="center">

**Kush Tokas** · 2023296 · IIIT Delhi · Team **MDMA**

<sub>Slides belong to their instructors.</sub>

</div>

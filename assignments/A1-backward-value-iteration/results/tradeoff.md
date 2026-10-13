# Easy / medium / hard x BVI / ABVI

Travel time = path cost x 0.4 s per move (robot animation rate).
Both algorithms give identical J on every cell of every map.

| map | seed | algorithm | sweeps / pops | state updates | successor evals | value changes | runtime [ms] | J(start) | turns | travel time [s] | certificate |
|---|---|---|---|---|---|---|---|---|---|---|---|
| easy | 219 | BVI (sync) | 57 sweeps | 40,983 | 126,768 | 717 | 29.70 | 54 | 14 | 21.6 | PASS |
| easy | 219 | ABVI (queue) | 718 pops | 2,224 | 7,372 | 717 | 2.15 | 54 | 14 | 21.6 | PASS |
| medium | 435 | BVI (sync) | 57 sweeps | 40,983 | 127,452 | 717 | 29.88 | 54 | 17 | 21.6 | PASS |
| medium | 435 | ABVI (queue) | 718 pops | 2,236 | 7,420 | 717 | 1.99 | 54 | 17 | 21.6 | PASS |
| hard | 288 | BVI (sync) | 59 sweeps | 42,421 | 130,921 | 717 | 33.17 | 56 | 16 | 22.4 | PASS |
| hard | 288 | ABVI (queue) | 718 pops | 2,219 | 7,323 | 717 | 2.14 | 56 | 16 | 22.4 | PASS |

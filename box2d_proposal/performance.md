# Performance

**Harness:** `experiments/ghost_bench`. Methodology is in its [README](../../experiments/ghost_bench/README.md); the full tables are in [results.md](../../experiments/ghost_bench/results.md) and the raw data in [results.csv](../../experiments/ghost_bench/results.csv).
- **Integration:** used exactly as a game would. One heap context per protected body, pre-solve events only on protected shapes, and `TopoGhost_PreStep` before every `b2World_Step`. A dispatch callback uses body user data, so it is O(1) per call.
- **Build and world:** Release `-O3`, single-threaded world (workerCount 1, no task system), 60 Hz, 4 substeps.
- **Runs:** 60 warm-up steps plus 600 measured steps per run, 7 repeats, three modes interleaved with rotating order.
- **Modes:**
  - **off:** stock Box2D with polygon tiles;
  - **topo:** the mitigation on the same tiles;
  - **chain:** a `b2ChainDef` floor with no mitigation, the ghost-free reference.
- **Statistic:** median and p10/p90 of all pooled measured steps. `TopoGhost_PreStep` and `b2World_Step` are timed separately; pre-solve time is inside the topo-mode step.
- **Determinism:** final-state checksums were identical across all repeats in every mode.

**Machine:** i5-1135G7 laptop, gcc 13.3, pinned to one core with `taskset`. Turbo and the `powersave` governor were on, and the machine was not idle. A second invocation reproduced the medians within about 10% (PreStep within 3%).

**Which comparison to read.** Prefer *overhead vs chain*. Without mitigation the driven body is regularly launched by ghost contacts and spends steps airborne with no contacts, which makes the "off" step artificially cheap.

## Results (µs per step, medians)

**Simple case** (body over 2–4 flush boxes, K ≈ 1):

| Config | PreStep | Overhead vs chain |
|---|---|---|
| Circle, 2 boxes | 0.5 | +0.7 (+28%) |
| Box, 4 boxes | 0.6 | +0.9 (+41%) |

The single-body world step is about 2.3 µs, so the percentage is large and the absolute cost is about 1 µs.

**Neighbourhood density** (1×1 box at 10 m/s, tile width w):

| w (m) | K | E | PreStep | Step topo − chain | Overhead vs chain |
|---|---|---|---|---|---|
| 2 | 2.1 | 8.6 | 1.1 | 0.3 | +1.4 |
| 0.5 | 5.6 | 22 | 3.2 | 0.5 | +3.6 |
| 0.125 | 19 | 77 | 14.8 | 1.8 | +16.7 |
| 0.0625 | 38 | 150 | 32.2 | 2.7 | +34.9 |
| 0.03125 | 74 | 296 | 90.2 | 5.6 | +95.9 |
| 0.015625 | 107 (cap hit) | 426 | 254 | — | mitigation fails |

- PreStep is about 0.5–0.8 µs per neighbour up to K ≈ 40, then superlinear (about 1.2 µs per neighbour at K = 74).
- The per-contact part (replacing manifolds) stays at 0.3–6 µs. The cost is the topology build.

**Speed** (0.5 m tiles): speed matters only through the query reach, `queryMargin + 2|v|dt`.

| v (m/s) | K | PreStep |
|---|---|---|
| 2.5 | 4.6 | 2.5 |
| 80 | 14.8 | 8.4 |

**Protected bodies** (0.5 m tiles, 5 m/s): linear, about 2.6 µs PreStep and 3.2 µs total per body.

| N | Total overhead vs chain | Memory |
|---|---|---|
| 0 | 0.0 | none: no cost when unused |
| 1 | +2.7 | 384 KB |
| 16 | +47 | 6 MB |
| 64 | +193 | 24.6 MB |

**World size** (one protected body, plus 0–4005 awake dynamic boxes elsewhere): the overhead stays at 6–14 µs, which is 1–4% of a 170–1700 µs step. PreStep creeps from 2.7 to 7.9 µs, consistent with a deeper broad-phase tree and colder caches, not more work.

**Shape type** (equal K about 5): 2.5–3.1 µs PreStep for circle, capsule, box and hexagon. The total differs by the number of contacts: 1.7 pre-solve calls per step for the circle, 3.1 for the box.

## Worst case: overlapping debris

Small static convex polygons embedded in the floor overlap each other pairwise. That isn't the flush-tile case, and the cost becomes roughly quadratic in the local edge count:

| Pieces | K | E | PreStep (2 protected bodies) |
|---|---|---|---|
| 250 | 13.5 | 61 | 55 µs |
| 1000 | 45 | 214 | 478 µs |
| 2500 | 110 | 525 | 2.3 ms |
| 5000 | 128 (cap) | 616 | 4.2 ms |

- **Per body:** about 0.24 ms at K = 45, against 0.03 ms for tiles at K = 38.
- **Profile** (callgrind, `-fno-inline`), all in `ComputeTopology`:
  - `FindGhost` about 43%;
  - `IsCovered` about 27%;
  - `CancelInteriorEdges` about 27%.
- **Why it's quadratic:** each piece scans the pieces of all owners adjacent to its owner. With tiles that is about 2 owners; with overlapping debris it is most of the neighbourhood.
- **Dynamic debris** (bodies ploughing through a loose pile): 1000 pieces gives 498 µs PreStep, on top of a 300–400 µs world step.

## Cap behaviour

At 1/64 m tiles the snapshot hits the 128-neighbour cap on most body-steps.

- **Originally:** query-order truncation dropped arbitrary neighbours, including the ground under the body. The topology was wrong: the box was launched (max y 1.99 against 0.50 resting) while paying 254 µs of PreStep.
- **Fixed in `TopoGhost_PreStep`:** on neighbour overflow, the step's topology is skipped (`context->bypass`) and both callbacks leave Box2D's contacts unchanged for that body and step. The same case now behaves exactly like no mitigation (max y 1.60, as off) at 5.7 µs PreStep. Ghosts are not removed there, but nothing is made worse.
- **Proper fix (in-engine or later):** trim the neighbourhood by distance to P's swept AABB instead of capping by query order.

## Interpretation for an in-engine version

- **Topology build, not pre-solve, is the dominant cost.** It's per protected body, serial, and repeated every step for static geometry that doesn't change. A per-static-body cache of edges, adjacency and cancellation removes the O(K²) adjacency and most of the cancellation for static terrain. Only the player-dependent parts remain per body: supports, covering, ghosts restricted to pieces near the contact.
- **Restrict ghost and covering work to pieces near P's swept AABB** rather than all pieces of the neighbourhood. For overlapping debris this bounds the quadratic term by local density rather than K.
- **Parallelism:** the build is independent per protected body and could run as a task in collide.
- **Memory:** the 384 KB per body is fixed worst-case arrays. Arena scratch per task would remove it.
- **For typical character use** (one to a few protected bodies on tiles or terrain), the external version costs about 1–15 µs per body per step, depending on local density.

# ghost_bench: cost of topological ghosts

A standalone, reproducible benchmark of the CPU and memory cost of topological ghosts
(`shared/topo_ghosts.{h,c}`). It replaces the single-run timings of `ghost_probe --bench`.

Results for the reference machine are in [results.md](results.md) (with [results.csv](results.csv)).

## Integration under test

The harness uses only Box2D's public API plus `topo_ghosts.h`, the way a game would:

- one `TopoGhostContext` per protected body, heap allocated (`TopoGhost_Init`, `config.timeStep = 1/60`);
- `enablePreSolveEvents` on the protected shapes only;
- `b2World_SetPreSolveCallback(world, DispatchPreSolve, DispatchPreContinuous, NULL)`: a tiny dispatch
  that finds the protected body's context through its body user data (O(1) per contact, whatever the
  number of protected bodies) and forwards to `TopoGhostPreSolve` / `TopoGhostPreContinuous`;
- `TopoGhost_PreStep` for every protected body before each `b2World_Step`.

Protected bodies are driven like a character controller: before each step the horizontal velocity is
reset to the configuration's speed (vertical velocity kept). Driving is not timed.

## Modes

Every configuration is run in up to three modes:

| mode | floor | mitigation |
|---|---|---|
| off | static box tiles (one polygon per tile) | none, stock Box2D, no pre-solve |
| topo | same tiles | topological ghosts |
| chain | one chain shape with a vertex at every tile seam | none (Box2D's own ghost vertices: the ideal, ghost-free reference) |

Two overheads are reported. **vs off** is what switching the mitigation on costs a game. **vs chain** is
the cost against the ideal result. They differ because without mitigation the protected body is
launched by ghost collisions and spends much of the run in the air with no contacts, so the off step
is cheaper than a correct one would be (the `max y off/topo` column shows it: the resting height is
0.5 for the 1 x 1 box and the r = 0.5 circle).

## Methodology

- Release build (`-O3 -DNDEBUG`, plus Box2D's own `-ffp-contract=off`), single-threaded world
  (`workerCount = 1`, no task system), fixed 60 Hz, 4 substeps. Asserts-on builds print a warning in the
  header.
- Per configuration and mode: build the scene, run W = 60 warm-up steps, then M = 600 measured steps.
  Each measured step times (a) `TopoGhost_PreStep` for all protected bodies and (b) `b2World_Step`
  separately with `clock_gettime(CLOCK_MONOTONIC)` (QueryPerformanceCounter on Windows,
  `timespec_get` as a portable fallback). In topo mode (b) includes the pre-solve callbacks.
- Each configuration is repeated R = 7 times. The modes are interleaved within each repeat and their
  order rotates every repeat (off, topo, chain / topo, chain, off / ...) so slow drifts in clock
  frequency or thermal state hit all modes equally.
- Statistic: the median, p10 and p90 of the per-step times pooled over all measured steps of all
  repeats (R x M = 4200 samples per mode). The median of pooled steps is robust to interrupts and
  frequency transitions, which only show in the tails, and it is the typical frame cost a game sees.
  The median of per-run means was the alternative, but with 7 runs it is coarser and folds rare spikes
  into every run's mean. Means are also written to the CSV (`*_mean`, `mean_overhead_us`).
- overhead = median(PreStep + Step, topo) - median(Step, reference), in µs and in % of the reference.
- Topology statistics over the measured steps, per protected body and step: K = neighbors snapshotted
  (`context->neighborCount`), E and P = step-scene edges and surviving pieces (`stepScenes[i].scene`),
  calls = pre-solve calls (`stats.replaced + culled + kept + unchanged`). cap steps = body-steps whose
  snapshot hit `TOPO_MAX_NEIGHBORS`, overflow = `stats.overflow` events.
- Memory: N x `sizeof(TopoGhostContext)` (about 384 KB per protected body).
- Determinism: a checksum (FNV-1a over the final position and rotation bits of all dynamic bodies) per
  run; `det = ok` when all repeats of every mode agree.
- Run pinned to one core (`taskset -c 2`) when available. The header records the CPU, the allowed CPU
  list, the compiler and the flags.

## Benchmarks

1. **Baseline simple**: a circle or a 1 x 1 box driven at 1.5 m/s over 2 or 4 flush static boxes (20 m).
2. **Neighborhood density**: 1 x 1 box at 10 m/s over tiles of width 2 m down to 1/64 m; the last two
   widths exceed the 128-neighbor cap on purpose.
3. **Speed**: 1 x 1 box over 0.5 m tiles at 2.5 to 80 m/s (query reach = queryMargin + 2 |v| dt).
4. **Protected-body count**: N = 0 to 64 bodies (alternating circle / box) 3 m apart on 0.5 m tiles.
   N = 0 is the noise floor.
5. **Dense debris**: a protected circle and box crossing 250 to 5000 small seeded random convex
   polygons embedded in the floor (static gravel), or a loose dynamic pile of 200 or 1000.
6. **Unprotected-body scaling**: one protected box plus a far-away pyramid of up to 4005 awake boxes.
7. **Shape type**: circle, capsule, box and hexagon on 0.5 m tiles.

## Build and run

```sh
cmake -S . -B build/rel -DCMAKE_BUILD_TYPE=Release -DBOX2D_SAMPLES=OFF
cmake --build build/rel --target ghost_bench
taskset -c 2 build/rel/bin/ghost_bench --csv results.csv > results.md
```

Options: `--repeats R` (default 7), `--warmup W` (60), `--steps M` (600), `--bench 1247` (subset),
`--csv file`, `--quick` (2 repeats, 10 + 60 steps: smoke test, e.g. in a Debug build).
The full run takes about 75 s on the reference laptop.

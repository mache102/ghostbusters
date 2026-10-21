# Topological ghosts: summary for Erin

This follows up the Discord thread: what it is, what it fixes, what it costs, and how it could go into the engine. Branch [`mache102/ghostbusters`](https://github.com/mache102/ghostbusters) (v3.2.0 + 2, `a292912`).
- The module is `shared/topo_ghosts.{h,c}`, external, using only the public API.
- The implementation, tests and benchmarks were written with an LLM, from my localSeg algorithm and the edge filtering in my game. Treat the code as experimental.

## What it is

A pre-solve mitigation for designated dynamic shapes. Once per step, before `b2World_Step`, it snapshots the protected shape's neighbourhood (any body type: polygons, segments, chain segments) and builds a local surface topology:
1. **Interior cancellation:** coincident antiparallel edges between neighbours are removed (localSeg phase 1, generalised off-grid).
2. **Covering:** steep faces behind a surface the shape is resting on (sunk ≤ 4·slop) are ignored. This handles hidden faces that have no coincident partner: gaps, moving tiles, faces under a one-way lip.
3. **Ghost vertices:** chosen on the fly from neighbouring surviving edges (the localSeg smallest-turn rule).

In pre-solve, each manifold of the protected shape is **replaced** by re-colliding it against the touched shape's surviving edges as `b2ChainSegment`s, using your chain colliders. Bodies with their centroid inside a solid are pushed out through outer faces only. Time-of-impact hits on interior features are vetoed.

It does not gather and filter a shape's contacts; it is per-manifold replacement against a per-step topology.

Details, pseudocode, rule conditions and code references: [method.md](method.md).

## Results

From [correctness.md](correctness.md): 22 scenarios plus multi-body runs, comparing none, your GhostCulling (Semrau) filter, and topological.
- **Fixes what chains fix,** without building chains: box, segment and slanted-quad tiles, curved and sliced terrain.
- **Also fixes what Semrau doesn't:**
  - 1-point contacts: circles, capsules, tilted or rotating polygons, vertex-vertex speculative contacts;
  - two-sided segments;
  - sinks deeper than 4·slop;
  - flush kinematic, rotating and dynamic tiles.
- **Shape coverage:** circle, capsule, triangle, pentagon, hexagon and box all match their smooth-chain reference on triangle-tile slopes, dynamic crate floors and sliced curved terrain.
- **Phase-independent** over 41-speed sweeps, 41/41 everywhere tested, apart from a 15 mm step where being blocked is correct. Semrau is 41/41 on upright boxes over box tiles, 3–5/41 on segments, tilts and fast landings.
- **Real geometry still blocks:** walls, cliffs, real lips (15 mm blocks a sliding box; 5 mm misalignment is treated as flush), and thin walls at 80 m/s. Semrau's floor-material time-of-impact veto lets 80 m/s bodies through a thin wall.
- **Push-out:** a body spawned 15 cm inside a two-tile floor falls through with stock Box2D. With this it exits through the top.

**Not solved:**
- Pre-solve can't create contacts your collider dropped (hollow chains under extreme load, degenerate loose-segment ghosts).
- Push-out pops at about 10 cm penetration.

## Cost

From [performance.md](performance.md) and the `experiments/ghost_bench` harness: Release, single thread, 7 interleaved repeats, medians, compared with a chain floor.

| Case | Extra per step |
|---|---|
| Simple case (2–4 flush boxes) | about 1 µs |
| Typical tiles | about 0.5–0.8 µs per neighbour shape within reach, up to ~40 neighbours |
| Protected bodies | linear: about 3.2 µs per body |
| No protected bodies | 0 |
| World size | independent of it (one protected body next to 4000 awake boxes: +6–14 µs) |

- **Dominant cost:** the per-step topology build. It's serial, rebuilt every step even for static geometry, and stored in fixed arrays (384 KB per protected body).
- **Worst case:** overlapping debris (gravel embedded in the floor). It's roughly quadratic in local edges: 0.5 ms at 45 neighbours, 4.2 ms at the 128 cap. Above the cap the module now falls back to native contacts.

## Integration

From [integration.md](integration.md):
- **Where it would run:** opt-in per shape, with the manifold built in `b2UpdateContact` in place of the registry `fcn` for protected pairs.
- **What the engine gains:**
  - it can create contacts;
  - no snapshot;
  - the topology build runs as a parallel task during collide;
  - static topology is cached per static body, which removes most of the cost;
  - the same topology feeds the chain-segment early-out in `b2ContinuousQueryCallback`.
- **Minimal alternative:** allow pre-solve on zero-point manifolds, plus read-only queries during collide. That alone removes the external module's main limits.
- **Open questions:** scope (players only, or all dynamics), thresholds (4·slop, 60° crest continuation), rounded and circle terrain, the character mover and shape casts.

[appendix_rounding.md](appendix_rounding.md) covers the large-convex-radius suggestion from the thread: it's phase-dependent and turns snags into launches.

## Reproduce

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
build/bin/ghost_probe --defaults                       # every scenario: none / semrau / rounded / topological
build/bin/ghost_probe --players "Shape Parade" 3 3 6   # per body vs smooth reference (mitigation 3 = topological)
build/bin/ghost_probe --trace "Parabola Hills" 0 0 3   # per step state and manifold normals
taskset -c 2 build/bin/ghost_bench                     # cost benchmarks (all seven; --bench N for one)
build/bin/samples                                      # category "Ghost Collisions", mitigation dropdown
```

**Scenario and sample code:** `shared/ghost_scenarios.c`, `samples/sample_ghost_collisions.cpp`.

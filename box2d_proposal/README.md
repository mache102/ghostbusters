# Topological ghosts: summary

For Erin, following the Discord thread. Branch [`mache102/ghostbusters`](https://github.com/mache102/ghostbusters) (v3.2.0 + 2). Module: `shared/topo_ghosts.{h,c}`, external, public API only. Implemented with an LLM from my localSeg algorithm and the edge filtering in my game; treat the code as experimental.

## The method in one paragraph

For each protected dynamic shape, before `b2World_Step`, snapshot its neighbourhood (any body type: polygons, segments, chain segments) and build a local topology:
1. cancel coincident antiparallel edges between neighbours;
2. ignore steep faces behind the surface the shape rests on (sunk ≤ 4·slop);
3. choose ghost vertices on the fly from the surviving neighbouring edges (the localSeg smallest-turn rule).

In pre-solve, each of the shape's manifolds is **replaced** by re-colliding against the touched shape's surviving edges as `b2ChainSegment`s, through your chain colliders. It does not gather and filter contacts.
→ [method.md](method.md): pipeline, pseudocode, exact rules and tolerances, code references, which rule does the work in each case.

## Key results

- **Fixes ghosts that chains fix, without chains,** on arbitrary pieces: box, segment and sliced-quad tiles, curved terrain, and static, kinematic, rotating or dynamic neighbours.
- **Fixes what the GhostCulling (Semrau) filter can't:**
  - 1-point contacts: circles, capsules, tilted or rotating polygons;
  - two-sided segments;
  - sinks deeper than 4·slop.

  Circle, capsule, triangle, pentagon, hexagon and box all match their smooth-chain reference.
- **Phase-independent:** clean in 41/41 launch speeds in every case tested, except a 15 mm step that correctly blocks. Semrau: 41/41 on upright boxes over box tiles; 3–5/41 on segments, tilts and fast landings.
- **Real geometry still blocks:** walls, cliffs, lips, and thin walls at 80 m/s (Semrau's TOI floor-material veto lets those through).
- **Push-out:** a body buried at a two-tile seam exits through the top; stock Box2D drops it through the floor.
- **Not solved:** contacts your collider already dropped (pre-solve can't create them), and push-out pops at about 10 cm penetration.

→ [correctness.md](correctness.md): per-scenario and per-body tables, the speed sweep, valid behaviour left alone, bugs found and fixed.

## Cost

Release, single thread, medians of 7 interleaved repeats, compared with a chain floor:

| Case | Extra per step |
|---|---|
| Simple case | about 1 µs |
| Tiles | about 0.5–0.8 µs per neighbour within reach (up to ~40 neighbours) |
| Protected bodies | linear, about 3.2 µs and 384 KB per body |
| Unused, or world size | no cost / independent |
| Worst case: overlapping debris | roughly quadratic in local edges; 0.5 ms at 45 neighbours, 4.2 ms at the 128 cap (above it, falls back to native contacts) |

Almost all of it is the per-step topology build: serial, rebuilt every step even for static geometry.
→ [performance.md](performance.md): methodology, all sweeps, profile, cap behaviour.

## Integration

- **Proposal:** opt-in per shape, with the topological manifold built in `b2UpdateContact` instead of the registry `fcn` for protected pairs. That lets it create contacts.
- **Topology build:** a parallel task during collide, with the static part cached per static body. That removes most of the cost.
- **Continuous:** the same topology drives the chain-segment early-out in `b2ContinuousQueryCallback`.
- **Minimal alternative:** pre-solve on zero-point manifolds, plus read-only queries during collide.
- **Open questions:** scope (players or all dynamics), the empirical thresholds (4·slop, 60° crest continuation), rounded and circle terrain, the character mover and shape casts.

→ [integration.md](integration.md)

## Reproduce

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
build/bin/ghost_probe --defaults                       # every scenario, none / semrau / rounded / topological
build/bin/ghost_probe --players "Shape Parade" 3 3 6   # per body vs smooth reference
taskset -c 2 build/bin/ghost_bench                     # cost benchmarks
build/bin/samples                                      # "Ghost Collisions" category, mitigation dropdown
```

Separate note on the large-convex-radius suggestion from the thread: [../large_convex_radius.md](../large_convex_radius.md).

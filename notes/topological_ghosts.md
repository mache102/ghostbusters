# Topological ghosts: results for game developers

Branch `ghostbusters` (Box2D v3.2.0+2). Updated 2026-10-21.

This is the experiment that follows [ghost_collisions.md](ghost_collisions.md). Raw data:
- [ghost_defaults.md](ghost_defaults.md): every sample at its default settings.
- [ghost_probe_results.md](ghost_probe_results.md): full parameter sweeps.

## The short version

A *ghost collision* is when a body sliding or rolling across a surface made of several pieces catches on a seam between them, as if there were a tiny wall or bump. It stops dead, gets kicked into the air, or tumbles.

Box2D's built-in fix is the **chain shape**. It only works for static terrain merged into one chain. The upstream sample's fix is **Semrau's filter**: it drops short, flat contacts. It works for upright boxes, but does nothing for balls, tilted or rotating shapes, or two-sided segments, and it fails under heavy load.

**Topological ghosts** is a new mitigation built on Box2D v3.2's pre-solve callback. For each protected body, it works out every step which edges of the nearby shapes form the real surface, ignores the hidden ones, and rebuilds the contact as if that surface were one smooth chain. It is recomputed at the current poses, so it also works across separate bodies: moving platforms, dynamic crates, pieces of different shapes, curved and sliced terrain.

## Q&A

**Does it touch Box2D's code?**
No. `shared/topo_ghosts.{h,c}` uses only Box2D v3.2's public API:
- the pre-solve and pre-continuous callbacks;
- read-only shape and body getters;
- `b2World_OverlapAABB`;
- the public chain-segment colliders (`b2CollideChainSegmentAnd*`).

This branch also carries a few engine additions (the `b2Custom*` ghost-vertex API in `src/shape.c`, `src/contact.c` and `box2d.h`). They are a separate experiment, and the method does not use them.

**Can it be used without merging anything into Box2D?**
Yes. Copy the two files into a game that uses stock Box2D v3.2, call `TopoGhost_PreStep` before each `b2World_Step`, and install the callbacks (see "Using it in a game" below). Two Box2D rules shape how it works:
- World queries are not allowed inside pre-solve, so the neighbourhood is snapshotted just before the step. Collision runs at the start of the step, so the poses match.
- A recycled contact skips pre-solve, so contact recycling is turned off for each protected body.

**Would it benefit from being integrated into Box2D?**
Yes, in several ways:
- **Every body, not just designated ones.** Inside the engine it could run in the contact update for every dynamic body. It would then work for crates on crates, debris and ragdolls, not only a player.
- **Creating contacts, not just editing them.** Pre-solve only sees contacts Box2D's collider already produced. Inside the collider it could also *add* the right contact, which fixes the remaining hollow-chain fall-through and the degenerate end-cap case.
- **No snapshot and no per-body copies.** The engine can read the broad-phase directly during the collide phase, which does not modify the trees. It could also share one topology per area between all bodies, instead of about 400 KB of state per protected body.
- **Parallelism.** The engine already runs contact updates on its worker threads. Today the per-step build runs serially in `TopoGhost_PreStep`.
- **Continuous collision.** The engine could apply the same topology inside time-of-impact, instead of vetoing hits after the fact.

**How complex is the method?**
Let *K* be the number of shapes near a protected body, *E* the number of their edges (at most 8 per polygon, about 8K), and *A* the number of pairs of nearby shapes that touch each other.

| Stage | Cost |
|---|---|
| Neighbourhood query (the broad-phase tree) | O(log N + K) |
| Finding which nearby shapes touch | O(K²) bounding-box tests |
| Interior cancellation | O(A × 8 × 8) |
| Covering and ghost selection | O(K) per edge, so O(E × K) |
| Per contact | O(edges of the touched shape) chain-segment collisions |

There are fixed caps: 128 neighbours, 8 edges per shape, 6 surviving pieces per edge, 16 support surfaces. Past a cap it counts an overflow and falls back to safe behaviour: it leaves edges uncut and keeps Box2D's own contact.

**Where does it scale worst?**
- **Fast bodies.** The query reaches |v| × 2 steps beyond the body, so the neighbourhood grows with speed. A box at 40 m/s over 0.5 m tiles was the most expensive case measured.
- **Dense neighbourhoods.** Many small shapes near the body (fine tile grids, rubble, debris) push K up, and the O(K²) and O(E × K) terms grow with it. Past 128 neighbours the topology is incomplete.
- **Many protected bodies.** Each one builds its own topology and holds about 400 KB of state, so cost and memory grow linearly. The engine-integrated version would share the work.
- **Large worlds.** Bounds are world-space floats, so precision drops far from the origin in double-precision builds.

**What is the performance overhead?**
Measured with `ghost_probe --bench` (single thread, Release), as extra time per world step:

| Scenario | No mitigation | Topological | Extra |
|---|---|---|---|
| Box on segment tiles | 4.0 µs | 5.7 µs | +1.7 µs |
| Shape Parade, 6 bodies, slope tiles | 1.8 µs | 2.9 µs | +1.1 µs |
| Shape Parade, 6 bodies, dynamic crate floor | 13.5 µs | 14.8 µs | +1.3 µs |
| Parabola Crates, dynamic slices | 14.6 µs | 19.3 µs | +4.7 µs |
| Parabola Hills, 20 static slices | 3.9 µs | 7.9 µs | +4.0 µs |
| Fast landing, 40 m/s over 0.5 m tiles (worst measured) | 2.0 µs | 16.0 µs | +14 µs |

So it costs a few microseconds per protected body per step in normal play, rising to around 15 µs for a fast body over fine tiles. That is small next to a 16 ms frame, but it is per protected body, so protect the bodies that need it (players, vehicles, balls) rather than everything.

**Where would it be useful in a game?**
- **Tile-map platformers and top-down games.** A character sliding or running across grid tiles, along tiled walls, or between tiles of different materials, without merging the level into chains. It also handles slopes and non-square tiles.
- **Moving platforms, elevators, conveyor pieces, rotating platforms.** Seams between a moving body and static terrain, which chains cannot express.
- **Crate and block puzzles.** Pushing or rolling things across rows of loose crates or blocks, which are dynamic and shift slightly.
- **Destructible or procedural terrain.** Fractured pieces (e.g. Voronoi cuts), chunked or streamed worlds, and runtime-edited levels, where the seams change every frame and rebuilding chains is impractical.
- **Driving and vehicle games.** Wheels and chassis crossing modular track pieces, ramps and crests.
- **Ball and marble games.** Pinball, mini-golf and rolling-ball track builders, where a ball rolls over pieces placed by hand or by players. Ghost bumps there are very visible.
- **Level editors and user-generated content.** Pieces snapped together with tiny misalignments and arbitrary shapes.
- **Physics-heavy characters.** Tumbling or rotating player shapes (wheels, rolling polygons, compound bodies), where Semrau's box-only filter does not apply.

## 1. What topological ghosts solves

Measured with `ghost_probe` (deterministic, 60 Hz, 4 substeps). Each row is a sample at its default settings; "ok" means it behaves like the same body on a smooth, seamless surface.

| Scenario (what it tests) | No mitigation | Semrau | Topological |
|---|---|---|---|
| **Shape Parade**: ball, capsule, triangle, pentagon, hexagon and box sliding down a slope of triangle tiles | all 6 bumpy (kicks up to 3.6 m/s) | only pentagon and box fixed | **all 6 ok** |
| Shape Parade on a dynamic crate floor | capsule, triangle, box stuck; others bumpy | circle and capsule still bumpy | **all 6 ok** |
| Shape Parade on parabola crates | all worse than the smooth reference | about the same as none | **all 6 match the reference** |
| **Parabola Hills**: ball over a 21 m bowl of static slanted slices with sine-wave hills | leaves the ground 12 times (89 steps in the air) | same | **never leaves the ground**, like the smooth chain |
| **Parabola Crates**: ball over a curved bowl of dynamic, unevenly sliced quadrilateral crates | bumpy (kick 1.1 m/s, 31% speed loss) | same | **ok** (only crate compression remains) |
| Ball on a flush rotating platform | bumpy (kick 3.0 m/s) | same | **ok** |
| Free box sliding down triangle tiles | tumbles (kick 7 m/s) | same | **ok** |
| Box held at a 10° tilt, sliding on its corner | bumpy (kick 1.7 m/s, 26% loss) | same | **ok** |
| Ball rolling over dynamic crates | bumpy (kick 2.3 m/s) | same | **ok** |
| Box on two-sided segment tiles / small gaps / sliding down a segment wall | stuck | stuck / hangs | **ok** |
| 1 mm misaligned tiles | stuck | launched 1.2 m | **ok** |
| Fast diagonal landing on tiles | launched 18 m | same | **ok** |
| Two-wheel vehicle over a crest onto tiles | bumpy | bumpy | **ok** |
| Thin plank under 2000x gravity (7 cm sink) | stuck | stuck | **finishes** (one settling spike on first contact, as with every method) |
| Body spawned 15 cm inside the floor next to a seam | **falls through** | falls through | **pushed out on top**, keeps going |
| marblebuilds: marble over a one-way valve / over tiles that keep interior faces | kicked up 12 / 9.7 m/s | same | **fixed**, no special case needed |

**Both fix** (cases Semrau was designed for): an upright box on box tiles, on moving or kinematic platforms, and as a two-box compound player.

**Correctness is kept.** Real walls and cliffs still block (walls up to 80 m/s), and real lips over about 2 cm (15 mm and 30 mm misaligned tiles) still stop a sliding box. At 80 m/s Semrau's continuous-collision filter lets a box **pass through** a thin real wall; topological stops it.

**Why it generalizes.** The rules are pure geometry on edges:
- cancel faces shared between touching pieces;
- ignore faces lying behind the surface the body rests on;
- pick ghost vertices from the neighbouring edges;
- push buried bodies out through outer faces only.

They don't care whether a piece is a box, a slanted quad or a segment, whether it's static, kinematic or dynamic, or whether the body is round, pointed or flat.

## 2. What it does not change, because the behaviour is valid

These look like problems in the numbers but are real physics, and topological correctly leaves them alone:

| Case | What happens | Why it is right |
|---|---|---|
| Real steps and lips: a 15 mm step, a 9 mm lip on a rocking platform | a sliding box stops; a ball bumps | Geometry above the surface is real. Under about 2 cm of sink, faces *behind* the surface are ignored and faces *above* it are kept. |
| Real gaps: 0.25 m gaps next to moving platforms | a ball dips into the gap and bumps out | the gap exists |
| Tumbling polygons: pentagon and hexagon in Shape Parade / Hills | they hop as they tip over corners, even on a perfectly smooth chain | measured against the smooth-chain reference: equal lift-offs (4 vs 4, 7 vs 6) |
| Rocking at polyline corners: flat faces crossing a bend in a smooth chain | small rocking kicks on the reference itself | the surface really is a polyline |
| Dynamic crates compressing under a ball | a small push-out kick (0.2–0.4 m/s) | the crates sink 2–5 cm under load and push back |
| Crushing: a kinematic platform driving under a wall | the player is squeezed through | Box2D behaviour, the same with every method |
| Free flight: a ball leaving a crest faster than the surface curves away | it flies | the smooth reference does too (the hills are tuned so it doesn't) |

## 3. What remains unsolved or unknown

**Unsolved (measured):**

| Case | What happens | Why |
|---|---|---|
| Hollow chains under extreme load (thin plank, ≥ 3000x gravity) | falls through the chain | Box2D drops the contact before pre-solve runs. The method can remove or replace contacts but not create them. |
| Loose chain segments with degenerate default ghosts (ghost = endpoint) | one end cap is ignored | same reason; a fix to the default is in [ghost_collisions.md](ghost_collisions.md) §7 |
| Extreme penetration on box tiles (≥ 3000x gravity, about 10 cm sink) | occasional pop on push-out (one run: 344 m/s kick, still finished) | the push-out manifold at large depth is crude |
| Capsule on sliced hills | 1 airborne step (1.4 cm hop) vs 0 on the reference | marginal and not traced to a seam; it appeared with the crest fix below |

**Limits of the current implementation:**
1. **Designated bodies only.** World queries are not allowed in pre-solve, so the portable version snapshots each protected body's neighbourhood before the step. It does support several bodies at once (Shape Parade protects six).
   - In our fork, the same rules could run inside `contact.c` for every dynamic body, using a lock-free read-only broad-phase query during collide.
   - That would also remove the "cannot create contacts" limit.
2. **Memory and cost.** The topology is built once per step per protected body. That is about 400 KB of state per body and 1–3 µs extra per step in these scenarios. Building once per step did not measurably speed up these small scenes (one or two contacts per step); it should pay off in dense scenes.
3. **Unsupported terrain:** rounded polygons, circles and capsules are ignored when building the topology (they need radius-aware matching).
4. **Custom collision rules:** filtering beyond filter bits (custom filter callback, joints, one-way platforms in your own pre-solve) must be mirrored in `filterFcn`. Otherwise those shapes still shape the topology.
5. **Large worlds:** the neighbourhood query uses world-space float bounds, which lose precision far from the origin in double-precision builds.

**Unknown (not tested yet):**
- character movers (`b2World_CastMover`), which treat chain segments as plain segments and have no ghost handling;
- ragdolls and joint chains;
- dense scenes with many protected bodies (cost and memory at scale);
- fast bodies tilted past 45° hitting a seam vertex (the code review predicts a vetoed continuous-collision hit; not measured);
- seams between shapes on the *protected* body itself beyond the two-box compound case (e.g. a multi-shape vehicle chassis);
- tolerance edge cases: steps right at the 5 mm and 2 cm thresholds, curves bending close to the 60° "walkable continuation" limit.

## How it works, in plain terms

Before each step, the shapes near each protected body are snapshotted, and the local topology is built once:

1. **Cancel interior faces.** Where two pieces touch with opposite-facing coincident faces (two tiles side by side, two crate slices sharing a slanted cut), both faces are interior and are ignored. This is *localSeg* phase 1 from marblebuilds, done at the current poses and across bodies.
2. **Cancel hidden faces.** While the body rests on a surface (in front of it, over it, sunk less than 2 cm), a *steep* face lying behind that surface's line can only be reached by sinking through the surface, so it is ignored.
   - This catches a tile's side face, a moving platform's side face in a small gap, and the exposed corner after a one-way valve.
   - Faces within 60° of the surface's orientation are walkable continuations (a convex crest) and are kept.
   - Real obstacles rise *above* the line, so they also stay.
3. **Pick ghost vertices.** Each remaining edge takes its ghost vertices from the neighbouring remaining edges by the smallest turn (the localSeg rule). The contact is recomputed with Box2D's own chain-segment collider, which already knows how to ignore seams on a smooth chain.
4. **Interior relevance.**
   - Centre inside a solid piece: the body is pushed out through that piece's outer faces only, never sideways through a shared face.
   - Fully buried: the neighbour's shared face is kept as a wall so the body cannot tunnel along.
5. **Fast motion.** A continuous-collision hit is vetoed only if it lands on an interior part, or on a seam vertex with a normal no real face there could produce. Hits are judged at the pose the body started the step from.

## Using it in a game (stock Box2D v3.2)

```c
#include "topo_ghosts.h"   // shared/topo_ghosts.{h,c}, no engine changes needed

TopoGhostContext* topo = malloc( sizeof( TopoGhostContext ) );   // ~400 KB, keep off the stack
TopoGhost_Init( topo, worldId, playerBodyId );
topo->config.timeStep = 1.0f / 60.0f;
b2World_SetPreSolveCallback( worldId, TopoGhostPreSolve, TopoGhostPreContinuous, topo );
// every player shape: shapeDef.enablePreSolveEvents = true

// game loop
TopoGhost_PreStep( topo );           // snapshot + build topology (also turns off contact recycling on the player)
b2World_Step( worldId, dt, 4 );
```

For several protected bodies, use one context per body and a dispatching callback that forwards each contact to the right context. `shared/ghost_scenarios.c` (`MultiPreSolve` / `MultiPreContinuous`, which route each contact to its player) shows how.

## How it was verified

- **22 scenarios**, each run with none, Semrau and topological, at default settings and full parameter sweeps. Several have multiple bodies, reported per body against a smooth-surface reference.
  - Correctness checks: Tile Wall, Cliff Pit, CCD Back Face.
  - The earlier set of 33 was curated down: scenarios whose comparison duplicated another, or was identical for all three mitigations, were removed (list in the curation commit).
- **Review and testing:**
  - a code review found three critical issues, all fixed;
  - an adversarial tester found three regressions, all fixed;
  - traces found three more: ghosts picked from hidden faces, and flat bodies losing support at convex crests of a smooth chain under both covering rules. All fixed and re-measured.
- **Builds and tests:** unit tests pass, the samples build, and all Ghost Collisions samples smoke-run headless under Xvfb, including with topological on and four solver workers.

## Reproducing

```sh
just probe --defaults                                       # every sample at default settings
just probe "Parabola Hills"                                 # one scenario, full sweep
./build/bin/ghost_probe --players "Shape Parade" 3 3 6      # per body: scenario, variant, mitigation (0 none, 1 semrau, 3 topological), sweep value
./build/bin/ghost_probe --trace "Parabola Hills" 0 0 3      # per step state and contact normals
./build/bin/ghost_probe --bench "Slope Tiles" 4 20          # cost, none vs topological
just samples                                                # category "Ghost Collisions", use the Mitigation dropdown
```

**Source**
- Method: `shared/topo_ghosts.{h,c}`.
- Scenarios: `shared/ghost_scenarios.c`.
- Samples: `samples/sample_ghost_collisions.cpp`.
- Probe: `experiments/ghost_probe/main.c`.

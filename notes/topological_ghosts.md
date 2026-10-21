# Topological ghosts: results for game developers

Branch `ghostbusters` (Box2D v3.2.0+2). Written 2026-10-21.

This is the experiment that follows [ghost_collisions.md](ghost_collisions.md). Raw data:
- [ghost_defaults.md](ghost_defaults.md): every sample at its default settings.
- [ghost_probe_results.md](ghost_probe_results.md): full parameter sweeps.

## The short version

A *ghost collision* is when a body sliding or rolling across a flat surface made of several pieces catches on the seam between them, as if there were a tiny wall. It stops dead, gets kicked into the air, or tumbles.

Box2D's built-in fix is the **chain shape**. It works, but only for static terrain merged into one chain. The upstream sample's fix is **Semrau's filter**: it drops short, flat contacts. It works for upright boxes, but does nothing for balls, tilted or rotating shapes, or two-sided segments, and it fails under heavy load.

**Topological ghosts** is a new mitigation built on Box2D v3.2's pre-solve callback. For one "player" body it works out, every step, which edges of the nearby shapes form the real surface. It ignores the hidden ones and rebuilds the contact as if the surface were one smooth chain. Because it is recomputed each step at the current poses, it also works across separate bodies: moving platforms, dynamic crates, and pieces of different shapes.

At default settings the samples show the following.

**Topological fixes cases that Semrau cannot:**

| Scenario | No mitigation | Semrau | Topological |
|---|---|---|---|
| Ball rolling over dynamic crates | bumpy (kick 2.3 m/s, 26% speed loss) | same as none | **smooth** |
| Ball on a flush rotating platform | bumpy (kick 3.0 m/s) | same as none | **smooth** |
| Free (rotating) box sliding down a slope of triangle tiles | tumbles (kick 7 m/s, 2.8 m hops) | same | **slides smoothly** |
| Box held at a 10° tilt, sliding on its corner | bumpy (kick 1.7 m/s, 26% loss) | same | **smooth** |
| Box on two-sided segment tiles | stuck at the first seam | stuck | **smooth** |
| Small gaps between segment tiles | stuck | stuck | **smooth** |
| Box sliding down a wall made of segments | stuck | hangs | **slides** |
| 1 mm misaligned tiles | stuck | launched 1.2 m into the air | **smooth** |
| Fast diagonal landing on tiles | launched 18 m | launched 18 m | **smooth** |
| Two-wheel vehicle over a crest onto a tiled slope | bumpy | bumpy | **smooth** |
| Thin plank under 2000x gravity (7 cm sink) | stuck | stuck | **finishes** (one settling spike on the first contact, as with every method) |
| Body spawned inside the floor next to a seam | falls through the floor | falls through | **pushed out on top**, keeps going |
| Marble over a one-way valve (marblebuilds bug) | kicked up 12 m/s | same | **fixed**, no special case needed |
| Marble over tiles that keep interior faces (marblebuilds) | kicked up 9.7 m/s | same | **smooth** |

**Both fix:** upright locked boxes on box tiles, kinematic tiles, dynamic tiles, moving, rotating and elevator platforms, and a compound (two-box) player. These are the cases Semrau was designed for.

**Correctness, where blocking is the right answer:**
- Real walls, steps, cliffs and thin walls block with every method, including topological at 80 m/s.
- At 80 m/s, Semrau's continuous-collision filter **lets the player pass through** a thin wall. Topological stops it.

**Cost:** about 1–2 µs extra per step for one player (0.002–0.004 ms vs 0.002–0.008 ms without), measured with `ghost_probe --bench`.

## How it works, in plain terms

Every step, before Box2D's solver uses a contact between the player and some shape T:

1. **Look around.** Before the step, take a snapshot of the shapes near the player: polygons, segments and chain segments, on any body type, with their current poses. Box2D does not allow world queries inside the callback, but collision runs at the start of the step, so the poses match.
2. **Cancel interior faces.** Where two shapes touch with opposite-facing coincident faces (two tiles side by side), both faces are interior and are ignored. This is *localSeg* phase 1 from marblebuilds, but done per contact, at the current poses, across bodies.
3. **Cancel hidden faces.** While the player rests on a surface (it is in front of it, over it, and sunk less than 2 cm), any face lying behind that surface's line cannot be touched without sinking through the surface, so it is ignored.
   - This catches a tile's side face next to the floor the player stands on, a moving platform's side face in a small gap, and the exposed corner after a one-way valve.
   - Real obstacles rise *above* the line, so they stay. A tall box falling past a cliff is sunk far deeper than 2 cm, so the cliff face stays live.
4. **Pick ghost vertices.** Each remaining edge of T takes its ghost vertices from the neighbouring edges by the smallest turn (the localSeg rule). The contact is then recomputed with Box2D's own chain-segment collider, which already knows how to ignore seams on a smooth chain.
5. **Interior relevance.** These are the rules you proposed:
   - If the player's centre is *inside* a solid tile, it is pushed out through that tile's outer faces only, never sideways through a shared face into the next tile.
   - If the player is *fully* buried in one tile, the next tile's shared face is kept as a wall, so the player cannot tunnel along the floor.
   - Otherwise (on or near the surface), shared faces are cancelled.
6. **Fast motion.** A continuous-collision hit is vetoed only when it lands on an interior (cancelled) part of T, or on a seam vertex with a normal that no real face there could produce. Hits are judged at the pose the player started the step from.

The touched edge, its neighbours and the player are all just geometry, so the method covers circles, capsules and polygons at any angle, and static, kinematic or dynamic terrain.

## Using it in a game (stock Box2D v3.2)

```c
#include "topo_ghosts.h"   // shared/topo_ghosts.{h,c}, no engine changes needed

TopoGhostContext topo;     // must stay at a fixed address while the world lives
TopoGhost_Init( &topo, worldId, playerBodyId );
topo.config.timeStep = 1.0f / 60.0f;
b2World_SetPreSolveCallback( worldId, TopoGhostPreSolve, TopoGhostPreContinuous, &topo );
// every player shape: shapeDef.enablePreSolveEvents = true

// game loop
TopoGhost_PreStep( &topo );          // snapshot the neighbourhood (also turns off contact recycling on the player)
b2World_Step( worldId, dt, 4 );
```

**Requirements and caveats:**
- **Contact recycling** must be off for the player, because a recycled contact skips pre-solve. `TopoGhost_PreStep` does this.
- **Custom collision rules:** if you filter collisions beyond filter bits (custom filter callback, joints with `collideConnected = false`, one-way platforms in your own pre-solve), set `topo.filterFcn` to mirror them. Otherwise those shapes still shape the topology.
- **One designated body:** it works for a player body, not for every body in the world (see "What remains").

## What it solves

- **Seams between any pieces, on any bodies:** static tiles, kinematic and moving platforms, rocking platforms, elevators, dynamic crates, and pieces of different types (box, triangle, segment, chain segment).
- **Shapes other than upright boxes:**
  - balls (1-point contacts, which Semrau never sees);
  - polygons at any angle, free to rotate, which stops tumbling at seams;
  - capsules;
  - compound bodies.
- **Two-sided segments**, which neither chains nor Semrau handle.
- **Small misalignment and small gaps** (under about 2 cm), with real steps and lips above that kept.
- **Heavy load** (up to about 2000x gravity in the tests), where Semrau's fixed 2 cm threshold fails.
- **Sunk or spawned-inside bodies:** pushed out, not through. Stock Box2D drops a body 15 cm into a two-tile floor straight through it.
- **The two marblebuilds snags** (one-way valve, pass-through tiles), without per-block special cases.
- **High-speed seams** for continuous collision, without letting real thin walls be tunnelled.

## What it does not solve (measured)

| Case | What happens | Why |
|---|---|---|
| Hollow chains under extreme load (thin plank, ≥ 3000x gravity) | falls through | Chains are one-sided and Box2D drops the contact before pre-solve runs. The method can remove or replace contacts but not create them. |
| Degenerate loose chain segment ghosts (ghost = endpoint) | one end cap is ignored | Same reason: Box2D's collider drops the contact first. A one-line fix to the default ghosts is in [ghost_collisions.md](ghost_collisions.md) §7. |
| Real geometry | a 15 mm step stops a sliding box; a ball dips into a 0.25 m gap; a rocking tile's 9 mm lip bumps a ball | Correct behaviour, not ghosts. Tolerances are about 5 mm for "touching" and 2 cm for "sunk". |
| Extreme penetration (≥ 3000x gravity, about 10 cm sink, box tiles) | occasional pop on push-out (one run: 344 m/s kick, still finished) | The push-out manifold at large depth is crude (deepest points only) |
| Being crushed by a kinematic platform | player squeezed through | Box2D behaviour, the same with every method |

## What remains

1. **Every body, not just one player.** World queries are not allowed in pre-solve, so the portable version snapshots one body's neighbourhood before the step.
   - This branch is our own fork. The same rules can run inside the library, in `contact.c`'s manifold functions, for every dynamic body, using a lock-free read-only broad-phase query during the collide phase, which does not modify the trees.
   - That would also remove the "cannot create contacts" limit (the hollow-chain and default-ghost cases above).
2. **Build the scene once per step.** It is rebuilt per contact today. Building it in `PreStep`, in the player's frame, would cut most of the per-contact cost (O(edges²) cancellation).
3. **Unsupported neighbours:** rounded polygons, circles and capsules as *terrain* are ignored when building the topology. They need radius-aware matching.
4. **Fast tilted bodies** (from the code review, not measured): a fast body tilted past 45° hitting a seam vertex can have its continuous-collision hit vetoed. Push-out should recover it, but it is a known weak spot.
5. **Large worlds:** the neighbourhood query uses world-space float bounds, which loses precision far from the origin in double-precision builds.
6. **Not tested yet:**
   - character movers (`b2World_CastMover`), which treat chain segments as plain segments;
   - ragdolls and joint chains;
   - many simultaneous players.

## How it was verified

- **30 scenarios plus 7 legacy samples**, each with none, Semrau and topological. 12 of them were written by an adversarial tester specifically to break the method: real walls, steps, cliffs, thin walls, ceilings, crush, fast back-face hits, crests.
- **A code review** found three critical issues, all fixed and re-run:
  - edge-split overflow could delete real ground;
  - continuous-collision hits were judged at the end-of-step pose;
  - contact recycling silently disabled the method.
- **The adversarial tests** found three regressions, all fixed:
  - a hole at convex crests;
  - compound bodies using the body centre;
  - a continuous-collision back-face tunnel.
- **A final trace** fixed one more case: ghosts picked from faces that were themselves hidden.
- **Unit tests** pass, the samples build, and all 37 Ghost Collisions entries were smoke-run headless under Xvfb with no crashes. That run covered "none" and "topological" with four solver workers; Semrau was exercised only in the single-threaded probe.

## Reproducing

```sh
just probe --defaults          # the table above, every sample at default settings
just probe "Slope Tiles"       # one scenario, full sweep
./build/bin/ghost_probe --trace "Slope Tiles" 4 3 5   # per step: scenario, variant, mitigation (0 none, 1 semrau, 3 topological), sweep value
./build/bin/ghost_probe --bench "Slope Tiles" 4 20    # cost, none vs topological
just samples                   # category "Ghost Collisions", use the Mitigation dropdown
```

**Source**
- Method: `shared/topo_ghosts.{h,c}`.
- Scenarios: `shared/ghost_scenarios.c`.
- Samples: `samples/sample_ghost_collisions.cpp`.
- Probe: `experiments/ghost_probe/main.c`.

# Topological ghosts: how it works inside

The technical companion to [topological_ghosts.md](topological_ghosts.md). It covers what each rule does, exactly when it applies, and which rule actually removes the ghost in each kind of case. Everything here refers to `shared/topo_ghosts.c` on branch `ghostbusters`. Numbers come from `ghost_probe`, and from temporary instrumentation that is not committed.

## Pipeline per step

Notation:
- **Protected body (P):** the body being protected.
- **Neighbours:** the polygons, segments and chain segments near P, on any body type.
- **Edge:** one face of a neighbour, oriented so its outward normal is on the right of `p1 -> p2`, as in Box2D chains. Two-sided segments contribute one edge in each direction.
- **Piece:** the part of an edge that survives cancellation.

`TopoGhost_PreStep` (before `b2World_Step`, single-threaded):
1. **Snapshot.** Query the broad-phase around P, with margin `queryMargin + 2 |v| dt`. Copy each neighbour's local geometry, filter, bounds and body transform. Box2D forbids world queries inside pre-solve; collide runs at the start of the step with exactly these poses.
2. **Build the step scene.** For each player shape, in a frame at P's position with world axes:
   1. edges of every neighbour that P collides with (filter bits plus the optional `filterFcn`);
   2. adjacency: which neighbours' bounds touch, within 2 × linearTolerance;
   3. **interior cancellation** (rule 1), giving pieces;
   4. **supports** and **covered** flags (rule 2);
   5. **ghost vertices** for every live piece (rule 3).

`TopoGhostPreSolve` (Box2D worker threads, per contact of P with a shape T):
4. **Inside T?** If P's centroid is inside T (a polygon), build a **push-out** manifold along T's surviving outer face with the least depth (rule 4).
5. **Otherwise,** for each live (not covered) piece of T: transform it and its ghosts into T's body frame, and collide P against it as a `b2ChainSegment` with Box2D's public colliders. Keep the deepest result.
6. **Replace or cull.** Replace Box2D's manifold with the result, or set `pointCount = 0` if no piece produced one. The ids are keyed by T's edge index and interval, so warm starting stays stable.

`TopoGhostPreContinuous` (time-of-impact events):
7. Veto the hit only if it lands on a cancelled portion of T, or on a seam vertex with a normal that no live face there can produce (rule 5).

## The rules

| # | Rule | Applies when | Effect |
|---|---|---|---|
| 1 | **Interior cancellation** | Two different shapes have **antiparallel, coincident** edges: angle under 0.01 rad, both endpoints within 5 mm of the other's line, overlapping spans | The overlapping part of both edges is removed; slivers under 5 mm snap away. Exception: if P is fully inside one solid, the *other* solid's face is kept as a wall. |
| 2a | **Support covering** | P rests on a surface S: centroid in front of S's line, over its span, deepest point between −2 cm (speculative) and +2 cm (penetration tolerance) relative to it | Any piece whose endpoints are both on or behind S's line (within 5 mm) and whose normal is more than 60° from S's normal is **covered** (ignored). |
| 2b | **Vertex covering** | A piece meets another live piece C at a shared vertex, P rests on C (same depth band), is over C's span, and the piece lies behind C | The piece is covered, unless it is within 60° of C (a walkable crest continuation). |
| 3 | **Ghost vertices** | Every live piece | At each end, the ghost is the far end of the neighbouring live piece with the **smallest turn**: clockwise for ghost1, counter-clockwise for ghost2 (the localSeg rule). If a live surface P stands in front of hides this piece's end, the most concave such choice wins. Covered pieces are never candidates. No neighbour means a convex corner (ghost = vertex − normal). |
| 4 | **Push-out** | P's centroid is inside T | Push out along T's surviving outer face with the least depth, never through a cancelled face. |
| 5 | **Continuous veto** | A time-of-impact hit on T | Vetoed only if it lands on a cancelled part, or on a seam vertex whose normal matches no live face (dot < 0.7) and that is not a convex corner by the computed ghosts. Judged at the pre-step pose. |

## How "ignore the hidden surface" works when it can't just be cancelled

Cancellation (rule 1) only works when two shapes have faces that exactly overlap back to back, like two tiles side by side. Many hidden faces have no such partner:
- a tile's side face across a small gap;
- a moving platform's side face next to the floor;
- the outer side and bottom of the tile you're standing on;
- the face of the next block just past a one-way platform.

Covering (rule 2) handles these with one physical argument, in plain terms:

> If you are standing on a floor, and a face is entirely below that floor's line and steep (a wall going down, or an underside), you can only touch it by sinking into the floor. You're sunk less than 2 cm, so any contact with that face must be a ghost. Ignore it.

The three conditions on standing make it safe:
- **In front of the floor:** you are above it, not inside or below.
- **Over its span:** you are actually on it, not beside it. A tall box falling past a cliff is beside the floor, so the cliff face stays live.
- **Barely sunk:** your deepest point is within 2 cm of the floor line. If you sink deeper (heavy load, a fast impact), the rule turns off and real geometry takes over.

Faces that rise *above* the floor line (walls, steps, lips over about 5 mm) are never covered. Neither are faces within 60° of the floor's direction: those are the next stretch of the same surface over a crest, and you reach them by moving along, not by sinking.

Rule 2b is the same idea for a face that shares a vertex with the surface you stand on. It is what moves the corner of an exposed block next to a one-way valve onto the valve, the case marblebuilds fixed by hand.

## A ball above two flush static boxes: which rule does what

Instrumented per step: Crate Floor Circle, "static box tiles" variant, 10 m/s. There were 165 steps with two tiles in range.

| Rule | What it does each step |
|---|---|
| 1 Interior cancellation | removes the **2 touching side faces** at the seam (8 edges → 6 pieces) |
| 2 Covering | marks the other **4 pieces** covered: the two outer side faces and both bottoms. The two tops stay live. One support (the tile top under the ball). |
| 3 Ghost vertices | the two live tops get each other's far ends as ghosts, so the seam reads as a straight line |
| 5 Pre-solve result | the ball collides only with the tile tops, through the chain collider |

All three run every step, but the ghost vertices are the part this case depends on:
- **Ghosts off:** the ball bumps again (kick 2.0 m/s, 27% speed loss), exactly as without mitigation.
- **Cancellation or covering off:** no effect. Each one alone still removes the seam's side faces, so they back each other up.

Why the ghost vertices matter for a ball: the side faces are not the problem. A ball resting slightly sunk near the seam is within Box2D's 2 cm speculative distance of the next tile's top-left *corner*, so it gets a contact whose normal points back at it.
- With a ghost vertex continuing the line, the chain collider treats that corner as part of a flat surface.
- The ball's centre is in the neighbouring top's Voronoi region, so the collider returns no contact for that corner.
- Semrau has nothing equivalent, so it never helps balls.

## Which rule does the work, by case

Measured at default settings. Each rule was disabled in turn with temporary switches, and the topological verdict recorded.

| Case | All on | No cancellation | No covering | No ghosts | Cancellation and covering both off | Does the work |
|---|---|---|---|---|---|---|
| Upright box across flush box tiles | ok | ok | ok | ok | **stuck** | removing the hidden side face; rule 1 **or** rule 2, either suffices |
| Ball across flush static tiles | ok | ok | ok | **bumpy** (2.0 m/s) | bumpy | ghost vertices |
| Box held at 10° tilt (corner contact) | ok | ok | ok | **bumpy** (1.7 m/s) | ok | ghost vertices |
| Fast diagonal landing on tiles (40 m/s) | ok | ok | ok | **launched** | launched | ghost vertices (also used by the continuous veto to tell real corners from seams) |
| Six bodies down triangle-tile slope | ok | ok | **bumpy** (5.4 m/s) | **bumpy** (2.6 m/s) | bumpy | covering **and** ghost vertices |
| Small gaps between tiles (not flush) | ok | ok | **stuck** | ok | stuck | covering (there are no overlapping faces to cancel) |
| Body spawned 15 cm inside the floor | ok | **falls through** | ok | slowed | falls through | interior cancellation (P is not resting on anything, so covering can't apply) |

In short:
- **Ghost vertices** fix anything that touches the seam *corner*: balls, tilted or rotating polygons, fast bodies.
- **Hiding faces** fixes a flat face running into a face at the seam. Cancellation does it when the faces overlap; covering does it when they don't (gaps, moving pieces, exposed faces).
- **Cancellation** is also what makes push-out go through the top instead of sideways into the next tile.

## Tolerances and why

| Constant | Value | Role |
|---|---|---|
| linearTolerance | 5 mm (`B2_LINEAR_SLOP`) | coincident lines and shared vertices; misalignment up to this reads as flush |
| angularTolerance | 0.01 rad | antiparallel test for cancellation; convex-corner test in the continuous veto |
| penetrationTolerance | 2 cm (4 × slop) | maximum sink for "resting on"; deeper means real penetration, so rules 2a/2b turn off |
| speculative band | −2 cm (`B2_SPECULATIVE_DISTANCE`) | how far above a surface still counts as resting on it |
| continuation angle | 60° (dot > 0.5) | faces closer to the support's direction than this are walkable crests, never covered |
| continuous normal match | dot > 0.7 | a hit normal within about 45° of a live face counts as a real face hit |

## Known edge cases

- **Contact creation:** pre-solve only sees contacts Box2D's collider produced. Rules can remove or replace contacts, never create them (hollow chains under extreme load, degenerate loose-segment ghosts).
- **Threshold-sitting geometry:** behaviour around exactly 5 mm, 2 cm and 60° is untested.
- **Push-out at extreme depth:** the manifold uses the deepest player points only and can pop at about 10 cm of penetration.

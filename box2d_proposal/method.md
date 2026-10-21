# Method

Code: `shared/topo_ghosts.{h,c}` on `ghostbusters` (Box2D v3.2.0 + 2, `a292912`). It uses only the public API. Line numbers refer to `acda980`.

## One clarification first

It does not gather a shape's contacts and filter them as a set. Each pre-solve call independently **replaces** the single manifold for (protected shape P, touched shape T). It re-collides P against T's surviving edges as `b2ChainSegment`s, with ghosts computed from P's local neighbourhood. The neighbourhood topology is built once per step, before `b2World_Step`. Pre-solve is a lookup plus 1–8 chain-segment collisions.

The 1e-4 vertex-merge tolerance mentioned in the chat belongs to the original localSeg grid algorithm. This implementation uses `B2_LINEAR_SLOP` (5 mm) for all coincidence tests.

## Data flow

```
TopoGhost_PreStep(ctx)                        // serial, before b2World_Step
  snapshot = OverlapAABB(bounds(P) + queryMargin + 2|v|dt)   // geometry, filter, AABB, b2WorldTransform per shape
  for each shape S of P (max 2 cached):
    scene = BuildStepScene(S, snapshot)       // frame: P.position, world axes
      edges      <- outward-right edges of each neighbour (polygon CCW; segment both directions; chain segment 1)
      adjacency  <- owner pairs whose AABBs touch (2*slop)                     O(K^2)
      CancelInteriorEdges                     // rule 1                        O(A*8*8)
      CollectPieces                           // surviving intervals per edge
      supports   <- neighbour edges P rests on (rule 2a)
      covered[i] <- IsCovered(piece i)        // rule 2a/2b                    O(E*K_adj)
      ghost1/2   <- FindGhost(piece i)        // rule 3, covered excluded      O(E*K_adj)

TopoGhostPreSolve(A, B, manifold)             // b2UpdateContact, worker threads
  T = the non-P shape; owner = FindOwner(scene, T)   (miss -> per-contact rebuild, BuildLocalScene)
  if centroid(P) in T: local = PushOut(T outer pieces)                        // rule 4
  else: local = deepest of b2CollideChainSegmentAnd*(piece(T), P) over live pieces
  manifold = marshal(local) or pointCount = 0

TopoGhostPreContinuous(A, B, point, normal)   // b2SolveContinuous
  veto iff the hit lies only on cancelled parts of T, or on a seam vertex
  whose normal matches no live face and which is not convex by the computed ghosts   // rule 5
```

**Entry points:**

| Entry point | Location |
|---|---|
| `TopoGhost_PreStep` | `topo_ghosts.c:977` |
| `BuildStepScene` | `:945` |
| `ComputeTopology` | `:822` |
| `CancelInteriorEdges` | `:413` |
| `IsCovered` | `:560` |
| `FindGhost` | `:648` |
| `TopoGhostPreSolve` | `:1272` |
| `ReplaceManifold` | `:1167` |
| `PushOut` | `:1107` |
| `TopoGhostPreContinuous` / `JudgeContinuous` | `:1416` / `:1335` |

## Rules

Tolerances: lin = `B2_LINEAR_SLOP`, ang = 0.01, pen = 4·slop, spec = `B2_SPECULATIVE_DISTANCE`.

**Rule 1: interior cancellation.** For edges e and f of different, adjacent owners:
- **Test:** antiparallel (|cross| < ang), and both of f's endpoints within lin of e's line.
- **Effect:** subtract f's projected span from e's surviving intervals. Ends within lin snap away. On interval overflow (6 per edge) the edge stays uncut.
- **Exception:** if P is *fully* inside f's owner (every P point behind every face by its radius) and not fully inside e's owner, e is kept, as a wall against tunnelling between solids.

**Rule 2a: support covering.** A support is a neighbour edge s where:
- centroid(P) is in front of s's line, and its projection lies within s's span;
- −spec ≤ depth(P, s) ≤ pen, where depth is the maximum penetration of P's core points behind s's line plus P's radius.

A piece is **covered** if both its endpoints are on or behind a support line (within lin) and dot(n_piece, n_support) ≤ 0.5. Covered pieces are never collided, and never used as ghost candidates.

**Rule 2b: vertex covering.** A piece sharing a vertex with a live piece c is covered if:
- centroid(P) is in front of c and over c's span;
- the piece's far endpoint is behind c;
- −spec ≤ depth(P, c) ≤ pen;
- dot(n_piece, n_c) ≤ 0.5.

This generalises a hand-written rule from the user's game: the exposed face below a one-way platform lip.

**The 0.5 exemption.** A face within 60° of the covering surface is a walkable continuation (a convex crest), reached by moving along it, not by sinking. Without the exemption, flat-faced bodies lost all support for 2–3 steps at crest vertices of a smooth chain.

**Rule 3: ghost vertices.** For piece s = (a → b):
- **ghost1:** the start of the incoming live piece at a, by smallest clockwise turn from (b − a).
- **ghost2:** the end of the outgoing live piece at b, by smallest counter-clockwise turn from (a − b). This is the localSeg rule.
- **Preference:** before the angle rule, a live piece c at the vertex that P is in front of and that hides s takes priority. When several qualify, take the one giving the most concave corner.
- **Pass-through pieces** (T-junctions) contribute the part on the matching side.
- **No candidate:** convex corner, ghost = vertex − n.

Then `b2CollideChainSegmentAnd{Circle,Capsule,Polygon}` does the rest: Voronoi clamping for circles; `b2ClassifyNormal` skip/admit/snap for polygons and capsules (`manifold.c:1296`).

**Rule 4: push-out.**
- **When:** centroid(P) is inside T, and T is a polygon.
- **Manifold:** along T's surviving piece of least depth, using up to 2 deepest P points, separation = −depth.
- **Interaction with rule 1:** cancelled faces are never exits, so a body buried at a seam leaves through the top, not into the neighbour.

**Rule 5: continuous veto.** Judged on the pre-step topology, because poses are being written by other workers during `b2FinalizeBodiesTask`. Covering is not used here; it is a resting-contact concept. A hit is allowed if:
- it is not on T, or
- it is near a live piece whose normal is within about 45° of the hit normal (dot > 0.7), or
- it is near a live piece endpoint that is convex by the computed ghosts.

Otherwise it is vetoed.

## Manifold marshalling

At pre-solve time the anchors are relative to body **origins**: `b2UpdateContact` shifts them by `centerOffset` afterwards (`contact.c:682`).

```
normal_world = rotate(qT, local.normal)          // negate if P is shape A
fromT        = rotate(qT, local.point_i)
fromP        = fromT + (pT - pP)
anchorA/B    = P is A ? (fromP, fromT) : (fromT, fromP)
id           = 0x8000 | (((localEdge & 0xf)<<3 | interval & 0x7 | pushOut<<6) << 8) | (local.id & 0xff)   // stable step to step
```

Impulses are zeroed; Box2D re-matches ids for warm starting.

## Requirements and assumptions

- **Contact recycling off for P** (`b2Body_EnableContactRecycling`). A recycled contact skips pre-solve, so the replaced manifold would go stale. `PreStep` enforces it.
- **Pre-solve fires only when Box2D's own manifold has points** (`contact.c:642`). The method can remove or replace contacts, never create one Box2D's collider dropped. That matters for one-sided chain segments with degenerate ghosts, and for hollow chains under extreme load.
- **Collision filtering beyond filter bits** must be mirrored in `filterFcn`: the custom filter callback, joint `collideConnected`, and one-way logic in user pre-solve.
- **Unsupported shapes:**
  - Neighbours: circles, capsules and rounded polygons are snapshotted but contribute no edges.
  - Protected shapes: circle, capsule and polygon only; others pass through unchanged.
- **Caps:**
  - 128 neighbours, 8 edges per shape, 6 intervals per edge, 16 supports, 2 cached player shapes.
  - On neighbour overflow, the body's topology is skipped for that step (`context->bypass`) and native contacts are kept.
  - On other overflows, pre-solve keeps Box2D's manifold if the touched shape's pieces were truncated, and the continuous veto is disabled for that step.
- **Context size:** about 400 KB, because the step scene is in fixed arrays.

## Ablation

Each rule was disabled in turn, measured at the samples' default parameters.

| Case | Fixed by |
|---|---|
| Upright box across flush tiles | hiding the side face: rule 1 **or** rule 2, either suffices |
| Ball / tilted box / fast landing across flush tiles | rule 3. With ghosts off: kick 2.0 / 1.7 m/s / launched. |
| Six body shapes down triangle-tile slope | rules 2 **and** 3 |
| Tiles with 5 cm gaps (no coincident faces) | rule 2 |
| Body spawned 15 cm inside a two-tile floor | rule 1 (+ rule 4). With cancellation off, it falls through. |

For a ball over two flush static boxes, every step: rule 1 removes the 2 touching faces, rule 2 covers the 2 outer sides and 2 bottoms, and rule 3 links the two tops. Only rule 3 is load-bearing for the ball. The failure without it is a speculative vertex contact with the next top's corner, at 0.1 mm sink and about 1.3 cm separation.

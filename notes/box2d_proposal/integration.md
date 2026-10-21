# Integration options

The current module is external and uses only the public API, so its structure follows from what pre-solve allows. Below:
- what moving it into the engine would change;
- a concrete proposal;
- the open design questions.

## Constraints the external version works around

| Constraint | External workaround | Cost |
|---|---|---|
| `b2World_OverlapAABB` asserts unlocked, so no queries in pre-solve | snapshot the neighbourhood in `TopoGhost_PreStep` before `b2World_Step` | serial pass per protected body; geometry copied (about 400 KB fixed state per body) |
| pre-solve only fires for manifolds with points (`contact.c:642`) | none | cannot restore a contact the native collider dropped (degenerate chain-segment ghosts, hollow chains under extreme penetration) |
| recycling skips pre-solve (`physics_world.c:531`) | recycling forced off for protected bodies | loses recycling on those bodies |
| pre-continuous runs in `b2FinalizeBodiesTask` while other workers write transforms | judge on pre-step snapshot only | the veto can't account for the swept pose |
| one pre-solve callback per world | user-side dispatch by body | none in-engine |
| user filtering (custom filter, joint `collideConnected`, one-way pre-solve) is invisible | mirror it in `filterFcn` | user burden, easy to get wrong |

## Proposal: opt-in per shape, inside the contact update

**API sketch.**

```c
b2ShapeDef def = b2DefaultShapeDef();
def.enableTopologicalGhosts = true;   // this shape's contacts use local topology (dynamic shapes)
// optional world tuning
b2World_SetTopologicalGhostTolerances( worldId, linear, penetration, continuationCos );
```

**Where it runs.**
1. **Topology build: in the collide stage, per protected shape, in parallel.** During `b2Collide` the broad-phase trees and transforms are read-only. A per-shape task can query `world->broadPhase` directly; the snapshot copy disappears. It could also be built lazily the first time `b2UpdateContact` hits a protected shape this step, with a per-shape cache keyed by step index.
2. **Manifold: in `b2UpdateContact`, before the pre-solve callback** (`contact.c:587`). Replace `fcn( shapeA, shapeB, relativeTransform, cache )` for protected pairs with the topological variant:
   - It returns a `b2LocalManifold` in frame A, so there is no marshalling hack.
   - It can return a contact even when the native collider would not, which removes the "cannot create contacts" limit.
   - User pre-solve still runs afterwards.
3. **Recycling.** Recycling stays valid if the cache key includes the touched shape's neighbourhood version, e.g. a per-shape topology hash, or "no neighbour moved by more than the recycle distance". In the simplest version, recycling stays off for protected shapes, as now.
4. **Continuous.** In `b2ContinuousQueryCallback` (`solver.c:157`, chain-segment early-out at `:235`), extend that early-out to protected pairs, using the cached topology's cancelled spans and convex flags. That replaces the after-the-fact veto, and the topology is from the same step.
5. **Filtering.** In-engine, neighbour acceptance reuses `b2ShouldShapesCollide`, the custom filter callback and joint edges, the same predicate the broad-phase pair logic uses, so `filterFcn` goes away.

**Sharing.** Static and kinematic geometry is the bulk of every neighbourhood. Two options:
- **Cache static topology per static body, invalidated on shape add/remove/modify.** Then only the dynamic-neighbour part and the player-dependent flags (inside / support / covering) are per-protected-shape. Rule 1 and the edge lists are player-independent except for the fully-inside exception, so they are the cacheable part.
- **Precompute static ghosts at creation time,** as chains do, and run only the dynamic and covering parts per step. That is close to what `b2CreateChain` does today, extended to arbitrary polygon sets.

## What changes in cost

See [performance.md](performance.md) for measurements of the external version. Expected in-engine:
- **No snapshot copy, no 400 KB per body.** Scratch can come from the arena allocator per task.
- **Parallel topology build** (currently serial in `PreStep`).
- **Cacheable static part**, which removes most of the O(K²) adjacency and O(A·64) cancellation for static terrain.
- **Remaining per-protected-shape work:** supports, covering and ghost selection, O(E·K_adj) on the shapes near P.

## Trade-offs and open questions

1. **Scope: player-only vs all dynamic shapes.**
   - Per-shape opt-in keeps the cost where users want it.
   - "All dynamic shapes" would need the shared static cache to be affordable, and raises the question of dynamic-vs-dynamic topology, which is player-dependent on both sides.
   - The measurements so far cover 1–64 protected bodies.
2. **One-sided semantics.**
   - The method recollides polygon edges as one-sided chain segments. For solid polygons that's safe because push-out (rule 4) handles centroid-inside.
   - Two-sided segments contribute both directions.
   - Whether chain shapes themselves should go through the same path (adding solid push-out for chains) is open. The external version can't add contacts, so it can't fix hollow chains.
3. **Thresholds.**
   - The 2 cm penetration tolerance (4·slop) and the 60° continuation limit are empirical.
   - They interact with soft-contact penetration under load: rule 2 turns off above 2 cm sink.
   - Untested at the edges: 5 mm and 2 cm steps, about 60° crests.
4. **Rounded polygons and circle/capsule terrain** contribute no edges. In-engine they could contribute offset edges or arcs; unexplored.
5. **Warm-start ids.** The external version keys ids by (touched edge, interval, push-out). In-engine, the chain-segment feature ids could be namespaced the same way.
6. **Determinism.** The external build iterates neighbours in `OverlapAABB` order, which is deterministic for a given tree. In-engine it should sort by shape id to be independent of tree state, for cross-platform determinism and replays.
7. **Recording / replay.** No new world state if the topology is derived per step. A static cache would need to be rebuilt on replay seek.
8. **Not covered yet:** the character mover (`b2World_CastMover` / `b2SolvePlanes` treat chain segments as plain segments), sensors, and shape casts. The same topology could serve casts (ignoring cancelled and covered faces) for character controllers.

## Minimal first step

If a smaller change is preferred:
1. Allow pre-solve to run for manifolds with zero points, opt-in per shape.
2. Allow read-only world queries inside pre-solve during collide.

Those two would let the external module drop the snapshot and the "cannot create contacts" limit without moving any of the algorithm into the engine.

# Ghost collisions in Box2D v3.2: what is solved, what is not

Branch `ghostbusters`, Box2D v3.2.0+2 (`a292912`). Written 2026-10-21 as groundwork for experiments that detect ghost collisions and try to resolve them in a new way.

**Follow-up:** the per-contact topological ghost idea from section 6 has been implemented and measured. See [topological_ghosts.md](topological_ghosts.md).

Every claim marked **measured** comes from `experiments/ghost_probe`. The full tables are in [ghost_probe_results.md](ghost_probe_results.md), and each scenario can also be watched in the samples app under **Ghost Collisions**. Claims marked *code reading* were traced through the source but not run.

## TL;DR

- **The library algorithm has not changed since v3.0.**
  - Chains (ghost vertices, one-sided, Voronoi / Gauss-map classification) still solve ghost collisions *inside a single chain or a set of loose chain segments with correct ghosts*.
  - Measured: no snags at up to 100x gravity, 30 mm steps, or 30 degree tilted boxes.
- **v3.1 and v3.2 added hooks, not a fix.**
  - The hooks are an editable pre-solve manifold, a pre-continuous callback that can veto a time-of-impact (TOI) hit, and per-body contact recycling control.
  - Semrau's pre-solve technique ships only as the upstream sample *Character / Ghost Culling* (`695700d`, issue #1017). It is not in the library.
- **localGridSeg (marblebuilds) solves the tile-seam problem by topology.** It cancels interior faces and assigns chain ghosts locally, which is our clean *merged surface* case. Two game hooks then patch specific leftovers: a covering ghost for one-way valves (D1) and per-entity pre-solve rejection for pass-through blocks (D2). See section 4.
- **Semrau's filter works where it was designed to: axis-aligned boxes on box tiles.** Measured, it also fixes kinematic-tile and dynamic-tile seams. It fails on:
  - two-sided segment tiles (the "contact area" is the segment length);
  - point-face contacts (tilted boxes, vertex-vertex, circles);
  - sub-slop misalignment (a sporadic 1-point launch);
  - fast diagonal landings.
- **The two marblebuilds bugs reproduce in plain Box2D, and the root cause is not the "sink".**
  - The marble needs only about 0.1 mm of penetration.
  - A convex vertex that is *flush with the travel surface* sits inside the 2 cm speculative margin, so it produces a speculative vertex contact. Its normal is tilted against the motion and launches the marble (12 m/s up, 2 m hop).
  - This is a *ghost-vertex topology* problem: which neighbour a vertex's Voronoi region is bounded by. It is not a solver-depth problem.
- **Chains are hollow (Semrau's point, measured).** Under extreme forced gravity (3000x) a thin plank falls straight through a chain or custom-ghost segments. Box tiles hold it at about 10 cm of sink.
- **Our custom ghost code has an asymmetry bug** (measured, section 7). Loose chain segments whose ghosts default to their own endpoints lose the end cap at `point1` but keep it at `point2`.

## 1. What v3.1 / v3.2 changed

| Version | Change | Effect on ghost collisions |
|---|---|---|
| v3.1 | `b2SmoothSegment` became `b2ChainSegment`; per-segment materials | Mixed materials no longer force separate shapes, which removes one source of seams |
| v3.1 | Pre-solve also runs in continuous collision; "reduced clipping into chain shapes by fast bodies" (`2c939c2`) | Fewer tunnelling dips at chain vertices |
| v3.2 | Chain CCD early-out gains `B2_CORE_FRACTION` and a `sep2 > core` guard (`fb8b81f`, #956) | Better tunnelling prevention against chains only |
| v3.2 | World shape casts ignore chain segments hit from the back (`fcc60b7`) | Query-side only |
| v3.2 | Contact recycling, on by default; `b2BodyDef::enableContactRecycling` (`04b0e92`) | A recycled contact skips pre-solve until about 5 cm / 11 degrees of motion |
| v3.2 | Redesigned `b2PreSolveCallbackFcn` (edit the manifold or set `pointCount = 0`) and new `b2PreContinuousCallbackFcn` that can veto a TOI (`d710ba7`) | These are the hooks Semrau-style filters need |
| v3.2 | Loose chain segments: `b2CreateChainSegmentShape` and `b2Shape_SetChainSegment` (`241aa82`, #1016) | Per-segment ghosts without a chain; used by marblebuilds and by our `b2Custom*` API |
| v3.2 | Open chains take explicit `ghost1`/`ghost2`, and every point is collidable (`a38faae`, #1107) | Two chains can now be joined end to end with matching ghosts |
| v3.2 | *Ghost Culling* sample (`695700d`) | Semrau's technique, in the sample only |

*Code reading:* after normalising names, `b2CollideChainSegmentAndPolygon` and `…AndCircle` (`src/manifold.c`, around lines 1112–1713) are algorithmically identical across v3.0, v3.1 and v3.2.

**Key constants**

| Constant | Value |
|---|---|
| `B2_LINEAR_SLOP` | 5 mm |
| `B2_SPECULATIVE_DISTANCE` | 4 × slop = 2 cm |
| Chain convexity tolerance | 0.01 (about 0.57 degrees) |
| `B2_CORE_FRACTION` | 0.25 |

## 2. Semrau, "Dealing with Ghost Collisions" (Oct 2023)

<https://briansemrau.github.io/dealing-with-ghost-collisions/>

**Technique (Box2D v2 `PreSolve`).** Disable a player contact for one step when all of these hold:
1. `worldManifold.pointCount > 1`, i.e. a face-face contact.
2. The normal opposes the player's velocity (code: `b2Dot(normal, v) >= 0` after orienting the normal).
3. The two manifold points are closer than `4 * b2_linearSlop`, i.e. a small overlap.

The idea: a genuine wall contact has a large face overlap, while a seam contact's overlap is only as deep as the player has sunk into the floor. As soon as the overlap grows, the contact comes back.

**Pros**
- Works across bodies and shapes: moving platforms flush with terrain, dynamic crates, one-way platforms. Chains cannot do any of this.
- Keeps solid polygons, which push out (chains are hollow).
- About 20 lines; no preprocessing (no polygon union, no chain building).

**Cons** (the author's own caveats in quotes, ours unquoted)
- "I haven't designed this for anything other than axis-aligned rectangles."
- "You may still encounter ghost collisions with point-face contacts."
- It is a heuristic on contact *shape*, not geometry. It cannot tell a seam from a real 1–2 cm lip, so it has a load-dependent threshold (C2).
- It needs fresh manifolds every step, so contact recycling must be off for the player.
- CCD does not see the manifold. Upstream therefore pairs it with a pre-continuous veto keyed on a floor material id plus a horizontal normal. That is a second heuristic.
- It is per-player and per-step policy in user code. Every new shape or contact type needs its own rule.

**How Box2D adopted it.** `samples/sample_character.cpp:873` (`GhostCulling`). That sample:
- requires `pointCount == 2`;
- culls when `b2Distance(anchorA0, anchorA1) < 4 * B2_LINEAR_SLOP`;
- disables contact recycling on the player;
- vetoes horizontal CCD hits against tiles tagged `userMaterialId = 42`.

Our probe's `semrau` mitigation (`shared/ghost_scenarios.c`, `GhostPreSolve` and `GhostPreContinuous`) is a line-for-line port of that sample.

## 3. Results (measured)

Defaults are 60 Hz, 4 substeps and gravity −10. Probe column legend:
- **time**: progress divided by the ideal progress.
- **minV**: minimum tangential speed divided by launch speed.
- **kick**: maximum velocity along the surface normal.
- **hop / sink**: maximum displacement above / below the resting height.

| # | Scenario | Variant | Result | Semrau |
|---|---|---|---|---|
| C1 | Sliding Box, 0.5 m locked box at 10 m/s | box tiles | **Stops dead at the first seam** (minV 0, time 0.03) at every load 1–100x | Fixes it at all loads |
| C3 | Sliding Box | two-sided segment tiles | **Stops dead** | **Does not fix it**: the segment's length makes the "contact area" large |
| — | Sliding Box | custom ghost segments / chain | Clean at 1–100x | n/a |
| C10 | Sliding Box | Semrau with recycling **on** | Still clean | Not reproduced: each seam is a new shape pair, so its first manifold is fresh |
| C11 | Misaligned Tiles, alternate tiles raised by h | box tiles | Stops at the first lip for every h from 1 to 30 mm | h = 5, 15 mm: climbs correctly. h = 30 mm: wall, correctly kept. **h = 1 mm: launched** (vy +4.8 m/s, speed 10 → 6, 1.2 m hop) at step 154 |
| C11 | Misaligned Tiles | custom ghost segments / loose chain segments | Clean for all h, including 30 mm; the box rides over each step at full speed (likely because the ghosts make each lip read as a slight bend; not traced) | n/a |
| C4 | Tilted Box, locked at 2/10/30 degrees, sliding on a corner | box tiles | Kicks 0.35 / 1.7 / 4.0 m/s, speed loss to 0.97 / 0.74 / 0.30, hop up to 1 m | **No effect** (1-point contacts) |
| C4 | Tilted Box | chain | Clean | n/a |
| C5 | Capsule, r = 0.25 | box / segment tiles / chain | Clean at 1–40x | Rounding is enough for sliding |
| C6 | Thin Plank, 2 × 0.08 m, 5 m/s, forced gravity | chain / custom ghost segments | Clean up to 2000x. **Falls through at ≥ 3000x** (sinks by kilometres) | n/a |
| C6 | Thin Plank | box tiles | Snags at seams at every load (ordinary C1). At 3000–4000x it holds at 10–14 cm of sink; it does not fall | n/a |
| C7 | Fast Landing, box with (v, −15) onto 0.5 m tiles | chain, v = 20/40/80 | Clean, time 0.997 (no measurable CCD hitch) | n/a |
| C7 | Fast Landing | box tiles | v = 20: stops at a seam. **v = 40/80: launched** (hop 18 / 73 m) | Fixes v = 20; **does not fix v = 40/80** |
| C13 | Kinematic Seam, flush kinematic tile between static floors | chain or box tiles | **Stops dead** at the kinematic tile | Fixes it |
| C14 | Dynamic Tiles, 20 resting dynamic boxes | — | **Stops dead** | Fixes it, even at 40x |
| D1 | Valve Corner (marblebuilds), marble r = 0.2, g = 20, about 34 m/s off a curve | local ghost rule | **Snag + launch** at start phases 0 and 0.2: kick 12–13.7 m/s, hop 1.9–2.4 m, speed to 0.79–0.84 | n/a (circle) |
| D1 | Valve Corner | ghost routed via the valve's far endpoint (the marblebuilds fix) | Clean at all phases | n/a |
| D1 | Valve Corner | solid under the valve (no exposed face) | Clean | n/a |
| D1 | Valve Corner | either rule on a kinematic ground body | Same as static (phase 0 snags with the local rule) | n/a |
| D2 | Interior Faces (marblebuilds), each tile a closed loop | per-tile loops | **Launch at the first seam** for v = 10/25/50 (kick 2.6 / 9.7 / 2.1 m/s, hop up to 2.45 m). v = 100 happened to be clean | n/a |
| D2 | Interior Faces | merged surface | Clean | n/a |

### Mechanism notes from traces (`ghost_probe --trace …`)

- **D1 / D2: speculative convex-vertex contact.**
  - At step 18 the marble is at y = 0.1999 (only 0.1 mm low), 7.5 cm before the exposed corner (3, 0).
  - The circle-to-corner distance is 0.2133, so the separation is 1.3 cm: inside the 2 cm speculative margin.
  - Box2D's circle-vs-chain test keeps the vertex contact, because the corner is classified convex (ghost1 = the tile's own top). It produces normal (−0.35, 0.94) against the motion, and the next step shows vy = +12.2 m/s.
  - **The sink that marblebuilds blamed is not required.** The real issue is that a vertex is reachable even though it is "covered" by a flush surface belonging to a *different* shape. Only routing the ghost through the covering surface (the marblebuilds fix) removes it.
  - The effect is phase-dependent: it only fires when a step starts within the speculative window before the corner, so roughly window / (v·dt) of runs snag. This is why some speeds look clean.
- **C11 launch with Semrau on.** At step 154 the manifold is a 1-point contact with normal (0.62, −0.78) from the player's side and separation 1.2 mm. It is a speculative vertex-vertex contact as the box steps *down* off a raised tile (the polygon "disjoint edges → vertex-vertex" path in `b2CollidePolygons`). Semrau only inspects 2-point contacts, so it passes through.
- **C6.** Once the plank's centroid passes the segment line, the chain-segment collider returns an empty manifold (one-sided, *code reading*: `manifold.c` around line 1376), so nothing pushes back. Box tiles keep pushing out from inside.

## 4. What localGridSeg, localSeg and the marblebuilds hooks solve

All marblebuilds paths below are relative to `~/projects/marblebuilds`.

The game builds its terrain in three layers:
- **localGridSeg:** turns grid tiles into one-sided loose chain segments and assigns their ghosts.
- **localSeg:** an experiment that generalises the same algorithm off the grid.
- **Hooks:** the game's own rules on top, for block types that the geometry alone cannot express.

Box2D is a custom v3.2.0 fork with `mb*` functions that match our `b2Custom*` loose-segment API.

### 4.1 localGridSeg (production)

Files: `src/utils/experiments/segments.ts` (`Segmenter`) and `src/core/simulation/physics/segmentBodies.ts`; write-up in `benchmarks/localGridSeg/local-grid-seg-handwritten.md`.

**The problem it targets.** Grid-aligned tile hitboxes produce the classic ghost collision (C1). The earlier fix was a global polygon union (Martinez–Rueda–Feito) fed into `b2CreateChain`. It worked, but every add, remove or move meant recomputing the whole world.

**What it solves**
1. **Interior seams, incrementally.**
   - Phase 1 sweeps each dirty cell boundary and cancels coincident opposite-facing edges, including partial overlaps.
   - What survives is the outline of the union, so there are no internal faces.
   - This is our *merged surface* / *chain* configuration, which is clean in every probe scenario (Sliding Box, Misaligned Tiles, Tilted Box, Interior Faces).
2. **Chain-quality ghosts without building chains.**
   - Phase 2 picks each segment's ghost at a vertex from the neighbour with the smallest clockwise / counter-clockwise turn.
   - This reproduces chain ghosts on the union outline.
   - It also handles vertices touched by three or more segments (corner-touching tiles), which a single `b2ChainDef` cannot represent.
3. **Locality.** Only dirty cells and vertices are recomputed. In C++, a 100×100 world rebuilds in about 4–18 ms (benchmarks in the write-up), so tiles can be added, removed and moved at runtime.
4. **Seams across bodies (reasoning, not measured).**
   - Every cell is its own body, and ghosts are plain coordinates pointing at neighbouring cells' points.
   - So segments on different bodies still get correct ghosts while those bodies stay fixed relative to each other.
   - This covers the static part of C13. Moving groups are teleported, and their ghosts are not refreshed yet (commit `4d379068`: "the segments arent updated yet").

**What it does not solve by itself**
- **Faces it deliberately keeps (`shouldPreserveSegs`, `segmentHelpers.ts:456`).**
  - It keeps both faces when the blocks have different types, are both one-way, or exactly one is destructible.
  - Those kept faces are antiparallel interior faces: the D2 class, which our *per-tile loops* variant shows snapping marbles upward at 2–10 m/s.
  - Two blocks that can both be cut (`canCutPolygon`) still cancel first. It is worth checking which type pairs fall through to the `type1 != type2` branch.
- **Exposed faces next to open one-way cells.** The local angle rule gives them a legitimate convex corner, which is D1 (fixed by hook 1 below).
- Speculative convex-vertex contacts in general (section 3), the hollow-chain push-out (C6), and anything that depends on which entity is colliding.

### 4.2 localSeg / "localsuperseg" (experiment, geometry only)

Files: `benchmarks/localSeg/` (`idea.md`, `plan.md`, `localsuperseg.md`, `ts/`, `web/`).

**What it solves**
- It drops the grid assumption. Phase 1 becomes a bucketed sweep over arbitrary lines, so cancellation works for any non-overlapping polygon partition: bricks, uneven bricks, Voronoi cells, rotated rectangles, triangles, gaps.
- It also handles antiparallel, collinear, partly overlapping edges, such as a half-height slab beside a full block.
- Phase 2 (the angle rule) is unchanged.
- Tolerances: 1e-6 position, 1e-3 rad angle, vertex hashing quantised to 1e-4.

**What it does not do.** It is never connected to Box2D and has no physics measurements. In principle it extends section 4.1's topology result to non-grid levels. Physically it is unverified. In particular, nobody has checked whether the 1e-4 quantisation creates sub-slop misalignment (C11) or degenerate segments near the 0.01-rad convexity tolerance (C8).

### 4.3 marblebuilds hooks on top of localGridSeg

| Hook | Where | Mechanism | What it solves | Limits |
|---|---|---|---|---|
| **Covering one-way ghost** | `segments.ts:663,707` (`_findCoveringOneWay`, commit `48919534`); `notes/bugfixes/one_way_segments.md` | Static ghost reassignment. A non-one-way segment that shares a vertex with a one-way segment and lies behind it gets its ghost from the one-way segment's far endpoint, so Box2D hands that vertex's region to the one-way segment | D1. Game sweep went from 12/24 failures to 24/24 finishes (30/30 at 51–280 m/s, 4/4 at 400 m/s). **Our probe:** local rule snags at phases 0 / 0.2 (kick 12–13.7 m/s); covering rule clean at every phase. Our traces refine *why* it works: it removes a speculative convex-vertex contact, and the "sink" is not needed | One-way segments only. The game note says it fails once the sink exceeds the marble radius. Any other covering surface from a *different* shape (pass-through, glass, a moving body) is not handled |
| **Per-entity interior-face rejection** | `Box2dHandling.ts:704-708,786` (`isInteriorFaceFor`, commit `80b7af4a`); `notes/bugfixes/filter_and_wall.md` | Pre-solve. Each axis-aligned unit face knows the cell it faces. The contact is dropped if that cell is solid *for this entity*, and the face or the cell belongs to a pass-through block (filter, grate, speed-pass). Glass, breakables and trapdoors are excluded because they can open | D2 for pass-through blocks. Game: 33/64 → 0/64 kicks at 25–101 m/s. This is the only fix in the stack that is **per entity**, exactly what per-shape ghosts cannot express ("one side stops snagging, the other side starts") | Axis-aligned unit faces only. Needs `enablePreSolveEvents` on every segment. Under v3.2 contact recycling a recycled contact skips pre-solve, so it relies on the contact being new or recycling being off for marbles (not verified in the game). Glass / breakable / trapdoor seams keep their interior faces and remain D2 candidates |
| **Back-face culling** (disabled) | `segmentBodies.ts:39` `BACKFACE_CULLING = false` | Pre-solve. While an entity touches a segment, it ignores segments behind that segment's infinite line | Intended for D1 / D2. **Failed:** its trigger excluded one-way blocks; the snagging corner lies *on* the line; begin-contact events arrive a step late; and it let marbles fly through concave pocket walls | The lesson: culling by plane is not topology. A half-plane cannot tell "covered" from "around the corner" |
| **Preserve destructible faces** | `shouldPreserveSegs` (`segmentHelpers.ts:456`); `notes/bugfixes/going_too_fast_glass.md` | Keep both faces wherever exactly one side is destructible | Tunnelling after glass breaks: the neighbour's cancelled face used to be missing. Not a ghost fix | Brings back interior faces at every glass/solid seam, which `isInteriorFaceFor` deliberately does not cover. This is a D2 risk (not measured) |

### 4.4 Net effect

Mapped onto the classes in section 5:

| Class | Covered by | Status |
|---|---|---|
| 2 (no ghost info on tiles) | localGridSeg | Covered, by turning tiles into a union outline with chain ghosts |
| 1 (cross-shape vertex topology) | Covering one-way hook, interior-face hook | Covered only for one-way and pass-through blocks, each with its own special case |
| 3 (point contacts) | — | Not addressed; marbles are circles, but box entities exist (`notes/bugfixes/dropbug.md`) |
| 4 (hollow chains) | — | Not addressed |
| Moving groups | — | Not addressed (stale ghosts) |

Each new kind of covering surface so far has needed a new hand-written rule. That is the gap a general, topology-aware per-contact test (section 6, first bullet) would close.

## 5. What remains unsolved

The table shows what is left under each fix applied alone, and under both together. "Semrau" means the upstream *Ghost Culling* filter (pre-solve and CCD veto, recycling off). "localGridSeg" means localGridSeg plus the two active marblebuilds hooks (covering one-way ghost, per-entity interior-face rejection). Cells are tagged:
- **(m):** measured by the probe.
- **(g):** reported by the game's notes.
- **(r):** reasoning only.

| Class | Semrau only | localGridSeg only | Both |
|---|---|---|---|
| **1. Cross-shape vertex topology.** A vertex covered by another shape's surface (D1, D2, per-tile loops) | **Unsolved.** The failing bodies are circles, which have 1-point manifolds, so the filter never fires (m: Valve Corner, Interior Faces are unaffected by design) | **Solved only for one-way valves and pass-through blocks** (m for D1; g for D2). **Unsolved** for every other face it keeps: glass / breakable / trapdoor seams, and different-type pairs that are not both cuttable (r) | Same as localGridSeg only |
| **2. Seams with no ghost information.** Polygon tiles (C1), two-sided segments (C3), cross-body seams (C13, C14), fast landings (C7) | **Solved for axis-aligned boxes** on box tiles, kinematic tiles and dynamic tiles (m). **Unsolved:** segment tiles, fast diagonal landings at 40–80 m/s, any non-box player (m) | **Solved for static tile seams**: the outline is merged and the ghosts are chain-quality (m: the merged / chain variants are clean). **Unsolved:** moving groups (ghosts not refreshed, g), and seams between dynamic bodies such as crates (r) | Static seams: solved. Moving or dynamic seams: solved for **axis-aligned boxes only**. Circles and other shapes on moving or dynamic seams: **unsolved** (r) |
| **3. Point contacts.** Tilted / rotating polygons, vertex-vertex speculative contacts (C4, C11 launch) | **Unsolved**; it can even cause one: the 1 mm misalignment launch happens *with* the filter on (m) | **Solved on merged surfaces**: chain snapping (m: Tilted Box and Misaligned Tiles chain variants are clean). **Unsolved** wherever class 1 or 2 is unsolved (r) | Same as localGridSeg only. Semrau adds nothing for 1-point contacts |
| **4. Hollow one-sided chains.** No push-out once the centroid is behind the line (C6) | Not applicable: polygons push out | **Unsolved**: a thin plank falls through at 3000x (m). The game note's valve fix also "fails once the sink exceeds the marble radius" (g) | **Unsolved.** Semrau does not touch chain penetration |
| **5. Policy in user callbacks.** Hand-tuned thresholds and special cases | Load-dependent `4 * slop` threshold; CCD veto keyed on a material tag and a horizontal normal (m: it fails on the fast landings) | One rule per block type (one-way, pass-through); back-face culling already failed (g) | Both sets of rules, plus an interaction to verify (r): contact recycling skips pre-solve, so with recycling on, neither Semrau nor `isInteriorFaceFor` re-checks a recycled contact. The probe measured no effect for Semrau at tile seams, because those contacts are always new |

**After both fixes, what remains:**
1. Covered vertices at faces that are kept but not one-way or pass-through (class 1).
2. Moving or dynamic seams for non-box bodies, which is most of a marble game (class 2).
3. Point contacts at those same seams (class 3).
4. Chain penetration and push-out (class 4).
5. Fragile, hand-written policy (class 5).

**Semrau and marblebuilds barely overlap.** The game's main entities are circles, and Semrau never fires for circles. Its only contribution to marblebuilds would be box entities on seams that are moving or between bodies.

Not reproduced, but plausible (*code reading*, worth a targeted scenario):
- contact recycling keeping a stale normal across a convex crest or cliff edge (C10 variant);
- chain convexity flip-flopping on curves near the 0.01 bend tolerance (C8);
- grazing shape casts hitting internal chain endpoints (C15);
- chains on moving bodies with stale ghosts.

## 6. Directions for a new resolution

All of these are ideas for the upcoming experiments; none are implemented.

- **Per-contact topological ghosts.** At manifold time, query the broad-phase for shapes covering a vertex's neighbourhood (a flush or overlapping surface within slop) and treat that vertex as concave / snapped. This targets classes 1 and 2 without preprocessing, and works across bodies.
- **Swept-direction admission.** Reject a speculative or shallow contact whose normal points against the motion when the shape's swept path stays on the supporting surface's side. This generalises Semrau from "2-point, area < 4·slop" to any manifold, including 1-point contacts (class 3).
- **Support-surface continuity.** Track the surface the player was supported by. At a seam, accept a new contact only if its normal is within ε of the previous support normal, or it is a genuine step taller than the current penetration plus slop. Penetration is known from the previous step's separation, which fixes Semrau's load dependence (C2).
- **Solid chain fallback.** For one-sided chains, run a point-in-polygon or winding-number test only when the centroid is behind the line, to recover push-out (class 4). This is the 2020 Box2D post's reason for avoiding two-sided chains; it may be cheap enough if done only in that branch.
- **Detection first.** Flag a ghost collision when a contact's normal impulse has a component against the tangent velocity while the body's previous support normal is unchanged and no geometry rises above the support line within the body's extent. The probe's kick/snag metrics are a coarse version of this, usable as ground truth.

## 7. Issues found in our custom code

- **Asymmetric default ghosts (measured, Default Ghosts scenario).** `b2CustomCreateChainSegmentShape` (and upstream `b2CreateChainSegmentShape` with ghost = endpoint) produces a zero-length ghost edge.
  - In circle collision the `point1` side test is `dot(p1 - ghost1, c - p1) <= 0`, which is always true, so the end cap at `point1` is **ignored**. Measured: the circle passes into the platform and pops onto the top with no deflection.
  - The `point2` side test is `> 0`, which is always false, so the end cap at `point2` is **kept**. Measured: deflected exactly like a convex corner or a plain segment.
  - The `contact.c` comment ("safe: behaves like a plain segment") is wrong.
  - Suggested fix: default missing ghosts to *convex corners*, `ghost1 = p1 - n` and `ghost2 = p2 - n`, where `n = b2RightPerp(b2Normalize(p2 - p1))` is the outward normal. The "convex corner ghosts" variant uses exactly this and behaves like a plain segment at both ends. Alternatively, document that ghosts must be set.
  - The same applies to `b2CustomShape_SetGhostVertices`' one-sided fallback. It is currently unreachable through the API, which always sets both ghosts.
- **Custom ghost segments are still `b2_segmentShape` for CCD**, so they miss the chain early-out in `solver.c` (around line 235). No measured difference yet.
- **Not recorded or snapshotted** (see the port commit). Probe runs do not depend on this.

## 8. Reproducing

```sh
just probe                 # every scenario, markdown tables (about 0.3 s)
just probe Valve           # one scenario
./build/bin/ghost_probe --trace Valve 0 0 0    # per-step state + manifold normals: scenario variant mitigation sweepValue
just samples               # then pick category "Ghost Collisions"
```

**Source**
- Scenarios: `shared/ghost_scenarios.c`. Add one by appending to `g_ghostScenarios` and registering it in `samples/sample_ghost_collisions.cpp`.
- Probe: `experiments/ghost_probe/main.c`.

**marblebuilds references**
- `notes/bugfixes/one_way_segments.md` (D1)
- `notes/bugfixes/filter_and_wall.md` (D2)
- `benchmarks/localGridSeg/local-grid-seg-handwritten.md` (the ghost assignment rule)
- `benchmarks/localSeg/` (grid-free generalisation, geometry only)

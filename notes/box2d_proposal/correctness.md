# Correctness

**How it was measured.** All results are from `experiments/ghost_probe`: deterministic, 60 Hz, 4 substeps, gravity −10, Release, single precision.
- **Scenarios:** `shared/ghost_scenarios.c`, 22 of them, each also a sample under "Ghost Collisions" with a mitigation dropdown.
- **Mitigations compared:**
  - none;
  - Semrau: your `GhostCulling` sample filter ported line for line, with recycling off on the protected body;
  - topological.
- **Metrics:** progress along the surface, hop and sink relative to resting height, kick (velocity along the surface normal), speed ratio, snags (more than 10% tangential speed loss in one step), fell-through, and for some scenarios airborne steps and lift-offs.
- **Curved surfaces** are measured against the local surface frame.
- **Multi-body scenarios** report each body against the same body on a smooth chain with no mitigation (`--players`).

Full tables: [`../ghost_probe_results.md`](../ghost_probe_results.md), [`../ghost_defaults.md`](../ghost_defaults.md).

## Results at sample defaults

The "ok" verdict means the run matches the smooth-surface behaviour.

| Scenario | Contact type exercised | none | Semrau | topological |
|---|---|---|---|---|
| Sliding Box, segment tiles | box face vs two-sided segment endpoints | stuck | stuck | ok |
| Small Gaps, segment tiles (5 cm gaps) | face vs exposed side across a gap | stuck | stuck | ok |
| Wall Slide Down, segment wall | sliding down two-sided segments | stuck | hangs | ok |
| Misaligned Tiles, 1 mm | sub-slop step | stuck | **launched 1.2 m** (1-point vertex contact passes the filter) | ok |
| Tilted Box, 10° locked | 1-point corner vs seam | bumpy (kick 1.7) | same | ok |
| Thin Plank, 2000x gravity | 7 cm sink, beyond 4·slop | stuck | stuck | finishes (one settling spike on first contact, all methods) |
| Fast Landing, box tiles, (40, −15) m/s | TOI against seam corners | launched 18 m | launched 18 m | ok |
| Valve Corner | circle vs convex vertex of a face hidden under a one-sided segment | kick 12 | same | fixed |
| Interior Faces | circle over tiles that keep coincident interior faces | kick 9.7 | same | ok |
| Moving Platform | box across flush kinematic tiles | stuck | ok | ok |
| Rotating Platform, ball | circle across a flush rocking kinematic tile | kick 3.0 | same | ok |
| Slope Tiles, free box | rotating box down 30° triangle tiles | tumbles (kick 7) | same | ok |
| Compound Player | two-box body across box tiles | stuck | ok | ok |
| Crate Floor Circle | circle across dynamic crates | kick 2.3 | same | ok |
| Interior Push-out | circle spawned 15 cm inside a two-tile floor | **falls through** | falls through | pushed out on top |
| Vehicle Crest | wheeled compound over a crest onto tiles | bumpy | bumpy | ok |
| Parabola Crates | circle over dynamic, unevenly sliced quads on a curved bowl | kick 1.1, 31% speed loss | same | ok |
| Parabola Hills | circle over 20 static slices with a sine top, 21 m bowl | 12 lift-offs | 12 | **0** (same as smooth chain) |
| Shape Parade, triangle-tile slope | circle, capsule, triangle, pentagon, hexagon, box | all bumpy | pentagon and box fixed only | all ok |
| Tile Wall | real 1 m wall, 10–80 m/s | blocked | blocked | blocked |
| Cliff Pit | tall box falling past a cliff face | correct | correct | correct |
| CCD Back Face, 80 m/s | TOI end pose behind a thin wall | blocked | **passes through** (floor-material veto) | blocked |

**Shape Parade per body** (`--players`, speed 6):

| Surface | none | Semrau | topological |
|---|---|---|---|
| triangle-tile slope | all 6 bumpy (kick 2.2–3.6) | pentagon and box ok | all 6 ok |
| dynamic crate floor | capsule, triangle and box stuck; others bumpy | triangle, pentagon, hexagon and box ok | all 6 ok |
| parabola crates | all worse than the reference | ≈ none | all match the reference |

The Semrau failures are all structural:
- 1-point manifolds: circle, triangle corner, capsule cap, vertex-vertex speculative contacts.
- 2-point manifolds with a large anchor spread: the hexagon's lower face.
- Load above 4·slop.
- Two-sided segments, where the "area" is the whole segment length.

## Phase robustness (speed sweep)

At sample defaults, the launch speed often divides the tile width into a whole number of steps. Every seam is then met at the same phase, which can hide or exaggerate snags. The table repeats key cases over 41 launch speeds:
- 5–15 m/s in 0.25 m/s steps;
- Fast Landing: 20–80 m/s in 1.5 m/s steps.

A run is clean when minV ≥ 0.9, hop ≤ 0.1 m, kick ≤ 0.5 m/s and there is no fall-through. Source: [`experiments/ghost_probe/rounded_results.md`](../../experiments/ghost_probe/rounded_results.md), "Phase robustness".

| Case | none | Semrau | topological |
|---|---|---|---|
| Sliding Box, box tiles, g 1 | 3/41 | 41/41 | 41/41 |
| Sliding Box, segment tiles | 3/41 | 3/41 | 41/41 |
| Sliding Box, box tiles, g 100 | 3/41 | 41/41 | 41/41 |
| Misaligned Tiles, 5 mm | 3/41 | 19/41 | 41/41 |
| Misaligned Tiles, 15 mm | 4/41 | 26/41 | 4/41 (real lip, blocked correctly) |
| Tilted Box, 30° | 5/41 | 5/41 | 41/41 |
| Compound Player, box tiles | 0/41 | 41/41 | 41/41 |
| Small Gaps, segment tiles, 50 mm | 3/41 | 3/41 | 41/41 |
| Fast Landing, box tiles, 20–80 m/s | 0/41 | 3/41 | 41/41 |
| Tile Wall (blocked = correct) | 41/41 | 41/41 | 41/41 |
| Cliff Pit (did not cross = correct) | 41/41 | 41/41 | 41/41 |

On box tiles, Semrau is robust for the upright-box cases it was designed for. Topological is phase-independent everywhere tested.

## Valid behaviour, deliberately not changed

| Case | Behaviour | Reason |
|---|---|---|
| 15 mm / 30 mm lips (Misaligned Tiles) | sliding box stopped | real geometry above the support line |
| 9 mm lip on a rocking tile | ball bumps | real |
| 0.25 m gaps beside moving tiles | ball dips | real |
| Pentagon / hexagon tumbling | lift-offs equal to the smooth reference (4 vs 4, 7 vs 6) | rolling polygons hop over polyline corners |
| Dynamic crates compressing under a ball | 0.2–0.4 m/s push-out kick | soft-contact sink of 2–5 cm |
| Kinematic platform crushing a body under a wall | body squeezed through | same for every mitigation |

## Not solved

| Case | Behaviour | Cause |
|---|---|---|
| Thin plank on chain / loose chain segments, ≥ 3000x gravity | falls through | native one-sided collider drops the contact; pre-solve never sees it |
| Loose chain segment with ghost = endpoint, circle at the `point1` cap | cap ignored | same; `uPrev = 0 <= 0` in `b2CollideChainSegmentAndCircle` (`manifold.c:1143`) |
| Box tiles, ≥ 3000x gravity (about 10 cm sink) | occasional push-out pop (one run: 344 m/s) | push-out manifold uses the deepest points only |
| Capsule on Parabola Hills slices | 1 airborne step, 1.4 cm hop (reference: 0) | marginal; appeared with the 60° crest exemption |

## Bugs found during development, all fixed

An adversarial test pass and an independent code review were run against the method. Findings:

| Finding | Effect | Fix |
|---|---|---|
| Interval overflow on edges split more than 4 times | real ground deleted under the body | leave the edge uncut on overflow; 6 intervals |
| Continuous veto judged at the end-of-step pose | real hits vetoed, tunnelling | judge on the pre-step snapshot; covering not used for vetoes |
| Recycling silently disabled the mitigation | stale manifolds | force recycling off on protected bodies |
| Data race: veto read poses written by finalize workers | torn reads | snapshot only |
| Body centre used instead of shape centroid | compound bodies fell through crests | shape centroid |
| Vertex-covering rule without a depth check | body sank through a crest | require resting depth on the covering surface |
| Covered pieces used as ghost candidates | ball bumped on chain-to-kinematic seams | exclude covered pieces from `FindGhost` |
| Both covering rules hid gentle crest continuations | flat bodies lost support at smooth-chain crests | 60° continuation exemption |
| Neighbour snapshot truncated at the 128 cap in query order | arbitrary shapes dropped; box launched at 1/64 m tiles | bypass: on overflow, leave native contacts for that body and step (found by the benchmark density sweep) |

**Adversarial scenarios kept as regression tests:** Tile Wall, Cliff Pit, Wall Slide Down, Vehicle Crest, Small Gaps, CCD Back Face. Real walls and cliffs block, thin walls are not tunnelled at 80 m/s, and crests don't drop bodies.

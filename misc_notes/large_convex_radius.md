# Large convex radius on the protected polygon

This tests a suggestion from the Discord thread: give the player polygon a large radius (5–10% of its width) so its corners cross seams smoothly.

**How it was set up.**
- The protected polygon's faces are inset by r = f × (minimum width) and r is added back as the polygon radius. The outer extent is unchanged; for a box this equals `b2MakeRoundedBox(hx - r, hy - r, r)`.
- No callbacks. f ∈ {0.02, 0.05, 0.08, 0.10, 0.15, 0.25}.
- Mitigation `ghostMitigation_rounded`, `ghost_probe --round` / `--roundness`, full data in [`experiments/ghost_probe/rounded_results.md`](../experiments/ghost_probe/rounded_results.md).

## Result: phase-dependent, and it converts snags into launches

**At sample defaults it looks like it works.** With f = 0.05–0.15, the box clears seams on box and segment tiles, 15 mm misalignment, the compound player, and fast landings. But those defaults are commensurate with the tiles: 10 m/s over 2 m tiles is exactly 12 steps per tile, so every seam is met at the same phase.

**Over 41 launch speeds** (5–15 m/s, or 20–80 m/s for Fast Landing). Cells count clean runs out of 41:

| Case | none | Semrau | topological | rounded, best f | rounded, other f |
|---|---|---|---|---|---|
| Sliding Box, box tiles | 3 | 41 | 41 | 9 (f = 0.05) | 2–7 |
| Sliding Box, segment tiles | 3 | 3 | 41 | 9 (f = 0.05) | 2–7 |
| Misaligned 5 mm | 3 | 19 | 41 | 7 (f = 0.08) | 1–5 |
| Tilted Box 30° | 5 | 5 | 41 | 5 (f ≤ 0.08) | 0–4 |
| Compound Player | 0 | 41 | 41 | 9 (f = 0.05) | 0–6 |
| Small Gaps, segment tiles | 3 | 3 | 41 | 5 (f = 0.05) | 0–2 |
| Fast Landing, box tiles | 0 | 3 | 41 | 4 (f = 0.02) | 0–2 |
| Tile Wall, stays blocked (correct) | 41 | 41 | 41 | 31 (f = 0.02) | 21–30 (the rest launched over a 1 m wall) |
| Cliff Pit, stays in the pit (correct) | 41 | 41 | 41 | 38 (f = 0.15) | 34–36 (the rest carried over a 4 m pit) |

**Mechanism.** Collision goes through the core polygon. At a seam, the next tile's corner and the player's core corner are about r apart, which yields a 1-point vertex-vertex speculative manifold with a diagonal normal (|n.x| 0.32–0.71 in traces). It is created while the core corner is still up to sqrt((r+0.02)² − r²) behind the seam. Whether it fires depends on where in the step the corner arrives.
- **Effect:** forward speed becomes vertical speed. At f = 0.02, 10 m/s becomes (4.9, 4.9) and the box flies 1.3 m; at f = 0.25, (9.0, 3.0).
- **Larger r** lowers the angle but widens the firing window. No radius works reliably.

## Other effects

- **Not fixed at any f:**
  - thin plank on box tiles (stuck);
  - box tiles next to moving or rotating kinematic tiles;
  - triangle-tile slopes (stuck becomes 0.4–1.8 m hops);
  - small gaps and segment walls (stuck becomes bumpy);
  - box spawned inside the floor (still falls through);
  - all Shape Parade and Hills parade surfaces.
- **Climbs real lips:** 15 mm at f = 0.08, 30 mm at 0.10, 80 mm at 0.15. A sharp box stops at 20 mm or more under every other mitigation.
- **Tumbling:** rounding shrinks the flat support from a to a − r·tan(π/n), so polygons tip once friction > (a − r·tan(π/n)) / h_cm. Measured thresholds match that exactly: hexagon from f = 0.08, pentagon 0.15, triangle 0.25, box never at μ = 0.49.
- **Thin plank under 2000x gravity on a chain falls through at every f** (the sharp plank stays on). At the seam the rounded plank keeps one contact instead of two, sinks faster, and its centroid ends up behind the one-sided chain. Not traced into the solver.
- **Unchanged:** resting and sliding on smooth slopes; rest heights.

## Conclusion

A large convex radius trades a horizontal snag for a diagonal hop whose occurrence depends on step phase. It doesn't remove the seam contact, so it's not a substitute for removing seam features. It remains a reasonable cosmetic complement (softer corners against real geometry), at the cost of lip climbing and easier tipping.

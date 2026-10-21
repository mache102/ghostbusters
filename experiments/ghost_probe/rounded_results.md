# Rounded polygons (large convex radius) vs ghost collisions

The suggestion under test: give the player polygon "a very large convex radius (e.g. 5-10% of
player width)" so its corners are round and it crosses seams smoothly. `ghostMitigation_rounded`
("rounded") installs no callbacks; every polygon player shape is built with
`GhostRoundPolygon( polygon, f )`:

- width = the polygon's minimum width across its face normals (caliper width; a 1 m box: 1 m, the
  0.08 m thin plank: 0.08 m, parade hexagon: 0.52 m), r = f * width, default f = 0.08.
- Erode then dilate: every face plane moves inward by r (core vertex
  `v - r (na + nb) / (1 + na.nb)`), then the convex radius grows by r. The faces end exactly where
  they were, so the outer extent, rest heights and the measurement frames are unchanged; only the
  corners become radius r arcs (for a box this is `b2MakeRoundedBox( hx - r, hy - r, r )`). The core
  inradius drops by r. Tilted Box (rests on a corner) recomputes its rest height from the rounded
  shape's support point. Compound players round each box; the vehicle chassis is rounded too.
- Circles and capsules are untouched (rounded = none for them). Scenarios without a polygon player
  (Valve Corner, Interior Faces, Crate Floor Circle, Parabola Crates) do not offer rounded.

Probe: `ghost_probe --defaults` has a rounded column; `ghost_probe --round <scenario> <variant>
<sweep value>` runs none / semrau / topological and rounded at f = 0.02, 0.05, 0.08, 0.10, 0.15,
0.25 (one row per player for multi-body runs, judged against the same body on the smooth
reference); `--roundness <f>` and `--speed <v>` prefix any mode. 60 Hz, 4 substeps. The existing
none / semrau / topological rows of the full tables, `--defaults` and `--players` are byte
identical to acda980.

## Summary

**What rounding fixes.** At the scenarios' default parameters it looks good: f = 0.08 clears the
locked box on box tiles and segment tiles (Sliding Box, where Semrau fails on segment tiles),
Misaligned Tiles up to 15 mm, Compound Player, and even Fast Landing at 20/40/80 m/s; f = 0.05-0.15
clears Tilted Box at 30 degrees. **This is an aliasing artifact.** The defaults are commensurate
with the tiles (10 m/s over 2 m tiles = exactly 12 steps per tile, 20/40/80 m/s over 0.5 m tiles =
1.5 / 0.75 / 0.375 tiles per step), so every seam is met at the same point of a step and one lucky
phase clears all of them. Sweeping the launch speed from 5 to 15 m/s in 0.25 m/s steps (table
"Phase robustness"), a rounded box crosses a seamed floor cleanly in 2-9 of 41 runs for every f,
about the same as no mitigation (3/41). Semrau (on box tiles) and topological ghosts: 41/41.

**Mechanism** (traces below). Box2D collides a rounded polygon through its core: the next tile's
corner and the player's core corner are separated by about r, so the manifold is a single
vertex-vertex point with the normal along the line between them, a diagonal (normal x component 0.32 to 0.71 in the traces) instead of the horizontal side face. That normal is created as a
speculative contact (s = 0.006-0.013 m) while the core corner is still up to
sqrt((r + 0.02)^2 - r^2) behind the seam, and the solver stops the approach along it whenever the
tangential travel per substep exceeds what the gap allows, roughly when the corner is within
2 v h of the seam (h = substep). So rounding turns a snag (horizontal normal, speed lost) into a
hop/launch (diagonal normal, speed converted into vertical velocity): at f = 0.02, 10 m/s becomes
vx 4.9 / vy 4.9 and the box flies 1.3 m high; at f = 0.25 vx 9.0 / vy 3.0. Whether a given seam
fires depends on where in the step the corner arrives, hence the phase lottery. A larger radius
lowers the angle but widens the window, so no radius works reliably.

**What it doesn't fix.** Thin Plank on box tiles (STUCK for every f), Moving / Rotating Platform
with box tiles (STUCK or bumpy at every f), Slope Tiles on triangle tiles (locked box: STUCK ->
bumpy with 0.4-1.8 m hops; free box: tumbling reduced, kick 6.96 -> 2.35, still bumpy), Small
Gaps (STUCK -> bumpy), Wall Slide Down (STUCK -> bumpy, kick 2.0-5.7), Interior Push-out box
(FELL at every f), Shape Parade on tiles / crates (all bumpy or STUCK at every f; the triangle stays
STUCK on the dynamic crate floor for f <= 0.15), Parabola Hills parade on slices (LIFTS OFF at
every f).

**Correctness regressions (real geometry).**
- Tile Wall (1 m wall, box pushed right): the seam hop launches the box over the wall. Default
  (speed 10): PASSED THROUGH for f = 0.05 and 0.08 (kick 9.7 m/s, 3.1 m high). Speed sweep 5-15 m/s:
  the box clears the 1 m wall in 10-20 of 41 runs for every f. (none's "ok (blocked)" is vacuous: it
  snags at the first seam; Semrau / topological genuinely reach the wall and stop at x = 11.5.)
- Cliff Pit: the seam hop carries the box over the 4 m pit in 3-7 of 41 runs (e.g. f = 0.08 at
  12.5 m/s: n = (0.43, -0.90) at the x = 4 seam, vy 4.8, lands beyond the far wall).
- CCD Back Face, end offset 0.3: f = 0.10 PASSED THROUGH (kick 24 m/s), f = 0.08 kicked 8.6 m up.
- Thin Plank under 2000 x gravity on a chain / custom ghost segments: FELL for every f (none: bumpy
  but stays on), with the same metrics as Semrau's fall. Trace (`--roundness 0.02 --trace "Thin
  Plank" 0 4 2000`): crossing the x = 2 seam (step 13) the rounded plank has one chain contact (the
  new segment's is missing) where the sharp plank has two, then sinks faster (30 mm vs 17 mm at
  step 16), its centroid gets behind the one-sided chain at step 19 and it drops. (Likely the corner-vs-seam contact taking the
  vertex-vertex path, whose normal the chain's ghost vertex rejects; not traced into the solver.)
- Real lips: Misaligned Tiles with alternating steps of 15-80 mm (speed 10, the commensurate
  phase). Sharp boxes stop at a real step of 20 mm or more under every mitigation (Semrau still
  passes 15 mm). A rounded box slides straight up the steps: f = 0.08 (r = 80 mm) up to 15 mm,
  f = 0.10 up to 30 mm, f = 0.15 up to 80 mm; taller steps launch it (hop 0.5-1.2 m) and it sticks
  later. So yes, at f = 0.08-0.15 a 1 m box climbs 15-30 mm lips a sharp box is stopped by.

**Side effects on smooth surfaces** (where none, semrau and topological are clean):
- Tumbling: rounded polygons tip over more easily. A sliding polygon tips when friction exceeds the
  half support width over the center height; rounding shrinks the flat support from a to
  a - r tan(pi/n). The parade (mixed friction 0.49) then tips: hexagon from f = 0.08
  (0.126 / 0.26 = 0.48), pentagon from f = 0.15, triangle at f = 0.25, box never (0.5) - exactly the
  thresholds where the Shape Parade flat / slope smooth chain rows turn bumpy (hexagon rocks with
  growing amplitude, kick up to 1.9 m/s, hop 0.2 m). Parabola Hills parade on the smooth chain:
  ok -> LIFTS OFF for the box from f = 0.08 and the triangle at 0.10 / 0.25.
- Kinematic platforms on a chain (Moving / Rotating Platform, chain terrain): ok -> bumpy for
  f >= 0.02-0.05 (kick 1-5 m/s); the rounded corner meets the platform tiles' corners the same way.
- Vehicle Crest, locked box on box + slope tiles: ok -> bumpy (kick 3.5, minV 0.51) at f = 0.08.
- Resting / sliding on a smooth slope: unchanged (Slope Tiles chain rows identical to none for box
  and free box at speed 0/5/10). Rest heights hold: hop and sink stay below 1 mm on flat chains.

**Radius window.** There is no radius at which rounding works in general. At the commensurate
default speeds f = 0.08-0.15 clears the flat seams and f = 0.02 (r = 20 mm) and 0.25 fail (launch
at 45 degrees and hop at 19 degrees respectively); over a speed sweep every f fails most runs.
Lip climbing starts around f = 0.08 (15 mm) and grows with f; tumbling of hexagons starts at
f = 0.08.

**Surprises.** (1) The default-parameter result is almost entirely aliasing; (2) rounding makes the
box hop over a 1 m wall and a 4 m pit, i.e. it converts a ghost snag into the opposite failure;
(3) a rounded thin plank falls through a chain that holds the sharp one; (4) the tumbling
thresholds match the tipping-ratio formula exactly.

## Mechanism traces

`ghost_probe --roundness 0.02 --trace "Sliding Box" 0 4 1` (columns: step x y vx vy angle contacts,
manifold normal pointing away from the player):

```
10 1.5883 0.5767 4.911 4.917 0.000 2 | polygon/static x[1.98,4.02] y[-0.52,0.02] n=(0.71,-0.71) s=0.0083 | polygon/static x[-0.02,2.02] ... n=(-0.00,-1.00) s=-0.0000 s=-0.0000
11 1.6702 0.6569 4.911 4.750 0.000 2
16 2.0794 1.0163 4.911 3.917 0.000 0
```

`ghost_probe --roundness 0.25 --trace "Sliding Box" 0 4 1`:

```
11 1.8206 0.5371 8.962 2.988 0.000 2 | polygon/static x[1.98,4.02] y[-0.52,0.02] n=(0.32,-0.95) s=0.0135 | ...
```

`ghost_probe --trace "Tile Wall" 0 4 10` (f = 0.08, pushed right at 20 m/s^2): the seams at x = 2,
4 and 6 pass at this phase, the x = 8 seam fires at 20 m/s and launches the box over the wall:

```
30 7.5208 0.5000 20.000 0.000 0.000 1 | polygon/static x[5.98,8.02] y[-0.52,0.02] n=(-0.00,-1.00) s=-0.0000 s=-0.0000
31 7.7494 0.6446 13.062 9.663 0.000 2 | polygon/static x[7.98,10.02] y[-0.52,0.02] n=(0.59,-0.80) s=0.0195 | ...
45 11.3513 2.6222 17.729 7.329 0.000 0      <- 2.1 m above the top of the 1 m wall at x = 12
```

`ghost_probe --trace "Cliff Pit" 0 4 12.5`:

```
18 3.7142 0.5729 10.100 4.841 0.000 2 | polygon/static x[3.98,6.02] y[-0.52,0.02] n=(0.43,-0.90) s=0.0087 | ...
```

`ghost_probe --trace "Shape Parade" 6 4 6` (flat smooth chain, f = 0.08), hexagon every 15 steps:
angle -0.07, -0.59, -1.93, 2.90, 1.41 ... (rolling over; none: angle 0.000 throughout).

## --defaults with the rounded column (f = 0.08)

`ghost_probe --defaults`, * = the variant the sample opens with.

| scenario | variant | none | semrau | rounded | topological |
|---|---|---|---|---|---|
| Sliding Box | box tiles | STUCK (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Sliding Box | segment tiles * | STUCK (kick 0.00, minV 0.00) | STUCK (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Sliding Box | custom ghost segments | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Sliding Box | chain | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Misaligned Tiles | box tiles * | STUCK (kick 0.00, minV -0.00) | STUCK (kick 4.81, minV 0.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Misaligned Tiles | custom ghost segments | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Misaligned Tiles | loose chain segments | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Tilted Box | box tiles * | bumpy (kick 1.66, minV 0.74) | bumpy (kick 1.66, minV 0.74) | bumpy (kick 1.71, minV 0.74) | ok (kick 0.00, minV 1.00) |
| Tilted Box | chain | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Thin Plank | chain | bumpy (kick 20.00, minV 1.00) | FELL (kick 0.00, minV 0.00) | FELL (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.00) |
| Thin Plank | box tiles * | STUCK (kick 20.00, minV -0.00) | STUCK (kick 0.00, minV 0.00) | STUCK (kick 0.00, minV -0.00) | bumpy (kick 9.59, minV 1.00) |
| Thin Plank | custom ghost segments | bumpy (kick 20.00, minV 1.00) | FELL (kick 0.00, minV 0.00) | FELL (kick 327.73, minV 0.00) | bumpy (kick 9.59, minV 1.00) |
| Fast Landing | chain | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Fast Landing | box tiles * | STUCK (kick 15.25, minV 0.35) | STUCK (kick 15.25, minV 0.35) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Fast Landing | single segment | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Valve Corner | local ghost rule * | bumpy (kick 12.16, minV 0.84) | bumpy (kick 12.16, minV 0.84) | - | bumpy (kick 0.65, minV 0.97) |
| Valve Corner | ghost via valve | bumpy (kick 0.65, minV 0.97) | bumpy (kick 0.65, minV 0.97) | - | bumpy (kick 0.65, minV 0.97) |
| Valve Corner | solid under valve | bumpy (kick 0.65, minV 0.97) | bumpy (kick 0.65, minV 0.97) | - | bumpy (kick 0.65, minV 0.97) |
| Interior Faces | per-tile loops * | bumpy (kick 9.68, minV 0.83) | bumpy (kick 9.68, minV 0.83) | - | ok (kick 0.00, minV 0.99) |
| Interior Faces | merged surface | ok (kick 0.00, minV 0.99) | ok (kick 0.00, minV 0.99) | - | ok (kick 0.00, minV 0.99) |
| Moving Platform | box tiles terrain * | STUCK (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.00) | STUCK (kick 3.55, minV 0.28) | ok (kick 0.00, minV 1.00) |
| Moving Platform | chain terrain | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | bumpy (kick 4.05, minV 0.81) | ok (kick 0.00, minV 1.00) |
| Moving Platform | ball, box tiles terrain | STUCK (kick 2.49, minV 0.34) | STUCK (kick 2.49, minV 0.34) | STUCK (kick 2.49, minV 0.34) | bumpy (kick 3.18, minV 0.68) |
| Moving Platform | ball, chain terrain | bumpy (kick 2.50, minV 0.79) | bumpy (kick 2.50, minV 0.79) | bumpy (kick 2.50, minV 0.79) | bumpy (kick 2.50, minV 0.79) |
| Rotating Platform | box tiles | STUCK (kick 0.05, minV 0.00) | ok (kick 0.05, minV 1.00) | bumpy (kick 3.88, minV 0.80) | ok (kick 0.05, minV 1.00) |
| Rotating Platform | chain | ok (kick 0.05, minV 1.00) | ok (kick 0.05, minV 1.00) | bumpy (kick 3.85, minV 0.63) | ok (kick 0.05, minV 1.00) |
| Rotating Platform | ball, box tiles | STUCK (kick 2.69, minV 0.51) | STUCK (kick 2.69, minV 0.51) | STUCK (kick 2.69, minV 0.51) | bumpy (kick 2.40, minV 0.90) |
| Rotating Platform | ball, chain * | bumpy (kick 2.96, minV 0.84) | bumpy (kick 2.96, minV 0.84) | bumpy (kick 2.96, minV 0.84) | ok (kick 0.00, minV 1.00) |
| Slope Tiles | box, triangle tiles | STUCK (kick 0.02, minV 0.00) | ok (kick 0.05, minV 1.10) | bumpy (kick 2.56, minV 0.75) | ok (kick 0.05, minV 1.10) |
| Slope Tiles | box, chain | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) |
| Slope Tiles | ball, triangle tiles | bumpy (kick 2.34, minV 1.10) | bumpy (kick 2.34, minV 1.10) | bumpy (kick 2.34, minV 1.10) | ok (kick 0.00, minV 1.10) |
| Slope Tiles | ball, chain | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) |
| Slope Tiles | free box, triangle tiles * | bumpy (kick 6.96, minV 1.00) | bumpy (kick 7.10, minV 1.03) | bumpy (kick 2.35, minV 0.79) | ok (kick 0.05, minV 1.10) |
| Slope Tiles | free box, chain | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) | ok (kick 0.00, minV 1.10) |
| Compound Player | box tiles * | STUCK (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Compound Player | chain | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) | ok (kick 0.00, minV 1.00) |
| Crate Floor Circle | dynamic crates * | bumpy (kick 2.32, minV 0.74) | bumpy (kick 2.32, minV 0.74) | - | ok (kick 0.00, minV 0.99) |
| Crate Floor Circle | static box tiles | bumpy (kick 2.03, minV 0.73) | bumpy (kick 2.03, minV 0.73) | - | ok (kick 0.03, minV 0.99) |
| Interior Push-out | circle * | FELL (kick 0.00, minV 0.00) | FELL (kick 0.00, minV 0.00) | FELL (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.00) |
| Interior Push-out | box | FELL (kick 0.00, minV 0.00) | FELL (kick 0.00, minV 0.00) | FELL (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.00) |
| Tile Wall | box tiles + wall box * | ok (blocked) (kick 0.10, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) | PASSED THROUGH (kick 9.66, minV 1.10) | ok (blocked) (kick 0.02, minV 0.00) |
| Tile Wall | box tiles + tiled wall | ok (blocked) (kick 0.10, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) | PASSED THROUGH (kick 9.66, minV 1.10) | ok (blocked) (kick 0.02, minV 0.00) |
| Tile Wall | chain into wall | ok (blocked) (kick 0.02, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) | ok (blocked) (kick 0.01, minV 0.00) | ok (blocked) (kick 0.02, minV 0.00) |
| Tile Wall | thin 0.1 m wall | ok (blocked) (kick 0.10, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) | PASSED THROUGH (kick 9.66, minV 1.10) | ok (blocked) (kick 0.02, minV 0.00) |
| Tile Wall | segment tiles + segment wall | ok (blocked) (kick 0.10, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) | PASSED THROUGH (kick 9.66, minV 1.10) | ok (blocked) (kick 0.02, minV 0.00) |
| Cliff Pit | box tiles * | ok (blocked) (kick 0.00, minV 0.00) | ok (blocked) (kick 0.00, minV -0.00) | ok (blocked) (kick 0.27, minV -0.07) | ok (blocked) (kick 0.00, minV -0.00) |
| Cliff Pit | chain | ok (blocked) (kick 0.00, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) | ok (blocked) (kick 0.00, minV -0.00) | ok (blocked) (kick 0.00, minV 0.00) |
| Cliff Pit | tall box, box tiles | ok (blocked) (kick 0.00, minV 0.00) | ok (blocked) (kick 0.00, minV -0.00) | ok (blocked) (kick 0.27, minV -0.07) | ok (blocked) (kick 0.00, minV -0.00) |
| Cliff Pit | tall box, chain | ok (blocked) (kick 0.00, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) | ok (blocked) (kick 0.00, minV 0.00) |
| Wall Slide Down | box wall tiles | STUCK (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.58) | bumpy (kick 2.62, minV 1.33) | ok (kick 0.00, minV 1.58) |
| Wall Slide Down | chain wall | ok (kick 0.00, minV 1.58) | ok (kick 0.00, minV 1.58) | ok (kick 0.00, minV 1.58) | ok (kick 0.00, minV 1.58) |
| Wall Slide Down | segment wall tiles * | STUCK (kick 0.00, minV 0.00) | slowed (kick 0.00, minV 0.00) | bumpy (kick 2.48, minV 1.33) | ok (kick 0.00, minV 1.58) |
| Vehicle Crest | vehicle, chain | ok (kick 0.00, minV 1.15) | ok (kick 0.00, minV 1.15) | ok (kick 0.00, minV 1.15) | ok (kick 0.00, minV 1.15) |
| Vehicle Crest | vehicle, box + slope tiles * | bumpy (kick 1.38, minV 0.87) | bumpy (kick 1.38, minV 0.87) | bumpy (kick 1.27, minV 0.86) | ok (kick 0.00, minV 1.15) |
| Vehicle Crest | plank, chain | ok (kick 0.00, minV 1.11) | ok (kick 0.00, minV 1.11) | ok (kick 0.00, minV 1.11) | ok (kick 0.00, minV 1.11) |
| Vehicle Crest | plank, box + slope tiles | STUCK (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.11) | bumpy (kick 1.74, minV 1.04) | ok (kick 0.00, minV 1.11) |
| Vehicle Crest | locked box, chain | ok (kick 0.00, minV 1.15) | ok (kick 0.00, minV 1.15) | ok (kick 0.00, minV 1.15) | ok (kick 0.00, minV 1.15) |
| Vehicle Crest | locked box, box + slope tiles | ok (kick 0.00, minV 1.15) | ok (kick 0.00, minV 1.15) | bumpy (kick 3.48, minV 0.51) | ok (kick 0.00, minV 1.15) |
| Small Gaps | box tiles | STUCK (kick 0.00, minV 0.00) | ok (kick 0.00, minV 1.00) | bumpy (kick 3.37, minV 0.76) | ok (kick 0.00, minV 1.00) |
| Small Gaps | segment tiles * | STUCK (kick 0.00, minV 0.00) | STUCK (kick 0.00, minV 0.00) | bumpy (kick 3.37, minV 0.76) | ok (kick 0.00, minV 1.00) |
| CCD Back Face | thin box wall, box tiles * | ok (blocked) (kick 0.05, minV 0.00) | PASSED THROUGH (kick 0.00, minV 1.00) | ok (blocked) (kick 0.14, minV -0.00) | ok (blocked) (kick 0.14, minV 0.00) |
| CCD Back Face | segment wall, box tiles | ok (blocked) (kick 0.04, minV 0.00) | PASSED THROUGH (kick 0.00, minV 1.00) | ok (blocked) (kick 0.12, minV -0.00) | ok (blocked) (kick 0.13, minV 0.00) |
| CCD Back Face | thin box wall, chain floor | ok (blocked) (kick 0.14, minV 0.00) | PASSED THROUGH (kick 0.00, minV 1.00) | ok (blocked) (kick 0.14, minV -0.00) | ok (blocked) (kick 0.14, minV 0.00) |
| CCD Back Face | segment wall, chain floor | ok (blocked) (kick 0.13, minV 0.00) | PASSED THROUGH (kick 0.00, minV 1.00) | ok (blocked) (kick 0.12, minV -0.00) | ok (blocked) (kick 0.13, minV 0.00) |
| Parabola Crates | dynamic slices * | bumpy (kick 1.10, minV 0.69) | bumpy (kick 1.10, minV 0.69) | - | ok (kick 0.24, minV 0.93) |
| Parabola Crates | static slices | bumpy (kick 0.84, minV 0.76) | bumpy (kick 0.84, minV 0.76) | - | ok (kick 0.07, minV 0.93) |
| Parabola Crates | smooth chain | ok (kick 0.00, minV 0.92) | ok (kick 0.00, minV 0.92) | - | ok (kick 0.00, minV 0.92) |
| Shape Parade | parabola crates (dynamic) | STUCK (kick 2.34, minV -0.06) | STUCK (kick 2.25, minV -0.02) | STUCK (kick 2.69, minV -0.03) | bumpy (kick 0.79, minV 0.71) |
| Shape Parade | parabola crates (static) | STUCK (kick 2.19, minV -0.00) | STUCK (kick 2.75, minV -0.10) | bumpy (kick 2.07, minV 0.19) | bumpy (kick 1.10, minV 0.73) |
| Shape Parade | parabola smooth chain | bumpy (kick 1.35, minV 0.71) | bumpy (kick 1.35, minV 0.72) | bumpy (kick 1.60, minV 0.71) | bumpy (kick 1.48, minV 0.71) |
| Shape Parade | slope triangle tiles * | bumpy (kick 3.55, minV 0.10) | bumpy (kick 2.68, minV 0.10) | bumpy (kick 4.37, minV 0.32) | ok (kick 0.14, minV 1.05) |
| Shape Parade | slope smooth chain | ok (kick 0.00, minV 1.05) | ok (kick 0.00, minV 1.05) | bumpy (kick 0.85, minV 1.05) | ok (kick 0.00, minV 1.05) |
| Shape Parade | dynamic crate floor | STUCK (kick 2.80, minV -0.01) | bumpy (kick 1.15, minV 0.19) | STUCK (kick 2.86, minV -0.01) | ok (kick 0.00, minV 0.99) |
| Shape Parade | flat smooth chain | ok (kick 0.00, minV 0.99) | ok (kick 0.00, minV 0.99) | bumpy (kick 1.93, minV 0.87) | ok (kick 0.00, minV 0.99) |
| Parabola Hills | static slices * | LIFTS OFF (kick 1.47, minV -0.68, lifts 12, air 89) | LIFTS OFF (kick 1.47, minV -0.68, lifts 12, air 88) | LIFTS OFF (kick 1.47, minV -0.68, lifts 12, air 89) | ok (kick 0.31, minV 0.79, lifts 0, air 0) |
| Parabola Hills | static slices, separate bodies | LIFTS OFF (kick 1.47, minV -0.68, lifts 12, air 89) | LIFTS OFF (kick 1.47, minV -0.68, lifts 12, air 88) | LIFTS OFF (kick 1.47, minV -0.68, lifts 12, air 89) | ok (kick 0.31, minV 0.79, lifts 0, air 0) |
| Parabola Hills | smooth chain | ok (kick 0.31, minV 0.79, lifts 0, air 0) | ok (kick 0.31, minV 0.79, lifts 0, air 0) | ok (kick 0.31, minV 0.79, lifts 0, air 0) | ok (kick 0.31, minV 0.79, lifts 0, air 0) |
| Parabola Hills | parade, static slices | LIFTS OFF (kick 2.49, minV -0.02, lifts 28, air 295) | LIFTS OFF (kick 2.49, minV -0.00, lifts 19, air 220) | LIFTS OFF (kick 2.61, minV 0.18, lifts 35, air 298) | LIFTS OFF (kick 1.90, minV 0.76, lifts 12, air 91) |
| Parabola Hills | parade, smooth chain | ok (kick 1.88, minV 0.67, lifts 10, air 94) | bumpy (kick 2.21, minV 0.67, lifts 11, air 94) | LIFTS OFF (kick 1.43, minV 0.76, lifts 13, air 104) | ok (kick 1.88, minV 0.67, lifts 10, air 99) |

## Phase robustness (speed sweep)

Same scenarios with the launch speed swept (`ghost_probe --speed <v> --round ...`, script below). Everything that passes on the seamed floors passes at speeds where a tile is a whole number of steps (8, 10, 12, 15 m/s over 2 m tiles) or close to it.

| case | none | semrau | topological | rounded 0.02 | rounded 0.05 | rounded 0.08 | rounded 0.10 | rounded 0.15 | rounded 0.25 |
|---|---|---|---|---|---|---|---|---|---|---|
| Sliding Box / box tiles / g 1 | 3/41 | 41/41 | 41/41 | 3/41 | 9/41 | 7/41 | 6/41 | 2/41 | 3/41 |
| Sliding Box / segment tiles / g 1 | 3/41 | 3/41 | 41/41 | 3/41 | 9/41 | 7/41 | 6/41 | 2/41 | 3/41 |
| Sliding Box / box tiles / g 100 | 3/41 | 41/41 | 41/41 | 3/41 | 2/41 | 7/41 | 3/41 | 2/41 | 3/41 |
| Misaligned Tiles / box tiles / 5 mm | 3/41 | 19/41 | 41/41 | 3/41 | 2/41 | 7/41 | 5/41 | 1/41 | 3/41 |
| Misaligned Tiles / box tiles / 15 mm | 4/41 | 26/41 | 4/41 | 3/41 | 2/41 | 7/41 | 5/41 | 1/41 | 2/41 |
| Tilted Box / box tiles / 30 deg | 5/41 | 5/41 | 41/41 | 5/41 | 5/41 | 5/41 | 3/41 | 4/41 | 0/41 |
| Compound Player / box tiles / g 1 | 0/41 | 41/41 | 41/41 | 0/41 | 9/41 | 6/41 | 5/41 | 3/41 | 2/41 |
| Small Gaps / segment tiles / 50 mm | 3/41 | 3/41 | 41/41 | 2/41 | 5/41 | 1/41 | 2/41 | 0/41 | 1/41 |
| Tile Wall / box tiles + wall box (launch speed, then pushed) | 41/41 | 41/41 | 41/41 | 31/41 | 21/41 | 23/41 | 28/41 | 23/41 | 30/41 |
| Tile Wall / segment tiles + segment wall | 41/41 | 41/41 | 41/41 | 31/41 | 21/41 | 23/41 | 28/41 | 23/41 | 30/41 |
| Cliff Pit / box tiles | 41/41 | 41/41 | 41/41 | 36/41 | 36/41 | 36/41 | 34/41 | 38/41 | 36/41 |
| Fast Landing / box tiles (speeds 20 to 80 m/s in 1.5 m/s steps) | 0/41 | 3/41 | 41/41 | 4/41 | 0/41 | 2/41 | 2/41 | 2/41 | 0/41 |

Speeds: 5 to 15 m/s in 0.25 m/s steps (41 runs per cell); cell = clean runs (minV >= 0.9, hop <= 0.1 m, kick <= 0.5 m/s, no fall through; the step cap may end slow runs early); Tile Wall / Cliff Pit (speed is their sweep field): runs judged ok (blocked), i.e. that did not pass the wall or cross the pit.

<details><summary>phase.py</summary>

```python
"""Phase robustness: run --round over many launch speeds and count verdicts per configuration."""
import subprocess
import sys
from collections import Counter, OrderedDict

PROBE = sys.argv[1]
cases = [
    ("Sliding Box", 0, "1", "Sliding Box / box tiles / g 1"),
    ("Sliding Box", 1, "1", "Sliding Box / segment tiles / g 1"),
    ("Sliding Box", 0, "100", "Sliding Box / box tiles / g 100"),
    ("Misaligned Tiles", 0, "0.005", "Misaligned Tiles / box tiles / 5 mm"),
    ("Misaligned Tiles", 0, "0.015", "Misaligned Tiles / box tiles / 15 mm"),
    ("Tilted Box", 0, "30", "Tilted Box / box tiles / 30 deg"),
    ("Compound Player", 0, "1", "Compound Player / box tiles / g 1"),
    ("Small Gaps", 1, "0.05", "Small Gaps / segment tiles / 50 mm"),
    ("Tile Wall", 0, None, "Tile Wall / box tiles + wall box (launch speed, then pushed)"),
    ("Tile Wall", 4, None, "Tile Wall / segment tiles + segment wall"),
    ("Cliff Pit", 0, None, "Cliff Pit / box tiles"),
    ("Fast Landing", 1, None, "Fast Landing / box tiles (speeds 20 to 80 m/s in 1.5 m/s steps)"),
]
speeds = [5.0 + 0.25 * i for i in range(41)]

print("| case | %s |" % " | ".join(["none", "semrau", "topological"] + ["rounded %.2f" % f for f in (0.02, 0.05, 0.08, 0.10, 0.15, 0.25)]))
print("|---" * 10 + "|---|")
for name, variant, value, label in cases:
    counts = OrderedDict()
    caseSpeeds = [20.0 + 1.5 * i for i in range(41)] if name == "Fast Landing" else speeds
    for v in caseSpeeds:
        out = subprocess.run([PROBE, "--speed", str(v), "--round", name, str(variant), value if value is not None else str(v)],
                             capture_output=True, text=True, check=True).stdout
        for line in out.splitlines():
            cells = [c.strip() for c in line.split("|")]
            if len(cells) < 4 or cells[1] in ("mitigation", "") or cells[1].startswith("---"):
                continue
            minV, hop, kick = float(cells[6]), float(cells[8]), float(cells[10])
            if name in ("Tile Wall", "Cliff Pit"):
                good = cells[3] == "ok (blocked)"
            else:
                # Clean run: no snag, hop or kick symptom while it ran (a slow run may stop at the
                # scenario's step cap before the end of the path, which is not a symptom)
                good = cells[3] != "FELL" and minV >= 0.9 and hop <= 0.1 and kick <= 0.5
            counts.setdefault(cells[1], Counter())["good" if good else "bad"] += 1
    row = []
    for config, c in counts.items():
        good = c.get("good", 0)
        row.append("%d/%d" % (good, len(speeds)))
    print("| %s | %s |" % (label, " | ".join(row)))
print()
print("Speeds: 5 to 15 m/s in 0.25 m/s steps (%d runs per cell); cell = clean runs (minV >= 0.9, hop <= 0.1 m, kick <= 0.5 m/s, no fall through; the step cap may end slow runs early); Tile Wall / Cliff Pit (speed is their sweep field): runs judged ok (blocked), i.e. that did not pass the wall or cross the pit." % len(speeds))
```

</details>

## Rounding sweeps (`ghost_probe --round`)

Columns as in the full tables; air / lifts only count in measureAirborne scenarios. Multi-body tables: verdict = this body against the same body on the smooth reference variant without mitigation (circle and capsule rows omitted: rounding does not change them). Commands:

```sh
ghost_probe --round "Sliding Box" 0 1; ghost_probe --round "Sliding Box" 0 100; ghost_probe --round "Sliding Box" 1 1; ghost_probe --round "Sliding Box" 1 100
ghost_probe --round "Misaligned Tiles" 0 0.001; ghost_probe --round "Misaligned Tiles" 0 0.005; ghost_probe --round "Misaligned Tiles" 0 0.015
ghost_probe --round "Tilted Box" 0 2; ghost_probe --round "Tilted Box" 0 10; ghost_probe --round "Tilted Box" 0 30
ghost_probe --round "Thin Plank" 1 1; ghost_probe --round "Thin Plank" 1 2000; ghost_probe --round "Thin Plank" 0 2000; ghost_probe --round "Thin Plank" 2 2000
ghost_probe --round "Fast Landing" 1 20; ghost_probe --round "Fast Landing" 1 40; ghost_probe --round "Fast Landing" 1 80
ghost_probe --round "Compound Player" 0 10
ghost_probe --round "Small Gaps" 1 0.01; ghost_probe --round "Small Gaps" 1 0.05; ghost_probe --round "Small Gaps" 1 0.2
ghost_probe --round "Slope Tiles" 0 5; ghost_probe --round "Slope Tiles" 4 5
ghost_probe --round "Moving Platform" 0 1; ghost_probe --round "Moving Platform" 1 1
ghost_probe --round "Rotating Platform" 0 0.5; ghost_probe --round "Rotating Platform" 1 0.5
ghost_probe --round "Vehicle Crest" 1 8; ghost_probe --round "Vehicle Crest" 3 8; ghost_probe --round "Vehicle Crest" 5 8; ghost_probe --round "Vehicle Crest" 4 8
ghost_probe --round "Wall Slide Down" 2 2
ghost_probe --round "Shape Parade" 3 6; ghost_probe --round "Shape Parade" 4 6; ghost_probe --round "Shape Parade" 1 6; ghost_probe --round "Shape Parade" 5 6; ghost_probe --round "Shape Parade" 6 6
ghost_probe --round "Parabola Hills" 3 3; ghost_probe --round "Parabola Hills" 4 3
ghost_probe --round "Tile Wall" 0 10; ghost_probe --round "Tile Wall" 1 10; ghost_probe --round "Tile Wall" 2 10; ghost_probe --round "Tile Wall" 3 10; ghost_probe --round "Tile Wall" 4 10; ghost_probe --round "Tile Wall" 0 40
ghost_probe --round "Cliff Pit" 0 10; ghost_probe --round "Cliff Pit" 2 10
ghost_probe --round "CCD Back Face" 0 0; ghost_probe --round "CCD Back Face" 0 0.3; ghost_probe --round "CCD Back Face" 1 0
ghost_probe --round "Interior Push-out" 1 0.15
ghost_probe --round "Misaligned Tiles" 0 0.015; ghost_probe --round "Misaligned Tiles" 0 0.02; ghost_probe --round "Misaligned Tiles" 0 0.03; ghost_probe --round "Misaligned Tiles" 0 0.05; ghost_probe --round "Misaligned Tiles" 0 0.08
ghost_probe --round "Misaligned Tiles" 2 0.03
```

### Seams on flat floors

#### Sliding Box / box tiles / gravity scale 1

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.030 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.297 | 0.089 | 3 | 1.2749 | 0.0276 | 4.917 | 0 | 0 |
| rounded 0.05 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.731 | 0.496 | 4 | 0.4771 | 0.0163 | 2.988 | 0 | 0 |

#### Sliding Box / box tiles / gravity scale 100

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.030 | 0.000 | 1 | 0.0000 | 0.0035 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.016 | -0.151 | 1 | 0.0000 | 0.0041 | 0.000 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.024 | -0.101 | 1 | 0.0000 | 0.0036 | 0.000 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.071 | -0.106 | 3 | 0.0252 | 0.0050 | 1.699 | 0 | 0 |

#### Sliding Box / segment tiles / gravity scale 1

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.030 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.030 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.299 | 0.098 | 3 | 1.2749 | 0.0276 | 4.917 | 0 | 0 |
| rounded 0.05 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.731 | 0.496 | 4 | 0.4771 | 0.0163 | 2.988 | 0 | 0 |

#### Sliding Box / segment tiles / gravity scale 100

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.030 | 0.000 | 1 | 0.0000 | 0.0035 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.030 | 0.000 | 1 | 0.0000 | 0.0035 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.016 | -0.151 | 1 | 0.0000 | 0.0041 | 0.000 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.024 | -0.101 | 1 | 0.0000 | 0.0036 | 0.000 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0028 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.071 | -0.106 | 3 | 0.0252 | 0.0050 | 1.699 | 0 | 0 |

#### Misaligned Tiles / box tiles / step height 0.001

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0010 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.630 | 0.000 | 2 | 1.2280 | 0.0434 | 4.811 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0010 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.251 | 0.067 | 5 | 1.2746 | 0.0284 | 4.915 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.374 | 0.154 | 5 | 1.2653 | 0.0386 | 4.914 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0010 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0010 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0010 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.679 | 0.445 | 4 | 0.4951 | 0.0000 | 3.057 | 0 | 0 |

#### Misaligned Tiles / box tiles / step height 0.005

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0049 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0049 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0049 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.458 | 0.167 | 4 | 1.2315 | 0.0319 | 4.827 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.373 | 0.244 | 2 | 1.2509 | 0.0043 | 4.880 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0049 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0049 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0049 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.695 | 0.465 | 4 | 0.4635 | 0.0173 | 2.943 | 0 | 0 |

#### Misaligned Tiles / box tiles / step height 0.015

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0148 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0149 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0148 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.711 | 0.528 | 2 | 0.9983 | 0.0316 | 4.330 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.225 | -0.037 | 3 | 1.1125 | 0.0241 | 4.586 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0149 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0149 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0149 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.777 | 0.672 | 2 | 0.5357 | 0.0000 | 3.131 | 0 | 0 |

#### Tilted Box / box tiles / tilt (deg) 2

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok | yes | 0.982 | 0.966 | 0 | 0.0114 | 0.0001 | 0.354 | 0 | 0 |
| semrau | - | ok | yes | 0.982 | 0.966 | 0 | 0.0114 | 0.0001 | 0.354 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| rounded 0.02 | - | ok | yes | 0.983 | 0.967 | 0 | 0.0111 | 0.0001 | 0.354 | 0 | 0 |
| rounded 0.05 | - | ok | yes | 0.983 | 0.966 | 0 | 0.0112 | 0.0001 | 0.354 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 0.982 | 0.966 | 0 | 0.0112 | 0.0001 | 0.354 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 0.982 | 0.966 | 0 | 0.0109 | 0.0001 | 0.354 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 0.981 | 0.965 | 0 | 0.0114 | 0.0001 | 0.354 | 0 | 0 |
| rounded 0.25 | - | ok | yes | 0.981 | 0.964 | 0 | 0.0111 | 0.0001 | 0.354 | 0 | 0 |

#### Tilted Box / box tiles / tilt (deg) 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | bumpy | yes | 0.851 | 0.735 | 0 | 0.1607 | 0.0036 | 1.662 | 0 | 0 |
| semrau | - | bumpy | yes | 0.851 | 0.735 | 0 | 0.1607 | 0.0034 | 1.662 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 0.840 | 0.710 | 0 | 0.1638 | 0.0022 | 1.662 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 0.840 | 0.710 | 0 | 0.1683 | 0.0024 | 1.664 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.859 | 0.742 | 0 | 0.1677 | 0.0063 | 1.706 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.825 | 0.686 | 0 | 0.1554 | 0.0064 | 1.663 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.859 | 0.753 | 0 | 0.1486 | 0.0044 | 1.683 | 0 | 0 |

#### Tilted Box / box tiles / tilt (deg) 30

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.557 | 0.297 | 3 | 0.9764 | 0.0180 | 3.958 | 0 | 0 |
| semrau | - | STUCK | no | 0.557 | 0.297 | 3 | 0.9765 | 0.0180 | 3.958 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.574 | 0.236 | 4 | 0.7052 | 0.0238 | 3.582 | 0 | 0 |
| rounded 0.05 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.671 | 0.524 | 2 | 0.5446 | 0.0195 | 2.886 | 0 | 0 |

#### Thin Plank / box tiles / gravity scale 1

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.040 | -0.000 | 1 | 0.0000 | 0.0000 | 0.010 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.102 | 0.065 | 1 | 0.1083 | 0.0000 | 1.310 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.284 | 0.162 | 3 | 0.3264 | 0.0000 | 2.417 | 0 | 0 |
| rounded 0.08 | - | STUCK | no | 0.176 | -0.043 | 3 | 0.3257 | 0.0014 | 2.417 | 0 | 0 |
| rounded 0.10 | - | STUCK | no | 0.200 | -0.054 | 3 | 0.3252 | 0.0030 | 2.417 | 0 | 0 |
| rounded 0.15 | - | STUCK | no | 0.279 | -0.017 | 4 | 0.3240 | 0.0050 | 2.417 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.272 | 0.129 | 5 | 0.3217 | 0.0050 | 2.417 | 0 | 0 |

#### Thin Plank / box tiles / gravity scale 2000

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.040 | -0.000 | 1 | 0.0000 | 0.0704 | 19.999 | 0 | 0 |
| semrau | - | STUCK | no | 0.120 | 0.000 | 1 | 0.0000 | 0.0704 | 0.000 | 0 | 0 |
| topological | - | bumpy | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0371 | 9.587 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.040 | 0.000 | 1 | 0.0000 | 0.0704 | 13.599 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.040 | -0.000 | 1 | 0.0000 | 0.0704 | 0.000 | 0 | 0 |
| rounded 0.08 | - | STUCK | no | 0.040 | -0.000 | 1 | 0.0000 | 0.0704 | 0.000 | 0 | 0 |
| rounded 0.10 | - | STUCK | no | 0.040 | 0.000 | 1 | 0.0000 | 0.0704 | 0.000 | 0 | 0 |
| rounded 0.15 | - | STUCK | no | 0.040 | -0.001 | 1 | 0.0000 | 0.0704 | 0.021 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.040 | -0.007 | 1 | 0.0000 | 0.0704 | 0.068 | 0 | 0 |

#### Thin Plank / chain / gravity scale 2000

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | bumpy | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0345 | 19.997 | 0 | 0 |
| semrau | - | FELL | no | 0.071 | 0.000 | 4 | 0.0000 | 1870.1772 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0345 | 0.000 | 0 | 0 |
| rounded 0.02 | - | FELL | no | 0.071 | 0.000 | 4 | 0.0000 | 1870.1772 | 0.000 | 0 | 0 |
| rounded 0.05 | - | FELL | no | 0.071 | 0.000 | 4 | 0.0000 | 1870.1772 | 0.000 | 0 | 0 |
| rounded 0.08 | - | FELL | no | 0.071 | 0.000 | 4 | 0.0000 | 1870.1772 | 0.000 | 0 | 0 |
| rounded 0.10 | - | FELL | no | 0.071 | 0.000 | 4 | 0.0000 | 1870.1772 | 0.000 | 0 | 0 |
| rounded 0.15 | - | FELL | no | 0.071 | 0.000 | 4 | 0.0000 | 1870.1772 | 0.000 | 0 | 0 |
| rounded 0.25 | - | FELL | no | 0.071 | 0.000 | 4 | 0.0000 | 1870.1772 | 0.000 | 0 | 0 |

#### Thin Plank / custom ghost segments / gravity scale 2000

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | bumpy | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0343 | 19.999 | 0 | 0 |
| semrau | - | FELL | no | 0.138 | 0.000 | 4 | 0.0000 | 1736.8431 | 0.000 | 0 | 0 |
| topological | - | bumpy | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0371 | 9.587 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0343 | 13.599 | 0 | 0 |
| rounded 0.05 | - | FELL | no | 0.138 | 0.000 | 4 | 0.0000 | 1736.8431 | 0.000 | 0 | 0 |
| rounded 0.08 | - | FELL | no | 0.298 | 0.000 | 4 | 1.9795 | 1403.5114 | 327.733 | 0 | 0 |
| rounded 0.10 | - | FELL | no | 0.298 | 0.000 | 4 | 1.8726 | 1403.5114 | 321.333 | 0 | 0 |
| rounded 0.15 | - | FELL | no | 0.298 | 0.000 | 4 | 1.7339 | 1403.5114 | 313.027 | 0 | 0 |
| rounded 0.25 | - | FELL | no | 0.298 | 0.000 | 4 | 1.5685 | 1403.5114 | 303.124 | 0 | 0 |

#### Fast Landing / box tiles / speed 20

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.044 | 0.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.331 | 0.029 | 2 | 4.9156 | 0.0438 | 6.944 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.427 | 0.181 | 2 | 5.0226 | 0.1058 | 7.226 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.569 | 0.448 | 2 | 0.9980 | 0.0226 | 3.786 | 0 | 0 |

#### Fast Landing / box tiles / speed 40

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.377 | 0.353 | 0 | 18.4953 | 0.0000 | 15.254 | 0 | 0 |
| semrau | - | STUCK | no | 0.377 | 0.353 | 0 | 18.4953 | 0.0000 | 15.254 | 0 | 0 |
| topological | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.443 | 0.419 | 0 | 19.5884 | 0.0000 | 16.818 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.490 | 0.468 | 0 | 20.0247 | 0.0000 | 17.375 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 0.997 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.894 | 0.873 | 0 | 7.4441 | 0.0050 | 10.901 | 0 | 0 |

#### Fast Landing / box tiles / speed 80

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.371 | 0.346 | 0 | 72.8986 | 0.0000 | 34.204 | 0 | 0 |
| semrau | - | STUCK | no | 0.371 | 0.346 | 0 | 72.8986 | 0.0000 | 34.204 | 0 | 0 |
| topological | - | ok | yes | 0.996 | 1.000 | 0 | 0.0000 | -0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.444 | 0.420 | 0 | 78.0854 | 0.0000 | 36.565 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.491 | 0.469 | 0 | 79.7522 | 0.0000 | 37.338 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 0.996 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 0.996 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 0.996 | 1.000 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.841 | 0.833 | 0 | 25.8226 | 0.0000 | 20.919 | 0 | 0 |

#### Compound Player / box tiles / gravity scale 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.020 | 0.000 | 1 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.020 | -0.000 | 1 | 0.0001 | 0.0262 | 0.103 | 0 | 0 |
| rounded 0.05 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.218 | -0.023 | 12 | 0.0675 | 0.0185 | 2.437 | 0 | 0 |

#### Small Gaps / segment tiles / gap 0.01

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.030 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.030 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 0.997 | 0.970 | 0 | 0.1569 | 0.0000 | 1.634 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 0.948 | 0.797 | 1 | 0.8069 | 0.0000 | 3.886 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.965 | 0.876 | 0 | 0.3152 | 0.0000 | 2.380 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 0.920 | 0.736 | 1 | 0.8706 | 0.0003 | 4.059 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.927 | 0.856 | 1 | 0.4473 | 0.0000 | 2.875 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.965 | 0.762 | 1 | 0.3540 | 0.0000 | 2.898 | 0 | 0 |

#### Small Gaps / segment tiles / gap 0.05

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.237 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.237 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 0.868 | 0.822 | 1 | 0.6302 | 0.0210 | 3.412 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.529 | 0.163 | 5 | 1.1001 | 0.0467 | 4.565 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.840 | 0.758 | 1 | 0.6117 | 0.0000 | 3.374 | 0 | 0 |
| rounded 0.10 | - | STUCK | no | 0.508 | 0.293 | 5 | 0.9369 | 0.0294 | 4.231 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.851 | 0.611 | 2 | 0.8107 | 0.0279 | 3.925 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.833 | 0.707 | 0 | 0.4218 | 0.0037 | 2.801 | 0 | 0 |

#### Small Gaps / segment tiles / gap 0.2

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.170 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.170 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.508 | 0.145 | 2 | 1.2747 | 0.0278 | 4.917 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.476 | 0.208 | 4 | 0.9836 | 0.0400 | 4.330 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.675 | 0.330 | 3 | 0.9271 | 0.0117 | 4.188 | 0 | 0 |
| rounded 0.10 | - | STUCK | no | 0.567 | 0.374 | 3 | 0.9173 | 0.0296 | 4.183 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.818 | 0.691 | 2 | 0.4810 | 0.0072 | 2.984 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.924 | 0.858 | 0 | 0.2929 | 0.0071 | 2.310 | 0 | 0 |

### Slopes, platforms, crests

#### Slope Tiles / box, triangle tiles / speed 5

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.860 | 0.000 | 1 | 0.0002 | 0.0000 | 0.017 | 0 | 0 |
| semrau | - | ok | yes | 2.233 | 1.100 | 0 | 0.0002 | 0.0000 | 0.045 | 0 | 0 |
| topological | - | ok | yes | 2.233 | 1.100 | 0 | 0.0002 | 0.0000 | 0.045 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 1.909 | 1.100 | 1 | 1.7770 | 0.0066 | 5.429 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 1.757 | 0.843 | 2 | 1.0255 | 0.0275 | 4.119 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 2.018 | 0.755 | 1 | 0.4019 | 0.0047 | 2.560 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 2.173 | 1.100 | 0 | 0.5609 | 0.0000 | 3.005 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 1.914 | 1.079 | 3 | 1.1475 | 0.0000 | 5.480 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 2.104 | 1.100 | 2 | 0.5853 | 0.0300 | 4.934 | 0 | 0 |

#### Slope Tiles / free box, triangle tiles / speed 5

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | bumpy | yes | 1.918 | 1.002 | 1 | 2.8017 | 0.0000 | 6.957 | 0 | 0 |
| semrau | - | bumpy | yes | 1.965 | 1.025 | 1 | 2.6457 | 0.0001 | 7.097 | 0 | 0 |
| topological | - | ok | yes | 2.232 | 1.100 | 0 | 0.0000 | 0.0000 | 0.045 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 1.531 | 0.803 | 2 | 1.2192 | 0.0033 | 4.371 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 1.804 | 0.946 | 4 | 0.7870 | 0.0000 | 3.332 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 1.994 | 0.785 | 1 | 0.3398 | 0.0001 | 2.346 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 2.154 | 1.100 | 1 | 0.8853 | 0.0001 | 3.791 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 1.996 | 1.097 | 2 | 1.8410 | 0.0000 | 5.415 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 2.029 | 1.100 | 1 | 0.5847 | 0.0000 | 2.928 | 0 | 0 |

#### Moving Platform / box tiles terrain / platform speed 1

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.147 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.445 | -0.010 | 4 | 0.9124 | 0.0499 | 4.134 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.514 | 0.186 | 4 | 1.1318 | 0.0217 | 4.633 | 0 | 0 |
| rounded 0.08 | - | STUCK | no | 0.598 | 0.278 | 4 | 0.6770 | 0.0197 | 3.553 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 0.600 | 0.269 | 5 | 0.9019 | 0.0078 | 4.131 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.830 | 0.625 | 2 | 0.6251 | 0.0164 | 3.438 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.877 | 0.804 | 0 | 0.3834 | 0.0244 | 2.665 | 0 | 0 |

#### Moving Platform / chain terrain / platform speed 1

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.02 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 0.854 | 0.592 | 1 | 1.3810 | 0.0001 | 5.130 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.903 | 0.812 | 1 | 0.8745 | 0.0497 | 4.055 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 0.991 | 0.987 | 0 | 0.0622 | 0.0001 | 0.986 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.899 | 0.844 | 1 | 0.5214 | 0.0010 | 3.121 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.985 | 0.970 | 0 | 0.1537 | 0.0001 | 1.631 | 0 | 0 |

#### Rotating Platform / box tiles / amplitude (deg) 0.5

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.511 | 0.000 | 1 | 0.0053 | 0.0010 | 0.054 | 0 | 0 |
| semrau | - | ok | yes | 1.001 | 1.000 | 0 | 0.0053 | 0.0016 | 0.054 | 0 | 0 |
| topological | - | ok | yes | 1.001 | 1.000 | 0 | 0.0035 | 0.0016 | 0.055 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.704 | 0.158 | 2 | 1.2784 | 0.0205 | 4.924 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 0.888 | 0.641 | 1 | 1.1110 | 0.0248 | 4.588 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.838 | 0.795 | 1 | 0.8033 | 0.0013 | 3.882 | 0 | 0 |
| rounded 0.10 | - | STUCK | no | 0.543 | 0.208 | 5 | 0.6041 | 0.0089 | 3.353 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.870 | 0.691 | 1 | 0.7237 | 0.0136 | 3.681 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.899 | 0.633 | 2 | 0.6578 | 0.0099 | 3.533 | 0 | 0 |

#### Rotating Platform / chain / amplitude (deg) 0.5

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok | yes | 1.001 | 1.000 | 0 | 0.0053 | 0.0010 | 0.055 | 0 | 0 |
| semrau | - | ok | yes | 1.001 | 1.000 | 0 | 0.0053 | 0.0016 | 0.054 | 0 | 0 |
| topological | - | ok | yes | 1.001 | 1.000 | 0 | 0.0035 | 0.0015 | 0.055 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 0.994 | 0.975 | 0 | 0.1402 | 0.0012 | 1.537 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 0.893 | 0.687 | 1 | 1.1108 | 0.0250 | 4.587 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.822 | 0.628 | 1 | 0.7958 | 0.0045 | 3.853 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 0.831 | 0.729 | 2 | 0.7532 | 0.0181 | 3.759 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.976 | 0.953 | 0 | 0.2452 | 0.0121 | 2.085 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.978 | 0.945 | 0 | 0.2086 | 0.0133 | 1.913 | 0 | 0 |

#### Vehicle Crest / vehicle, box + slope tiles / speed 8

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | bumpy | yes | 0.902 | 0.872 | 0 | 0.1968 | 0.0001 | 1.381 | 0 | 0 |
| semrau | - | bumpy | yes | 0.902 | 0.872 | 0 | 0.1968 | 0.0001 | 1.381 | 0 | 0 |
| topological | - | ok | yes | 1.064 | 1.151 | 0 | 0.0000 | 0.0012 | 0.000 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 0.902 | 0.846 | 0 | 0.2289 | 0.0001 | 1.543 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 0.904 | 0.863 | 0 | 0.1614 | 0.0037 | 1.170 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.905 | 0.858 | 0 | 0.1795 | 0.0010 | 1.269 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 0.905 | 0.859 | 0 | 0.1977 | 0.0029 | 1.388 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.908 | 0.858 | 0 | 0.1523 | 0.0000 | 1.123 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.910 | 0.872 | 0 | 0.1612 | 0.0003 | 1.162 | 0 | 0 |

#### Vehicle Crest / plank, box + slope tiles / speed 8

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | -0.001 | 0.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.038 | 1.114 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| topological | - | ok | yes | 1.038 | 1.113 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | ok | yes | 1.042 | 1.112 | 0 | 0.0047 | 0.0000 | 0.181 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 1.013 | 0.767 | 1 | 0.6373 | 0.0000 | 3.222 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 1.018 | 1.039 | 0 | 0.2375 | 0.0002 | 1.740 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 0.881 | 0.909 | 0 | 0.1613 | 0.0000 | 1.413 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.933 | 0.889 | 1 | 0.4057 | 0.0003 | 2.288 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 1.035 | 0.968 | 1 | 0.3459 | 0.0000 | 2.272 | 0 | 0 |

#### Vehicle Crest / locked box, box + slope tiles / speed 8

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 1.035 | 0.881 | 1 | 0.7713 | 0.0000 | 3.800 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 0.997 | 0.536 | 1 | 0.4689 | 0.0084 | 3.952 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 0.833 | 0.515 | 1 | 0.9073 | 0.0496 | 3.479 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0007 | 0.000 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 0.842 | 0.649 | 2 | 0.4233 | 0.0001 | 2.810 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 0.874 | 0.708 | 2 | 0.2716 | 0.0081 | 2.264 | 0 | 0 |

#### Vehicle Crest / locked box, chain / speed 8

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.05 | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0003 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.062 | 1.151 | 0 | 0.0000 | 0.0008 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.063 | 1.151 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | ok | yes | 1.066 | 1.158 | 0 | 0.0000 | 0.0010 | 0.000 | 0 | 0 |

#### Wall Slide Down / segment wall tiles / gravity scale 2

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.750 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | slowed | yes | 3.409 | 0.000 | 2 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 4.937 | 2.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | bumpy | yes | 3.298 | 0.997 | 2 | 0.3384 | 0.0145 | 3.450 | 0 | 0 |
| rounded 0.05 | - | bumpy | yes | 4.540 | 1.987 | 1 | 0.6577 | 0.0000 | 4.871 | 0 | 0 |
| rounded 0.08 | - | bumpy | yes | 4.339 | 1.873 | 2 | 0.4209 | 0.0000 | 5.705 | 0 | 0 |
| rounded 0.10 | - | bumpy | yes | 4.025 | 1.792 | 2 | 0.3503 | 0.0188 | 4.308 | 0 | 0 |
| rounded 0.15 | - | bumpy | yes | 4.225 | 1.766 | 2 | 0.6350 | 0.0105 | 4.833 | 0 | 0 |
| rounded 0.25 | - | bumpy | yes | 4.666 | 2.003 | 0 | 0.1192 | 0.0000 | 1.967 | 0 | 0 |

### Multi-body: Shape Parade and Parabola Hills parade (verdict vs the same body on the smooth reference)

#### Shape Parade / slope triangle tiles / speed 6

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | triangle | bumpy | yes | 1.421 | 0.101 | 6 | 0.6833 | 0.0001 | 2.607 | 0 | 0 |
| none | pentagon | bumpy | yes | 1.689 | 0.595 | 5 | 0.8061 | 0.0001 | 3.547 | 0 | 0 |
| none | hexagon | bumpy | yes | 1.738 | 0.595 | 3 | 0.3276 | 0.0001 | 2.206 | 0 | 0 |
| none | box | bumpy | yes | 1.614 | 0.487 | 4 | 0.5923 | 0.0001 | 2.600 | 0 | 0 |
| semrau | triangle | bumpy | yes | 1.418 | 0.101 | 6 | 0.6829 | 0.0001 | 2.607 | 0 | 0 |
| semrau | pentagon | ok | yes | 1.865 | 1.058 | 0 | 0.0006 | 0.0002 | 0.138 | 0 | 0 |
| semrau | hexagon | bumpy | yes | 1.737 | 0.591 | 3 | 0.2306 | 0.0001 | 1.661 | 0 | 0 |
| semrau | box | ok | yes | 1.860 | 1.058 | 0 | 0.0007 | 0.0001 | 0.141 | 0 | 0 |
| topological | triangle | ok | yes | 1.859 | 1.058 | 0 | 0.0006 | 0.0002 | 0.137 | 0 | 0 |
| topological | pentagon | ok | yes | 1.865 | 1.058 | 0 | 0.0007 | 0.0002 | 0.138 | 0 | 0 |
| topological | hexagon | ok | yes | 1.867 | 1.058 | 0 | 0.0007 | 0.0002 | 0.138 | 0 | 0 |
| topological | box | ok | yes | 1.860 | 1.058 | 0 | 0.0007 | 0.0002 | 0.141 | 0 | 0 |
| rounded 0.02 | triangle | bumpy | yes | 1.395 | 0.292 | 4 | 1.5160 | 0.0049 | 4.756 | 0 | 0 |
| rounded 0.02 | pentagon | bumpy | yes | 1.692 | 0.895 | 2 | 0.9166 | 0.0001 | 3.743 | 0 | 0 |
| rounded 0.02 | hexagon | bumpy | yes | 1.739 | 0.590 | 4 | 0.8303 | 0.0062 | 3.682 | 0 | 0 |
| rounded 0.02 | box | bumpy | yes | 1.479 | 0.520 | 3 | 1.6153 | 0.0042 | 4.943 | 0 | 0 |
| rounded 0.05 | triangle | bumpy | yes | 1.376 | 0.177 | 4 | 0.5187 | 0.0051 | 2.164 | 0 | 0 |
| rounded 0.05 | pentagon | bumpy | yes | 1.827 | 0.875 | 2 | 0.4144 | 0.0019 | 2.472 | 0 | 0 |
| rounded 0.05 | hexagon | bumpy | yes | 1.745 | 0.552 | 3 | 0.3412 | 0.0052 | 2.101 | 0 | 0 |
| rounded 0.05 | box | bumpy | yes | 1.621 | 0.561 | 3 | 1.2025 | 0.0001 | 3.761 | 0 | 0 |
| rounded 0.08 | triangle | bumpy | yes | 1.233 | 0.318 | 6 | 1.1081 | 0.0050 | 3.937 | 0 | 0 |
| rounded 0.08 | pentagon | bumpy | yes | 1.803 | 0.656 | 4 | 0.3243 | 0.0056 | 2.057 | 0 | 0 |
| rounded 0.08 | hexagon | bumpy | yes | 1.735 | 0.538 | 3 | 0.4731 | 0.0058 | 2.726 | 0 | 0 |
| rounded 0.08 | box | bumpy | yes | 1.571 | 0.507 | 5 | 1.2747 | 0.0052 | 4.374 | 0 | 0 |
| rounded 0.10 | triangle | bumpy | yes | 1.625 | 0.444 | 4 | 0.7254 | 0.0061 | 3.325 | 0 | 0 |
| rounded 0.10 | pentagon | bumpy | yes | 1.773 | 0.600 | 3 | 0.3294 | 0.0052 | 1.890 | 0 | 0 |
| rounded 0.10 | hexagon | bumpy | yes | 1.723 | 0.600 | 5 | 0.4432 | 0.0015 | 2.370 | 0 | 0 |
| rounded 0.10 | box | bumpy | yes | 1.835 | 0.684 | 3 | 0.7829 | 0.0041 | 3.739 | 0 | 0 |
| rounded 0.15 | triangle | bumpy | yes | 1.826 | 0.858 | 5 | 0.8686 | 0.0054 | 3.641 | 0 | 0 |
| rounded 0.15 | pentagon | bumpy | yes | 1.711 | 0.588 | 5 | 0.3143 | 0.0055 | 2.230 | 0 | 0 |
| rounded 0.15 | hexagon | bumpy | yes | 1.861 | 1.054 | 2 | 0.3701 | 0.0053 | 2.344 | 0 | 0 |
| rounded 0.15 | box | bumpy | yes | 1.682 | 0.620 | 5 | 0.3760 | 0.0057 | 2.017 | 0 | 0 |
| rounded 0.25 | triangle | bumpy | yes | 1.974 | 1.055 | 0 | 0.2202 | 0.0060 | 1.728 | 0 | 0 |
| rounded 0.25 | pentagon | bumpy | yes | 1.860 | 1.052 | 2 | 0.1963 | 0.0050 | 1.675 | 0 | 0 |
| rounded 0.25 | hexagon | bumpy | yes | 1.915 | 1.052 | 1 | 0.1624 | 0.0051 | 1.439 | 0 | 0 |
| rounded 0.25 | box | bumpy | yes | 1.833 | 0.791 | 3 | 0.2504 | 0.0058 | 1.695 | 0 | 0 |

#### Shape Parade / slope smooth chain / speed 6

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | triangle | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| none | pentagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| none | hexagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.003 | 0 | 0 |
| none | box | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| semrau | triangle | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| semrau | pentagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| semrau | hexagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.003 | 0 | 0 |
| semrau | box | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.002 | 0 | 0 |
| topological | triangle | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| topological | pentagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| topological | hexagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.003 | 0 | 0 |
| topological | box | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| rounded 0.02 | triangle | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| rounded 0.02 | pentagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| rounded 0.02 | hexagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.003 | 0 | 0 |
| rounded 0.02 | box | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| rounded 0.05 | triangle | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| rounded 0.05 | pentagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| rounded 0.05 | hexagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.005 | 0 | 0 |
| rounded 0.05 | box | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| rounded 0.08 | triangle | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| rounded 0.08 | pentagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.006 | 0 | 0 |
| rounded 0.08 | hexagon | bumpy | yes | 1.978 | 1.056 | 0 | 0.0768 | 0.0002 | 0.850 | 0 | 0 |
| rounded 0.08 | box | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| rounded 0.10 | triangle | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| rounded 0.10 | pentagon | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.003 | 0 | 0 |
| rounded 0.10 | hexagon | bumpy | yes | 1.978 | 1.055 | 0 | 0.1698 | 0.0002 | 1.573 | 0 | 0 |
| rounded 0.10 | box | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.001 | 0 | 0 |
| rounded 0.15 | triangle | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0001 | 0.002 | 0 | 0 |
| rounded 0.15 | pentagon | ok | yes | 1.980 | 1.056 | 0 | 0.0056 | 0.0002 | 0.152 | 0 | 0 |
| rounded 0.15 | hexagon | bumpy | yes | 1.980 | 1.053 | 0 | 0.1044 | 0.0010 | 1.050 | 0 | 0 |
| rounded 0.15 | box | ok | yes | 1.982 | 1.058 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| rounded 0.25 | triangle | bumpy | yes | 1.980 | 1.054 | 0 | 0.0691 | 0.0038 | 0.847 | 0 | 0 |
| rounded 0.25 | pentagon | bumpy | yes | 1.981 | 1.052 | 0 | 0.1250 | 0.0029 | 1.157 | 0 | 0 |
| rounded 0.25 | hexagon | bumpy | yes | 1.978 | 1.052 | 0 | 0.0787 | 0.0026 | 1.085 | 0 | 0 |
| rounded 0.25 | box | ok | yes | 1.980 | 1.057 | 0 | 0.0064 | 0.0002 | 0.179 | 0 | 0 |

#### Shape Parade / parabola crates (static) / speed 6

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | triangle | bumpy | yes | 0.688 | 0.134 | 3 | 0.3424 | 0.0022 | 1.965 | 0 | 0 |
| none | pentagon | bumpy | yes | 0.756 | 0.453 | 5 | 0.2462 | 0.0037 | 2.194 | 0 | 0 |
| none | hexagon | bumpy | yes | 0.889 | 0.641 | 5 | 0.2132 | 0.0028 | 2.109 | 0 | 0 |
| none | box | STUCK | no | 0.071 | -0.001 | 5 | 0.3016 | 0.0058 | 2.169 | 0 | 0 |
| semrau | triangle | STUCK | no | 0.073 | -0.100 | 6 | 0.3424 | 0.0022 | 1.744 | 0 | 0 |
| semrau | pentagon | bumpy | yes | 0.845 | 0.550 | 4 | 0.2268 | 0.0138 | 2.358 | 0 | 0 |
| semrau | hexagon | bumpy | yes | 0.889 | 0.641 | 5 | 0.2132 | 0.0028 | 2.109 | 0 | 0 |
| semrau | box | bumpy | yes | 0.894 | 0.372 | 1 | 0.2402 | 0.0168 | 2.747 | 0 | 0 |
| topological | triangle | ok | yes | 0.922 | 0.832 | 0 | 0.0079 | 0.0188 | 0.613 | 0 | 0 |
| topological | pentagon | ok | yes | 0.951 | 0.752 | 3 | 0.0540 | 0.0121 | 1.101 | 0 | 0 |
| topological | hexagon | ok | yes | 0.987 | 0.895 | 3 | 0.0182 | 0.0084 | 0.780 | 0 | 0 |
| topological | box | ok | yes | 0.928 | 0.726 | 0 | 0.0106 | 0.0111 | 0.650 | 0 | 0 |
| rounded 0.02 | triangle | STUCK | no | 0.071 | -0.000 | 9 | 0.2845 | 0.0043 | 2.509 | 0 | 0 |
| rounded 0.02 | pentagon | bumpy | yes | 0.786 | 0.454 | 4 | 0.2410 | 0.0060 | 2.105 | 0 | 0 |
| rounded 0.02 | hexagon | bumpy | yes | 0.988 | 0.717 | 3 | 0.1716 | 0.0059 | 1.585 | 0 | 0 |
| rounded 0.02 | box | bumpy | yes | 0.678 | 0.278 | 3 | 0.2143 | 0.0069 | 1.571 | 0 | 0 |
| rounded 0.05 | triangle | STUCK | no | 0.052 | -0.001 | 4 | 0.3243 | 0.0217 | 3.329 | 0 | 0 |
| rounded 0.05 | pentagon | ok | yes | 0.988 | 0.823 | 2 | 0.0939 | 0.0087 | 1.325 | 0 | 0 |
| rounded 0.05 | hexagon | bumpy | yes | 0.963 | 0.674 | 4 | 0.2081 | 0.0049 | 2.012 | 0 | 0 |
| rounded 0.05 | box | bumpy | yes | 0.735 | 0.395 | 3 | 0.2511 | 0.0043 | 1.634 | 0 | 0 |
| rounded 0.08 | triangle | bumpy | yes | 0.799 | 0.444 | 6 | 0.2088 | 0.0051 | 1.822 | 0 | 0 |
| rounded 0.08 | pentagon | ok | yes | 1.005 | 0.784 | 2 | 0.1401 | 0.0045 | 1.502 | 0 | 0 |
| rounded 0.08 | hexagon | bumpy | yes | 0.964 | 0.648 | 4 | 0.1507 | 0.0057 | 2.067 | 0 | 0 |
| rounded 0.08 | box | bumpy | yes | 0.613 | 0.191 | 7 | 0.2525 | 0.0283 | 1.482 | 0 | 0 |
| rounded 0.10 | triangle | STUCK | no | 0.058 | -0.000 | 5 | 0.3011 | 0.0052 | 1.817 | 0 | 0 |
| rounded 0.10 | pentagon | slowed | yes | 0.937 | 0.685 | 5 | 0.1260 | 0.0050 | 1.385 | 0 | 0 |
| rounded 0.10 | hexagon | slowed | yes | 0.952 | 0.665 | 4 | 0.1242 | 0.0059 | 1.488 | 0 | 0 |
| rounded 0.10 | box | bumpy | yes | 0.811 | 0.481 | 4 | 0.2858 | 0.0053 | 1.989 | 0 | 0 |
| rounded 0.15 | triangle | bumpy | yes | 0.797 | 0.280 | 4 | 0.1576 | 0.0052 | 2.253 | 0 | 0 |
| rounded 0.15 | pentagon | bumpy | yes | 0.974 | 0.714 | 4 | 0.2020 | 0.0052 | 1.976 | 0 | 0 |
| rounded 0.15 | hexagon | bumpy | yes | 0.951 | 0.666 | 2 | 0.1326 | 0.0056 | 1.739 | 0 | 0 |
| rounded 0.15 | box | bumpy | yes | 0.839 | 0.575 | 4 | 0.2507 | 0.0062 | 2.169 | 0 | 0 |
| rounded 0.25 | triangle | bumpy | yes | 0.901 | 0.671 | 3 | 0.2370 | 0.0054 | 2.380 | 0 | 0 |
| rounded 0.25 | pentagon | ok | yes | 0.975 | 0.790 | 2 | 0.0929 | 0.0059 | 1.299 | 0 | 0 |
| rounded 0.25 | hexagon | ok | yes | 0.953 | 0.735 | 3 | 0.0786 | 0.0052 | 0.775 | 0 | 0 |
| rounded 0.25 | box | bumpy | yes | 0.908 | 0.660 | 3 | 0.1644 | 0.0047 | 1.832 | 0 | 0 |

#### Shape Parade / dynamic crate floor / speed 6

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | triangle | STUCK | no | 0.140 | -0.008 | 8 | 0.2866 | 0.0027 | 1.031 | 0 | 0 |
| none | pentagon | bumpy | yes | 0.921 | 0.460 | 11 | 0.3018 | 0.0332 | 2.031 | 0 | 0 |
| none | hexagon | bumpy | yes | 0.867 | 0.540 | 14 | 0.4700 | 0.0237 | 2.797 | 0 | 0 |
| none | box | STUCK | no | 0.350 | 0.029 | 11 | 0.2749 | 0.0418 | 1.508 | 0 | 0 |
| semrau | triangle | ok | yes | 0.987 | 0.983 | 0 | 0.0000 | 0.0012 | 0.021 | 0 | 0 |
| semrau | pentagon | ok | yes | 0.987 | 0.985 | 0 | 0.0000 | 0.0012 | 0.016 | 0 | 0 |
| semrau | hexagon | ok | yes | 0.987 | 0.985 | 0 | 0.0000 | 0.0014 | 0.017 | 0 | 0 |
| semrau | box | ok | yes | 0.987 | 0.986 | 0 | 0.0000 | 0.0009 | 0.001 | 0 | 0 |
| topological | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0009 | 0.000 | 0 | 0 |
| topological | pentagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0008 | 0.000 | 0 | 0 |
| topological | hexagon | ok | yes | 0.986 | 0.985 | 0 | 0.0000 | 0.0009 | 0.002 | 0 | 0 |
| topological | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0008 | 0.000 | 0 | 0 |
| rounded 0.02 | triangle | STUCK | no | 0.092 | -0.008 | 5 | 0.3300 | 0.0010 | 1.297 | 0 | 0 |
| rounded 0.02 | pentagon | bumpy | yes | 0.633 | 0.474 | 24 | 0.2137 | 0.0108 | 1.628 | 0 | 0 |
| rounded 0.02 | hexagon | bumpy | yes | 0.929 | 0.566 | 15 | 0.1937 | 0.0173 | 1.700 | 0 | 0 |
| rounded 0.02 | box | STUCK | no | 0.263 | -0.023 | 20 | 0.2298 | 0.0097 | 1.250 | 0 | 0 |
| rounded 0.05 | triangle | STUCK | no | 0.258 | 0.021 | 11 | 0.1805 | 0.0206 | 0.954 | 0 | 0 |
| rounded 0.05 | pentagon | bumpy | yes | 0.957 | 0.530 | 12 | 0.3494 | 0.0374 | 2.495 | 0 | 0 |
| rounded 0.05 | hexagon | bumpy | yes | 1.037 | 0.560 | 10 | 0.4104 | 0.0358 | 2.591 | 0 | 0 |
| rounded 0.05 | box | STUCK | no | 0.237 | -0.020 | 21 | 0.1992 | 0.0076 | 1.081 | 0 | 0 |
| rounded 0.08 | triangle | STUCK | no | 0.140 | -0.007 | 10 | 0.3323 | 0.0155 | 1.440 | 0 | 0 |
| rounded 0.08 | pentagon | bumpy | yes | 1.067 | 0.771 | 11 | 0.4992 | 0.0094 | 2.858 | 0 | 0 |
| rounded 0.08 | hexagon | bumpy | yes | 1.016 | 0.618 | 10 | 0.2364 | 0.0214 | 1.852 | 0 | 0 |
| rounded 0.08 | box | bumpy | yes | 0.782 | 0.395 | 17 | 0.4040 | 0.0245 | 2.336 | 0 | 0 |
| rounded 0.10 | triangle | STUCK | no | 0.141 | -0.000 | 13 | 0.2969 | 0.0039 | 2.411 | 0 | 0 |
| rounded 0.10 | pentagon | bumpy | yes | 0.938 | 0.681 | 14 | 0.2733 | 0.0134 | 2.036 | 0 | 0 |
| rounded 0.10 | hexagon | bumpy | yes | 1.183 | 0.863 | 6 | 0.3200 | 0.0227 | 2.397 | 0 | 0 |
| rounded 0.10 | box | bumpy | yes | 0.891 | 0.405 | 14 | 0.6977 | 0.0373 | 3.471 | 0 | 0 |
| rounded 0.15 | triangle | STUCK | no | 0.332 | -0.021 | 25 | 0.2039 | 0.0128 | 1.425 | 0 | 0 |
| rounded 0.15 | pentagon | bumpy | yes | 0.992 | 0.616 | 11 | 0.1932 | 0.0091 | 1.762 | 0 | 0 |
| rounded 0.15 | hexagon | bumpy | yes | 1.210 | 0.799 | 9 | 0.3610 | 0.0252 | 2.528 | 0 | 0 |
| rounded 0.15 | box | bumpy | yes | 0.999 | 0.439 | 11 | 0.8079 | 0.0376 | 3.569 | 0 | 0 |
| rounded 0.25 | triangle | bumpy | yes | 1.241 | 0.834 | 7 | 0.2060 | 0.0230 | 1.900 | 0 | 0 |
| rounded 0.25 | pentagon | bumpy | yes | 1.229 | 0.830 | 6 | 0.1026 | 0.0056 | 1.225 | 0 | 0 |
| rounded 0.25 | hexagon | bumpy | yes | 1.209 | 0.791 | 2 | 0.1254 | 0.0050 | 1.409 | 0 | 0 |
| rounded 0.25 | box | bumpy | yes | 1.146 | 0.758 | 9 | 0.4966 | 0.0352 | 2.851 | 0 | 0 |

#### Shape Parade / flat smooth chain / speed 6

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| none | pentagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| none | hexagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.004 | 0 | 0 |
| none | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| semrau | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| semrau | pentagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| semrau | hexagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.004 | 0 | 0 |
| semrau | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| topological | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| topological | pentagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| topological | hexagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.004 | 0 | 0 |
| topological | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.02 | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| rounded 0.02 | pentagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| rounded 0.02 | hexagon | ok | yes | 0.986 | 0.985 | 0 | 0.0000 | 0.0002 | 0.005 | 0 | 0 |
| rounded 0.02 | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.05 | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| rounded 0.05 | pentagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.001 | 0 | 0 |
| rounded 0.05 | hexagon | ok | yes | 0.986 | 0.985 | 0 | 0.0000 | 0.0002 | 0.008 | 0 | 0 |
| rounded 0.05 | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.08 | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| rounded 0.08 | pentagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.002 | 0 | 0 |
| rounded 0.08 | hexagon | bumpy | yes | 1.048 | 0.871 | 13 | 0.2159 | 0.0185 | 1.932 | 0 | 0 |
| rounded 0.08 | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.10 | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.10 | pentagon | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.006 | 0 | 0 |
| rounded 0.10 | hexagon | bumpy | yes | 1.109 | 0.872 | 12 | 0.3005 | 0.0094 | 2.334 | 0 | 0 |
| rounded 0.10 | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.15 | triangle | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0001 | 0.000 | 0 | 0 |
| rounded 0.15 | pentagon | bumpy | yes | 1.074 | 0.863 | 13 | 0.2052 | 0.0246 | 1.689 | 0 | 0 |
| rounded 0.15 | hexagon | bumpy | yes | 1.181 | 0.874 | 6 | 0.2185 | 0.0187 | 1.914 | 0 | 0 |
| rounded 0.15 | box | ok | yes | 0.986 | 0.986 | 0 | 0.0000 | 0.0002 | 0.000 | 0 | 0 |
| rounded 0.25 | triangle | bumpy | yes | 1.226 | 0.905 | 6 | 0.1521 | 0.0245 | 1.559 | 0 | 0 |
| rounded 0.25 | pentagon | bumpy | yes | 1.278 | 0.928 | 0 | 0.0764 | 0.0166 | 1.710 | 0 | 0 |
| rounded 0.25 | hexagon | bumpy | yes | 1.280 | 0.923 | 1 | 0.1771 | 0.0096 | 1.742 | 0 | 0 |
| rounded 0.25 | box | ok | yes | 0.985 | 0.905 | 15 | 0.0118 | 0.0002 | 0.213 | 0 | 0 |

#### Parabola Hills / parade, static slices / speed 3

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | triangle | LIFTS OFF | no | 0.102 | -0.015 | 6 | 0.1399 | 0.0002 | 0.594 | 32 | 2 |
| none | pentagon | bumpy | yes | 0.946 | 0.702 | 5 | 0.2434 | 0.0168 | 2.492 | 54 | 5 |
| none | hexagon | LIFTS OFF | yes | 0.878 | 0.661 | 6 | 0.1176 | 0.0000 | 1.270 | 88 | 7 |
| none | box | LIFTS OFF | yes | 0.800 | 0.361 | 5 | 0.3087 | 0.0055 | 1.631 | 73 | 6 |
| semrau | triangle | LIFTS OFF | no | 0.107 | -0.000 | 6 | 0.1400 | 0.0024 | 0.594 | 32 | 2 |
| semrau | pentagon | bumpy | yes | 0.946 | 0.702 | 5 | 0.2434 | 0.0168 | 2.492 | 54 | 5 |
| semrau | hexagon | LIFTS OFF | yes | 0.921 | 0.586 | 5 | 0.2911 | 0.0000 | 2.114 | 86 | 4 |
| semrau | box | ok | yes | 0.959 | 0.834 | 1 | 0.0171 | 0.0252 | 1.157 | 0 | 0 |
| topological | triangle | ok | yes | 0.948 | 0.853 | 1 | 0.0100 | 0.0172 | 0.436 | 0 | 0 |
| topological | pentagon | ok | yes | 0.955 | 0.761 | 5 | 0.0731 | 0.0168 | 0.608 | 42 | 4 |
| topological | hexagon | bumpy | yes | 1.000 | 0.882 | 4 | 0.2924 | 0.0032 | 1.902 | 48 | 7 |
| topological | box | ok | yes | 0.953 | 0.809 | 1 | 0.0172 | 0.0202 | 1.159 | 0 | 0 |
| rounded 0.02 | triangle | LIFTS OFF | yes | 0.719 | 0.197 | 3 | 0.6316 | 0.0040 | 3.467 | 44 | 1 |
| rounded 0.02 | pentagon | bumpy | yes | 0.956 | 0.669 | 4 | 0.3745 | 0.0151 | 2.431 | 55 | 3 |
| rounded 0.02 | hexagon | bumpy | yes | 0.930 | 0.615 | 3 | 0.3848 | 0.0054 | 2.504 | 65 | 2 |
| rounded 0.02 | box | LIFTS OFF | yes | 0.771 | 0.370 | 6 | 0.2498 | 0.0053 | 1.520 | 74 | 5 |
| rounded 0.05 | triangle | LIFTS OFF | yes | 0.872 | 0.687 | 2 | 0.1060 | 0.0020 | 1.623 | 12 | 1 |
| rounded 0.05 | pentagon | bumpy | yes | 0.951 | 0.658 | 4 | 0.3664 | 0.0125 | 2.542 | 56 | 3 |
| rounded 0.05 | hexagon | bumpy | yes | 0.959 | 0.724 | 5 | 0.2372 | 0.0057 | 1.894 | 63 | 7 |
| rounded 0.05 | box | ok | yes | 0.954 | 0.819 | 1 | 0.0182 | 0.0071 | 1.230 | 0 | 0 |
| rounded 0.08 | triangle | LIFTS OFF | yes | 0.622 | 0.184 | 6 | 0.1691 | 0.0051 | 0.816 | 44 | 9 |
| rounded 0.08 | pentagon | LIFTS OFF | yes | 0.958 | 0.641 | 4 | 0.5331 | 0.0055 | 2.610 | 85 | 6 |
| rounded 0.08 | hexagon | LIFTS OFF | yes | 0.957 | 0.723 | 7 | 0.1817 | 0.0023 | 1.791 | 75 | 8 |
| rounded 0.08 | box | LIFTS OFF | yes | 0.825 | 0.362 | 5 | 0.2176 | 0.0144 | 1.523 | 46 | 4 |
| rounded 0.10 | triangle | LIFTS OFF | yes | 0.825 | 0.413 | 4 | 0.1482 | 0.0044 | 2.075 | 49 | 4 |
| rounded 0.10 | pentagon | ok | yes | 0.966 | 0.719 | 4 | 0.2977 | 0.0022 | 2.160 | 55 | 4 |
| rounded 0.10 | hexagon | LIFTS OFF | yes | 0.986 | 0.779 | 2 | 0.1237 | 0.0036 | 1.515 | 75 | 9 |
| rounded 0.10 | box | LIFTS OFF | yes | 0.718 | 0.335 | 8 | 0.2050 | 0.0148 | 1.621 | 81 | 8 |
| rounded 0.15 | triangle | LIFTS OFF | yes | 0.536 | 0.055 | 10 | 0.2781 | 0.0051 | 2.292 | 51 | 3 |
| rounded 0.15 | pentagon | ok | yes | 0.936 | 0.672 | 4 | 0.2720 | 0.0053 | 1.817 | 62 | 6 |
| rounded 0.15 | hexagon | LIFTS OFF | yes | 0.970 | 0.751 | 3 | 0.2353 | 0.0060 | 1.916 | 75 | 9 |
| rounded 0.15 | box | LIFTS OFF | yes | 0.958 | 0.620 | 4 | 0.2732 | 0.0059 | 1.900 | 80 | 7 |
| rounded 0.25 | triangle | LIFTS OFF | yes | 1.018 | 0.852 | 3 | 0.1644 | 0.0061 | 1.627 | 62 | 9 |
| rounded 0.25 | pentagon | ok | yes | 0.990 | 0.785 | 4 | 0.0813 | 0.0055 | 1.018 | 60 | 11 |
| rounded 0.25 | hexagon | ok | yes | 1.009 | 0.888 | 3 | 0.0621 | 0.0061 | 0.801 | 50 | 8 |
| rounded 0.25 | box | LIFTS OFF | yes | 0.960 | 0.794 | 5 | 0.3202 | 0.0054 | 2.203 | 75 | 6 |

#### Parabola Hills / parade, smooth chain / speed 3

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | triangle | ok | yes | 0.960 | 0.845 | 1 | 0.0117 | 0.0206 | 0.598 | 0 | 0 |
| none | pentagon | ok | yes | 0.958 | 0.668 | 4 | 0.2619 | 0.0168 | 1.877 | 48 | 4 |
| none | hexagon | ok | yes | 1.002 | 0.856 | 3 | 0.0932 | 0.0055 | 1.656 | 46 | 6 |
| none | box | ok | yes | 0.951 | 0.810 | 2 | 0.0151 | 0.0257 | 0.967 | 0 | 0 |
| semrau | triangle | ok | yes | 0.960 | 0.845 | 1 | 0.0117 | 0.0206 | 0.598 | 0 | 0 |
| semrau | pentagon | ok | yes | 0.958 | 0.668 | 4 | 0.2619 | 0.0168 | 1.877 | 48 | 4 |
| semrau | hexagon | bumpy | yes | 0.992 | 0.761 | 6 | 0.2305 | 0.0190 | 2.214 | 46 | 7 |
| semrau | box | ok | yes | 0.951 | 0.810 | 2 | 0.0151 | 0.0257 | 0.967 | 0 | 0 |
| topological | triangle | ok | yes | 0.960 | 0.845 | 1 | 0.0117 | 0.0206 | 0.598 | 0 | 0 |
| topological | pentagon | ok | yes | 0.958 | 0.668 | 4 | 0.2619 | 0.0168 | 1.877 | 48 | 4 |
| topological | hexagon | ok | yes | 0.996 | 0.823 | 3 | 0.1171 | 0.0194 | 1.231 | 51 | 6 |
| topological | box | ok | yes | 0.951 | 0.809 | 2 | 0.0161 | 0.0257 | 0.967 | 0 | 0 |
| rounded 0.02 | triangle | ok | yes | 0.959 | 0.838 | 1 | 0.0103 | 0.0174 | 0.560 | 0 | 0 |
| rounded 0.02 | pentagon | ok | yes | 0.965 | 0.767 | 4 | 0.2349 | 0.0151 | 1.494 | 44 | 6 |
| rounded 0.02 | hexagon | bumpy | yes | 0.983 | 0.726 | 4 | 0.2925 | 0.0096 | 2.336 | 48 | 5 |
| rounded 0.02 | box | ok | yes | 0.955 | 0.812 | 1 | 0.0191 | 0.0215 | 1.066 | 0 | 0 |
| rounded 0.05 | triangle | ok | yes | 0.961 | 0.843 | 1 | 0.0094 | 0.0247 | 0.611 | 0 | 0 |
| rounded 0.05 | pentagon | ok | yes | 0.960 | 0.726 | 4 | 0.1850 | 0.0282 | 1.278 | 45 | 4 |
| rounded 0.05 | hexagon | bumpy | yes | 0.995 | 0.783 | 3 | 0.2672 | 0.0098 | 1.736 | 49 | 5 |
| rounded 0.05 | box | ok | yes | 0.951 | 0.802 | 3 | 0.0182 | 0.0187 | 1.231 | 0 | 0 |
| rounded 0.08 | triangle | ok | yes | 0.959 | 0.840 | 1 | 0.0091 | 0.0195 | 0.591 | 0 | 0 |
| rounded 0.08 | pentagon | ok | yes | 0.994 | 0.830 | 1 | 0.1121 | 0.0174 | 0.966 | 48 | 6 |
| rounded 0.08 | hexagon | ok | yes | 0.975 | 0.763 | 5 | 0.1400 | 0.0153 | 1.433 | 55 | 6 |
| rounded 0.08 | box | LIFTS OFF | yes | 0.960 | 0.809 | 5 | 0.0141 | 0.0236 | 1.211 | 1 | 1 |
| rounded 0.10 | triangle | LIFTS OFF | yes | 0.954 | 0.828 | 1 | 0.0108 | 0.0169 | 0.550 | 1 | 1 |
| rounded 0.10 | pentagon | ok | yes | 0.984 | 0.761 | 3 | 0.2652 | 0.0268 | 2.049 | 50 | 3 |
| rounded 0.10 | hexagon | ok | yes | 1.002 | 0.849 | 3 | 0.1263 | 0.0209 | 1.477 | 47 | 7 |
| rounded 0.10 | box | LIFTS OFF | yes | 0.963 | 0.788 | 4 | 0.0271 | 0.0214 | 1.227 | 4 | 2 |
| rounded 0.15 | triangle | ok | yes | 0.958 | 0.837 | 2 | 0.0089 | 0.0223 | 0.354 | 0 | 0 |
| rounded 0.15 | pentagon | ok | yes | 0.971 | 0.722 | 4 | 0.2343 | 0.0192 | 1.930 | 55 | 5 |
| rounded 0.15 | hexagon | ok | yes | 0.979 | 0.781 | 5 | 0.1632 | 0.0218 | 1.673 | 53 | 5 |
| rounded 0.15 | box | LIFTS OFF | yes | 0.971 | 0.773 | 5 | 0.1697 | 0.0172 | 1.444 | 38 | 5 |
| rounded 0.25 | triangle | LIFTS OFF | yes | 1.009 | 0.886 | 3 | 0.1521 | 0.0199 | 1.258 | 45 | 5 |
| rounded 0.25 | pentagon | ok | yes | 1.023 | 0.871 | 3 | 0.1206 | 0.0162 | 1.490 | 44 | 11 |
| rounded 0.25 | hexagon | ok | yes | 1.013 | 0.881 | 2 | 0.1170 | 0.0024 | 1.415 | 39 | 13 |
| rounded 0.25 | box | LIFTS OFF | yes | 1.000 | 0.863 | 1 | 0.0987 | 0.0128 | 1.158 | 46 | 8 |

### Correctness: real walls, pits, CCD

#### Tile Wall / box tiles + wall box / speed 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.117 | 0.000 | 1 | 0.0000 | 0.0000 | 0.098 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.015 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.2516 | 0.0008 | 2.106 | 0 | 0 |
| rounded 0.05 | - | PASSED THROUGH | yes | 1.557 | 0.892 | 1 | 1.8475 | 0.0000 | 5.950 | 0 | 0 |
| rounded 0.08 | - | PASSED THROUGH | yes | 1.582 | 1.100 | 1 | 3.1059 | 0.0000 | 9.663 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.8286 | 0.0476 | 3.938 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0630 | 0.0000 | 0.994 | 0 | 0 |

#### Tile Wall / box tiles + tiled wall / speed 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.117 | 0.000 | 1 | 0.0000 | 0.0000 | 0.098 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.015 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.2516 | 0.0008 | 2.106 | 0 | 0 |
| rounded 0.05 | - | PASSED THROUGH | yes | 1.557 | 0.892 | 1 | 1.8475 | 0.0000 | 5.950 | 0 | 0 |
| rounded 0.08 | - | PASSED THROUGH | yes | 1.582 | 1.100 | 1 | 3.1059 | 0.0000 | 9.663 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.383 | -0.099 | 3 | 0.8286 | 0.0000 | 3.938 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0630 | 0.0000 | 0.994 | 0 | 0 |

#### Tile Wall / chain into wall / speed 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.015 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.015 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.05 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.08 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |

#### Tile Wall / thin 0.1 m wall / speed 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.117 | 0.000 | 1 | 0.0000 | 0.0000 | 0.098 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.015 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.2516 | 0.0008 | 2.106 | 0 | 0 |
| rounded 0.05 | - | PASSED THROUGH | yes | 1.557 | 0.892 | 1 | 1.8475 | 0.0000 | 5.950 | 0 | 0 |
| rounded 0.08 | - | PASSED THROUGH | yes | 1.582 | 1.100 | 1 | 3.1059 | 0.0000 | 9.663 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.8286 | 0.0476 | 3.938 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0630 | 0.0000 | 0.994 | 0 | 0 |

#### Tile Wall / segment tiles + segment wall / speed 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.117 | 0.000 | 1 | 0.0000 | 0.0000 | 0.098 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.015 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.2516 | 0.0008 | 2.106 | 0 | 0 |
| rounded 0.05 | - | PASSED THROUGH | yes | 1.557 | 0.892 | 1 | 1.8475 | 0.0000 | 5.950 | 0 | 0 |
| rounded 0.08 | - | PASSED THROUGH | yes | 1.582 | 1.100 | 1 | 3.1059 | 0.0000 | 9.663 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0000 | 0.0000 | 0.011 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.383 | -0.114 | 2 | 0.8286 | 0.0130 | 3.938 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.383 | 0.000 | 1 | 0.0630 | 0.0000 | 0.994 | 0 | 0 |

#### Tile Wall / box tiles + wall box / speed 40

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.013 | 0.000 | 1 | 0.0004 | 0.0000 | 0.130 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.096 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.096 | 0.000 | 1 | 0.0006 | 0.0000 | 0.143 | 0 | 0 |
| rounded 0.02 | - | PASSED THROUGH | yes | 0.438 | 0.000 | 1 | 2.5186 | 0.0000 | 6.976 | 0 | 0 |
| rounded 0.05 | - | PASSED THROUGH | yes | 0.880 | 0.742 | 1 | 4.9524 | 0.0000 | 19.797 | 0 | 0 |
| rounded 0.08 | - | ok (blocked) | no | 0.096 | 0.000 | 1 | 0.0006 | 0.0000 | 0.141 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.096 | 0.000 | 1 | 0.0006 | 0.0000 | 0.141 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.096 | 0.000 | 1 | 0.0006 | 0.0000 | 0.141 | 0 | 0 |
| rounded 0.25 | - | PASSED THROUGH | yes | 1.047 | 1.008 | 0 | 1.8429 | 0.0000 | 11.815 | 0 | 0 |

#### Cliff Pit / box tiles / speed 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.038 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.338 | -0.000 | 1 | 0.0000 | 3.0844 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.338 | -0.000 | 1 | 0.0000 | 3.0844 | 0.000 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.338 | -0.000 | 2 | 1.2749 | 2.0188 | 4.917 | 0 | 0 |
| rounded 0.05 | - | ok (blocked) | no | 0.262 | -0.172 | 1 | 0.0000 | 3.0000 | 0.726 | 0 | 0 |
| rounded 0.08 | - | ok (blocked) | no | 0.304 | -0.073 | 1 | 0.0000 | 3.0131 | 0.267 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.281 | -0.115 | 1 | 0.0000 | 3.0180 | 0.437 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.301 | -0.071 | 1 | 0.0000 | 3.0232 | 0.248 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.267 | -0.127 | 2 | 0.4771 | 3.0215 | 2.988 | 0 | 0 |

#### Cliff Pit / tall box, box tiles / speed 10

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.038 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.338 | -0.000 | 1 | 0.0000 | 3.0844 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.338 | -0.000 | 1 | 0.0000 | 3.0844 | 0.000 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.338 | -0.000 | 2 | 1.2749 | 2.0188 | 4.917 | 0 | 0 |
| rounded 0.05 | - | ok (blocked) | no | 0.262 | -0.172 | 1 | 0.0000 | 3.0000 | 0.726 | 0 | 0 |
| rounded 0.08 | - | ok (blocked) | no | 0.304 | -0.073 | 1 | 0.0000 | 3.0131 | 0.267 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.281 | -0.115 | 1 | 0.0000 | 3.0180 | 0.437 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.301 | -0.071 | 1 | 0.0000 | 3.0232 | 0.248 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.267 | -0.127 | 2 | 0.4771 | 3.0215 | 2.988 | 0 | 0 |

#### CCD Back Face / thin box wall, box tiles / end offset 0

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.009 | 0.000 | 1 | 0.0000 | 0.0000 | 0.055 | 0 | 0 |
| semrau | - | PASSED THROUGH | yes | 1.045 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.072 | 0.000 | 1 | 0.0006 | 0.0000 | 0.138 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.1562 | 0.0000 | 0.137 | 0 | 0 |
| rounded 0.05 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.1531 | 0.0000 | 0.137 | 0 | 0 |
| rounded 0.08 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0005 | 0.0000 | 0.137 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0005 | 0.0000 | 0.137 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0005 | 0.0000 | 0.137 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0005 | 0.0000 | 0.137 | 0 | 0 |

#### CCD Back Face / thin box wall, box tiles / end offset 0.3

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.009 | 0.000 | 1 | 0.0000 | 0.0000 | 0.092 | 0 | 0 |
| semrau | - | ok (blocked) | no | 0.072 | 0.000 | 1 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.072 | 0.000 | 1 | 0.0000 | 0.0000 | 0.009 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0000 | 0.0000 | 0.008 | 0 | 0 |
| rounded 0.05 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0000 | 0.0000 | 0.008 | 0 | 0 |
| rounded 0.08 | - | ok (blocked) | no | 0.072 | 0.000 | 1 | 8.5964 | 0.0000 | 12.968 | 0 | 0 |
| rounded 0.10 | - | PASSED THROUGH | yes | 0.987 | 0.899 | 1 | 3.1223 | 0.0000 | 23.984 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0000 | 0.0000 | 0.008 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0000 | 0.0000 | 0.008 | 0 | 0 |

#### CCD Back Face / segment wall, box tiles / end offset 0

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok (blocked) | no | 0.009 | 0.000 | 1 | 0.0000 | 0.0000 | 0.042 | 0 | 0 |
| semrau | - | PASSED THROUGH | yes | 1.037 | 1.000 | 0 | 0.0000 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok (blocked) | no | 0.072 | 0.000 | 1 | 0.0004 | 0.0000 | 0.126 | 0 | 0 |
| rounded 0.02 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.1560 | 0.0000 | 0.124 | 0 | 0 |
| rounded 0.05 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.1530 | 0.0000 | 0.124 | 0 | 0 |
| rounded 0.08 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0003 | 0.0000 | 0.124 | 0 | 0 |
| rounded 0.10 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0003 | 0.0000 | 0.124 | 0 | 0 |
| rounded 0.15 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0003 | 0.0000 | 0.124 | 0 | 0 |
| rounded 0.25 | - | ok (blocked) | no | 0.072 | -0.000 | 1 | 0.0003 | 0.0000 | 0.124 | 0 | 0 |

#### Interior Push-out / box / depth 0.15

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | FELL | no | 0.080 | 0.000 | 1 | 0.0000 | 80.4832 | 0.000 | 0 | 0 |
| semrau | - | FELL | no | 0.080 | 0.000 | 1 | 0.0000 | 80.4832 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0000 | 0.3486 | 0.000 | 0 | 0 |
| rounded 0.02 | - | FELL | no | 0.080 | 0.000 | 1 | 0.0000 | 80.4832 | 0.000 | 0 | 0 |
| rounded 0.05 | - | FELL | no | 0.080 | 0.000 | 1 | 0.0000 | 80.4832 | 0.000 | 0 | 0 |
| rounded 0.08 | - | FELL | no | 0.080 | 0.000 | 1 | 0.0000 | 80.4832 | 0.000 | 0 | 0 |
| rounded 0.10 | - | FELL | no | 0.080 | 0.000 | 1 | 0.0000 | 80.4832 | 0.000 | 0 | 0 |
| rounded 0.15 | - | FELL | no | 0.080 | 0.000 | 1 | 0.0000 | 80.4832 | 0.000 | 0 | 0 |
| rounded 0.25 | - | FELL | no | 0.077 | 0.000 | 1 | 0.0000 | 80.4832 | 0.000 | 0 | 0 |

### Real lips: Misaligned Tiles with step heights of 15 to 80 mm

#### Misaligned Tiles / box tiles / step height 0.015

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0148 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0149 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0148 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.711 | 0.528 | 2 | 0.9983 | 0.0316 | 4.330 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.225 | -0.037 | 3 | 1.1125 | 0.0241 | 4.586 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0149 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0149 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0149 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.777 | 0.672 | 2 | 0.5357 | 0.0000 | 3.131 | 0 | 0 |

#### Misaligned Tiles / box tiles / step height 0.02

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0197 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0197 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0197 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0197 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.174 | 0.039 | 3 | 0.9785 | 0.0000 | 4.289 | 0 | 0 |
| rounded 0.08 | - | STUCK | no | 0.231 | -0.023 | 4 | 1.1584 | 0.0069 | 4.693 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0198 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0198 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.650 | 0.349 | 4 | 0.5541 | 0.0188 | 3.184 | 0 | 0 |

#### Misaligned Tiles / box tiles / step height 0.03

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0296 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0296 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0296 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0296 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.146 | -0.013 | 2 | 0.5935 | 0.0000 | 3.304 | 0 | 0 |
| rounded 0.08 | - | STUCK | no | 0.215 | 0.101 | 2 | 1.0137 | 0.0000 | 4.374 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.590 | 0.281 | 5 | 0.6957 | 0.0000 | 3.588 | 0 | 0 |

#### Misaligned Tiles / box tiles / step height 0.05

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0494 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0494 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0494 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0494 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0494 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.08 | - | STUCK | no | 0.146 | -0.018 | 2 | 0.5380 | 0.0000 | 3.142 | 0 | 0 |
| rounded 0.10 | - | STUCK | no | 0.131 | -0.040 | 2 | 0.7992 | 0.0000 | 3.867 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0496 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.575 | 0.184 | 5 | 0.6805 | 0.0194 | 3.526 | 0 | 0 |

#### Misaligned Tiles / box tiles / step height 0.08

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0791 | 0.0126 | 0.000 | 0 | 0 |
| semrau | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0791 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0791 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0791 | 0.0126 | 0.000 | 0 | 0 |
| rounded 0.05 | - | STUCK | no | 0.070 | -0.000 | 1 | 0.0791 | 0.0126 | 0.000 | 0 | 0 |
| rounded 0.08 | - | STUCK | no | 0.128 | 0.062 | 1 | 0.2550 | 0.0000 | 1.982 | 0 | 0 |
| rounded 0.10 | - | STUCK | no | 0.148 | -0.005 | 2 | 0.4841 | 0.0000 | 2.894 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0795 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | STUCK | no | 0.492 | 0.113 | 4 | 0.8388 | 0.0000 | 3.920 | 0 | 0 |

#### Misaligned Tiles / loose chain segments / step height 0.03

| mitigation | player | verdict | done | time | minV | snags | hop | sink | kick | air | lifts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| none | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| semrau | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| topological | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.02 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.05 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.08 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.10 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.15 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0298 | 0.0000 | 0.000 | 0 | 0 |
| rounded 0.25 | - | ok | yes | 1.000 | 1.000 | 0 | 0.0296 | 0.0000 | 0.000 | 0 | 0 |


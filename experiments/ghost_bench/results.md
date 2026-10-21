# ghost_bench results

Commit: branch `ghostbusters`, parent acda980 (topology code unchanged from it). Methodology in
[README.md](README.md); the raw numbers, with means, p10 and travel / height sanity columns, are in
[results.csv](results.csv).

## Machine and command

- 11th Gen Intel Core i5-1135G7 laptop (4 cores / 8 threads, 2.4 GHz base, 4.2 GHz turbo, 8 MB L3),
  Linux 6.8.0, governor `powersave` / EPP `balance_performance`, turbo on. Frequency scaling was not
  disabled and the machine was not idle (load average about 2 from other work), so expect a few
  percent of noise. A second invocation of benchmarks 1, 2, 4 and 7 reproduced the medians within
  about 10 % (PreStep within 3 % except the sub-µs rows), with bit-identical checksums.
- gcc 13.3.0, `-O3 -DNDEBUG -ffp-contract=off`, Box2D 3.2.0 fork, single precision.

```sh
cmake -S . -B build/rel -DCMAKE_BUILD_TYPE=Release -DBOX2D_SAMPLES=OFF
cmake --build build/rel --target ghost_bench
taskset -c 2 build/rel/bin/ghost_bench --csv experiments/ghost_bench/results.csv
```

## Summary

- **Simple case** (a body on 2 to 4 flush boxes, K about 1): PreStep 0.5 to 0.7 µs, total overhead
  about 1 µs per step (+30 to 50 % of a 2.3 µs single-body world step).
- **Overhead grows with the neighborhood size K**, linearly at first: PreStep is about 0.5 to 0.8 µs
  per neighbor up to K of about 40 (1.1 µs at K = 2, 3.2 at 5.6, 5.9 at 10, 15 at 19, 32 at 38) and
  becomes superlinear beyond (90 µs at K = 74, 1.2 µs per neighbor). The pre-solve part (Step topo
  minus Step chain) adds 0.3 to 6 µs.
- **Speed** only matters through K: the query reach grows with |v| (K 4.6 at 2.5 m/s to 14.8 at 80 m/s
  on 0.5 m tiles), PreStep 2.5 to 8.4 µs.
- **Protected-body count**: strictly linear. About 2.6 µs PreStep and 3.2 µs total per protected body
  on 0.5 m tiles (207 µs total for 64 bodies), plus 384 KB per body (24.6 MB for 64). The N = 0 row
  (callback registered, nothing protected) is 0.0 µs: no cost when unused.
- **World size does not matter**: one protected body next to 0 to 4005 awake boxes stays at 6 to 14 µs
  overhead (1 to 4 % of the step at 500+ bodies). PreStep creeps from 2.7 to 7.9 µs at 4005 bodies,
  consistent with a deeper broad-phase tree and colder caches after the large step, not with more work.
- **Worst case: overlapping debris.** Small static polygons embedded in the floor (all pairwise
  overlapping, unlike flush tiles) make the per-step topology superlinear. PreStep for the two
  protected bodies together: 55 µs at K = 13, 478 µs at K = 45, 2.3 ms at K = 110 and 4.2 ms at the cap
  (K = 128), i.e. about 0.24 ms per body at K = 45 against 0.03 ms for tiles at K = 38. Profiling
  (callgrind, `-fno-inline`) puts this in `ComputeTopology`: ghost search (`FindGhost`) about 43 %,
  covering (`IsCovered`) 27 %, interior cancellation 27 %; with dense overlap every owner is adjacent to
  most others, so each piece scans most pieces of the neighborhood (roughly quadratic in local edge
  count). Tiles at a similar K are 5 to 10 times cheaper because each tile is adjacent to only two
  others.
- **Neighbor cap.** When the snapshot hits 128 neighbors the query stops early and the topology is
  incomplete. At 1/64 m tiles (K would be about 140) the protected box gets launched (max y 1.99 vs
  0.50 resting) and spends most steps airborne (0.3 pre-solve calls per step), i.e. the mitigation fails,
  while still paying 254 µs of PreStep. Dense debris at the cap kept working in this run.
- **Shape type** hardly matters at equal K: 2.5 to 3.1 µs PreStep for circle, capsule, box and
  hexagon; the total differs mainly through the number of contacts (box 3.1 calls per step, circle 1.7).
- **Determinism**: every configuration and mode produced identical final-state checksums across all 7
  repeats (and across separate invocations).

Note on "overhead vs off": with the mitigation off the driven box is regularly launched by ghost
collisions (max y up to 37 m at 80 m/s) and its step is cheaper because it is airborne, so the
percentage against off overstates the cost of a correct simulation. The chain floor column is the
fair reference where it exists.

## Raw output

# Topological ghosts: cost benchmarks

- Box2D 3.2.0, build type Release, flags ` -O3 -DNDEBUG`
- compiler: gcc 13.3.0
- CPU: 11th Gen Intel(R) Core(TM) i5-1135G7 @ 2.40GHz, allowed CPUs: 2
- clock: clock_gettime(CLOCK_MONOTONIC)
- world: workerCount 1, no task system, dt 1/60 s, 4 substeps
- per configuration: 60 warm-up + 600 measured steps, 7 repeats, modes interleaved with rotating order
- sizeof(TopoGhostContext) = 393160 bytes (383.9 KB) per protected body

Columns: K, E, P = average neighbors snapshotted, step-scene edges and pieces per protected body and step; calls = pre-solve calls per protected body and step; PreStep = TopoGhost_PreStep for all protected bodies; Step = b2World_Step (in topo mode it includes the pre-solve callbacks). All times are µs per step, medians over the pooled measured steps of all repeats. overhead vs off = median(PreStep + Step, topo) - median(Step, off): what turning the mitigation on costs a game. overhead vs chain = the same against the chain floor with no mitigation (Box2D's own ghost vertices): the cost against the ideal, ghost-free result. Without mitigation the protected body is often launched by ghost collisions and flies with no contacts, which makes the off step cheaper; max y off/topo shows it (resting height 0.5 for the 1 x 1 box and circle). p10/p90 are the per-step spread. cap steps = body-steps whose snapshot hit TOPO_MAX_NEIGHBORS (128); overflow = TopoGhostStats.overflow events; mem = N x sizeof(TopoGhostContext); det = final-state checksums identical across repeats in every mode.

## 1. Baseline simple case

One protected body driven at 1.5 m/s over 2 or 4 flush static boxes spanning 20 m.

| config | K | E | P | calls | PreStep | Step off | Step topo | Step chain | overhead vs off | overhead vs chain | off p10/p90 | topo total p10/p90 | cap steps | overflow | mem KB | max y off/topo | det |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| circle, 2 boxes | 1.1 | 4.5 | 4.2 | 1.02 | 0.5 | 2.3 | 2.5 | 2.4 | +0.7 (+31%) | +0.7 (+28%) | 2/3 | 3/4 | 0 | 0 | 384 | 0.50/0.50 | ok |
| circle, 4 boxes | 1.3 | 5.4 | 4.7 | 1.06 | 0.7 | 2.4 | 2.5 | 2.4 | +1.1 (+48%) | +1.1 (+48%) | 2/4 | 3/5 | 0 | 0 | 384 | 0.50/0.50 | ok |
| box, 2 boxes | 1.1 | 4.5 | 4.2 | 1.07 | 0.5 | 2.3 | 2.7 | 2.3 | +0.9 (+38%) | +0.9 (+39%) | 2/4 | 3/5 | 0 | 0 | 384 | 0.71/0.50 | ok |
| box, 4 boxes | 1.4 | 5.4 | 4.7 | 1.22 | 0.6 | 2.2 | 2.6 | 2.3 | +1.0 (+47%) | +0.9 (+41%) | 2/3 | 3/5 | 0 | 0 | 384 | 0.71/0.50 | ok |

## 2. Neighborhood density sweep

Protected 1 x 1 box driven at 10 m/s over static box tiles of width w. The two narrowest widths go past the 128 neighbor cap on purpose.

| config | K | E | P | calls | PreStep | Step off | Step topo | Step chain | overhead vs off | overhead vs chain | off p10/p90 | topo total p10/p90 | cap steps | overflow | mem KB | max y off/topo | det |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| w = 2 m | 2.1 | 8.6 | 6.3 | 1.52 | 1.1 | 1.5 | 3.4 | 3.1 | +3.0 (+199%) | +1.4 (+45%) | 1/3 | 4/5 | 0 | 0 | 384 | 1.76/0.50 | ok |
| w = 1 m | 3.3 | 13.1 | 8.6 | 2.04 | 1.6 | 1.5 | 3.6 | 3.4 | +3.9 (+260%) | +2.0 (+60%) | 1/2 | 5/6 | 0 | 0 | 384 | 2.07/0.50 | ok |
| w = 0.5 m | 5.6 | 22.3 | 13.1 | 3.08 | 3.2 | 1.5 | 4.9 | 4.4 | +6.5 (+429%) | +3.6 (+81%) | 1/3 | 7/9 | 0 | 0 | 384 | 1.98/0.50 | ok |
| w = 0.25 m | 10.1 | 40.5 | 22.3 | 5.17 | 5.9 | 1.5 | 7.1 | 6.3 | +11.6 (+755%) | +6.9 (+110%) | 1/3 | 13/23 | 0 | 0 | 384 | 1.71/0.50 | ok |
| w = 0.125 m | 19.3 | 77.1 | 40.5 | 9.31 | 14.8 | 1.5 | 11.8 | 10.0 | +25.2 (+1626%) | +16.7 (+167%) | 1/5 | 24/41 | 0 | 0 | 384 | 1.93/0.50 | ok |
| w = 0.0625 m | 37.5 | 150.2 | 77.1 | 17.63 | 32.2 | 1.5 | 19.9 | 17.2 | +50.6 (+3324%) | +34.9 (+203%) | 1/6 | 49/77 | 0 | 0 | 384 | 1.76/0.50 | ok |
| w = 0.03125 m | 74.1 | 296.4 | 150.2 | 34.26 | 90.2 | 1.5 | 34.2 | 28.6 | +122.9 (+8075%) | +95.9 (+335%) | 1/19 | 115/147 | 0 | 0 | 384 | 1.61/0.50 | ok |
| w = 0.015625 m | 106.5 | 425.8 | 217.8 | 0.30 | 254.5 | 1.5 | 1.6 | 47.4 | +262.1 (+17553%) | +216.2 (+457%) | 1/30 | 2/298 | 3493 | 3493 | 384 | 1.60/1.99 | ok |

## 3. Speed sweep

Protected 1 x 1 box over 0.5 m tiles. The query reach is queryMargin + 2 |v| dt.

| config | K | E | P | calls | PreStep | Step off | Step topo | Step chain | overhead vs off | overhead vs chain | off p10/p90 | topo total p10/p90 | cap steps | overflow | mem KB | max y off/topo | det |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| v = 2.5 m/s | 4.6 | 18.3 | 11.1 | 3.08 | 2.5 | 2.8 | 4.6 | 3.9 | +4.1 (+146%) | +3.0 (+78%) | 2/4 | 7/8 | 0 | 0 | 384 | 0.71/0.50 | ok |
| v = 5 m/s | 4.9 | 19.6 | 11.8 | 3.08 | 2.7 | 2.0 | 4.7 | 4.2 | +5.4 (+275%) | +3.2 (+76%) | 1/3 | 7/10 | 0 | 0 | 384 | 0.97/0.50 | ok |
| v = 10 m/s | 5.6 | 22.3 | 13.1 | 3.08 | 3.3 | 1.5 | 4.9 | 4.3 | +6.6 (+445%) | +3.8 (+89%) | 1/3 | 7/9 | 0 | 0 | 384 | 1.98/0.50 | ok |
| v = 20 m/s | 6.8 | 27.3 | 15.7 | 3.08 | 3.7 | 1.5 | 6.2 | 4.5 | +8.3 (+559%) | +5.4 (+120%) | 1/2 | 9/11 | 0 | 0 | 384 | 4.44/0.50 | ok |
| v = 40 m/s | 9.5 | 38.0 | 21.0 | 3.08 | 5.2 | 1.5 | 6.8 | 4.7 | +10.5 (+700%) | +7.4 (+158%) | 1/2 | 11/14 | 0 | 0 | 384 | 13.13/0.50 | ok |
| v = 80 m/s | 14.8 | 59.4 | 31.7 | 3.07 | 8.4 | 1.4 | 8.0 | 4.8 | +15.0 (+1044%) | +11.6 (+239%) | 1/2 | 16/18 | 0 | 0 | 384 | 37.37/0.50 | ok |

## 4. Protected-body count sweep

N protected bodies (alternating circle r = 0.5 and 1 x 1 box) 3 m apart, driven at 5 m/s over 0.5 m tiles. K, E, P and calls are per protected body; overhead is the total. N = 0 is the noise floor (topo mode registers the callback but has no protected body).

| config | K | E | P | calls | PreStep | Step off | Step topo | Step chain | overhead vs off | overhead vs chain | off p10/p90 | topo total p10/p90 | cap steps | overflow | mem KB | max y off/topo | det |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| N = 0 | 0.0 | 0.0 | 0.0 | 0.00 | 0.0 | 0.3 | 0.2 | 0.2 | -0.0 (-2%) | +0.0 (+5%) | 0/0 | 0/0 | 0 | 0 | 0 | 0.00/0.00 | ok |
| N = 1 | 5.0 | 20.0 | 12.0 | 1.67 | 2.5 | 2.9 | 2.7 | 2.5 | +2.2 (+75%) | +2.7 (+106%) | 2/3 | 5/6 | 0 | 0 | 384 | 0.53/0.50 | ok |
| N = 2 | 5.0 | 19.8 | 11.9 | 2.38 | 5.1 | 3.2 | 5.5 | 4.8 | +7.4 (+228%) | +5.9 (+123%) | 3/4 | 10/11 | 0 | 0 | 768 | 1.00/0.50 | ok |
| N = 4 | 5.0 | 19.8 | 11.9 | 2.38 | 10.5 | 4.6 | 7.8 | 6.6 | +14.0 (+304%) | +12.0 (+183%) | 3/6 | 17/20 | 0 | 0 | 1536 | 1.04/0.50 | ok |
| N = 8 | 5.0 | 19.8 | 11.9 | 2.38 | 20.7 | 6.6 | 11.5 | 9.1 | +25.9 (+395%) | +23.4 (+257%) | 5/8 | 31/35 | 0 | 0 | 3072 | 1.02/0.50 | ok |
| N = 16 | 5.0 | 19.8 | 11.9 | 2.38 | 41.8 | 11.0 | 19.5 | 14.5 | +50.7 (+461%) | +47.3 (+327%) | 8/14 | 59/67 | 0 | 0 | 6143 | 1.04/0.50 | ok |
| N = 32 | 5.0 | 19.8 | 11.9 | 2.38 | 85.7 | 18.8 | 36.4 | 25.8 | +104.3 (+555%) | +97.3 (+377%) | 15/23 | 115/132 | 0 | 0 | 12286 | 1.11/0.50 | ok |
| N = 64 | 5.0 | 19.8 | 11.9 | 2.38 | 170.2 | 34.9 | 70.3 | 49.0 | +207.4 (+594%) | +193.3 (+394%) | 28/41 | 230/267 | 0 | 0 | 24572 | 1.11/0.50 | ok |

## 5. Dense debris

A protected circle (r = 0.5) and a protected 1 x 1 box, 3 m apart, driven at 4 m/s across a 50 m field of small random convex polygons (seeded). Static: gravel embedded in the floor top. Dynamic: a loose 4-row pile they plough through. No chain reference (the surface is not a polyline).

| config | K | E | P | calls | PreStep | Step off | Step topo | Step chain | overhead vs off | overhead vs chain | off p10/p90 | topo total p10/p90 | cap steps | overflow | mem KB | max y off/topo | det |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| static, 250 pieces | 13.5 | 61.2 | 57.1 | 0.67 | 55.3 | 4.2 | 4.8 | — | +55.9 (+1339%) | — | 3/6 | 41/88 | 0 | 0 | 768 | 0.92/0.93 | ok |
| static, 1000 pieces | 45.3 | 213.6 | 209.5 | 0.68 | 477.9 | 7.6 | 8.7 | — | +479.1 (+6271%) | — | 4/11 | 389/598 | 0 | 0 | 768 | 0.89/0.90 | ok |
| static, 2500 pieces | 109.8 | 524.8 | 520.6 | 0.81 | 2303.8 | 13.3 | 17.8 | — | +2309.0 (+17369%) | — | 7/20 | 1963/2627 | 924 | 721 | 768 | 0.88/0.96 | ok |
| static, 5000 pieces | 128.0 | 615.6 | 612.9 | 0.97 | 4177.7 | 20.5 | 39.2 | — | +4223.3 (+20575%) | — | 10/32 | 3869/4708 | 8386 | 8421 | 768 | 0.92/0.89 | ok |
| dynamic, 200 pieces | 13.2 | 61.8 | 57.7 | 1.85 | 3.3 | 5.5 | 6.3 | — | +4.1 (+75%) | — | 4/103 | 8/622 | 0 | 0 | 768 | 1.07/1.14 | ok |
| dynamic, 1000 pieces | 48.0 | 234.8 | 230.0 | 2.69 | 498.5 | 299.2 | 399.1 | — | +600.2 (+201%) | — | 186/384 | 761/1014 | 0 | 0 | 768 | 1.03/1.10 | ok |

## 6. Unprotected-body scaling

One protected 1 x 1 box driven at 5 m/s on 0.5 m tiles, plus a pyramid of B(B+1)/2 dynamic boxes far away. Sleep disabled so the pyramid stays awake.

| config | K | E | P | calls | PreStep | Step off | Step topo | Step chain | overhead vs off | overhead vs chain | off p10/p90 | topo total p10/p90 | cap steps | overflow | mem KB | max y off/topo | det |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 0 bodies | 4.9 | 19.6 | 11.8 | 3.08 | 2.7 | 1.9 | 4.7 | — | +5.5 (+285%) | — | 1/3 | 7/8 | 0 | 0 | 384 | 0.97/0.50 | ok |
| 496 bodies | 4.9 | 19.6 | 11.8 | 3.08 | 2.9 | 171.5 | 174.5 | — | +6.1 (+4%) | — | 167/183 | 169/191 | 0 | 0 | 384 | 0.97/0.50 | ok |
| 990 bodies | 4.9 | 19.6 | 11.8 | 3.08 | 3.4 | 352.3 | 355.3 | — | +6.5 (+2%) | — | 329/384 | 329/392 | 0 | 0 | 384 | 0.97/0.50 | ok |
| 2016 bodies | 4.9 | 19.6 | 11.8 | 3.08 | 3.8 | 709.6 | 719.6 | — | +14.0 (+2%) | — | 675/804 | 683/819 | 0 | 0 | 384 | 0.97/0.50 | ok |
| 4005 bodies | 4.9 | 19.6 | 11.8 | 3.08 | 7.9 | 1673.1 | 1678.6 | — | +13.8 (+1%) | — | 1520/2026 | 1542/2031 | 0 | 0 | 384 | 0.97/0.50 | ok |

## 7. Shape-type sweep

One protected body driven at 5 m/s over 0.5 m tiles.

| config | K | E | P | calls | PreStep | Step off | Step topo | Step chain | overhead vs off | overhead vs chain | off p10/p90 | topo total p10/p90 | cap steps | overflow | mem KB | max y off/topo | det |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| circle r 0.5 | 5.0 | 20.0 | 12.0 | 1.67 | 2.5 | 3.0 | 2.7 | 2.5 | +2.2 (+73%) | +2.7 (+107%) | 2/3 | 5/6 | 0 | 0 | 384 | 0.53/0.50 | ok |
| capsule 0.5 + 2 x 0.25 | 4.9 | 19.6 | 11.8 | 2.42 | 2.6 | 1.9 | 4.0 | 3.6 | +4.6 (+246%) | +3.0 (+83%) | 1/4 | 6/7 | 0 | 0 | 384 | 0.82/0.25 | ok |
| box 1 x 1 | 4.9 | 19.6 | 11.8 | 3.08 | 2.7 | 2.0 | 4.8 | 4.2 | +5.5 (+274%) | +3.3 (+77%) | 1/3 | 7/10 | 0 | 0 | 384 | 0.97/0.50 | ok |
| hexagon r 0.5 | 4.8 | 19.3 | 11.7 | 0.23 | 3.1 | 1.8 | 1.8 | 2.0 | +3.1 (+172%) | +3.0 (+151%) | 1/3 | 4/6 | 0 | 0 | 384 | 0.71/0.68 | ok |

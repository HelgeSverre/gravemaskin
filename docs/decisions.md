# Decision log

Short entries; the reasoning behind bigger decisions lives in SPEC.md.

## 2026-08-28 — Track drive mechanism (Phase 0 spike verdict)

Reflected over the installed BepuPhysics 2.5.0-beta.29 assembly:
`PairMaterialProperties` = { FrictionCoefficient, MaximumRecoveryVelocity,
SpringSettings } and `ConvexContactManifold` carries no per-contact target
velocity. The "conveyor belt surface velocity" trick is therefore NOT
expressible at the contact-material level. Per the SPEC's pre-committed
fallback, tracks will be per-contact tractive impulses capped at μ×N applied
inside `Step` (3–4 contact boxes per side, anisotropic friction), and
`Excavator/Tracks.fs` is written against that from the start.

## 2026-08-28 — .slnx over .sln

.NET 10's `dotnet new sln` emits the XML `.slnx` format; kept it (justfile and
CI reference `Gravemaskin.slnx`).

## 2026-08-28 — Soil facade compiles last, not first

SPEC lists `Soil/Api.fs` above `Soil/Volume.fs`, but F# compile order forbids
a facade before the types it re-exposes. The seam survives as the LAST soil
file (`Soil/Api.fs` defining the `Soil` module); the rule stands: `Sim.fs` and
`Excavator/*` call only the `Soil` facade, never chunk internals.

## 2026-08-29 — Deferred items closed

- **Moisture dynamics**: implemented as a water-table model (Soil/Moisture.fs)
  — moisture is a property of the ground, wicking up and evaporating, driving
  effective cohesion (capillary sandcastle bell, clay saturation weakening).
- **Compound bucket**: open-plate dynamic compound (bottom/back/sides);
  opening faces −X so curl swings it up. Cradles bodies, scoops loose clumps
  into the payload.
- **Renderer GL leak on F9 load**: Renderer implements IDisposable; the load
  path disposes before rebuilding.
- **Surface Nets**: still not used — the render mesh is now a 2×-subdivided,
  bilinearly smoothed heightfield with micro-noise and crevice AO, which
  delivers the smooth-dirt goal Surface Nets was for. The soil model remains
  2.5D columns; Surface Nets only earns its place if overhangs ever exist.

## 2026-08-29 — The grain layer (falling-sand presentation)

Loose-soil mass still rides in the capped BEPU clump bodies (ledger,
determinism, machine interaction) — but clumps are no longer drawn as balls.
Presentation is a falling-sand grain layer (Soil/Grains.fs): up to 45k small
grains that fall, bounce, stack on a half-cell pile field, avalanche past
repose, and re-mobilize when dug out from under; clumps render as grain
clusters; pours/dig-crumble/track-spray emit from sim events. Per the SPEC's
cosmetic contract the layer runs on wall-clock in the shell, carries zero
mass, and stays outside the determinism hash — the Release sim gates are
untouched by it.

## 2026-08-29 — Hard stroke limits (the 'bending backwards' question)

Checked against how real machines and training sims work: a hydraulic
cylinder is an ABSOLUTE mechanical end stop — the piston bottoms on the
cylinder cap (with hydraulic cushioning), so a linkage joint physically
cannot exceed the angle range its cylinder stroke defines; doing so means
bursting steel. AGX/Vortex model stroke ends as hard unilateral constraints,
stiffer than any relief valve. Our previous software velocity-clamp was
game-y and leaked under load. Now every cylinder joint carries a BEPU
TwistLimit (identity bases measure Z-twist with our exact sign convention —
verified by spike); an abuse test drives 40/15/12 kN·m of sustained external
torque into the joints and requires the stops to hold. The swing has no
limit — real houses rotate continuously.

Also: the bucket payload is now VISIBLE — rendered as a grain heap inside
the bucket that grows while digging and drains while pouring — and dig
crumble flows toward the mouth. The mass transport itself remains the
ledgered payload scalar (the training-sim standard); the heap is its honest
presentation. The bucket shell is drawn tilted 45° (mouth down-back at
rest, up at carry) over the axis-aligned collision container.

## 2026-11-11 — Review pass: swept cut, gravity pour, job site

- **Blade cut is a fractional column cut** (`Carve.bowl` → `Volume.cutColumnTo`):
  columns under the edge are lowered to the bowl's underside in occupancy
  units, so removal is proportional to what the lip swept. The old sphere
  carve popped out whole 0.25 m cells. Removed mass is the exact difference
  of the cells' scan masses, so the ledger still balances to the bit.
  `Carve.sphere` stays for the debug brush.
- **Cut geometry derives from the rig**: bowl radius × √scale, lip width ×
  scale, several cut points along a wide lip, fill rate × capacity. The Cat
  320 was digging with the U17's constants.
- **Spill rides the blade** (`World.SpillPending`): un-captured cut soil
  accumulates and leaves as shovelful clumps. It is mass in flight — the
  conservation check counts it next to clumps and the payload.
- **Gravity empties the bucket**: retention is a function of the mouth's
  world tilt (`Machine.Retention`), not the bucket joint angle; teeth below
  the bench hold the load in. `MachineRig.DumpAngle` is gone.
- **FEE samples the bench** (`Soil.sampleBelow`) instead of the cell at the
  edge, which the previous tick had emptied — clay now resists like clay.
  Resistance also rises with bucket fill (surcharge).
- **Tracks**: three contact points per side, drive along the chassis (slope-
  following), travel brakes when the lever is home, `TrackSlip` raised.
- **World boundary**: four static walls; the soil mesh used to just end.
- **Job site** (`Soil/Terrain.fs`): graded yard and road, staked foundation
  pit with a progress readout, loose stockpiles, knoll, gravel-pit pond,
  creek under the water table, sand/gravel lenses in the clay, fractional
  surface heights. Worlds under 56 m get plain terrain. Scenery (spruce,
  boulders, cabin) is seeded by `World.SeedScenery`.
- **Save format GRAV3**: adds machine heading, trees and the site layout;
  GRAV2 still loads.

## 2026-11-12 — Loose soil redone: packets in flight, piles as terrain

Loose soil had three overlapping representations: BEPU sphere bodies
("clumps") that rolled, rested 18 ticks and then banked into one column;
a cosmetic grain layer with its own pile-height field that decayed after
9 s; and a four-neighbour settle pass. The result was dirt that rolled like
marbles, heaps that appeared twice and then faded, and diamond-shaped piles.

Now there is one model:

- **In flight:** `ClumpPool` (Soil/Particles.fs) holds ballistic mass
  packets — position, velocity, f64 mass, material. No rigid bodies, no
  collisions. A packet that reaches the ground lands: its mass is split
  bilinearly over the four nearest columns (`ClumpPool.Land`) and the
  packet is gone. Cap 3000.
- **At rest:** the terrain. `Settle` relaxes every column against all
  eight neighbours with the true distance, so heaps are round cones at
  `Settle.reposeSlope` (tan φ plus a capped cohesion term). Cohesionless
  bank (dry sand, gravel) follows the same rule; cohesive bank still fails
  as a wedge, which now falls to the low side (`WallFailure.LowX/LowZ`).
- **Bucket vs loose soil:** terrain is a static collision mesh, so a berm
  would be a wall. The bucket shell ploughs instead: loose (uncompacted)
  soil overlapping the bucket body is removed (`Soil.ploughLoose`, bank is
  untouched) and thrown ahead of the bucket's motion. The clump-absorbing
  "scoop" is gone; a dumped pile reloads through the ordinary cut.
- **Grains:** airborne dust and crumbs only. A grain that lands sinks into
  the surface within 0.3 s; the layer keeps no pile of its own.

Consequences: packets fall through machine bodies (nothing rests on the
boom or tracks); a sleeping machine is now woken by arm/swing input
(previously live clump bodies kept it awake by accident).

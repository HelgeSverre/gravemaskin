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

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

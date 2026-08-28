# GRAVEMASKIN — Phased Plan / Spec

A 3D excavator physics sandbox in F#. Silk.NET (OpenGL 4.1 core, the macOS ceiling), BEPUphysics v2, .NET 10, macOS-first but cross-platform desktop. Asset-free and procedural, in the house style of [fsharp-of-duty](../fsharp-of-duty) and [bloom](../bloom).

This spec was produced by a 13-agent research + design workflow (2026-08-28): six parallel deep-dives (full write-ups in `docs/research/`), three competing phased plans, a three-judge panel, and a completeness critique whose 20 amendments are folded in below. A verified minimal BEPU 2.5.0-beta.29 F# program (struct callbacks, hinge + servo constraints) lives in `spikes/bepu-fsharp/` and seeds `Physics.fs`.

---

## 1. Vision

Dirt is a physical material, not a heightmap. The excavator is a machine, not an animation.

The three-representation soil pipeline:

```text
UNDISTURBED SOIL   chunked occupancy volume + 2D surface cache
      ↓ bucket fails the soil (Rankine active wedge)
LOOSE SOIL         rigid clump particles (BEPU spheres, hard-capped)
      ↓ comes to rest
SETTLED SOIL       written back into the volume at loose density
      ↓ track passes
COMPACTED SOIL     density rises back toward bank
```

The excavator is a rigid-body linkage driven by force-limited hydraulic motors. Stalling, straining, self-lifting on the bucket, and tipping over are all **emergent** — no special-case code. Mass is conserved end to end, enforced by a ledger asserted every debug tick.

## 2. Design pillars (non-negotiable)

1. **Emergence over scripting.** Tipping = real COM leaving the support polygon. Stall = motor force cap reached. Flow sharing = pump budget arithmetic. If a behavior needs a canned animation, the model is wrong.
2. **Mass conservation is an invariant, not a goal.** Per-material f64 mass ledger, asserted in debug every tick, FsCheck property from Phase 1 onward. Skipping the assert "to go faster" is forbidden — the research is unanimous that mass leaks cost weeks.
3. **60 Hz fixed-step deterministic sim** (same-machine; cross-platform determinism explicitly not promised — ARM ≠ x64 FP).
4. **Zero steady-state allocation on the tick path**, enforced by test.
5. **Asset-free.** Procedural meshes, materials, font, audio — house style.
6. **Risk retirement order.** Phases are ordered by what can kill the project, not by demo appeal.

## 3. Technology decisions

| Decision | Choice | Rationale |
|---|---|---|
| Physics | **BEPUphysics 2.5.0-beta.29** (pinned) | Pure managed (no native packaging ×3 OSes), source-steppable, servo/motor constraints with `MaximumForce` map exactly onto hydraulics. F# struct-generic callbacks verified compiling and running on .NET 10 / macOS arm64 (`spikes/bepu-fsharp/`). Jolt's vehicle controller doesn't cover the arm anyway. |
| Soil model | **Servin/agxTerrain wholesale**: 3D occupancy grid + 2D surface cache + Rankine active zone + ~1000–1500 variable-size clumps + swell-factor resettle | The only published design achieving the exact three-representation vision at 60 Hz (16.7 ms steps, RTF 1.5, within 10–25 % of DEM). No MPM (GPU-only; GL 4.1 has no compute), no DEM. |
| Dig forces | **Holz discretized FEE** (Fundamental Equation of Earthmoving), 8–16 slices across bucket width, applied as external force/torque on the bucket body | Closed-form, cheap, the training-sim standard. Servin aggregate body is the named upgrade if feel demands it — an upgrade path, not a redesign. |
| Cylinders | **Analytic, not rigid bodies**: torque = F_cyl(P×A) × moment_arm(θ), applied via `AngularAxisMotor` caps on plain `Hinge` joints | Avoids closed-loop constraint cycles (house–boom–cylinder triangles) that iterative solvers handle poorly at 400 kN and 20:1 mass ratios. The 4-bar bucket linkage is law-of-cosines math. Fallback to real cylinder bodies (`LinearAxisMotor` + `PointOnLineServo` + `LinearAxisLimit`) is a named pivot, A/B'd on one joint first. |
| Hydraulics | **Quasistatic flow budget** (no fluid): per-function flow-sharing scale-down + relief-pressure force caps + engine corner-power clamp, extend/retract area asymmetry kept | One rule set produces "three functions at once are slower", stall, and boost feel — the documented Vortex / arXiv 2102.11381 pattern. |
| Tracks | **Conveyor pattern**: 3–4 contact boxes per side, tangential surface-velocity, anisotropic friction (μ_long 0.9 / μ_lat 0.6 firm, scaled by moisture/looseness), tractive cap μ×N per box. Never per-shoe. | Pecka et al. + every training sim agree. **Phase 0 must spike the actual mechanism** — `PairMaterialProperties` has no tangential-velocity field; if contact tangent target velocity isn't settable, the pre-committed fallback is per-contact tractive impulses capped at μ×N applied in `Step`, and `Tracks.fs` is written against that from the start. |
| Rendering | **Silk.NET OpenGL 4.1 core**, all GL behind `Renderer`/`GlUtil`/`Shaders` | macOS ceiling (no compute/SSBO/persistent mapping) doesn't hurt: physics, soil, meshing are CPU-side by design. WebGPU/NeoVeldrid is a two-file escape hatch, revisited each macOS major. |
| Meshing | **Naive Surface Nets** per 32³ chunk (chunk size gated by a Phase 1 benchmark, see §10 Phase 1) | No sharp features in dirt → no dual contouring; smoother than marching cubes; 0.1–0.5 ms/chunk Burst reference with 2–4× .NET slack budgeted. |
| World state | **Mutable `World` class** (bloom precedent) | The house invariant is *headless determinism behind `Step(InputFrame)`*, not immutability. BEPU is pool-based and mutable; soil is flat arrays. |
| First machine | **Kubota U17-class mini** (1730 kg) | Solver-friendlier forces; three hard-assigned pump circuits make flow-sharing audible; 15.2 kN breakout vs 17 kN weight makes the machine visibly rock while digging. Cat 320 is a data-file addition later. |
| GC | Workstation GC + `SustainedLowLatency`, zero-alloc tick path | Real-world reports show 20–55 ms pauses no latency mode fixes; the only winning move is not allocating. Revisit concurrent-GC on/off with real p99 data in Phase 4. |

## 4. Repository layout & architecture

Mirrors fsharp-of-duty: `global.json` (SDK 10.0.100, rollForward latestMajor), `Directory.Build.props` (net10.0, LangVersion preview, TreatWarningsAsErrors, Deterministic, `WarnOn 3517` in Core), justfile with `_sdk` shim and `check = lint+build+test`, CI/release workflows near-verbatim (self-contained single-file with **loose Silk.NET natives** — the load-bearing fsproj comment carries over — universal macOS .app, ad-hoc codesign).

### src/Gravemaskin.Core (pure/headless; references BepuPhysics; NO Silk.NET)

Compile order:

```text
Prelude.fs              units of measure (float32<m>, <kg>, <kPa>, <s>), xorshift32 struct Rng (bloom)
Noise.fs                lifted verbatim from Ironsight (value noise + fbm2)
MathEx.fs               Aabb/ray/closest-point lifted from Ironsight
Domain.fs               types ONLY: SoilMaterial, ChunkId, ActuatorCommand, MachineSpec,
                        [<Struct>] InputFrame, GameEvent, RenderState (POD snapshot struct)
Tuning.fs               TickRate = 60 [<Literal>]; soil material table; machine tables (§7);
                        every constant with a rationale comment (house style)
Soil/Api.fs             THE soil seam: surfaceHeight / carve / deposit / resistance /
                        settleTick / dirtyChunks. Sim.fs and Excavator/* touch soil ONLY
                        through this signature — keeps FEE/tracks/bucket-fill decoupled from
                        chunk internals and makes the Phase 1 heightfield-rescope pivot executable.
Soil/Volume.fs          32³ chunks, SoA flat arrays: occupancy/material/moisture/compaction bytes;
                        pooled; cold chunks RLE (storage-only; defer if it fights the phase —
                        flat 256×64×256 SoA is ~34 MB, affordable)
Soil/Surface.fs         2D height cache, dirty columns; all settle/slope/deposition queries here
Soil/Carve.fs           CSG max(f, −f_tool) near the tool; exact ΔV from occupancy deltas;
                        f64 per-material MASS ledger asserted each debug tick
Soil/Particles.fs       clump SoA (pos/vel/radius/material), BEPU spheres, hard cap ~1500,
                        merge-on-overflow + spawn-rate cap; kill-plane despawn returns mass
                        to the ledger; settle-detection → write-back at loose density
Soil/Settle.fs          Margolus block-CA repose settling on the surface cache (order-free ⇒
                        deterministic), budgeted with carried dirty set; cohesion critical
                        height h_crit ≈ 4c/γ → wedge failure → clump burst
Soil/Mesher.fs          Surface Nets; TWO paths (see §9): render meshes async on workers,
                        collision meshes deterministic; 24 B vertex (pos 3×f32, INT_2_10_10_10
                        normal, 4×u8 material weights, moisture/compaction/AO u8); neighbor-chunk
                        border SAMPLING, never duplicated data (Astroneer crack bug)
Excavator/Linkage.fs    analytic: cylinder anchors, current length, moment arm d(θ) per joint;
                        4-bar bucket dog-bone via law-of-cosines. Pure functions.
Excavator/Hydraulics.fs quasistatic flow budget → ActuatorCommand[] {TargetVelocity, MaxForce}
Excavator/Tracks.fs     conveyor contact boxes (or the impulse fallback per Phase 0 spike)
Excavator/Fee.fs        discretized FEE: per-slice depth/slope from surface cache;
                        F = γd²N_γ + cdN_c + QN_Q + c_a·d·N_a; singularity guard
                        (δ+ρ+φ+β ≥ π → tuned depth-scaled resistance); tanh friction-sign
                        smoothing; low-pass on output
Bepu.fs                 byref-ceremony wrappers so gameplay code never writes `let mutable desc … &desc`;
                        callback structs from spikes/bepu-fsharp/Program.fs
Physics.fs              the ONLY module owning BEPU state: BufferPool, Simulation
                        (Deterministic=true, SolveDescription(8,4)), fixed-thread dispatcher,
                        handle tables, per-chunk static Mesh registry (remove/add swap,
                        Shapes.RecursivelyRemoveAndDispose), collision-group bitfield
                        (RagdollDemo pattern) so linked parts don't self-collide
ProcGen/ExcavatorMesh.fs, ProcGen/Materials.fs
Sim.fs (LAST)           mutable World class; member Step(input: InputFrame)
                        : struct (GameEvent[] * RenderState)  — events drained from a pooled
                        buffer, NOT an allocated list (see §9 allocation policy)
```

Tick pipeline inside `Step`: input → `Hydraulics.allocate` → linkage torques → apply to BEPU (change-guarded `ApplyDescription`, TankController pattern, so the machine can sleep) → FEE forces via `ApplyImpulse` → `Simulation.Timestep(1/60)` → carve (bucket swept volume) → particle spawn/settle (budgeted) → CA → snapshot + drain events.

### src/Gravemaskin (exe, namespace Gravemaskin.Shell; Silk.NET only here)

```text
Settings.fs  Input.fs (InputSampler lifted: deadzone/latch/menu-suppression + pattern mapping)
GlUtil.fs (lifted; exposes ONLY the orphaning upload — no glBufferSubData on hot buffers)
Shaders.fs (GLSL 410 strings: triplanar procedural soil + moisture darkening, instanced clods,
            overcast-industrial sky, single 2048–4096 ortho shadow map + 3×3 PCF, std140 UBO)
Font.fs (bloom's 5×7)  Hud.fs  Particles.fs  Audio.fs  Renderer.fs  Program.fs (LAST)
```

`Program.fs` is the Ironsight loop verbatim: GL 4.1 Core/ForwardCompatible context, accumulator clamped 0.25 s, `while acc ≥ 1/60 → Step`, render interpolates a **`RenderState` double buffer** by alpha — *not* world snapshots; a mutable World makes previous/current aliasing a silent bug, so the POD snapshot struct is the interpolation unit.

**Cosmetic particles are shell-owned** (amendment): `RenderState`'s particle slice carries only the ≤1.5k BEPU clumps (interpolated). The ≤100k cosmetic dust/debris layer lives in the shell, stepped on wall-clock against a read-only surface-cache snapshot, carries zero mass, never touches the ledger, and never enters the determinism hash. macOS quirk: point-sprite size is clamped — large near-camera dust falls back to instanced camera-facing quads.

### tests/ and benchmarks/

`tests/Gravemaskin.Tests`: `TestKit.fs` first (flat-ground builders, `stepAll` loops, dig-scenario builders), xUnit + FsCheck, `Category=Integration` filter. `benchmarks/`: BenchmarkDotNet with MemoryDiagnoser; any B/op > 0 in a sim kernel fails review.

## 5. Soil model detail

**Cell mass** (amendment — this definition is load-bearing): occupancy is a volume fraction; mass = occupancy × cellVolume × ρ(material, compaction). The f64 per-material ledger tracks **kg, not volume**. Compaction and settling adjust occupancy inversely to density so the ledger balances; the FsCheck conservation property includes compaction passes. Swell on resettle: loose occupancy written at w = 1/(1+S) of bank (Servin swell factor S per material).

**Material table** (Tuning.fs, Servin values): gravel φ=44° c=0 · dry sand φ=39° c≈0 · wet sand φ=33° c=8.7 kPa · dirt/topsoil φ=40° c=2.1 kPa · clay φ=21° c=4.8 kPa. Repose angle = φ. Cohesive critical wall height h_crit ≈ 4c/γ; over-steep cohesive faces fail as a whole Rankine wedge → clump burst + `WallCollapsed`. Vibration (tracks near edge, bucket strike) lowers effective c.

**Dig trigger**: Rankine active-zone wedge, θ = π/2 − (φ+β)/2, evaluated where the cutting edge actually fails soil — carve + spawn only there; FEE supplies the resistance. Bucket-interior detection converts resting clumps to a **load scalar** (mass + COM shift on the bucket body, re-emitted as clumps on dump) — the training-sim trick that saves most of the particle budget *and* routes payload into COM/tipping/hydraulic load exactly as the vision requires.

**Overhangs**: the 2.5D surface cache is a chosen simplification. Repose CA runs on the topmost surface only; rare undercuts collapse via the cohesion-failure path. Multi-level heightfield spans are a scoped pivot only if play shows routine tunneling — the largest unbudgeted item (see §11).

## 6. Excavator model detail

**Rig** (BEPU): chassis (compound, real COM) —Hinge(Y)+AngularAxisMotor (torque cap = swing's own relief, plus swing brake)→ house —Hinge→ boom —Hinge→ stick —Hinge→ bucket. Per-tick `TargetVelocity` (from flow/area/moment-arm) and `MaximumForce` (pressure × piston-or-annulus area × moment arm — extend/retract asymmetry kept; operators notice boom-down being weaker). Angular limits at stroke ends. SpringSettings frequency ≤ ~half the substepped rate (≈120 Hz at 4 substeps) — tune down, never crank stiffness to fix sag.

**Bucket collision shape** (amendment): a **compound of convex plates** — back, two sides, bottom, cutting-edge boxes — so it can physically hold clumps; bucket-interior detection tests against the same compound. BEPU convexes can't be concave; without this the MVP bucket-fill loop has nothing to rest in.

**Hydraulics**: per function demandᵢ = |inputᵢ|×Q_maxᵢ; if Σdemand > Q_pump scale all (the U17's three hard-assigned circuits are three separate budgets — two functions on one circuit halve each other, functions on different circuits don't). Force cap per direction from relief pressure × area. Engine corner-power clamp Σ(Pᵢ×Qᵢ) ≤ P_engine×0.85.

**Machine data** (frozen in Tuning.fs):

- **U17-class**: mass 1730 kg · pumps 17.3 + 17.3 L/min @ 21.6 MPa + 10.4 L/min @ 18.6 MPa · bucket breakout 15.2 kN · arm crowd 8.5 kN · swing 9.1 rpm · travel 2.25/4.25 km/h · ground pressure 25.5 kPa · dig depth 2.31 m · reach 3.84 m · plus boom-swing and dozer blade circuits.
- **Cat 320-class**: 21 700 kg · 429 L/min @ 35 MPa (38 boost) · boom cyl 2×⌀120×1260 mm, stick ⌀140×1504, bucket ⌀120×1104 · breakout 150 kN · stick 106 kN · swing 11.25 rpm @ 82 kN·m (27.5 MPa own relief) · drawbar 205 kN · component masses: undercarriage 4390 + 2×2690, house ~1910+9500, counterweight 4200, boom 1900, stick 1110, bucket 940.

Validation targets: bucket-tip breakout at the ISO 6015 reference pose must **emerge** from cylinder force × linkage leverage within ±10–15 % of spec (never hardcoded); over-side tipping load ≈ 1.33× rated (ISO 10567).

## 7. Controls spec

Dual-stick, ISO default, SAE toggle in Settings. Shared: left-X = swing; right-X = bucket (left=curl, right=dump). ISO: left-Y = stick (fwd=out), right-Y = boom (fwd=down). SAE: boom/stick swap hands. Tracks: independent per-track axes — gamepad triggers+bumpers as fwd/rev per side; pivot and counter-rotation fall out.

Per-axis pipeline (in Core, deterministic, part of `Step`): 10 % deadband → x^1.7 curve → first-order lag τ = 100 ms → target velocity. Valve = velocity control; feathering falls out.

**Keyboard mapping** (amendment — keyboard-only players get full arm control): `A/D` swing · `W/S` stick · `↑/↓` (or `I/K`) boom · `←/→` (or `J/L`) bucket · `Q/E` left track fwd/rev · `Z/C` right track fwd/rev (final keys tunable in Settings).

**Auxiliary functions** (amendment — gamepad axes are fully allocated before boom-swing and dozer blade arrive in Phase 5): **hold-LB axis-shift** remaps left stick to boom-swing (X) / dozer blade (Y), matching real minis' switchable pedal. Decided now so muscle memory never retrains.

## 8. Performance budget

@60 Hz, p99 ≤ 12 ms on Apple M-series (budget against NEON — `Vector<T>` is 4-wide there, not AVX2 blog numbers):

| System | ms |
|---|---|
| input + controls | 0.1 |
| hydraulics + linkage | 0.3 |
| BEPU step (15–25 bodies/~30 constraints, ≤1.5k clumps, ~200 mostly-sleeping rocks, substepped 8,4) | 2.5–4.0 |
| carve + dirty marking | 0.3–0.8 |
| settle CA (amortized) | 0.5–1.0 |
| collision/render remesh (workers; ≤4–8 chunk uploads/frame) | 1.0–2.0 |
| render submit (orphaned VBOs only) | 2.0–3.0 |
| shell cosmetic particles (≤100k SoA, wall-clock) | 1.5–2.5 |
| audio/UI | 0.5 |
| GC + OS reserve | ≥ 2.0 |

Amortized systems degrade by **deferring**, never by blowing the frame; a debug counter on the accumulator clamp surfaces dropped time (the "soil teleports" symptom). Kernels are for-loops over SoA arrays, `[<Struct; IsReadOnly>]`, InlineIfLambda + WarnOn 3517, `Vector128` where SIMD pays.

**Allocation policy** (amendment — decided now, not Phase 5): `Step` returns events from a pooled buffer (drained `ResizeArray<GameEvent>` or struct events in a pooled array), **not** a freshly allocated F# list — because the Phase 4 MVP gate is "zero GC collections for 5 continuous minutes" and per-tick list allocation fails it by construction.

## 9. Determinism policy

Same-machine only. `Simulation.Deterministic = true`, pinned thread count, fixed chunk→worker partitioning, Margolus CA (order-free), seeded struct Rng.

**Mesher split** (amendment): render meshes may be built async on workers (timing-nondeterministic is fine — they only feed the GPU). **Collision** Mesh builds and their swap into the Simulation happen at a deterministic tick in deterministic chunk order (budgeted count per tick, joined before `Timestep`). Otherwise the swap tick depends on worker completion timing and the headline invariant silently breaks.

**Determinism gate** (amendment — beyond body poses): from Phase 1 onward the test compares bit-identical **hashes of the soil volume** (occupancy+material+compaction) **and the mass ledger** after 10k ticks of scripted digging, with worker-thread soil jobs and the multithreaded dispatcher enabled. Re-run at every phase exit (full rig in P3, FEE+CA in P4/P6).

**Input replay** (amendment): F-key-toggled recording of `InputFrame` streams to disk + deterministic playback through `Step` + a justfile recipe for headless replay. Turns any solver explosion or force pop into a one-command repro and doubles as the source of adversarial test scripts.

---

## 10. Phases

Ordered by risk. Each phase is independently verifiable; regression gates (determinism hash, mass conservation, zero-alloc, perf p99) re-run at every phase exit.

### Phase 0 — Bedrock (scaffold + BEPU-in-F# + determinism gate)

**Goal**: house-style repo standing; BEPU 2.5.0-beta.29 driven from F# at fixed 60 Hz; same-machine determinism proven.

Deliverables:
- Repo scaffold copied from fsharp-of-duty with names swapped: global.json, Directory.Build.props, sln, justfile (`_sdk` shim, `check`), ci.yml.
- `Gravemaskin.Core` with Prelude/Noise/MathEx/Domain/Tuning (U17 + Cat 320 + soil tables frozen with rationale comments) and a walking-skeleton `Sim.fs` (mutable World, `Step` returns pooled events + RenderState — the allocation policy of §8 from day one).
- `Bepu.fs` wrappers + `Physics.fs` seeded from `spikes/bepu-fsharp/Program.fs`; Deterministic=true, SolveDescription(8,4), fixed-thread dispatcher.
- **Track-mechanism spike** (amendment): verify against the real API whether contact tangent target velocity is settable (conveyor trick). If not, pre-commit the per-contact tractive-impulse fallback and write `Tracks.fs` against it from the start. Discovering this in Phase 3 means redesigning locomotion mid-phase.
- Tests: TestKit, ball-drops-and-rests smoke, determinism property (same seed + input script → bit-identical poses after 10k ticks, twice, threads on), zero-alloc scaffold (GC.CollectionCount delta over 1000 ticks = 0).

Exit: `just check` green on macOS arm64 · determinism test passes with threads · zero-alloc scaffold in place · 1000 empty ticks < 100 ms (sanity).

Risks: BEPU beta signature drift (pin + match installed package). No kill-criterion — this phase can only delay. If multithreaded determinism fails, pivot to single-threaded BEPU (fine at this body count).

### Phase 1 — Soil Round-Trip Core (THE spike: volume→particles→volume + BEPU collision)

**Goal**: retire the project-killer headlessly. Carve a chunked occupancy volume, spawn capped BEPU clumps, settle, convert back, swap per-chunk static Meshes — exact mass conservation, inside budget. No rendering.

Deliverables:
- `Soil/Api.fs` (the frozen seam — amendment), `Soil/Volume.fs`, `Soil/Surface.fs`, `Soil/Carve.fs` (CSG + exact ΔV + f64 **mass** ledger per §5).
- `Soil/Particles.fs`: variable-size clumps from carved ΔV, cap 1500, merge-on-overflow, spawn-rate cap, settle-detection write-back at w = 1/(1+S). **Escape guards** (amendment): kill-plane/world-bounds despawn returns mass to the ledger (logged, asserted); per-clump speed clamp / speculative-margin tuning against tunneling.
- `Physics.fs` chunk registry: static Mesh per chunk from Surface-Nets triangles (mesher stub OK), remove/add swap, `RecursivelyRemoveAndDispose`, budgeted N swaps/tick with queue.
- **Carve-to-collision-lag policy** (amendment): occupancy edits land the same tick, mesh swaps are queued — the representations disagree for several ticks. Rule: chunks with clump spawns this tick **jump the swap queue** (same-tick swap), or clump-vs-stale-chunk contacts are suppressed via the collision-group bitfield until the swap lands, or clumps spawn strictly above the old surface. Test: carve-and-spawn in one tick; no clump ever penetrates or rests on removed geometry.
- Deterministic collision-mesh scheduling per §9; determinism gate extended with volume + ledger hashes.
- Scripted "ghost bucket" (kinematic box) carves a 2 m trench over 600 ticks; FsCheck property: total **mass** per material conserved to 1e-6 relative across arbitrary carve/settle/compaction scripts.
- BenchmarkDotNet kernels: carve-and-mark, settle write-back, chunk Mesh rebuild+swap — **plus the chunk-size gate** (amendment): Surface Nets throughput at 32³ vs 16³(×8) on M-series in plain .NET, run BEFORE freezing chunk size; if real .NET lands at 3–6× the Burst reference, the chunk size changes here, because it's the one constant brutally expensive to change after Phase 2.

Exit: mass-conservation FsCheck green · headless trench p99 ≤ 6 ms over 10k ticks (carve ≤0.8, particles ≤2.5, settle ≤1.0, BEPU ≤4.0 with 1.5k clumps) · zero Gen0 collections across the scenario · BufferPool clean after 100 swap cycles · clump cap holds under 30 s max-rate carving with no mass drop · determinism hash green.

**KILL/PIVOT GATE (the big one)**: if BEPU + 1.5k clumps + chunk swaps can't fit ~6 ms headless, the ladder is (a) cap ~800 with bigger merged radii (Servin used ~950), (b) Servin aggregate-body coupling (one body per dig, particles cosmetic-only), (c) only if both fail — the full-physics-soil vision is dead; fall back to heightfield-only soil (MudRunner class, executable thanks to the Soil/Api seam) and re-scope. **Decide within this phase, not after rendering exists.** Secondary: swap churn during cave-ins → budget+queue already designed; if still spiky, `Tree.RefitAndRefine` for height-only changes.

### Phase 2 — See The Dirt (Silk.NET shell + meshing + debug dig tool)

**Goal**: first visual, independently shippable build — fly over seeded terrain, carve with a mouse-ray ghost bucket, watch clumps spill and resettle. Proves mesher, upload path, and render interpolation against the mutable World.

Deliverables: shell (Program loop with RenderState double buffer, Settings, Input, GlUtil, Shaders v1 flat-lit + sky, Renderer) · `Soil/Mesher.fs` finished (render path async per §9; 24 B vertex; baked AO from 3×3×3 tap; neighbor-border sampling; 4–8 orphaned chunk uploads/frame; pooled VAO/VBO/IBO) · instanced clod rendering (one hash-deformed 20–80-tri clod mesh, per-instance buffer orphaned per frame, driven from RenderState clump slice; cosmetic dust is shell-owned per §4) · debug tooling: fly camera, mouse-ray carve/deposit brush, F3 overlay (p50/p95/p99, GC counts, live clumps, dirty-queue depth, accumulator-clamp counter) · `just run`.

Exit: carve anywhere with no chunk-seam cracks (seam-torture scene) and no visual/physics mismatch (clumps rest ON the rendered surface) · 60 fps sustained at retina res while carving; remesh queue drains within 2 s · interpolation verified smooth on a 120 Hz display (the aliasing trap check) · CI builds run on Linux/Windows (the RenderDoc escape hatch — macOS has no GL frame debugger).

Risks: a single chunk mesh > ~3 ms → half-res LOD for distant chunks before micro-optimizing · `glBufferSubData` sneaking into a hot path (GlUtil only exposes orphaning) · per-frame allocation in the upload path (no Seq.collect habits here — preallocated arrays + Span from day one).

### Phase 3 — The Machine (excavator rig on rigid ground)

**Goal**: a drivable, diggable-in-air U17 on undeformable terrain: full linkage, quasistatic hydraulics with flow sharing, tracks, emergent tipping. Retires constraint-stability risk (mass ratios, stiff linkages) before soil coupling complicates diagnosis.

Deliverables:
- `Excavator/Linkage.fs`, `Excavator/Hydraulics.fs` (three U17 circuits, relief caps, corner-power clamp, extend/retract asymmetry), `Excavator/Tracks.fs` (mechanism per the Phase 0 spike result).
- `Physics.fs` rig per §6, **including the compound bucket** (amendment), collision-group self-collision masking, change-guarded `ApplyDescription` so the machine sleeps.
- `ProcGen/ExcavatorMesh.fs`: procedural U17 (~7 draws); boom-swing offset + dozer blade static-geometry for now.
- Control mapping per §7 (ISO/SAE, per-track axes, deadband→curve→lag in Core).
- **Minimal orbit-follow camera** (amendment): fixed-offset orbit, rotate + zoom, no polish — Phase 3's exit criteria are unverifiable through a free-fly camera while both hands operate the machine.
- **Live tuning + physics debug draw, pulled forward** (amendment): (a) debug-build file-watched overrides file (key=value, re-read every N ticks) covering FEE knobs, hydraulic caps, input-curve params; (b) debug-draw layer (points/quads only — glLineWidth clamps to 1.0 on macOS core) for FEE slice force vectors, contact impulses, COM + support polygon, active-zone wedge, chunk bounds, dirty-column heatmap. Phase 4 predicts "days of hand-tuning"; a recompile-per-knob loop makes that weeks.
- **tools/plot-linkage.fsx** (amendment, house fsi pattern): plots cylinder force, moment arm, joint torque over stroke per joint — separates "linkage math is wrong" from "solver is fighting" in minutes.
- **Input-recording replay** (amendment, per §9).
- TestKit scenarios: boom lifts rated load and stalls above cap · three simultaneous functions slower than one · scripted over-side moment tips the machine · pivot/counter-rotation.

Exit: no jitter/explosion over 10k ticks of adversarial scripts (full-speed reversals, all functions at once) · **breakout force emerges** at the ISO 6015 reference pose within ±10–15 % of 15.2 kN, never hardcoded, and boom-down measurably weaker than boom-up (amendment) · **two functions on one U17 circuit halve each other; cross-circuit don't** (amendment) · over-side tipping ≈ 1.33× rated (ISO 10567) · machine visibly rocks when the arm strikes ground at speed · machine sleeps within 2 s of idle input (change-guard check) · determinism gate green with full rig · headless machine-only p99 ≤ 1.5 ms.

Risks: **PIVOT GATE** — if analytic cylinders feel wrong (no compliance, over-crisp stalls), A/B real cylinder bodies on ONE joint first; never rebuild the rig speculatively · SpringSettings above ~120 Hz → instability; tune down · forgetting swing relief/brake → whippy house (one-line Tuning entries; test catches).

### Phase 4 — Steel Meets Soil (FEE coupling + bucket fill + deformable ground contact) — **MVP**

**Goal**: join Phases 1–3. The machine digs real soil: FEE resistance, active-zone spawning, bucket-load scalar, tracks on deformable surface, drag-yourself-with-the-bucket.

Deliverables: `Excavator/Fee.fs` wired into `Step` (per-slice depth/slope from surface cache, low-passed force+torque on the bucket, singularity guard, surcharge Q from clump weight on the wedge) · Rankine wedge triggering carve+spawn only where the blade fails soil · bucket-interior → load scalar (mass+COM on bucket body, re-emitted on dump) · track contact against the CURRENT surface cache each step (analytic contacts, no remeshed statics under the machine; μ scaled by moisture/looseness; sinkage-rate clamp — explicit "dig under own tracks" test so the machine never falls through fresh cells) · hooked-into-solid-soil path: bucket vs unbroken volume = high-force contact against the static, so track slide / house pivot emerge · GameEvents flowing (DigStarted, HydraulicStall, TrackSlip, SoilDumped, TipWarning) into HUD text + placeholder audio · Integration test: full dig-swing-dump-resettle cycle asserting end-to-end mass conservation including the load scalar.

Exit (MVP gate):
- Scripted full dig cycle (2 m³) conserves mass to 1e-6, p99 ≤ 12 ms for 5 continuous minutes on M-series, **zero GC collections** (achievable because pooled events were decided in Phase 0).
- Curling the bucket against dense soil at max reach lifts the front of the tracks — self-lift emerges.
- Swinging a full bucket fast over the side tips the machine; empty doesn't.
- Stall: stick-crowd into undisturbed clay stops that joint while other functions keep moving.
- FEE force continuity: no pops crossing cell boundaries (recorded force traces in a test).
- **The fun gate** (amendment, judge-unanimous): a non-operator friend plays unprompted for 10 minutes and keeps digging. The MVP is a toy worth shipping, not only a measurement.

Risks: **KILL/PIVOT GATE #2** — if direct FEE feels mushy or exploitable (bucket phases through soil under load), upgrade to the Servin aggregate body (temporary rigid body of wedge mass coupled by contacts) — budgeted upgrade, not redesign; if BOTH feel wrong, the realism bar drops, not the architecture · FEE singularity WILL be hit in normal play — c1/nominal-depth/singular-scale are Tuning calibration knobs; expect days of hand-tuning (tooling from Phase 3 exists for exactly this) · full frame budget now loaded — amortized systems defer first; accumulator-clamp counter surfaces dropped time.

### Phase 5 — Operator Feel (controls polish, cameras, HUD, audio)

**Goal**: it feels like operating a machine, not debugging one. Shippable "sandbox toy" build.

Deliverables: Settings UI (Ironsight Menu/SettingsUi trimmed: ISO/SAE, deadzone, invert, display, volume; GRAVEMASKIN_HOME) · gamepad + keyboard finalized per §7 **including the hold-LB axis-shift** for boom-swing/dozer blade (scheme pre-decided, amendment) · cameras: orbit-follow default, cab view, free-fly debug; shadow ortho box tracks machine + dig area · HUD: engine-load bar (corner-power utilization), per-circuit flow bars (makes flow-sharing legible), payload kg, tilt indicator + TipWarning flash — all from RenderState/events, 5×7 font · `Audio.fs`: engine loop pitched by load, hydraulic whine by flow, relief squeal on stall, track clatter, soil pour — all event-driven (sampled loops fine; procedural only if house AudioSynth ports trivially) · terrain shading v2: triplanar materials, moisture darkening, fresh-cut vs undisturbed, overcast-industrial palette, gamma+grade lifted from Ironsight.

Exit: an operator (or YouTube-calibrated proxy) can feather a load — verified subjectively AND by trace (commanded vs actual cylinder velocity linear until relief) · both patterns verified against ISO 10968 / SAE J1177 tables; mid-session switching works · all feedback event-driven — grep proves Renderer/Audio never read Core soil arrays · still p99 ≤ 12 ms, still zero-alloc.

Risks: feel tuning is unbounded — timebox; knobs live in the Phase 3 live-tuning file · audio rabbit hole — sampled loops are fine.

### Phase 6 — Living Ground (collapse, compaction, moisture, buried rocks)

**Goal**: soil behaves like a material over time. The "dirt is real" promise, completed.

Deliverables: cohesion face-failure (h_crit ≈ 4c/γ; wedge bursts; vibration triggers) · Margolus CA finished for continuous repose settling with backpressure (coarser steps under load, never frame blowout; dirty-region discipline applies to moisture diffusion too) · compaction: track passes raise the compaction byte toward bank density (affects μ, sinkage, dig resistance), **occupancy adjusted inversely so the mass ledger balances** (§5); ruts emerge from real volume deformation + remesh, no parallax tricks · moisture: per-cell byte (cohesion, unit weight, visual darkening; downward diffusion, evaporation stub) · buried rocks: convex-hull rigid bodies seeded in the volume, exposed by carving (occupancy override), RockStruck event + FEE bypass (rock = contact force, not soil failure) · Bekker-lite sinkage per contact box from compaction/moisture lookup (the deferred Phase 4 hook).

Exit: dig a vertical clay trench — wall stands; drive beside it — wall collapses onto the tracks (the signature moment; scripted test asserts collapse fires and mass conserves) · sand vs clay vs gravel blind-distinguishable by behavior alone · driving over a spoil pile compacts it (lower height, higher density, machine sits higher on pass two) · settle slice still ≤ 1.0 ms amortized · determinism + conservation gates green with CA + moisture + compaction live.

Risks: overhang/undercut fidelity (§5 — scoped pivot, not rewrite) · collapse bursts spike clump spawns (merge-on-overflow handles it) but big cave-ins queue chunk swaps — verify the Phase 1 budget queue under a worst-case 20-chunk collapse.

### Phase 7 — Ship It (Cat 320, content, save/load, packaging, hardening)

**Goal**: a releasable sandbox.

Deliverables: Cat 320 as a Tuning data entry + scaled procedural mesh; machine select (validates MachineSpec is actually data-driven) · sandbox dig site: seeded terrain with material strata, buried rocks, light objectives (dig-to-depth, load-a-volume — counters over the mass ledger, nothing more; a campaign is a different project) · **save/load with the in-flight-state policy** (amendment): save is only allowed after a forced settle pass converts resting clumps to volume; airborne clumps serialize as (pos, vel, radius, material); the bucket-load scalar folds into saved machine state; ledger written and re-verified on load · RLE+Deflate chunk serialization, machine pose, Settings, GRAVEMASKIN_HOME layout · release pipeline from Ironsight (loose Silk.NET natives comment MUST survive the copy; universal macOS .app x64+arm64 launcher, **ad-hoc codesign** — unsigned arm64 won't exec; pkgbuild; Linux tar.gz+deb; Windows zip+Inno) · hardening: ~120 hidden warmup ticks for tiered JIT, dotnet-trace/samply session folded in, accumulator-clamp telemetry review, final zero-alloc audit.

Exit: tag push produces installable artifacts on all three OSes · Cat 320 plays correctly with zero code changes outside Tuning/ProcGen (breakout ≈ 150 kN at reference pose) · **blind A/B** (amendment): U17 and Cat 320 unmistakably distinguishable by feel and sound alone — proving MachineSpec drives feel, not just a wired-up table · 30-minute session on base M1: stable RSS, no GC pause > 2 ms, p99 ≤ 12 ms, BufferPool clean at exit · save→quit→load preserves the mass ledger exactly.

---

## 11. Open questions (honest unknowns)

1. **Direct FEE vs Servin aggregate body**: plan starts direct (cheapest) with the aggregate as the Phase 4 pivot — but research says the aggregate gives the best coupling quality. If direct-path feel-tuning burns > a week, jumping straight to the aggregate may be cheaper overall.
2. **Analytic vs real cylinder bodies**: analytic avoids constraint cycles but loses oil compliance and cylinder collision. The Phase 3 one-joint A/B decides; no research settles which *feels* better in BEPU specifically.
3. **Clump cap calibration**: Servin's ~950 was validated against DEM for one scenario; whether 1500 BEPU spheres + cosmetic layer *reads* as enough dirt is a taste call for Phases 1–2.
4. **Overhang fidelity**: single surface cache + cohesion-failure fallback is the chosen simplification; players will try to tunnel. Multi-level heightfield spans are the largest unbudgeted item.
5. **macOS OpenGL end-of-life**: Tahoe still ships conformant GL 4.1 but degradation signals exist (AGL removal). The two-file renderer boundary is the hedge; revisit each macOS major.
6. **BEPU beta pinning**: 2.5.0-beta.29 is the de-facto current line (Stride ships on betas) but signatures drift; a needed fix landing only on master forces a migration cost.
7. **Concurrent GC on/off**: bloom disables it; perf research says workstation+concurrent+SustainedLowLatency. Phase 4 p99 data settles it empirically.
8. **Cross-platform determinism** is not promised (ARM ≠ x64 FP). If shareable replays ever matter, that's a fundamental constraint, not a bug.
9. **Track model on deformable soil**: conveyor boxes are the researched consensus on rigid ground; nothing tests them on voxel soil with per-cell μ/sinkage at 60 Hz. Phase 4's dig-under-own-tracks test is the first real evidence; kinematic-ICR (Martinez) is the named fallback.

## 12. References

- `docs/research/soil-sim.md` — terrain/soil techniques (Servin/agxTerrain, Holz, MPM verdict, CA settling, meshing perf)
- `docs/research/excavator-physics.md` — linkage, hydraulics, tracks, real spec-sheet numbers, control patterns
- `docs/research/bepu-fsharp.md` — BEPU v2 API, constraints, F# interop, Jolt comparison, deformable-terrain collision patterns
- `docs/research/rendering.md` — GL 4.1 ceiling, chunk streaming, instancing, house renderer patterns
- `docs/research/repo-arch.md` — Ironsight/bloom archaeology: loop, project split, tests, CI, packaging
- `docs/research/perf-dotnet.md` — F#/.NET hot-loop idioms, GC strategy, SIMD on Apple Silicon, budgets
- `spikes/bepu-fsharp/` — verified minimal BEPU 2.5.0-beta.29 F# program (callback structs, hinge + servo)

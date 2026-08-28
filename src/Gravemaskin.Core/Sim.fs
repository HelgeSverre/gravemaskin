namespace Gravemaskin

open System.Numerics

/// The mutable world (bloom precedent). The house invariant is headless
/// determinism behind Step(InputFrame), not immutability: BEPU is pool-based
/// and soil is flat arrays.
type World(seed: uint64, threadCount: int, soil: (SoilConfig * SoilMaterial * float32) option) =
    let physics = new Physics(threadCount)
    let mutable rng = Rng.create seed
    let mutable tick = 0L
    // Pooled event buffer: cleared at the start of each Step, valid until the
    // next Step. Never allocated per tick (the zero-alloc gate depends on it).
    let events = ResizeArray<GameEvent>(64)
    // Carve scratch (per-material kg) — reused every carve, never allocated.
    let carveScratch = Array.zeroCreate<float> 5

    let soilState =
        match soil with
        | Some(config, mat, groundHeight) ->
            let state = Soil.create config mat groundHeight
            Some state
        | None ->
            // No soil volume: a plain rigid slab so physics tests have ground.
            Bepu.addStaticBox physics.Simulation (Vector3(0.0f, -0.5f, 0.0f)) (Vector3(200.0f, 1.0f, 200.0f))
            |> ignore

            None

    let clumps = ClumpPool()

    do
        // Initial collision build: every tile, synchronously, before tick 0.
        match soilState with
        | Some state ->
            for tile in 0 .. state.TilesX * state.TilesZ - 1 do
                physics.SwapTileMesh(state, tile)

            Array.fill state.DirtyMesh 0 state.DirtyMesh.Length false
        | None -> ()

    /// Collision-mesh swaps allowed per tick once running (amortization).
    static member val MeshSwapBudget = 8 with get

    member _.Physics = physics
    member _.Tick = tick
    member _.SoilState = soilState
    member _.Clumps = clumps

    /// Events raised by the most recent Step; consume before stepping again.
    member _.Events: ResizeArray<GameEvent> = events

    /// Carve a sphere from the soil and spawn the removed mass as loose
    /// clumps above the cut (cap overflow deposits straight back instead).
    member _.CarveSphere(center: Vector3, radius: float32) =
        match soilState with
        | None -> 0.0
        | Some state ->
            let total = Soil.carveSphere state center radius carveScratch

            if total > 0.0 then
                events.Add DigStarted

                // Split each material's mass into clump-sized pieces.
                for materialIndex in 0 .. carveScratch.Length - 1 do
                    let mutable remaining = carveScratch.[materialIndex]

                    if remaining > 0.0 then
                        let material = Volume.materialOfByte (byte materialIndex)
                        // ~0.12 m radius clump at loose density.
                        let props = Tuning.soil material

                        let clumpMass =
                            float (
                                Volume.looseDensity props
                                * (4.0f / 3.0f * System.MathF.PI * 0.12f * 0.12f * 0.12f)
                            )

                        while remaining > 0.0 do
                            let mass = min remaining clumpMass
                            remaining <- remaining - mass

                            let jitter =
                                Vector3(
                                    (Rng.nextFloat32 &rng - 0.5f) * radius,
                                    Rng.nextFloat32 &rng * 0.3f + 0.2f,
                                    (Rng.nextFloat32 &rng - 0.5f) * radius
                                )

                            // Spawn strictly above BOTH surfaces: the new one
                            // and the stale collision mesh (swap is budgeted,
                            // so the old surface can persist a few ticks —
                            // spawning inside it would pop the clump out).
                            // Carving from above means the sphere top bounds
                            // the old surface in the carved region.
                            let clearY =
                                max (Soil.surfaceHeight state center.X center.Z) (center.Y + radius)

                            let position = Vector3(center.X, clearY, center.Z) + jitter

                            if not (clumps.TrySpawn(physics.Simulation, position, mass, material)) then
                                // Cap overflow: bank it immediately, mass-neutral.
                                Soil.deposit state position mass material

            total

    member _.Step(_input: InputFrame) : RenderState =
        events.Clear()
        physics.Step()

        match soilState with
        | Some state ->
            clumps.SettlePass(physics.Simulation, state)
            Soil.settleTick state
            physics.SwapDirtyTiles(state, World.MeshSwapBudget) |> ignore
        | None -> ()

        tick <- tick + 1L

        { Tick = tick
          BodyCount = physics.BodyCount }

    interface System.IDisposable with
        member _.Dispose() = (physics :> System.IDisposable).Dispose()

[<RequireQualifiedAccess>]
module Sim =
    /// Pinned thread count for same-machine determinism (never
    /// Environment.ProcessorCount directly — a replay on the same box must
    /// use the same count).
    let defaultThreadCount = 4

    let createWorld seed = new World(seed, defaultThreadCount, None)

    /// Standard soil test bed: 32×32 m at 0.25 m cells, 8 m tall, 2 m of soil.
    let defaultSoilConfig =
        { CellSize = 0.25f
          CellsX = 128
          CellsY = 32
          CellsZ = 128 }

    let createSoilWorld seed mat groundHeight =
        new World(seed, defaultThreadCount, Some(defaultSoilConfig, mat, groundHeight))

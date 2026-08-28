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
    let mutable machine: Machine option = None
    // Low-passed FEE resistance (force never pops across cell boundaries).
    let mutable feeForce = Vector3.Zero

    let surfaceHeight (x: float32) (z: float32) =
        match soilState with
        | Some state -> Soil.surfaceHeight state x z
        | None -> 0.0f

    do
        // Initial collision build: every tile, synchronously, before tick 0.
        match soilState with
        | Some state ->
            for tile in 0 .. state.TilesX * state.TilesZ - 1 do
                physics.SwapTileMesh(state, tile)

            Array.fill state.DirtyMesh 0 state.DirtyMesh.Length false
        | None -> ()

    /// Collision-mesh swaps allowed per tick once running. Each swap builds
    /// a fresh BVH for a 2048-triangle tile (~ms); one per tick keeps p99
    /// inside budget while a dig trail still refreshes within a few ticks.
    /// ponytail: swap-cost ceiling — if refresh latency ever bites, go to
    /// 16³ tiles or Tree.RefitAndRefine for height-only changes.
    static member val MeshSwapBudget = 1 with get

    member _.Physics = physics
    member _.Tick = tick
    member _.SoilState = soilState
    member _.Clumps = clumps
    member _.Machine = machine

    /// Spawn the excavator with its tracks resting at the surface under
    /// `position` (XZ).
    member _.SpawnMachine(position: Vector3) =
        let ground = surfaceHeight position.X position.Z
        let spawned = Machine(physics, Tuning.u17, Vector3(position.X, ground + 0.01f, position.Z))
        machine <- Some spawned
        spawned

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

    /// Spawn one clump, banking the mass instead if the pool is full.
    member private _.SpawnClump(position: Vector3, velocity: Vector3, mass: float, material: SoilMaterial) =
        match soilState with
        | Some state ->
            if clumps.TrySpawn(physics.Simulation, position, mass, material) then
                // Give it the bucket's velocity so pours look like pours.
                let handle = clumps.Handles.[clumps.Count - 1]
                let mutable bodyRef = physics.Simulation.Bodies.[handle]
                bodyRef.Velocity.Linear <- velocity
            else
                Soil.deposit state position mass material
        | None -> ()

    /// The dig system: bucket cutting edge vs the soil volume. Carve where
    /// the moving edge is below the surface, absorb into the payload, resist
    /// via FEE, pour out when the bucket opens.
    member private this.DigTick() =
        match machine, soilState with
        | Some m, Some state ->
            let edge = m.BucketTipPosition
            let edgeVelocity = m.CuttingEdgeVelocity
            let surface = Soil.surfaceHeight state edge.X edge.Z
            let depth = surface - edge.Y
            let speed = edgeVelocity.Length()
            let mutable targetForce = Vector3.Zero

            if depth > 0.02f && speed > Tuning.CutMinSpeed then
                // Peek the material where the edge cuts (for FEE), then carve.
                let config = state.Config
                let cx = int (edge.X / config.CellSize) |> max 0 |> min (config.CellsX - 1)
                let cz = int (edge.Z / config.CellSize) |> max 0 |> min (config.CellsZ - 1)
                let cy = int (edge.Y / config.CellSize) |> max 0 |> min (config.CellsY - 1)
                let index = state.Index(cx, cy, cz)
                let matByte = state.Material.[index]
                let compByte = state.Compaction.[index]
                let props = Tuning.soil (Volume.materialOfByte matByte)

                let removed = Soil.carveSphere state edge Tuning.CutRadius carveScratch

                if removed > 0.0 then
                    events.Add DigStarted

                    for materialIndex in 0..4 do
                        if carveScratch.[materialIndex] > 0.0 then
                            let absorbed = m.TryAbsorb(carveScratch.[materialIndex], materialIndex)
                            let spill = carveScratch.[materialIndex] - absorbed

                            if spill > 0.001 then
                                // Overflow spills over the bucket as clumps.
                                let jitter =
                                    Vector3(
                                        (Rng.nextFloat32 &rng - 0.5f) * 0.4f,
                                        0.3f + Rng.nextFloat32 &rng * 0.2f,
                                        (Rng.nextFloat32 &rng - 0.5f) * 0.4f
                                    )

                                this.SpawnClump(
                                    Vector3(edge.X, max surface (edge.Y + Tuning.CutRadius), edge.Z) + jitter,
                                    Vector3.Zero,
                                    spill,
                                    Volume.materialOfByte (byte materialIndex)
                                )

                // FEE resistance opposing the cut direction.
                let magnitude = Fee.resistance props compByte depth Tuning.CutWidth
                targetForce <- -Vector3.Normalize edgeVelocity * magnitude

            feeForce <- Vector3.Lerp(feeForce, targetForce, Tuning.FeeSmoothing)

            if feeForce.LengthSquared() > 1.0f then
                let mutable bucketRef = physics.Simulation.Bodies.[m.Bucket]

                if not bucketRef.Awake then
                    bucketRef.Awake <- true

                bucketRef.ApplyImpulse(feeForce * Tuning.FixedDt, edge - bucketRef.Pose.Position)

            // Pour the payload out of an open bucket.
            match m.DumpTick() with
            | ValueSome(released, materialIndex) ->
                events.Add(SoilDumped(float32 released))

                this.SpawnClump(
                    edge + Vector3(0.0f, -0.1f, 0.0f),
                    edgeVelocity,
                    released,
                    Volume.materialOfByte (byte materialIndex)
                )
            | ValueNone -> ()

            m.RefreshLoadInertia()
        | _ -> ()

    member this.Step(input: InputFrame) : RenderState =
        events.Clear()

        match machine with
        | Some m ->
            m.Step(input, Tuning.FixedDt, surfaceHeight)

            if m.StallActive then
                events.Add HydraulicStall

            if m.ChassisTilt > 0.20f then
                events.Add TipWarning
        | None -> ()

        this.DigTick()
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

    /// Copy the render-relevant clump state into a shell-owned snapshot.
    /// Preallocated arrays, no allocation: the shell keeps two of these and
    /// interpolates between them by handle.
    member _.SnapshotInto(snapshot: RenderSnapshot) =
        snapshot.Tick <- tick
        let count = min clumps.Count snapshot.Capacity
        snapshot.Count <- count

        for i in 0 .. count - 1 do
            let bodyRef = physics.Simulation.Bodies.[clumps.Handles.[i]]
            let position = bodyRef.Pose.Position
            snapshot.Handles.[i] <- clumps.Handles.[i].Value
            snapshot.X.[i] <- position.X
            snapshot.Y.[i] <- position.Y
            snapshot.Z.[i] <- position.Z
            snapshot.Materials.[i] <- clumps.Materials.[i]

            let props = Tuning.soil (Volume.materialOfByte clumps.Materials.[i])
            let volume = float32 clumps.Masses.[i] / Volume.looseDensity props

            snapshot.Radius.[i] <-
                System.MathF.Cbrt(volume * 3.0f / (4.0f * System.MathF.PI))
                |> max Clumps.MinRadius
                |> min Clumps.MaxRadius

        match machine with
        | Some m -> m.FillSnapshot snapshot
        | None -> snapshot.MachinePartCount <- 0

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

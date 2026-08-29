namespace Gravemaskin

open System.Numerics

/// The mutable world (bloom precedent). The house invariant is headless
/// determinism behind Step(InputFrame), not immutability: BEPU is pool-based
/// and soil is flat arrays.
type SoilSetup =
    | FlatSoil of SoilConfig * SoilMaterial * float32
    | TerrainSoil of SoilConfig * int * float32 * float32
    | PrebuiltSoil of SoilState

type World(seed: uint64, threadCount: int, soil: SoilSetup option) =
    let physics = new Physics(threadCount)
    let mutable rng = Rng.create seed
    let mutable tick = 0L
    // Pooled event buffer: cleared at the start of each Step, valid until the
    // next Step. Never allocated per tick (the zero-alloc gate depends on it).
    let events = ResizeArray<GameEvent>(64)
    // Carve scratch (per-material kg) — reused every carve, never allocated.
    let carveScratch = Array.zeroCreate<float> MaterialCount

    let soilState =
        match soil with
        | Some(FlatSoil(config, mat, groundHeight)) -> Some(Soil.create config mat groundHeight)
        | Some(TerrainSoil(config, terrainSeed, baseHeight, relief)) ->
            Some(Soil.createTerrain config terrainSeed baseHeight relief)
        | Some(PrebuiltSoil state) -> Some state
        | None ->
            // No soil volume: a plain rigid slab so physics tests have ground.
            Bepu.addStaticBox physics.Simulation (Vector3(0.0f, -0.5f, 0.0f)) (Vector3(200.0f, 1.0f, 200.0f))
            |> ignore

            None

    let clumps = ClumpPool()
    let mutable machine: Machine option = None
    // Low-passed FEE resistance (force never pops across cell boundaries).
    let mutable feeForce = Vector3.Zero
    // Wall-failure scratch (allocation-free after warmup).
    let wallFailures = ResizeArray<WallFailure>(16)
    // Buried rocks: kinematic while buried, dynamic once exposed.
    let rockHandles = ResizeArray<BepuPhysics.BodyHandle>(64)
    let rockRadii = ResizeArray<float32>(64)
    let rockExposed = ResizeArray<bool>(64)
    let mutable rockStruckCooldown = 0
    // Rotating column cursor for the budgeted moisture pass.
    let mutable moistureCursor = 0

    let surfaceHeight (x: float32) (z: float32) =
        match soilState with
        | Some state -> Soil.surfaceHeight state x z
        | None -> 0.0f

    // Bound ONCE: passing the let-bound function directly to Machine.Step
    // materialized a fresh closure every tick (24 B/tick — review finding).
    let surfaceHeightF: float32 -> float32 -> float32 = surfaceHeight

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
        let spawned = Machine(physics, Tuning.u17Rig, Vector3(position.X, ground + 0.01f, position.Z))
        machine <- Some spawned
        spawned

    member _.SpawnMachineRig(rig: MachineRig, position: Vector3) =
        let ground = surfaceHeight position.X position.Z
        let spawned = Machine(physics, rig, Vector3(position.X, ground + 0.01f, position.Z))
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
                let moistByte = state.Moisture.[index]
                let props = Tuning.soil (Volume.materialOfByte matByte)

                let removed = Soil.carveSphere state edge Tuning.CutRadius carveScratch

                if removed > 0.0 then
                    events.Add DigStarted

                    for materialIndex in 0 .. MaterialCount - 1 do
                        if carveScratch.[materialIndex] > 0.0 then
                            let absorbed = m.TryAbsorb(carveScratch.[materialIndex], materialIndex)
                            let spill = carveScratch.[materialIndex] - absorbed

                            // Sub-clump spill still exists: bank it in the
                            // residual rather than destroying it (the only
                            // path that ever leaked mass — review finding).
                            if spill > 0.0 && spill <= 0.001 then
                                Soil.creditUnbanked state materialIndex spill

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
                let magnitude = Fee.resistance props compByte moistByte depth Tuning.CutWidth
                targetForce <- -Vector3.Normalize edgeVelocity * magnitude

            feeForce <- Vector3.Lerp(feeForce, targetForce, Tuning.FeeSmoothing)

            if feeForce.LengthSquared() > 1.0f then
                let mutable bucketRef = physics.Simulation.Bodies.[m.Bucket]

                if not bucketRef.Awake then
                    bucketRef.Awake <- true

                bucketRef.ApplyImpulse(feeForce * Tuning.FixedDt, edge - bucketRef.Pose.Position)

            // Scoop: loose clumps cradled inside a well-curled bucket
            // convert to payload (the training-sim trick — clump budget
            // freed, weight still on the linkage).
            if m.BucketAngle < -1.0f then
                let bucketCenter = physics.Simulation.Bodies.[m.Bucket].Pose.Position
                let mouthRadius = 0.45f * m.Rig.Scale
                let mutable i = 0

                while i < clumps.Count do
                    let clumpPosition = physics.Simulation.Bodies.[clumps.Handles.[i]].Pose.Position

                    if Vector3.DistanceSquared(clumpPosition, bucketCenter) < mouthRadius * mouthRadius then
                        let taken = m.TryAbsorb(clumps.Masses.[i], int clumps.Materials.[i])

                        if taken >= clumps.Masses.[i] - 1e-9 then
                            clumps.RemoveAt(physics.Simulation, i)
                        else
                            // No room: leave it be (partial absorb would
                            // need a mass edit on a live body).
                            m.TryAbsorb(-taken, int clumps.Materials.[i]) |> ignore
                            i <- i + 1
                    else
                        i <- i + 1

            // Pour the payload out of an open bucket.
            match m.DumpTick() with
            | ValueSome(released, materialIndex) ->
                events.Add(SoilDumped(float32 released, byte materialIndex))

                // Dirt slides out with the bucket BODY's motion; the lip's
                // rotational velocity would catapult it skyward.
                let bodyVelocity = physics.Simulation.Bodies.[m.Bucket].Velocity.Linear

                let pourVelocity =
                    Vector3(bodyVelocity.X, System.MathF.Min(bodyVelocity.Y, 0.2f) - 0.4f, bodyVelocity.Z)

                this.SpawnClump(
                    edge + Vector3(0.0f, -0.1f, 0.0f),
                    pourVelocity,
                    released,
                    Volume.materialOfByte (byte materialIndex)
                )
            | ValueNone -> ()

            m.RefreshLoadInertia()
        | _ -> ()

    /// Track passes squeeze loose soil toward bank density: same mass,
    /// less volume — ruts emerge from real deformation, no decals.
    member private _.CompactionTick(state: SoilState) =
        match machine with
        | Some m ->
            for side in 0..1 do
                if System.MathF.Abs(m.TrackAxis side) > 0.15f then
                    let point = m.TrackContactPoint side
                    let config = state.Config
                    let x = int (point.X / config.CellSize) |> max 0 |> min (config.CellsX - 1)
                    let z = int (point.Z / config.CellSize) |> max 0 |> min (config.CellsZ - 1)
                    // Only compact ground the track actually presses on.
                    if point.Y - state.Heights.[state.ColumnIndex(x, z)] < 0.35f then
                        let mutable y = config.CellsY - 1

                        while y >= 0 && state.Occupancy.[state.Index(x, y, z)] = 0uy do
                            y <- y - 1

                        if y >= 0 then
                            let index = state.Index(x, y, z)
                            let compOld = state.Compaction.[index]

                            if compOld < 255uy then
                                let matByte = state.Material.[index]
                                let occ = int state.Occupancy.[index]
                                let compNew = byte (min 255 (int compOld + 3))
                                let mpuOld = Volume.massPerOccUnit config matByte compOld
                                let mpuNew = Volume.massPerOccUnit config matByte compNew
                                let massOld = float occ * mpuOld
                                let occNew = int (massOld / mpuNew)
                                state.Compaction.[index] <- compNew
                                state.Occupancy.[index] <- byte (min 255 occNew)
                                // Quantization residual stays ledgered.
                                state.Unbanked.[int matByte] <-
                                    state.Unbanked.[int matByte]
                                    + (massOld - float (min 255 occNew) * mpuNew)

                                Volume.refreshColumnHeight state x z
                                state.MarkDirty(x, z)
        | None -> ()

    /// Seed buried rocks: kinematic (soil "holds" them) until exposed.
    member _.SeedRocks(count: int) =
        match soilState with
        | Some state ->
            let config = state.Config

            for _ in 1..count do
                let x = 4.0f + Rng.nextFloat32 &rng * (float32 config.CellsX * config.CellSize - 8.0f)
                let z = 4.0f + Rng.nextFloat32 &rng * (float32 config.CellsZ * config.CellSize - 8.0f)
                let radius = 0.2f + Rng.nextFloat32 &rng * 0.18f
                let surface = Soil.surfaceHeight state x z
                let y = surface - radius - 0.15f - Rng.nextFloat32 &rng * 0.6f

                if y > radius then
                    let handle =
                        Bepu.addKinematicSphere physics.Simulation (Vector3(x, y, z)) radius

                    rockHandles.Add handle
                    rockRadii.Add radius
                    rockExposed.Add false
        | None -> ()

    member _.Rocks = rockHandles

    /// Expose rocks the digging uncovers (kinematic → dynamic) and raise
    /// RockStruck when the cutting edge slams one.
    member private _.RockTick(state: SoilState) =
        rockStruckCooldown <- max 0 (rockStruckCooldown - 1)

        // Exposure is amortized: a few rocks per tick, round-robin by tick.
        for i in 0 .. rockHandles.Count - 1 do
            if not rockExposed.[i] && (int (tick % 16L) = i % 16) then
                let bodyRef = physics.Simulation.Bodies.[rockHandles.[i]]
                let position = bodyRef.Pose.Position
                let surface = Soil.surfaceHeight state position.X position.Z

                // Convert only once the rock's center clears the remaining
                // floor: a dynamic body released below the one-sided surface
                // mesh falls out of the world.
                if surface < position.Y - rockRadii.[i] * 0.25f then
                    rockExposed.[i] <- true
                    let shape = BepuPhysics.Collidables.Sphere(rockRadii.[i])
                    let volume = 4.0f / 3.0f * System.MathF.PI * rockRadii.[i] ** 3.0f
                    let mutable inertia = shape.ComputeInertia(volume * 2600.0f)
                    // Kinematic→dynamic MUST go through SetLocalInertia: a
                    // direct LocalInertia write corrupts solver batches
                    // (AccessViolation in ScatterInertia, found the hard way).
                    physics.Simulation.Bodies.SetLocalInertia(rockHandles.[i], &inertia)
                    let mutable exposedRef = physics.Simulation.Bodies.[rockHandles.[i]]
                    exposedRef.Awake <- true

        match machine with
        | Some m when rockStruckCooldown = 0 ->
            let edge = m.BucketTipPosition
            let speed = m.CuttingEdgeVelocity.Length()

            if speed > 0.4f then
                for i in 0 .. rockHandles.Count - 1 do
                    let position = physics.Simulation.Bodies.[rockHandles.[i]].Pose.Position

                    if
                        rockStruckCooldown = 0
                        && Vector3.DistanceSquared(position, edge) < (rockRadii.[i] + 0.2f) ** 2.0f
                    then
                        events.Add RockStruck
                        rockStruckCooldown <- 30
        | _ -> ()

    /// Restore one rock from a save.
    member _.AddRock(position: Vector3, radius: float32, exposed: bool) =
        let handle = Bepu.addKinematicSphere physics.Simulation position radius
        rockHandles.Add handle
        rockRadii.Add radius
        rockExposed.Add exposed

        if exposed then
            let shape = BepuPhysics.Collidables.Sphere(radius)
            let volume = 4.0f / 3.0f * System.MathF.PI * radius ** 3.0f
            let mutable inertia = shape.ComputeInertia(volume * 2600.0f)
            physics.Simulation.Bodies.SetLocalInertia(handle, &inertia)

    /// Save policy (SPEC amendment): force-settle first — all airborne
    /// clumps and the bucket payload bank into the volume, so the ledger
    /// round-trips exactly and nothing in flight is lost.
    member this.Save(path: string) =
        match soilState with
        | None -> ()
        | Some state ->
            // Bank every live clump where it is.
            while clumps.Count > 0 do
                let position = physics.Simulation.Bodies.[clumps.Handles.[0]].Pose.Position
                Soil.deposit state position clumps.Masses.[0] (Volume.materialOfByte clumps.Materials.[0])
                clumps.RemoveAt(physics.Simulation, 0)

            // Fold the payload out onto the ground under the bucket.
            match machine with
            | Some m ->
                for i in 0 .. MaterialCount - 1 do
                    if m.BucketLoad.[i] > 0.0 then
                        Soil.deposit state m.BucketTipPosition m.BucketLoad.[i] (Volume.materialOfByte (byte i))
                        m.BucketLoad.[i] <- 0.0
            | None -> ()

            use stream = System.IO.File.Create path
            use writer = new System.IO.BinaryWriter(stream)
            writer.Write "GRAV2"
            writer.Write state.Config.CellSize
            writer.Write state.Config.CellsX
            writer.Write state.Config.CellsY
            writer.Write state.Config.CellsZ

            let writeCompressed (data: byte[]) =
                use buffer = new System.IO.MemoryStream()

                do
                    use deflate =
                        new System.IO.Compression.DeflateStream(
                            buffer,
                            System.IO.Compression.CompressionLevel.Fastest,
                            true
                        )

                    deflate.Write(data, 0, data.Length)

                writer.Write(int buffer.Length)
                writer.Write(buffer.ToArray())

            writeCompressed state.Occupancy
            writeCompressed state.Material
            writeCompressed state.Compaction
            writeCompressed state.Moisture
            writer.Write state.WaterTableHeight

            for i in 0 .. MaterialCount - 1 do
                writer.Write state.Ledger.[i]
                writer.Write state.Unbanked.[i]

            match machine with
            | Some m ->
                writer.Write true
                writer.Write m.Rig.Spec.Name
                let position = physics.Simulation.Bodies.[m.Chassis].Pose.Position
                writer.Write position.X
                writer.Write position.Z
            | None -> writer.Write false

            writer.Write rockHandles.Count

            for i in 0 .. rockHandles.Count - 1 do
                let position = physics.Simulation.Bodies.[rockHandles.[i]].Pose.Position
                writer.Write position.X
                writer.Write position.Y
                writer.Write position.Z
                writer.Write rockRadii.[i]
                writer.Write rockExposed.[i]

    member this.Step(input: InputFrame) : RenderState =
        events.Clear()

        match machine with
        | Some m ->
            m.Step(input, Tuning.FixedDt, surfaceHeightF)

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
            Soil.settleTick state wallFailures

            // Cohesive wedge failures tumble off the face as clumps.
            for failure in wallFailures do
                events.Add WallCollapsed

                let mass =
                    float failure.Units
                    * Volume.massPerOccUnit state.Config failure.MaterialByte failure.CompactionByte

                let position =
                    Vector3(
                        (float32 failure.X + 0.5f) * state.Config.CellSize,
                        state.Heights.[state.ColumnIndex(failure.X, failure.Z)] + 0.35f,
                        (float32 failure.Z + 0.5f) * state.Config.CellSize
                    )

                this.SpawnClump(position, Vector3.Zero, mass, Volume.materialOfByte failure.MaterialByte)

            wallFailures.Clear()
            moistureCursor <- Moisture.tick state state.WaterTableHeight moistureCursor
            this.CompactionTick state
            this.RockTick state
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

        let rockCount = min rockHandles.Count snapshot.RockPositions.Length
        snapshot.RockCount <- rockCount

        for i in 0 .. rockCount - 1 do
            snapshot.RockPositions.[i] <- physics.Simulation.Bodies.[rockHandles.[i]].Pose.Position
            snapshot.RockRadii.[i] <- rockRadii.[i]

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
        new World(seed, defaultThreadCount, Some(FlatSoil(defaultSoilConfig, mat, groundHeight)))

    /// The sandbox: 64×64 m rolling terrain — grass meadows, sand and
    /// gravel patches, wet lowlands over a water table, clay underneath.
    let sandboxSoilConfig =
        { CellSize = 0.25f
          CellsX = 256
          CellsY = 48
          CellsZ = 256 }

    let createTerrainWorld seed =
        new World(seed, defaultThreadCount, Some(TerrainSoil(sandboxSoilConfig, int seed, 3.2f, 1.6f)))

    /// Load a save written by World.Save. The arm respawns in its parked
    /// pose (poses of five constrained bodies aren't worth serializing);
    /// terrain, ledger, machine placement, and rocks round-trip exactly.
    let loadWorld (seed: uint64) (path: string) =
        use stream = System.IO.File.OpenRead path
        use reader = new System.IO.BinaryReader(stream)

        if reader.ReadString() <> "GRAV2" then
            failwith "not a Gravemaskin save"

        let config =
            { CellSize = reader.ReadSingle()
              CellsX = reader.ReadInt32()
              CellsY = reader.ReadInt32()
              CellsZ = reader.ReadInt32() }

        let state = SoilState(config)

        let readCompressed (target: byte[]) =
            let length = reader.ReadInt32()
            let compressed = reader.ReadBytes length
            use buffer = new System.IO.MemoryStream(compressed)

            use deflate =
                new System.IO.Compression.DeflateStream(buffer, System.IO.Compression.CompressionMode.Decompress)

            let mutable offset = 0
            let mutable read = 1

            while read > 0 && offset < target.Length do
                read <- deflate.Read(target, offset, target.Length - offset)
                offset <- offset + read

        readCompressed state.Occupancy
        readCompressed state.Material
        readCompressed state.Compaction
        readCompressed state.Moisture
        state.WaterTableHeight <- reader.ReadSingle()

        for i in 0 .. MaterialCount - 1 do
            state.Ledger.[i] <- reader.ReadDouble()
            state.Unbanked.[i] <- reader.ReadDouble()

        for z in 0 .. config.CellsZ - 1 do
            for x in 0 .. config.CellsX - 1 do
                Volume.refreshColumnHeight state x z

        let world = new World(seed, defaultThreadCount, Some(PrebuiltSoil state))

        if reader.ReadBoolean() then
            let name = reader.ReadString()
            let x = reader.ReadSingle()
            let z = reader.ReadSingle()
            world.SpawnMachineRig(Tuning.rigByName name, System.Numerics.Vector3(x, 0.0f, z)) |> ignore

        let rockCount = reader.ReadInt32()

        for _ in 1..rockCount do
            let x = reader.ReadSingle()
            let y = reader.ReadSingle()
            let z = reader.ReadSingle()
            let radius = reader.ReadSingle()
            let exposed = reader.ReadBoolean()
            world.AddRock(System.Numerics.Vector3(x, y, z), radius, exposed)

        world

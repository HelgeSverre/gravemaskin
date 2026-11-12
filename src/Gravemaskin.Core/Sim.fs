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
    // Cut soil the bucket didn't capture, waiting to leave the blade as a
    // shovelful clump (kg per material). Mass in flight: counted by the
    // conservation check alongside clumps and the payload.
    let spillPending = Array.zeroCreate<float> MaterialCount
    let mutable spillPosition = Vector3.Zero
    // Where the spill is thrown (blade motion) and how wide (lip half-span).
    let mutable spillVelocity = Vector3.Zero
    let mutable spillSpread = Vector3.Zero
    // Scenery: trees are static trunks; the shell draws them at ground level.
    let treePositions = ResizeArray<Vector3>(96)
    let treeHeights = ResizeArray<float32>(96)

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

            // World boundary: the soil mesh simply ends at the map edge, and
            // a machine driven over it fell forever. Four tall static walls
            // just outside the volume keep machines, rocks and clumps in.
            let worldX = float32 state.Config.CellsX * state.Config.CellSize
            let worldZ = float32 state.Config.CellsZ * state.Config.CellSize
            let wall (center: Vector3) (size: Vector3) =
                Bepu.addStaticBox physics.Simulation center size |> ignore

            wall (Vector3(-2.0f, 20.0f, worldZ * 0.5f)) (Vector3(4.0f, 60.0f, worldZ + 8.0f))
            wall (Vector3(worldX + 2.0f, 20.0f, worldZ * 0.5f)) (Vector3(4.0f, 60.0f, worldZ + 8.0f))
            wall (Vector3(worldX * 0.5f, 20.0f, -2.0f)) (Vector3(worldX + 8.0f, 60.0f, 4.0f))
            wall (Vector3(worldX * 0.5f, 20.0f, worldZ + 2.0f)) (Vector3(worldX + 8.0f, 60.0f, 4.0f))
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
    member this.SpawnMachine(position: Vector3) = this.SpawnMachineRig(Tuning.u17Rig, position)

    /// Swap the machine for another rig at the same spot and heading. The
    /// payload is deposited on the ground first so the ledger stays exact.
    member this.SwapMachine(rig: MachineRig) =
        match machine with
        | Some m ->
            let position = physics.Simulation.Bodies.[m.Chassis].Pose.Position
            let yaw = m.Yaw

            match soilState with
            | Some state ->
                for i in 0 .. MaterialCount - 1 do
                    if m.BucketLoad.[i] > 0.0 then
                        Soil.deposit state m.BucketTipPosition m.BucketLoad.[i] (Volume.materialOfByte (byte i))
                        m.BucketLoad.[i] <- 0.0
            | None -> ()

            m.Despawn()
            machine <- None
            this.SpawnMachineRig(rig, Vector3(position.X, 0.0f, position.Z), yaw)
        | None -> this.SpawnMachineRig(rig, Vector3(32.0f, 0.0f, 32.0f))

    member this.SpawnMachineRig(rig: MachineRig, position: Vector3) =
        this.SpawnMachineRig(rig, position, 0.0f)

    /// Spawn a rig at `position` (XZ) heading `yaw` radians about +Y.
    member _.SpawnMachineRig(rig: MachineRig, position: Vector3, yaw: float32) =
        let ground = surfaceHeight position.X position.Z
        let spawned = Machine(physics, rig, Vector3(position.X, ground + 0.01f, position.Z), yaw)
        machine <- Some spawned
        spawned

    /// Cut soil waiting on the blade (kg per material) — mass in flight.
    member _.SpillPending = spillPending

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
                        let clumpMass = Tuning.SpillClumpKg

                        while remaining > 0.0 do
                            let mass = min remaining clumpMass
                            remaining <- remaining - mass

                            // The brush throws its spoil up and outward as
                            // a ring around the cut.
                            let angle = Rng.nextFloat32 &rng * System.MathF.Tau
                            let reach = 0.4f + Rng.nextFloat32 &rng * 0.6f
                            let outward = Vector3(System.MathF.Cos angle, 0.0f, System.MathF.Sin angle)

                            let clearY =
                                max (Soil.surfaceHeight state center.X center.Z) (center.Y + radius)

                            let position =
                                Vector3(center.X, clearY + 0.1f + Rng.nextFloat32 &rng * 0.3f, center.Z)
                                + outward * (radius * 0.5f)

                            // Thrown well clear (1.5–3 m): a trench swept
                            // with the brush must not backfill itself.
                            let velocity = outward * (3.0f + reach * 3.0f) + Vector3(0.0f, 3.0f, 0.0f)

                            if not (clumps.TrySpawn(position, velocity, mass, material)) then
                                // Cap overflow: bank it immediately, mass-neutral.
                                Soil.deposit state position mass material

            total

    /// Spawn one clump, banking the mass instead if the pool is full.
    member private _.SpawnClump(position: Vector3, velocity: Vector3, mass: float, material: SoilMaterial) =
        match soilState with
        | Some state ->
            if not (clumps.TrySpawn(position, velocity, mass, material)) then
                Soil.deposit state position mass material
        | None -> ()

    /// Turn pending spill into clumps at the blade: whole shovelfuls while
    /// cutting, everything that's left once the cut stops.
    member private this.FlushSpill(state: SoilState, pieceMass: float, everything: bool) =
        for materialIndex in 0 .. MaterialCount - 1 do
            let mutable pending = spillPending.[materialIndex]

            if pending > 0.0 then
                let material = Volume.materialOfByte (byte materialIndex)

                while pending >= pieceMass do
                    pending <- pending - pieceMass

                    // Spread along the lip and fan a little, so the spoil
                    // lands as a windrow ahead of the blade.
                    let across = (Rng.nextFloat32 &rng - 0.5f) * 2.0f

                    let scatter =
                        Vector3(
                            (Rng.nextFloat32 &rng - 0.5f) * 0.5f,
                            Rng.nextFloat32 &rng * 0.4f,
                            (Rng.nextFloat32 &rng - 0.5f) * 0.5f
                        )

                    this.SpawnClump(
                        spillPosition + spillSpread * across + Vector3(0.0f, Rng.nextFloat32 &rng * 0.15f, 0.0f),
                        spillVelocity + scatter,
                        pieceMass,
                        material
                    )

                if everything && pending > 0.0 then
                    // Sub-clump leftovers bank in the residual rather than
                    // vanishing (the only path that ever leaked mass).
                    if pending > 0.001 then
                        this.SpawnClump(spillPosition, spillVelocity, pending, material)
                    else
                        Soil.creditUnbanked state materialIndex pending

                    pending <- 0.0

                spillPending.[materialIndex] <- pending

    /// The dig system: bucket cutting edge vs the soil volume. Cut what the
    /// moving lip sweeps, absorb into the payload, resist via FEE, spill
    /// whatever the bowl can't hold at its tilt.
    member private this.DigTick() =
        match machine, soilState with
        | Some m, Some state ->
            let scale = m.Rig.Scale
            let edge = m.BucketTipPosition
            let edgeVelocity = m.CuttingEdgeVelocity
            let surface = Soil.surfaceHeight state edge.X edge.Z
            let horizontal = Vector3(edgeVelocity.X, 0.0f, edgeVelocity.Z)
            let moving = horizontal.LengthSquared() > 1e-4f
            let heading = if moving then Vector3.Normalize horizontal else Vector3.Zero

            // Cut depth is measured against the BENCH AHEAD of the edge,
            // not the pothole the edge just carved under itself — the face
            // being sheared is what resists (and what the teeth attack).
            let ahead = edge + heading * (0.3f * scale)

            let surfaceAhead =
                if moving then Soil.surfaceHeight state ahead.X ahead.Z else surface

            let depth = max surface surfaceAhead - edge.Y
            let speed = edgeVelocity.Length()
            let mutable targetForce = Vector3.Zero
            let capacity = m.Rig.BucketCapacityKg
            // Per-tick masses are tuned for the U17's 70 kg bucket.
            let loadScale = capacity / Tuning.BucketCapacityKg
            let pieceMass = Tuning.SpillClumpKg * float (scale * scale)
            let cutting = depth > 0.02f && speed > Tuning.CutMinSpeed

            if cutting then
                // The soil being sheared: sampled in the bench, not in the
                // cell at the edge (which the last tick's cut emptied).
                let bench = if surfaceAhead > surface then ahead else edge

                let struct (matByte, compByte, moistByte) =
                    Soil.sampleBelow state bench.X edge.Y bench.Z

                let props = Tuning.soil (Volume.materialOfByte matByte)

                // Swept cut across the lip's width.
                Array.fill carveScratch 0 MaterialCount 0.0
                let lip = m.LipAxis
                let mutable removed = 0.0

                for i in 0 .. m.CutPoints - 1 do
                    let t =
                        if m.CutPoints = 1 then
                            0.0f
                        else
                            float32 i / float32 (m.CutPoints - 1) * 2.0f - 1.0f

                    removed <- removed + Soil.carveBowl state (edge + lip * (t * m.CutSpan)) m.CutRadius carveScratch

                // Mouth gate: soil shears over the lip into the bowl only
                // when the open face leads the motion (a curl-through cut).
                // Otherwise the shell back-drags: the carve still churns
                // the ground, but everything berms at the edge instead of
                // teleporting into the payload.
                let mouthLeads =
                    Vector3.Dot(edgeVelocity, m.MouthDirection) > speed * Tuning.MouthLeadFraction

                // Soil shears over the lip at a finite rate; one budget
                // across all materials per tick.
                let mutable fillBudget =
                    if mouthLeads then Tuning.BucketFillPerTick * loadScale else 0.0

                if removed > 0.0 then
                    events.Add DigStarted

                    // Un-captured carve gets PUSHED AHEAD of the cut (the
                    // blade's push-mound), not dropped back into it: loose
                    // clumps leave the blade displaced along its motion so
                    // berms build on uncut ground.
                    spillPosition <-
                        Vector3(edge.X, max surface (edge.Y + m.CutRadius) + 0.3f, edge.Z)
                        + heading * (0.5f * scale)

                    spillVelocity <- heading * System.MathF.Min(speed, 1.5f) + Vector3(0.0f, 0.3f, 0.0f)
                    spillSpread <- lip * m.CutSpan

                    for materialIndex in 0 .. MaterialCount - 1 do
                        if carveScratch.[materialIndex] > 0.0 then
                            let absorbed =
                                if fillBudget > 0.0 then
                                    let taken =
                                        m.TryAbsorb(min carveScratch.[materialIndex] fillBudget, materialIndex)

                                    fillBudget <- fillBudget - taken
                                    taken
                                else
                                    0.0

                            spillPending.[materialIndex] <-
                                spillPending.[materialIndex] + carveScratch.[materialIndex] - absorbed

                // FEE resistance opposing the cut direction, plus the
                // surcharge of the heap a filling bucket pushes ahead.
                let fill = float32 (m.BucketLoadKg / capacity)

                let magnitude =
                    Fee.resistanceAt m.CutMaxDepth props compByte moistByte depth m.CutWidth
                    * (1.0f + Tuning.FeeSurchargeFactor * fill)

                targetForce <- -Vector3.Normalize edgeVelocity * magnitude

            this.FlushSpill(state, pieceMass, not cutting)
            feeForce <- Vector3.Lerp(feeForce, targetForce, Tuning.FeeSmoothing)

            if feeForce.LengthSquared() > 1.0f then
                let mutable bucketRef = physics.Simulation.Bodies.[m.Bucket]

                if not bucketRef.Awake then
                    bucketRef.Awake <- true

                bucketRef.ApplyImpulse(feeForce * Tuning.FixedDt, edge - bucketRef.Pose.Position)

            // The shell ploughs loose spoil: a berm or dumped pile is not a
            // wall. Whatever loose soil the bucket body overlaps is pushed
            // off ahead of its motion — so the bucket can sweep, grade and
            // backfill, and never jams against its own windrow.
            let bucketBody = physics.Simulation.Bodies.[m.Bucket]
            let bodyCenter = bucketBody.Pose.Position
            let ploughRadius = Tuning.PloughRadius * scale
            Array.fill carveScratch 0 MaterialCount 0.0
            let mutable ploughed = 0.0

            for i in -1..1 do
                ploughed <-
                    ploughed
                    + Soil.ploughLoose
                        state
                        (bodyCenter + m.LipAxis * (float32 i * 0.2f * scale))
                        ploughRadius
                        carveScratch

            if ploughed > 0.0 then
                let push = Vector3(bucketBody.Velocity.Linear.X, 0.0f, bucketBody.Velocity.Linear.Z)

                let direction =
                    if push.LengthSquared() > 0.01f then
                        Vector3.Normalize push
                    else
                        let angle = Rng.nextFloat32 &rng * System.MathF.Tau
                        Vector3(System.MathF.Cos angle, 0.0f, System.MathF.Sin angle)

                for materialIndex in 0 .. MaterialCount - 1 do
                    let mutable left = carveScratch.[materialIndex]

                    while left > 0.0 do
                        let piece = min left Tuning.SpillClumpKg
                        left <- left - piece
                        let across = (Rng.nextFloat32 &rng - 0.5f) * 0.6f * scale

                        this.SpawnClump(
                            bodyCenter
                            + direction * (ploughRadius + 0.2f)
                            + m.LipAxis * across
                            + Vector3(0.0f, 0.1f + Rng.nextFloat32 &rng * 0.15f, 0.0f),
                            direction * (System.MathF.Min(push.Length(), 2.0f) + 0.6f)
                            + Vector3(0.0f, 0.5f, 0.0f),
                            piece,
                            Volume.materialOfByte (byte materialIndex)
                        )

            // Gravity empties the bowl: whatever it can't hold at its
            // current tilt runs out over the lip — unless the teeth are
            // below the bench, where the bank itself keeps the load in.
            match m.PourTick(depth > 0.02f) with
            | ValueSome(released, materialIndex) ->
                events.Add(SoilDumped(float32 released, byte materialIndex))

                // Dirt slides out with the bucket BODY's motion; the lip's
                // rotational velocity would catapult it skyward.
                let bodyVelocity = physics.Simulation.Bodies.[m.Bucket].Velocity.Linear

                let pourVelocity =
                    Vector3(bodyVelocity.X, System.MathF.Min(bodyVelocity.Y, 0.2f) - 0.4f, bodyVelocity.Z)

                // A curtain across the lip, not one lump: several small
                // packets spread over the bucket's width.
                let material = Volume.materialOfByte (byte materialIndex)
                let pieces = int (System.Math.Ceiling(released / Tuning.PourPieceKg)) |> max 1 |> min 8
                let piece = released / float pieces
                let lip = m.LipAxis
                let mutable left = released

                for pieceIndex in 1..pieces do
                    let mass = if pieceIndex = pieces then left else piece
                    left <- left - mass
                    let across = (Rng.nextFloat32 &rng - 0.5f) * 2.0f * m.CutSpan

                    let scatter =
                        Vector3(
                            (Rng.nextFloat32 &rng - 0.5f) * 0.35f,
                            (Rng.nextFloat32 &rng - 0.5f) * 0.3f,
                            (Rng.nextFloat32 &rng - 0.5f) * 0.35f
                        )

                    this.SpawnClump(
                        edge + lip * across + Vector3(0.0f, -0.05f - Rng.nextFloat32 &rng * 0.1f, 0.0f),
                        pourVelocity + scatter,
                        mass,
                        material
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
              for trackPoint in 0..2 do
                if System.MathF.Abs(m.TrackAxis side) > 0.15f then
                    let point = m.TrackContactPointAt(side, trackPoint)
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

    /// Plant one tree: a static trunk the machine can't drive through.
    member _.AddTree(x: float32, z: float32, height: float32) =
        let ground = surfaceHeight x z
        // The trunk runs well below grade so it still blocks after the
        // ground beside it has been dug away.
        Bepu.addStaticCylinder physics.Simulation (Vector3(x, ground + height * 0.5f - 1.5f, z)) (0.05f * height) (height + 3.0f)
        |> ignore

        treePositions.Add(Vector3(x, ground, z))
        treeHeights.Add height

    member _.TreeCount = treePositions.Count

    /// Dress the job site: spruce on the open ground, loose boulders on
    /// the surface, the site cabin by the yard. No-op without a site.
    member this.SeedScenery() =
        match soilState with
        | Some state when state.Site.IsSome ->
            let site = state.Site.Value

            for _ in 1..260 do
                let x = 3.0f + Rng.nextFloat32 &rng * (site.WorldX - 6.0f)
                let z = 3.0f + Rng.nextFloat32 &rng * (site.WorldZ - 6.0f)
                let height = 3.2f + Rng.nextFloat32 &rng * 3.0f
                let struct (matByte, _) = Soil.surfaceSample state x z
                let material = Volume.materialOfByte matByte
                let mutable clear = treePositions.Count < 72

                for i in 0 .. treePositions.Count - 1 do
                    if Vector3.DistanceSquared(Vector3(x, treePositions.[i].Y, z), treePositions.[i]) < 6.5f then
                        clear <- false

                if
                    clear
                    && (material = Grass || material = Topsoil)
                    && Terrain.isOpenGround site x z
                    && surfaceHeight x z > state.WaterTableHeight + 0.4f
                then
                    this.AddTree(x, z, height)

            for _ in 1..26 do
                let x = 3.0f + Rng.nextFloat32 &rng * (site.WorldX - 6.0f)
                let z = 3.0f + Rng.nextFloat32 &rng * (site.WorldZ - 6.0f)
                let radius = 0.28f + Rng.nextFloat32 &rng * 0.34f

                if Terrain.isOpenGround site x z && rockHandles.Count < 120 then
                    this.AddRock(Vector3(x, surfaceHeight x z + radius * 0.8f, z), radius, true)

            // Site cabin: a 5 × 2.5 × 2.4 m box on the yard's north side.
            Bepu.addStaticBox
                physics.Simulation
                (Vector3(site.CabinX, site.PadHeight + 1.25f, site.CabinZ))
                (Vector3(5.0f, 2.5f, 2.4f))
            |> ignore
        | _ -> ()

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
                Soil.deposit state (clumps.Position 0) clumps.Masses.[0] (Volume.materialOfByte clumps.Materials.[0])
                clumps.RemoveAt 0

            // Soil still on the blade banks where it was cut.
            for i in 0 .. MaterialCount - 1 do
                if spillPending.[i] > 0.0 then
                    Soil.deposit state spillPosition spillPending.[i] (Volume.materialOfByte (byte i))
                    spillPending.[i] <- 0.0

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
            writer.Write "GRAV3"
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
                writer.Write m.Yaw
            | None -> writer.Write false

            writer.Write rockHandles.Count

            for i in 0 .. rockHandles.Count - 1 do
                let position = physics.Simulation.Bodies.[rockHandles.[i]].Pose.Position
                writer.Write position.X
                writer.Write position.Y
                writer.Write position.Z
                writer.Write rockRadii.[i]
                writer.Write rockExposed.[i]

            writer.Write treePositions.Count

            for i in 0 .. treePositions.Count - 1 do
                writer.Write treePositions.[i].X
                writer.Write treePositions.[i].Z
                writer.Write treeHeights.[i]

            match state.Site with
            | ValueSome site ->
                writer.Write true

                for value in
                    [| site.WorldX; site.WorldZ; site.YardX; site.YardZ; site.PadHeight; site.PitMinX
                       site.PitMinZ; site.PitMaxX; site.PitMaxZ; site.PitDepth; site.CabinX; site.CabinZ |] do
                    writer.Write value
            | ValueNone -> writer.Write false

    member this.Step(input: InputFrame) : RenderState =
        events.Clear()

        match machine with
        | Some m ->
            m.Step(input, Tuning.FixedDt, surfaceHeightF)

            if m.StallActive then
                events.Add HydraulicStall

            // 0.38 rad ≈ 22°: genuinely sketchy for a tracked machine —
            // parking on ordinary hillsides (10–15°) is not a warning.
            if m.ChassisTilt > 0.38f then
                events.Add TipWarning

            if m.TrackSlipping then
                events.Add TrackSlip
        | None -> ()

        this.DigTick()
        physics.Step()

        match soilState with
        | Some state ->
            clumps.Step(state, Tuning.FixedDt)
            Soil.settleTick state wallFailures

            // Cohesive wedge failures tumble off the face as clumps — split
            // into shovelful-sized pieces (one 40+ kg body rendered as a
            // fist of floating lumps was the ugliest thing in the game).
            for failure in wallFailures do
                events.Add WallCollapsed

                let mutable remaining =
                    float failure.Units
                    * Volume.massPerOccUnit state.Config failure.MaterialByte failure.CompactionByte

                // The wedge leaves the face over the low side and falls
                // onto the trench floor as loose spoil.
                let size = state.Config.CellSize
                let high = Vector3((float32 failure.X + 0.5f) * size, 0.0f, (float32 failure.Z + 0.5f) * size)
                let low = Vector3((float32 failure.LowX + 0.5f) * size, 0.0f, (float32 failure.LowZ + 0.5f) * size)
                let outward = Vector3.Normalize(low - high)
                let top = state.Heights.[state.ColumnIndex(failure.X, failure.Z)]

                while remaining > 0.0 do
                    let piece = min remaining Tuning.SpillClumpKg
                    remaining <- remaining - piece

                    let jitter =
                        Vector3(
                            (Rng.nextFloat32 &rng - 0.5f) * size,
                            Rng.nextFloat32 &rng * 0.3f,
                            (Rng.nextFloat32 &rng - 0.5f) * size
                        )

                    this.SpawnClump(
                        Vector3(low.X, top + 0.05f, low.Z) + jitter,
                        outward * (0.3f + Rng.nextFloat32 &rng * 0.6f),
                        piece,
                        Volume.materialOfByte failure.MaterialByte
                    )

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
            snapshot.Handles.[i] <- clumps.Ids.[i]
            snapshot.X.[i] <- clumps.PositionsX.[i]
            snapshot.Y.[i] <- clumps.PositionsY.[i]
            snapshot.Z.[i] <- clumps.PositionsZ.[i]
            snapshot.VelocityX.[i] <- clumps.VelocitiesX.[i]
            snapshot.VelocityY.[i] <- clumps.VelocitiesY.[i]
            snapshot.VelocityZ.[i] <- clumps.VelocitiesZ.[i]
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

        // Trees ride the ground they stand on (dig beside one and it sinks
        // with the bank rather than hovering).
        let treeCount = min treePositions.Count snapshot.TreePositions.Length
        snapshot.TreeCount <- treeCount

        for i in 0 .. treeCount - 1 do
            let position = treePositions.[i]
            snapshot.TreePositions.[i] <- Vector3(position.X, surfaceHeight position.X position.Z, position.Z)
            snapshot.TreeHeights.[i] <- treeHeights.[i]

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

        // GRAV2 saves predate heading, trees and the site layout.
        let version =
            match reader.ReadString() with
            | "GRAV3" -> 3
            | "GRAV2" -> 2
            | _ -> failwith "not a Gravemaskin save"

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
            let yaw = if version >= 3 then reader.ReadSingle() else 0.0f

            world.SpawnMachineRig(Tuning.rigByName name, System.Numerics.Vector3(x, 0.0f, z), yaw)
            |> ignore

        let rockCount = reader.ReadInt32()

        for _ in 1..rockCount do
            let x = reader.ReadSingle()
            let y = reader.ReadSingle()
            let z = reader.ReadSingle()
            let radius = reader.ReadSingle()
            let exposed = reader.ReadBoolean()
            world.AddRock(System.Numerics.Vector3(x, y, z), radius, exposed)

        if version >= 3 then
            let treeCount = reader.ReadInt32()

            for _ in 1..treeCount do
                let x = reader.ReadSingle()
                let z = reader.ReadSingle()
                world.AddTree(x, z, reader.ReadSingle())

            if reader.ReadBoolean() then
                let read () = reader.ReadSingle()

                let site =
                    { WorldX = read ()
                      WorldZ = read ()
                      YardX = read ()
                      YardZ = read ()
                      PadHeight = read ()
                      PitMinX = read ()
                      PitMinZ = read ()
                      PitMaxX = read ()
                      PitMaxZ = read ()
                      PitDepth = read ()
                      CabinX = read ()
                      CabinZ = read () }

                state.Site <- ValueSome site

                Bepu.addStaticBox
                    world.Physics.Simulation
                    (System.Numerics.Vector3(site.CabinX, site.PadHeight + 1.25f, site.CabinZ))
                    (System.Numerics.Vector3(5.0f, 2.5f, 2.4f))
                |> ignore

        world

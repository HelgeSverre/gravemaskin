namespace Gravemaskin

open System
open System.Numerics
open BepuPhysics
open BepuPhysics.Constraints
open BepuUtilities
open BepuUtilities.Memory

/// The ONLY module that owns BEPU state. Everything mutable and pooled lives
/// behind this type; Sim.fs drives it, tests inspect it through members.
type Physics(threadCount: int) =
    let pool = new BufferPool()

    let dispatcher =
        if threadCount > 1 then
            new ThreadDispatcher(threadCount)
        else
            null

    let simulation =
        Simulation.Create(
            pool,
            Bepu.NarrowPhaseCallbacks(),
            Bepu.PoseIntegratorCallbacks(Vector3(0.0f, -9.81f, 0.0f)),
            SolveDescription(Tuning.SolverVelocityIterations, Tuning.SolverSubsteps)
        )

    do
        // Same-machine replay/regression determinism with the multithreaded
        // dispatcher: without this, contact/constraint add ordering diverges
        // between identical runs once bodies churn (verified empirically —
        // clump counts differed run-to-run until this was set).
        simulation.Deterministic <- true

    // Soil tile statics: handle + shape per tile, swapped when terrain changes.
    let mutable tileStatics: StaticHandle[] = Array.empty
    let mutable tileShapes: Collidables.TypedIndex[] = Array.empty
    let mutable tilesLive: bool[] = Array.empty

    member _.Simulation = simulation
    member _.Pool = pool

    /// (Re)build the collision mesh for one soil tile, disposing the old one.
    /// Deterministic: called at fixed points in Step, in tile-index order.
    member _.SwapTileMesh(state: SoilState, tile: int) =
        if tileStatics.Length = 0 then
            tileStatics <- Array.zeroCreate (state.TilesX * state.TilesZ)
            tileShapes <- Array.zeroCreate (state.TilesX * state.TilesZ)
            tilesLive <- Array.zeroCreate (state.TilesX * state.TilesZ)

        if tilesLive.[tile] then
            simulation.Statics.Remove(tileStatics.[tile])
            simulation.Shapes.RecursivelyRemoveAndDispose(tileShapes.[tile], pool)

        let mutable mesh =
            SoilCollision.buildTileMesh state pool (tile % state.TilesX) (tile / state.TilesX)

        let shape = simulation.Shapes.Add(&mesh)
        let desc = StaticDescription(Vector3.Zero, shape)
        tileStatics.[tile] <- simulation.Statics.Add(&desc)
        tileShapes.[tile] <- shape
        tilesLive.[tile] <- true

    /// Swap up to `budget` dirty tiles this tick, lowest tile index first.
    member this.SwapDirtyTiles(state: SoilState, budget: int) =
        let mutable swapped = 0
        let mutable tile = 0

        while swapped < budget && tile < state.DirtyMesh.Length do
            if state.DirtyMesh.[tile] then
                state.DirtyMesh.[tile] <- false
                this.SwapTileMesh(state, tile)
                swapped <- swapped + 1

            tile <- tile + 1

        swapped

    member _.Step() =
        if isNull (box dispatcher) then
            simulation.Timestep(Tuning.FixedDt)
        else
            simulation.Timestep(Tuning.FixedDt, dispatcher)

    member _.BodyCount = simulation.Bodies.ActiveSet.Count

    /// Poses of every body in every set, for determinism hashing and tests.
    member _.HashBodyPoses() =
        let mutable hash = 14695981039346656037UL // FNV-1a offset basis

        let mix (value: float32) =
            let bits = uint64 (BitConverter.SingleToUInt32Bits value)
            hash <- (hash ^^^ bits) * 1099511628211UL

        for setIndex in 0 .. simulation.Bodies.Sets.Length - 1 do
            let set = simulation.Bodies.Sets.[setIndex]

            if set.Allocated then
                for bodyIndex in 0 .. set.Count - 1 do
                    let state = set.DynamicsState.[bodyIndex]
                    let pose = state.Motion.Pose
                    mix pose.Position.X
                    mix pose.Position.Y
                    mix pose.Position.Z
                    mix pose.Orientation.X
                    mix pose.Orientation.Y
                    mix pose.Orientation.Z
                    mix pose.Orientation.W

        hash

    interface IDisposable with
        member _.Dispose() =
            simulation.Dispose()

            if not (isNull (box dispatcher)) then
                dispatcher.Dispose()

            pool.Clear()

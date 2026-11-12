namespace Gravemaskin

open System.Numerics

/// THE soil seam. Sim.fs and Excavator/* call only this module — never
/// Volume/Carve/Settle internals — so soil internals can be reorganized (or
/// the whole backend swapped, per the Phase 1 kill-gate pivot) behind one
/// signature.
[<RequireQualifiedAccess>]
module Soil =

    let create (config: SoilConfig) (mat: SoilMaterial) (groundHeight: float32) =
        let state = SoilState(config)
        Volume.fillFlat state mat groundHeight
        state

    let createTerrain (config: SoilConfig) (seed: int) (baseHeight: float32) (relief: float32) =
        let state = SoilState(config)
        Terrain.fill state seed baseHeight relief
        state

    let surfaceHeight (state: SoilState) (x: float32) (z: float32) =
        let config = state.Config
        let cx = int (x / config.CellSize) |> max 0 |> min (config.CellsX - 1)
        let cz = int (z / config.CellSize) |> max 0 |> min (config.CellsZ - 1)
        state.Heights.[state.ColumnIndex(cx, cz)]

    /// Carve a sphere out of the volume; removed mass per material lands in
    /// the caller-owned `removedByMaterial` (kg). Returns total kg removed.
    let carveSphere (state: SoilState) (center: Vector3) (radius: float32) (removedByMaterial: float[]) =
        Carve.sphere state center radius removedByMaterial

    /// The blade's swept cut (see Carve.bowl): accumulates into
    /// `removedByMaterial` without zeroing it. Returns kg removed.
    let carveBowl (state: SoilState) (center: Vector3) (radius: float32) (removedByMaterial: float[]) =
        Carve.bowl state center radius removedByMaterial

    /// Loose spoil under a sphere (the bucket shell), removed down to the
    /// sphere's underside; bank soil is untouched. Accumulates like
    /// `carveBowl`. Returns kg removed.
    let ploughLoose (state: SoilState) (center: Vector3) (radius: float32) (removedByMaterial: float[]) =
        Carve.looseBowl state center radius removedByMaterial

    /// Material, compaction and moisture of the soil the cutting edge is
    /// actually in: the first occupied cell at or below `y` in that column.
    /// (The cell AT the edge is usually the one the last tick carved empty.)
    let sampleBelow (state: SoilState) (x: float32) (y: float32) (z: float32) =
        let config = state.Config
        let cx = int (x / config.CellSize) |> max 0 |> min (config.CellsX - 1)
        let cz = int (z / config.CellSize) |> max 0 |> min (config.CellsZ - 1)
        let mutable cy = int (y / config.CellSize) |> max 0 |> min (config.CellsY - 1)

        while cy > 0 && state.Occupancy.[state.Index(cx, cy, cz)] = 0uy do
            cy <- cy - 1

        let index = state.Index(cx, cy, cz)
        struct (state.Material.[index], state.Compaction.[index], state.Moisture.[index])

    /// Deposit loose mass (kg) at a world position's column.
    let deposit (state: SoilState) (position: Vector3) (mass: float) (mat: SoilMaterial) =
        let config = state.Config
        let x = int (position.X / config.CellSize) |> max 0 |> min (config.CellsX - 1)
        let z = int (position.Z / config.CellSize) |> max 0 |> min (config.CellsZ - 1)
        Volume.deposit state x z mass mat

    /// Deposit mass that did NOT come from this world's soil (scenario
    /// setup, delivered fill): banks it AND credits the ledger, unlike
    /// `deposit`, which is strictly for recycling carved mass.
    let injectLoose (state: SoilState) (position: Vector3) (mass: float) (mat: SoilMaterial) =
        state.Ledger.[int (Volume.byteOfMaterial mat)] <-
            state.Ledger.[int (Volume.byteOfMaterial mat)] + mass

        deposit state position mass mat

    /// Material and moisture of the top cell at a world position — what the
    /// grain layer samples to color and wet its spray.
    let surfaceSample (state: SoilState) (x: float32) (z: float32) =
        let config = state.Config
        let cx = int (x / config.CellSize) |> max 0 |> min (config.CellsX - 1)
        let cz = int (z / config.CellSize) |> max 0 |> min (config.CellsZ - 1)
        let mutable y = config.CellsY - 1

        while y > 0 && state.Occupancy.[state.Index(cx, y, cz)] = 0uy do
            y <- y - 1

        let index = state.Index(cx, y, cz)
        struct (state.Material.[index], state.Moisture.[index])

    /// Bank a small mass directly into the quantization residual (used for
    /// amounts too small to be worth a clump — never dropped).
    let creditUnbanked (state: SoilState) (materialIndex: int) (mass: float) =
        state.Unbanked.[materialIndex] <- state.Unbanked.[materialIndex] + mass

    /// Budgeted settling for this tick; cohesive wall failures are recorded
    /// into `failures` for the caller to turn into clumps.
    let settleTick (state: SoilState) (failures: ResizeArray<WallFailure>) =
        Settle.tick state failures |> ignore

    /// Volume mass per material by full scan, plus unbanked residual —
    /// compare against Ledger + live clump mass in tests.
    let massTotals (state: SoilState) =
        let totals = Volume.scanMassByMaterial state

        for i in 0 .. totals.Length - 1 do
            totals.[i] <- totals.[i] + state.Unbanked.[i]

        totals

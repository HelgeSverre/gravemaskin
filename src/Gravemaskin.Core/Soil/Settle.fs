namespace Gravemaskin

open System

/// One recorded cohesive-wall failure: the caller (World) turns it into
/// tumbling clumps + a WallCollapsed event.
[<Struct>]
type WallFailure =
    { X: int
      Z: int
      /// The low column the wedge falls toward.
      LowX: int
      LowZ: int
      Units: int
      MaterialByte: byte
      CompactionByte: byte }

/// Soil settling: a budgeted cellular pass over dirty tiles in index order
/// (deterministic), relaxing each column against all eight neighbours:
///  - loose material and cohesionless bank (sand, gravel) flow toward
///    their angle of repose;
///  - cohesive bank holds until the face exceeds the critical height
///    h_crit ≈ 4c/γ, then fails as a wedge (the trench-wall collapse).
[<RequireQualifiedAccess>]
module Settle =

    /// Tiles relaxed per tick. Amortized: a big collapse takes a few ticks to
    /// finish flowing, which also happens to look right.
    [<Literal>]
    let TileBudget = 16

    /// Compaction at/above this is "bank": rules by cohesion, not repose.
    [<Literal>]
    let LooseThreshold = 128uy

    /// Cells' worth of occupancy shed per wedge failure event.
    [<Literal>]
    let WedgeUnits = 510

    /// h_crit ≈ 4c/γ (c in kPa → Pa; γ = ρg), with c at the cell's moisture.
    let criticalHeightAt (props: SoilProperties) (moisture: byte) =
        let c = Moisture.effectiveCohesion props moisture

        if c <= 0.01f then
            0.0f
        else
            4.0f * c * 1000.0f / (float32 props.BankDensity * 9.81f)

    /// Dry-cell convenience (kept for tests/back-compat).
    let criticalHeight (props: SoilProperties) = criticalHeightAt props 0uy

    /// Slope (rise per run) a loose heap of this soil stands at: friction
    /// plus a little from cohesion — clay spoil lies in clods that stack
    /// steeper than its low friction angle alone would allow.
    let reposeSlope (props: SoilProperties) =
        MathF.Min(MathF.Tan props.FrictionAngle + MathF.Min(0.05f * float32 props.Cohesion, 0.35f), 1.0f)

    /// Smallest loose flow worth moving (occupancy units ≈ mm): below it a
    /// slope counts as at rest, so piles stop instead of creeping forever.
    [<Literal>]
    let MinFlowUnits = 3

    /// Relax one neighbour pair `distance` cells apart (1 or √2). Using all
    /// eight neighbours with the true distance makes piles round cones — the
    /// old four-neighbour rule built diamonds.
    let private relaxPair
        (state: SoilState)
        (failures: ResizeArray<WallFailure>)
        (xa: int)
        (za: int)
        (xb: int)
        (zb: int)
        (distance: float32)
        =
        let config = state.Config
        let ha = state.Heights.[state.ColumnIndex(xa, za)]
        let hb = state.Heights.[state.ColumnIndex(xb, zb)]

        let struct (hiX, hiZ, loX, loZ, diff) =
            if ha >= hb then
                struct (xa, za, xb, zb, ha - hb)
            else
                struct (xb, zb, xa, za, hb - ha)

        if diff > config.CellSize * 0.2f then
            // Peek the high column's top cell.
            let mutable y = config.CellsY - 1

            while y >= 0 && state.Occupancy.[state.Index(hiX, y, hiZ)] = 0uy do
                y <- y - 1

            if y >= 0 then
                let index = state.Index(hiX, y, hiZ)
                let matByte = state.Material.[index]
                let compByte = state.Compaction.[index]
                let moistByte = state.Moisture.[index]
                let props = Tuning.soil (Volume.materialOfByte matByte)
                let holds = criticalHeightAt props moistByte

                if compByte < LooseThreshold || holds <= 0.0f then
                    // Loose soil (and cohesionless bank: dry sand, gravel)
                    // flows toward its angle of repose.
                    let critical = reposeSlope props * config.CellSize * distance

                    if diff > critical then
                        // Move a third of the excess: with eight neighbours
                        // relaxed in sequence this converges without
                        // overshoot.
                        let excessUnits = int ((diff - critical) / config.CellSize * 255.0f / 3.0f)

                        if excessUnits >= MinFlowUnits then
                            let struct (units, takenMat, takenComp) =
                                Volume.takeTop state hiX hiZ excessUnits

                            if units > 0 then
                                Volume.putUnits state loX loZ units takenMat takenComp
                                state.MarkDirty(hiX, hiZ)
                elif diff > holds + config.CellSize then
                    // Cohesive bank over-steep beyond what cohesion holds:
                    // the face fails as a wedge that falls to the low side
                    // (recorded here, thrown by World).
                    let struct (units, takenMat, takenComp) =
                        Volume.takeTop state hiX hiZ WedgeUnits

                    if units > 0 then
                        failures.Add
                            { X = hiX
                              Z = hiZ
                              LowX = loX
                              LowZ = loZ
                              Units = units
                              MaterialByte = takenMat
                              CompactionByte = takenComp }

                        state.MarkDirty(hiX, hiZ)
                        state.MarkDirty(loX, loZ)

    let private diagonal = 1.41421356f

    /// One budgeted settling pass; wall failures land in `failures`.
    let tick (state: SoilState) (failures: ResizeArray<WallFailure>) =
        let config = state.Config
        let mutable processed = 0
        let mutable tile = 0

        while processed < TileBudget && tile < state.DirtySettle.Length do
            if state.DirtySettle.[tile] then
                state.DirtySettle.[tile] <- false
                processed <- processed + 1
                let tileX = tile % state.TilesX
                let tileZ = tile / state.TilesX
                // Start one column early so pairs across the tile's left/top
                // boundary are relaxed too — otherwise a pile hugging a tile
                // edge only ever spreads rightward/downward.
                let x0 = max 0 (tileX * SoilConfig.TileSize - 1)
                let z0 = max 0 (tileZ * SoilConfig.TileSize - 1)
                let x1 = min (tileX * SoilConfig.TileSize + SoilConfig.TileSize) config.CellsX
                let z1 = min (tileZ * SoilConfig.TileSize + SoilConfig.TileSize) config.CellsZ

                for z in z0 .. z1 - 1 do
                    for x in x0 .. x1 - 1 do
                        if x + 1 < config.CellsX then
                            relaxPair state failures x z (x + 1) z 1.0f

                        if z + 1 < config.CellsZ then
                            relaxPair state failures x z x (z + 1) 1.0f

                            if x + 1 < config.CellsX then
                                relaxPair state failures x z (x + 1) (z + 1) diagonal

                            if x > 0 then
                                relaxPair state failures x z (x - 1) (z + 1) diagonal

            tile <- tile + 1

        processed

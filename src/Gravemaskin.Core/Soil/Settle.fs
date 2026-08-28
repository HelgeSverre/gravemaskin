namespace Gravemaskin

open System

/// One recorded cohesive-wall failure: the caller (World) turns it into
/// tumbling clumps + a WallCollapsed event.
[<Struct>]
type WallFailure =
    { X: int
      Z: int
      Units: int
      MaterialByte: byte
      CompactionByte: byte }

/// Soil settling: a budgeted cellular pass over dirty tiles in index order
/// (deterministic). Two regimes per neighbor pair:
///  - loose material (low compaction) flows toward its angle of repose;
///  - bank material holds until the face exceeds the cohesive critical
///    height h_crit ≈ 4c/γ, then fails as a wedge → clumps (the trench-wall
///    collapse). Cohesionless bank (sand, gravel) just relaxes by repose.
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

    /// h_crit ≈ 4c/γ (c in kPa → Pa; γ = ρg).
    let criticalHeight (props: SoilProperties) =
        if props.Cohesion <= 0.01f<kPa> then
            0.0f
        else
            4.0f * float32 props.Cohesion * 1000.0f
            / (float32 props.BankDensity * 9.81f)

    let private relaxPair
        (state: SoilState)
        (failures: ResizeArray<WallFailure>)
        (xa: int)
        (za: int)
        (xb: int)
        (zb: int)
        =
        let config = state.Config
        let ha = state.Heights.[state.ColumnIndex(xa, za)]
        let hb = state.Heights.[state.ColumnIndex(xb, zb)]

        let struct (hiX, hiZ, loX, loZ, diff) =
            if ha >= hb then
                struct (xa, za, xb, zb, ha - hb)
            else
                struct (xb, zb, xa, za, hb - ha)

        if diff > config.CellSize * 0.5f then
            // Peek the high column's top cell.
            let mutable y = config.CellsY - 1

            while y >= 0 && state.Occupancy.[state.Index(hiX, y, hiZ)] = 0uy do
                y <- y - 1

            if y >= 0 then
                let index = state.Index(hiX, y, hiZ)
                let matByte = state.Material.[index]
                let compByte = state.Compaction.[index]
                let props = Tuning.soil (Volume.materialOfByte matByte)

                if compByte < LooseThreshold then
                    // Loose: angle-of-repose flow toward the low column.
                    let critical = MathF.Tan props.FrictionAngle * config.CellSize

                    if diff > critical then
                        let excessUnits = int ((diff - critical) / config.CellSize * 255.0f / 2.0f)

                        if excessUnits > 0 then
                            let struct (units, takenMat, takenComp) =
                                Volume.takeTop state hiX hiZ excessUnits

                            if units > 0 then
                                Volume.putUnits state loX loZ units takenMat takenComp
                                state.MarkDirty(hiX, hiZ)
                elif diff > criticalHeight props + config.CellSize then
                    // Bank: the face is over-steep beyond what cohesion can
                    // hold. Cohesionless bank just flows; cohesive bank
                    // fails as a whole wedge → clumps (recorded, spawned by
                    // World).
                    if criticalHeight props <= 0.0f then
                        let struct (units, takenMat, takenComp) =
                            Volume.takeTop state hiX hiZ (WedgeUnits / 2)

                        if units > 0 then
                            Volume.putUnits state loX loZ units takenMat takenComp
                            state.MarkDirty(hiX, hiZ)
                    else
                        let struct (units, takenMat, takenComp) =
                            Volume.takeTop state hiX hiZ WedgeUnits

                        if units > 0 then
                            failures.Add
                                { X = hiX
                                  Z = hiZ
                                  Units = units
                                  MaterialByte = takenMat
                                  CompactionByte = takenComp }

                            state.MarkDirty(hiX, hiZ)
                            state.MarkDirty(loX, loZ)

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
                            relaxPair state failures x z (x + 1) z

                        if z + 1 < config.CellsZ then
                            relaxPair state failures x z x (z + 1)

            tile <- tile + 1

        processed

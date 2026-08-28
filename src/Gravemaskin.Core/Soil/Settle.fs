namespace Gravemaskin

open System

/// Angle-of-repose settling: a budgeted cellular pass over dirty tiles in
/// index order (deterministic). Loose material (low compaction) flows toward
/// its material's repose slope; bank material holds (cohesion failure is a
/// Phase 6 feature).
[<RequireQualifiedAccess>]
module Settle =

    /// Tiles relaxed per tick. Amortized: a big collapse takes a few ticks to
    /// finish flowing, which also happens to look right.
    [<Literal>]
    let TileBudget = 16

    /// Compaction at/above this holds its slope regardless of repose (it is
    /// "bank enough"); below it, the CA may move it.
    [<Literal>]
    let LooseThreshold = 128uy

    let private relaxPair (state: SoilState) (xa: int) (za: int) (xb: int) (zb: int) =
        let config = state.Config
        let ha = state.Heights.[state.ColumnIndex(xa, za)]
        let hb = state.Heights.[state.ColumnIndex(xb, zb)]
        let struct (hiX, hiZ, loX, loZ, diff) =
            if ha >= hb then
                struct (xa, za, xb, zb, ha - hb)
            else
                struct (xb, zb, xa, za, hb - ha)

        if diff > config.CellSize * 0.5f then
            // Peek the high column's top material to get its repose slope.
            let mutable y = config.CellsY - 1

            while y >= 0 && state.Occupancy.[state.Index(hiX, y, hiZ)] = 0uy do
                y <- y - 1

            if y >= 0 then
                let index = state.Index(hiX, y, hiZ)

                if state.Compaction.[index] < LooseThreshold then
                    let props = Tuning.soil (Volume.materialOfByte state.Material.[index])
                    let critical = MathF.Tan props.FrictionAngle * config.CellSize

                    if diff > critical then
                        // Move half the excess, capped at the top cell.
                        let excessUnits =
                            int ((diff - critical) / config.CellSize * 255.0f / 2.0f)

                        if excessUnits > 0 then
                            let struct (units, matByte, compByte) =
                                Volume.takeTop state hiX hiZ excessUnits

                            if units > 0 then
                                Volume.putUnits state loX loZ units matByte compByte
                                state.MarkDirty(hiX, hiZ)
                                true
                            else
                                false
                        else
                            false
                    else
                        false
                else
                    false
            else
                false
        else
            false

    /// One budgeted settling pass. Returns the number of tiles processed.
    let tick (state: SoilState) =
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
                            relaxPair state x z (x + 1) z |> ignore

                        if z + 1 < config.CellsZ then
                            relaxPair state x z x (z + 1) |> ignore

            tile <- tile + 1

        processed

namespace Gravemaskin

open System

/// Per-cell moisture (0 dry … 255 saturated), modeled as a property of the
/// GROUND, not of transported soil: a water table keeps deep cells wet,
/// moisture wicks upward/laterally, and the surface dries out. Digging into
/// the water table therefore finds wet soil, and spoil piles dry from the
/// top — without moving any ledgered mass around.
[<RequireQualifiedAccess>]
module Moisture =

    /// Columns processed per tick (rotating cursor — deterministic).
    [<Literal>]
    let ColumnBudget = 512

    /// Effective cohesion (kPa) at a moisture level. Two real effects:
    ///  - capillary bridges give near-cohesionless sand apparent cohesion
    ///    that peaks damp and vanishes both dry and saturated (sandcastles);
    ///  - genuinely cohesive soils (clay) WEAKEN as they saturate.
    let effectiveCohesion (props: SoilProperties) (moisture: byte) =
        let m = float32 moisture / 255.0f
        let baseC = float32 props.Cohesion

        if baseC < 1.0f then
            // Sand-like: bell-shaped capillary cohesion, up to ~3 kPa damp.
            baseC + 3.0f * 4.0f * m * (1.0f - m)
        else
            // Cohesive: strength falls toward ~40% when saturated.
            baseC * (1.0f - 0.6f * m)

    /// One budgeted pass: `cursor` rotates through columns. For each column:
    /// water-table cells stay saturated, moisture wicks up toward drier
    /// cells above, and the surface cell loses a step to evaporation.
    let tick (state: SoilState) (waterTableHeight: float32) (cursor: int) =
        let config = state.Config
        let columns = config.CellsX * config.CellsZ
        let tableCells = int (waterTableHeight / config.CellSize)

        for offset in 0 .. ColumnBudget - 1 do
            let column = (cursor + offset) % columns
            let x = column % config.CellsX
            let z = column / config.CellsX
            let mutable top = -1

            for y in config.CellsY - 1 .. -1 .. 0 do
                if top < 0 && state.Occupancy.[state.Index(x, y, z)] > 0uy then
                    top <- y

            if top >= 0 then
                // Water table: saturate everything at/below it.
                for y in 0 .. min top (tableCells - 1) do
                    state.Moisture.[state.Index(x, y, z)] <- 255uy

                // Capillary wicking: each cell pulls moisture from the
                // wetter cell below (slow: one step per visit).
                for y in max 1 tableCells .. top do
                    let index = state.Index(x, y, z)
                    let below = state.Moisture.[state.Index(x, y - 1, z)]
                    let current = state.Moisture.[index]

                    if below > current + 24uy then
                        state.Moisture.[index] <- current + 8uy

                // Surface evaporation.
                let surfaceIndex = state.Index(x, top, z)

                if top >= tableCells && state.Moisture.[surfaceIndex] > 0uy then
                    state.Moisture.[surfaceIndex] <- state.Moisture.[surfaceIndex] - 4uy

        (cursor + ColumnBudget) % columns

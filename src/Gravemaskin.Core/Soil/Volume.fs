namespace Gravemaskin

open System

/// The undisturbed-soil volume: flat SoA byte arrays over the whole world.
/// ponytail: no 32³ chunk objects — flat arrays + dirty XZ tiles are enough at
/// current world sizes (~34 MB for 256×128×256); revisit if worlds grow.
///
/// Column model (Phase 1): soil in a column is contiguous from y=0 — no
/// overhangs. Carving mid-column compacts the column (material above the cut
/// falls down instantly). ponytail: real overhangs arrive with Phase 6
/// cohesion failure.
type SoilConfig =
    { /// Cell edge length in meters.
      CellSize: float32
      CellsX: int
      CellsY: int
      CellsZ: int }

[<RequireQualifiedAccess>]
module SoilConfig =
    /// Collision/render tile: 32×32 columns per tile.
    [<Literal>]
    let TileSize = 32

type SoilState(config: SoilConfig) =
    let cells = config.CellsX * config.CellsY * config.CellsZ
    let columns = config.CellsX * config.CellsZ
    // Occupancy: 0..255 volume fraction of the cell. THE ledgered quantity is
    // mass = occ/255 × cellVolume × density(material, compaction).
    let occupancy = Array.zeroCreate<byte> cells
    let material = Array.zeroCreate<byte> cells
    // 255 = bank (undisturbed) density, 0 = freshly deposited loose.
    let compaction = Array.zeroCreate<byte> cells
    // Surface height per column (meters), maintained incrementally.
    let heights = Array.zeroCreate<float32> columns
    // Per-material mass ledger (kg, f64): volume mass + live clump mass +
    // unbanked residual must always equal this. Set at fill; carve/deposit are
    // ledger-neutral by construction. Asserted by tests.
    let ledger = Array.zeroCreate<float> 5
    // Deposit quantization residual (< one occupancy unit per material):
    // occupancy bytes can't represent arbitrary masses, so the remainder
    // waits here and folds into the next deposit. Counted by conservation.
    let unbanked = Array.zeroCreate<float> 5
    // Dirty XZ tiles (TileSize² columns each): settle CA and collision
    // remesh each keep their own flags, processed in index order so the
    // budgeted work stays deterministic.
    let tilesX = (config.CellsX + SoilConfig.TileSize - 1) / SoilConfig.TileSize
    let tilesZ = (config.CellsZ + SoilConfig.TileSize - 1) / SoilConfig.TileSize
    let dirtySettle = Array.zeroCreate<bool> (tilesX * tilesZ)
    let dirtyMesh = Array.zeroCreate<bool> (tilesX * tilesZ)
    // Renderer's own dirty flags (cleared by the shell, not by physics —
    // collision swaps and render rebuilds run on different budgets).
    let dirtyRender = Array.zeroCreate<bool> (tilesX * tilesZ)

    member _.Config = config
    member _.Occupancy = occupancy
    member _.Material = material
    member _.Compaction = compaction
    member _.Heights = heights
    member _.Ledger = ledger
    member _.Unbanked = unbanked
    member _.TilesX = tilesX
    member _.TilesZ = tilesZ
    member _.DirtySettle = dirtySettle
    member _.DirtyMesh = dirtyMesh
    member _.DirtyRender = dirtyRender

    member _.Index(x: int, y: int, z: int) =
        (y * config.CellsZ + z) * config.CellsX + x

    member _.ColumnIndex(x: int, z: int) = z * config.CellsX + x

    member _.TileIndex(x: int, z: int) =
        (z / SoilConfig.TileSize) * tilesX + (x / SoilConfig.TileSize)

    member this.MarkDirty(x: int, z: int) =
        let tile = this.TileIndex(x, z)
        dirtySettle.[tile] <- true
        dirtyMesh.[tile] <- true
        dirtyRender.[tile] <- true

[<RequireQualifiedAccess>]
module Volume =

    let materialOfByte (value: byte) : SoilMaterial =
        match int value with
        | 0 -> Topsoil
        | 1 -> DrySand
        | 2 -> WetSand
        | 3 -> Gravel
        | _ -> Clay

    let byteOfMaterial (value: SoilMaterial) : byte =
        match value with
        | Topsoil -> 0uy
        | DrySand -> 1uy
        | WetSand -> 2uy
        | Gravel -> 3uy
        | Clay -> 4uy

    /// Loose (freshly excavated) density: bank / (1 + swell).
    let looseDensity (props: SoilProperties) =
        float32 props.BankDensity / (1.0f + props.Swell)

    /// Density as a function of compaction byte: linear from loose to bank.
    let density (props: SoilProperties) (compaction: byte) =
        let loose = looseDensity props
        loose + (float32 props.BankDensity - loose) * (float32 compaction / 255.0f)

    let cellVolume (config: SoilConfig) =
        config.CellSize * config.CellSize * config.CellSize

    /// Mass of one cell in kg.
    let cellMass (config: SoilConfig) (occ: byte) (mat: byte) (comp: byte) =
        let props = Tuning.soil (materialOfByte mat)
        float32 occ / 255.0f * cellVolume config * density props comp

    /// Recompute one column's surface height (contiguous-fill model: height =
    /// filled cells summed as occupancy fractions from the bottom).
    let refreshColumnHeight (state: SoilState) (x: int) (z: int) =
        let config = state.Config
        let mutable height = 0.0f

        for y in 0 .. config.CellsY - 1 do
            let occ = state.Occupancy.[state.Index(x, y, z)]

            if occ > 0uy then
                height <- height + float32 occ / 255.0f * config.CellSize

        state.Heights.[state.ColumnIndex(x, z)] <- height

    /// Re-pack a column so occupancy is contiguous from y=0 (no gaps), then
    /// refresh its height. Mass-neutral by construction: cells move, they
    /// don't change.
    let compactColumn (state: SoilState) (x: int) (z: int) =
        let config = state.Config
        let mutable write = 0

        for y in 0 .. config.CellsY - 1 do
            let index = state.Index(x, y, z)
            let occ = state.Occupancy.[index]

            if occ > 0uy then
                if write <> y then
                    let target = state.Index(x, write, z)
                    state.Occupancy.[target] <- occ
                    state.Material.[target] <- state.Material.[index]
                    state.Compaction.[target] <- state.Compaction.[index]
                    state.Occupancy.[index] <- 0uy

                write <- write + 1

        refreshColumnHeight state x z

    /// Total volume mass per material by full scan (kg) — test/debug only.
    let scanMassByMaterial (state: SoilState) =
        let config = state.Config
        let totals = Array.zeroCreate<float> 5

        for i in 0 .. state.Occupancy.Length - 1 do
            let occ = state.Occupancy.[i]

            if occ > 0uy then
                totals.[int state.Material.[i]] <-
                    totals.[int state.Material.[i]]
                    + float (cellMass config occ state.Material.[i] state.Compaction.[i])

        totals

    /// Deposit loose mass (kg) onto a column, folding in any unbanked
    /// residual for the material. Whatever the occupancy quantization can't
    /// express stays in Unbanked — ledger-neutral either way.
    let deposit (state: SoilState) (x: int) (z: int) (mass: float) (mat: SoilMaterial) =
        let config = state.Config
        let matByte = byteOfMaterial mat
        let props = Tuning.soil mat
        let massPerOccUnit = float (cellVolume config * looseDensity props) / 255.0
        let pool = state.Unbanked.[int matByte] + mass
        let mutable occUnits = int (pool / massPerOccUnit)
        state.Unbanked.[int matByte] <- pool - float occUnits * massPerOccUnit

        if occUnits > 0 then
            // Find the first cell that can take loose material: top up a
            // partial cell only when material AND compaction match exactly
            // (anything else changes mass mid-cell); otherwise start above.
            let mutable y = 0

            while y < config.CellsY && state.Occupancy.[state.Index(x, y, z)] = 255uy do
                y <- y + 1

            while occUnits > 0 && y < config.CellsY do
                let index = state.Index(x, y, z)
                let occ = state.Occupancy.[index]

                if occ = 0uy then
                    let take = min occUnits 255
                    state.Occupancy.[index] <- byte take
                    state.Material.[index] <- matByte
                    state.Compaction.[index] <- 0uy
                    occUnits <- occUnits - take
                elif state.Material.[index] = matByte && state.Compaction.[index] = 0uy then
                    let take = min occUnits (255 - int occ)
                    state.Occupancy.[index] <- byte (int occ + take)
                    occUnits <- occUnits - take
                else
                    // Incompatible partial cell below the surface: skip above.
                    ()

                y <- y + 1

            // Column full to the sky: the leftover goes back to Unbanked
            // rather than vanishing.
            if occUnits > 0 then
                state.Unbanked.[int matByte] <- state.Unbanked.[int matByte] + float occUnits * massPerOccUnit

            refreshColumnHeight state x z
            state.MarkDirty(x, z)

    /// Mass of one occupancy unit (1/255 of a cell) for given material/compaction.
    let massPerOccUnit (config: SoilConfig) (matByte: byte) (compByte: byte) =
        let props = Tuning.soil (materialOfByte matByte)
        float (cellVolume config * density props compByte) / 255.0

    /// Take up to `maxUnits` occupancy units off the top of a column.
    /// Returns struct(units, matByte, compByte); units = 0 if the column is empty.
    let takeTop (state: SoilState) (x: int) (z: int) (maxUnits: int) =
        let config = state.Config
        let mutable y = config.CellsY - 1

        while y >= 0 && state.Occupancy.[state.Index(x, y, z)] = 0uy do
            y <- y - 1

        if y < 0 then
            struct (0, 0uy, 0uy)
        else
            let index = state.Index(x, y, z)
            let occ = int state.Occupancy.[index]
            let take = min occ maxUnits
            state.Occupancy.[index] <- byte (occ - take)

            if occ - take = 0 then
                refreshColumnHeight state x z
            else
                state.Heights.[state.ColumnIndex(x, z)] <-
                    state.Heights.[state.ColumnIndex(x, z)]
                    - float32 take / 255.0f * config.CellSize

            struct (take, state.Material.[index], state.Compaction.[index])

    /// Put occupancy units of explicit (material, compaction) onto a column,
    /// merging only into an exactly-matching partial cell. Column-overflow
    /// converts to Unbanked mass — never lost.
    let putUnits (state: SoilState) (x: int) (z: int) (units: int) (matByte: byte) (compByte: byte) =
        let config = state.Config
        let mutable remaining = units
        let mutable y = 0

        while remaining > 0 && y < config.CellsY do
            let index = state.Index(x, y, z)
            let occ = int state.Occupancy.[index]

            if occ = 0 then
                let take = min remaining 255
                state.Occupancy.[index] <- byte take
                state.Material.[index] <- matByte
                state.Compaction.[index] <- compByte
                remaining <- remaining - take
            elif occ < 255 && state.Material.[index] = matByte && state.Compaction.[index] = compByte then
                let take = min remaining (255 - occ)
                state.Occupancy.[index] <- byte (occ + take)
                remaining <- remaining - take

            y <- y + 1

        if remaining > 0 then
            state.Unbanked.[int matByte] <-
                state.Unbanked.[int matByte]
                + float remaining * massPerOccUnit config matByte compByte

        refreshColumnHeight state x z
        state.MarkDirty(x, z)

    /// Noise-seeded terrain: rolling height, clay under topsoil, dry-sand
    /// patches. Initializes heights + ledger.
    let fillTerrain (state: SoilState) (seed: int) (baseHeight: float32) (relief: float32) =
        let config = state.Config

        for z in 0 .. config.CellsZ - 1 do
            for x in 0 .. config.CellsX - 1 do
                let point =
                    System.Numerics.Vector2(float32 x * 0.045f, float32 z * 0.045f)

                let height =
                    baseHeight + (Noise.fbm2 seed 4 point - 0.5f) * 2.0f * relief

                let sandiness = Noise.fbm2 (seed + 7919) 3 (point * 0.6f)
                let fullCells = max 1 (int (height / config.CellSize))

                for y in 0 .. min (fullCells - 1) (config.CellsY - 1) do
                    let index = state.Index(x, y, z)
                    let depthFromTop = float32 (fullCells - 1 - y) * config.CellSize

                    let mat =
                        if depthFromTop > 0.6f then Clay
                        elif sandiness > 0.62f then DrySand
                        else Topsoil

                    state.Occupancy.[index] <- 255uy
                    state.Material.[index] <- byteOfMaterial mat
                    state.Compaction.[index] <- 255uy

                refreshColumnHeight state x z

        let masses = scanMassByMaterial state
        Array.blit masses 0 state.Ledger 0 5

    /// Fill the volume as flat bank-density terrain of one material up to
    /// `groundHeight` meters, and initialize heights + ledger.
    let fillFlat (state: SoilState) (mat: SoilMaterial) (groundHeight: float32) =
        let config = state.Config
        let matByte = byteOfMaterial mat
        let fullCells = int (groundHeight / config.CellSize)

        for z in 0 .. config.CellsZ - 1 do
            for x in 0 .. config.CellsX - 1 do
                for y in 0 .. min (fullCells - 1) (config.CellsY - 1) do
                    let index = state.Index(x, y, z)
                    state.Occupancy.[index] <- 255uy
                    state.Material.[index] <- matByte
                    state.Compaction.[index] <- 255uy

                refreshColumnHeight state x z

        let masses = scanMassByMaterial state
        Array.blit masses 0 state.Ledger 0 5

namespace Gravemaskin

open System
open System.Numerics

/// The sandbox generator: rolling noise terrain with a job site laid over
/// it — a graded gravel yard and access road, a staked foundation pit, loose
/// stockpiles to load from, a knoll, an old gravel pit, and a creek cut
/// below the water table. Strata run topsoil → sand/gravel lenses → clay,
/// so a deep cut shows its layers.
///
/// Worlds too small to hold the site (tests) get the plain rolling terrain.
[<RequireQualifiedAccess>]
module Terrain =

    let inline private smooth (t: float32) =
        let c = Math.Clamp(t, 0.0f, 1.0f)
        c * c * (3.0f - 2.0f * c)

    /// Smallest world edge (m) the site layout fits in.
    let MinSiteWorld = 56.0f

    /// Yard pad: rounded box around the spawn, long toward the pit (+X).
    let private padHalfX = 11.5f
    let private padHalfZ = 7.5f
    let private padCenterOffsetX = 4.5f
    let private roadHalfWidth = 2.2f
    /// Stockpile cones: radius and repose slope (rise per meter).
    let private pileHeight = 1.3f
    let private pileSlope = 0.58f

    let layout (config: SoilConfig) (baseHeight: float32) : SiteLayout =
        let worldX = float32 config.CellsX * config.CellSize
        let worldZ = float32 config.CellsZ * config.CellSize
        let yardX = worldX * 0.5f
        let yardZ = worldZ * 0.5f

        { WorldX = worldX
          WorldZ = worldZ
          YardX = yardX
          YardZ = yardZ
          PadHeight = baseHeight + 0.1f
          PitMinX = yardX + 8.0f
          PitMinZ = yardZ - 3.0f
          PitMaxX = yardX + 14.0f
          PitMaxZ = yardZ + 1.0f
          PitDepth = 1.25f
          CabinX = yardX - 3.0f
          CabinZ = yardZ - 5.5f }

    /// Signed distance (m) to the yard pad outline; negative inside.
    let padDistance (site: SiteLayout) (x: float32) (z: float32) =
        let qx = MathF.Abs(x - (site.YardX + padCenterOffsetX)) - padHalfX
        let qz = MathF.Abs(z - site.YardZ) - padHalfZ
        let outside = MathF.Sqrt(MathF.Max(qx, 0.0f) ** 2.0f + MathF.Max(qz, 0.0f) ** 2.0f)
        outside + MathF.Min(MathF.Max(qx, qz), 0.0f)

    /// Distance (m) outside the access road strip (west of the yard).
    let roadDistance (site: SiteLayout) (x: float32) (z: float32) =
        if x > site.YardX then
            1000.0f
        else
            MathF.Abs(z - site.YardZ) - roadHalfWidth

    /// Creek centerline X at a given Z: a slow meander down the east side.
    let creekCenter (site: SiteLayout) (z: float32) =
        site.WorldX * 0.87f
        + site.WorldX * 0.03f * MathF.Sin(z / site.WorldZ * 9.4f + 1.0f)

    let private knollCenter (site: SiteLayout) =
        Vector2(site.WorldX * 0.22f, site.WorldZ * 0.25f)

    let private quarryCenter (site: SiteLayout) =
        Vector2(site.WorldX * 0.25f, site.WorldZ * 0.77f)

    /// Stockpile i (0..2): center XZ and material.
    let stockpile (site: SiteLayout) (index: int) =
        let x = site.YardX - 4.0f + float32 index * 5.0f
        let z = site.YardZ + 5.0f

        let material =
            match index with
            | 0 -> DrySand
            | 1 -> Topsoil
            | _ -> Gravel // east end: off the gravel hardstand, so it reads

        struct (x, z, material)

    /// True where scenery (trees, boulders) may stand: clear of the yard,
    /// road, creek, and the old gravel pit.
    let isOpenGround (site: SiteLayout) (x: float32) (z: float32) =
        padDistance site x z > 5.0f
        && roadDistance site x z > 2.5f
        && MathF.Abs(x - creekCenter site z) > 5.5f
        && Vector2.Distance(Vector2(x, z), quarryCenter site) > 9.0f
        && x > 3.0f
        && z > 3.0f
        && x < site.WorldX - 3.0f
        && z < site.WorldZ - 3.0f

    /// Foundation-pit progress 0..1: excavated volume inside the staked
    /// rectangle against the target (soil that slumps back in counts against
    /// it). −1 when the world has no site.
    let pitProgress (state: SoilState) =
        match state.Site with
        | ValueNone -> -1.0f
        | ValueSome site ->
            let config = state.Config
            let x0 = int (site.PitMinX / config.CellSize)
            let x1 = int (site.PitMaxX / config.CellSize) - 1
            let z0 = int (site.PitMinZ / config.CellSize)
            let z1 = int (site.PitMaxZ / config.CellSize) - 1
            let mutable removed = 0.0f

            for z in z0..z1 do
                for x in x0..x1 do
                    removed <-
                        removed
                        + Math.Clamp(site.PadHeight - state.Heights.[state.ColumnIndex(x, z)], 0.0f, site.PitDepth)

            removed / (float32 ((x1 - x0 + 1) * (z1 - z0 + 1)) * site.PitDepth)

    /// Noise-seeded terrain + job site. Initializes heights, moisture, the
    /// water table and the mass ledger.
    let fill (state: SoilState) (seed: int) (baseHeight: float32) (relief: float32) =
        let config = state.Config
        let size = config.CellSize
        // Groundwater sits under the low ground; lowlands dig up wet.
        state.WaterTableHeight <- baseHeight - relief * 0.55f
        let table = state.WaterTableHeight
        let site = layout config baseHeight
        let hasSite = site.WorldX >= MinSiteWorld && site.WorldZ >= MinSiteWorld

        if hasSite then
            state.Site <- ValueSome site

        let knoll = knollCenter site
        let quarry = quarryCenter site

        for z in 0 .. config.CellsZ - 1 do
            for x in 0 .. config.CellsX - 1 do
                let worldX = (float32 x + 0.5f) * size
                let worldZ = (float32 z + 0.5f) * size
                let here = Vector2(worldX, worldZ)
                // Broad landforms (long wavelength) so there are workable
                // flats between the hills — it's a dig site, not the Alps.
                let point = Vector2(float32 x * 0.02f, float32 z * 0.02f)
                let mutable height = baseHeight + (Noise.fbm2 seed 4 point - 0.5f) * 2.0f * relief
                let mutable padWeight = 0.0f
                let mutable hardstand = false
                let mutable creekBed = false
                let mutable quarryWeight = 0.0f
                let mutable knollWeight = 0.0f
                let mutable pileExtra = 0.0f
                let mutable pileMaterial = Topsoil

                if hasSite then
                    // Knoll: a round hill worth climbing (and tipping on).
                    knollWeight <- smooth (1.0f - Vector2.Distance(here, knoll) / 10.0f)
                    height <- height + 2.8f * knollWeight * knollWeight
                    // Old gravel pit: a bowl that reaches the water table.
                    quarryWeight <- smooth (1.0f - Vector2.Distance(here, quarry) / 8.0f)
                    height <- height - 2.0f * quarryWeight
                    // Creek: a channel cut below the table, so it holds water.
                    let creekDistance = MathF.Abs(worldX - creekCenter site worldZ)
                    let creekWeight = 1.0f - smooth ((creekDistance - 1.4f) / 3.6f)
                    height <- MathF.Min(height, height + (table - 0.5f - height) * creekWeight)
                    creekBed <- creekDistance < 2.4f
                    // Graded yard + access road, level at the pad height.
                    let pad = padDistance site worldX worldZ
                    let road = roadDistance site worldX worldZ

                    padWeight <-
                        MathF.Max(1.0f - smooth (pad / 4.0f), 1.0f - smooth (road / 3.0f))

                    height <- height + (site.PadHeight - height) * padWeight

                    hardstand <-
                        road < 0.0f
                        || Vector2.Distance(here, Vector2(site.YardX, site.YardZ)) < 6.5f

                    for pile in 0..2 do
                        let struct (pileX, pileZ, material) = stockpile site pile

                        let extra =
                            pileHeight - Vector2.Distance(here, Vector2(pileX, pileZ)) * pileSlope

                        if extra > pileExtra then
                            pileExtra <- extra
                            pileMaterial <- material

                height <- Math.Clamp(height, size, float32 (config.CellsY - 8) * size)
                let sandiness = Noise.fbm2 (seed + 7919) 2 (point * 0.32f)
                let rockiness = Noise.fbm2 (seed + 4211) 2 (point * 0.42f)
                let meadow = Noise.fbm2 (seed + 611) 2 (point * 0.22f)
                let lens = Noise.fbm2 (seed + 97) 2 (point * 0.55f)
                // Whole cells plus a fractional top cell: the surface follows
                // the noise instead of terracing in 0.25 m steps.
                let fullCells = max 1 (int (height / size))
                let topUnits = int ((height / size - float32 fullCells) * 255.0f)
                let topCell = if topUnits > 0 then fullCells else fullCells - 1
                let nearWater = height < table + 0.35f

                for y in 0..topCell do
                    let index = state.Index(x, y, z)
                    let depthFromTop = float32 (topCell - y) * size
                    let isSurface = y = topCell

                    // Thresholds set from the measured fbm2 percentiles
                    // (2-octave noise is narrower than 4-octave): ~12%
                    // sand, ~11% gravel, ~30% grass meadow.
                    let natural =
                        if depthFromTop > 0.6f then
                            // Lenses in the clay: a sand seam, a gravel seam.
                            if depthFromTop < 1.6f && lens > 0.56f then
                                (if float32 y * size < table then WetSand else DrySand)
                            elif depthFromTop >= 1.1f && depthFromTop < 2.1f && lens < 0.4f then
                                Gravel
                            else
                                Clay
                        elif rockiness > 0.70f then Gravel
                        elif sandiness > 0.44f then (if nearWater then WetSand else DrySand)
                        elif isSurface && meadow > 0.51f && not nearWater then Grass
                        else Topsoil

                    let mat =
                        if depthFromTop > 0.6f then
                            natural
                        elif creekBed then
                            (if rockiness > 0.5f then Gravel else WetSand)
                        elif quarryWeight > 0.35f then
                            (if sandiness > 0.4f then DrySand else Gravel)
                        elif padWeight > 0.6f then
                            // Stripped and graded: hardstand under the
                            // machine, bare topsoil over the pit.
                            (if hardstand && depthFromTop < 0.25f then Gravel else Topsoil)
                        elif knollWeight > 0.75f && depthFromTop < 0.25f then
                            Gravel
                        else
                            natural

                    state.Occupancy.[index] <- (if isSurface && topUnits > 0 then byte topUnits else 255uy)
                    state.Material.[index] <- Volume.byteOfMaterial mat
                    state.Compaction.[index] <- 255uy
                    // Pre-wet up to the table so the world starts settled.
                    if float32 y * size < table then
                        state.Moisture.[index] <- 255uy

                // Stockpile: loose cells stacked on the graded pad.
                if pileExtra > 0.0f then
                    let mutable units = int (pileExtra / size * 255.0f)
                    let mutable y = topCell + 1

                    while units > 0 && y < config.CellsY do
                        let index = state.Index(x, y, z)
                        let take = min units 255
                        state.Occupancy.[index] <- byte take
                        state.Material.[index] <- Volume.byteOfMaterial pileMaterial
                        state.Compaction.[index] <- 0uy
                        units <- units - take
                        y <- y + 1

                Volume.refreshColumnHeight state x z

        let masses = Volume.scanMassByMaterial state
        Array.blit masses 0 state.Ledger 0 MaterialCount

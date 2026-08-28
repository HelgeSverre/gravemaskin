namespace Gravemaskin

open System.Numerics
open BepuPhysics
open BepuPhysics.Collidables
open BepuUtilities.Memory

/// Static collision meshes for the soil surface, one per XZ tile: a grid of
/// two triangles per column built from bilinear corner heights.
/// ponytail: heightfield triangles, not Surface Nets — Phase 1 needs
/// collision correctness only; the render mesher (Phase 2) upgrades this.
[<RequireQualifiedAccess>]
module SoilCollision =

    /// Corner height = average of the (up to 4) columns sharing that corner.
    let private cornerHeight (state: SoilState) (x: int) (z: int) =
        let config = state.Config
        let mutable total = 0.0f
        let mutable count = 0

        for dz in -1 .. 0 do
            for dx in -1 .. 0 do
                let cx = x + dx
                let cz = z + dz

                if cx >= 0 && cx < config.CellsX && cz >= 0 && cz < config.CellsZ then
                    total <- total + state.Heights.[state.ColumnIndex(cx, cz)]
                    count <- count + 1

        total / float32 count

    /// Build a BEPU Mesh for one tile from the current surface heights.
    /// Caller owns disposal (the pool buffer lives inside the Mesh).
    let buildTileMesh (state: SoilState) (pool: BufferPool) (tileX: int) (tileZ: int) =
        let config = state.Config
        let x0 = tileX * SoilConfig.TileSize
        let z0 = tileZ * SoilConfig.TileSize
        let x1 = min (x0 + SoilConfig.TileSize) config.CellsX
        let z1 = min (z0 + SoilConfig.TileSize) config.CellsZ
        let cellsX = x1 - x0
        let cellsZ = z1 - z0
        let triangleCount = cellsX * cellsZ * 2
        let mutable triangles = Unchecked.defaultof<Buffer<Triangle>>
        pool.Take(triangleCount, &triangles)
        let size = config.CellSize
        let mutable i = 0

        for z in z0 .. z1 - 1 do
            for x in x0 .. x1 - 1 do
                let h00 = cornerHeight state x z
                let h10 = cornerHeight state (x + 1) z
                let h01 = cornerHeight state x (z + 1)
                let h11 = cornerHeight state (x + 1) (z + 1)
                let p00 = Vector3(float32 x * size, h00, float32 z * size)
                let p10 = Vector3(float32 (x + 1) * size, h10, float32 z * size)
                let p01 = Vector3(float32 x * size, h01, float32 (z + 1) * size)
                let p11 = Vector3(float32 (x + 1) * size, h11, float32 (z + 1) * size)
                // Wound so the face normal points +Y (up).
                triangles.[i] <- Triangle(p00, p01, p10)
                triangles.[i + 1] <- Triangle(p10, p01, p11)
                i <- i + 2

        Mesh(triangles, Vector3.One, pool)

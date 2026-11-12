namespace Gravemaskin

open System
open System.Numerics

/// Removing soil from the volume. Exact ΔM accounting: whatever leaves the
/// volume is returned to the caller (to become clumps) and debited nowhere —
/// the ledger tracks volume+clumps together, so carve is ledger-neutral.
[<RequireQualifiedAccess>]
module Carve =

    /// Result of one carve op: mass removed per material (kg), written into a
    /// caller-owned array (no allocation on the tick path).
    /// Returns total removed mass.
    let sphere (state: SoilState) (center: Vector3) (radius: float32) (removedByMaterial: float[]) =
        let config = state.Config
        Array.fill removedByMaterial 0 removedByMaterial.Length 0.0
        let inv = 1.0f / config.CellSize
        let minX = max 0 (int ((center.X - radius) * inv))
        let maxX = min (config.CellsX - 1) (int ((center.X + radius) * inv))
        let minY = max 0 (int ((center.Y - radius) * inv))
        let maxY = min (config.CellsY - 1) (int ((center.Y + radius) * inv))
        let minZ = max 0 (int ((center.Z - radius) * inv))
        let maxZ = min (config.CellsZ - 1) (int ((center.Z + radius) * inv))
        let radiusSq = radius * radius
        let mutable total = 0.0

        for z in minZ..maxZ do
            for x in minX..maxX do
                let mutable touched = false

                for y in minY..maxY do
                    let cellCenter =
                        Vector3(
                            (float32 x + 0.5f) * config.CellSize,
                            (float32 y + 0.5f) * config.CellSize,
                            (float32 z + 0.5f) * config.CellSize
                        )

                    if Vector3.DistanceSquared(cellCenter, center) <= radiusSq then
                        let index = state.Index(x, y, z)
                        let occ = state.Occupancy.[index]

                        if occ > 0uy then
                            let mass =
                                Volume.cellMass config occ state.Material.[index] state.Compaction.[index]

                            removedByMaterial.[int state.Material.[index]] <-
                                removedByMaterial.[int state.Material.[index]] + float mass

                            total <- total + float mass
                            state.Occupancy.[index] <- 0uy
                            touched <- true

                if touched then
                    Volume.compactColumn state x z
                    state.MarkDirty(x, z)

        total

    /// The blade's cut: every column under the sphere is lowered to the
    /// sphere's underside (a bowl below the cutting edge), fractionally.
    /// Same footprint as `sphere` when cutting from above, but the removal
    /// is proportional to what the edge actually swept this tick. Removed
    /// mass ACCUMULATES into `removedByMaterial` (caller zeroes it), so one
    /// tick can cut at several points along a wide lip.
    let private bowlWith
        (state: SoilState)
        (center: Vector3)
        (radius: float32)
        (removedByMaterial: float[])
        (looseOnly: bool)
        =
        let config = state.Config
        let inv = 1.0f / config.CellSize
        let minX = max 0 (int ((center.X - radius) * inv))
        let maxX = min (config.CellsX - 1) (int ((center.X + radius) * inv))
        let minZ = max 0 (int ((center.Z - radius) * inv))
        let maxZ = min (config.CellsZ - 1) (int ((center.Z + radius) * inv))
        let radiusSq = radius * radius
        let mutable total = 0.0

        for z in minZ..maxZ do
            for x in minX..maxX do
                let dx = (float32 x + 0.5f) * config.CellSize - center.X
                let dz = (float32 z + 0.5f) * config.CellSize - center.Z
                let distanceSq = dx * dx + dz * dz

                if distanceSq < radiusSq then
                    let target = center.Y - MathF.Sqrt(radiusSq - distanceSq)
                    total <- total + Volume.cutColumnWith state x z target removedByMaterial looseOnly

        total

    let bowl (state: SoilState) (center: Vector3) (radius: float32) (removedByMaterial: float[]) =
        bowlWith state center radius removedByMaterial false

    /// The bucket shell shouldering loose spoil aside: like `bowl`, but it
    /// takes only loose (uncompacted) soil and leaves the bank alone.
    let looseBowl (state: SoilState) (center: Vector3) (radius: float32) (removedByMaterial: float[]) =
        bowlWith state center radius removedByMaterial true

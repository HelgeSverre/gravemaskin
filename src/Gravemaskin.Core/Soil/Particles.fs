namespace Gravemaskin

open System
open System.Numerics

/// Soil in flight: ballistic mass packets with per-packet f64 mass so the
/// ledger stays exact. A packet is NOT a rigid body — dirt doesn't roll or
/// bounce like a marble. It falls, and where it meets the ground its mass
/// goes straight into the terrain, which then flows to its angle of repose
/// (Settle). One model for every pile: the ground itself.
[<RequireQualifiedAccess>]
module Clumps =
    [<Literal>]
    let MaxClumps = 3000

    /// Render radius bounds (m).
    let MinRadius = 0.04f
    let MaxRadius = 0.12f

    /// Horizontal air drag (1/s): a thrown shovelful loses its sideways
    /// speed over a second or so and drops.
    let AirDrag = 0.9f

    /// Terminal fall speed (m/s).
    let MaxFallSpeed = 12.0f

type ClumpPool() =
    let ids = Array.zeroCreate<int> Clumps.MaxClumps
    let positionsX = Array.zeroCreate<float32> Clumps.MaxClumps
    let positionsY = Array.zeroCreate<float32> Clumps.MaxClumps
    let positionsZ = Array.zeroCreate<float32> Clumps.MaxClumps
    let velocitiesX = Array.zeroCreate<float32> Clumps.MaxClumps
    let velocitiesY = Array.zeroCreate<float32> Clumps.MaxClumps
    let velocitiesZ = Array.zeroCreate<float32> Clumps.MaxClumps
    let masses = Array.zeroCreate<float> Clumps.MaxClumps
    let materials = Array.zeroCreate<byte> Clumps.MaxClumps
    let mutable count = 0
    let mutable nextId = 1

    member _.Count = count
    /// Stable per-packet ids (the shell interpolates by id).
    member _.Ids = ids
    member _.PositionsX = positionsX
    member _.PositionsY = positionsY
    member _.PositionsZ = positionsZ
    member _.VelocitiesX = velocitiesX
    member _.VelocitiesY = velocitiesY
    member _.VelocitiesZ = velocitiesZ
    member _.Masses = masses
    member _.Materials = materials

    member _.Position(index: int) =
        Vector3(positionsX.[index], positionsY.[index], positionsZ.[index])

    /// Total in-flight mass per material, added into `totals`.
    member _.AddMassTotals(totals: float[]) =
        for i in 0 .. count - 1 do
            totals.[int materials.[i]] <- totals.[int materials.[i]] + masses.[i]

    member _.TrySpawn(position: Vector3, velocity: Vector3, mass: float, material: SoilMaterial) =
        if count >= Clumps.MaxClumps || mass <= 0.0 then
            false
        else
            ids.[count] <- nextId
            nextId <- nextId + 1
            positionsX.[count] <- position.X
            positionsY.[count] <- position.Y
            positionsZ.[count] <- position.Z
            velocitiesX.[count] <- velocity.X
            velocitiesY.[count] <- velocity.Y
            velocitiesZ.[count] <- velocity.Z
            masses.[count] <- mass
            materials.[count] <- Volume.byteOfMaterial material
            count <- count + 1
            true

    /// Swap-remove packet i.
    member _.RemoveAt(index: int) =
        count <- count - 1

        if index < count then
            ids.[index] <- ids.[count]
            positionsX.[index] <- positionsX.[count]
            positionsY.[index] <- positionsY.[count]
            positionsZ.[index] <- positionsZ.[count]
            velocitiesX.[index] <- velocitiesX.[count]
            velocitiesY.[index] <- velocitiesY.[count]
            velocitiesZ.[index] <- velocitiesZ.[count]
            masses.[index] <- masses.[count]
            materials.[index] <- materials.[count]

    /// Land a mass on the ground at a world XZ: split bilinearly over the
    /// four nearest columns, so a stream builds a smooth mound rather than
    /// one-column spikes. Exact: the last share is the remainder.
    static member Land(state: SoilState, x: float32, z: float32, mass: float, material: SoilMaterial) =
        let config = state.Config
        let gx = x / config.CellSize - 0.5f
        let gz = z / config.CellSize - 0.5f
        let x0 = int (MathF.Floor gx)
        let z0 = int (MathF.Floor gz)
        let fx = float (gx - float32 x0)
        let fz = float (gz - float32 z0)
        let clampX (v: int) = v |> max 0 |> min (config.CellsX - 1)
        let clampZ (v: int) = v |> max 0 |> min (config.CellsZ - 1)
        let m00 = mass * (1.0 - fx) * (1.0 - fz)
        let m10 = mass * fx * (1.0 - fz)
        let m01 = mass * (1.0 - fx) * fz
        let m11 = mass - m00 - m10 - m01
        Volume.deposit state (clampX x0) (clampZ z0) m00 material
        Volume.deposit state (clampX (x0 + 1)) (clampZ z0) m10 material
        Volume.deposit state (clampX x0) (clampZ (z0 + 1)) m01 material

        if m11 > 0.0 then
            Volume.deposit state (clampX (x0 + 1)) (clampZ (z0 + 1)) m11 material
        else
            // Rounding left the remainder at/below zero: fold it into the
            // residual so the ledger still balances to the last bit.
            state.Unbanked.[int (Volume.byteOfMaterial material)] <-
                state.Unbanked.[int (Volume.byteOfMaterial material)] + m11

    /// One tick: integrate every packet; those that reach the ground land
    /// (mass into the terrain) and are removed. Nothing ever just despawns.
    member this.Step(state: SoilState, dt: float32) =
        let config = state.Config
        let worldX = float32 config.CellsX * config.CellSize
        let worldZ = float32 config.CellsZ * config.CellSize
        let drag = MathF.Max(0.0f, 1.0f - Clumps.AirDrag * dt)
        let mutable i = 0

        while i < count do
            velocitiesY.[i] <- MathF.Max(velocitiesY.[i] - 9.81f * dt, -Clumps.MaxFallSpeed)
            velocitiesX.[i] <- velocitiesX.[i] * drag
            velocitiesZ.[i] <- velocitiesZ.[i] * drag
            // The world edge is a wall: soil stops there and drops.
            let x = Math.Clamp(positionsX.[i] + velocitiesX.[i] * dt, 0.01f, worldX - 0.01f)
            let z = Math.Clamp(positionsZ.[i] + velocitiesZ.[i] * dt, 0.01f, worldZ - 0.01f)
            let y = positionsY.[i] + velocitiesY.[i] * dt
            positionsX.[i] <- x
            positionsY.[i] <- y
            positionsZ.[i] <- z
            let cx = int (x / config.CellSize) |> max 0 |> min (config.CellsX - 1)
            let cz = int (z / config.CellSize) |> max 0 |> min (config.CellsZ - 1)

            if y <= state.Heights.[state.ColumnIndex(cx, cz)] then
                ClumpPool.Land(state, x, z, masses.[i], Volume.materialOfByte materials.[i])
                this.RemoveAt i
            // no increment after swap-remove: the swapped-in packet is next.
            else
                i <- i + 1

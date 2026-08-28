namespace Gravemaskin

open System
open System.Numerics
open BepuPhysics

/// Loose-soil clumps: BEPU sphere bodies with per-clump f64 mass so the
/// ledger stays exact. Hard-capped; overflow mass deposits straight into the
/// volume instead of spawning (conserves mass with zero body churn).
[<RequireQualifiedAccess>]
module Clumps =
    [<Literal>]
    let MaxClumps = 1500

    /// Below this speed for SettleTicks consecutive ticks (or asleep), a
    /// clump banks back into the volume.
    let SettleSpeedSq = 0.05f * 0.05f

    [<Literal>]
    let SettleTicks = 30

    let MinRadius = 0.08f
    let MaxRadius = 0.35f

type ClumpPool() =
    let handles = Array.zeroCreate<BodyHandle> Clumps.MaxClumps
    let masses = Array.zeroCreate<float> Clumps.MaxClumps
    let materials = Array.zeroCreate<byte> Clumps.MaxClumps
    let stillTicks = Array.zeroCreate<int> Clumps.MaxClumps
    let mutable count = 0

    member _.Count = count
    member _.Handles = handles
    member _.Masses = masses
    member _.Materials = materials

    /// Total live clump mass per material, added into `totals`.
    member _.AddMassTotals(totals: float[]) =
        for i in 0 .. count - 1 do
            totals.[int materials.[i]] <- totals.[int materials.[i]] + masses.[i]

    member _.TrySpawn(simulation: Simulation, position: Vector3, mass: float, material: SoilMaterial) =
        if count >= Clumps.MaxClumps || mass <= 0.0 then
            false
        else
            let props = Tuning.soil material
            let volume = float32 mass / Volume.looseDensity props

            let radius =
                MathF.Cbrt(volume * 3.0f / (4.0f * MathF.PI))
                |> max Clumps.MinRadius
                |> min Clumps.MaxRadius

            handles.[count] <- Bepu.addDynamicSphere simulation position radius (float32 mass)
            masses.[count] <- mass
            materials.[count] <- Volume.byteOfMaterial material
            stillTicks.[count] <- 0
            count <- count + 1
            true

    /// Swap-remove clump i, removing its body from the simulation.
    member _.RemoveAt(simulation: Simulation, index: int) =
        simulation.Bodies.Remove(handles.[index])
        count <- count - 1

        if index < count then
            handles.[index] <- handles.[count]
            masses.[index] <- masses.[count]
            materials.[index] <- materials.[count]
            stillTicks.[index] <- stillTicks.[count]

    /// Per-tick pass: settle resting clumps back into the volume, and return
    /// escaped clumps' mass to the ledger via a clamped-column deposit
    /// (mass-loss guard: nothing ever just despawns).
    member this.SettlePass(simulation: Simulation, state: SoilState) =
        let config = state.Config
        let mutable i = 0

        while i < count do
            let bodyRef = simulation.Bodies.[handles.[i]]
            let position = bodyRef.Pose.Position
            let escaped = position.Y < -10.0f

            let resting =
                if not bodyRef.Awake then
                    true
                else
                    let velocity = bodyRef.Velocity.Linear

                    if velocity.LengthSquared() < Clumps.SettleSpeedSq then
                        stillTicks.[i] <- stillTicks.[i] + 1
                        stillTicks.[i] >= Clumps.SettleTicks
                    else
                        stillTicks.[i] <- 0
                        false

            if escaped || resting then
                let x = int (position.X / config.CellSize) |> max 0 |> min (config.CellsX - 1)
                let z = int (position.Z / config.CellSize) |> max 0 |> min (config.CellsZ - 1)
                Volume.deposit state x z masses.[i] (Volume.materialOfByte materials.[i])
                this.RemoveAt(simulation, i)
            // note: no i increment after swap-remove — the swapped-in clump
            // gets evaluated next loop.
            else
                i <- i + 1

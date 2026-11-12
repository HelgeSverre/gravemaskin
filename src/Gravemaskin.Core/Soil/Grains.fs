namespace Gravemaskin

open System
open System.Numerics

/// The dust-and-crumb layer: small visual grains that fly with the soil in
/// motion — the spray off the teeth, the fines in a pour, the dirt a track
/// throws. Purely presentation: zero mass, stepped on wall-clock by the
/// shell, never touches the ledger or the determinism hash.
///
/// Grains do NOT pile. The pile is the terrain itself, which rises where
/// the mass packets (ClumpPool) land; a grain that reaches the ground takes
/// one dead bounce at most, then sinks into the surface and is gone. (The
/// old layer kept its own pile-height field that the real terrain then had
/// to catch up with — two piles for one heap of dirt.)
type GrainPool(capacity: int, state: SoilState) =
    let positionsX = Array.zeroCreate<float32> capacity
    let positionsY = Array.zeroCreate<float32> capacity
    let positionsZ = Array.zeroCreate<float32> capacity
    let velocitiesX = Array.zeroCreate<float32> capacity
    let velocitiesY = Array.zeroCreate<float32> capacity
    let velocitiesZ = Array.zeroCreate<float32> capacity
    let sizes = Array.zeroCreate<float32> capacity
    let materials = Array.zeroCreate<byte> capacity
    let wetness = Array.zeroCreate<byte> capacity
    // < 0 = airborne; >= 0 = landed, value is seconds since landing (the
    // grain is sinking away).
    let restTimers = Array.zeroCreate<float32> capacity
    let flightTimes = Array.zeroCreate<float32> capacity
    let mutable count = 0
    // Ring cursor: at capacity, the oldest grain is recycled.
    let mutable writeCursor = 0
    let mutable rngState = 0x9E3779B9u

    let nextRandom () =
        rngState <- rngState * 1664525u + 1013904223u
        float32 (rngState >>> 8) / 16777216.0f

    let removeAt (i: int) =
        count <- count - 1

        if i < count then
            positionsX.[i] <- positionsX.[count]
            positionsY.[i] <- positionsY.[count]
            positionsZ.[i] <- positionsZ.[count]
            velocitiesX.[i] <- velocitiesX.[count]
            velocitiesY.[i] <- velocitiesY.[count]
            velocitiesZ.[i] <- velocitiesZ.[count]
            sizes.[i] <- sizes.[count]
            materials.[i] <- materials.[count]
            wetness.[i] <- wetness.[count]
            restTimers.[i] <- restTimers.[count]
            flightTimes.[i] <- flightTimes.[count]

        if writeCursor >= count && count > 0 then
            writeCursor <- writeCursor % count

    /// Seconds a landed grain takes to sink into the surface.
    static member SinkSeconds = 0.3f

    /// Longest a grain may stay airborne (s) before it is dropped.
    static member MaxFlightSeconds = 8.0f

    /// Ground the grain layer sees: the real surface.
    member _.GroundHeight(x: float32, z: float32) = Soil.surfaceHeight state x z

    member _.Count = count
    member _.Capacity = capacity
    member _.PositionsX = positionsX
    member _.PositionsY = positionsY
    member _.PositionsZ = positionsZ
    member _.Sizes = sizes
    member _.VelocitiesX = velocitiesX
    member _.VelocitiesY = velocitiesY
    member _.VelocitiesZ = velocitiesZ
    member _.Materials = materials
    member _.Wetness = wetness
    member _.RestTimers = restTimers

    /// Spawn one grain. At capacity the oldest is recycled.
    member _.Spawn(position: Vector3, velocity: Vector3, material: SoilMaterial, wet: byte, size: float32) =
        let index =
            if count < capacity then
                let i = count
                count <- count + 1
                i
            else
                let i = writeCursor
                writeCursor <- (writeCursor + 1) % capacity
                i

        positionsX.[index] <- position.X
        positionsY.[index] <- position.Y
        positionsZ.[index] <- position.Z
        velocitiesX.[index] <- velocity.X
        velocitiesY.[index] <- velocity.Y
        velocitiesZ.[index] <- velocity.Z
        sizes.[index] <- size
        materials.[index] <- Volume.byteOfMaterial material
        wetness.[index] <- wet
        restTimers.[index] <- -1.0f
        flightTimes.[index] <- 0.0f

    /// Convenience: a burst of grains in a cone.
    member this.SpawnBurst
        (origin: Vector3, baseVelocity: Vector3, spread: float32, material: SoilMaterial, wet: byte, grains: int)
        =
        let props = Tuning.soil material

        // Grain size by material: sand fine, gravel chunky.
        let baseSize =
            if props.FrictionAngle > 0.72f then 0.013f // gravel
            elif props.Cohesion > 3.0f<kPa> then 0.010f // clay/turf: crumbs
            else 0.007f // sands, topsoil

        for _ in 1..grains do
            let jitterVelocity =
                Vector3(
                    (nextRandom () - 0.5f) * 2.0f * spread,
                    (nextRandom () - 0.5f) * spread,
                    (nextRandom () - 0.5f) * 2.0f * spread
                )

            let jitterPosition =
                Vector3((nextRandom () - 0.5f) * 0.12f, (nextRandom () - 0.5f) * 0.12f, (nextRandom () - 0.5f) * 0.12f)

            this.Spawn(
                origin + jitterPosition,
                baseVelocity + jitterVelocity,
                material,
                wet,
                baseSize * (0.7f + nextRandom () * 0.8f)
            )

    /// One wall-clock step. Airborne grains fall; on reaching the ground a
    /// fast dry grain takes one dead bounce, anything else lands. Landed
    /// grains shrink into the surface and are removed.
    member this.Step(dt: float32) =
        let dt = Math.Clamp(dt, 0.0f, 0.05f)
        let mutable i = 0

        while i < count do
            if restTimers.[i] < 0.0f then
                flightTimes.[i] <- flightTimes.[i] + dt
                velocitiesY.[i] <- velocitiesY.[i] - 9.81f * dt
                positionsX.[i] <- positionsX.[i] + velocitiesX.[i] * dt
                positionsY.[i] <- positionsY.[i] + velocitiesY.[i] * dt
                positionsZ.[i] <- positionsZ.[i] + velocitiesZ.[i] * dt
                let ground = this.GroundHeight(positionsX.[i], positionsZ.[i])

                if positionsY.[i] <= ground + sizes.[i] * 0.35f then
                    positionsY.[i] <- ground + sizes.[i] * 0.35f

                    if velocitiesY.[i] < -2.5f && wetness.[i] <= 128uy then
                        // One dead bounce with a little scatter.
                        velocitiesY.[i] <- -velocitiesY.[i] * 0.12f
                        velocitiesX.[i] <- velocitiesX.[i] * 0.5f + (nextRandom () - 0.5f) * 0.5f
                        velocitiesZ.[i] <- velocitiesZ.[i] * 0.5f + (nextRandom () - 0.5f) * 0.5f
                    else
                        restTimers.[i] <- 0.0f
                        velocitiesX.[i] <- 0.0f
                        velocitiesY.[i] <- 0.0f
                        velocitiesZ.[i] <- 0.0f

                    i <- i + 1
                elif flightTimes.[i] > GrainPool.MaxFlightSeconds then
                    removeAt i
                else
                    i <- i + 1
            else
                // Landed: sink into the surface (which may itself be rising
                // as the pile grows) and vanish.
                restTimers.[i] <- restTimers.[i] + dt

                if restTimers.[i] >= GrainPool.SinkSeconds then
                    removeAt i
                else
                    sizes.[i] <- sizes.[i] * MathF.Max(0.0f, 1.0f - dt / GrainPool.SinkSeconds * 1.5f)
                    positionsY.[i] <- this.GroundHeight(positionsX.[i], positionsZ.[i]) + sizes.[i] * 0.35f
                    i <- i + 1

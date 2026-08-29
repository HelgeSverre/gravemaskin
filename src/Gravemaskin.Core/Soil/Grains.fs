namespace Gravemaskin

open System
open System.Numerics

/// The falling-sand layer: a dense pool of small visual grains that fall,
/// bounce, stack, and avalanche. This is the *presentation* of loose soil —
/// the mass itself rides in the (invisible) clump bodies and the ledger.
/// Per the SPEC's cosmetic-particle contract: stepped on wall-clock by the
/// shell, carries zero mass, never touches the ledger or determinism hash.
///
/// Piling is the classic falling-sand trick: resting grains deposit into a
/// fine height grid (half-cell resolution) that later grains land on, so
/// streams stack into cones; cells steeper than repose shed their grains
/// back into flight (avalanche). The grid decays as the real terrain mesh
/// catches up with the settled mass underneath.
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
    // < 0 = airborne; >= 0 = resting, value is seconds at rest.
    let restTimers = Array.zeroCreate<float32> capacity
    let mutable count = 0
    // Ring cursor: at capacity, the oldest grain is recycled (falling-sand
    // pools do the same — the newest motion is what the eye follows).
    let mutable writeCursor = 0

    // Visual pile field at 4× column resolution (6 cm cells): cones and
    // avalanche fronts resolve finely enough to read as granular.
    let pileResolution = 4
    let pileCellSize = state.Config.CellSize / float32 pileResolution
    let pileWidth = state.Config.CellsX * pileResolution
    let pileDepth = state.Config.CellsZ * pileResolution
    let pile = Array.zeroCreate<float32> (pileWidth * pileDepth)
    let mutable pileDecayCursor = 0
    let mutable rngState = 0x9E3779B9u

    let nextRandom () =
        rngState <- rngState * 1664525u + 1013904223u
        float32 (rngState >>> 8) / 16777216.0f

    let pileIndex (x: float32) (z: float32) =
        let px = int (x / pileCellSize) |> max 0 |> min (pileWidth - 1)
        let pz = int (z / pileCellSize) |> max 0 |> min (pileDepth - 1)
        pz * pileWidth + px

    /// Ground the grain layer sees: real surface + visual pile.
    member _.GroundHeight(x: float32, z: float32) =
        Soil.surfaceHeight state x z + pile.[pileIndex x z]

    member _.Count = count
    member _.Capacity = capacity
    member _.PositionsX = positionsX
    member _.PositionsY = positionsY
    member _.PositionsZ = positionsZ
    member _.Sizes = sizes
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

    /// Convenience: a burst of grains in a cone.
    member this.SpawnBurst
        (origin: Vector3, baseVelocity: Vector3, spread: float32, material: SoilMaterial, wet: byte, grains: int)
        =
        let props = Tuning.soil material

        // Grain size by material: sand fine, gravel chunky.
        let baseSize =
            if props.FrictionAngle > 0.72f then 0.026f // gravel
            elif props.Cohesion > 3.0f<kPa> then 0.019f // clay/turf: crumbs
            else 0.013f // sands, topsoil

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

    /// One wall-clock step. Airborne grains integrate and bounce; slow ones
    /// rest and deposit into the pile field; resting grains on over-steep
    /// pile cells re-mobilize (avalanche); old resting grains expire.
    member this.Step(dt: float32) =
        let dt = Math.Clamp(dt, 0.0f, 0.05f)
        let mutable i = 0

        while i < count do
            if restTimers.[i] < 0.0f then
                // Airborne: integrate.
                velocitiesY.[i] <- velocitiesY.[i] - 9.81f * dt
                positionsX.[i] <- positionsX.[i] + velocitiesX.[i] * dt
                positionsY.[i] <- positionsY.[i] + velocitiesY.[i] * dt
                positionsZ.[i] <- positionsZ.[i] + velocitiesZ.[i] * dt

                let ground = this.GroundHeight(positionsX.[i], positionsZ.[i])

                if positionsY.[i] <= ground + sizes.[i] then
                    positionsY.[i] <- ground + sizes.[i]

                    let speedSq =
                        velocitiesX.[i] * velocitiesX.[i]
                        + velocitiesY.[i] * velocitiesY.[i]
                        + velocitiesZ.[i] * velocitiesZ.[i]

                    // Wet grains splat; dry ones bounce a little.
                    let restitution = if wetness.[i] > 128uy then 0.02f else 0.18f

                    if speedSq < 0.25f then
                        // Rest and stack: deposit into the pile field.
                        restTimers.[i] <- 0.0f
                        velocitiesX.[i] <- 0.0f
                        velocitiesY.[i] <- 0.0f
                        velocitiesZ.[i] <- 0.0f
                        let cell = pileIndex positionsX.[i] positionsZ.[i]

                        pile.[cell] <-
                            min (pile.[cell] + sizes.[i] * 0.9f) 1.5f
                    else
                        // Bounce: reflect vertically, drag tangentially, and
                        // pick up downslope drift from the local gradient.
                        velocitiesY.[i] <- -velocitiesY.[i] * restitution
                        let step = pileCellSize

                        let gradientX =
                            this.GroundHeight(positionsX.[i] + step, positionsZ.[i])
                            - this.GroundHeight(positionsX.[i] - step, positionsZ.[i])

                        let gradientZ =
                            this.GroundHeight(positionsX.[i], positionsZ.[i] + step)
                            - this.GroundHeight(positionsX.[i], positionsZ.[i] - step)

                        let friction = if wetness.[i] > 128uy then 0.45f else 0.72f
                        velocitiesX.[i] <- velocitiesX.[i] * friction - gradientX * 1.6f
                        velocitiesZ.[i] <- velocitiesZ.[i] * friction - gradientZ * 1.6f

                i <- i + 1
            else
                // Resting.
                restTimers.[i] <- restTimers.[i] + dt
                let x = positionsX.[i]
                let z = positionsZ.[i]
                let step = pileCellSize
                let here = this.GroundHeight(x, z)

                let lowestNeighbor =
                    min
                        (min (this.GroundHeight(x + step, z)) (this.GroundHeight(x - step, z)))
                        (min (this.GroundHeight(x, z + step)) (this.GroundHeight(x, z - step)))

                let props = Tuning.soil (Volume.materialOfByte materials.[i])

                if here - lowestNeighbor > MathF.Tan props.FrictionAngle * step * 1.4f then
                    // Avalanche: this spot is over-steep — take the grain's
                    // contribution back out of the pile and let it slide off
                    // downhill (no upward pop: avalanches slump, they don't
                    // leap).
                    let cell = pileIndex x z
                    pile.[cell] <- max (pile.[cell] - sizes.[i] * 0.9f) 0.0f
                    restTimers.[i] <- -1.0f

                    let gradientX =
                        this.GroundHeight(x + step, z) - this.GroundHeight(x - step, z)

                    let gradientZ =
                        this.GroundHeight(x, z + step) - this.GroundHeight(x, z - step)

                    velocitiesX.[i] <- -gradientX * 2.0f
                    velocitiesY.[i] <- 0.0f
                    velocitiesZ.[i] <- -gradientZ * 2.0f
                    i <- i + 1
                elif here < positionsY.[i] - sizes.[i] - 0.25f then
                    // Support truly vanished (dug out from under): fall.
                    restTimers.[i] <- -1.0f
                    i <- i + 1
                elif here < positionsY.[i] - sizes.[i] - 0.005f then
                    // Support sank a little (pile decaying into the terrain
                    // mesh): follow it down QUIETLY and stay at rest — the
                    // old re-fall here made settled piles shimmer forever.
                    positionsY.[i] <- here + sizes.[i]
                    i <- i + 1
                elif restTimers.[i] > 9.0f then
                    // Expired: the settled mass is in the terrain by now.
                    // Swap-remove.
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

                    if writeCursor >= count && count > 0 then
                        writeCursor <- writeCursor % count
                else
                    i <- i + 1

        // Pile decay, amortized: the real terrain mesh rises where mass
        // settles, so the visual pile hands over gradually.
        let cellsPerStep = max 1 (pile.Length / 90)

        for _ in 1..cellsPerStep do
            pile.[pileDecayCursor] <- pile.[pileDecayCursor] * 0.985f
            pileDecayCursor <- (pileDecayCursor + 1) % pile.Length

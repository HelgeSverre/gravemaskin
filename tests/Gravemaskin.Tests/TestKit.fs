namespace Gravemaskin.Tests

open Gravemaskin

/// Canonical world builders and step loops. Tests build on these so they
/// don't break when internals reorganize.
[<RequireQualifiedAccess>]
module TestKit =

    let defaultSeed = 0xDEADBEEFUL

    let isReleaseBuild =
#if DEBUG
        false
#else
        true
#endif

    let flatWorld () = Sim.createWorld defaultSeed

    /// Step `count` ticks feeding the same input; returns the last snapshot.
    let stepAll (count: int) (input: InputFrame) (world: World) =
        let mutable last = Unchecked.defaultof<RenderState>

        for _ in 1..count do
            last <- world.Step input

        last

    /// Small soil world (16×16 m, 0.25 m cells, 2 m of soil) for fast tests.
    let smallSoilConfig =
        { CellSize = 0.25f
          CellsX = 64
          CellsY = 32
          CellsZ = 64 }

    let soilWorld (mat: SoilMaterial) =
        new World(defaultSeed, Sim.defaultThreadCount, Some(smallSoilConfig, mat, 2.0f))

    /// Conservation check: volume scan + unbanked + live clumps vs ledger.
    /// Returns the largest relative error across materials.
    let conservationError (world: World) =
        match world.SoilState with
        | None -> 0.0
        | Some state ->
            let totals = Soil.massTotals state
            world.Clumps.AddMassTotals totals

            // Payload in the bucket is mass in flight, not mass lost.
            match world.Machine with
            | Some m ->
                for i in 0..4 do
                    totals.[i] <- totals.[i] + m.BucketLoad.[i]
            | None -> ()
            let mutable worst = 0.0

            for i in 0 .. totals.Length - 1 do
                let expected = state.Ledger.[i]

                if expected > 0.0 then
                    worst <- max worst (abs (totals.[i] - expected) / expected)
                else
                    worst <- max worst (abs totals.[i])

            worst

    /// FNV-1a over the soil arrays + ledger — the determinism gate's second
    /// half (poses alone don't catch a nondeterministic soil pipeline).
    let hashSoil (world: World) =
        match world.SoilState with
        | None -> 0UL
        | Some state ->
            let mutable hash = 14695981039346656037UL

            let inline mixByte (value: byte) =
                hash <- (hash ^^^ uint64 value) * 1099511628211UL

            for i in 0 .. state.Occupancy.Length - 1 do
                mixByte state.Occupancy.[i]
                mixByte state.Material.[i]
                mixByte state.Compaction.[i]

            for i in 0 .. state.Ledger.Length - 1 do
                hash <- (hash ^^^ uint64 (System.BitConverter.DoubleToUInt64Bits state.Ledger.[i])) * 1099511628211UL
                hash <- (hash ^^^ uint64 (System.BitConverter.DoubleToUInt64Bits state.Unbanked.[i])) * 1099511628211UL

            hash

    /// Deterministic scripted input: axes derived from the tick index so
    /// replay tests exercise moving controls without an Rng in the test.
    let scriptedInput (tick: int) =
        { InputFrame.empty with
            Sequence = int64 tick
            Swing = sin (float32 tick * 0.013f)
            Boom = sin (float32 tick * 0.007f)
            Stick = cos (float32 tick * 0.011f)
            Bucket = sin (float32 tick * 0.017f)
            LeftTrack = sin (float32 tick * 0.005f)
            RightTrack = cos (float32 tick * 0.006f) }

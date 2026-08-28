namespace Gravemaskin.Tests

open Gravemaskin

/// Canonical world builders and step loops. Tests build on these so they
/// don't break when internals reorganize.
[<RequireQualifiedAccess>]
module TestKit =

    let defaultSeed = 0xDEADBEEFUL

    let flatWorld () = Sim.createWorld defaultSeed

    /// Step `count` ticks feeding the same input; returns the last snapshot.
    let stepAll (count: int) (input: InputFrame) (world: World) =
        let mutable last = Unchecked.defaultof<RenderState>

        for _ in 1..count do
            last <- world.Step input

        last

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

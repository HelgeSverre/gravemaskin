namespace Gravemaskin

open System.Numerics

/// The mutable world (bloom precedent). The house invariant is headless
/// determinism behind Step(InputFrame), not immutability: BEPU is pool-based
/// and soil will be flat arrays.
type World(seed: uint64, threadCount: int) =
    let physics = new Physics(threadCount)
    let mutable rng = Rng.create seed
    let mutable tick = 0L
    // Pooled event buffer: cleared at the start of each Step, valid until the
    // next Step. Never allocated per tick (the zero-alloc gate depends on it).
    let events = ResizeArray<GameEvent>(64)

    do
        // Flat ground slab; real terrain arrives with the soil volume (Phase 1).
        Bepu.addStaticBox physics.Simulation (Vector3(0.0f, -0.5f, 0.0f)) (Vector3(200.0f, 1.0f, 200.0f))
        |> ignore

    member _.Physics = physics
    member _.Tick = tick

    /// Events raised by the most recent Step; consume before stepping again.
    member _.Events: ResizeArray<GameEvent> = events

    member _.Rng: byref<Rng.State> = &rng

    member _.Step(_input: InputFrame) : RenderState =
        events.Clear()
        physics.Step()
        tick <- tick + 1L

        { Tick = tick
          BodyCount = physics.BodyCount }

    interface System.IDisposable with
        member _.Dispose() = (physics :> System.IDisposable).Dispose()

[<RequireQualifiedAccess>]
module Sim =
    /// Standard world: multithreaded with a pinned thread count for
    /// same-machine determinism (never Environment.ProcessorCount directly —
    /// a replay on the same box must use the same count).
    let defaultThreadCount = 4

    let createWorld seed = new World(seed, defaultThreadCount)

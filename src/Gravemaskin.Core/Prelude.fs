namespace Gravemaskin

/// Units of measure for the physical sim. Raw float32 leaves a function
/// boundary only inside hot kernels; everything tunable carries a unit.
[<Measure>]
type m

[<Measure>]
type kg

[<Measure>]
type s

[<Measure>]
type kPa

[<Measure>]
type kN

[<RequireQualifiedAccess>]
module Rng =
    /// splitmix64 — deterministic, struct-threaded through the world.
    [<Struct>]
    type State = private State of uint64

    let create seed = State seed

    let nextUInt64 (state: byref<State>) =
        let (State value) = state
        let next = value + 0x9E3779B97F4A7C15UL
        state <- State next
        let mutable z = next
        z <- (z ^^^ (z >>> 30)) * 0xBF58476D1CE4E5B9UL
        z <- (z ^^^ (z >>> 27)) * 0x94D049BB133111EBUL
        z ^^^ (z >>> 31)

    let nextFloat32 (state: byref<State>) =
        let bits = nextUInt64 &state >>> 40
        float32 bits / float32 (1UL <<< 24)

[<RequireQualifiedAccess>]
module Units =
    let inline raw (value: float32<'u>) = float32 value

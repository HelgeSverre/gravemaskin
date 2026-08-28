namespace Gravemaskin

open System

/// Quasistatic hydraulic model: no fluid, just budgets. Per tick:
///   demandᵢ = |axisᵢ| × QMaxᵢ, summed per circuit;
///   oversubscribed circuits scale every consumer down (flow sharing — this
///   single rule is why three functions at once feel slower);
///   force caps come from relief pressure × cylinder area × moment arm.
[<RequireQualifiedAccess>]
module Hydraulics =

    [<Literal>]
    let FunctionCount = 6

    [<Literal>]
    let CircuitCount = 3

    /// Function indices (fixed order everywhere).
    [<Literal>]
    let Boom = 0

    [<Literal>]
    let Stick = 1

    [<Literal>]
    let Bucket = 2

    [<Literal>]
    let Swing = 3

    [<Literal>]
    let TrackL = 4

    [<Literal>]
    let TrackR = 5

    let private litersPerMinToM3PerSec = 1.0f / 60000.0f

    /// Input shaping: deadband → power curve → (lag applied by caller).
    let shapeAxis (raw: float32) =
        let clamped = Math.Clamp(raw, -1.0f, 1.0f)
        let magnitude = MathF.Abs clamped

        if magnitude < Tuning.InputDeadband then
            0.0f
        else
            let normalized =
                (magnitude - Tuning.InputDeadband) / (1.0f - Tuning.InputDeadband)

            MathF.Sign clamped |> float32
            |> (*) (MathF.Pow(normalized, Tuning.InputCurveExponent))

    /// First-order valve lag toward the shaped target.
    let lag (previous: float32) (target: float32) (dt: float32) =
        let alpha = dt / (float32 Tuning.ValveLagSeconds + dt)
        previous + (target - previous) * alpha

    /// Resolve flow sharing: writes each function's granted flow (m³/s for
    /// cylinders; the swing/track entries get their scale factor 0..1 in
    /// grantedScale). Caller-owned scratch arrays, zero allocation.
    let allocate
        (spec: MachineSpec)
        (axes: float32[])
        (circuitOf: int[])
        (qMax: float32[])
        (grantedFlow: float32[])
        (grantedScale: float32[])
        (circuitDemand: float32[])
        =
        Array.fill circuitDemand 0 CircuitCount 0.0f

        for i in 0 .. FunctionCount - 1 do
            circuitDemand.[circuitOf.[i]] <- circuitDemand.[circuitOf.[i]] + MathF.Abs axes.[i] * qMax.[i]

        for i in 0 .. FunctionCount - 1 do
            let circuit = circuitOf.[i]
            let supply = fst spec.Circuits.[min circuit (spec.Circuits.Length - 1)]

            let scale =
                if circuitDemand.[circuit] > supply then
                    supply / circuitDemand.[circuit]
                else
                    1.0f

            grantedScale.[i] <- scale

            grantedFlow.[i] <- MathF.Abs axes.[i] * qMax.[i] * scale * litersPerMinToM3PerSec

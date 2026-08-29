namespace Gravemaskin

open System
open System.Numerics

/// Simplified Fundamental Equation of Earthmoving: resistance a cutting edge
/// feels moving through soil, per meter of blade width:
///   F/w = ½·γ·g·d²·N_γ  +  c·d·N_c
/// (surcharge and adhesion terms dropped; N-factors folded into two Tuning
/// knobs because the closed forms assume geometry we don't track — the SPEC
/// budgets days of hand-tuning here and these are the knobs.)
[<RequireQualifiedAccess>]
module Fee =

    /// Resistance magnitude (N) for a cut of depth `d` (m) with blade width
    /// `w` (m) in the given material.
    let resistance (props: SoilProperties) (compaction: byte) (moisture: byte) (depth: float32) (width: float32) =
        let d = Math.Clamp(depth, 0.0f, 0.6f)
        let gamma = Volume.density props compaction * 9.81f
        let gammaTerm = 0.5f * gamma * d * d * Tuning.FeeGammaFactor

        let cohesionTerm =
            Moisture.effectiveCohesion props moisture * 1000.0f * d * Tuning.FeeCohesionFactor
        // Compacted bank cuts harder than fresh spoil.
        let compactionScale = 0.5f + 0.5f * float32 compaction / 255.0f
        (gammaTerm + cohesionTerm) * width * compactionScale

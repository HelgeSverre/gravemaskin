namespace Gravemaskin.ProcGen

open System

/// Procedural machine sounds — no audio assets (house style). Loops use
/// integer-Hz components over whole seconds so they wrap seamlessly.
[<RequireQualifiedAccess>]
module AudioSynth =

    type Sound = { Samples: int16[]; SampleRate: int }

    [<Literal>]
    let private Rate = 22050

    let private make (seconds: float32) (generate: int -> float32 -> float32) =
        let count = int (float32 Rate * seconds)
        let samples = Array.zeroCreate<int16> count
        let mutable noiseState = 0x12345u

        let noise () =
            noiseState <- noiseState * 1664525u + 1013904223u
            float32 (int (noiseState >>> 16) % 65536 - 32768) / 32768.0f

        for i in 0 .. count - 1 do
            let t = float32 i / float32 Rate
            let value = generate i t + 0.0f * noise ()
            samples.[i] <- int16 (Math.Clamp(value, -1.0f, 1.0f) * 32000.0f)

        samples

    let private rng = Random 1234
    let private noiseAt = Array.init Rate (fun _ -> float32 (rng.NextDouble() * 2.0 - 1.0))

    /// Idle diesel: layered integer-Hz harmonics + combustion grumble. 1 s loop.
    let engineLoop () =
        let samples =
            make 1.0f (fun i t ->
                let fundamental = 44.0f
                let cycle = MathF.Sin(2.0f * MathF.PI * fundamental * t)
                let second = 0.55f * MathF.Sin(2.0f * MathF.PI * fundamental * 2.0f * t + 0.7f)
                let third = 0.3f * MathF.Sin(2.0f * MathF.PI * fundamental * 3.0f * t + 1.9f)
                // Combustion roughness: noise gated at the firing rate.
                let gate = MathF.Max(0.0f, MathF.Sin(2.0f * MathF.PI * fundamental * t))
                let grumble = noiseAt.[i % Rate] * 0.25f * gate
                (cycle + second + third + grumble) * 0.30f)

        { Samples = samples; SampleRate = Rate }

    /// Hydraulic pump whine: tone + narrowband hiss. 1 s loop.
    let hydraulicLoop () =
        let samples =
            make 1.0f (fun i t ->
                let tone = MathF.Sin(2.0f * MathF.PI * 620.0f * t)
                let overtone = 0.4f * MathF.Sin(2.0f * MathF.PI * 1240.0f * t)
                let hiss = noiseAt.[i % Rate] * 0.5f
                (tone * 0.5f + overtone * 0.3f + hiss * 0.35f) * 0.35f)

        { Samples = samples; SampleRate = Rate }

    /// Relief-valve squeal: rising chirp, one-shot.
    let reliefSqueal () =
        let samples =
            make 0.45f (fun _ t ->
                let sweep = 1350.0f + 700.0f * (t / 0.45f)
                let envelope = MathF.Min(t * 20.0f, 1.0f) * MathF.Exp(-t * 4.0f)
                MathF.Sin(2.0f * MathF.PI * sweep * t) * envelope * 0.5f)

        { Samples = samples; SampleRate = Rate }

    /// Soil pouring out of the bucket: decaying rough noise.
    let soilPour () =
        let samples =
            make 0.5f (fun i t ->
                let envelope = MathF.Min(t * 30.0f, 1.0f) * MathF.Exp(-t * 5.0f)
                noiseAt.[(i * 3) % Rate] * envelope * 0.55f)

        { Samples = samples; SampleRate = Rate }

    /// Metallic strike (rock hit, hard contact).
    let clank () =
        let samples =
            make 0.25f (fun _ t ->
                let envelope = MathF.Exp(-t * 22.0f)

                (MathF.Sin(2.0f * MathF.PI * 2100.0f * t)
                 + 0.6f * MathF.Sin(2.0f * MathF.PI * 3170.0f * t)
                 + 0.4f * MathF.Sin(2.0f * MathF.PI * 830.0f * t))
                * envelope
                * 0.4f)

        { Samples = samples; SampleRate = Rate }

    /// Warning beep for TipWarning.
    let warningBeep () =
        let samples =
            make 0.2f (fun _ t ->
                let envelope = MathF.Min(t * 60.0f, 1.0f) * (if t > 0.15f then 0.0f else 1.0f)
                MathF.Sin(2.0f * MathF.PI * 880.0f * t) * envelope * 0.4f)

        { Samples = samples; SampleRate = Rate }

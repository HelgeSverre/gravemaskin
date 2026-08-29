namespace Gravemaskin.Shell

open System
open Silk.NET.OpenAL
open Gravemaskin
open Gravemaskin.ProcGen

/// Event-driven audio: two looping beds (engine, hydraulics) modulated by
/// machine state, one-shots for events. If OpenAL init fails the game plays
/// on silently (house rule).
type AudioSystem(volume: float32) =
    let context = new AudioContext()
    let al = AL.GetApi()

    let upload (sound: AudioSynth.Sound) =
        let buffer = al.GenBuffer()
        al.BufferData(buffer, BufferFormat.Mono16, sound.Samples, sound.SampleRate)
        buffer

    let engineBuffer = upload (AudioSynth.engineLoop ())
    let hydraulicBuffer = upload (AudioSynth.hydraulicLoop ())
    let squealBuffer = upload (AudioSynth.reliefSqueal ())
    let pourBuffer = upload (AudioSynth.soilPour ())
    let clankBuffer = upload (AudioSynth.clank ())
    let beepBuffer = upload (AudioSynth.warningBeep ())

    let engineSource = al.GenSource()
    let hydraulicSource = al.GenSource()
    let oneShots = Array.init 8 (fun _ -> al.GenSource())
    let mutable oneShotIndex = 0
    let mutable squealCooldown = 0.0f
    let mutable beepCooldown = 0.0f
    let mutable pourCooldown = 0.0f

    do
        al.SetSourceProperty(engineSource, SourceBoolean.Looping, true)
        al.SetSourceProperty(engineSource, SourceInteger.Buffer, int engineBuffer)
        al.SetSourceProperty(engineSource, SourceFloat.Gain, 0.5f * volume)
        al.SourcePlay engineSource
        al.SetSourceProperty(hydraulicSource, SourceBoolean.Looping, true)
        al.SetSourceProperty(hydraulicSource, SourceInteger.Buffer, int hydraulicBuffer)
        al.SetSourceProperty(hydraulicSource, SourceFloat.Gain, 0.0f)
        al.SourcePlay hydraulicSource

    let playOneShot buffer gain =
        let source = oneShots.[oneShotIndex]
        oneShotIndex <- (oneShotIndex + 1) % oneShots.Length
        al.SourceStop source
        al.SetSourceProperty(source, SourceInteger.Buffer, int buffer)
        al.SetSourceProperty(source, SourceFloat.Gain, gain * volume)
        al.SourcePlay source

    /// Per-frame: engine pitch follows demand, whine follows flow.
    member _.Update(machine: Machine option, events: ResizeArray<GameEvent>, dt: float32) =
        squealCooldown <- max 0.0f (squealCooldown - dt)
        beepCooldown <- max 0.0f (beepCooldown - dt)
        pourCooldown <- max 0.0f (pourCooldown - dt)

        match machine with
        | Some m ->
            // Demand proxy: sum of granted scales weighted per function.
            let mutable demand = 0.0f

            for i in 0 .. Hydraulics.FunctionCount - 1 do
                demand <- demand + (1.0f - m.GrantedScale i) * 0.5f

            let working = if m.StallActive then 1.0f else demand
            al.SetSourceProperty(engineSource, SourceFloat.Pitch, 0.9f + working * 0.35f)
            al.SetSourceProperty(hydraulicSource, SourceFloat.Gain, MathF.Min(demand * 0.8f, 0.6f) * volume)
        | None -> ()

        for event in events do
            match event with
            | HydraulicStall when squealCooldown = 0.0f ->
                squealCooldown <- 0.8f
                playOneShot squealBuffer 0.5f
            | SoilDumped _ when pourCooldown = 0.0f ->
                pourCooldown <- 0.35f
                playOneShot pourBuffer 0.6f
            | RockStruck -> playOneShot clankBuffer 0.7f
            | TipWarning when beepCooldown = 0.0f ->
                beepCooldown <- 2.5f
                playOneShot beepBuffer 0.45f
            | _ -> ()

    interface IDisposable with
        member _.Dispose() =
            al.SourceStop engineSource
            al.SourceStop hydraulicSource

            for source in oneShots do
                al.SourceStop source
                al.DeleteSource source

            al.DeleteSource engineSource
            al.DeleteSource hydraulicSource

            for buffer in [| engineBuffer; hydraulicBuffer; squealBuffer; pourBuffer; clankBuffer; beepBuffer |] do
                al.DeleteBuffer buffer

            (context :> IDisposable).Dispose()

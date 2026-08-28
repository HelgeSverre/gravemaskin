namespace Gravemaskin

open System

/// Types only — no logic, no constants (those live in Tuning.fs).
[<AutoOpen>]
module Domain =

    type SoilMaterial =
        | Topsoil
        | DrySand
        | WetSand
        | Gravel
        | Clay

    /// Per-material physical description. Instances live in Tuning.SoilTable.
    type SoilProperties =
        { /// Bank (undisturbed) unit weight.
          BankDensity: float32<kg/m^3>
          /// Cohesion c — what lets clay stand in vertical walls.
          Cohesion: float32<kPa>
          /// Internal friction angle φ in radians. Doubles as the angle of repose.
          FrictionAngle: float32
          /// Swell factor S: loose volume = bank volume × (1+S).
          Swell: float32 }

    /// One hydraulic function's resolved command for this tick: the motor gets
    /// a velocity target and a force ceiling — exactly BEPU's servo model.
    [<Struct>]
    type ActuatorCommand =
        { TargetVelocity: float32
          MaxForce: float32 }

    /// Excavator data sheet. Machines are data; the rig code is generic.
    type MachineSpec =
        { Name: string
          OperatingMass: float32<kg>
          /// Independent pump circuits as (flow L/min, relief MPa). Functions
          /// hard-assigned to a circuit share only that circuit's flow.
          Circuits: (float32 * float32)[]
          BucketBreakout: float32<kN>
          ArmCrowd: float32<kN>
          SwingRpm: float32
          TravelKmhLowHigh: float32 * float32
          GroundPressure: float32<kPa>
          DigDepth: float32<m>
          Reach: float32<m> }

    [<Flags>]
    type InputButtons =
        | None = 0
        | ThrottleUp = 1
        | ThrottleDown = 2

    /// Everything the deterministic sim is allowed to see from the outside.
    /// Axes are -1..1 after the shell's deadzone; response curve and valve lag
    /// are applied inside the sim so replays stay bit-identical.
    [<Struct>]
    type InputFrame =
        { Sequence: int64
          Swing: float32
          Boom: float32
          Stick: float32
          Bucket: float32
          LeftTrack: float32
          RightTrack: float32
          Buttons: InputButtons }

    [<RequireQualifiedAccess>]
    module InputFrame =
        let empty =
            { Sequence = 0L
              Swing = 0.0f
              Boom = 0.0f
              Stick = 0.0f
              Bucket = 0.0f
              LeftTrack = 0.0f
              RightTrack = 0.0f
              Buttons = InputButtons.None }

    /// Presentation-facing facts. Renderer/audio consume these; they never
    /// read sim internals. Structs so the per-tick event buffer stays pooled.
    [<Struct>]
    type GameEvent =
        | DigStarted
        | HydraulicStall
        | TrackSlip
        | SoilDumped of massKg: float32
        | WallCollapsed
        | RockStruck
        | TipWarning

    /// POD snapshot handed to the shell each tick; the render thread
    /// interpolates between two of these (never between mutable worlds).
    [<Struct>]
    type RenderState =
        { Tick: int64
          BodyCount: int }

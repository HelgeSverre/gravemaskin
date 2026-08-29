namespace Gravemaskin

open System
open System.Numerics

/// Types only — no logic, no constants (those live in Tuning.fs).
[<AutoOpen>]
module Domain =

    type SoilMaterial =
        | Topsoil
        | DrySand
        | WetSand
        | Gravel
        | Clay
        | Grass

    /// Number of soil materials — sizes every per-material array (ledger,
    /// unbanked, payload, carve scratch). Keep in sync with SoilMaterial.
    [<Literal>]
    let MaterialCount = 6

    /// Per-material physical description. Instances live in Tuning.soil.
    /// Struct: Tuning.soil is called in per-cell hot loops — a reference
    /// record would allocate every call.
    [<Struct>]
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
        | SoilDumped of massKg: float32 * materialByte: byte
        | WallCollapsed
        | RockStruck
        | TipWarning

    type CylinderJoint =
        { /// Distance pivot → cylinder anchor on the parent link (m).
          Ra: float32
          /// Distance pivot → cylinder anchor on the child link (m).
          Rb: float32
          /// Angle between the two anchor rays when the joint angle is 0.
          AngleOffset: float32
          /// Cylinder bore diameter (m) — piston side area.
          Bore: float32
          /// Rod diameter (m) — annulus = piston − rod area (retract is weaker;
          /// operators genuinely notice, so the asymmetry is kept).
          Rod: float32
          /// Positive joint rotation = cylinder extend? (drives which area caps
          /// which direction)
          ExtendPositive: bool
          /// Joint angle limits (rad), enforced in software at the motor.
          MinAngle: float32
          MaxAngle: float32
          /// Hydraulic circuit index and this function's max flow share (L/min).
          Circuit: int
          QMax: float32 }

    /// Everything that defines a machine: physics geometry, masses, joints,
    /// drives. Machines are data — the rig code is generic.
    type MachineRig =
        { Spec: MachineSpec
          BoomJoint: CylinderJoint
          StickJoint: CylinderJoint
          BucketJoint: CylinderJoint
          SwingTorque: float32
          SwingMaxVel: float32
          SwingQMax: float32
          TrackMaxForce: float32
          TrackMaxSpeed: float32
          TrackQMax: float32
          TrackGain: float32
          /// Component masses (kg): chassis, house, boom, stick, bucket.
          Masses: float32 * float32 * float32 * float32 * float32
          /// Overall scale relative to the U17 assembly layout (part sizes
          /// and pivot offsets multiply by this).
          Scale: float32
          BucketTipRadius: float32
          BucketCapacityKg: float
          /// Angle past which the payload pours out.
          DumpAngle: float32
          DumpRatePerTick: float }

    /// POD snapshot handed to the shell each tick; the render thread
    /// interpolates between two of these (never between mutable worlds).
    [<Struct>]
    type RenderState =
        { Tick: int64
          BodyCount: int }

/// Shell-owned POD buffers for interpolated rendering (defined here because
/// World fills it; contains no GL types).
type RenderSnapshot(capacity: int) =
    member val Tick = 0L with get, set
    member val Count = 0 with get, set
    member val Capacity = capacity
    member val Handles = Array.zeroCreate<int> capacity
    member val X = Array.zeroCreate<float32> capacity
    member val Y = Array.zeroCreate<float32> capacity
    member val Z = Array.zeroCreate<float32> capacity
    member val Radius = Array.zeroCreate<float32> capacity
    member val VelocityX = Array.zeroCreate<float32> capacity
    member val VelocityY = Array.zeroCreate<float32> capacity
    member val VelocityZ = Array.zeroCreate<float32> capacity
    member val Materials = Array.zeroCreate<byte> capacity
    member val MachinePartCount = 0 with get, set
    member val MachinePositions = Array.zeroCreate<Vector3> 8
    member val MachineOrientations = Array.zeroCreate<Quaternion> 8
    member val MachineScale = 1.0f with get, set
    member val MachineName = "" with get, set
    member val PayloadKg = 0.0f with get, set
    member val PayloadCapacityKg = 1.0f with get, set
    member val PayloadMaterial = 0uy with get, set
    member val RockCount = 0 with get, set
    member val RockPositions = Array.zeroCreate<Vector3> 64
    member val RockRadii = Array.zeroCreate<float32> 64

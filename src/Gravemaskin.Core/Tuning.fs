namespace Gravemaskin

/// Every gameplay/physical constant, with rationale. Numbers come from the
/// research digest in docs/research/ (spec sheets and the Servin/agxTerrain
/// soil literature); change them here, never inline.
[<RequireQualifiedAccess>]
module Tuning =

    /// Fixed simulation rate. The whole game is built around this being
    /// constant; render interpolates between ticks.
    [<Literal>]
    let TickRate = 60

    let FixedDt = 1.0f / float32 TickRate

    /// BEPU solver: 8 velocity iterations × 4 substeps. Substepping is what
    /// keeps a 20:1 mass-ratio linkage stiff without cranking spring
    /// frequencies past stability (~half the substepped rate ≈ 120 Hz).
    let SolverVelocityIterations = 8
    let SolverSubsteps = 4

    /// Soil material table — Servin et al. values. FrictionAngle is radians;
    /// it is also the angle of repose used by the settling CA.
    let private deg d = d * System.MathF.PI / 180.0f

    let soil material =
        match material with
        | Gravel ->
            { BankDensity = 1800.0f<kg/m^3>
              Cohesion = 0.0f<kPa>
              FrictionAngle = deg 44.0f
              Swell = 0.15f }
        | DrySand ->
            { BankDensity = 1600.0f<kg/m^3>
              Cohesion = 0.0f<kPa>
              FrictionAngle = deg 39.0f
              Swell = 0.12f }
        | WetSand ->
            { BankDensity = 1900.0f<kg/m^3>
              Cohesion = 8.7f<kPa>
              FrictionAngle = deg 33.0f
              Swell = 0.12f }
        | Topsoil ->
            { BankDensity = 1400.0f<kg/m^3>
              Cohesion = 2.1f<kPa>
              FrictionAngle = deg 40.0f
              Swell = 0.25f }
        | Clay ->
            { BankDensity = 1750.0f<kg/m^3>
              Cohesion = 4.8f<kPa>
              FrictionAngle = deg 21.0f
              Swell = 0.30f }
        | Grass ->
            // Turf: topsoil bound by roots — noticeably more cohesive than
            // bare topsoil, a bit lighter, swells the most when torn up.
            { BankDensity = 1350.0f<kg/m^3>
              Cohesion = 6.5f<kPa>
              FrictionAngle = deg 38.0f
              Swell = 0.30f }

    /// Kubota U17-class mini excavator. First playable machine: small forces
    /// are solver-friendly, and 15.2 kN breakout against 17 kN of weight
    /// makes the machine visibly rock while digging.
    let u17 =
        { Name = "U17"
          OperatingMass = 1730.0f<kg>
          // Three hard-assigned circuits (L/min, MPa): P1/P2 piston pumps for
          // boom/stick/travel, P3 gear pump for swing/blade/aux. Flow sharing
          // within a circuit is what makes two-function moves slow down.
          Circuits = [| 17.3f, 21.6f; 17.3f, 21.6f; 10.4f, 18.6f |]
          BucketBreakout = 15.2f<kN>
          ArmCrowd = 8.5f<kN>
          SwingRpm = 9.1f
          TravelKmhLowHigh = 2.25f, 4.25f
          GroundPressure = 25.5f<kPa>
          DigDepth = 2.31f<m>
          Reach = 3.84f<m> }

    /// Cat 320-class 22-tonne excavator. Frozen now so MachineSpec stays
    /// honest about being data; playable in Phase 7.
    let cat320 =
        { Name = "Cat 320"
          OperatingMass = 21700.0f<kg>
          Circuits = [| 429.0f, 35.0f |]
          BucketBreakout = 150.0f<kN>
          ArmCrowd = 106.0f<kN>
          SwingRpm = 11.25f
          TravelKmhLowHigh = 3.2f, 5.5f
          GroundPressure = 40.0f<kPa>
          DigDepth = 6.72f<m>
          Reach = 9.86f<m> }

    /// U17 linkage joints. Anchor geometry is estimated from machine
    /// proportions; bores are chosen so the PUBLISHED breakout forces emerge
    /// from pressure × area × moment arm (verified by test, never hardcoded):
    /// bucket ≈ 15.8 kN at the tip (spec 15.2), stick crowd ≈ 9.3 kN
    /// (spec 8.5), boom torque ≈ 22 kN·m.
    let u17BoomJoint =
        { Ra = 0.35f
          Rb = 0.90f
          AngleOffset = 1.4f
          Bore = 0.065f
          Rod = 0.035f
          ExtendPositive = true
          MinAngle = -0.5f
          MaxAngle = 1.15f
          Circuit = 0
          QMax = 17.3f }

    let u17StickJoint =
        { Ra = 0.75f
          Rb = 0.28f
          AngleOffset = 2.6f
          Bore = 0.060f
          Rod = 0.032f
          ExtendPositive = false // extend crowds the stick in (negative θ)
          MinAngle = -2.3f
          MaxAngle = 0.25f
          Circuit = 1
          QMax = 17.3f }

    let u17BucketJoint =
        { Ra = 0.35f
          Rb = 0.22f
          AngleOffset = 2.0f
          Bore = 0.055f
          Rod = 0.030f
          ExtendPositive = false // extend curls the bucket in (negative θ)
          MinAngle = -2.4f
          MaxAngle = 0.35f
          Circuit = 0
          QMax = 17.3f }

    /// Swing drive: 9.1 rpm ≈ 0.95 rad/s, small machine ≈ 2.5 kN·m at its
    /// own (lower) relief. Swing brake = holding cap when idle.
    let u17SwingTorque = 2500.0f
    let u17SwingMaxVel = 0.95f
    let u17SwingQMax = 10.4f

    /// Track drive: velocity-servo per side, tractive cap ≈ μN per track.
    let u17TrackMaxForce = 6000.0f
    let u17TrackMaxSpeed = 1.18f // 4.25 km/h high range
    let u17TrackQMax = 8.0f
    /// Proportional gain: force ramps in over ~a few tenths of m/s of slip.
    let u17TrackGain = 20000.0f

    /// Component masses (kg): 860 undercarriage, 550 house+counterweight,
    /// 180 boom, 90 stick, 50 bucket = 1730 total (spec operating mass).
    let u17Masses = {| Chassis = 860.0f; House = 550.0f; Boom = 180.0f; Stick = 90.0f; Bucket = 50.0f |}

    /// Reference radii for validation: bucket pivot → cutting edge, and
    /// max reach used in stall tests.
    let u17BucketTipRadius = 0.7f

    /// Complete machine rigs. The U17 is the assembly-layout reference
    /// (Scale 1); the Cat 320 is ~2.4× the linear size with its own spec-
    /// sheet joints (two 120 mm boom cylinders folded into one equivalent
    /// bore; 35 MPa relief).
    let u17Rig =
        { Spec = u17
          BoomJoint = u17BoomJoint
          StickJoint = u17StickJoint
          BucketJoint = u17BucketJoint
          SwingTorque = u17SwingTorque
          SwingMaxVel = u17SwingMaxVel
          SwingQMax = u17SwingQMax
          TrackMaxForce = u17TrackMaxForce
          TrackMaxSpeed = u17TrackMaxSpeed
          TrackQMax = u17TrackQMax
          TrackGain = u17TrackGain
          Masses = 860.0f, 550.0f, 180.0f, 90.0f, 50.0f
          Scale = 1.0f
          BucketTipRadius = 0.7f
          BucketCapacityKg = 70.0
          DumpAngle = -0.35f
          DumpRatePerTick = 1.2 }

    let cat320Rig =
        { Spec = cat320
          BoomJoint =
            { Ra = 0.9f
              Rb = 2.3f
              AngleOffset = 1.4f
              Bore = 0.170f // two 120 mm cylinders, equivalent single bore
              Rod = 0.085f
              ExtendPositive = true
              MinAngle = -0.5f
              MaxAngle = 1.15f
              Circuit = 0
              QMax = 214.0f }
          StickJoint =
            { Ra = 1.9f
              Rb = 0.72f
              AngleOffset = 2.6f
              Bore = 0.140f
              Rod = 0.075f
              ExtendPositive = false
              MinAngle = -2.3f
              MaxAngle = 0.25f
              Circuit = 0
              QMax = 214.0f }
          BucketJoint =
            { Ra = 0.95f
              Rb = 0.58f
              AngleOffset = 2.0f
              Bore = 0.120f
              Rod = 0.060f
              ExtendPositive = false
              MinAngle = -2.4f
              MaxAngle = 0.35f
              Circuit = 0
              QMax = 160.0f }
          SwingTorque = 82000.0f
          SwingMaxVel = 1.18f // 11.25 rpm
          SwingQMax = 120.0f
          TrackMaxForce = 100000.0f // drawbar 205 kN across both tracks-ish
          TrackMaxSpeed = 1.53f // 5.5 km/h
          TrackQMax = 100.0f
          TrackGain = 250000.0f
          Masses = 9770.0f, 7980.0f, 1900.0f, 1110.0f, 940.0f
          Scale = 2.4f
          BucketTipRadius = 1.5f
          BucketCapacityKg = 1600.0
          DumpAngle = -0.35f
          DumpRatePerTick = 25.0 }

    let rigByName (name: string) =
        if name.Contains "320" then cat320Rig else u17Rig

    /// FEE calibration knobs (see Excavator/Fee.fs — folded N-factors).
    /// Expect hand-tuning; that is their job.
    let FeeGammaFactor = 6.0f
    let FeeCohesionFactor = 5.0f
    /// Low-pass on the dig resistance (per-tick lerp) so force never pops
    /// crossing cell boundaries.
    let FeeSmoothing = 0.25f
    /// Cutting geometry: effective blade width and carve radius per tick.
    let CutWidth = 0.6f
    let CutRadius = 0.28f
    /// Minimum edge speed (m/s) that counts as cutting.
    let CutMinSpeed = 0.12f
    /// U17 bucket payload ceiling (kg, loose) — ~0.05 m³ heaped.
    let BucketCapacityKg = 70.0
    /// Dump release rate (kg per tick) once the bucket opens — ~1 s to
    /// empty the U17's 70 kg, which reads as a pour instead of a dump-valve.
    let DumpRatePerTick = 1.2
    /// Bucket angle above which the load pours out.
    let DumpAngle = -0.35f

    /// Input shaping (applied inside the sim for determinism):
    /// deadband → power curve → first-order valve lag.
    let InputDeadband = 0.10f
    /// x^1.7 gives fine control near center without killing top speed.
    let InputCurveExponent = 1.7f
    /// Real pilot-valve response is ~100 ms; feathering falls out of this.
    let ValveLagSeconds = 0.1f<s>

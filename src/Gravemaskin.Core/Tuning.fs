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

    /// Input shaping (applied inside the sim for determinism):
    /// deadband → power curve → first-order valve lag.
    let InputDeadband = 0.10f
    /// x^1.7 gives fine control near center without killing top speed.
    let InputCurveExponent = 1.7f
    /// Real pilot-valve response is ~100 ms; feathering falls out of this.
    let ValveLagSeconds = 0.1f<s>

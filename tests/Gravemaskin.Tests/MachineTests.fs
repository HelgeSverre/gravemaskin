module Gravemaskin.Tests.MachineTests

open System
open System.Numerics
open Xunit
open Gravemaskin

let private input boom stick bucket swing trackL trackR =
    { InputFrame.empty with
        Boom = boom
        Stick = stick
        Bucket = bucket
        Swing = swing
        LeftTrack = trackL
        RightTrack = trackR }

/// World on the rigid slab with a machine at the origin, settled for a
/// second so spawn sag and contact resolution are done.
let private machineWorld () =
    let world = Sim.createWorld TestKit.defaultSeed
    let machine = world.SpawnMachine(Vector3.Zero)
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    world, machine

/// Apply a downward force (N) at the bucket tip during a step loop.
let private stepWithTipLoad (world: World) (machine: Machine) (frame: InputFrame) (loadN: float32) (ticks: int) =
    for _ in 1..ticks do
        let tip = machine.BucketTipPosition
        let mutable bucketRef = world.Physics.Simulation.Bodies.[machine.Bucket]

        if not bucketRef.Awake then
            bucketRef.Awake <- true

        bucketRef.ApplyImpulse(Vector3(0.0f, -loadN * Tuning.FixedDt, 0.0f), tip - bucketRef.Pose.Position)
        world.Step frame |> ignore

// ---- hydraulics unit level ----

[<Fact>]
let ``retract force is weaker than extend (rod-side annulus)`` () =
    let joint = Tuning.u17BoomJoint
    let extend = Linkage.torqueCap joint 0.3f 1.0f 21.6e6f
    let retract = Linkage.torqueCap joint 0.3f -1.0f 21.6e6f
    Assert.True(retract < extend * 0.85f, $"extend {extend}, retract {retract}")

[<Fact>]
let ``input shaping has a deadband and a progressive curve`` () =
    Assert.Equal(0.0f, Hydraulics.shapeAxis 0.05f)
    Assert.True(Hydraulics.shapeAxis 0.5f < 0.5f) // progressive under-response
    Assert.True(Hydraulics.shapeAxis 1.0f > 0.99f)
    Assert.True(Hydraulics.shapeAxis -1.0f < -0.99f)

// ---- rig behavior ----

[<Fact>]
let ``boom lifts the arm and holds it when the stick goes dead`` () =
    let world, machine = machineWorld ()
    use _ = world
    let sagged = machine.BoomAngle
    TestKit.stepAll 240 (input 1.0f 0.0f 0.0f 0.0f 0.0f 0.0f) world |> ignore
    let raised = machine.BoomAngle
    Assert.True(raised > sagged + 0.3f, $"boom should rise: {sagged} -> {raised}")

    // Dead stick: check-valve hold, no droop.
    TestKit.stepAll 240 InputFrame.empty world |> ignore
    Assert.True(MathF.Abs(machine.BoomAngle - raised) < 0.1f, $"boom drooped: {raised} -> {machine.BoomAngle}")

[<Fact>]
let ``boom torque ceiling emerges from pressure × area × moment arm`` () =
    // Analytic cap ≈ 24.5 kN·m at the spawn pose (never hardcoded in the
    // rig). Commanding boom-up against a pure resisting torque: 15 kN·m is
    // overcome, 30 kN·m is not. (Tip-force framing doesn't work here: the
    // moment arm of a tip load collapses as the boom rises.)
    let riseAgainst (resistTorque: float32) =
        let world, machine = machineWorld ()
        use _ = world

        // 450 ticks: the limit-restoring stick motor (a later fix) reacts
        // through the linkage and slows the boom's early rise a little.
        for _ in 1..450 do
            let mutable boomRef = world.Physics.Simulation.Bodies.[machine.Boom]

            if not boomRef.Awake then
                boomRef.Awake <- true

            boomRef.ApplyAngularImpulse(Vector3(0.0f, 0.0f, -resistTorque * Tuning.FixedDt))
            world.Step(input 1.0f 0.0f 0.0f 0.0f 0.0f 0.0f) |> ignore

        machine.BoomAngle

    Assert.True(riseAgainst 15000.0f > 0.4f, $"15 kN·m should be overcome: {riseAgainst 15000.0f}")
    Assert.True(riseAgainst 30000.0f < 0.0f, $"30 kN·m should stall the boom: {riseAgainst 30000.0f}")

[<Fact>]
let ``bucket breakout torque emerges within spec brackets`` () =
    // Published U17 breakout 15.2 kN at 0.7 m tip ⇒ ~10.6 kN·m at the
    // joint; the linkage cap runs ≈ 9.4→11 kN·m through the working range.
    // The resistance is a solver-integrated friction brake: a second motor
    // on the same joint with TargetVelocity 0 and MaxForce = brakeTorque.
    // Drive cap > brake ⇒ the joint still moves; brake ≥ drive ⇒ it holds.
    // (Every hand-rolled impulse harness tried here injected or ratcheted
    // energy at tick granularity; letting the solver arbitrate is exact.)
    let curlAgainst (brakeTorque: float32) =
        let world, machine = machineWorld ()
        use _ = world
        // Hold the bucket clear of the ground so contact doesn't add brake.
        TestKit.stepAll 150 (input 1.0f 0.0f 0.0f 0.0f 0.0f 0.0f) world |> ignore

        let mutable guard = 0

        while machine.BucketAngle > -0.4f && guard < 300 do
            world.Step(input 0.0f 0.0f -1.0f 0.0f 0.0f 0.0f) |> ignore
            guard <- guard + 1

        let brake =
            Bepu.addAngularMotor world.Physics.Simulation machine.Stick machine.Bucket Vector3.UnitZ

        Bepu.retuneAngularMotor
            world.Physics.Simulation
            brake
            Vector3.UnitZ
            { TargetVelocity = 0.0f
              MaxForce = brakeTorque }

        TestKit.stepAll 300 (input 0.0f 0.0f -1.0f 0.0f 0.0f 0.0f) world |> ignore
        machine.BucketAngle

    Assert.True(curlAgainst 8500.0f < -1.3f, $"8.5 kN·m brake should be overcome: {curlAgainst 8500.0f}")
    Assert.True(curlAgainst 13000.0f > -1.1f, $"13 kN·m brake should hold the curl: {curlAgainst 13000.0f}")

[<Fact>]
let ``two functions on one circuit share flow; different circuits do not`` () =
    // Boom and bucket share circuit 0; stick is circuit 1.
    let boomProgress (frame: InputFrame) =
        let world, machine = machineWorld ()
        use _ = world
        let start = machine.BoomAngle
        TestKit.stepAll 150 frame world |> ignore
        machine.BoomAngle - start

    let solo = boomProgress (input 1.0f 0.0f 0.0f 0.0f 0.0f 0.0f)
    let sameCircuit = boomProgress (input 1.0f 0.0f -1.0f 0.0f 0.0f 0.0f)
    let crossCircuit = boomProgress (input 1.0f -1.0f 0.0f 0.0f 0.0f 0.0f)

    Assert.True(sameCircuit < solo * 0.7f, $"same-circuit should slow the boom: solo {solo}, shared {sameCircuit}")

    Assert.True(
        MathF.Abs(crossCircuit - solo) < solo * 0.2f,
        $"cross-circuit should not: solo {solo}, cross {crossCircuit}"
    )

[<Fact>]
let ``tracks drive the machine forward, roughly straight`` () =
    let world, machine = machineWorld ()
    use _ = world
    let start = world.Physics.Simulation.Bodies.[machine.Chassis].Pose.Position
    TestKit.stepAll 300 (input 0.0f 0.0f 0.0f 0.0f 1.0f 1.0f) world |> ignore
    let finish = world.Physics.Simulation.Bodies.[machine.Chassis].Pose.Position
    let moved = finish - start
    Assert.True(moved.X > 1.5f, $"should drive forward: {moved}")
    Assert.True(MathF.Abs moved.Z < moved.X * 0.25f, $"should be roughly straight: {moved}")

[<Fact>]
let ``opposite tracks pivot the machine in place`` () =
    let world, machine = machineWorld ()
    use _ = world
    let chassisRef = world.Physics.Simulation.Bodies.[machine.Chassis]
    let startForward = Vector3.Transform(Vector3.UnitX, chassisRef.Pose.Orientation)
    TestKit.stepAll 400 (input 0.0f 0.0f 0.0f 0.0f 1.0f -1.0f) world |> ignore
    let endForward = Vector3.Transform(Vector3.UnitX, chassisRef.Pose.Orientation)

    let yawChange =
        MathF.Atan2(endForward.Z, endForward.X) - MathF.Atan2(startForward.Z, startForward.X)

    let position = chassisRef.Pose.Position

    Assert.True(MathF.Abs yawChange > 0.4f, $"should pivot: yaw change {yawChange}")
    Assert.True(position.Length() < 1.5f, $"pivot should stay in place: {position}")

[<Fact>]
let ``a heavy over-side load tips the machine; unloaded it stays level`` () =
    // Swing the house 90° so the arm hangs over the tracks' side, then hang
    // 500 kg at the tip: the support polygon is only ±0.7 m that way and the
    // machine goes over. No load → it stays up. No tipping code exists.
    // Operator pose, not spawn pose: crowd the stick (its cylinder arm is
    // weak at full extension — really), raise the boom so the tip is
    // airborne (a grounded tip routes the load into the ground), THEN load.
    let tiltAfter (loadN: float32) =
        let world, machine = machineWorld ()
        use _ = world
        TestKit.stepAll 140 (input 0.0f 0.0f 0.0f 1.0f 0.0f 0.0f) world |> ignore
        TestKit.stepAll 90 (input 0.0f -1.0f 0.0f 0.0f 0.0f 0.0f) world |> ignore
        TestKit.stepAll 150 (input 1.0f 0.0f 0.0f 0.0f 0.0f 0.0f) world |> ignore
        stepWithTipLoad world machine InputFrame.empty loadN 500
        machine.ChassisTilt

    Assert.True(tiltAfter 3800.0f > 0.5f, $"390 kg over the side should tip: tilt {tiltAfter 3800.0f}")
    Assert.True(tiltAfter 0.0f < 0.15f, $"unloaded machine should stay level: tilt {tiltAfter 0.0f}")

[<Fact>]
let ``idle machine falls asleep`` () =
    let world, machine = machineWorld ()
    use _ = world
    TestKit.stepAll 600 InputFrame.empty world |> ignore
    let bodies = world.Physics.Simulation.Bodies

    for handle in [| machine.Chassis; machine.House; machine.Boom; machine.Stick; machine.Bucket |] do
        Assert.False(bodies.[handle].Awake, $"body {handle.Value} still awake after 10 s idle")

[<Fact>]
[<Trait("Category", "Integration")>]
let ``full rig determinism: 10k adversarial ticks bit-identical`` () =
    let run () =
        let world, _ = machineWorld ()
        use _ = world

        for tick in 0..9_999 do
            // Full-speed direction reversals on everything at once.
            let sign = if (tick / 30) % 2 = 0 then 1.0f else -1.0f

            world.Step(input sign -sign sign -sign sign -sign) |> ignore

        world.Physics.HashBodyPoses()

    Assert.Equal(run (), run ())

[<Fact>]
let ``machine stepping does not allocate`` () =
    let world, _ = machineWorld ()
    use _ = world
    TestKit.stepAll 300 (input 0.5f -0.5f 0.5f 0.3f 0.2f -0.2f) world |> ignore
    GC.Collect()
    GC.WaitForPendingFinalizers()
    GC.Collect()
    let gen0 = GC.CollectionCount 0

    for tick in 0..999 do
        world.Step(TestKit.scriptedInput tick) |> ignore

    Assert.Equal(gen0, GC.CollectionCount 0)

[<Fact>]
let ``no external force can bend a joint past its stroke limits`` () =
    // Real hydraulic cylinders bottom out mechanically: exceeding stroke
    // means bursting steel, so it simply doesn't happen. Abuse every joint
    // with torque far beyond anything the sim can produce and require the
    // hard TwistLimit constraints to hold within a small compliance margin.
    let world, machine = machineWorld ()
    use _ = world
    let bodies = world.Physics.Simulation.Bodies

    // Torque scale: ~2× the worst in-sim transient per joint (motor caps
    // are 24.5/16.7/11 kN·m; contact spikes are of that order). Unbounded
    // lump impulses would just inject teleport-grade spin between substeps
    // that no positional constraint can be expected to catch.
    let abuse = [| 40_000.0f; 15_000.0f; 12_000.0f |]

    for direction in [| 1.0f; -1.0f |] do
        for _ in 1..300 do
            let joints = [| machine.Boom; machine.Stick; machine.Bucket |]

            for j in 0..2 do
                let mutable bodyRef = bodies.[joints.[j]]

                if not bodyRef.Awake then
                    bodyRef.Awake <- true

                bodyRef.ApplyAngularImpulse(Vector3(0.0f, 0.0f, direction * abuse.[j] * Tuning.FixedDt))

            world.Step InputFrame.empty |> ignore

        let margin = 0.2f
        let boom = Tuning.u17BoomJoint
        let stick = Tuning.u17StickJoint
        let bucket = Tuning.u17BucketJoint
        Assert.InRange(machine.BoomAngle, boom.MinAngle - margin, boom.MaxAngle + margin)
        Assert.InRange(machine.StickAngle, stick.MinAngle - margin, stick.MaxAngle + margin)
        Assert.InRange(machine.BucketAngle, bucket.MinAngle - margin, bucket.MaxAngle + margin)

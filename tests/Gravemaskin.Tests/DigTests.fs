module Gravemaskin.Tests.DigTests

open System
open System.Numerics
open Xunit
open Gravemaskin

let private inp b s bk sw =
    { InputFrame.empty with
        Boom = b
        Stick = s
        Bucket = bk
        Swing = sw }

let private digWorld (mat: SoilMaterial) =
    let world = TestKit.soilWorld mat
    let machine = world.SpawnMachine(Vector3(8.0f, 0.0f, 8.0f))
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    world, machine

[<Fact>]
[<Trait("Category", "Integration")>]
let ``full dig-swing-dump cycle: payload fills, pours out, mass conserved`` () =
    let world, machine = digWorld Topsoil
    use _ = world
    let state = world.SoilState.Value
    let startSurface = Soil.surfaceHeight state 11.9f 8.0f

    // Curl in the dirt, boom down, crowd the stick — the scripted dig pass
    // (script validated interactively; the assertions are the contract).
    TestKit.stepAll 60 (inp 0.0f 0.0f -1.0f 0.0f) world |> ignore
    TestKit.stepAll 90 (inp -1.0f 0.0f 0.0f 0.0f) world |> ignore
    TestKit.stepAll 240 (inp 0.0f -1.0f -0.6f 0.0f) world |> ignore

    Assert.True(machine.BucketLoadKg > 20.0, $"bucket should fill while digging: {machine.BucketLoadKg} kg")

    let dugSurface = Soil.surfaceHeight state 11.9f 8.0f
    // Threshold recalibrated for the plate-compound bucket (the solid box
    // cut a slightly deeper scripted pass); the hole is what matters.
    Assert.True(dugSurface < startSurface - 0.12f, $"a hole should form: {startSurface} -> {dugSurface}")

    // Lift, swing away, open the bucket, let everything settle.
    TestKit.stepAll 120 (inp 1.0f 0.0f -0.3f 0.0f) world |> ignore
    TestKit.stepAll 100 (inp 0.0f 0.0f 0.0f 1.0f) world |> ignore
    let loadBeforeDump = machine.BucketLoadKg
    TestKit.stepAll 250 (inp 0.0f 0.0f 1.0f 0.0f) world |> ignore
    Assert.True(machine.BucketLoadKg < loadBeforeDump * 0.1, $"bucket should pour out: {machine.BucketLoadKg} kg left")

    TestKit.stepAll 300 InputFrame.empty world |> ignore
    let error = TestKit.conservationError world
    Assert.True(error < 1e-6, $"end-to-end conservation including the load scalar: {error}")

[<Fact>]
let ``payload makes the bucket heavier for the hydraulics`` () =
    let world, machine = digWorld Topsoil
    use _ = world
    // Dig until loaded.
    TestKit.stepAll 60 (inp 0.0f 0.0f -1.0f 0.0f) world |> ignore
    TestKit.stepAll 90 (inp -1.0f 0.0f 0.0f 0.0f) world |> ignore
    Assert.True(machine.BucketLoadKg > 20.0)
    // The inertia refresh must have pushed the payload into the body.
    let inertia = world.Physics.Simulation.Bodies.[machine.Bucket].LocalInertia
    let effectiveMass = 1.0f / inertia.InverseMass
    Assert.True(effectiveMass > 60.0f, $"bucket body should weigh bucket+payload: {effectiveMass} kg")

[<Fact>]
let ``the machine jacks its own front up on the arm (emergent self-lift)`` () =
    // Crowd the stick into rigid ground: the crowd force exceeds the
    // machine's front-weight leverage, so the chassis rotates up on the arm.
    // No special-case code — force caps and gravity.
    let world = Sim.createWorld TestKit.defaultSeed
    use _ = world
    let machine = world.SpawnMachine(Vector3.Zero)
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    TestKit.stepAll 250 (inp 0.0f -1.0f 0.0f 0.0f) world |> ignore
    Assert.True(machine.ChassisTilt > 0.25f, $"stick-crowd should lift the front: tilt {machine.ChassisTilt}")

[<Fact>]
let ``deep wet sand stalls the stick while the swing keeps moving`` () =
    // FEE resistance in a deep cohesive cut exceeds the stick's torque cap;
    // swing is on its own circuit and keeps its speed — the flow model and
    // force caps interacting, per the MVP gate.
    let world, machine = digWorld WetSand
    use _ = world
    // Bury the edge deep.
    TestKit.stepAll 60 (inp 0.0f 0.0f -1.0f 0.0f) world |> ignore
    TestKit.stepAll 140 (inp -1.0f 0.0f 0.0f 0.0f) world |> ignore

    let mutable stallSeen = false
    let stickBefore = machine.StickAngle

    for _ in 1..300 do
        world.Step(inp 0.0f -1.0f 0.0f 0.3f) |> ignore

        if world.Events.Contains HydraulicStall then
            stallSeen <- true

    Assert.True(stallSeen, "deep cohesive cut should stall a cylinder at some point")

    // The stick specifically is what's stuck: 5 s of full crowd command in
    // free air covers ~2 rad; in the cut it barely creeps.
    let stickMoved = abs (machine.StickAngle - stickBefore)
    Assert.True(stickMoved < 0.5f, $"the stick should be pinned by the cut: moved {stickMoved}")

    // Flow independence (not mechanical — a buried arm really does anchor
    // the swing): the saturated dig circuit never starves the swing's own.
    Assert.Equal(1.0f, machine.GrantedScale Hydraulics.Swing)

[<Fact>]
let ``digging under the machine's own tracks drops it onto the new surface`` () =
    let world, machine = digWorld Topsoil
    use _ = world
    let chassisRef = world.Physics.Simulation.Bodies.[machine.Chassis]
    let before = chassisRef.Pose.Position

    // Carve the ground out from under the tracks — wide enough that the
    // chassis corners can't bridge the hole (they really do otherwise).
    for tick in 0..149 do
        if tick % 5 = 0 then
            let y = 1.9f - float32 tick * 0.004f
            world.CarveSphere(Vector3(before.X - 0.5f, y, before.Z), 1.2f) |> ignore
            world.CarveSphere(Vector3(before.X + 0.5f, y, before.Z), 1.2f) |> ignore

        world.Step InputFrame.empty |> ignore

    TestKit.stepAll 200 InputFrame.empty world |> ignore
    let after = chassisRef.Pose.Position
    let surface = Soil.surfaceHeight world.SoilState.Value after.X after.Z

    Assert.True(after.Y < before.Y - 0.1f, $"machine should sink into its own excavation: {before.Y} -> {after.Y}")
    Assert.True(after.Y > surface - 0.5f, $"but never fall through: y {after.Y} vs surface {surface}")
    Assert.True(TestKit.conservationError world < 1e-6)

/// The gameplay pipeline end to end: machine + soil + FEE + payload +
/// compaction + rocks, hashed. This is the determinism claim's real gate —
/// pose-only and soil-only sessions each miss half the system.
let private runMachineDigSession (ticks: int) =
    let world = TestKit.soilWorld Topsoil
    let machine = world.SpawnMachine(Vector3(8.0f, 0.0f, 8.0f))
    world.SeedRocks 6
    use _ = world
    ignore machine

    for tick in 0 .. ticks - 1 do
        let frame =
            match (tick / 90) % 5 with
            | 0 -> inp 0.0f 0.0f -1.0f 0.0f
            | 1 -> inp -1.0f 0.0f -0.4f 0.0f
            | 2 -> inp 0.0f -1.0f -0.6f 0.0f
            | 3 -> inp 1.0f 0.0f 1.0f 0.6f
            | _ ->
                { inp 0.0f 0.0f 0.0f 0.0f with
                    LeftTrack = 1.0f
                    RightTrack = -0.5f }

        world.Step { frame with Sequence = int64 tick } |> ignore

    struct (world.Physics.HashBodyPoses(), TestKit.hashSoil world)

[<Fact>]
let ``1k machine-digging ticks: poses AND soil bit-identical across runs`` () =
    let struct (poses1, soil1) = runMachineDigSession 1_000
    let struct (poses2, soil2) = runMachineDigSession 1_000
    Assert.Equal(poses1, poses2)
    Assert.Equal(soil1, soil2)

[<Fact>]
[<Trait("Category", "Integration")>]
let ``10k machine-digging ticks: poses AND soil bit-identical across runs`` () =
    let struct (poses1, soil1) = runMachineDigSession 10_000
    let struct (poses2, soil2) = runMachineDigSession 10_000
    Assert.Equal(poses1, poses2)
    Assert.Equal(soil1, soil2)

[<Fact>]
let ``digging a loaded world still allocates nothing`` () =
    let world, _ = digWorld Topsoil
    use _ = world
    TestKit.stepAll 60 (inp 0.0f 0.0f -1.0f 0.0f) world |> ignore
    TestKit.stepAll 300 (inp -0.5f -0.5f -0.5f 0.2f) world |> ignore
    GC.Collect()
    GC.WaitForPendingFinalizers()
    GC.Collect()
    let gen0 = GC.CollectionCount 0
    // CollectionCount alone lets sub-budget leaks hide for years (review
    // finding: a 24 B/tick closure passed every gate). Measure actual bytes.
    let bytesBefore = GC.GetAllocatedBytesForCurrentThread()

    for tick in 0..599 do
        let phase = (tick / 100) % 4

        let frame =
            match phase with
            | 0 -> inp 0.0f -1.0f -0.6f 0.0f
            | 1 -> inp 1.0f 0.0f -0.3f 0.0f
            | 2 -> inp 0.0f 0.0f 1.0f 0.5f
            | _ -> inp -1.0f 0.5f 0.0f -0.5f

        world.Step frame |> ignore

    let allocated = GC.GetAllocatedBytesForCurrentThread() - bytesBefore
    Assert.Equal(gen0, GC.CollectionCount 0)

    // Byte-exact gate only in Release: Debug F# codegen allocates closures
    // the optimizer removes. `just perf` runs this in Release.
    if TestKit.isReleaseBuild then
        Assert.True(allocated < 2048L, $"steady-state digging allocated {allocated} bytes over 600 ticks")

[<Fact>]
let ``a curled bucket physically cradles a clump-sized body`` () =
    // The open-plate compound at work: drop a ball into a deep-curled
    // bucket held off the ground — it must stay in the bucket, not roll out.
    let world = Sim.createWorld TestKit.defaultSeed
    use _ = world
    let machine = world.SpawnMachine(Vector3.Zero)
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    // Raise the boom, then curl deep.
    TestKit.stepAll 180 (inp 1.0f 0.0f 0.0f 0.0f) world |> ignore
    TestKit.stepAll 240 (inp 0.0f 0.0f -1.0f 0.0f) world |> ignore
    Assert.True(machine.BucketAngle < -1.5f, $"bucket should be curled: {machine.BucketAngle}")

    // Spawn inside the cavity (the compound's COM sits in it), with a
    // nudge toward the opening's interior — a vertical drop from above can
    // glance off a plate edge and miss.
    let bucketRef = world.Physics.Simulation.Bodies.[machine.Bucket]

    let dropPoint =
        bucketRef.Pose.Position
        + Vector3.Transform(Vector3(-0.05f, 0.08f, 0.0f), bucketRef.Pose.Orientation)

    let ball = Bepu.addDynamicSphere world.Physics.Simulation dropPoint 0.09f 8.0f
    TestKit.stepAll 300 (inp 0.0f 0.0f 0.0f 0.0f) world |> ignore

    let ballPosition = world.Physics.Simulation.Bodies.[ball].Pose.Position
    let bucketCenter = world.Physics.Simulation.Bodies.[machine.Bucket].Pose.Position
    let distance = Vector3.Distance(ballPosition, bucketCenter)
    Assert.True(distance < 0.55f, $"the ball should rest IN the bucket: {distance} m from center")
    Assert.True(ballPosition.Y > 0.5f, $"held aloft, not on the ground: y {ballPosition.Y}")

[<Fact>]
let ``a curled bucket scoops loose clumps into the payload`` () =
    // Soil world: spill clumps, curl the bucket around them → they convert
    // to payload mass (clump budget freed, weight still on the linkage).
    let world, machine = digWorld Topsoil
    use _ = world
    // Fill by digging, dump nearby to make a loose pile, then re-scoop it.
    TestKit.stepAll 60 (inp 0.0f 0.0f -1.0f 0.0f) world |> ignore
    Assert.True(machine.BucketLoadKg > 20.0)
    TestKit.stepAll 90 (inp 1.0f 0.0f 0.0f 0.0f) world |> ignore
    TestKit.stepAll 150 (inp 0.0f 0.0f 1.0f 0.0f) world |> ignore
    let afterDump = machine.BucketLoadKg
    Assert.True(afterDump < 5.0, $"dumped: {afterDump} kg left")

    // Bring the bucket down into the spill and curl.
    TestKit.stepAll 120 (inp -1.0f 0.0f -0.6f 0.0f) world |> ignore
    TestKit.stepAll 240 (inp 0.0f -0.4f -1.0f 0.0f) world |> ignore

    Assert.True(
        machine.BucketLoadKg > afterDump + 5.0,
        $"re-scooping the pile should refill the payload: {machine.BucketLoadKg} kg"
    )
    Assert.True(TestKit.conservationError world < 1e-6)

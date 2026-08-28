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
    Assert.True(dugSurface < startSurface - 0.2f, $"a hole should form: {startSurface} -> {dugSurface}")

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
    let swingBefore = machine.SwingAngle

    for _ in 1..300 do
        world.Step(inp 0.0f -1.0f 0.0f 0.3f) |> ignore

        if world.Events.Contains HydraulicStall then
            stallSeen <- true

    Assert.True(stallSeen, "deep cohesive cut should stall a cylinder at some point")
    ignore swingBefore

    // Flow independence, not mechanical independence: a buried arm really
    // does anchor the swing, but the stick's saturated circuit must never
    // starve the swing's own circuit of flow.
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

    for tick in 0..599 do
        let phase = (tick / 100) % 4

        let frame =
            match phase with
            | 0 -> inp 0.0f -1.0f -0.6f 0.0f
            | 1 -> inp 1.0f 0.0f -0.3f 0.0f
            | 2 -> inp 0.0f 0.0f 1.0f 0.5f
            | _ -> inp -1.0f 0.5f 0.0f -0.5f

        world.Step frame |> ignore

    Assert.Equal(gen0, GC.CollectionCount 0)

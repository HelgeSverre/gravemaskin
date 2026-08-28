module Gravemaskin.Tests.CoreTests

open System
open System.Numerics
open Xunit
open Gravemaskin

[<Fact>]
let ``dropped ball comes to rest on the ground`` () =
    use world = TestKit.flatWorld ()

    let ball =
        Bepu.addDynamicSphere world.Physics.Simulation (Vector3(0.0f, 5.0f, 0.0f)) 0.5f 10.0f

    TestKit.stepAll 300 InputFrame.empty world |> ignore
    let y = world.Physics.Simulation.Bodies.[ball].Pose.Position.Y
    Assert.InRange(y, 0.35f, 0.65f)

[<Fact>]
let ``hinged arm driven by angular motor swings and can be retuned`` () =
    use world = TestKit.flatWorld ()
    let simulation = world.Physics.Simulation
    let anchor = Bepu.addKinematicBox simulation (Vector3(0.0f, 5.0f, 0.0f)) (Vector3(0.5f, 0.5f, 0.5f))
    let arm = Bepu.addDynamicBox simulation (Vector3(0.0f, 3.9f, 0.0f)) (Vector3(0.4f, 2.0f, 0.4f)) 10.0f

    Bepu.addHinge simulation anchor arm Vector3.UnitX (Vector3(0.0f, -0.25f, 0.0f)) (Vector3(0.0f, 1.1f, 0.0f))
    |> ignore

    let motor = Bepu.addAngularMotor simulation anchor arm Vector3.UnitX

    Bepu.retuneAngularMotor simulation motor Vector3.UnitX { TargetVelocity = 1.0f; MaxForce = 50.0f }
    TestKit.stepAll 120 InputFrame.empty world |> ignore
    let swungZ = simulation.Bodies.[arm].Pose.Position.Z

    Bepu.retuneAngularMotor simulation motor Vector3.UnitX { TargetVelocity = -1.0f; MaxForce = 50.0f }
    TestKit.stepAll 120 InputFrame.empty world |> ignore
    let returnedZ = simulation.Bodies.[arm].Pose.Position.Z

    Assert.True(abs swungZ > 0.2f, $"arm should swing away from rest, z={swungZ}")
    Assert.True(abs (returnedZ - swungZ) > 0.2f, $"retuned motor should move the arm back, z={returnedZ}")

[<Fact>]
let ``force-capped motor stalls under a load it cannot lift`` () =
    // The hydraulic model in miniature: MaxForce is the relief valve. A weak
    // motor must NOT hold a heavy arm horizontal against gravity.
    use world = TestKit.flatWorld ()
    let simulation = world.Physics.Simulation
    let anchor = Bepu.addKinematicBox simulation (Vector3(0.0f, 10.0f, 0.0f)) (Vector3(0.5f, 0.5f, 0.5f))

    let arm =
        Bepu.addDynamicBox simulation (Vector3(0.0f, 8.9f, 0.0f)) (Vector3(0.4f, 2.0f, 0.4f)) 100.0f

    Bepu.addHinge simulation anchor arm Vector3.UnitX (Vector3(0.0f, -0.25f, 0.0f)) (Vector3(0.0f, 1.1f, 0.0f))
    |> ignore

    let motor = Bepu.addAngularMotor simulation anchor arm Vector3.UnitX

    // ~1 N·m ceiling against a 100 kg arm: must stall (arm hangs, no lift).
    Bepu.retuneAngularMotor simulation motor Vector3.UnitX { TargetVelocity = 2.0f; MaxForce = 1.0f }
    TestKit.stepAll 300 InputFrame.empty world |> ignore
    let weakY = simulation.Bodies.[arm].Pose.Position.Y
    Assert.True(weakY < 9.2f, $"weak motor must not lift the arm, y={weakY}")

[<Fact>]
let ``machine spec numbers are wired`` () =
    Assert.Equal(1730.0f<kg>, Tuning.u17.OperatingMass)
    Assert.Equal(3, Tuning.u17.Circuits.Length)
    Assert.Equal(150.0f<kN>, Tuning.cat320.BucketBreakout)

[<Fact>]
let ``soil table has sane physicality`` () =
    for material in [ Topsoil; DrySand; WetSand; Gravel; Clay ] do
        let soil = Tuning.soil material
        Assert.InRange(float32 soil.BankDensity, 1000.0f, 2500.0f)
        Assert.InRange(soil.FrictionAngle, 0.3f, 0.9f)
        Assert.InRange(soil.Swell, 0.05f, 0.5f)

    // Clay stands walls, sand does not: cohesion ordering matters downstream.
    Assert.True((Tuning.soil Clay).Cohesion > (Tuning.soil DrySand).Cohesion)

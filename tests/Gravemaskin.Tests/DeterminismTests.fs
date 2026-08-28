module Gravemaskin.Tests.DeterminismTests

open System
open System.Numerics
open Xunit
open Gravemaskin

/// Builds a world with some dynamic clutter and runs a scripted session,
/// returning the pose hash. Used twice: identical results required.
let private runSession (ticks: int) =
    use world = TestKit.flatWorld ()
    let simulation = world.Physics.Simulation

    // A pile of spheres and boxes that will collide, roll, and sleep.
    for i in 0..19 do
        let x = float32 (i % 5) * 0.9f - 2.0f
        let z = float32 (i / 5) * 0.9f - 2.0f

        Bepu.addDynamicSphere simulation (Vector3(x, 2.0f + float32 i * 0.7f, z)) 0.4f 5.0f
        |> ignore

    let anchor = Bepu.addKinematicBox simulation (Vector3(0.0f, 6.0f, 0.0f)) (Vector3(0.5f, 0.5f, 0.5f))
    let arm = Bepu.addDynamicBox simulation (Vector3(0.0f, 4.9f, 0.0f)) (Vector3(0.4f, 2.0f, 0.4f)) 20.0f

    Bepu.addHinge simulation anchor arm Vector3.UnitX (Vector3(0.0f, -0.25f, 0.0f)) (Vector3(0.0f, 1.1f, 0.0f))
    |> ignore

    let motor = Bepu.addAngularMotor simulation anchor arm Vector3.UnitX

    for tick in 0 .. ticks - 1 do
        // Exercise per-tick constraint retuning like the hydraulics will.
        let command =
            { TargetVelocity = sin (float32 tick * 0.01f)
              MaxForce = 40.0f }

        Bepu.retuneAngularMotor simulation motor Vector3.UnitX command
        world.Step(TestKit.scriptedInput tick) |> ignore

    world.Physics.HashBodyPoses()

[<Fact>]
[<Trait("Category", "Integration")>]
let ``10k scripted ticks are bit-identical across runs with threads enabled`` () =
    let first = runSession 10_000
    let second = runSession 10_000
    Assert.Equal(first, second)

[<Fact>]
let ``1k scripted ticks are bit-identical across runs (fast gate)`` () =
    let first = runSession 1_000
    let second = runSession 1_000
    Assert.Equal(first, second)

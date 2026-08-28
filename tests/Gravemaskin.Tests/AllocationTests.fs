module Gravemaskin.Tests.AllocationTests

open System
open System.Numerics
open Xunit
open Gravemaskin

[<Fact>]
let ``steady-state tick path does not trigger garbage collections`` () =
    use world = TestKit.flatWorld ()

    Bepu.addDynamicSphere world.Physics.Simulation (Vector3(0.0f, 3.0f, 0.0f)) 0.5f 10.0f
    |> ignore

    // Warmup: tiered JIT, BEPU internal pool growth, event buffer growth.
    TestKit.stepAll 300 InputFrame.empty world |> ignore

    GC.Collect()
    GC.WaitForPendingFinalizers()
    GC.Collect()
    let gen0 = GC.CollectionCount 0
    let gen1 = GC.CollectionCount 1
    let gen2 = GC.CollectionCount 2

    for tick in 0..999 do
        world.Step(TestKit.scriptedInput tick) |> ignore

    Assert.Equal(gen0, GC.CollectionCount 0)
    Assert.Equal(gen1, GC.CollectionCount 1)
    Assert.Equal(gen2, GC.CollectionCount 2)

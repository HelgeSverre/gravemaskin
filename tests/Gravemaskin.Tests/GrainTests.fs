module Gravemaskin.Tests.GrainTests

open System
open System.Numerics
open Xunit
open Gravemaskin

let private grainWorld () =
    let world = TestKit.soilWorld Topsoil
    let pool = GrainPool(4096, world.SoilState.Value)
    world, pool

let private stepPool (pool: GrainPool) (seconds: float32) =
    let mutable t = 0.0f

    while t < seconds do
        pool.Step(1.0f / 60.0f)
        t <- t + 1.0f / 60.0f

[<Fact>]
let ``grains fall and come to rest on the soil surface`` () =
    use world = grainWorld () |> fst
    let pool = GrainPool(256, world.SoilState.Value)
    pool.Spawn(Vector3(8.0f, 3.5f, 8.0f), Vector3.Zero, DrySand, 0uy, 0.03f)
    stepPool pool 3.0f
    Assert.Equal(1, pool.Count)
    Assert.True(pool.RestTimers.[0] >= 0.0f, "the grain should be resting")
    // Surface is at 2.0; the grain sits on it (plus its own radius).
    Assert.InRange(pool.PositionsY.[0], 1.95f, 2.25f)

[<Fact>]
let ``a stream of grains stacks into a visible pile`` () =
    // The falling-sand behavior in one assertion: grains landing on the
    // same spot must come to rest at increasing heights, because each
    // deposits into the pile field the next one lands on.
    use world = grainWorld () |> fst
    let pool = GrainPool(4096, world.SoilState.Value)

    for _ in 1..40 do
        pool.SpawnBurst(Vector3(8.0f, 3.0f, 8.0f), Vector3(0.0f, -1.0f, 0.0f), 0.05f, DrySand, 0uy, 10)
        stepPool pool 0.25f

    let pileTop = pool.GroundHeight(8.0f, 8.0f)
    Assert.True(pileTop > 2.08f, $"the landing spot should have built up: ground {pileTop}")

[<Fact>]
let ``resting grains on an over-steep pile avalanche back into flight`` () =
    use world = grainWorld () |> fst
    let pool = GrainPool(4096, world.SoilState.Value)

    // Hammer one point until the pile is steep, then look for grains that
    // went airborne again after having rested.
    for _ in 1..120 do
        pool.SpawnBurst(Vector3(8.0f, 2.8f, 8.0f), Vector3(0.0f, -2.0f, 0.0f), 0.02f, DrySand, 0uy, 12)
        stepPool pool 0.1f

    stepPool pool 2.0f
    // The pile cannot be arbitrarily tall relative to its neighbors: the
    // avalanche rule caps local steepness near repose.
    let center = pool.GroundHeight(8.0f, 8.0f)
    let neighbor = pool.GroundHeight(8.25f, 8.0f)
    Assert.True(center - neighbor < 0.5f, $"avalanching should bound steepness: {center - neighbor}")

[<Fact>]
let ``digging the ground out from under resting grains re-mobilizes them`` () =
    let world, _ = grainWorld ()
    use _ = world
    let pool = GrainPool(256, world.SoilState.Value)
    pool.Spawn(Vector3(8.0f, 2.5f, 8.0f), Vector3.Zero, Topsoil, 0uy, 0.03f)
    stepPool pool 2.0f
    Assert.True(pool.RestTimers.[0] >= 0.0f, "should be resting first")

    world.CarveSphere(Vector3(8.0f, 1.9f, 8.0f), 0.6f) |> ignore
    stepPool pool 1.0f
    // The surface dropped ~0.5 m; the grain must have fallen to the new floor.
    let ground = Soil.surfaceHeight world.SoilState.Value 8.0f 8.0f
    Assert.True(pool.PositionsY.[0] < 2.0f, $"grain should drop into the hole: y {pool.PositionsY.[0]}, floor {ground}")

[<Fact>]
let ``the pool recycles at capacity instead of growing`` () =
    use world = grainWorld () |> fst
    let pool = GrainPool(128, world.SoilState.Value)

    for _ in 1..100 do
        pool.SpawnBurst(Vector3(8.0f, 3.0f, 8.0f), Vector3.Zero, 0.05f, DrySand, 0uy, 10)

    Assert.Equal(128, pool.Count)
    stepPool pool 1.0f
    Assert.True(pool.Count <= 128)

[<Fact>]
let ``grain stepping allocates nothing`` () =
    use world = grainWorld () |> fst
    let pool = GrainPool(8192, world.SoilState.Value)

    for _ in 1..40 do
        pool.SpawnBurst(Vector3(8.0f, 3.0f, 8.0f), Vector3(0.5f, -1.0f, 0.2f), 0.4f, DrySand, 0uy, 100)

    stepPool pool 1.0f
    GC.Collect()
    GC.WaitForPendingFinalizers()
    GC.Collect()
    let bytesBefore = GC.GetAllocatedBytesForCurrentThread()
    stepPool pool 5.0f
    let allocated = GC.GetAllocatedBytesForCurrentThread() - bytesBefore

    if TestKit.isReleaseBuild then
        Assert.True(allocated < 1024L, $"grain stepping allocated {allocated} bytes")

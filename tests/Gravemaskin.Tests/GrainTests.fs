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
let ``grains fall to the soil surface and sink away`` () =
    use world = grainWorld () |> fst
    let pool = GrainPool(256, world.SoilState.Value)
    pool.Spawn(Vector3(8.0f, 3.5f, 8.0f), Vector3.Zero, DrySand, 0uy, 0.03f)
    stepPool pool 0.3f
    Assert.Equal(1, pool.Count)
    Assert.True(pool.PositionsY.[0] > 2.0f, "still falling: the surface is at 2.0")
    stepPool pool 3.0f
    // Grains never pile: the terrain is the pile.
    Assert.Equal(0, pool.Count)

[<Fact>]
let ``a stream of grains leaves no pile of its own`` () =
    use world = grainWorld () |> fst
    let pool = GrainPool(4096, world.SoilState.Value)

    for _ in 1..40 do
        pool.SpawnBurst(Vector3(8.0f, 3.0f, 8.0f), Vector3(0.0f, -1.0f, 0.0f), 0.05f, DrySand, 0uy, 10)
        stepPool pool 0.25f

    Assert.Equal(Soil.surfaceHeight world.SoilState.Value 8.0f 8.0f, pool.GroundHeight(8.0f, 8.0f))
    stepPool pool 3.0f
    Assert.Equal(0, pool.Count)

[<Fact>]
let ``grains fall into a hole dug under their path`` () =
    let world, _ = grainWorld ()
    use _ = world
    let pool = GrainPool(256, world.SoilState.Value)
    world.CarveSphere(Vector3(8.0f, 1.9f, 8.0f), 0.6f) |> ignore
    pool.Spawn(Vector3(8.0f, 2.5f, 8.0f), Vector3.Zero, Topsoil, 128uy, 0.03f)
    let mutable lowest = 10.0f

    for _ in 1..120 do
        pool.Step(1.0f / 60.0f)

        if pool.Count > 0 then
            lowest <- min lowest pool.PositionsY.[0]

    Assert.True(lowest < 1.9f, $"grain should drop into the hole: lowest y {lowest}")

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

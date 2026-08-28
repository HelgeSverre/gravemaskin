module Gravemaskin.Tests.SoilDeterminismTests

open System
open System.Diagnostics
open System.Numerics
open Xunit
open Gravemaskin

/// A scripted digging session: carve pass along a moving line while physics,
/// settling, and mesh swaps all run. Returns (poseHash, soilHash).
let private runDigSession (ticks: int) =
    use world = TestKit.soilWorld Topsoil

    for tick in 0 .. ticks - 1 do
        if tick % 7 = 0 then
            let t = float32 (tick % 350) / 350.0f
            let center = Vector3(4.0f + t * 8.0f, 1.9f - t * 0.8f, 6.0f + t * 4.0f)
            world.CarveSphere(center, 0.4f) |> ignore

        world.Step(TestKit.scriptedInput tick) |> ignore

    struct (world.Physics.HashBodyPoses(), TestKit.hashSoil world)

[<Fact>]
let ``1k digging ticks: poses AND soil volume bit-identical across runs`` () =
    let struct (poses1, soil1) = runDigSession 1_000
    let struct (poses2, soil2) = runDigSession 1_000
    Assert.Equal(poses1, poses2)
    Assert.Equal(soil1, soil2)

[<Fact>]
[<Trait("Category", "Integration")>]
let ``10k digging ticks: poses AND soil volume bit-identical across runs`` () =
    let struct (poses1, soil1) = runDigSession 10_000
    let struct (poses2, soil2) = runDigSession 10_000
    Assert.Equal(poses1, poses2)
    Assert.Equal(soil1, soil2)

[<Fact>]
let ``steady-state digging does not trigger garbage collections`` () =
    use world = TestKit.soilWorld Topsoil

    // Warmup covers JIT, BEPU pool growth, clump-pool churn, mesh swaps.
    for tick in 0..499 do
        if tick % 7 = 0 then
            world.CarveSphere(Vector3(4.0f + float32 (tick % 40) * 0.2f, 1.8f, 8.0f), 0.4f)
            |> ignore

        world.Step(TestKit.scriptedInput tick) |> ignore

    GC.Collect()
    GC.WaitForPendingFinalizers()
    GC.Collect()
    let gen0 = GC.CollectionCount 0

    for tick in 500..1499 do
        if tick % 7 = 0 then
            world.CarveSphere(Vector3(4.0f + float32 (tick % 40) * 0.2f, 1.6f, 8.0f), 0.4f)
            |> ignore

        world.Step(TestKit.scriptedInput tick) |> ignore

    Assert.Equal(gen0, GC.CollectionCount 0)

[<Fact>]
[<Trait("Category", "Integration")>]
let ``headless trench dig stays inside the tick budget`` () =
    // Strict 6 ms p99 gate only in Release (just perf); Debug is a sanity
    // bound only — suite contention inflates it, so it gets generous slack.
    let budgetMs = if TestKit.isReleaseBuild then 6.0 else 45.0
    use world = TestKit.soilWorld Topsoil
    let times = Array.zeroCreate<float> 10_000
    let watch = Stopwatch()

    for tick in 0..9_999 do
        watch.Restart()

        if tick % 7 = 0 then
            let t = float32 (tick % 350) / 350.0f
            world.CarveSphere(Vector3(4.0f + t * 8.0f, 1.9f - t * 0.8f, 8.0f), 0.4f)
            |> ignore

        world.Step(TestKit.scriptedInput tick) |> ignore
        times.[tick] <- watch.Elapsed.TotalMilliseconds

    Array.sortInPlace times
    let p99 = times.[9_899]
    let p50 = times.[5_000]
    // Surfaced in test output either way — the budget table's ground truth.
    Assert.True(p99 < budgetMs, $"p99 {p99:F2} ms (p50 {p50:F2} ms) over budget {budgetMs} ms")

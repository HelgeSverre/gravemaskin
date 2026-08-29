module Gravemaskin.Tests.MoistureTests

open System
open System.Numerics
open Xunit
open Gravemaskin

[<Fact>]
let ``capillary cohesion peaks damp and vanishes dry and saturated (sandcastles)`` () =
    let sand = Tuning.soil DrySand
    let dry = Moisture.effectiveCohesion sand 0uy
    let damp = Moisture.effectiveCohesion sand 128uy
    let soaked = Moisture.effectiveCohesion sand 255uy
    Assert.True(dry < 0.1f, $"dry sand has no cohesion: {dry}")
    Assert.True(damp > 2.0f, $"damp sand should hold a sandcastle: {damp}")
    Assert.True(soaked < 0.1f, $"saturated sand flows again: {soaked}")

[<Fact>]
let ``clay weakens as it saturates`` () =
    let clay = Tuning.soil Clay
    let dry = Moisture.effectiveCohesion clay 0uy
    let wet = Moisture.effectiveCohesion clay 255uy
    Assert.True(wet < dry * 0.5f, $"saturated clay should lose most strength: {dry} -> {wet}")

[<Fact>]
[<Trait("Category", "Integration")>]
let ``a trench wall that stands in dry clay collapses in waterlogged clay`` () =
    // Dry clay h_crit ≈ 1.1 m holds a 0.9 m face; saturated clay's ≈ 0.45 m
    // does not. Same trench, same script — only the groundwater differs.
    let collapses (waterlogged: bool) =
        use world = TestKit.soilWorld Clay
        let state = world.SoilState.Value

        if waterlogged then
            // Swamp: table above the surface keeps everything saturated
            // (evaporation only dries cells above the table).
            state.WaterTableHeight <- 2.5f
            Array.fill state.Moisture 0 state.Moisture.Length 255uy

        let mutable x = 6.0f

        while x <= 10.0f do
            let mutable y = 1.95f

            while y > 1.1f do
                world.CarveSphere(Vector3(x, y, 8.0f), 0.3f) |> ignore
                y <- y - 0.2f

            x <- x + 0.2f

        let mutable seen = 0

        for tick in 0..899 do
            world.Step(TestKit.scriptedInput tick) |> ignore
            seen <- seen + (world.Events |> Seq.filter ((=) WallCollapsed) |> Seq.length)

        Assert.True(TestKit.conservationError world < 1e-6)
        seen

    Assert.Equal(0, collapses false)
    Assert.True(collapses true > 0, "the waterlogged wall should shed wedges")

[<Fact>]
let ``a damp patch with no groundwater dries out`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value
    state.WaterTableHeight <- 0.0f
    let surfaceIndex = state.Index(20, 7, 20) // top cell of the 2 m fill
    state.Moisture.[surfaceIndex] <- 120uy
    TestKit.stepAll 600 InputFrame.empty world |> ignore
    Assert.True(state.Moisture.[surfaceIndex] < 40uy, $"should dry out: {state.Moisture.[surfaceIndex]}")

[<Fact>]
let ``moisture wicks upward from the water table`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value
    state.WaterTableHeight <- 1.0f

    for z in 0 .. state.Config.CellsZ - 1 do
        for x in 0 .. state.Config.CellsX - 1 do
            for y in 0..3 do
                state.Moisture.[state.Index(x, y, z)] <- 255uy

    let midIndex = state.Index(20, 4, 20) // just above the table
    Assert.Equal(0uy, state.Moisture.[midIndex])
    TestKit.stepAll 600 InputFrame.empty world |> ignore
    Assert.True(state.Moisture.[midIndex] > 0uy, $"should wick up: {state.Moisture.[midIndex]}")

[<Fact>]
let ``terrain generation wets the ground below the water table`` () =
    let config =
        { CellSize = 0.25f
          CellsX = 64
          CellsY = 32
          CellsZ = 64 }

    use world =
        new World(TestKit.defaultSeed, Sim.defaultThreadCount, Some(TerrainSoil(config, 7, 2.2f, 0.8f)))

    let state = world.SoilState.Value
    Assert.True(state.WaterTableHeight > 0.5f, "terrain should set a water table")

    let mutable wetCells = 0

    for i in 0 .. state.Moisture.Length - 1 do
        if state.Moisture.[i] = 255uy then
            wetCells <- wetCells + 1

    Assert.True(wetCells > 1000, $"deep ground should start saturated: {wetCells} wet cells")

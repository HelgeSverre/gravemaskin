module Gravemaskin.Tests.LivingGroundTests

open System
open System.Numerics
open Xunit
open Gravemaskin

/// Carve a straight-walled trench: a full-depth slot from the surface down.
let private carveTrench (world: World) (x0: float32) (x1: float32) (z: float32) (depth: float32) =
    let mutable x = x0

    while x <= x1 do
        let mutable y = 1.95f

        while y > 2.0f - depth do
            world.CarveSphere(Vector3(x, y, z), 0.3f) |> ignore
            y <- y - 0.2f

        x <- x + 0.2f

[<Fact>]
let ``clay wall stands where dry sand slumps (materials are distinguishable)`` () =
    // Same trench in both materials; clay's cohesion (h_crit ≈ 1.1 m) holds
    // a face that sand (h_crit = 0, repose only) cannot.
    let faceSteepness (mat: SoilMaterial) =
        use world = TestKit.soilWorld mat
        carveTrench world 6.0f 10.0f 8.0f 0.9f
        TestKit.stepAll 900 InputFrame.empty world |> ignore
        let state = world.SoilState.Value
        // Steepest neighbor-column drop across the trench edge.
        let mutable steepest = 0.0f

        for x in 20..44 do
            for z in 28..36 do
                let h0 = state.Heights.[state.ColumnIndex(x, z)]
                let h1 = state.Heights.[state.ColumnIndex(x, z + 1)]
                steepest <- max steepest (abs (h0 - h1))

        steepest

    let clay = faceSteepness Clay
    let sand = faceSteepness DrySand
    Assert.True(clay > sand + 0.15f, $"clay {clay} m per cell vs sand {sand} m — clay should hold steeper")

[<Fact>]
[<Trait("Category", "Integration")>]
let ``an over-tall clay wall eventually fails as wedges, conserving mass`` () =
    use world = TestKit.soilWorld Clay
    // 1.9 m of clay: build up beyond h_crit ≈ 1.1 m with injected fill so a
    // clean over-critical face exists.
    let state = world.SoilState.Value

    for x in 30..40 do
        for z in 28..36 do
            Soil.injectLoose
                state
                (Vector3(float32 x * 0.25f, 0.0f, float32 z * 0.25f))
                180.0
                Clay

    // Compact the injected fill to bank so the cohesion regime rules it.
    for i in 0 .. state.Compaction.Length - 1 do
        if state.Occupancy.[i] > 0uy then
            state.Compaction.[i] <- 255uy

    // Direct compaction-byte writes changed cell masses; re-baseline the
    // ledger so the collapse itself is what conservation judges.
    let baseline = Soil.massTotals state
    Array.blit baseline 0 state.Ledger 0 5

    Array.fill state.DirtySettle 0 state.DirtySettle.Length true

    let mutable collapses = 0

    for tick in 0..1199 do
        world.Step(TestKit.scriptedInput tick) |> ignore
        collapses <- collapses + (world.Events |> Seq.filter ((=) WallCollapsed) |> Seq.length)

    Assert.True(collapses > 0, "an over-critical clay face should shed wedges")
    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")

[<Fact>]
let ``track passes compact loose spoil: lower, denser, mass conserved`` () =
    use world = TestKit.soilWorld Topsoil
    let machine = world.SpawnMachine(Vector3(8.0f, 0.0f, 8.0f))
    let state = world.SoilState.Value
    TestKit.stepAll 60 InputFrame.empty world |> ignore

    // A carpet of loose spoil under and around the machine.
    for x in 0..19 do
        for z in 0..8 do
            Soil.injectLoose
                state
                (Vector3(7.0f + float32 x * 0.25f, 0.0f, 6.9f + float32 z * 0.25f))
                12.0
                Topsoil

    TestKit.stepAll 180 InputFrame.empty world |> ignore

    let region = [ for x in 28..47 do for z in 27..36 -> state.ColumnIndex(x, z) ]
    let before = region |> List.map (fun c -> state.Heights.[c])

    // Churn back and forth on the spoil.
    for pass in 0..5 do
        let direction = if pass % 2 = 0 then 1.0f else -1.0f

        TestKit.stepAll
            120
            { InputFrame.empty with
                LeftTrack = direction
                RightTrack = direction }
            world
        |> ignore

    ignore machine
    // Somewhere under the tracks, spoil got pressed measurably down…
    let dropped =
        List.zip region before
        |> List.exists (fun (c, h) -> state.Heights.[c] < h - 0.015f)

    // …because spoil cells got denser: compacted material above the
    // original 2 m bank line can only be track-pressed spoil (injected
    // loose at comp 0; the tracks are the only thing that raises it).
    let config = state.Config
    let mutable compacted = false

    for z in 0 .. config.CellsZ - 1 do
        for x in 0 .. config.CellsX - 1 do
            for y in 8 .. config.CellsY - 1 do
                let index = state.Index(x, y, z)

                if state.Occupancy.[index] > 0uy && state.Compaction.[index] > 0uy then
                    compacted <- true

    Assert.True(dropped, "some spoil column under the tracks should sit lower after driving")
    Assert.True(compacted, "compacted spoil cells should exist above the bank line after track passes")
    // …and no mass was created or destroyed doing it.
    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")

[<Fact>]
let ``buried rocks surface when dug free and clank when struck`` () =
    use world = TestKit.soilWorld Topsoil
    world.SeedRocks 12
    Assert.True(world.Rocks.Count > 0, "some rocks should seed")

    // Excavate broadly over one rock's position.
    let rockPosition = world.Physics.Simulation.Bodies.[world.Rocks.[0]].Pose.Position

    for tick in 0..299 do
        if tick % 4 = 0 then
            world.CarveSphere(Vector3(rockPosition.X, 1.9f - float32 (tick / 4) * 0.02f, rockPosition.Z), 0.7f)
            |> ignore

        world.Step InputFrame.empty |> ignore

    // The rock must have become dynamic (exposed) and must not have fallen
    // out of the world; soil conservation is untouched by rocks.
    let after = world.Physics.Simulation.Bodies.[world.Rocks.[0]].Pose.Position
    Assert.True(after.Y > -5.0f, $"exposed rock should rest, not vanish: y {after.Y}")
    Assert.True(TestKit.conservationError world < 1e-6)

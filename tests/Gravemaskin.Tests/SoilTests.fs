module Gravemaskin.Tests.SoilTests

open System
open System.Numerics
open Xunit
open FsCheck
open FsCheck.Xunit
open Gravemaskin

let private scratch () = Array.zeroCreate<float> 5

[<Fact>]
let ``fresh soil world conserves mass exactly`` () =
    use world = TestKit.soilWorld Topsoil
    Assert.True(TestKit.conservationError world < 1e-9)

[<Fact>]
let ``carve removes mass from the volume and hands it to the caller`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value
    let before = (Soil.massTotals state).[0]
    let removed = world.CarveSphere(Vector3(8.0f, 1.8f, 8.0f), 0.6f)
    let after = (Soil.massTotals state).[0]
    Assert.True(removed > 10.0, $"a 0.6 m sphere at the surface should remove real mass, got {removed} kg")
    Assert.Equal(before - removed, after, 3)
    // Removed mass is now clumps (or overflow deposits) — total conserved.
    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")

[<Fact>]
let ``carved clumps settle back into the volume and the trench survives`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value
    let surfaceBefore = Soil.surfaceHeight state 8.0f 8.0f
    world.CarveSphere(Vector3(8.0f, 1.9f, 8.0f), 0.7f) |> ignore

    // Let everything fall, roll, rest, and bank back in.
    TestKit.stepAll 600 InputFrame.empty world |> ignore

    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")
    let surfaceAfter = Soil.surfaceHeight state 8.0f 8.0f

    Assert.True(
        surfaceAfter < surfaceBefore - 0.1f,
        $"the hole should persist (spoil lands around it): {surfaceBefore} -> {surfaceAfter}"
    )

[<Fact>]
[<Trait("Category", "Integration")>]
let ``ghost bucket digs a trench over 600 ticks with exact mass conservation`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value

    // Sweep a carve sphere along a 6 m line, lowering as it goes — a crude
    // scripted dig pass. Carve every 10th tick, then let physics chew.
    for tick in 0..599 do
        if tick % 10 = 0 && tick < 300 then
            let t = float32 tick / 300.0f
            let center = Vector3(5.0f + t * 6.0f, 1.9f - t * 0.9f, 8.0f)
            world.CarveSphere(center, 0.45f) |> ignore

        world.Step(TestKit.scriptedInput tick) |> ignore

    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")
    // The trench line is measurably lower than untouched ground.
    let trenchHeight = Soil.surfaceHeight state 8.0f 8.0f
    let virginHeight = Soil.surfaceHeight state 8.0f 14.0f
    Assert.True(trenchHeight < virginHeight - 0.15f, $"trench {trenchHeight} vs virgin {virginHeight}")

[<Fact>]
let ``bodies rest ON the soil mesh — the winding regression guard`` () =
    // The escape guard (mass recycling at y < −10) once masked total mesh
    // collision failure: everything fell through and conservation stayed
    // green. This asserts actual support, which the guard cannot fake.
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value

    let probe =
        Bepu.addDynamicSphere world.Physics.Simulation (Vector3(8.0f, 3.0f, 8.0f)) 0.4f 20.0f

    TestKit.stepAll 240 InputFrame.empty world |> ignore
    let position = world.Physics.Simulation.Bodies.[probe].Pose.Position
    let surface = Soil.surfaceHeight state position.X position.Z
    Assert.InRange(position.Y - surface, 0.2f, 0.6f)

[<Fact>]
let ``no clump ever tunnels below the soil or escapes the world`` () =
    use world = TestKit.soilWorld Topsoil
    world.CarveSphere(Vector3(8.0f, 1.9f, 8.0f), 0.7f) |> ignore

    for tick in 0..299 do
        world.Step(TestKit.scriptedInput tick) |> ignore

        // Every live clump must be above the bottom of the world.
        for i in 0 .. world.Clumps.Count - 1 do
            let y =
                world.Physics.Simulation.Bodies.[world.Clumps.Handles.[i]].Pose.Position.Y

            Assert.True(y > -10.0f, $"clump {i} fell out of the world at tick {tick}, y={y}")

    // And whatever escaped or settled was banked: conservation still exact.
    Assert.True(TestKit.conservationError world < 1e-6)

[<Fact>]
let ``clump cap holds under greedy carving and drops no mass`` () =
    use world = TestKit.soilWorld Topsoil

    // Max-rate carving: a new sphere every tick for 120 ticks.
    for tick in 0..119 do
        let x = 4.0f + float32 (tick % 8) * 1.2f
        let z = 4.0f + float32 (tick / 8) * 0.8f
        world.CarveSphere(Vector3(x, 1.8f, z), 0.5f) |> ignore
        world.Step(TestKit.scriptedInput tick) |> ignore
        Assert.True(world.Clumps.Count <= 1500, $"cap breached: {world.Clumps.Count}")

    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")

[<Fact>]
let ``deposited pile relaxes toward the angle of repose`` () =
    use world = TestKit.soilWorld DrySand
    let state = world.SoilState.Value

    // Dump 400 kg of loose sand on one column (fits the column height; a
    // bigger dump parks the surplus in Unbanked by design — ponytail: point
    // deposits that exceed a column go unbanked, spread-on-deposit if it
    // ever matters in play).
    Soil.injectLoose state (Vector3(8.0f, 0.0f, 8.0f)) 400.0 DrySand

    for tick in 0..599 do
        world.Step(TestKit.scriptedInput tick) |> ignore

    Assert.True(TestKit.conservationError world < 1e-6)
    // Max neighbor slope near the pile should be near/below repose.
    let props = Tuning.soil DrySand
    let critical = MathF.Tan props.FrictionAngle * state.Config.CellSize
    let mutable worstExcess = 0.0f

    for z in 24..40 do
        for x in 24..40 do
            let h0 = state.Heights.[state.ColumnIndex(x, z)]
            let h1 = state.Heights.[state.ColumnIndex(x + 1, z)]
            let h2 = state.Heights.[state.ColumnIndex(x, z + 1)]
            worstExcess <- max worstExcess (abs (h0 - h1) - critical)
            worstExcess <- max worstExcess (abs (h0 - h2) - critical)

    // Allow one cell of graininess: the CA moves half-excess quanta.
    Assert.True(
        worstExcess < state.Config.CellSize,
        $"pile still steeper than repose by {worstExcess} m over critical {critical} m"
    )

[<Fact>]
let ``mesh swap keeps the static count constant across 200 carve cycles`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value
    let staticCount = world.Physics.Simulation.Statics.Count

    for cycle in 0..199 do
        let x = 4.0f + float32 (cycle % 12) * 0.9f
        world.CarveSphere(Vector3(x, 1.8f, 10.0f), 0.35f) |> ignore
        world.Step(TestKit.scriptedInput cycle) |> ignore

    // Swaps replace, never accumulate: same number of tile statics.
    Assert.Equal(staticCount, world.Physics.Simulation.Statics.Count)
    Assert.True(TestKit.conservationError world < 1e-6)

[<Property(MaxTest = 8)>]
let ``mass is conserved across arbitrary carve scripts``
    (script: (byte * byte * byte) list)
    =
    use world = TestKit.soilWorld Clay

    for (bx, bz, br) in script |> List.truncate 20 do
        let x = 2.0f + float32 bx / 255.0f * 12.0f
        let z = 2.0f + float32 bz / 255.0f * 12.0f
        let radius = 0.2f + float32 br / 255.0f * 0.6f
        world.CarveSphere(Vector3(x, 1.9f, z), radius) |> ignore
        TestKit.stepAll 5 InputFrame.empty world |> ignore

    TestKit.stepAll 100 InputFrame.empty world |> ignore
    let error = TestKit.conservationError world
    Assert.True(error < 1e-6, $"conservation error {error}")

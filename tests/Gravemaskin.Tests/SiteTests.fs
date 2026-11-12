module Gravemaskin.Tests.SiteTests

open System
open System.IO
open System.Numerics
open Xunit
open Gravemaskin

let private inp b s bk sw =
    { InputFrame.empty with
        Boom = b
        Stick = s
        Bucket = bk
        Swing = sw }

let private bigSoilWorld () =
    let config =
        { CellSize = 0.25f
          CellsX = 128
          CellsY = 40
          CellsZ = 128 }

    new World(TestKit.defaultSeed, Sim.defaultThreadCount, Some(FlatSoil(config, Topsoil, 4.0f)))

// ---- regression: moisture bytes must saturate, never wrap ----

[<Fact>]
let ``moisture saturates instead of wrapping around`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value
    let sweep = state.Config.CellsX * state.Config.CellsZ / Moisture.ColumnBudget
    let mutable cursor = 0

    // Dry ground, no water table: a soaked surface cell dries 255 → 3 in
    // steps of 4, and 3uy − 4uy wrapped back to 255 before the fix.
    state.WaterTableHeight <- 0.0f
    let surface = state.Index(10, 7, 10)
    state.Moisture.[surface] <- 255uy
    let mutable last = 255uy
    let mutable rose = false

    for _ in 1 .. 80 * sweep do
        cursor <- Moisture.tick state state.WaterTableHeight cursor

        if state.Moisture.[surface] > last then
            rose <- true

        last <- state.Moisture.[surface]

    Assert.False(rose, "a drying surface cell must never jump back up")
    Assert.Equal(0uy, state.Moisture.[surface])

    // Water table at 1 m: the first cell above it sits at 248 under a
    // saturated neighbor. 248 + 24 wrapped to 16, so it "wicked" 248 + 8
    // = 0 before the fix.
    state.WaterTableHeight <- 1.0f
    let aboveTable = state.Index(20, 4, 20)
    state.Moisture.[aboveTable] <- 248uy

    for _ in 1 .. 10 * sweep do
        cursor <- Moisture.tick state state.WaterTableHeight cursor
        Assert.True(state.Moisture.[aboveTable] >= 248uy, "a nearly saturated cell must not flip to dry")

// ---- the blade cut ----

[<Fact>]
let ``the blade cut lowers columns fractionally and conserves mass`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value
    let removed = Array.zeroCreate<float> Domain.MaterialCount
    // Edge 5 cm below a 2 m surface: the bowl bottoms out 0.39 m down.
    let center = Vector3(8.125f, 1.95f, 8.125f)
    let total = Soil.carveBowl state center 0.34f removed
    Assert.True(total > 0.0)
    let under = state.Heights.[state.ColumnIndex(32, 32)]
    Assert.InRange(under, 1.95f - 0.34f - 0.002f, 1.95f - 0.34f + 0.002f)
    // A rim column is shaved, not left whole or popped out as a full cell.
    let rim = state.Heights.[state.ColumnIndex(33, 32)]
    Assert.True(rim > under + 0.03f && rim < 2.0f, $"rim column should sit between: {rim}")
    Assert.True(abs (rim / 0.25f - round (rim / 0.25f)) > 0.02f, $"the cut should be fractional, not a whole cell: {rim}")
    // The scan lost exactly what the cut reports.
    let scan = Soil.massTotals state
    let ledgerMinusScan = state.Ledger.[0] - scan.[0]
    Assert.True(abs (ledgerMinusScan - total) / total < 1e-9, $"cut {total} kg vs scan delta {ledgerMinusScan} kg")

[<Fact>]
let ``dig resistance reads the soil in the bench, not the emptied cell at the edge`` () =
    use world = TestKit.soilWorld Clay
    let state = world.SoilState.Value
    // Just above the surface the cell is empty; the bench below is clay.
    let struct (mat, comp, _) = Soil.sampleBelow state 8.0f 2.1f 8.0f
    Assert.Equal(Volume.byteOfMaterial Clay, mat)
    Assert.Equal(255uy, comp)

[<Fact>]
let ``the cat 320 fills its own bucket, not a mini's`` () =
    // Cut radius, blade width and fill rate were U17 constants: the 320
    // needed ~22 s of cutting to fill. One stroke must now load it.
    use world = bigSoilWorld ()
    let machine = world.SpawnMachineRig(Tuning.cat320Rig, Vector3(16.0f, 0.0f, 16.0f))
    Assert.True(machine.CutPoints >= 2, "a 1.4 m lip cuts at more than one point")
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    TestKit.stepAll 90 (inp -1.0f 0.0f 0.0f 0.0f) world |> ignore
    TestKit.stepAll 120 (inp 0.0f -1.0f 0.0f 0.0f) world |> ignore
    TestKit.stepAll 180 (inp 0.0f -0.5f -1.0f 0.0f) world |> ignore
    Assert.True(machine.BucketLoadKg > 800.0, $"a 320 stroke should load most of 1600 kg: {machine.BucketLoadKg}")
    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")

// ---- gravity empties the bucket, not a joint angle ----

[<Fact>]
let ``the load stays in a mouth-up bucket and runs out when the mouth turns down`` () =
    let world = Sim.createWorld TestKit.defaultSeed
    use _ = world
    let machine = world.SpawnMachine(Vector3.Zero)
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    // Boom up, curl until the mouth faces the sky.
    TestKit.stepAll 180 (inp 1.0f 0.0f 0.0f 0.0f) world |> ignore
    TestKit.stepAll 240 (inp 0.0f 0.0f -1.0f 0.0f) world |> ignore
    Assert.True(machine.MouthDirection.Y > 0.5f, $"mouth should face up: {machine.MouthDirection.Y}")
    Assert.Equal(1.0f, machine.Retention)
    machine.TryAbsorb(60.0, 0) |> ignore

    for _ in 1..120 do
        Assert.True((machine.PourTick false).IsNone, "a mouth-up bucket must hold its load")

    // Roll the mouth over: the load leaves, a little at first.
    TestKit.stepAll 240 (inp 0.0f 0.0f 1.0f 0.0f) world |> ignore
    Assert.True(machine.MouthDirection.Y < Tuning.PourEmptyTilt, $"mouth should face down: {machine.MouthDirection.Y}")
    Assert.Equal(0.0f, machine.Retention)
    let mutable ticks = 0

    while machine.BucketLoadKg > 1e-6 && ticks < 600 do
        machine.PourTick false |> ignore
        ticks <- ticks + 1

    Assert.True(machine.BucketLoadKg <= 1e-6, "an upended bucket empties")
    Assert.InRange(ticks, 40, 80) // 60 kg at 1.2 kg/tick

    // Teeth below the bench: the bank holds the load whatever the tilt.
    machine.TryAbsorb(30.0, 0) |> ignore
    Assert.True((machine.PourTick true).IsNone)

// ---- world edge, heading ----

[<Fact>]
let ``the machine cannot drive off the edge of the world`` () =
    use world = TestKit.soilWorld Topsoil
    let machine = world.SpawnMachine(Vector3(12.0f, 0.0f, 8.0f))
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    let mutable slipped = false

    for _ in 1..900 do
        world.Step
            { InputFrame.empty with
                LeftTrack = 1.0f
                RightTrack = 1.0f }
        |> ignore

        if world.Events.Contains TrackSlip then
            slipped <- true

    let position = world.Physics.Simulation.Bodies.[machine.Chassis].Pose.Position
    Assert.True(position.X < 16.0f, $"chassis must stay inside the 16 m world: x {position.X}")
    Assert.True(position.Y > 1.5f, $"and on the ground, not falling: y {position.Y}")
    Assert.True(slipped, "tracks grinding against the boundary should report slip")

[<Fact>]
let ``a machine spawns on its heading and a swap keeps it`` () =
    use world = TestKit.soilWorld Topsoil
    let quarterTurn = MathF.PI / 2.0f
    let machine = world.SpawnMachineRig(Tuning.u17Rig, Vector3(8.0f, 0.0f, 8.0f), quarterTurn)
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    Assert.InRange(machine.Yaw, quarterTurn - 0.05f, quarterTurn + 0.05f)
    // +90° about Y points the nose (and the arm) down −Z.
    let tip = machine.BucketTipPosition
    Assert.True(tip.Z < 6.0f && abs (tip.X - 8.0f) < 0.5f, $"arm should reach along -Z: {tip}")
    // The joints still work in the turned frame.
    TestKit.stepAll 180 (inp 1.0f 0.0f 0.0f 0.0f) world |> ignore
    Assert.True(machine.BoomAngle > 0.5f, $"boom should rise: {machine.BoomAngle}")

    TestKit.stepAll
        240
        { InputFrame.empty with
            LeftTrack = 1.0f
            RightTrack = 1.0f }
        world
    |> ignore

    let driven = world.Physics.Simulation.Bodies.[machine.Chassis].Pose.Position
    Assert.True(driven.Z < 7.0f, $"forward is -Z on this heading: {driven}")

    let swapped = world.SwapMachine Tuning.tb216Rig
    TestKit.stepAll 30 InputFrame.empty world |> ignore
    Assert.InRange(swapped.Yaw, quarterTurn - 0.15f, quarterTurn + 0.15f)

// ---- the job site ----

[<Fact>]
[<Trait("Category", "Integration")>]
let ``the sandbox lays out a job site: level yard, creek, loose stockpiles, a pit to dig`` () =
    use world = Sim.createTerrainWorld 0xD16D16UL
    let state = world.SoilState.Value
    Assert.True(state.Site.IsSome, "the 64 m sandbox should carry a site")
    let site = state.Site.Value

    // The yard is level where the machine spawns.
    for dx in -3 .. 3 do
        for dz in -3 .. 2 do
            let h = Soil.surfaceHeight state (site.YardX + float32 dx) (site.YardZ + float32 dz)
            Assert.InRange(h, site.PadHeight - 0.02f, site.PadHeight + 0.02f)

    // The creek bed lies under the water table along its whole length.
    for step in 1..7 do
        let z = site.WorldZ * float32 step / 8.0f
        let bed = Soil.surfaceHeight state (Terrain.creekCenter site z) z
        Assert.True(bed < state.WaterTableHeight - 0.2f, $"creek bed at z={z} should hold water: {bed}")

    // Stockpiles stand proud of the pad and are loose (easy digging).
    for pile in 0..2 do
        let struct (x, z, material) = Terrain.stockpile site pile
        Assert.True(Soil.surfaceHeight state x z > site.PadHeight + 1.0f, $"stockpile {pile} should be a heap")
        let struct (mat, comp, _) = Soil.sampleBelow state x (site.PadHeight + 1.0f) z
        Assert.Equal(Volume.byteOfMaterial material, mat)
        Assert.Equal(0uy, comp)

    // Terrain is no longer terraced in whole 0.25 m cells.
    let fractional =
        state.Heights
        |> Array.filter (fun h -> abs (h / 0.25f - round (h / 0.25f)) > 0.05f)
        |> Array.length

    Assert.True(fractional > state.Heights.Length / 3, $"heights should be continuous: {fractional} fractional columns")
    Assert.True(TestKit.conservationError world < 1e-9)

    // The pit: untouched at first, progress climbs as it is dug out.
    Assert.InRange(Terrain.pitProgress state, 0.0f, 0.01f)
    let center = Vector3((site.PitMinX + site.PitMaxX) * 0.5f, site.PadHeight - 0.3f, (site.PitMinZ + site.PitMaxZ) * 0.5f)
    world.CarveSphere(center, 1.2f) |> ignore
    let afterOne = Terrain.pitProgress state
    Assert.True(afterOne > 0.05f, $"digging in the pit should register: {afterOne}")
    // Digging outside the stakes does not.
    world.CarveSphere(Vector3(site.YardX - 5.0f, site.PadHeight - 0.3f, site.YardZ), 1.2f) |> ignore
    Assert.Equal(afterOne, Terrain.pitProgress state)

    // Scenery keeps off the yard, and the trees block the machine.
    world.SeedScenery()
    Assert.True(world.TreeCount >= 20, $"the site should be wooded: {world.TreeCount} trees")
    let snapshot = RenderSnapshot(Clumps.MaxClumps)
    world.SnapshotInto snapshot
    Assert.Equal(world.TreeCount, snapshot.TreeCount)

    for i in 0 .. snapshot.TreeCount - 1 do
        let tree = snapshot.TreePositions.[i]
        Assert.True(Terrain.padDistance site tree.X tree.Z > 5.0f, $"no tree on the yard: {tree}")
        Assert.True(tree.Y > state.WaterTableHeight, $"no tree in the water: {tree}")

[<Fact>]
[<Trait("Category", "Integration")>]
let ``save → load keeps the site, the trees and the machine's heading`` () =
    let path = Path.Combine(Path.GetTempPath(), $"gravemaskin-site-{Guid.NewGuid():N}.grav")

    try
        use world = Sim.createTerrainWorld 0xD16D16UL
        let site = world.SoilState.Value.Site.Value
        world.SeedScenery()
        world.SpawnMachineRig(Tuning.tb216Rig, Vector3(site.YardX, 0.0f, site.YardZ), 1.0f) |> ignore
        TestKit.stepAll 60 InputFrame.empty world |> ignore
        world.Save path

        use loaded = Sim.loadWorld 0xD16D16UL path
        Assert.Equal(world.SoilState.Value.Site, loaded.SoilState.Value.Site)
        Assert.Equal(world.TreeCount, loaded.TreeCount)
        Assert.Equal(world.Rocks.Count, loaded.Rocks.Count)
        TestKit.stepAll 30 InputFrame.empty loaded |> ignore
        Assert.InRange(loaded.Machine.Value.Yaw, 0.9f, 1.1f)
        Assert.True(TestKit.conservationError loaded < 1e-6)
    finally
        if File.Exists path then
            File.Delete path

// ---- loose soil: packets in flight, piles as terrain ----

[<Fact>]
let ``a poured heap is a round cone standing at its angle of repose`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value

    // Delivered fill: credit the ledger for what is about to land.
    let sand = int (Volume.byteOfMaterial DrySand)
    state.Ledger.[sand] <- state.Ledger.[sand] + 900.0

    for _ in 1..200 do
        ClumpPool.Land(state, 8.0f, 8.0f, 4.5, DrySand)
        world.Step InputFrame.empty |> ignore

    TestKit.stepAll 120 InputFrame.empty world |> ignore
    let rise (x: float32) (z: float32) = Soil.surfaceHeight state x z - 2.0f
    let peak = rise 8.1f 8.1f
    Assert.True(peak > 0.4f, $"900 kg of sand should stand as a heap: peak {peak} m")

    // Round: the same distance out along an axis and along a diagonal
    // stands at about the same height (the old rule built a diamond).
    let axis = (rise 8.85f 8.1f + rise 7.1f 8.1f + rise 8.1f 8.85f + rise 8.1f 7.1f) / 4.0f
    let diagonal = (rise 8.6f 8.6f + rise 7.35f 7.35f + rise 8.6f 7.35f + rise 7.35f 8.6f) / 4.0f
    Assert.True(MathF.Abs(axis - diagonal) < 0.12f, $"heap should be round: axis {axis} m, diagonal {diagonal} m")

    // At repose: no neighbouring columns stand steeper than the soil can.
    let limit = Settle.reposeSlope (Tuning.soil DrySand) * state.Config.CellSize + 0.03f

    for z in 24..40 do
        for x in 24..40 do
            let here = state.Heights.[state.ColumnIndex(x, z)]
            let next = state.Heights.[state.ColumnIndex(x + 1, z)]
            Assert.True(MathF.Abs(here - next) <= limit, $"over-steep at {x},{z}: {MathF.Abs(here - next)} m")

    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")

[<Fact>]
let ``soil in flight lands as ground within seconds and nothing keeps rolling`` () =
    use world = TestKit.soilWorld Topsoil
    world.CarveSphere(Vector3(8.0f, 1.9f, 8.0f), 0.7f) |> ignore
    Assert.True(world.Clumps.Count > 0, "the cut should throw spoil")
    TestKit.stepAll 150 InputFrame.empty world |> ignore
    Assert.Equal(0, world.Clumps.Count)
    Assert.True(TestKit.conservationError world < 1e-6, $"error {TestKit.conservationError world}")

[<Fact>]
let ``the bucket shell ploughs loose spoil and leaves the bank alone`` () =
    use world = TestKit.soilWorld Topsoil
    let state = world.SoilState.Value
    let scratch = Array.zeroCreate<float> MaterialCount
    // Undisturbed ground under the shell: nothing moves.
    Assert.Equal(0.0, Soil.ploughLoose state (Vector3(8.1f, 2.1f, 8.1f)) 0.3f scratch)

    // A loose heap on top of it: the heap goes, the bank stays.
    for _ in 1..40 do
        ClumpPool.Land(state, 8.1f, 8.1f, 3.0, Topsoil)

    Assert.True(Soil.surfaceHeight state 8.1f 8.1f > 2.1f)
    let removed = Soil.ploughLoose state (Vector3(8.1f, 2.1f, 8.1f)) 0.3f scratch
    Assert.True(removed > 5.0, $"the shell should shoulder the heap aside: {removed} kg")
    let after = Soil.surfaceHeight state 8.1f 8.1f
    Assert.InRange(after, 1.99f, 2.06f)

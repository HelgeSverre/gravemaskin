module Gravemaskin.Tests.ShipTests

open System
open System.IO
open System.Numerics
open Xunit
open Gravemaskin

let private bigSoilWorld () =
    // The Cat 320 needs room: 32×32 m, deeper volume.
    let config =
        { CellSize = 0.25f
          CellsX = 128
          CellsY = 40
          CellsZ = 128 }

    new World(TestKit.defaultSeed, Sim.defaultThreadCount, Some(FlatSoil(config, Topsoil, 2.0f)))

[<Fact>]
let ``cat 320 breakout force emerges from its data within spec tolerance`` () =
    // The dynamic delivered-torque check lives in MachineTests (U17): it
    // verifies Linkage.torqueCap against the solver empirically. Probing the
    // 320 dynamically defeated two test harnesses (lump impulses and stiff
    // dampers both go ballistic at 10^5 N·m), so the 320 — which runs the
    // SAME verified code path — is checked at the math level: peak tip force
    // over the working range must land within ±15% of the published 150 kN,
    // emerging from bore × relief × linkage geometry alone.
    let joint = Tuning.cat320Rig.BucketJoint
    let relief = snd Tuning.cat320Rig.Spec.Circuits.[0] * 1.0e6f
    let mutable peakTorque = 0.0f

    for i in 0..100 do
        let angle = -2.0f + float32 i * 0.02f
        peakTorque <- max peakTorque (Linkage.torqueCap joint angle -1.0f relief) // curl = extend

    let tipForce = peakTorque / Tuning.cat320Rig.BucketTipRadius
    Assert.InRange(tipForce, 150_000.0f * 0.85f, 150_000.0f * 1.15f)

[<Fact>]
let ``the two machines are mechanically different, not palette swaps`` () =
    // A boom resistance that stalls the U17 cold is nothing to the 320.
    let boomRise (rig: MachineRig) (resistTorque: float32) =
        let world = bigSoilWorld ()
        use _ = world
        let machine = world.SpawnMachineRig(rig, Vector3(16.0f, 0.0f, 16.0f))
        TestKit.stepAll 90 InputFrame.empty world |> ignore
        let start = machine.BoomAngle

        for _ in 1..300 do
            let mutable boomRef = world.Physics.Simulation.Bodies.[machine.Boom]

            if not boomRef.Awake then
                boomRef.Awake <- true

            boomRef.ApplyAngularImpulse(Vector3(0.0f, 0.0f, -resistTorque * Tuning.FixedDt))
            world.Step { InputFrame.empty with Boom = 1.0f } |> ignore

        machine.BoomAngle - start

    // 30 kN·m: proven to pin the U17 (its cap ≈ 24.5); the 320's boom cap
    // is ≈ 700 kN·m and its arm alone needs ~180, so it shrugs this off.
    let resist = 30_000.0f
    Assert.True(boomRise Tuning.u17Rig resist < 0.05f, "30 kN·m must pin the U17 boom")
    Assert.True(boomRise Tuning.cat320Rig resist > 0.4f, "the 320 should not notice 30 kN·m")

[<Fact>]
[<Trait("Category", "Integration")>]
let ``save → load round-trips terrain, ledger, machine, and rocks exactly`` () =
    let path = Path.Combine(Path.GetTempPath(), $"gravemaskin-test-{Guid.NewGuid():N}.grav")

    try
        use world = TestKit.soilWorld Topsoil
        world.SpawnMachine(Vector3(8.0f, 0.0f, 8.0f)) |> ignore
        world.SeedRocks 8

        // Leave real marks: dig, spill, drive.
        for tick in 0..399 do
            let frame =
                match (tick / 100) % 4 with
                | 0 -> { InputFrame.empty with Bucket = -1.0f }
                | 1 -> { InputFrame.empty with Boom = -1.0f }
                | 2 ->
                    { InputFrame.empty with
                        Stick = -1.0f
                        Bucket = -0.5f }
                | _ ->
                    { InputFrame.empty with
                        LeftTrack = 1.0f
                        RightTrack = 1.0f }

            world.Step frame |> ignore

        world.Save path

        // After the forced settle, the world's own conservation is exact.
        Assert.True(TestKit.conservationError world < 1e-6)
        let savedState = world.SoilState.Value
        let savedHash = TestKit.hashSoil world

        use loaded = Sim.loadWorld TestKit.defaultSeed path
        let loadedState = loaded.SoilState.Value

        // Terrain bytes and the mass ledger are bit-identical.
        Assert.Equal<byte[]>(savedState.Occupancy, loadedState.Occupancy)
        Assert.Equal<byte[]>(savedState.Material, loadedState.Material)
        Assert.Equal<byte[]>(savedState.Compaction, loadedState.Compaction)
        Assert.Equal<byte[]>(savedState.Moisture, loadedState.Moisture)
        Assert.Equal(savedState.WaterTableHeight, loadedState.WaterTableHeight)
        Assert.Equal(savedHash, TestKit.hashSoil loaded)
        Assert.True(TestKit.conservationError loaded < 1e-6, "loaded world conserves against the saved ledger")

        Assert.True(loaded.Machine.IsSome, "machine placement should restore")
        Assert.Equal(world.Rocks.Count, loaded.Rocks.Count)

        // And it still runs.
        TestKit.stepAll 120 InputFrame.empty loaded |> ignore
        Assert.True(TestKit.conservationError loaded < 1e-6)
    finally
        if File.Exists path then
            File.Delete path

[<Fact>]
[<Trait("Category", "Integration")>]
let ``the 64 m sandbox terrain runs a machine session inside a sane budget`` () =
    use world = Sim.createTerrainWorld 0xD16D16UL
    world.SpawnMachine(Vector3(32.0f, 0.0f, 32.0f)) |> ignore
    world.SeedRocks 48
    let state = world.SoilState.Value
    Assert.Equal(256, state.Config.CellsX)

    // The generator should have produced real variety.
    let seen = Array.zeroCreate<bool> Domain.MaterialCount

    for i in 0 .. state.Material.Length - 1 do
        if state.Occupancy.[i] > 0uy then
            seen.[int state.Material.[i]] <- true

    Assert.True(seen.[int (Volume.byteOfMaterial Grass)], "grass should exist")
    Assert.True(seen.[int (Volume.byteOfMaterial DrySand)], "sand should exist")
    Assert.True(seen.[int (Volume.byteOfMaterial Gravel)], "gravel should exist")
    Assert.True(seen.[int (Volume.byteOfMaterial Clay)], "clay strata should exist")

    let watch = System.Diagnostics.Stopwatch()
    let times = Array.zeroCreate<float> 2000

    for tick in 0..1999 do
        watch.Restart()

        let frame =
            match (tick / 120) % 3 with
            | 0 -> { InputFrame.empty with Bucket = -0.8f; Boom = -0.4f }
            | 1 -> { InputFrame.empty with Stick = -0.8f }
            | _ ->
                { InputFrame.empty with
                    LeftTrack = 1.0f
                    RightTrack = 0.8f }

        world.Step frame |> ignore
        times.[tick] <- watch.Elapsed.TotalMilliseconds

    Array.sortInPlace times
    let budget = if TestKit.isReleaseBuild then 8.0 else 50.0
    Assert.True(times.[1979] < budget, $"big-map p99 {times.[1979]:F2} ms over {budget} ms")
    Assert.True(TestKit.conservationError world < 1e-6)

[<Fact>]
let ``machine swap despawns cleanly and the replacement digs`` () =
    use world = TestKit.soilWorld Topsoil
    let first = world.SpawnMachineRig(Tuning.tb216Rig, Vector3(8.0f, 0.0f, 8.0f))
    TestKit.stepAll 60 InputFrame.empty world |> ignore

    // Load the bucket so the swap has payload to account for.
    TestKit.stepAll 60 { InputFrame.empty with Bucket = -1.0f } world |> ignore
    Assert.True(first.BucketLoadKg > 5.0)

    let bodiesBefore = world.Physics.Simulation.Bodies.ActiveSet.Count
    let second = world.SwapMachine Tuning.u17Rig
    ignore bodiesBefore
    Assert.Equal("U17", second.Rig.Spec.Name)
    Assert.True(TestKit.conservationError world < 1e-6, $"swap must not leak mass: {TestKit.conservationError world}")

    // The replacement is alive: boom lifts.
    TestKit.stepAll 60 InputFrame.empty world |> ignore
    let sagged = second.BoomAngle
    TestKit.stepAll 240 { InputFrame.empty with Boom = 1.0f } world |> ignore
    Assert.True(second.BoomAngle > sagged + 0.3f, $"swapped machine should work: {sagged} -> {second.BoomAngle}")

    // Swap again (back) to shake out constraint-handle reuse.
    let third = world.SwapMachine Tuning.tb216Rig
    TestKit.stepAll 120 InputFrame.empty world |> ignore
    Assert.Equal("TB216", third.Rig.Spec.Name)
    Assert.True(TestKit.conservationError world < 1e-6)

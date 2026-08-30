module Gravemaskin.Shell.Program

#nowarn "9"

open System
open System.Diagnostics
open System.Numerics
open Silk.NET.Input
open Silk.NET.Maths
open Silk.NET.Windowing
open Silk.NET.OpenGL
open Gravemaskin

/// March a ray against the soil surface heights. Returns the hit point.
let private raycastSurface (state: SoilState) (origin: Vector3) (direction: Vector3) =
    let mutable t = 0.0f
    let mutable hit = ValueNone

    while hit.IsNone && t < 120.0f do
        let point = origin + direction * t

        if
            point.X > 0.0f
            && point.Z > 0.0f
            && point.X < float32 state.Config.CellsX * state.Config.CellSize
            && point.Z < float32 state.Config.CellsZ * state.Config.CellSize
            && point.Y <= Soil.surfaceHeight state point.X point.Z
        then
            hit <- ValueSome point

        t <- t + 0.08f

    hit

/// Write the default framebuffer to a 24-bit BMP (bottom-up rows match BMP's
/// native order, so glReadPixels output goes straight in).
let private screenshot (gl: GL) (width: int) (height: int) (path: string) =
    let pixels = Array.zeroCreate<byte> (width * height * 4)

    do
        use ptr = fixed pixels
        gl.ReadPixels(0, 0, uint32 width, uint32 height, PixelFormat.Rgba, PixelType.UnsignedByte, NativeInterop.NativePtr.toVoidPtr ptr)

    let rowBytes = width * 3
    let padding = (4 - rowBytes % 4) % 4
    let dataSize = (rowBytes + padding) * height
    use stream = IO.File.Create path
    use writer = new IO.BinaryWriter(stream)
    writer.Write [| 'B'B; 'M'B |]
    writer.Write(54 + dataSize)
    writer.Write 0
    writer.Write 54
    writer.Write 40
    writer.Write width
    writer.Write height
    writer.Write 1us
    writer.Write 24us
    writer.Write 0
    writer.Write dataSize
    writer.Write 2835
    writer.Write 2835
    writer.Write 0
    writer.Write 0

    for y in 0 .. height - 1 do
        for x in 0 .. width - 1 do
            let i = (y * width + x) * 4
            writer.Write pixels.[i + 2]
            writer.Write pixels.[i + 1]
            writer.Write pixels.[i]

        for _ in 1..padding do
            writer.Write 0uy

[<EntryPoint>]
let main args =
    let argValue name =
        args
        |> Array.tryFindIndex ((=) name)
        |> Option.bind (fun i -> if i + 1 < args.Length then Some args.[i + 1] else None)

    let maxFrames = argValue "--frames" |> Option.map int
    let shotTick = argValue "--shot-tick" |> Option.map int64
    // Model-preview mode (the gun-preview idea from IRONSIGHT): flat stage,
    // fixed camera angles, one PNG-able BMP per angle, then exit — for
    // comparing the machine model against reference photos.
    let previewDir = argValue "--preview"

    let previewAngles =
        [| "side", Vector3(0.0f, 0.7f, 3.0f)
           "side-left", Vector3(0.0f, 0.7f, -3.0f)
           "front", Vector3(4.2f, 0.9f, 0.0f)
           "rear", Vector3(-3.4f, 0.9f, 0.0f)
           "quarter", Vector3(2.6f, 1.5f, 2.6f)
           "quarter-rear", Vector3(-2.4f, 1.4f, 2.4f) |]
    let demoMode = args |> Array.contains "--demo"

    let screenshotPath = argValue "--screenshot"
    let mutable settings = Settings.load ()

    // Input recording/replay: the deterministic sim makes a stream of
    // InputFrames a perfect reproduction of a session.
    let recordWriter =
        argValue "--record"
        |> Option.map (fun path -> new IO.BinaryWriter(IO.File.Create path))

    let replayFrames =
        argValue "--replay"
        |> Option.map (fun path ->
            use reader = new IO.BinaryReader(IO.File.OpenRead path)
            let frames = ResizeArray<InputFrame>()

            while reader.BaseStream.Position < reader.BaseStream.Length do
                frames.Add
                    { Sequence = reader.ReadInt64()
                      Swing = reader.ReadSingle()
                      Boom = reader.ReadSingle()
                      Stick = reader.ReadSingle()
                      Bucket = reader.ReadSingle()
                      LeftTrack = reader.ReadSingle()
                      RightTrack = reader.ReadSingle()
                      Buttons = enum (reader.ReadInt32()) }

            frames)

    let mutable replayIndex = 0

    let mutable options = WindowOptions.Default
    options.Title <- "GRAVEMASKIN"
    options.Size <- Vector2D<int>(settings.WindowWidth, settings.WindowHeight)
    options.VSync <- true

    options.API <-
        GraphicsAPI(ContextAPI.OpenGL, ContextProfile.Core, ContextFlags.ForwardCompatible, APIVersion(4, 1))

    use window = Window.Create options

    let mutable gl = Unchecked.defaultof<GL>
    let mutable input = Unchecked.defaultof<IInputContext>
    let mutable renderer = Unchecked.defaultof<Renderer>
    let mutable hud = Unchecked.defaultof<Hud>
    let mutable audio: AudioSystem option = None
    let mutable world = Unchecked.defaultof<World>
    let mutable state = Unchecked.defaultof<SoilState>
    let mutable grains = Unchecked.defaultof<GrainPool>
    let mutable patternToggleLatch = false
    let mutable menuOpen = false
    let mutable menuLatch = false
    let mutable helpOpen = previewDir.IsNone && not demoMode
    let mutable helpLatch = false
    let mutable saveLatch = false
    let mutable loadLatch = false

    let savePath () =
        let home =
            Environment.GetEnvironmentVariable "GRAVEMASKIN_HOME"
            |> Option.ofObj
            |> Option.defaultWith (fun () ->
                IO.Path.Combine(
                    Environment.GetFolderPath Environment.SpecialFolder.ApplicationData,
                    "Gravemaskin"
                ))

        IO.Directory.CreateDirectory home |> ignore
        IO.Path.Combine(home, "sandbox.grav")

    // Fixed-step accumulator (house loop) + snapshot double buffer.
    let mutable accumulator = 0.0
    let fixedStep = 1.0 / float Tuning.TickRate
    let mutable previous = RenderSnapshot(Clumps.MaxClumps)
    let mutable current = RenderSnapshot(Clumps.MaxClumps)
    let mutable inputSequence = 0L

    // Cameras: orbit-follow (default, machine controls) and free fly (F1,
    // brush controls).
    let mutable flyMode = Environment.GetEnvironmentVariable "GRAV_FLY" = "1"
    let mutable cameraPosition = Vector3(32.0f, 10.0f, 44.0f)
    let mutable yaw = -1.57f
    let mutable pitch = -0.45f
    let mutable orbitYaw = 2.5f
    let mutable orbitPitch = 0.5f
    let mutable orbitDistance =
        argValue "--zoom" |> Option.map float32 |> Option.defaultValue 9.0f
    let mutable lastMouse = Vector2.Zero
    let mutable mouseInitialized = false
    let mutable flyToggleLatch = false
    let mutable brushHit: Vector3 voption = ValueNone
    let mutable cameraForward = Vector3(0.0f, -0.4f, -0.9f)
    let brushRadius = 0.45f

    let frameWatch = Stopwatch.StartNew()
    let mutable frameCount = 0L
    let mutable statFrames = 0
    let mutable statTime = 0.0
    let mutable statFps = 0.0f

    window.add_Load (fun () ->
        gl <- GL.GetApi window
        input <- window.CreateInput()

        world <-
            if previewDir.IsSome then
                // Clean flat stage for model previews.
                let config =
                    { CellSize = 0.25f
                      CellsX = 96
                      CellsY = 32
                      CellsZ = 96 }

                new World(0xD16D16UL, Sim.defaultThreadCount, Some(FlatSoil(config, Topsoil, 2.0f)))
            else
                Sim.createTerrainWorld 0xD16D16UL

        state <- world.SoilState.Value

        let rig =
            argValue "--machine"
            |> Option.map Tuning.rigByName
            |> Option.defaultValue Tuning.tb216Rig

        let spawnAt =
            if previewDir.IsSome then
                Vector3(12.0f, 0.0f, 12.0f)
            else
                Vector3(32.0f, 0.0f, 32.0f)

        world.SpawnMachineRig(rig, spawnAt) |> ignore

        if previewDir.IsNone then
            world.SeedRocks 48
        grains <- GrainPool(120_000, state)
        renderer <- new Renderer(gl, state)
        hud <- Hud(gl)

        // Audio is best-effort: no OpenAL, no problem, the game plays silent.
        audio <-
            try
                Some(new AudioSystem(settings.Volume))
            with _ ->
                None

        world.SnapshotInto previous
        world.SnapshotInto current

        input.Mice.[0].add_Scroll (fun _ scroll ->
            orbitDistance <- Math.Clamp(orbitDistance - scroll.Y * 1.2f, 3.0f, 30.0f)))

    window.add_Update (fun elapsed ->
        let keyboard = input.Keyboards.[0]
        let mouse = input.Mice.[0]

        // Camera: WASD + QE, hold right mouse to look.
        let mouseNow = Vector2(mouse.Position.X, mouse.Position.Y)

        if not mouseInitialized then
            lastMouse <- mouseNow
            mouseInitialized <- true

        // F1 toggles fly/brush mode (edge-latched).
        let f1 = keyboard.IsKeyPressed Key.F1

        if f1 && not flyToggleLatch then
            flyMode <- not flyMode

        flyToggleLatch <- f1

        let machinePosition =
            match world.Machine with
            | Some m -> world.Physics.Simulation.Bodies.[m.Chassis].Pose.Position
            | None -> Vector3.Zero

        if mouse.IsButtonPressed MouseButton.Right then
            let delta = mouseNow - lastMouse

            if flyMode then
                yaw <- yaw + delta.X * 0.003f
                pitch <- Math.Clamp(pitch - delta.Y * 0.003f, -1.5f, 1.5f)
            else
                orbitYaw <- orbitYaw + delta.X * 0.004f
                orbitPitch <- Math.Clamp(orbitPitch + delta.Y * 0.004f, 0.05f, 1.4f)

        lastMouse <- mouseNow

        let forward =
            if previewDir.IsSome then
                // Preview mode owns the camera (set per-angle in the render loop).
                cameraForward
            elif flyMode then
                Vector3(MathF.Cos yaw * MathF.Cos pitch, MathF.Sin pitch, MathF.Sin yaw * MathF.Cos pitch)
            else
                // Orbit: camera placed on a sphere around the machine.
                let offset =
                    Vector3(
                        MathF.Cos orbitYaw * MathF.Cos orbitPitch,
                        MathF.Sin orbitPitch,
                        MathF.Sin orbitYaw * MathF.Cos orbitPitch
                    )
                    * orbitDistance

                cameraPosition <- machinePosition + Vector3(0.0f, 1.0f, 0.0f) + offset
                Vector3.Normalize(machinePosition + Vector3(0.0f, 1.0f, 0.0f) - cameraPosition)

        if flyMode then
            let right = Vector3.Normalize(Vector3.Cross(forward, Vector3.UnitY))
            let speed = (if keyboard.IsKeyPressed Key.ShiftLeft then 24.0f else 9.0f) * float32 elapsed
            let mutable move = Vector3.Zero
            if keyboard.IsKeyPressed Key.W then move <- move + forward
            if keyboard.IsKeyPressed Key.S then move <- move - forward
            if keyboard.IsKeyPressed Key.D then move <- move + right
            if keyboard.IsKeyPressed Key.A then move <- move - right
            if keyboard.IsKeyPressed Key.E then move <- move + Vector3.UnitY
            if keyboard.IsKeyPressed Key.Q then move <- move - Vector3.UnitY

            if move.LengthSquared() > 0.0f then
                cameraPosition <- cameraPosition + Vector3.Normalize move * speed

        cameraForward <- forward
        brushHit <- if flyMode then raycastSurface state cameraPosition forward else ValueNone

        // Machine controls (ISO-flavored keyboard split, SPEC §7): A/D swing,
        // W/S stick, ↑/↓ or I/K boom, J/L bucket, Q/E left track, Z/C right.
        let axis negative positive =
            (if keyboard.IsKeyPressed positive then 1.0f else 0.0f)
            - (if keyboard.IsKeyPressed negative then 1.0f else 0.0f)

        // F5 save / F9 load (edge-latched). Loading swaps the whole world;
        // the renderer is rebuilt against the new soil state.
        let f5 = keyboard.IsKeyPressed Key.F5

        if f5 && not saveLatch then
            world.Save(savePath ())

        saveLatch <- f5
        let f9 = keyboard.IsKeyPressed Key.F9

        if f9 && not loadLatch && IO.File.Exists(savePath ()) then
            // Parse FIRST: a corrupt save must not take the live world with
            // it (review finding: dispose-then-parse crashed on bad files).
            match
                (try
                    Some(Sim.loadWorld 0xD16D16UL (savePath ()))
                 with _ ->
                     None)
            with
            | Some loadedWorld ->
                (world :> IDisposable).Dispose()
                world <- loadedWorld
                state <- world.SoilState.Value
                grains <- GrainPool(120_000, state)
                (renderer :> IDisposable).Dispose()
                renderer <- new Renderer(gl, state)
                world.SnapshotInto previous
                world.SnapshotInto current
            | None -> ()

        loadLatch <- f9

        // M toggles the machine menu; 1/2/3 swap rigs.
        let mKey = keyboard.IsKeyPressed Key.M

        if mKey && not menuLatch then
            menuOpen <- not menuOpen

        menuLatch <- mKey

        // H opens a compact control reference. Start with it visible so the
        // simulator's two-stick controls are discoverable without a README.
        let hKey = keyboard.IsKeyPressed Key.H

        if hKey && not helpLatch then
            helpOpen <- not helpOpen

        helpLatch <- hKey

        if menuOpen then
            let pick =
                if keyboard.IsKeyPressed Key.Number1 then Some Tuning.tb216Rig
                elif keyboard.IsKeyPressed Key.Number2 then Some Tuning.u17Rig
                elif keyboard.IsKeyPressed Key.Number3 then Some Tuning.cat320Rig
                else None

            match pick with
            | Some rig ->
                world.SwapMachine rig |> ignore
                menuOpen <- false
            | None -> ()

        // P toggles ISO/SAE (edge-latched, persisted).
        let pKey = keyboard.IsKeyPressed Key.P

        if pKey && not patternToggleLatch then
            settings <-
                { settings with
                    ControlPattern =
                        if settings.ControlPattern = ControlPattern.Iso then
                            ControlPattern.Sae
                        else
                            ControlPattern.Iso }

            Settings.save settings

        patternToggleLatch <- pKey

        let machineInput =
            if menuOpen || helpOpen then
                InputFrame.empty
            elif demoMode then
                // Scripted dig-swing-dump loop for demos and screenshots —
                // the real stroke: bite with the bucket open, crowd the
                // bench, curl through to fill, lift, swing, pour.
                match (world.Tick / 110L) % 8L with
                | 0L -> { InputFrame.empty with Boom = -1.0f }
                | 1L -> { InputFrame.empty with Stick = -1.0f }
                | 2L
                | 3L ->
                    { InputFrame.empty with
                        Stick = -0.5f
                        Bucket = -1.0f }
                | 4L -> { InputFrame.empty with Boom = 1.0f; Bucket = -0.3f }
                | 5L -> { InputFrame.empty with Swing = 0.8f }
                | _ -> { InputFrame.empty with Bucket = 1.0f } // long open: pour it all
            elif flyMode then
                InputFrame.empty
            else
                let keyboardFrame =
                    { InputFrame.empty with
                        Swing = axis Key.A Key.D
                        Stick = axis Key.S Key.W // W = stick out (extend away)
                        Boom = axis Key.K Key.I + axis Key.Down Key.Up
                        Bucket = axis Key.J Key.L
                        LeftTrack = axis Key.E Key.Q
                        RightTrack = axis Key.C Key.Z }

                if input.Gamepads.Count > 0 then
                    let pad = input.Gamepads.[0]

                    let dead (value: float32) =
                        if MathF.Abs value < settings.GamepadDeadzone then 0.0f else value

                    let leftX = dead pad.Thumbsticks.[0].X
                    let leftY = dead pad.Thumbsticks.[0].Y
                    let rightX = dead pad.Thumbsticks.[1].X
                    let rightY = dead pad.Thumbsticks.[1].Y

                    // Shared axes: left-X swing, right-X bucket. ISO: left-Y
                    // stick / right-Y boom; SAE swaps the Y axes.
                    let stickY, boomY =
                        if settings.ControlPattern = ControlPattern.Iso then
                            leftY, rightY
                        else
                            rightY, leftY

                    // Triggers drive tracks forward, bumpers reverse.
                    let trigger (value: float32) = MathF.Max(value, 0.0f)

                    let trackLeft =
                        trigger pad.Triggers.[0].Position
                        - (if pad.Buttons.[int ButtonName.LeftBumper].Pressed then 1.0f else 0.0f)

                    let trackRight =
                        trigger pad.Triggers.[1].Position
                        - (if pad.Buttons.[int ButtonName.RightBumper].Pressed then 1.0f else 0.0f)

                    { keyboardFrame with
                        Swing = keyboardFrame.Swing + leftX
                        Stick = keyboardFrame.Stick + stickY
                        Boom = keyboardFrame.Boom - boomY
                        Bucket = keyboardFrame.Bucket + rightX
                        LeftTrack = keyboardFrame.LeftTrack + trackLeft
                        RightTrack = keyboardFrame.RightTrack + trackRight }
                else
                    keyboardFrame

        // Fixed-step sim.
        accumulator <- min 0.25 (accumulator + elapsed)

        while accumulator >= fixedStep do
            // Dig/dump under the brush while held (one op per tick). Never
            // during replay: brush edits are live, unrecorded input and
            // would fork the reproduction (review finding).
            match (if replayFrames.IsSome then ValueNone else brushHit) with
            | ValueSome hit when mouse.IsButtonPressed MouseButton.Left ->
                world.CarveSphere(hit, brushRadius) |> ignore
            | ValueSome hit when keyboard.IsKeyPressed Key.G ->
                world.SoilState
                |> Option.iter (fun soil -> Soil.injectLoose soil hit 40.0 Topsoil)
            | _ -> ()

            inputSequence <- inputSequence + 1L

            let frame =
                match replayFrames with
                | Some frames when replayIndex < frames.Count ->
                    replayIndex <- replayIndex + 1
                    frames.[replayIndex - 1]
                | _ -> { machineInput with Sequence = inputSequence }

            match recordWriter with
            | Some writer ->
                writer.Write frame.Sequence
                writer.Write frame.Swing
                writer.Write frame.Boom
                writer.Write frame.Stick
                writer.Write frame.Bucket
                writer.Write frame.LeftTrack
                writer.Write frame.RightTrack
                writer.Write(int frame.Buttons)
            | None -> ()

            world.Step frame |> ignore

            match audio with
            | Some system -> system.Update(world.Machine, world.Events, float32 fixedStep)
            | None -> ()

            // Grain emission: the sim says WHAT happened; the grain layer
            // turns it into flying dirt.
            match world.Machine with
            | Some m ->
                let tip = m.BucketTipPosition
                let tipVelocity = m.CuttingEdgeVelocity
                let struct (matByte, moistByte) = Soil.surfaceSample state tip.X tip.Z
                let tipMaterial = Volume.materialOfByte matByte

                for event in world.Events do
                    match event with
                    | SoilDumped(mass, dumpedMat) ->
                        // A pouring stream: grains inherit the bucket edge's
                        // motion and rain out of the opening.
                        // Dirt leaves a bucket downward however fast the
                        // bucket itself is swinging up — clamp the vertical.
                        let pourVelocity =
                            Vector3(tipVelocity.X, MathF.Min(tipVelocity.Y, 0.2f) - 0.6f, tipVelocity.Z)

                        grains.SpawnBurst(
                            tip + Vector3(0.0f, -0.05f, 0.0f),
                            pourVelocity,
                            0.35f,
                            Volume.materialOfByte dumpedMat,
                            moistByte,
                            Math.Clamp(int (mass * 330.0f), 30, 700)
                        )
                    | DigStarted ->
                        // Crumble off the cutting edge, drawn INTO the mouth
                        // along with the bucket's sweep — the dirt visibly
                        // flows into the bucket it's filling.
                        let bucketCenter =
                            world.Physics.Simulation.Bodies.[m.Bucket].Pose.Position

                        let intoMouth =
                            let toCenter = bucketCenter - tip

                            if toCenter.LengthSquared() > 1e-4f then
                                Vector3.Normalize toCenter * 1.6f
                            else
                                Vector3.Zero

                        grains.SpawnBurst(tip, tipVelocity * 0.5f + intoMouth, 0.4f, tipMaterial, moistByte, 22)
                    | WallCollapsed ->
                        // The wedge clumps burst into clusters on their own;
                        // add a dust breath at the machine's general area.
                        ()
                    | RockStruck -> grains.SpawnBurst(tip, Vector3(0.0f, 0.8f, 0.0f), 1.2f, Gravel, 0uy, 24)
                    | _ -> ()

                // Track spray while driving.
                for side in 0..1 do
                    if MathF.Abs(m.TrackAxis side) > 0.35f then
                        let contact = m.TrackContactPoint side
                        let ground = Soil.surfaceHeight state contact.X contact.Z

                        if contact.Y - ground < 0.35f * m.Rig.Scale then
                            let struct (trackMat, trackMoist) = Soil.surfaceSample state contact.X contact.Z

                            grains.SpawnBurst(
                                Vector3(contact.X, ground + 0.06f, contact.Z),
                                Vector3(0.0f, 0.5f, 0.0f),
                                0.6f,
                                Volume.materialOfByte trackMat,
                                trackMoist,
                                4
                            )
            | None -> ()

            let swap = previous
            previous <- current
            current <- swap
            world.SnapshotInto current
            accumulator <- accumulator - fixedStep

        // The grain layer runs on wall-clock, outside the deterministic sim.
        grains.Step(float32 elapsed))

    window.add_Render (fun elapsed ->
        let size = window.FramebufferSize
        gl.Viewport(0, 0, uint32 size.X, uint32 size.Y)

        let view =
            Matrix4x4.CreateLookAt(cameraPosition, cameraPosition + cameraForward, Vector3.UnitY)

        let projection =
            Matrix4x4.CreatePerspectiveFieldOfView(
                1.05f,
                float32 size.X / float32 size.Y,
                0.05f,
                300.0f
            )

        renderer.RebuildDirtyTiles 4
        let alpha = float32 (accumulator / fixedStep)
        renderer.Draw(view * projection, cameraPosition, previous, current, alpha, grains, brushHit, brushRadius)

        // HUD overlay.
        let uiScale = MathF.Max(float32 size.Y / 600.0f, 1.5f)
        let showHud = previewDir.IsNone
        let white = Vector4(0.95f, 0.95f, 0.92f, 0.9f)
        let orange = Vector4(0.95f, 0.55f, 0.1f, 0.95f)
        let red = Vector4(0.95f, 0.2f, 0.15f, 0.95f)
        hud.Begin(size.X, size.Y)

        match (if showHud then world.Machine else None) with
        | Some m ->
            let margin = 10.0f * uiScale
            let line = 11.0f * uiScale
            hud.Text(margin, margin, uiScale, white, $"PAYLOAD %.0f{m.BucketLoadKg} KG")

            hud.Bar(
                margin,
                margin + line,
                70.0f * uiScale,
                5.0f * uiScale,
                float32 (m.BucketLoadKg / Tuning.BucketCapacityKg),
                orange
            )

            // Pump-circuit saturation, labeled by the dominant function on
            // each circuit: a full bar = that pump is maxed out, which is
            // exactly when everything on it slows down.
            let circuitOf = [| 0; 1; 0; 2; 0; 1 |]
            let circuitLabel = [| "BOOM"; "STICK"; "SWING" |]
            hud.Text(margin, margin + line * 2.5f, uiScale * 0.7f, Vector4(0.85f, 0.85f, 0.82f, 0.7f), "PUMP LOAD")

            for circuit in 0..2 do
                let mutable saturation = 0.0f

                for f in 0..5 do
                    if circuitOf.[f] = circuit then
                        saturation <- MathF.Max(saturation, 1.0f - m.GrantedScale f)

                let y = margin + line * (3.5f + float32 circuit)
                hud.Text(margin, y, uiScale * 0.8f, white, circuitLabel.[circuit])
                hud.Bar(margin + 34.0f * uiScale, y, 44.0f * uiScale, 4.0f * uiScale, saturation, orange)

            let tiltDegrees = m.ChassisTilt * 57.3f

            hud.Text(
                margin,
                margin + line * 7.0f,
                uiScale * 0.8f,
                (if tiltDegrees > 18.0f then red else white),
                $"TILT %.0f{tiltDegrees}"
            )

            if m.StallActive && (world.Tick / 15L) % 2L = 0L then
                hud.Text(margin, margin + line * 8.2f, uiScale, red, "STALL")
        | None -> ()

        // Small stat line under the tilt readout.
        if showHud then
            hud.Text(
            10.0f * uiScale,
            10.0f * uiScale + 11.0f * uiScale * 9.2f,
            uiScale * 0.6f,
            Vector4(0.9f, 0.9f, 0.88f, 0.55f),
                (let pattern = (if settings.ControlPattern = ControlPattern.Iso then "ISO" else "SAE")
                 $"%.0f{statFps} FPS  {pattern}")
            )

        if showHud then
            // Control diagram: a TB216-liveried side profile (dark tracks,
            // red skirt, white house/boom, glass, steel bucket) built from
            // rects — stepped rects fake the gooseneck diagonals. Keys sit
            // on the parts they drive.
            let u = uiScale
            let ox = float32 size.X - 156.0f * u
            let oy = float32 size.Y - 100.0f * u
            hud.Solid(ox, oy, 156.0f * u, 90.0f * u, Vector4(0.05f, 0.05f, 0.06f, 0.30f))
            let track = Vector4(0.13f, 0.13f, 0.14f, 0.72f)
            let wheel = Vector4(0.04f, 0.04f, 0.05f, 0.8f)
            let redSkirt = Vector4(0.72f, 0.10f, 0.11f, 0.72f)
            let body = Vector4(0.92f, 0.90f, 0.85f, 0.68f)
            let glass = Vector4(0.10f, 0.13f, 0.18f, 0.75f)
            let steel = Vector4(0.35f, 0.35f, 0.38f, 0.72f)
            let key = Vector4(1.0f, 1.0f, 1.0f, 0.95f)
            // undercarriage: track band, wheels, dozer blade
            hud.Solid(ox + 14.0f * u, oy + 60.0f * u, 48.0f * u, 10.0f * u, track)
            hud.Solid(ox + 17.0f * u, oy + 63.0f * u, 6.0f * u, 6.0f * u, wheel)
            hud.Solid(ox + 53.0f * u, oy + 63.0f * u, 6.0f * u, 6.0f * u, wheel)
            hud.Solid(ox + 60.0f * u, oy + 58.0f * u, 8.0f * u, 3.0f * u, steel)
            hud.Solid(ox + 66.0f * u, oy + 54.0f * u, 4.0f * u, 14.0f * u, steel)
            // house: red skirt band, white body, cab + glass
            hud.Solid(ox + 16.0f * u, oy + 54.0f * u, 44.0f * u, 6.0f * u, redSkirt)
            hud.Solid(ox + 18.0f * u, oy + 38.0f * u, 40.0f * u, 16.0f * u, body)
            hud.Solid(ox + 20.0f * u, oy + 22.0f * u, 18.0f * u, 20.0f * u, body)
            hud.Solid(ox + 31.0f * u, oy + 25.0f * u, 6.0f * u, 13.0f * u, glass)
            // gooseneck boom: white steps up to the knee...
            hud.Solid(ox + 58.0f * u, oy + 44.0f * u, 11.0f * u, 5.0f * u, body)
            hud.Solid(ox + 66.0f * u, oy + 38.0f * u, 11.0f * u, 5.0f * u, body)
            hud.Solid(ox + 74.0f * u, oy + 32.0f * u, 12.0f * u, 5.0f * u, body)
            // ...then the stick steps down to the bucket
            hud.Solid(ox + 86.0f * u, oy + 34.0f * u, 5.0f * u, 10.0f * u, body)
            hud.Solid(ox + 89.0f * u, oy + 43.0f * u, 5.0f * u, 10.0f * u, body)
            // bucket: steel with a tooth
            hud.Solid(ox + 88.0f * u, oy + 53.0f * u, 11.0f * u, 8.0f * u, steel)
            hud.Solid(ox + 95.0f * u, oy + 61.0f * u, 6.0f * u, 3.0f * u, wheel)
            // keys on the parts they drive
            hud.Text(ox + 6.0f * u, oy + 28.0f * u, u * 0.9f, key, "A")
            hud.Text(ox + 44.0f * u, oy + 28.0f * u, u * 0.9f, key, "D")
            hud.Text(ox + 66.0f * u, oy + 22.0f * u, u * 0.9f, key, "I")
            hud.Text(ox + 60.0f * u, oy + 52.0f * u, u * 0.9f, key, "K")
            hud.Text(ox + 100.0f * u, oy + 30.0f * u, u * 0.9f, key, "W")
            hud.Text(ox + 100.0f * u, oy + 42.0f * u, u * 0.9f, key, "S")
            hud.Text(ox + 78.0f * u, oy + 64.0f * u, u * 0.9f, key, "J")
            hud.Text(ox + 104.0f * u, oy + 64.0f * u, u * 0.9f, key, "L")
            hud.Text(ox + 14.0f * u, oy + 76.0f * u, u * 0.75f, key, "QE")
            hud.Text(ox + 42.0f * u, oy + 76.0f * u, u * 0.75f, key, "ZC")

            hud.Text(
                float32 size.X - 43.0f * u,
                10.0f * u,
                u * 0.65f,
                Vector4(0.95f, 0.95f, 0.92f, 0.65f),
                "H HELP"
            )

        if helpOpen then
            let u = uiScale
            let panelWidth = 280.0f * u
            let panelHeight = 204.0f * u
            let cx = (float32 size.X - panelWidth) * 0.5f
            let cy = (float32 size.Y - panelHeight) * 0.5f
            let muted = Vector4(0.78f, 0.80f, 0.80f, 0.82f)
            let pattern = if settings.ControlPattern = ControlPattern.Iso then "ISO" else "SAE"
            hud.Solid(cx, cy, panelWidth, panelHeight, Vector4(0.025f, 0.03f, 0.035f, 0.90f))
            hud.Solid(cx, cy, 4.0f * u, panelHeight, orange)
            hud.Text(cx + 14.0f * u, cy + 12.0f * u, u * 1.15f, white, "OPERATE THE MACHINE")
            hud.Text(cx + 14.0f * u, cy + 32.0f * u, u * 0.65f, muted, $"{pattern} TWO STICK CONTROLS")

            let helpLine row keys label =
                let y = cy + (51.0f + float32 row * 14.0f) * u
                hud.Text(cx + 14.0f * u, y, u * 0.78f, orange, keys)
                hud.Text(cx + 76.0f * u, y, u * 0.78f, white, label)

            helpLine 0 "A / D" "SWING LEFT / RIGHT"
            helpLine 1 "W / S" "STICK OUT / IN"
            helpLine 2 "I / K" "BOOM UP / DOWN"
            helpLine 3 "J / L" "BUCKET CURL / DUMP"
            helpLine 4 "Q / E" "LEFT TRACK"
            helpLine 5 "Z / C" "RIGHT TRACK"
            helpLine 6 "RMB" "ORBIT CAMERA"
            helpLine 7 "WHEEL" "CAMERA DISTANCE"
            hud.Text(cx + 14.0f * u, cy + 177.0f * u, u * 0.62f, muted, "F1 FREE CAMERA   M MACHINES   P ISO / SAE")
            hud.Text(cx + 14.0f * u, cy + 189.0f * u, u * 0.62f, muted, "F5 SAVE   F9 LOAD")
            hud.Text(cx + 217.0f * u, cy + 12.0f * u, u * 0.72f, orange, "H CLOSE")

        if menuOpen then
            let cx = float32 size.X * 0.5f - 90.0f * uiScale
            let cy = float32 size.Y * 0.5f - 50.0f * uiScale
            hud.Solid(cx, cy, 180.0f * uiScale, 92.0f * uiScale, Vector4(0.04f, 0.04f, 0.05f, 0.82f))
            hud.Text(cx + 12.0f * uiScale, cy + 10.0f * uiScale, uiScale, white, "MACHINE")

            let current =
                match world.Machine with
                | Some m -> m.Rig.Spec.Name
                | None -> ""

            let entry (index: int) (label: string) (name: string) =
                let color =
                    if name = current then
                        orange
                    else
                        Vector4(0.85f, 0.85f, 0.85f, 0.85f)

                hud.Text(cx + 12.0f * uiScale, cy + (26.0f + float32 index * 16.0f) * uiScale, uiScale * 0.9f, color, label)

            entry 0 "1  TAKEUCHI TB216" "TB216"
            entry 1 "2  KUBOTA U17" "U17"
            entry 2 "3  CAT 320" "Cat 320"
            hud.Text(cx + 12.0f * uiScale, cy + 78.0f * uiScale, uiScale * 0.65f, Vector4(0.7f, 0.7f, 0.7f, 0.7f), "M CLOSE")

        hud.End()

        // Frame stats feed the HUD, not the title.
        statFrames <- statFrames + 1
        statTime <- statTime + elapsed

        if statTime >= 0.5 then
            statFps <- float32 statFrames / float32 statTime
            statFrames <- 0
            statTime <- 0.0

        frameCount <- frameCount + 1L

        match previewDir with
        | Some directory ->
            let machineCenter =
                match world.Machine with
                | Some m ->
                    world.Physics.Simulation.Bodies.[m.Chassis].Pose.Position
                    + Vector3(0.6f, 0.3f, 0.0f)
                | None -> Vector3.Zero

            let angleIndex = int ((frameCount - 60L) / 20L)

            if frameCount >= 60L && angleIndex < previewAngles.Length then
                let _, offset = previewAngles.[angleIndex]
                cameraPosition <- machineCenter + offset
                cameraForward <- Vector3.Normalize(machineCenter - cameraPosition)

                // Shoot at the END of each angle's window (state settled).
                if (frameCount - 60L) % 20L = 19L then
                    let name, _ = previewAngles.[angleIndex]
                    IO.Directory.CreateDirectory directory |> ignore
                    screenshot gl size.X size.Y (IO.Path.Combine(directory, name + ".bmp"))
            elif angleIndex >= previewAngles.Length then
                window.Close()
        | None -> ()

        let captureReached =
            (match shotTick with
             | Some target -> world.Tick >= target
             | None -> false)
            || (match maxFrames with
                | Some target -> frameCount >= int64 target
                | None -> false)

        if captureReached then
            match screenshotPath with
            | Some path ->
                let directory = IO.Path.GetDirectoryName path

                if not (String.IsNullOrEmpty directory) then
                    IO.Directory.CreateDirectory directory |> ignore

                screenshot gl size.X size.Y path
                printfn $"screenshot written to {path}"

                match world.Machine with
                | Some m ->
                    let p = world.Physics.Simulation.Bodies.[m.Chassis].Pose.Position
                    printfn $"chassis at {p}, surface {Soil.surfaceHeight state p.X p.Z}"
                | None -> ()
            | None -> ()

            window.Close())

    window.add_Closing (fun () -> recordWriter |> Option.iter (fun writer -> writer.Dispose()))

    window.Run()
    0

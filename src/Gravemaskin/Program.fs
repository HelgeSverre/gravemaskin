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
    let mutable patternToggleLatch = false
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
    let mutable cameraPosition = Vector3(16.0f, 8.0f, 26.0f)
    let mutable yaw = -1.57f
    let mutable pitch = -0.45f
    let mutable orbitYaw = 2.5f
    let mutable orbitPitch = 0.5f
    let mutable orbitDistance = 9.0f
    let mutable lastMouse = Vector2.Zero
    let mutable mouseInitialized = false
    let mutable flyToggleLatch = false
    let mutable brushHit: Vector3 voption = ValueNone
    let mutable cameraForward = Vector3(0.0f, -0.4f, -0.9f)
    let brushRadius = 0.45f

    // Stats for the title bar (the Phase 2 "HUD").
    let frameWatch = Stopwatch.StartNew()
    let mutable frameCount = 0L
    let mutable statFrames = 0
    let mutable statTime = 0.0

    window.add_Load (fun () ->
        gl <- GL.GetApi window
        input <- window.CreateInput()

        world <- Sim.createTerrainWorld 0xD16D16UL
        state <- world.SoilState.Value

        let rig =
            argValue "--machine"
            |> Option.map Tuning.rigByName
            |> Option.defaultValue Tuning.u17Rig

        world.SpawnMachineRig(rig, Vector3(16.0f, 0.0f, 16.0f)) |> ignore
        world.SeedRocks 24
        renderer <- Renderer(gl, state)
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
            if flyMode then
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
            (world :> IDisposable).Dispose()
            world <- Sim.loadWorld 0xD16D16UL (savePath ())
            state <- world.SoilState.Value
            // ponytail: the old renderer's GL objects leak on load — a few
            // hundred KB per load, cleanup when it ever matters.
            renderer <- Renderer(gl, state)
            world.SnapshotInto previous
            world.SnapshotInto current

        loadLatch <- f9

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
            if flyMode then
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
            // Dig/dump under the brush while held (one op per tick).
            match brushHit with
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

            let swap = previous
            previous <- current
            current <- swap
            world.SnapshotInto current
            accumulator <- accumulator - fixedStep)

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

        renderer.RebuildDirtyTiles 8
        let alpha = float32 (accumulator / fixedStep)
        renderer.Draw(view * projection, cameraPosition, previous, current, alpha, brushHit, brushRadius)

        // HUD overlay.
        let uiScale = MathF.Max(float32 size.Y / 600.0f, 1.5f)
        let white = Vector4(0.95f, 0.95f, 0.92f, 0.9f)
        let orange = Vector4(0.95f, 0.55f, 0.1f, 0.95f)
        let red = Vector4(0.95f, 0.2f, 0.15f, 0.95f)
        hud.Begin(size.X, size.Y)

        match world.Machine with
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

            // Circuit saturation: a full bar = that pump is maxed out, which
            // is exactly when everything on it slows down.
            let circuitOf = [| 0; 1; 0; 2; 0; 1 |]

            for circuit in 0..2 do
                let mutable saturation = 0.0f

                for f in 0..5 do
                    if circuitOf.[f] = circuit then
                        saturation <- MathF.Max(saturation, 1.0f - m.GrantedScale f)

                let y = margin + line * (2.5f + float32 circuit)
                hud.Text(margin, y, uiScale * 0.8f, white, $"P%d{circuit + 1}")
                hud.Bar(margin + 18.0f * uiScale, y, 50.0f * uiScale, 4.0f * uiScale, saturation, orange)

            let tiltDegrees = m.ChassisTilt * 57.3f

            hud.Text(
                margin,
                margin + line * 6.0f,
                uiScale * 0.8f,
                (if tiltDegrees > 11.0f then red else white),
                $"TILT %.0f{tiltDegrees}"
            )

            if m.StallActive && (world.Tick / 15L) % 2L = 0L then
                hud.Text(margin, margin + line * 7.2f, uiScale, red, "STALL")
        | None -> ()

        let hint =
            if flyMode then
                "FLY  WASD MOVE  LMB DIG  G DUMP  F1 OPERATE"
            else
                let pattern = if settings.ControlPattern = ControlPattern.Iso then "ISO" else "SAE"
                $"{pattern}  AD SWING  WS STICK  IK BOOM  JL BUCKET  QE ZC TRACKS  P PATTERN  F1 FLY"

        hud.Text(10.0f * uiScale, float32 size.Y - 12.0f * uiScale, uiScale * 0.7f, white, hint)
        hud.End()

        // Title-bar stats (a real HUD arrives in Phase 5).
        statFrames <- statFrames + 1
        statTime <- statTime + elapsed

        if statTime >= 0.5 then
            let fps = float statFrames / statTime

            let payload =
                match world.Machine with
                | Some m -> $" · payload {m.BucketLoadKg:F0} kg" + (if m.StallActive then " · STALL" else "")
                | None -> ""

            window.Title <-
                $"GRAVEMASKIN — {fps:F0} fps · clumps {world.Clumps.Count}{payload} · AD swing WS stick IK boom JL bucket QE/ZC tracks · F1 fly/brush"

            statFrames <- 0
            statTime <- 0.0

        frameCount <- frameCount + 1L

        match maxFrames with
        | Some limit when frameCount >= int64 limit ->
            recordWriter |> Option.iter (fun writer -> writer.Dispose())

            match screenshotPath with
            | Some path ->
                screenshot gl size.X size.Y path
                printfn $"screenshot written to {path}"

                match world.Machine with
                | Some m ->
                    let p = world.Physics.Simulation.Bodies.[m.Chassis].Pose.Position
                    printfn $"chassis at {p}, surface {Soil.surfaceHeight state p.X p.Z}"
                | None -> ()
            | None -> ()

            window.Close()
        | _ -> ())

    window.Run()
    0

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

    let mutable options = WindowOptions.Default
    options.Title <- "GRAVEMASKIN"
    options.Size <- Vector2D<int>(1440, 900)
    options.VSync <- true

    options.API <-
        GraphicsAPI(ContextAPI.OpenGL, ContextProfile.Core, ContextFlags.ForwardCompatible, APIVersion(4, 1))

    use window = Window.Create options

    let mutable gl = Unchecked.defaultof<GL>
    let mutable input = Unchecked.defaultof<IInputContext>
    let mutable renderer = Unchecked.defaultof<Renderer>
    let mutable world = Unchecked.defaultof<World>
    let mutable state = Unchecked.defaultof<SoilState>

    // Fixed-step accumulator (house loop) + snapshot double buffer.
    let mutable accumulator = 0.0
    let fixedStep = 1.0 / float Tuning.TickRate
    let mutable previous = RenderSnapshot(Clumps.MaxClumps)
    let mutable current = RenderSnapshot(Clumps.MaxClumps)
    let mutable inputSequence = 0L

    // Fly camera.
    let mutable cameraPosition = Vector3(16.0f, 8.0f, 26.0f)
    let mutable yaw = -1.57f
    let mutable pitch = -0.45f
    let mutable lastMouse = Vector2.Zero
    let mutable mouseInitialized = false
    let mutable brushHit: Vector3 voption = ValueNone
    let brushRadius = 0.45f

    // Stats for the title bar (the Phase 2 "HUD").
    let frameWatch = Stopwatch.StartNew()
    let mutable frameCount = 0L
    let mutable statFrames = 0
    let mutable statTime = 0.0

    window.add_Load (fun () ->
        gl <- GL.GetApi window
        input <- window.CreateInput()

        world <- Sim.createSoilWorld 0xD16D16UL Topsoil 2.0f
        state <- world.SoilState.Value
        renderer <- Renderer(gl, state)
        world.SnapshotInto previous
        world.SnapshotInto current)

    window.add_Update (fun elapsed ->
        let keyboard = input.Keyboards.[0]
        let mouse = input.Mice.[0]

        // Camera: WASD + QE, hold right mouse to look.
        let mouseNow = Vector2(mouse.Position.X, mouse.Position.Y)

        if not mouseInitialized then
            lastMouse <- mouseNow
            mouseInitialized <- true

        if mouse.IsButtonPressed MouseButton.Right then
            let delta = mouseNow - lastMouse
            yaw <- yaw + delta.X * 0.003f
            pitch <- Math.Clamp(pitch - delta.Y * 0.003f, -1.5f, 1.5f)

        lastMouse <- mouseNow

        let forward =
            Vector3(MathF.Cos yaw * MathF.Cos pitch, MathF.Sin pitch, MathF.Sin yaw * MathF.Cos pitch)

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

        brushHit <- raycastSurface state cameraPosition forward

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

            world.Step { InputFrame.empty with Sequence = inputSequence } |> ignore

            let swap = previous
            previous <- current
            current <- swap
            world.SnapshotInto current
            accumulator <- accumulator - fixedStep)

    window.add_Render (fun elapsed ->
        let size = window.FramebufferSize
        gl.Viewport(0, 0, uint32 size.X, uint32 size.Y)

        let forward =
            Vector3(MathF.Cos yaw * MathF.Cos pitch, MathF.Sin pitch, MathF.Sin yaw * MathF.Cos pitch)

        let view = Matrix4x4.CreateLookAt(cameraPosition, cameraPosition + forward, Vector3.UnitY)

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

        // Title-bar stats (a real HUD arrives in Phase 5).
        statFrames <- statFrames + 1
        statTime <- statTime + elapsed

        if statTime >= 0.5 then
            let fps = float statFrames / statTime

            window.Title <-
                $"GRAVEMASKIN — {fps:F0} fps · tick {world.Tick} · clumps {world.Clumps.Count} · LMB dig · G dump · RMB look · WASD fly"

            statFrames <- 0
            statTime <- 0.0

        frameCount <- frameCount + 1L

        match maxFrames with
        | Some limit when frameCount >= int64 limit ->
            match screenshotPath with
            | Some path ->
                screenshot gl size.X size.Y path
                printfn $"screenshot written to {path}"
            | None -> ()

            window.Close()
        | _ -> ())

    window.Run()
    0

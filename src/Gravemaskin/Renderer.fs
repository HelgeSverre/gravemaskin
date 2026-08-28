namespace Gravemaskin.Shell

#nowarn "9"

open System
open System.Numerics
open Silk.NET.OpenGL
open Gravemaskin

/// All GL state. Terrain is one VAO/VBO per soil tile rebuilt on dirty
/// (orphaned uploads only — the classic 4.1 stall lives in glBufferSubData);
/// clumps are one instanced icosahedron draw.
type Renderer(gl: GL, state: SoilState) =
    let tileVerts = SoilConfig.TileSize + 1

    let terrainProgram = GlUtil.program gl Shaders.terrainVertex Shaders.terrainFragment
    let clodProgram = GlUtil.program gl Shaders.clodVertex Shaders.clodFragment
    let solidProgram = GlUtil.program gl Shaders.solidVertex Shaders.clodFragment

    // Unit cube with face normals, for machine parts.
    let cubeVerts =
        // 6 faces × 4 verts × (pos3 + normal3)
        let faces =
            [| Vector3.UnitX; -Vector3.UnitX; Vector3.UnitY; -Vector3.UnitY; Vector3.UnitZ; -Vector3.UnitZ |]

        let data = ResizeArray<float32>()

        for normal in faces do
            let u = if MathF.Abs normal.Y > 0.9f then Vector3.UnitX else Vector3.Cross(Vector3.UnitY, normal)
            let v = Vector3.Cross(normal, u)

            for (su, sv) in [| (-1.0f, -1.0f); (1.0f, -1.0f); (1.0f, 1.0f); (-1.0f, 1.0f) |] do
                let p = (normal + u * su + v * sv) * 0.5f

                for value in [| p.X; p.Y; p.Z; normal.X; normal.Y; normal.Z |] do
                    data.Add value

        data.ToArray()

    let cubeIndices =
        [| for face in 0..5 do
               let b = uint32 (face * 4)
               yield! [| b; b + 1u; b + 2u; b; b + 2u; b + 3u |] |]

    let cubeVao = gl.GenVertexArray()
    let cubeVbo = gl.GenBuffer()
    let cubeIbo = gl.GenBuffer()

    /// Visual boxes per machine body part: (body index, local offset, size, color).
    let machineBoxes =
        [| 0, Vector3(0.0f, -0.1f, -0.55f), Vector3(1.8f, 0.35f, 0.35f), Vector3(0.12f, 0.12f, 0.13f) // track L
           0, Vector3(0.0f, -0.1f, 0.55f), Vector3(1.8f, 0.35f, 0.35f), Vector3(0.12f, 0.12f, 0.13f) // track R
           0, Vector3(0.0f, 0.05f, 0.0f), Vector3(1.5f, 0.3f, 1.0f), Vector3(0.25f, 0.25f, 0.27f) // undercarriage
           1, Vector3(0.1f, 0.0f, 0.0f), Vector3(1.05f, 0.7f, 0.95f), Vector3(0.85f, 0.45f, 0.08f) // cab
           1, Vector3(-0.75f, -0.1f, 0.0f), Vector3(0.45f, 0.5f, 0.9f), Vector3(0.3f, 0.3f, 0.32f) // counterweight
           2, Vector3.Zero, Vector3(1.9f, 0.18f, 0.15f), Vector3(0.85f, 0.45f, 0.08f) // boom
           3, Vector3.Zero, Vector3(1.1f, 0.14f, 0.12f), Vector3(0.85f, 0.45f, 0.08f) // stick
           4, Vector3.Zero, Vector3(0.5f, 0.4f, 0.6f), Vector3(0.2f, 0.2f, 0.22f) |] // bucket

    // Shared index buffer: same grid topology for every tile.
    let tileIndexCount = SoilConfig.TileSize * SoilConfig.TileSize * 6

    let tileIndices =
        let indices = Array.zeroCreate<uint32> tileIndexCount
        let mutable i = 0

        for z in 0 .. SoilConfig.TileSize - 1 do
            for x in 0 .. SoilConfig.TileSize - 1 do
                let v0 = uint32 (z * tileVerts + x)
                let v1 = v0 + 1u
                let v2 = v0 + uint32 tileVerts
                let v3 = v2 + 1u
                indices.[i] <- v0
                indices.[i + 1] <- v2
                indices.[i + 2] <- v1
                indices.[i + 3] <- v1
                indices.[i + 4] <- v2
                indices.[i + 5] <- v3
                i <- i + 6

        indices

    let tileCount = state.TilesX * state.TilesZ
    let tileVaos = Array.zeroCreate<uint32> tileCount
    let tileVbos = Array.zeroCreate<uint32> tileCount
    let sharedIbo = gl.GenBuffer()
    // pos3 + normal3 + color3 interleaved.
    let vertexFloats = 9
    let vertexScratch = Array.zeroCreate<float32> (tileVerts * tileVerts * vertexFloats)

    // Unit icosahedron for clods.
    let icoVerts, icoIndices =
        let t = (1.0f + MathF.Sqrt 5.0f) / 2.0f

        let raw =
            [| Vector3(-1f, t, 0f)
               Vector3(1f, t, 0f)
               Vector3(-1f, -t, 0f)
               Vector3(1f, -t, 0f)
               Vector3(0f, -1f, t)
               Vector3(0f, 1f, t)
               Vector3(0f, -1f, -t)
               Vector3(0f, 1f, -t)
               Vector3(t, 0f, -1f)
               Vector3(t, 0f, 1f)
               Vector3(-t, 0f, -1f)
               Vector3(-t, 0f, 1f) |]
            |> Array.map Vector3.Normalize

        let faces =
            [| 0; 11; 5; 0; 5; 1; 0; 1; 7; 0; 7; 10; 0; 10; 11
               1; 5; 9; 5; 11; 4; 11; 10; 2; 10; 7; 6; 7; 1; 8
               3; 9; 4; 3; 4; 2; 3; 2; 6; 3; 6; 8; 3; 8; 9
               4; 9; 5; 2; 4; 11; 6; 2; 10; 8; 6; 7; 9; 8; 1 |]
            |> Array.map uint32

        let flat = Array.zeroCreate<float32> (raw.Length * 3)

        for i in 0 .. raw.Length - 1 do
            flat.[i * 3] <- raw.[i].X
            flat.[i * 3 + 1] <- raw.[i].Y
            flat.[i * 3 + 2] <- raw.[i].Z

        flat, faces

    let clodVao = gl.GenVertexArray()
    let clodVbo = gl.GenBuffer()
    let clodIbo = gl.GenBuffer()
    let clodInstanceVbo = gl.GenBuffer()
    // xyz + radius + rgb per instance.
    let instanceFloats = 7
    let instanceScratch = Array.zeroCreate<float32> ((Clumps.MaxClumps + 1) * instanceFloats)

    let materialColor (mat: byte) =
        match int mat with
        | 0 -> Vector3(0.36f, 0.27f, 0.18f) // topsoil
        | 1 -> Vector3(0.72f, 0.62f, 0.44f) // dry sand
        | 2 -> Vector3(0.52f, 0.44f, 0.32f) // wet sand
        | 3 -> Vector3(0.48f, 0.47f, 0.45f) // gravel
        | _ -> Vector3(0.45f, 0.34f, 0.24f) // clay

    let columnColor (x: int) (z: int) =
        let config = state.Config
        let cx = min x (config.CellsX - 1)
        let cz = min z (config.CellsZ - 1)
        // Top cell's material and compaction drive the color: fresh loose
        // fill reads lighter than undisturbed bank.
        let mutable y = config.CellsY - 1

        while y > 0 && state.Occupancy.[state.Index(cx, y, cz)] = 0uy do
            y <- y - 1

        let index = state.Index(cx, y, cz)
        let baseColor = materialColor state.Material.[index]
        let loosen = 1.0f + (1.0f - float32 state.Compaction.[index] / 255.0f) * 0.25f
        baseColor * loosen

    let cornerHeight (x: int) (z: int) =
        let config = state.Config
        let mutable total = 0.0f
        let mutable count = 0

        for dz in -1 .. 0 do
            for dx in -1 .. 0 do
                let cx = x + dx
                let cz = z + dz

                if cx >= 0 && cx < config.CellsX && cz >= 0 && cz < config.CellsZ then
                    total <- total + state.Heights.[state.ColumnIndex(cx, cz)]
                    count <- count + 1

        total / float32 count

    let rebuildTile (tile: int) =
        let config = state.Config
        let tileX = tile % state.TilesX
        let tileZ = tile / state.TilesX
        let x0 = tileX * SoilConfig.TileSize
        let z0 = tileZ * SoilConfig.TileSize
        let size = config.CellSize
        let mutable i = 0

        for z in 0 .. tileVerts - 1 do
            for x in 0 .. tileVerts - 1 do
                let gx = x0 + x
                let gz = z0 + z
                let h = cornerHeight gx gz
                // Normal from central differences of corner heights.
                let hL = cornerHeight (max 0 (gx - 1)) gz
                let hR = cornerHeight (min config.CellsX (gx + 1)) gz
                let hD = cornerHeight gx (max 0 (gz - 1))
                let hU = cornerHeight gx (min config.CellsZ (gz + 1))
                let normal = Vector3.Normalize(Vector3(hL - hR, 2.0f * size, hD - hU))
                let color = columnColor gx gz
                vertexScratch.[i] <- float32 gx * size
                vertexScratch.[i + 1] <- h
                vertexScratch.[i + 2] <- float32 gz * size
                vertexScratch.[i + 3] <- normal.X
                vertexScratch.[i + 4] <- normal.Y
                vertexScratch.[i + 5] <- normal.Z
                vertexScratch.[i + 6] <- color.X
                vertexScratch.[i + 7] <- color.Y
                vertexScratch.[i + 8] <- color.Z
                i <- i + 9

        gl.BindVertexArray tileVaos.[tile]
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, tileVbos.[tile])
        // Orphaning upload: full glBufferData every time, never SubData.
        GlUtil.upload gl BufferTargetARB.ArrayBuffer vertexScratch vertexScratch.Length BufferUsageARB.DynamicDraw

    do
        gl.Enable EnableCap.DepthTest
        gl.Enable EnableCap.CullFace
        gl.ClearColor(0.63f, 0.66f, 0.70f, 1.0f)

        gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, sharedIbo)
        GlUtil.upload gl BufferTargetARB.ElementArrayBuffer tileIndices tileIndices.Length BufferUsageARB.StaticDraw

        for tile in 0 .. tileCount - 1 do
            tileVaos.[tile] <- gl.GenVertexArray()
            tileVbos.[tile] <- gl.GenBuffer()
            gl.BindVertexArray tileVaos.[tile]
            gl.BindBuffer(BufferTargetARB.ArrayBuffer, tileVbos.[tile])
            gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, sharedIbo)
            let stride = uint32 (vertexFloats * 4)
            gl.EnableVertexAttribArray 0u
            gl.VertexAttribPointer(0u, 3, VertexAttribPointerType.Float, false, stride, IntPtr.Zero.ToPointer())
            gl.EnableVertexAttribArray 1u
            gl.VertexAttribPointer(1u, 3, VertexAttribPointerType.Float, false, stride, IntPtr(12).ToPointer())
            gl.EnableVertexAttribArray 2u
            gl.VertexAttribPointer(2u, 3, VertexAttribPointerType.Float, false, stride, IntPtr(24).ToPointer())
            rebuildTile tile

        // Unit cube for machine parts.
        gl.BindVertexArray cubeVao
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, cubeVbo)
        GlUtil.upload gl BufferTargetARB.ArrayBuffer cubeVerts cubeVerts.Length BufferUsageARB.StaticDraw
        gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, cubeIbo)
        GlUtil.upload gl BufferTargetARB.ElementArrayBuffer cubeIndices cubeIndices.Length BufferUsageARB.StaticDraw
        gl.EnableVertexAttribArray 0u
        gl.VertexAttribPointer(0u, 3, VertexAttribPointerType.Float, false, 24u, IntPtr.Zero.ToPointer())
        gl.EnableVertexAttribArray 1u
        gl.VertexAttribPointer(1u, 3, VertexAttribPointerType.Float, false, 24u, IntPtr(12).ToPointer())

        // Clod mesh + instance buffer.
        gl.BindVertexArray clodVao
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, clodVbo)
        GlUtil.upload gl BufferTargetARB.ArrayBuffer icoVerts icoVerts.Length BufferUsageARB.StaticDraw
        gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, clodIbo)
        GlUtil.upload gl BufferTargetARB.ElementArrayBuffer icoIndices icoIndices.Length BufferUsageARB.StaticDraw
        gl.EnableVertexAttribArray 0u
        gl.VertexAttribPointer(0u, 3, VertexAttribPointerType.Float, false, 12u, IntPtr.Zero.ToPointer())
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, clodInstanceVbo)
        let instanceStride = uint32 (instanceFloats * 4)
        gl.EnableVertexAttribArray 1u
        gl.VertexAttribPointer(1u, 4, VertexAttribPointerType.Float, false, instanceStride, IntPtr.Zero.ToPointer())
        gl.VertexAttribDivisor(1u, 1u)
        gl.EnableVertexAttribArray 2u
        gl.VertexAttribPointer(2u, 3, VertexAttribPointerType.Float, false, instanceStride, IntPtr(16).ToPointer())
        gl.VertexAttribDivisor(2u, 1u)

    /// Rebuild up to `budget` dirty tiles (render meshes only).
    member _.RebuildDirtyTiles(budget: int) =
        let mutable rebuilt = 0
        let mutable tile = 0

        while rebuilt < budget && tile < state.DirtyRender.Length do
            if state.DirtyRender.[tile] then
                state.DirtyRender.[tile] <- false
                rebuildTile tile
                rebuilt <- rebuilt + 1

            tile <- tile + 1

    /// Draw the frame: terrain + clumps interpolated between two snapshots.
    member _.Draw
        (
            viewProjection: Matrix4x4,
            cameraPosition: Vector3,
            previous: RenderSnapshot,
            current: RenderSnapshot,
            alpha: float32,
            brush: Vector3 voption,
            brushRadius: float32
        ) =
        gl.Clear(uint32 (ClearBufferMask.ColorBufferBit ||| ClearBufferMask.DepthBufferBit))

        let sun = Vector3.Normalize(Vector3(0.4f, 0.8f, 0.3f))

        gl.UseProgram terrainProgram
        let mutable vp = viewProjection
        gl.UniformMatrix4(gl.GetUniformLocation(terrainProgram, "viewProjection"), 1u, false, &vp.M11)
        GlUtil.uniform3f gl terrainProgram "sunDirection" sun
        GlUtil.uniform3f gl terrainProgram "cameraPosition" cameraPosition

        for tile in 0 .. tileCount - 1 do
            gl.BindVertexArray tileVaos.[tile]
            gl.DrawElements(PrimitiveType.Triangles, uint32 tileIndexCount, DrawElementsType.UnsignedInt, IntPtr.Zero.ToPointer())

        // Machine parts: interpolate each body pose, then draw its visual
        // boxes with the solid shader.
        if current.MachinePartCount > 0 then
            gl.UseProgram solidProgram
            gl.UniformMatrix4(gl.GetUniformLocation(solidProgram, "viewProjection"), 1u, false, &vp.M11)
            GlUtil.uniform3f gl solidProgram "sunDirection" sun
            GlUtil.uniform3f gl solidProgram "cameraPosition" cameraPosition
            gl.BindVertexArray cubeVao

            for (part, offset, size, color) in machineBoxes do
                if part < current.MachinePartCount then
                    let hasPrevious = part < previous.MachinePartCount

                    let position =
                        if hasPrevious then
                            Vector3.Lerp(previous.MachinePositions.[part], current.MachinePositions.[part], alpha)
                        else
                            current.MachinePositions.[part]

                    let orientation =
                        if hasPrevious then
                            Quaternion.Slerp(
                                previous.MachineOrientations.[part],
                                current.MachineOrientations.[part],
                                alpha
                            )
                        else
                            current.MachineOrientations.[part]

                    let mutable model =
                        Matrix4x4.CreateScale size
                        * Matrix4x4.CreateTranslation offset
                        * Matrix4x4.CreateFromQuaternion orientation
                        * Matrix4x4.CreateTranslation position

                    gl.UniformMatrix4(gl.GetUniformLocation(solidProgram, "model"), 1u, false, &model.M11)
                    GlUtil.uniform3f gl solidProgram "solidColor" color

                    gl.DrawElements(
                        PrimitiveType.Triangles,
                        uint32 cubeIndices.Length,
                        DrawElementsType.UnsignedInt,
                        IntPtr.Zero.ToPointer()
                    )

        // Clumps: interpolate by matching handles (swap-removes reorder the
        // pool, so index-matching would smear positions across clumps).
        let mutable instances = 0

        for i in 0 .. current.Count - 1 do
            let handle = current.Handles.[i]
            let mutable px = current.X.[i]
            let mutable py = current.Y.[i]
            let mutable pz = current.Z.[i]

            // Linear scan over previous handles: N ≤ 1500, fine.
            let mutable j = 0

            while j < previous.Count do
                if previous.Handles.[j] = handle then
                    px <- previous.X.[j] + (px - previous.X.[j]) * alpha
                    py <- previous.Y.[j] + (py - previous.Y.[j]) * alpha
                    pz <- previous.Z.[j] + (pz - previous.Z.[j]) * alpha
                    j <- previous.Count
                else
                    j <- j + 1

            let color = materialColor current.Materials.[i]
            let baseIndex = instances * instanceFloats
            instanceScratch.[baseIndex] <- px
            instanceScratch.[baseIndex + 1] <- py
            instanceScratch.[baseIndex + 2] <- pz
            instanceScratch.[baseIndex + 3] <- current.Radius.[i]
            instanceScratch.[baseIndex + 4] <- color.X
            instanceScratch.[baseIndex + 5] <- color.Y
            instanceScratch.[baseIndex + 6] <- color.Z
            instances <- instances + 1

        // Brush indicator rides along as one extra instance.
        match brush with
        | ValueSome position ->
            let baseIndex = instances * instanceFloats
            instanceScratch.[baseIndex] <- position.X
            instanceScratch.[baseIndex + 1] <- position.Y
            instanceScratch.[baseIndex + 2] <- position.Z
            instanceScratch.[baseIndex + 3] <- brushRadius
            instanceScratch.[baseIndex + 4] <- 0.9f
            instanceScratch.[baseIndex + 5] <- 0.55f
            instanceScratch.[baseIndex + 6] <- 0.2f
            instances <- instances + 1
        | ValueNone -> ()

        if instances > 0 then
            gl.UseProgram clodProgram
            gl.UniformMatrix4(gl.GetUniformLocation(clodProgram, "viewProjection"), 1u, false, &vp.M11)
            GlUtil.uniform3f gl clodProgram "sunDirection" sun
            GlUtil.uniform3f gl clodProgram "cameraPosition" cameraPosition
            gl.BindVertexArray clodVao
            gl.BindBuffer(BufferTargetARB.ArrayBuffer, clodInstanceVbo)

            GlUtil.upload
                gl
                BufferTargetARB.ArrayBuffer
                instanceScratch
                (instances * instanceFloats)
                BufferUsageARB.StreamDraw

            gl.DrawElementsInstanced(
                PrimitiveType.Triangles,
                uint32 icoIndices.Length,
                DrawElementsType.UnsignedInt,
                IntPtr.Zero.ToPointer(),
                uint32 instances
            )

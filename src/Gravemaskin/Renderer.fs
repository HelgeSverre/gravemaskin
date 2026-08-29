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
    // Render mesh at 2× the soil-column resolution: heights bilinearly
    // smoothed between column corners plus a whisper of value noise, colors
    // blended across material boundaries. Collision stays at column
    // resolution (physics is unchanged); this is purely finer visuals.
    [<Literal>]
    let SubRes = 4

    let tileVerts = SoilConfig.TileSize * SubRes + 1
    let cornerVerts = SoilConfig.TileSize + 1

    let terrainProgram = GlUtil.program gl Shaders.terrainVertex Shaders.terrainFragment
    let clodProgram = GlUtil.program gl Shaders.clodVertex Shaders.clodFragment
    let solidProgram = GlUtil.program gl Shaders.solidVertex Shaders.clodFragment
    let grainProgram = GlUtil.program gl Shaders.grainVertex Shaders.clodFragment

    let skyProgram =
        GlUtil.program
            gl
            """#version 410 core
out vec2 vUv;
void main()
{
    // Fullscreen triangle from gl_VertexID — no buffers.
    vec2 corners[3] = vec2[3](vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
    vUv = corners[gl_VertexID] * 0.5 + 0.5;
    gl_Position = vec4(corners[gl_VertexID], 0.99999, 1.0);
}
"""
            """#version 410 core
in vec2 vUv;
uniform vec3 sunDirection;
out vec4 fragColor;
void main()
{
    // Overcast-industrial gradient with a smeared sun glow.
    vec3 horizon = vec3(0.70, 0.72, 0.74);
    vec3 zenith = vec3(0.45, 0.52, 0.62);
    vec3 sky = mix(horizon, zenith, pow(clamp(vUv.y, 0.0, 1.0), 0.8));
    float glow = pow(max(0.0, 1.0 - distance(vUv, vec2(0.68, 0.75)) * 1.6), 3.0);
    fragColor = vec4(sky + vec3(0.35, 0.32, 0.25) * glow, 1.0);
}
"""

    let skyVao = gl.GenVertexArray()

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

    /// Visual boxes per machine body part:
    /// (body index, local offset, size, color, local Z rotation).
    /// The bucket group is rotated +45°: mouth down-and-back at rest, up at
    /// carry — the real profile. (Collision stays the proven axis-aligned
    /// container; this is presentation.)
    let bucketTilt = 0.785f

    /// Generic (U17/Cat-style) model.
    let machineBoxes =
        [| 0, Vector3(0.0f, -0.1f, -0.55f), Vector3(1.8f, 0.35f, 0.35f), Vector3(0.12f, 0.12f, 0.13f), 0.0f // track L
           0, Vector3(0.0f, -0.1f, 0.55f), Vector3(1.8f, 0.35f, 0.35f), Vector3(0.12f, 0.12f, 0.13f), 0.0f // track R
           0, Vector3(-0.85f, -0.1f, -0.55f), Vector3(0.22f, 0.42f, 0.42f), Vector3(0.09f, 0.09f, 0.1f), 0.0f // idler L
           0, Vector3(-0.85f, -0.1f, 0.55f), Vector3(0.22f, 0.42f, 0.42f), Vector3(0.09f, 0.09f, 0.1f), 0.0f // idler R
           0, Vector3(0.85f, -0.1f, -0.55f), Vector3(0.22f, 0.42f, 0.42f), Vector3(0.09f, 0.09f, 0.1f), 0.0f // sprocket L
           0, Vector3(0.85f, -0.1f, 0.55f), Vector3(0.22f, 0.42f, 0.42f), Vector3(0.09f, 0.09f, 0.1f), 0.0f // sprocket R
           0, Vector3(0.0f, 0.05f, 0.0f), Vector3(1.5f, 0.3f, 1.0f), Vector3(0.25f, 0.25f, 0.27f), 0.0f // undercarriage
           1, Vector3(0.1f, 0.0f, 0.0f), Vector3(1.05f, 0.7f, 0.95f), Vector3(0.85f, 0.45f, 0.08f), 0.0f // cab body
           1, Vector3(0.28f, 0.14f, 0.0f), Vector3(0.55f, 0.44f, 0.97f), Vector3(0.16f, 0.2f, 0.24f), 0.0f // glass
           1, Vector3(-0.35f, 0.42f, 0.28f), Vector3(0.1f, 0.22f, 0.1f), Vector3(0.1f, 0.1f, 0.11f), 0.0f // exhaust
           1, Vector3(-0.28f, 0.38f, -0.2f), Vector3(0.45f, 0.1f, 0.4f), Vector3(0.78f, 0.4f, 0.07f), 0.0f // engine hood
           1, Vector3(-0.75f, -0.1f, 0.0f), Vector3(0.45f, 0.5f, 0.9f), Vector3(0.3f, 0.3f, 0.32f), 0.0f // counterweight
           // Gooseneck boom: two angled segments meeting at a raised knee —
           // the single silhouette feature that makes an excavator arm.
           2, Vector3(-0.5f, 0.08f, 0.0f), Vector3(1.12f, 0.24f, 0.16f), Vector3(0.85f, 0.45f, 0.08f), 0.32f // boom lower
           2, Vector3(0.48f, 0.13f, 0.0f), Vector3(1.15f, 0.19f, 0.15f), Vector3(0.85f, 0.45f, 0.08f), -0.24f // boom upper
           2, Vector3(-0.05f, 0.26f, 0.0f), Vector3(0.34f, 0.16f, 0.17f), Vector3(0.75f, 0.39f, 0.07f), 0.04f // boom knee
           3, Vector3(-0.2f, 0.0f, 0.0f), Vector3(0.75f, 0.16f, 0.13f), Vector3(0.85f, 0.45f, 0.08f), 0.0f // stick root
           3, Vector3(0.33f, 0.0f, 0.0f), Vector3(0.5f, 0.12f, 0.11f), Vector3(0.82f, 0.43f, 0.08f), -0.06f // stick taper
           3, Vector3(-0.25f, 0.09f, 0.0f), Vector3(0.5f, 0.07f, 0.14f), Vector3(0.75f, 0.39f, 0.07f), 0.0f // stick flange
           2, Vector3(-0.95f, 0.0f, 0.0f), Vector3(0.1f, 0.24f, 0.2f), Vector3(0.3f, 0.3f, 0.32f), 0.0f // boom pivot boss
           3, Vector3(-0.55f, 0.0f, 0.0f), Vector3(0.08f, 0.19f, 0.16f), Vector3(0.3f, 0.3f, 0.32f), 0.0f // stick pivot boss
           4, Vector3(-0.28f, 0.0f, 0.0f), Vector3(0.09f, 0.16f, 0.16f), Vector3(0.3f, 0.3f, 0.32f), 0.0f // bucket pivot boss
           4, Vector3(0.3f, -0.2f, -0.2f), Vector3(0.14f, 0.06f, 0.05f), Vector3(0.34f, 0.32f, 0.3f), bucketTilt // tooth
           4, Vector3(0.3f, -0.2f, -0.067f), Vector3(0.14f, 0.06f, 0.05f), Vector3(0.34f, 0.32f, 0.3f), bucketTilt // tooth
           4, Vector3(0.3f, -0.2f, 0.067f), Vector3(0.14f, 0.06f, 0.05f), Vector3(0.34f, 0.32f, 0.3f), bucketTilt // tooth
           4, Vector3(0.3f, -0.2f, 0.2f), Vector3(0.14f, 0.06f, 0.05f), Vector3(0.34f, 0.32f, 0.3f), bucketTilt // tooth
           4, Vector3(0.0f, -0.17f, 0.0f), Vector3(0.5f, 0.06f, 0.6f), Vector3(0.2f, 0.2f, 0.22f), bucketTilt // bucket floor
           4, Vector3(0.22f, 0.0f, 0.0f), Vector3(0.06f, 0.4f, 0.6f), Vector3(0.2f, 0.2f, 0.22f), bucketTilt // bucket back
           4, Vector3(0.075f, 0.17f, 0.0f), Vector3(0.35f, 0.06f, 0.6f), Vector3(0.22f, 0.22f, 0.24f), bucketTilt // bucket shell top
           4, Vector3(0.0f, 0.0f, -0.27f), Vector3(0.5f, 0.4f, 0.06f), Vector3(0.17f, 0.17f, 0.19f), bucketTilt // bucket side
           4, Vector3(0.0f, 0.0f, 0.27f), Vector3(0.5f, 0.4f, 0.06f), Vector3(0.17f, 0.17f, 0.19f), bucketTilt |] // bucket side


    /// Takeuchi TB216, modeled against docs/takeuchi-tb216/images: white
    /// body, red skirt band and corner accents, tall glazed cab on the
    /// left, low engine cover rear-right, rounded white counterweight,
    /// WHITE gooseneck boom on a front swing post, dozer blade, dark steel
    /// bucket. Iterate with `just preview`.
    let tb216Boxes =
        let white = Vector3(0.88f, 0.87f, 0.83f)
        let whiteDim = Vector3(0.8f, 0.79f, 0.76f)
        let red = Vector3(0.68f, 0.07f, 0.09f)
        let glass = Vector3(0.07f, 0.09f, 0.12f)
        let dark = Vector3(0.1f, 0.1f, 0.11f)
        let steel = Vector3(0.42f, 0.42f, 0.45f)
        let grey = Vector3(0.3f, 0.3f, 0.32f)

        [| // undercarriage + tracks
           0, Vector3(0.0f, -0.1f, -0.55f), Vector3(1.75f, 0.38f, 0.34f), dark, 0.0f
           0, Vector3(0.0f, -0.1f, 0.55f), Vector3(1.75f, 0.38f, 0.34f), dark, 0.0f
           0, Vector3(-0.82f, -0.1f, -0.55f), Vector3(0.26f, 0.44f, 0.36f), dark * 0.8f, 0.0f
           0, Vector3(-0.82f, -0.1f, 0.55f), Vector3(0.26f, 0.44f, 0.36f), dark * 0.8f, 0.0f
           0, Vector3(0.82f, -0.1f, -0.55f), Vector3(0.26f, 0.44f, 0.36f), dark * 0.8f, 0.0f
           0, Vector3(0.82f, -0.1f, 0.55f), Vector3(0.26f, 0.44f, 0.36f), dark * 0.8f, 0.0f
           0, Vector3(0.0f, 0.06f, 0.0f), Vector3(1.25f, 0.3f, 0.95f), grey, 0.0f
           // dozer blade on the arm side
           0, Vector3(0.95f, -0.08f, -0.3f), Vector3(0.5f, 0.09f, 0.09f), grey, 0.1f
           0, Vector3(0.95f, -0.08f, 0.3f), Vector3(0.5f, 0.09f, 0.09f), grey, 0.1f
           0, Vector3(1.22f, -0.06f, 0.0f), Vector3(0.09f, 0.34f, 1.3f), steel * 0.75f, -0.08f
           // house: red skirt, white bodywork
           1, Vector3(0.0f, -0.14f, 0.0f), Vector3(1.34f, 0.2f, 1.04f), red, 0.0f
           1, Vector3(0.55f, 0.04f, 0.3f), Vector3(0.16f, 0.26f, 0.34f), red, 0.0f
           // cab (left side), glass front/side, white roof
           1, Vector3(0.14f, 0.34f, -0.18f), Vector3(0.88f, 1.08f, 0.62f), white, 0.0f
           1, Vector3(0.56f, 0.38f, -0.18f), Vector3(0.05f, 0.85f, 0.54f), glass, 0.0f
           1, Vector3(0.14f, 0.42f, -0.47f), Vector3(0.76f, 0.78f, 0.05f), glass, 0.0f
           1, Vector3(0.14f, 0.9f, -0.18f), Vector3(0.98f, 0.12f, 0.7f), white, 0.0f
           // engine cover right-rear, vent, counterweight
           1, Vector3(-0.18f, 0.08f, 0.3f), Vector3(0.85f, 0.66f, 0.42f), white, 0.0f
           1, Vector3(-0.18f, 0.14f, 0.5f), Vector3(0.55f, 0.34f, 0.03f), grey, 0.0f
           1, Vector3(-0.58f, 0.04f, 0.0f), Vector3(0.38f, 0.75f, 0.96f), white, 0.0f
           1, Vector3(-0.73f, 0.16f, 0.0f), Vector3(0.2f, 0.5f, 0.8f), whiteDim, 0.0f
           1, Vector3(-0.64f, -0.24f, -0.42f), Vector3(0.26f, 0.26f, 0.15f), red, 0.0f
           1, Vector3(-0.64f, -0.24f, 0.42f), Vector3(0.26f, 0.26f, 0.15f), red, 0.0f
           // swing post
           1, Vector3(0.56f, -0.08f, 0.0f), Vector3(0.3f, 0.36f, 0.28f), whiteDim, 0.0f
           // boom: WHITE gooseneck
           2, Vector3(-0.5f, 0.08f, 0.0f), Vector3(1.12f, 0.26f, 0.18f), white, 0.32f
           2, Vector3(0.48f, 0.13f, 0.0f), Vector3(1.15f, 0.2f, 0.16f), white, -0.24f
           2, Vector3(-0.05f, 0.28f, 0.0f), Vector3(0.36f, 0.18f, 0.19f), white, 0.04f
           2, Vector3(-0.95f, 0.0f, 0.0f), Vector3(0.12f, 0.26f, 0.22f), steel, 0.0f
           // stick: white, tapered
           3, Vector3(-0.2f, 0.0f, 0.0f), Vector3(0.75f, 0.17f, 0.14f), white, 0.0f
           3, Vector3(0.33f, 0.0f, 0.0f), Vector3(0.5f, 0.13f, 0.12f), whiteDim, -0.06f
           3, Vector3(-0.55f, 0.0f, 0.0f), Vector3(0.1f, 0.2f, 0.17f), steel, 0.0f
           // bucket: dark steel, existing tilt convention
           4, Vector3(-0.28f, 0.0f, 0.0f), Vector3(0.09f, 0.16f, 0.16f), steel, 0.0f
           4, Vector3(0.3f, -0.2f, -0.2f), Vector3(0.14f, 0.06f, 0.05f), steel * 0.8f, bucketTilt
           4, Vector3(0.3f, -0.2f, -0.067f), Vector3(0.14f, 0.06f, 0.05f), steel * 0.8f, bucketTilt
           4, Vector3(0.3f, -0.2f, 0.067f), Vector3(0.14f, 0.06f, 0.05f), steel * 0.8f, bucketTilt
           4, Vector3(0.3f, -0.2f, 0.2f), Vector3(0.14f, 0.06f, 0.05f), steel * 0.8f, bucketTilt
           4, Vector3(0.0f, -0.17f, 0.0f), Vector3(0.5f, 0.06f, 0.6f), grey * 0.8f, bucketTilt
           4, Vector3(0.22f, 0.0f, 0.0f), Vector3(0.06f, 0.4f, 0.6f), grey * 0.8f, bucketTilt
           4, Vector3(0.075f, 0.17f, 0.0f), Vector3(0.35f, 0.06f, 0.6f), grey * 0.9f, bucketTilt
           4, Vector3(0.0f, 0.0f, -0.27f), Vector3(0.5f, 0.4f, 0.06f), grey * 0.7f, bucketTilt
           4, Vector3(0.0f, 0.0f, 0.27f), Vector3(0.5f, 0.4f, 0.06f), grey * 0.7f, bucketTilt |]

    let boxesFor (name: string) =
        if name = "TB216" then tb216Boxes else machineBoxes

    // Shared index buffer: same grid topology for every tile.
    let tileIndexCount = SoilConfig.TileSize * SubRes * SoilConfig.TileSize * SubRes * 6

    let tileIndices =
        let indices = Array.zeroCreate<uint32> tileIndexCount
        let mutable i = 0

        for z in 0 .. SoilConfig.TileSize * SubRes - 1 do
            for x in 0 .. SoilConfig.TileSize * SubRes - 1 do
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

    // Unit icosphere (icosahedron subdivided once → 80 faces) for rocks,
    // plus the raw 20-face icosahedron for grains (they're centimeters big —
    // 20 triangles is plenty and 60k of them is already 1.2M tris).
    let icoVerts, icoIndices, grainVerts, grainIndices =
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

        // One subdivision pass: each face → 4, midpoints re-projected to
        // the unit sphere. 20 → 80 faces.
        let vertices = ResizeArray<Vector3>(raw)
        let midpointCache = System.Collections.Generic.Dictionary<struct (int * int), int>()

        let midpoint a b =
            let key = struct (min a b, max a b)

            match midpointCache.TryGetValue key with
            | true, index -> index
            | _ ->
                let index = vertices.Count
                vertices.Add(Vector3.Normalize((vertices.[a] + vertices.[b]) * 0.5f))
                midpointCache.[key] <- index
                index

        let subdivided = ResizeArray<uint32>()

        for face in 0 .. faces.Length / 3 - 1 do
            let a = int faces.[face * 3]
            let b = int faces.[face * 3 + 1]
            let c = int faces.[face * 3 + 2]
            let ab = midpoint a b
            let bc = midpoint b c
            let ca = midpoint c a

            for index in [| a; ab; ca; b; bc; ab; c; ca; bc; ab; bc; ca |] do
                subdivided.Add(uint32 index)

        let flat = Array.zeroCreate<float32> (vertices.Count * 3)

        for i in 0 .. vertices.Count - 1 do
            flat.[i * 3] <- vertices.[i].X
            flat.[i * 3 + 1] <- vertices.[i].Y
            flat.[i * 3 + 2] <- vertices.[i].Z

        let baseFlat = Array.zeroCreate<float32> (raw.Length * 3)

        for i in 0 .. raw.Length - 1 do
            baseFlat.[i * 3] <- raw.[i].X
            baseFlat.[i * 3 + 1] <- raw.[i].Y
            baseFlat.[i * 3 + 2] <- raw.[i].Z

        flat, subdivided.ToArray(), baseFlat, faces

    let clodVao = gl.GenVertexArray()
    let clodVbo = gl.GenBuffer()
    let clodIbo = gl.GenBuffer()
    let clodInstanceVbo = gl.GenBuffer()
    // xyz + radius + rgb per instance.
    let instanceFloats = 7
    let instanceScratch = Array.zeroCreate<float32> ((Clumps.MaxClumps + 65) * instanceFloats)

    // Grain layer: its own VAO over the 20-tri mesh; pool grains + clump
    // clusters share one big instance buffer.
    let grainVao = gl.GenVertexArray()
    let grainVbo = gl.GenBuffer()
    let grainIbo = gl.GenBuffer()
    let grainInstanceVbo = gl.GenBuffer()
    // pos3+size, color3, velocity3.
    let grainInstanceFloats = 10
    let grainScratch = Array.zeroCreate<float32> (200_000 * grainInstanceFloats)

    /// Deterministic per-(clump,grain) hash → [0,1).
    let hash01 (seed: int) =
        let mutable h = uint seed * 0x9E3779B9u
        h <- (h ^^^ (h >>> 16)) * 0x7FEB352Du
        h <- (h ^^^ (h >>> 15)) * 0x846CA68Bu
        float32 (h ^^^ (h >>> 16)) / 4294967296.0f

    let materialColor (mat: byte) =
        match int mat with
        | 0 -> Vector3(0.36f, 0.27f, 0.18f) // topsoil
        | 1 -> Vector3(0.72f, 0.62f, 0.44f) // dry sand
        | 2 -> Vector3(0.52f, 0.44f, 0.32f) // wet sand
        | 3 -> Vector3(0.48f, 0.47f, 0.45f) // gravel
        | 4 -> Vector3(0.45f, 0.34f, 0.24f) // clay
        | _ -> Vector3(0.28f, 0.42f, 0.16f) // grass

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
        // Wet ground reads darker — the classic soaked-soil cue.
        let darken = 1.0f - 0.38f * float32 state.Moisture.[index] / 255.0f
        // Steep faces expose raw substrate: blend toward a rocky tone by
        // slope so cut walls and scarps separate from the flats.
        let hL = state.Heights.[state.ColumnIndex(max 0 (cx - 1), cz)]
        let hR = state.Heights.[state.ColumnIndex(min (config.CellsX - 1) (cx + 1), cz)]
        let hD = state.Heights.[state.ColumnIndex(cx, max 0 (cz - 1))]
        let hU = state.Heights.[state.ColumnIndex(cx, min (config.CellsZ - 1) (cz + 1))]
        let slope = (abs (hL - hR) + abs (hD - hU)) / (2.0f * config.CellSize)
        let rockiness = Math.Clamp((slope - 0.8f) * 0.5f, 0.0f, 0.55f)
        let rockTone = Vector3(0.42f, 0.38f, 0.33f)
        Vector3.Lerp(baseColor * loosen * darken, rockTone, rockiness)

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

    // Per-tile scratch for the corner-resolution samples the subdivided
    // grid interpolates from (avoids re-scanning columns per subvertex).
    let cornerHeightScratch = Array.zeroCreate<float32> (cornerVerts * cornerVerts)
    let cornerColorScratch = Array.zeroCreate<Vector3> (cornerVerts * cornerVerts)

    let rebuildTile (tile: int) =
        let config = state.Config
        let tileX = tile % state.TilesX
        let tileZ = tile / state.TilesX
        let x0 = tileX * SoilConfig.TileSize
        let z0 = tileZ * SoilConfig.TileSize
        let size = config.CellSize

        for cz in 0 .. cornerVerts - 1 do
            for cx in 0 .. cornerVerts - 1 do
                cornerHeightScratch.[cz * cornerVerts + cx] <- cornerHeight (x0 + cx) (z0 + cz)
                cornerColorScratch.[cz * cornerVerts + cx] <- columnColor (x0 + cx) (z0 + cz)

        // Bilinear samplers over the corner grid, in sub-vertex coordinates.
        let inline heightAt (sx: float32) (sz: float32) =
            let fx = Math.Clamp(sx / float32 SubRes, 0.0f, float32 (cornerVerts - 1) - 0.001f)
            let fz = Math.Clamp(sz / float32 SubRes, 0.0f, float32 (cornerVerts - 1) - 0.001f)
            let ix = int fx
            let iz = int fz
            let tx = fx - float32 ix
            let tz = fz - float32 iz
            let h00 = cornerHeightScratch.[iz * cornerVerts + ix]
            let h10 = cornerHeightScratch.[iz * cornerVerts + ix + 1]
            let h01 = cornerHeightScratch.[(iz + 1) * cornerVerts + ix]
            let h11 = cornerHeightScratch.[(iz + 1) * cornerVerts + ix + 1]
            let lo = min (min h00 h10) (min h01 h11)
            let hi = max (max h00 h10) (max h01 h11)

            if hi - lo > 0.32f then
                // Edge-aware: a dig cut is a CLIFF — blending across it
                // melts trenches into dimples. Snap to the nearest corner.
                let cx = if tx < 0.5f then 0 else 1
                let cz = if tz < 0.5f then 0 else 1
                cornerHeightScratch.[(iz + cz) * cornerVerts + ix + cx]
            else
                let bottom = h00 + (h10 - h00) * tx
                let top = h01 + (h11 - h01) * tx
                bottom + (top - bottom) * tz

        let mutable i = 0

        for z in 0 .. tileVerts - 1 do
            for x in 0 .. tileVerts - 1 do
                let worldX = (float32 (x0 * SubRes + x)) * size / float32 SubRes
                let worldZ = (float32 (z0 * SubRes + z)) * size / float32 SubRes

                // Micro-relief: a whisper of value noise so smoothed dirt
                // doesn't read as vinyl. Deterministic in world space, so
                // tile borders agree.
                // Kept small: the grain layer rests against the ANALYTIC
                // surface, so render-only relief this large made settled
                // grains read as floaters.
                let micro =
                    (Noise.value2 1913 (Vector2(worldX * 2.1f, worldZ * 2.1f)) - 0.5f) * 0.02f
                    + (Noise.value2 7477 (Vector2(worldX * 9.3f, worldZ * 9.3f)) - 0.5f) * 0.008f

                let h = heightAt (float32 x) (float32 z) + micro
                let step = 1.0f
                let hL = heightAt (float32 x - step) (float32 z)
                let hR = heightAt (float32 x + step) (float32 z)
                let hD = heightAt (float32 x) (float32 z - step)
                let hU = heightAt (float32 x) (float32 z + step)
                let normal = Vector3.Normalize(Vector3(hL - hR, 2.0f * size / float32 SubRes, hD - hU))

                // Concavity-darkened color (cheap crevice AO).
                let curvature = (hL + hR + hD + hU) * 0.25f - h
                let ao = Math.Clamp(1.0f - curvature * 2.2f, 0.72f, 1.08f)

                let fx = Math.Clamp(float32 x / float32 SubRes, 0.0f, float32 (cornerVerts - 1) - 0.001f)
                let fz = Math.Clamp(float32 z / float32 SubRes, 0.0f, float32 (cornerVerts - 1) - 0.001f)
                let ix = int fx
                let iz = int fz
                let tx = fx - float32 ix
                let tz = fz - float32 iz
                let c00 = cornerColorScratch.[iz * cornerVerts + ix]
                let c10 = cornerColorScratch.[iz * cornerVerts + ix + 1]
                let c01 = cornerColorScratch.[(iz + 1) * cornerVerts + ix]
                let c11 = cornerColorScratch.[(iz + 1) * cornerVerts + ix + 1]
                let color = Vector3.Lerp(Vector3.Lerp(c00, c10, tx), Vector3.Lerp(c01, c11, tx), tz) * ao

                vertexScratch.[i] <- worldX
                vertexScratch.[i + 1] <- h
                vertexScratch.[i + 2] <- worldZ
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

        // Grain mesh + instance buffer.
        gl.BindVertexArray grainVao
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, grainVbo)
        GlUtil.upload gl BufferTargetARB.ArrayBuffer grainVerts grainVerts.Length BufferUsageARB.StaticDraw
        gl.BindBuffer(BufferTargetARB.ElementArrayBuffer, grainIbo)
        GlUtil.upload gl BufferTargetARB.ElementArrayBuffer grainIndices grainIndices.Length BufferUsageARB.StaticDraw
        gl.EnableVertexAttribArray 0u
        gl.VertexAttribPointer(0u, 3, VertexAttribPointerType.Float, false, 12u, IntPtr.Zero.ToPointer())
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, grainInstanceVbo)

        do
            let instanceStride = uint32 (grainInstanceFloats * 4)
            gl.EnableVertexAttribArray 1u
            gl.VertexAttribPointer(1u, 4, VertexAttribPointerType.Float, false, instanceStride, IntPtr.Zero.ToPointer())
            gl.VertexAttribDivisor(1u, 1u)
            gl.EnableVertexAttribArray 2u
            gl.VertexAttribPointer(2u, 3, VertexAttribPointerType.Float, false, instanceStride, IntPtr(16).ToPointer())
            gl.VertexAttribDivisor(2u, 1u)
            gl.EnableVertexAttribArray 3u
            gl.VertexAttribPointer(3u, 3, VertexAttribPointerType.Float, false, instanceStride, IntPtr(28).ToPointer())
            gl.VertexAttribDivisor(3u, 1u)

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

    /// Hydraulic cylinder visuals: (parent part, local anchor) →
    /// (child part, local anchor). Drawn as barrel+rod beams between the
    /// interpolated part poses, so the cylinders visibly stroke.
    let cylinderRuns =
        [| 1, Vector3(0.5f, -0.2f, -0.3f), 2, Vector3(-0.35f, 0.12f, -0.09f) // boom cyl L
           1, Vector3(0.5f, -0.2f, 0.3f), 2, Vector3(-0.35f, 0.12f, 0.09f) // boom cyl R
           2, Vector3(0.25f, 0.16f, 0.0f), 3, Vector3(-0.72f, 0.1f, 0.0f) // stick cyl
           3, Vector3(-0.35f, 0.14f, 0.0f), 4, Vector3(0.05f, 0.24f, 0.0f) |] // bucket cyl

    /// Model matrix stretching a unit cube into a beam from a to b.
    let beamMatrix (a: Vector3) (b: Vector3) (thickness: float32) =
        let axis = b - a
        let length = axis.Length()

        if length < 1e-4f then
            Matrix4x4.CreateScale 0.0f
        else
            let direction = axis / length

            let reference =
                if MathF.Abs direction.Y > 0.9f then
                    Vector3.UnitX
                else
                    Vector3.UnitY

            let side = Vector3.Normalize(Vector3.Cross(direction, reference))
            let up = Vector3.Cross(side, direction)
            let mutable m = Matrix4x4.Identity
            m.M11 <- direction.X * length
            m.M12 <- direction.Y * length
            m.M13 <- direction.Z * length
            m.M21 <- up.X * thickness
            m.M22 <- up.Y * thickness
            m.M23 <- up.Z * thickness
            m.M31 <- side.X * thickness
            m.M32 <- side.Y * thickness
            m.M33 <- side.Z * thickness
            let mid = (a + b) * 0.5f
            m.M41 <- mid.X
            m.M42 <- mid.Y
            m.M43 <- mid.Z
            m

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
            grains: GrainPool,
            brush: Vector3 voption,
            brushRadius: float32
        ) =
        gl.Clear(uint32 (ClearBufferMask.ColorBufferBit ||| ClearBufferMask.DepthBufferBit))

        let sun = Vector3.Normalize(Vector3(0.4f, 0.8f, 0.3f))

        // Sky: fullscreen gradient at max depth, no writes.
        gl.DepthMask false
        gl.UseProgram skyProgram
        GlUtil.uniform3f gl skyProgram "sunDirection" sun
        gl.BindVertexArray skyVao
        gl.DrawArrays(PrimitiveType.Triangles, 0, 3u)
        gl.DepthMask true

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

            for (part, offset, size, color, localRotation) in boxesFor current.MachineName do
                if part < current.MachinePartCount then
                    let offset = offset * current.MachineScale
                    let size = size * current.MachineScale
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
                        * Matrix4x4.CreateRotationZ localRotation
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

        // Hydraulic cylinders: barrel (thick, parent side) + rod (thin).
        if current.MachinePartCount > 0 then
            gl.UseProgram solidProgram
            gl.BindVertexArray cubeVao

            let partPose (part: int) =
                let hasPrevious = part < previous.MachinePartCount

                let position =
                    if hasPrevious then
                        Vector3.Lerp(previous.MachinePositions.[part], current.MachinePositions.[part], alpha)
                    else
                        current.MachinePositions.[part]

                let orientation =
                    if hasPrevious then
                        Quaternion.Slerp(previous.MachineOrientations.[part], current.MachineOrientations.[part], alpha)
                    else
                        current.MachineOrientations.[part]

                position, orientation

            let drawBeam (a: Vector3) (b: Vector3) (thickness: float32) (color: Vector3) =
                let mutable model = beamMatrix a b thickness
                gl.UniformMatrix4(gl.GetUniformLocation(solidProgram, "model"), 1u, false, &model.M11)
                GlUtil.uniform3f gl solidProgram "solidColor" color

                gl.DrawElements(
                    PrimitiveType.Triangles,
                    uint32 cubeIndices.Length,
                    DrawElementsType.UnsignedInt,
                    IntPtr.Zero.ToPointer()
                )

            for (parentPart, parentAnchor, childPart, childAnchor) in cylinderRuns do
                if childPart < current.MachinePartCount then
                    let parentPosition, parentOrientation = partPose parentPart
                    let childPosition, childOrientation = partPose childPart
                    let scaleFactor = current.MachineScale

                    let a =
                        parentPosition + Vector3.Transform(parentAnchor * scaleFactor, parentOrientation)

                    let b =
                        childPosition + Vector3.Transform(childAnchor * scaleFactor, childOrientation)

                    // Barrel covers the parent 55%; the rod runs the rest.
                    let barrelEnd = Vector3.Lerp(a, b, 0.55f)
                    drawBeam a barrelEnd (0.09f * scaleFactor) (Vector3(0.16f, 0.16f, 0.18f))
                    drawBeam barrelEnd b (0.045f * scaleFactor) (Vector3(0.75f, 0.76f, 0.78f))

        // Clumps are no longer drawn as balls: each renders as a cluster of
        // grains (below). Their interpolated positions seed the clusters.
        let mutable instances = 0

        // Rocks: same instancing, boulder grey.
        for i in 0 .. current.RockCount - 1 do
            let baseIndex = instances * instanceFloats
            instanceScratch.[baseIndex] <- current.RockPositions.[i].X
            instanceScratch.[baseIndex + 1] <- current.RockPositions.[i].Y
            instanceScratch.[baseIndex + 2] <- current.RockPositions.[i].Z
            instanceScratch.[baseIndex + 3] <- current.RockRadii.[i]
            instanceScratch.[baseIndex + 4] <- 0.42f
            instanceScratch.[baseIndex + 5] <- 0.42f
            instanceScratch.[baseIndex + 6] <- 0.44f
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

        // ---- the grain layer: pool grains + clump clusters ----
        let mutable grainInstances = 0
        let grainCapacity = grainScratch.Length / grainInstanceFloats

        let inline pushGrainMoving
            (x: float32)
            (y: float32)
            (z: float32)
            (size: float32)
            (color: Vector3)
            (vx: float32)
            (vy: float32)
            (vz: float32)
            =
            if grainInstances < grainCapacity then
                let baseIndex = grainInstances * grainInstanceFloats
                grainScratch.[baseIndex] <- x
                grainScratch.[baseIndex + 1] <- y
                grainScratch.[baseIndex + 2] <- z
                grainScratch.[baseIndex + 3] <- size
                grainScratch.[baseIndex + 4] <- color.X
                grainScratch.[baseIndex + 5] <- color.Y
                grainScratch.[baseIndex + 6] <- color.Z
                grainScratch.[baseIndex + 7] <- vx
                grainScratch.[baseIndex + 8] <- vy
                grainScratch.[baseIndex + 9] <- vz
                grainInstances <- grainInstances + 1

        let inline pushGrain (x: float32) (y: float32) (z: float32) (size: float32) (color: Vector3) =
            pushGrainMoving x y z size color 0.0f 0.0f 0.0f

        // Falling/resting grains from the pool, wet ones darker, each with a
        // stable per-slot tint so a stream shimmers instead of banding.
        for i in 0 .. grains.Count - 1 do
            let baseColor = materialColor grains.Materials.[i]
            let darken = 1.0f - 0.32f * float32 grains.Wetness.[i] / 255.0f
            // Lighter than before: instanced spheres shade half-dark, so a
            // neutral tint read as coffee beans against the lit ground.
            let tint = 1.0f + hash01 i * 0.35f

            pushGrainMoving
                grains.PositionsX.[i]
                grains.PositionsY.[i]
                grains.PositionsZ.[i]
                grains.Sizes.[i]
                (baseColor * darken * tint)
                grains.VelocitiesX.[i]
                grains.VelocitiesY.[i]
                grains.VelocitiesZ.[i]

        // Payload heap: the dug material sits IN the bucket as a mound of
        // grains that grows while digging and drains while pouring.
        if current.MachinePartCount > 4 && current.PayloadKg > 0.5f then
            let part = 4
            let hasPrevious = part < previous.MachinePartCount

            let bucketPosition =
                if hasPrevious then
                    Vector3.Lerp(previous.MachinePositions.[part], current.MachinePositions.[part], alpha)
                else
                    current.MachinePositions.[part]

            let bucketOrientation =
                if hasPrevious then
                    Quaternion.Slerp(previous.MachineOrientations.[part], current.MachineOrientations.[part], alpha)
                else
                    current.MachineOrientations.[part]

            let scaleFactor = current.MachineScale
            let tiltRotation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, bucketTilt)
            let fill = Math.Clamp(current.PayloadKg / MathF.Max(current.PayloadCapacityKg, 1.0f), 0.05f, 1.0f)
            let heapGrains = int (fill * 300.0f)
            let heapColor = materialColor current.PayloadMaterial

            for k in 0 .. heapGrains - 1 do
                let localPosition =
                    Vector3(
                        (hash01 (k * 7 + 1) - 0.55f) * 0.38f,
                        -0.14f + hash01 (k * 7 + 3) * 0.26f * fill,
                        (hash01 (k * 7 + 5) - 0.5f) * 0.46f
                    )
                    * scaleFactor

                let world =
                    bucketPosition
                    + Vector3.Transform(Vector3.Transform(localPosition, tiltRotation), bucketOrientation)

                pushGrain
                    world.X
                    world.Y
                    world.Z
                    (scaleFactor * (0.022f + hash01 (k * 7 + 9) * 0.02f))
                    (heapColor * (0.8f + hash01 (k * 7 + 11) * 0.4f))

        // Clump clusters: the mass carrier rendered as a wad of dirt.
        for i in 0 .. current.Count - 1 do
            let handle = current.Handles.[i]
            let mutable px = current.X.[i]
            let mutable py = current.Y.[i]
            let mutable pz = current.Z.[i]
            let mutable j = 0

            while j < previous.Count do
                if previous.Handles.[j] = handle then
                    px <- previous.X.[j] + (px - previous.X.[j]) * alpha
                    py <- previous.Y.[j] + (py - previous.Y.[j]) * alpha
                    pz <- previous.Z.[j] + (pz - previous.Z.[j]) * alpha
                    j <- previous.Count
                else
                    j <- j + 1

            let radius = current.Radius.[i]
            let baseColor = materialColor current.Materials.[i]
            let pieces = Math.Clamp(int (radius * 130.0f), 10, 44)

            for k in 0 .. pieces - 1 do
                let seed = handle * 31 + k
                let ox = (hash01 seed - 0.5f) * 1.5f * radius
                let oy = (hash01 (seed + 101) - 0.5f) * 1.5f * radius
                let oz = (hash01 (seed + 202) - 0.5f) * 1.5f * radius
                let tint = 0.8f + hash01 (seed + 303) * 0.4f

                pushGrain
                    (px + ox)
                    (py + oy)
                    (pz + oz)
                    (radius * (0.22f + hash01 (seed + 404) * 0.16f))
                    (baseColor * tint)

        if grainInstances > 0 then
            gl.UseProgram grainProgram
            gl.UniformMatrix4(gl.GetUniformLocation(grainProgram, "viewProjection"), 1u, false, &vp.M11)
            GlUtil.uniform3f gl grainProgram "sunDirection" sun
            GlUtil.uniform3f gl grainProgram "cameraPosition" cameraPosition
            gl.BindVertexArray grainVao
            gl.BindBuffer(BufferTargetARB.ArrayBuffer, grainInstanceVbo)

            GlUtil.upload
                gl
                BufferTargetARB.ArrayBuffer
                grainScratch
                (grainInstances * grainInstanceFloats)
                BufferUsageARB.StreamDraw

            gl.DrawElementsInstanced(
                PrimitiveType.Triangles,
                uint32 grainIndices.Length,
                DrawElementsType.UnsignedInt,
                IntPtr.Zero.ToPointer(),
                uint32 grainInstances
            )

    /// Deletes every GL object this renderer owns — the F9 world reload
    /// creates a fresh renderer against the new soil state.
    interface IDisposable with
        member _.Dispose() =
            for program in [| terrainProgram; clodProgram; solidProgram; grainProgram; skyProgram |] do
                gl.DeleteProgram program

            for tile in 0 .. tileCount - 1 do
                gl.DeleteVertexArray tileVaos.[tile]
                gl.DeleteBuffer tileVbos.[tile]

            gl.DeleteBuffer sharedIbo
            gl.DeleteVertexArray cubeVao
            gl.DeleteBuffer cubeVbo
            gl.DeleteBuffer cubeIbo
            gl.DeleteVertexArray clodVao
            gl.DeleteBuffer clodVbo
            gl.DeleteBuffer clodIbo
            gl.DeleteBuffer clodInstanceVbo
            gl.DeleteVertexArray grainVao
            gl.DeleteBuffer grainVbo
            gl.DeleteBuffer grainIbo
            gl.DeleteBuffer grainInstanceVbo
            gl.DeleteVertexArray skyVao

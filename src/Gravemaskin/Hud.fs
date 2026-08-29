namespace Gravemaskin.Shell

#nowarn "9"

open System
open System.Numerics
open Microsoft.FSharp.NativeInterop
open Silk.NET.OpenGL

/// Immediate-ish 2D overlay: 5×7 font text and solid bars, batched into one
/// orphaned VBO per frame. Pixel coordinates, origin top-left.
type Hud(gl: GL) =
    let program =
        GlUtil.program
            gl
            """#version 410 core
layout(location = 0) in vec2 position;
layout(location = 1) in vec2 uv;
layout(location = 2) in vec4 color;

uniform vec2 screenSize;

out vec2 vUv;
out vec4 vColor;

void main()
{
    vec2 ndc = vec2(position.x / screenSize.x * 2.0 - 1.0, 1.0 - position.y / screenSize.y * 2.0);
    vUv = uv;
    vColor = color;
    gl_Position = vec4(ndc, 0.0, 1.0);
}
"""
            """#version 410 core
in vec2 vUv;
in vec4 vColor;

uniform sampler2D atlas;

out vec4 fragColor;

void main()
{
    // uv.x < 0 marks a solid quad (bars, backgrounds).
    float glyph = vUv.x < 0.0 ? 1.0 : texture(atlas, vUv).r;
    fragColor = vec4(vColor.rgb, vColor.a * glyph);
}
"""

    let texture = gl.GenTexture()
    let vao = gl.GenVertexArray()
    let vbo = gl.GenBuffer()
    // pos2 + uv2 + color4 = 8 floats; 6 verts per quad; up to 2048 quads.
    let scratch = Array.zeroCreate<float32> (2048 * 6 * 8)
    let mutable floats = 0
    let mutable screenWidth = 1.0f
    let mutable screenHeight = 1.0f

    do
        let atlas = Font.buildAtlas ()
        gl.BindTexture(TextureTarget.Texture2D, texture)
        gl.PixelStore(PixelStoreParameter.UnpackAlignment, 1)

        do
            use pointer = fixed atlas

            gl.TexImage2D(
                TextureTarget.Texture2D,
                0,
                int InternalFormat.R8,
                uint32 Font.AtlasWidth,
                uint32 Font.AtlasHeight,
                0,
                PixelFormat.Red,
                PixelType.UnsignedByte,
                NativePtr.toVoidPtr pointer
            )

        gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, int TextureMinFilter.Nearest)
        gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, int TextureMagFilter.Nearest)

        gl.BindVertexArray vao
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, vbo)
        let stride = 32u
        gl.EnableVertexAttribArray 0u
        gl.VertexAttribPointer(0u, 2, VertexAttribPointerType.Float, false, stride, IntPtr.Zero.ToPointer())
        gl.EnableVertexAttribArray 1u
        gl.VertexAttribPointer(1u, 2, VertexAttribPointerType.Float, false, stride, IntPtr(8).ToPointer())
        gl.EnableVertexAttribArray 2u
        gl.VertexAttribPointer(2u, 4, VertexAttribPointerType.Float, false, stride, IntPtr(16).ToPointer())

    let vertex (x: float32) (y: float32) (u: float32) (v: float32) (color: Vector4) =
        if floats + 8 <= scratch.Length then
            scratch.[floats] <- x
            scratch.[floats + 1] <- y
            scratch.[floats + 2] <- u
            scratch.[floats + 3] <- v
            scratch.[floats + 4] <- color.X
            scratch.[floats + 5] <- color.Y
            scratch.[floats + 6] <- color.Z
            scratch.[floats + 7] <- color.W
            floats <- floats + 8

    let quad x0 y0 x1 y1 u0 v0 u1 v1 color =
        vertex x0 y0 u0 v0 color
        vertex x1 y0 u1 v0 color
        vertex x1 y1 u1 v1 color
        vertex x0 y0 u0 v0 color
        vertex x1 y1 u1 v1 color
        vertex x0 y1 u0 v1 color

    member _.Begin(width: int, height: int) =
        floats <- 0
        screenWidth <- float32 width
        screenHeight <- float32 height

    /// A plain filled quad (no background) — building block for icons.
    member _.Solid(x: float32, y: float32, width: float32, height: float32, color: Vector4) =
        quad x y (x + width) (y + height) -1.0f 0.0f -1.0f 0.0f color

    member _.Bar(x: float32, y: float32, width: float32, height: float32, fraction: float32, color: Vector4) =
        quad x y (x + width) (y + height) -1.0f 0.0f -1.0f 0.0f (Vector4(0.0f, 0.0f, 0.0f, 0.35f))
        let filled = Math.Clamp(fraction, 0.0f, 1.0f) * width

        if filled > 0.5f then
            quad x y (x + filled) (y + height) -1.0f 0.0f -1.0f 0.0f color

    member _.Text(x: float32, y: float32, scale: float32, color: Vector4, text: string) =
        let glyphWidth = 6.0f * scale
        let cellU = 1.0f / float32 Font.Columns
        let cellV = 1.0f / float32 Font.Rows
        let mutable penX = x

        for ch in text do
            let cell = Font.cellIndex ch

            if cell > 0 then
                let u0 = float32 (cell % Font.Columns) * cellU
                let v0 = float32 (cell / Font.Columns) * cellV

                quad
                    penX
                    y
                    (penX + 8.0f * scale)
                    (y + 8.0f * scale)
                    u0
                    v0
                    (u0 + cellU)
                    (v0 + cellV)
                    color

            penX <- penX + glyphWidth

    member _.End() =
        if floats > 0 then
            gl.Disable EnableCap.DepthTest
            // The NDC y-flip reverses winding: culling would eat every quad.
            gl.Disable EnableCap.CullFace
            gl.Enable EnableCap.Blend
            gl.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha)
            gl.UseProgram program
            GlUtil.uniform2f gl program "screenSize" screenWidth screenHeight
            GlUtil.uniform1i gl program "atlas" 0
            gl.ActiveTexture TextureUnit.Texture0
            gl.BindTexture(TextureTarget.Texture2D, texture)
            gl.BindVertexArray vao
            gl.BindBuffer(BufferTargetARB.ArrayBuffer, vbo)
            GlUtil.upload gl BufferTargetARB.ArrayBuffer scratch floats BufferUsageARB.StreamDraw
            gl.DrawArrays(PrimitiveType.Triangles, 0, uint32 (floats / 8))
            gl.Disable EnableCap.Blend
            gl.Enable EnableCap.CullFace
            gl.Enable EnableCap.DepthTest

namespace Gravemaskin.Shell

#nowarn "9"

open System
open System.Numerics
open Microsoft.FSharp.NativeInterop
open Silk.NET.OpenGL

/// Shader compilation, buffer upload and framebuffer plumbing. Everything here
/// is deliberately thin: the interesting code is in the GLSL, not around it.
[<RequireQualifiedAccess>]
module GlUtil =
    let private compile (gl: GL) (kind: ShaderType) (source: string) =
        let shader = gl.CreateShader kind
        gl.ShaderSource(shader, source)
        gl.CompileShader shader
        let mutable status = 0
        gl.GetShader(shader, ShaderParameterName.CompileStatus, &status)
        if status <> int GLEnum.True then
            let log = gl.GetShaderInfoLog shader
            gl.DeleteShader shader
            // Numbering the source makes driver line numbers usable; these
            // shaders are long enough that it matters.
            let numbered =
                source.Split '\n'
                |> Array.mapi (fun i line -> $"%4d{i + 1} | {line}")
                |> String.concat "\n"
            invalidOp $"{kind} compilation failed: {log}\n{numbered}"
        shader

    /// Compiles and links a vertex+fragment pair, deleting the intermediates
    /// either way. Raises with the driver's log (and numbered source) on failure.
    let program (gl: GL) (vertexSource: string) (fragmentSource: string) : uint32 =
        let vertex = compile gl ShaderType.VertexShader vertexSource
        let fragment = compile gl ShaderType.FragmentShader fragmentSource
        let id = gl.CreateProgram()
        gl.AttachShader(id, vertex)
        gl.AttachShader(id, fragment)
        gl.LinkProgram id
        let mutable status = 0
        gl.GetProgram(id, ProgramPropertyARB.LinkStatus, &status)
        gl.DeleteShader vertex
        gl.DeleteShader fragment
        if status <> int GLEnum.True then invalidOp $"Program link failed: {gl.GetProgramInfoLog id}"
        id

    let inline upload (gl: GL) (target: BufferTargetARB) (data: ^a[]) (count: int) (usage: BufferUsageARB) =
        use pointer = fixed data
        gl.BufferData(target, unativeint (count * sizeof< ^a>), NativePtr.toVoidPtr pointer, usage)

    let inline uploadSub (gl: GL) (target: BufferTargetARB) (data: ^a[]) (count: int) =
        if count > 0 then
            use pointer = fixed data
            gl.BufferSubData(target, nativeint 0, unativeint (count * sizeof< ^a>), NativePtr.toVoidPtr pointer)

    // Uniform setters. Locations are looked up by name every call: with a few
    // dozen uniforms per frame this is nowhere near the bottleneck, and the
    // call sites stay readable.
    let inline uniform1f (gl: GL) (p: uint32) (name: string) (v: float32) =
        gl.Uniform1(gl.GetUniformLocation(p, name), v)

    let inline uniform1i (gl: GL) (p: uint32) (name: string) (v: int) =
        gl.Uniform1(gl.GetUniformLocation(p, name), v)

    let inline uniform2f (gl: GL) (p: uint32) (name: string) (x: float32) (y: float32) =
        gl.Uniform2(gl.GetUniformLocation(p, name), x, y)

    let inline uniform3f (gl: GL) (p: uint32) (name: string) (v: Vector3) =
        gl.Uniform3(gl.GetUniformLocation(p, name), v.X, v.Y, v.Z)

    let inline uniform4f (gl: GL) (p: uint32) (name: string) (x, y, z, w) =
        gl.Uniform4(gl.GetUniformLocation(p, name), (x: float32), (y: float32), (z: float32), (w: float32))


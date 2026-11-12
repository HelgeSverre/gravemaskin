namespace Gravemaskin.Shell

open System
open System.Numerics

/// CPU-side mesh builder for the machine models: bevelled boxes, smooth
/// cylinders and extruded profiles, baked into one vertex buffer per body
/// part (pos3 + normal3 + color3). No GL here.
///
/// Winding is never reasoned about by hand: every face is given an outward
/// hint and flipped to match it.
type MeshBuilder() =
    let data = ResizeArray<float32>()
    let positions = ResizeArray<Vector3>()
    let indices = ResizeArray<uint32>()

    /// Applied to everything added while set (rotation + translation only).
    member val Transform = Matrix4x4.Identity with get, set

    member _.Vertices = data.ToArray()
    member _.Indices = indices.ToArray()

    member this.Vertex(position: Vector3, normal: Vector3, color: Vector3) =
        let p = Vector3.Transform(position, this.Transform)
        let n = Vector3.Normalize(Vector3.TransformNormal(normal, this.Transform))
        let index = positions.Count
        positions.Add p
        data.AddRange [| p.X; p.Y; p.Z; n.X; n.Y; n.Z; color.X; color.Y; color.Z |]
        index

    /// Triangle between existing vertices, wound to face `outward` (given
    /// in the builder's untransformed space).
    member this.Triangle(a: int, b: int, c: int, outward: Vector3) =
        let hint = Vector3.TransformNormal(outward, this.Transform)
        let normal = Vector3.Cross(positions.[b] - positions.[a], positions.[c] - positions.[a])

        if Vector3.Dot(normal, hint) >= 0.0f then
            indices.AddRange [| uint32 a; uint32 b; uint32 c |]
        else
            indices.AddRange [| uint32 a; uint32 c; uint32 b |]

    /// Flat-shaded convex polygon (fan), facing `outward`.
    member this.Face(points: Vector3[], outward: Vector3, color: Vector3) =
        let mutable normal = Vector3.Zero

        for i in 1 .. points.Length - 2 do
            normal <- normal + Vector3.Cross(points.[i] - points.[0], points.[i + 1] - points.[0])

        let normal =
            if Vector3.Dot(normal, outward) >= 0.0f then
                Vector3.Normalize normal
            else
                -Vector3.Normalize normal

        let first = this.Vertex(points.[0], normal, color)

        for i in 1 .. points.Length - 1 do
            this.Vertex(points.[i], normal, color) |> ignore

        for i in 1 .. points.Length - 2 do
            this.Triangle(first, first + i, first + i + 1, outward)

    /// Box with chamfered edges and corners — the bevel is what catches the
    /// light and stops a machine reading as stacked crates.
    member this.Box(center: Vector3, size: Vector3, color: Vector3, bevel: float32) =
        let half = size * 0.5f
        let b = MathF.Min(bevel, 0.45f * MathF.Min(half.X, MathF.Min(half.Y, half.Z)))
        let axes = [| Vector3.UnitX; Vector3.UnitY; Vector3.UnitZ |]
        let extent (axis: int) = if axis = 0 then half.X elif axis = 1 then half.Y else half.Z

        // Point with each axis either on its face (full) or pulled in by the bevel.
        let corner (sx: float32) (sy: float32) (sz: float32) (full: int) =
            let coordinate (axis: int) (sign: float32) =
                sign * (if axis = full then extent axis else extent axis - b)

            center + Vector3(coordinate 0 sx, coordinate 1 sy, coordinate 2 sz)

        let signs = [| -1.0f; 1.0f |]

        // Six faces.
        for axis in 0..2 do
            for s in signs do
                let u = (axis + 1) % 3
                let v = (axis + 2) % 3

                let point (su: float32) (sv: float32) =
                    center
                    + axes.[axis] * (s * extent axis)
                    + axes.[u] * (su * (extent u - b))
                    + axes.[v] * (sv * (extent v - b))

                this.Face([| point -1.0f -1.0f; point 1.0f -1.0f; point 1.0f 1.0f; point -1.0f 1.0f |], axes.[axis] * s, color)

        if b > 0.0f then
            // Twelve edge chamfers.
            for along in 0..2 do
                let a = (along + 1) % 3
                let c = (along + 2) % 3

                for sa in signs do
                    for sc in signs do
                        let point (onFace: int) (t: float32) =
                            center
                            + axes.[along] * (t * (extent along - b))
                            + axes.[a] * (sa * (if onFace = a then extent a else extent a - b))
                            + axes.[c] * (sc * (if onFace = c then extent c else extent c - b))

                        this.Face(
                            [| point a -1.0f; point a 1.0f; point c 1.0f; point c -1.0f |],
                            axes.[a] * sa + axes.[c] * sc,
                            color
                        )

            // Eight corner triangles.
            for sx in signs do
                for sy in signs do
                    for sz in signs do
                        this.Face(
                            [| corner sx sy sz 0; corner sx sy sz 1; corner sx sy sz 2 |],
                            Vector3(sx, sy, sz),
                            color
                        )

    /// Smooth-shaded capped cylinder. `axis`: 0 = X, 1 = Y, 2 = Z.
    member this.Cylinder(center: Vector3, axis: int, length: float32, diameter: float32, color: Vector3) =
        let sides = 20
        let along = if axis = 0 then Vector3.UnitX elif axis = 1 then Vector3.UnitY else Vector3.UnitZ
        let u = if axis = 0 then Vector3.UnitY elif axis = 1 then Vector3.UnitZ else Vector3.UnitX
        let v = Vector3.Cross(along, u)
        let radius = diameter * 0.5f

        let radial (i: int) =
            let angle = float32 i / float32 sides * MathF.PI * 2.0f
            u * MathF.Cos angle + v * MathF.Sin angle

        let sideStart = positions.Count

        for i in 0 .. sides - 1 do
            let r = radial i
            this.Vertex(center - along * (length * 0.5f) + r * radius, r, color) |> ignore
            this.Vertex(center + along * (length * 0.5f) + r * radius, r, color) |> ignore

        for i in 0 .. sides - 1 do
            let j = (i + 1) % sides
            let outward = radial i + radial j
            this.Triangle(sideStart + i * 2, sideStart + i * 2 + 1, sideStart + j * 2, outward)
            this.Triangle(sideStart + j * 2, sideStart + i * 2 + 1, sideStart + j * 2 + 1, outward)

        for s in [| -1.0f; 1.0f |] do
            let capCenter = center + along * (s * length * 0.5f)
            let middle = this.Vertex(capCenter, along * s, color)

            for i in 0 .. sides - 1 do
                this.Vertex(capCenter + radial i * radius, along * s, color) |> ignore

            for i in 0 .. sides - 1 do
                this.Triangle(middle, middle + 1 + i, middle + 1 + (i + 1) % sides, along * s)

    /// Convex XY profile extruded along Z from z0 to z1 (flat shaded).
    member this.Prism(profile: Vector2[], z0: float32, z1: float32, color: Vector3) =
        let mutable centroid = Vector2.Zero

        for p in profile do
            centroid <- centroid + p / float32 profile.Length

        this.Face(profile |> Array.map (fun p -> Vector3(p.X, p.Y, z0)), -Vector3.UnitZ, color)
        this.Face(profile |> Array.map (fun p -> Vector3(p.X, p.Y, z1)), Vector3.UnitZ, color)

        for i in 0 .. profile.Length - 1 do
            let a = profile.[i]
            let b = profile.[(i + 1) % profile.Length]
            let middle = (a + b) * 0.5f - centroid

            this.Face(
                [| Vector3(a.X, a.Y, z0); Vector3(b.X, b.Y, z0); Vector3(b.X, b.Y, z1); Vector3(a.X, a.Y, z1) |],
                Vector3(middle.X, middle.Y, 0.0f),
                color
            )

/// Paint scheme for the shared excavator model.
type Livery =
    { /// Cab frame, engine cover, upper bodywork.
      Body: Vector3
      /// Lower body band and tail.
      Skirt: Vector3
      /// Boom and stick.
      Arm: Vector3
      Counterweight: Vector3
      /// Minis carry a dozer blade; the 22-tonner doesn't.
      DozerBlade: bool }

/// The excavator, modeled part by part in each rigid body's local frame
/// (U17-class layout units; the renderer applies the rig's scale). Iterate
/// with `just preview`.
[<RequireQualifiedAccess>]
module MachineMesh =

    let tb216 =
        { Body = Vector3(0.9f, 0.89f, 0.85f)
          Skirt = Vector3(0.7f, 0.07f, 0.09f)
          Arm = Vector3(0.9f, 0.89f, 0.85f)
          Counterweight = Vector3(0.7f, 0.07f, 0.09f)
          DozerBlade = true }

    let u17 =
        { Body = Vector3(0.92f, 0.42f, 0.06f)
          Skirt = Vector3(0.24f, 0.24f, 0.26f)
          Arm = Vector3(0.92f, 0.42f, 0.06f)
          Counterweight = Vector3(0.24f, 0.24f, 0.26f)
          DozerBlade = true }

    let cat320 =
        { Body = Vector3(0.95f, 0.68f, 0.04f)
          Skirt = Vector3(0.08f, 0.08f, 0.09f)
          Arm = Vector3(0.95f, 0.68f, 0.04f)
          Counterweight = Vector3(0.95f, 0.68f, 0.04f)
          DozerBlade = false }

    let liveryFor (name: string) =
        if name = "TB216" then tb216
        elif name.Contains "320" then cat320
        else u17

    let private rubber = Vector3(0.07f, 0.07f, 0.075f)
    let private frame = Vector3(0.2f, 0.2f, 0.22f)
    let private steel = Vector3(0.46f, 0.46f, 0.49f)
    let private darkSteel = Vector3(0.24f, 0.24f, 0.26f)
    let private glass = Vector3(0.2f, 0.29f, 0.34f)
    let private v2 (x: float32) (y: float32) = Vector2(x, y)
    let private v3 (x: float32) (y: float32) (z: float32) = Vector3(x, y, z)

    /// Undercarriage: two tracks (belt, end wheels, rollers, grousers), the
    /// center frame and slew ring, optional dozer blade.
    let private chassis (livery: Livery) =
        let m = MeshBuilder()

        for side in [| -0.55f; 0.55f |] do
            // Belt: top and bottom runs wrapped round the end wheels. The
            // chassis body rides with its underside at y = −0.25.
            m.Box(v3 0.0f 0.085f side, v3 1.5f 0.04f 0.32f, rubber, 0.01f)
            m.Box(v3 0.0f -0.235f side, v3 1.5f 0.04f 0.32f, rubber, 0.01f)
            m.Cylinder(v3 -0.75f -0.075f side, 2, 0.32f, 0.36f, rubber)
            m.Cylinder(v3 0.75f -0.075f side, 2, 0.32f, 0.36f, rubber)
            // Sprocket (rear) and idler (front) hubs, proud of the belt.
            m.Cylinder(v3 -0.75f -0.075f side, 2, 0.35f, 0.2f, steel)
            m.Cylinder(v3 0.75f -0.075f side, 2, 0.35f, 0.22f, darkSteel)
            m.Cylinder(v3 0.75f -0.075f side, 2, 0.36f, 0.08f, steel)
            // Track frame with rollers between the runs.
            m.Box(v3 0.0f -0.06f side, v3 1.2f 0.15f 0.24f, frame, 0.02f)

            for x in [| -0.45f; -0.15f; 0.15f; 0.45f |] do
                m.Cylinder(v3 x -0.165f side, 2, 0.28f, 0.1f, darkSteel)

            // Grouser lugs on both runs.
            for i in 0..14 do
                let x = -0.7f + float32 i * 0.1f
                m.Box(v3 x 0.11f side, v3 0.045f 0.025f 0.32f, rubber * 1.6f, 0.0f)
                m.Box(v3 x -0.26f side, v3 0.045f 0.025f 0.32f, rubber * 1.6f, 0.0f)

        // Center frame, cross-members, slew ring.
        m.Box(v3 0.0f -0.04f 0.0f, v3 0.8f 0.2f 0.8f, frame, 0.03f)
        m.Box(v3 0.35f -0.08f 0.0f, v3 0.12f 0.12f 0.95f, frame, 0.02f)
        m.Box(v3 -0.35f -0.08f 0.0f, v3 0.12f 0.12f 0.95f, frame, 0.02f)
        m.Cylinder(v3 0.05f 0.11f 0.0f, 1, 0.1f, 0.72f, darkSteel)
        m.Cylinder(v3 0.05f 0.17f 0.0f, 1, 0.04f, 0.62f, steel)

        if livery.DozerBlade then
            // Blade arms and a blade with a top rib and wear edge.
            for z in [| -0.3f; 0.3f |] do
                m.Prism([| v2 0.45f -0.02f; v2 1.14f -0.2f; v2 1.14f -0.1f; v2 0.45f 0.08f |], z - 0.035f, z + 0.035f, frame)

            m.Box(v3 1.17f -0.12f 0.0f, v3 0.05f 0.3f 1.36f, darkSteel, 0.012f)
            m.Box(v3 1.2f 0.02f 0.0f, v3 0.07f 0.05f 1.36f, darkSteel * 1.2f, 0.01f)
            m.Box(v3 1.2f -0.27f 0.0f, v3 0.03f 0.05f 1.36f, steel, 0.0f)
            m.Cylinder(v3 0.9f -0.02f 0.0f, 0, 0.42f, 0.07f, steel * 0.5f)

        m

    /// Upper structure: deck, tail, engine cover, glazed cab, boom foot.
    let private house (livery: Livery) =
        let m = MeshBuilder()
        let body = livery.Body
        // Deck plate and the lower body band.
        m.Box(v3 -0.02f -0.345f 0.0f, v3 1.3f 0.05f 1.0f, frame, 0.01f)
        m.Box(v3 0.05f -0.22f 0.0f, v3 1.1f 0.22f 1.02f, livery.Skirt, 0.03f)
        // Rounded tail: the counterweight wraps the slew circle.
        m.Cylinder(v3 -0.22f -0.2f 0.0f, 1, 0.26f, 1.02f, livery.Counterweight)
        m.Cylinder(v3 -0.22f 0.08f 0.0f, 1, 0.34f, 1.0f, body)
        m.Cylinder(v3 -0.22f 0.265f 0.0f, 1, 0.03f, 0.94f, body * 0.9f)
        // Engine cover, right rear, with a louvred vent and filler cap.
        m.Box(v3 0.02f 0.0f 0.3f, v3 0.82f 0.4f 0.4f, body, 0.04f)

        for i in 0..4 do
            m.Box(v3 (-0.16f + float32 i * 0.09f) 0.02f 0.505f, v3 0.05f 0.2f 0.012f, frame, 0.0f)

        m.Cylinder(v3 0.3f 0.215f 0.34f, 1, 0.03f, 0.09f, frame)
        // Exhaust stack and its rain cap.
        m.Cylinder(v3 -0.38f 0.36f 0.3f, 1, 0.3f, 0.06f, frame)
        m.Cylinder(v3 -0.38f 0.5f 0.3f, 1, 0.03f, 0.085f, darkSteel)

        // Cab, left side: a sill, four posts, glazing all round, a roof.
        let cabX = 0.2f
        let cabZ = -0.2f
        let halfX = 0.38f
        let halfZ = 0.29f
        let floor = -0.11f
        let top = 1.0f
        m.Box(v3 cabX (floor + 0.09f) cabZ, v3 (halfX * 2.0f) 0.2f (halfZ * 2.0f), body, 0.02f)

        for sx in [| -1.0f; 1.0f |] do
            for sz in [| -1.0f; 1.0f |] do
                m.Box(
                    v3 (cabX + sx * (halfX - 0.025f)) ((floor + top) * 0.5f) (cabZ + sz * (halfZ - 0.025f)),
                    v3 0.05f (top - floor) 0.05f,
                    body,
                    0.012f
                )

        let glassMid = (floor + 0.2f + top) * 0.5f
        let glassTall = top - floor - 0.2f
        m.Box(v3 (cabX + halfX - 0.02f) glassMid cabZ, v3 0.015f glassTall (halfZ * 2.0f - 0.1f), glass, 0.0f)
        m.Box(v3 (cabX - halfX + 0.02f) (glassMid + 0.15f) cabZ, v3 0.015f (glassTall - 0.3f) (halfZ * 2.0f - 0.1f), glass, 0.0f)
        m.Box(v3 cabX glassMid (cabZ - halfZ + 0.02f), v3 (halfX * 2.0f - 0.1f) glassTall 0.015f, glass, 0.0f)
        m.Box(v3 cabX glassMid (cabZ + halfZ - 0.02f), v3 (halfX * 2.0f - 0.1f) glassTall 0.015f, glass, 0.0f)
        // Waist rail round the glazing.
        m.Box(v3 cabX (floor + 0.52f) (cabZ - halfZ + 0.012f), v3 (halfX * 2.0f - 0.08f) 0.03f 0.02f, body, 0.0f)
        m.Box(v3 cabX (floor + 0.52f) (cabZ + halfZ - 0.012f), v3 (halfX * 2.0f - 0.08f) 0.03f 0.02f, body, 0.0f)
        // Rear panel below the back window, door rail and handle.
        m.Box(v3 (cabX - halfX + 0.02f) (floor + 0.3f) cabZ, v3 0.03f 0.42f (halfZ * 2.0f - 0.1f), body, 0.0f)
        m.Box(v3 (cabX - 0.05f) glassMid (cabZ - halfZ + 0.012f), v3 0.035f glassTall 0.02f, body, 0.0f)
        m.Box(v3 (cabX + 0.02f) (floor + 0.55f) (cabZ - halfZ - 0.005f), v3 0.03f 0.12f 0.02f, frame, 0.0f)
        // Roof with an overhang, beacon, two work lights.
        m.Box(v3 (cabX + 0.02f) (top + 0.03f) cabZ, v3 (halfX * 2.0f + 0.1f) 0.07f (halfZ * 2.0f + 0.06f), body, 0.025f)
        m.Cylinder(v3 (cabX - 0.2f) (top + 0.1f) (cabZ + 0.1f), 1, 0.08f, 0.07f, v3 0.95f 0.5f 0.05f)

        for z in [| cabZ - 0.18f; cabZ + 0.18f |] do
            m.Box(v3 (cabX + halfX + 0.05f) (top - 0.02f) z, v3 0.05f 0.07f 0.1f, frame, 0.01f)
            m.Box(v3 (cabX + halfX + 0.078f) (top - 0.02f) z, v3 0.01f 0.05f 0.08f, v3 0.97f 0.95f 0.82f, 0.0f)

        // Handrail along the engine cover.
        m.Cylinder(v3 0.02f 0.3f 0.47f, 0, 0.7f, 0.025f, frame)

        for x in [| -0.31f; 0.35f |] do
            m.Cylinder(v3 x 0.25f 0.47f, 1, 0.1f, 0.025f, frame)

        // Boom foot: a pair of cheek plates carrying the boom pin.
        for z in [| -0.11f; 0.11f |] do
            m.Prism([| v2 0.45f -0.34f; v2 0.72f -0.3f; v2 0.72f 0.0f; v2 0.55f 0.02f; v2 0.45f -0.1f |], z - 0.02f, z + 0.02f, darkSteel)

        m.Cylinder(v3 0.6f -0.1f 0.0f, 2, 0.3f, 0.07f, steel)
        m

    /// Gooseneck boom: two box-section segments meeting at a raised knee.
    let private boom (livery: Livery) =
        let m = MeshBuilder()
        let half = 0.075f
        m.Prism([| v2 -0.99f -0.1f; v2 -0.06f 0.14f; v2 -0.02f 0.4f; v2 -0.3f 0.36f; v2 -0.99f 0.1f |], -half, half, livery.Arm)
        m.Prism([| v2 -0.06f 0.14f; v2 0.99f -0.085f; v2 0.99f 0.085f; v2 0.25f 0.36f; v2 -0.02f 0.4f |], -half, half, livery.Arm)
        // Welded side stiffeners, cylinder lugs, end bosses.
        m.Prism([| v2 -0.45f 0.12f; v2 -0.12f 0.2f; v2 -0.1f 0.32f; v2 -0.45f 0.22f |], -half - 0.012f, half + 0.012f, livery.Arm * 0.9f)
        m.Prism([| v2 0.15f 0.32f; v2 0.4f 0.27f; v2 0.3f 0.45f; v2 0.2f 0.45f |], -0.03f, 0.03f, darkSteel)
        m.Prism([| v2 -0.45f 0.02f; v2 -0.25f 0.07f; v2 -0.33f 0.18f |], -half - 0.03f, half + 0.03f, darkSteel)
        m.Cylinder(v3 -0.95f 0.0f 0.0f, 2, 0.2f, 0.2f, livery.Arm * 0.85f)
        m.Cylinder(v3 -0.95f 0.0f 0.0f, 2, 0.24f, 0.07f, steel)
        m.Cylinder(v3 0.95f 0.0f 0.0f, 2, 0.19f, 0.17f, livery.Arm * 0.85f)
        m.Cylinder(v3 0.95f 0.0f 0.0f, 2, 0.23f, 0.06f, steel)
        m

    /// Stick: tapered box section with the bucket-cylinder tower on top.
    let private stick (livery: Livery) =
        let m = MeshBuilder()
        let half = 0.06f
        m.Prism([| v2 -0.62f -0.09f; v2 0.58f -0.055f; v2 0.58f 0.055f; v2 -0.28f 0.15f; v2 -0.62f 0.1f |], -half, half, livery.Arm)
        m.Prism([| v2 -0.45f 0.1f; v2 -0.22f 0.12f; v2 -0.32f 0.24f; v2 -0.4f 0.24f |], -0.03f, 0.03f, darkSteel)
        m.Cylinder(v3 -0.55f 0.0f 0.0f, 2, 0.18f, 0.17f, livery.Arm * 0.85f)
        m.Cylinder(v3 -0.55f 0.0f 0.0f, 2, 0.22f, 0.06f, steel)
        m.Cylinder(v3 0.55f 0.0f 0.0f, 2, 0.17f, 0.13f, livery.Arm * 0.85f)
        m.Cylinder(v3 0.55f 0.0f 0.0f, 2, 0.21f, 0.05f, steel)
        m

    /// Bucket: a curved shell between two profiled side plates, a hardened
    /// lip with teeth, and the hanger ears. Authored in the shell's tilted
    /// frame (mouth toward −X), then turned by `tilt` like the collider.
    let private bucket (tilt: float32) =
        let m = MeshBuilder()
        let shell = Vector3(0.2f, 0.2f, 0.22f)
        let wear = Vector3(0.4f, 0.38f, 0.35f)
        m.Transform <- Matrix4x4.CreateRotationZ tilt

        // Side profile, mouth open between the first and last points.
        let profile =
            [| v2 -0.27f -0.2f
               v2 0.08f -0.2f
               v2 0.2f -0.14f
               v2 0.26f -0.02f
               v2 0.24f 0.1f
               v2 0.14f 0.19f
               v2 -0.08f 0.2f |]

        for z in [| -0.3f; 0.27f |] do
            m.Prism(profile, z, z + 0.03f, shell * 0.9f)

        // Shell skin: one plate per profile edge, thickness inward.
        let mutable centroid = Vector2.Zero

        for p in profile do
            centroid <- centroid + p / float32 profile.Length

        for i in 0 .. profile.Length - 2 do
            let a = profile.[i]
            let b = profile.[i + 1]
            let inward = Vector2.Normalize(centroid - (a + b) * 0.5f) * 0.025f
            m.Prism([| a; b; b + inward; a + inward |], -0.3f, 0.3f, (if i % 2 = 0 then shell else shell * 1.12f))

        // Side cutters, lip plate, teeth with adapters.
        m.Box(v3 -0.27f -0.19f 0.0f, v3 0.09f 0.04f 0.6f, wear, 0.008f)

        for z in [| -0.21f; -0.07f; 0.07f; 0.21f |] do
            m.Prism([| v2 -0.3f -0.215f; v2 -0.42f -0.2f; v2 -0.3f -0.165f |], z - 0.03f, z + 0.03f, wear * 1.15f)
            m.Box(v3 -0.28f -0.19f z, v3 0.07f 0.055f 0.075f, wear * 0.8f, 0.008f)

        // Hanger ears and pins (back in the body frame).
        m.Transform <- Matrix4x4.Identity

        for z in [| -0.1f; 0.1f |] do
            m.Prism([| v2 -0.36f -0.08f; v2 -0.12f -0.1f; v2 0.0f 0.12f; v2 -0.12f 0.2f; v2 -0.36f 0.08f |], z - 0.02f, z + 0.02f, darkSteel)

        m.Cylinder(v3 -0.28f 0.0f 0.0f, 2, 0.26f, 0.06f, steel)
        m.Cylinder(v3 -0.08f 0.12f 0.0f, 2, 0.26f, 0.05f, steel)
        m

    /// The five part meshes, in body order: chassis, house, boom, stick,
    /// bucket.
    let build (name: string) (bucketTilt: float32) =
        let livery = liveryFor name
        [| chassis livery; house livery; boom livery; stick livery; bucket bucketTilt |]

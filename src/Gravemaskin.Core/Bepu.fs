namespace Gravemaskin

open System.Numerics
open BepuPhysics
open BepuPhysics.Collidables
open BepuPhysics.CollisionDetection
open BepuPhysics.Constraints
open BepuUtilities

/// BEPU callback structs and byref-ceremony wrappers. Gameplay code calls
/// these; it never writes `let mutable desc … &desc` itself.
[<RequireQualifiedAccess>]
module Bepu =

    [<Struct>]
    type NarrowPhaseCallbacks =
        /// Collision group per body handle value; equal nonzero groups don't
        /// collide (how linked machine parts skip self-collision).
        val Groups: int[]
        new(groups: int[]) = { Groups = groups }

        interface INarrowPhaseCallbacks with
            member _.Initialize(_simulation: Simulation) = ()

            member this.AllowContactGeneration
                (_workerIndex: int, a: CollidableReference, b: CollidableReference, _speculativeMargin: byref<float32>)
                =
                if a.Mobility = CollidableMobility.Static && b.Mobility = CollidableMobility.Static then
                    false
                elif a.Mobility <> CollidableMobility.Static && b.Mobility <> CollidableMobility.Static then
                    let groupA = this.Groups.[a.BodyHandle.Value]
                    let groupB = this.Groups.[b.BodyHandle.Value]
                    groupA = 0 || groupA <> groupB
                else
                    true

            member _.AllowContactGeneration
                (_workerIndex: int, _pair: CollidablePair, _childIndexA: int, _childIndexB: int)
                =
                true

            member this.ConfigureContactManifold<'TManifold
                when 'TManifold: unmanaged and 'TManifold: struct and 'TManifold :> IContactManifold<'TManifold>>
                (
                    _workerIndex: int,
                    pair: CollidablePair,
                    _manifold: byref<'TManifold>,
                    pairMaterial: byref<PairMaterialProperties>
                ) =
                // Machine parts (group 1) get low-friction ground contact:
                // tractive effort is injected by the track model, so contact
                // friction on the hull only adds plow drag — with μ=1.0 the
                // chassis wedged on every terrain facet and crawled at
                // 0.06 m/s.
                let groups = this.Groups

                let isMachine (c: CollidableReference) =
                    c.Mobility <> CollidableMobility.Static && groups.[c.BodyHandle.Value] = 1

                pairMaterial.FrictionCoefficient <-
                    if isMachine pair.A || isMachine pair.B then 0.45f else 1.0f

                pairMaterial.MaximumRecoveryVelocity <- 2.0f
                pairMaterial.SpringSettings <- SpringSettings(30.0f, 1.0f)
                true

            member _.ConfigureContactManifold
                (
                    _workerIndex: int,
                    _pair: CollidablePair,
                    _childIndexA: int,
                    _childIndexB: int,
                    _manifold: byref<ConvexContactManifold>
                ) =
                true

            member _.Dispose() = ()

    [<Struct>]
    type PoseIntegratorCallbacks =
        val mutable Gravity: Vector3
        val mutable gravityWideDt: Vector3Wide

        new(gravity: Vector3) =
            { Gravity = gravity
              gravityWideDt = Unchecked.defaultof<Vector3Wide> }

        interface IPoseIntegratorCallbacks with
            member _.AngularIntegrationMode = AngularIntegrationMode.Nonconserving
            member _.AllowSubstepsForUnconstrainedBodies = false
            member _.IntegrateVelocityForKinematics = false
            member _.Initialize(_simulation: Simulation) = ()

            member this.PrepareForIntegration(dt: float32) =
                this.gravityWideDt <- Vector3Wide.Broadcast(this.Gravity * dt)

            member this.IntegrateVelocity
                (
                    _bodyIndices: Vector<int>,
                    _position: Vector3Wide,
                    _orientation: QuaternionWide,
                    _localInertia: BodyInertiaWide,
                    _integrationMask: Vector<int>,
                    _workerIndex: int,
                    _dt: Vector<float32>,
                    velocity: byref<BodyVelocityWide>
                ) =
                velocity.Linear <- velocity.Linear + this.gravityWideDt

    let addStaticBox (simulation: Simulation) (position: Vector3) (size: Vector3) =
        let shape = Box(size.X, size.Y, size.Z)
        let desc = StaticDescription(position, simulation.Shapes.Add(&shape))
        simulation.Statics.Add(&desc)

    let addDynamicSphere (simulation: Simulation) (position: Vector3) (radius: float32) (mass: float32) =
        let shape = Sphere(radius)

        let desc =
            BodyDescription.CreateDynamic(
                RigidPose(position),
                shape.ComputeInertia(mass),
                CollidableDescription(simulation.Shapes.Add(&shape)),
                BodyActivityDescription(0.01f)
            )

        simulation.Bodies.Add(&desc)

    let addDynamicBox (simulation: Simulation) (position: Vector3) (size: Vector3) (mass: float32) =
        let shape = Box(size.X, size.Y, size.Z)

        let desc =
            BodyDescription.CreateDynamic(
                RigidPose(position),
                shape.ComputeInertia(mass),
                CollidableDescription(simulation.Shapes.Add(&shape)),
                BodyActivityDescription(0.01f)
            )

        simulation.Bodies.Add(&desc)

    /// Bucket: a dynamic compound of five plates (floor, back, top shell,
    /// two sides) with ONE opening — the mouth at −X. Curl is a −Z rotation,
    /// which swings the mouth upward: a curled bucket cradles clumps, a
    /// dumped one pours them, and the closed top reads as the shell of a
    /// real bucket instead of an upright bin.
    let addDynamicOpenBucket
        (simulation: Simulation)
        (pool: BepuUtilities.Memory.BufferPool)
        (position: Vector3)
        (size: Vector3)
        (mass: float32)
        =
        let thickness = size.Y * 0.15f
        let mutable builder = new CompoundBuilder(pool, simulation.Shapes, 5)

        let addPlate (plateSize: Vector3) (offset: Vector3) (weight: float32) =
            let shape = Box(plateSize.X, plateSize.Y, plateSize.Z)
            let pose = RigidPose(offset)
            builder.Add(&shape, &pose, weight)

        addPlate (Vector3(size.X, thickness, size.Z)) (Vector3(0.0f, -(size.Y - thickness) * 0.5f, 0.0f)) (mass * 0.32f)
        addPlate (Vector3(thickness, size.Y, size.Z)) (Vector3((size.X - thickness) * 0.5f, 0.0f, 0.0f)) (mass * 0.24f)
        // Top shell: covers the rear 70% — the mouth keeps a wide lip.
        addPlate
            (Vector3(size.X * 0.7f, thickness, size.Z))
            (Vector3(size.X * 0.15f, (size.Y - thickness) * 0.5f, 0.0f))
            (mass * 0.2f)

        addPlate (Vector3(size.X, size.Y, thickness)) (Vector3(0.0f, 0.0f, -(size.Z - thickness) * 0.5f)) (mass * 0.12f)
        addPlate (Vector3(size.X, size.Y, thickness)) (Vector3(0.0f, 0.0f, (size.Z - thickness) * 0.5f)) (mass * 0.12f)

        let mutable children = Unchecked.defaultof<BepuUtilities.Memory.Buffer<CompoundChild>>
        let mutable inertia = Unchecked.defaultof<BodyInertia>
        let mutable center = Vector3.Zero
        builder.BuildDynamicCompound(&children, &inertia, &center)
        builder.Dispose()
        let mutable compound = Compound(children)

        let desc =
            BodyDescription.CreateDynamic(
                RigidPose(position + center),
                inertia,
                CollidableDescription(simulation.Shapes.Add(&compound)),
                BodyActivityDescription(0.01f)
            )

        simulation.Bodies.Add(&desc)

    let addKinematicSphere (simulation: Simulation) (position: Vector3) (radius: float32) =
        let shape = Sphere(radius)

        let desc =
            BodyDescription.CreateKinematic(
                RigidPose(position),
                CollidableDescription(simulation.Shapes.Add(&shape)),
                BodyActivityDescription(0.01f)
            )

        simulation.Bodies.Add(&desc)

    let addKinematicBox (simulation: Simulation) (position: Vector3) (size: Vector3) =
        let shape = Box(size.X, size.Y, size.Z)

        let desc =
            BodyDescription.CreateKinematic(
                RigidPose(position),
                CollidableDescription(simulation.Shapes.Add(&shape)),
                BodyActivityDescription(0.01f)
            )

        simulation.Bodies.Add(&desc)

    let addHinge
        (simulation: Simulation)
        (a: BodyHandle)
        (b: BodyHandle)
        (axis: Vector3)
        (offsetA: Vector3)
        (offsetB: Vector3)
        =
        let mutable hinge =
            Hinge(
                LocalHingeAxisA = axis,
                LocalHingeAxisB = axis,
                LocalOffsetA = offsetA,
                LocalOffsetB = offsetB,
                SpringSettings = SpringSettings(30.0f, 1.0f)
            )

        simulation.Solver.Add(a, b, &hinge)

    /// Hard stroke limit about a Z hinge: the solver-level equivalent of a
    /// hydraulic cylinder bottoming out — external force cannot push the
    /// joint past it (verified empirically: identity bases measure twist
    /// about Z with the same sign convention as our jointAngle).
    let addTwistLimit
        (simulation: Simulation)
        (a: BodyHandle)
        (b: BodyHandle)
        (minAngle: float32)
        (maxAngle: float32)
        =
        let mutable limit =
            TwistLimit(
                LocalBasisA = Quaternion.Identity,
                LocalBasisB = Quaternion.Identity,
                MinimumAngle = minAngle,
                MaximumAngle = maxAngle,
                SpringSettings = SpringSettings(30.0f, 1.0f)
            )

        simulation.Solver.Add(a, b, &limit)

    let addAngularMotor (simulation: Simulation) (a: BodyHandle) (b: BodyHandle) (axis: Vector3) =
        let mutable motor =
            AngularAxisMotor(LocalAxisA = axis, TargetVelocity = 0.0f, Settings = MotorSettings(1.0f, 1e-6f))

        simulation.Solver.Add(a, b, &motor)

    /// Per-tick hydraulic retune: velocity target + force ceiling.
    let retuneAngularMotor
        (simulation: Simulation)
        (handle: ConstraintHandle)
        (axis: Vector3)
        (command: ActuatorCommand)
        =
        // BEPU's AngularAxisMotor targets A's velocity relative to B; our
        // convention (and jointAngle) is child-relative-to-parent, so negate
        // here once instead of at every call site (verified empirically:
        // without this, "boom up" drives the boom into the ground).
        let mutable motor =
            AngularAxisMotor(
                LocalAxisA = axis,
                TargetVelocity = -command.TargetVelocity,
                Settings = MotorSettings(command.MaxForce, 1e-6f)
            )

        simulation.Solver.ApplyDescription(handle, &motor)

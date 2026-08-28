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
        interface INarrowPhaseCallbacks with
            member _.Initialize(_simulation: Simulation) = ()

            member _.AllowContactGeneration
                (_workerIndex: int, a: CollidableReference, b: CollidableReference, _speculativeMargin: byref<float32>)
                =
                a.Mobility = CollidableMobility.Dynamic || b.Mobility = CollidableMobility.Dynamic

            member _.AllowContactGeneration
                (_workerIndex: int, _pair: CollidablePair, _childIndexA: int, _childIndexB: int)
                =
                true

            member _.ConfigureContactManifold<'TManifold
                when 'TManifold: unmanaged and 'TManifold: struct and 'TManifold :> IContactManifold<'TManifold>>
                (
                    _workerIndex: int,
                    _pair: CollidablePair,
                    _manifold: byref<'TManifold>,
                    pairMaterial: byref<PairMaterialProperties>
                ) =
                pairMaterial.FrictionCoefficient <- 1.0f
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
        let mutable motor =
            AngularAxisMotor(
                LocalAxisA = axis,
                TargetVelocity = command.TargetVelocity,
                Settings = MotorSettings(command.MaxForce, 1e-6f)
            )

        simulation.Solver.ApplyDescription(handle, &motor)

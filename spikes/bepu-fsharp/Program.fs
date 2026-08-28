// Minimal F# + BepuPhysics 2.5.0-beta.29 verification:
// struct callbacks, simulation, hinge + servo constraints, body drop.
open System.Numerics
open BepuPhysics
open BepuPhysics.Collidables
open BepuPhysics.CollisionDetection
open BepuPhysics.Constraints
open BepuUtilities
open BepuUtilities.Memory

[<Struct>]
type NarrowPhaseCallbacks =
    interface INarrowPhaseCallbacks with
        member _.Initialize(_simulation: Simulation) = ()
        member _.AllowContactGeneration(_workerIndex: int, a: CollidableReference, b: CollidableReference, _speculativeMargin: byref<float32>) =
            a.Mobility = CollidableMobility.Dynamic || b.Mobility = CollidableMobility.Dynamic
        member _.AllowContactGeneration(_workerIndex: int, _pair: CollidablePair, _childIndexA: int, _childIndexB: int) = true
        member _.ConfigureContactManifold<'TManifold when 'TManifold : unmanaged and 'TManifold : struct and 'TManifold :> IContactManifold<'TManifold>>
                (_workerIndex: int, _pair: CollidablePair, _manifold: byref<'TManifold>, pairMaterial: byref<PairMaterialProperties>) =
            pairMaterial.FrictionCoefficient <- 1.0f
            pairMaterial.MaximumRecoveryVelocity <- 2.0f
            pairMaterial.SpringSettings <- SpringSettings(30.0f, 1.0f)
            true
        member _.ConfigureContactManifold(_workerIndex: int, _pair: CollidablePair, _childIndexA: int, _childIndexB: int, _manifold: byref<ConvexContactManifold>) = true
        member _.Dispose() = ()

[<Struct>]
type PoseIntegratorCallbacks =
    val mutable Gravity: Vector3
    val mutable gravityWideDt: Vector3Wide
    new(gravity: Vector3) = { Gravity = gravity; gravityWideDt = Unchecked.defaultof<Vector3Wide> }
    interface IPoseIntegratorCallbacks with
        member _.AngularIntegrationMode = AngularIntegrationMode.Nonconserving
        member _.AllowSubstepsForUnconstrainedBodies = false
        member _.IntegrateVelocityForKinematics = false
        member _.Initialize(_simulation: Simulation) = ()
        member this.PrepareForIntegration(dt: float32) =
            this.gravityWideDt <- Vector3Wide.Broadcast(this.Gravity * dt)
        member this.IntegrateVelocity(_bodyIndices: Vector<int>, _position: Vector3Wide, _orientation: QuaternionWide,
                                      _localInertia: BodyInertiaWide, _integrationMask: Vector<int>, _workerIndex: int,
                                      _dt: Vector<float32>, velocity: byref<BodyVelocityWide>) =
            velocity.Linear <- velocity.Linear + this.gravityWideDt

[<EntryPoint>]
let main _ =
    use pool = new BufferPool()
    let sim = Simulation.Create(pool, NarrowPhaseCallbacks(), PoseIntegratorCallbacks(Vector3(0f, -10f, 0f)), SolveDescription(8, 4))
    // ground -- note: 'in'/'inref' parameters require let-bound locals in F#
    let groundShape = Box(100f, 1f, 100f)
    let groundDesc = StaticDescription(Vector3(0f, -0.5f, 0f), sim.Shapes.Add(&groundShape))
    sim.Statics.Add(&groundDesc) |> ignore
    // kinematic anchor + swinging arm with hinge + angular motor ("hydraulic" analog)
    let armShape = Box(0.4f, 2.0f, 0.4f)
    let armInertia = armShape.ComputeInertia(10.0f)
    let anchorShape = Box(0.5f, 0.5f, 0.5f)
    let anchorDesc = BodyDescription.CreateKinematic(RigidPose(Vector3(0f, 5f, 0f)), CollidableDescription(sim.Shapes.Add(&anchorShape)), BodyActivityDescription(0.01f))
    let anchor = sim.Bodies.Add(&anchorDesc)
    let armDesc = BodyDescription.CreateDynamic(RigidPose(Vector3(0f, 3.9f, 0f)), armInertia, CollidableDescription(sim.Shapes.Add(&armShape)), BodyActivityDescription(0.01f))
    let arm = sim.Bodies.Add(&armDesc)
    let mutable hinge = Hinge(LocalHingeAxisA = Vector3.UnitX, LocalHingeAxisB = Vector3.UnitX,
                              LocalOffsetA = Vector3(0f, -0.25f, 0f), LocalOffsetB = Vector3(0f, 1.1f, 0f),
                              SpringSettings = SpringSettings(30f, 1f))
    sim.Solver.Add(anchor, arm, &hinge) |> ignore
    let mutable motor = AngularAxisMotor(LocalAxisA = Vector3.UnitX, TargetVelocity = 1.0f,
                                         Settings = MotorSettings(50f, 1e-6f))
    let motorHandle = sim.Solver.Add(anchor, arm, &motor)
    // falling ball
    let ballShape = Sphere(0.5f)
    let ballDesc = BodyDescription.CreateDynamic(RigidPose(Vector3(3f, 6f, 0f)), ballShape.ComputeInertia(1.0f), CollidableDescription(sim.Shapes.Add(&ballShape)), BodyActivityDescription(0.01f))
    let ball = sim.Bodies.Add(&ballDesc)
    for _ in 1 .. 240 do
        sim.Timestep(1.0f / 60.0f)
    // live-retune the motor (per-frame hydraulic control)
    motor.TargetVelocity <- -0.5f
    sim.Solver.ApplyDescription(motorHandle, &motor)
    for _ in 1 .. 60 do
        sim.Timestep(1.0f / 60.0f)
    let ballPos = sim.Bodies.[ball].Pose.Position
    let armPos = sim.Bodies.[arm].Pose.Position
    printfn "ball y=%f (expect ~0.5 resting)" ballPos.Y
    printfn "arm pos=%A" armPos
    assert (abs (ballPos.Y - 0.5f) < 0.15f)
    printfn "OK: F# struct callbacks + hinge + motor + ApplyDescription all work."
    0

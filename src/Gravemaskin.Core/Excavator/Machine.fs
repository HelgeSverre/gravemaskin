namespace Gravemaskin

open System
open System.Numerics
open BepuPhysics
open BepuPhysics.Constraints

/// The excavator rig: five rigid bodies (chassis, house, boom, stick,
/// bucket) joined by hinges whose AngularAxisMotors are retuned every tick
/// from the hydraulic model. Nothing is animated — stalls, sag, self-lift,
/// and tipping all emerge from force caps and gravity.
type Machine(physics: Physics, spec: MachineSpec, origin: Vector3) =
    let simulation = physics.Simulation

    // ---- assembly (spawned with the arm horizontal along +X; it sags onto
    // its motors at startup, which reads as the machine "waking up") ----
    let chassisCenter = origin + Vector3(0.0f, 0.45f, 0.0f)
    let houseCenter = origin + Vector3(-0.15f, 0.85f, 0.0f)
    let swingPivot = origin + Vector3(0.05f, 0.55f, 0.0f)
    let boomPivot = origin + Vector3(0.45f, 0.75f, 0.0f)
    let boomCenter = boomPivot + Vector3(0.95f, 0.0f, 0.0f)
    let stickPivot = boomPivot + Vector3(1.9f, 0.0f, 0.0f)
    let stickCenter = stickPivot + Vector3(0.55f, 0.0f, 0.0f)
    let bucketPivot = stickPivot + Vector3(1.1f, 0.0f, 0.0f)
    let bucketCenter = bucketPivot + Vector3(0.30f, 0.0f, 0.0f)

    let chassis =
        Bepu.addDynamicBox simulation chassisCenter (Vector3(1.7f, 0.5f, 1.4f)) Tuning.u17Masses.Chassis

    let house =
        Bepu.addDynamicBox simulation houseCenter (Vector3(1.25f, 0.7f, 0.95f)) Tuning.u17Masses.House

    let boom =
        Bepu.addDynamicBox simulation boomCenter (Vector3(1.9f, 0.18f, 0.15f)) Tuning.u17Masses.Boom

    let stick =
        Bepu.addDynamicBox simulation stickCenter (Vector3(1.1f, 0.14f, 0.12f)) Tuning.u17Masses.Stick

    // ponytail: single-box bucket (the open-plate compound is deferred —
    // the load-scalar payload made it unnecessary for bucket-fill). Note the
    // computed cutting edge extends ~0.15 m beyond this box: the edge point
    // carves/measures, the box collides.
    let bucket =
        Bepu.addDynamicBox simulation bucketCenter (Vector3(0.5f, 0.4f, 0.6f)) Tuning.u17Masses.Bucket

    // Machine parts don't self-collide (collision group; RagdollDemo pattern).
    do
        for handle in [| chassis; house; boom; stick; bucket |] do
            physics.SetCollisionGroup(handle, 1)

    let swingMotor =
        Bepu.addHinge simulation chassis house Vector3.UnitY (swingPivot - chassisCenter) (swingPivot - houseCenter)
        |> ignore

        Bepu.addAngularMotor simulation chassis house Vector3.UnitY

    let boomMotor =
        Bepu.addHinge simulation house boom Vector3.UnitZ (boomPivot - houseCenter) (boomPivot - boomCenter)
        |> ignore

        Bepu.addAngularMotor simulation house boom Vector3.UnitZ

    let stickMotor =
        Bepu.addHinge simulation boom stick Vector3.UnitZ (stickPivot - boomCenter) (stickPivot - stickCenter)
        |> ignore

        Bepu.addAngularMotor simulation boom stick Vector3.UnitZ

    let bucketMotor =
        Bepu.addHinge simulation stick bucket Vector3.UnitZ (bucketPivot - stickCenter) (bucketPivot - bucketCenter)
        |> ignore

        Bepu.addAngularMotor simulation stick bucket Vector3.UnitZ

    // ---- per-tick state (all preallocated; the tick path allocates nothing) ----
    let laggedAxes = Array.zeroCreate<float32> Hydraulics.FunctionCount
    let circuitOf = [| 0; 1; 0; 2; 0; 1 |] // boom, stick, bucket, swing, trackL, trackR

    let qMax =
        [| Tuning.u17BoomJoint.QMax
           Tuning.u17StickJoint.QMax
           Tuning.u17BucketJoint.QMax
           Tuning.u17SwingQMax
           Tuning.u17TrackQMax
           Tuning.u17TrackQMax |]

    let grantedFlow = Array.zeroCreate<float32> Hydraulics.FunctionCount
    let grantedScale = Array.zeroCreate<float32> Hydraulics.FunctionCount
    let circuitDemand = Array.zeroCreate<float32> Hydraulics.CircuitCount
    // Change-guard: last applied velocity/cap per motor so idle machines
    // stop touching the solver and can sleep. Two flat arrays, not tuples —
    // tuples are heap objects and this is the tick path.
    let lastVelocity = Array.create 4 0.0f
    let lastCap = Array.create 4 -1.0f
    // Payload (kg per material) + inertia change-guard.
    let bucketLoad = Array.zeroCreate<float> 5
    let mutable lastInertiaMass = 0.0f

    let joints =
        [| Tuning.u17BoomJoint; Tuning.u17StickJoint; Tuning.u17BucketJoint |]

    let motors = [| boomMotor; stickMotor; bucketMotor |]
    let motorAxes = [| Vector3.UnitZ; Vector3.UnitZ; Vector3.UnitZ |]

    /// Twist of the child body relative to its parent about a (shared local)
    /// hinge axis. Assembled at identity, so rest angle = 0 everywhere.
    let jointAngle (parent: BodyHandle) (child: BodyHandle) (axis: Vector3) =
        let qa = simulation.Bodies.[parent].Pose.Orientation
        let qb = simulation.Bodies.[child].Pose.Orientation
        let rel = Quaternion.Multiply(Quaternion.Conjugate qa, qb)
        let proj = Vector3.Dot(Vector3(rel.X, rel.Y, rel.Z), axis)
        let angle = 2.0f * MathF.Atan2(proj, rel.W)

        if angle > MathF.PI then angle - 2.0f * MathF.PI
        elif angle < -MathF.PI then angle + 2.0f * MathF.PI
        else angle

    let jointParents = [| house; boom; stick |]
    let jointChildren = [| boom; stick; bucket |]
    let partsScratch = [| chassis; house; boom; stick; bucket |]

    let retune (motorIndex: int) (motor: ConstraintHandle) (axis: Vector3) (velocity: float32) (cap: float32) =
        if MathF.Abs(lastVelocity.[motorIndex] - velocity) > 0.001f
           || MathF.Abs(lastCap.[motorIndex] - cap) > 1.0f then
            lastVelocity.[motorIndex] <- velocity
            lastCap.[motorIndex] <- cap

            Bepu.retuneAngularMotor
                simulation
                motor
                axis
                { TargetVelocity = velocity
                  MaxForce = cap }

    member _.Chassis = chassis
    member _.House = house
    member _.Boom = boom
    member _.Stick = stick
    member _.Bucket = bucket
    member _.BoomAngle = jointAngle house boom Vector3.UnitZ
    member _.StickAngle = jointAngle boom stick Vector3.UnitZ
    member _.BucketAngle = jointAngle stick bucket Vector3.UnitZ
    member _.SwingAngle = jointAngle chassis house Vector3.UnitY

    /// Chassis roll+pitch magnitude in radians (0 = level). Tipping shows up
    /// here without any special-case code.
    member _.ChassisTilt =
        let up = Vector3.Transform(Vector3.UnitY, simulation.Bodies.[chassis].Pose.Orientation)
        MathF.Acos(Math.Clamp(Vector3.Dot(up, Vector3.UnitY), -1.0f, 1.0f))

    member _.BucketTipPosition =
        let bucketRef = simulation.Bodies.[bucket]

        bucketRef.Pose.Position
        + Vector3.Transform(Vector3(Tuning.u17BucketTipRadius - 0.30f, -0.2f, 0.0f), bucketRef.Pose.Orientation)

    /// Granted flow scale for a function this tick (1 = full speed): the
    /// flow-sharing observable, surfaced for tests and the HUD.
    member _.GrantedScale(functionIndex: int) = grantedScale.[functionIndex]

    /// Payload in the bucket, kg per material (the load scalar: resting soil
    /// converts to bucket mass ASAP — it costs no clump budget and routes
    /// weight into COM, tipping, and hydraulic load exactly as it should).
    member _.BucketLoad = bucketLoad

    member _.BucketLoadKg = Array.sum bucketLoad

    /// Absorb up to the remaining capacity; returns what was taken (kg).
    member _.TryAbsorb(mass: float, materialIndex: int) =
        let space = Tuning.BucketCapacityKg - Array.sum bucketLoad

        if space <= 0.0 then
            0.0
        else
            let taken = min mass space
            bucketLoad.[materialIndex] <- bucketLoad.[materialIndex] + taken
            taken

    /// Pour out one tick's worth of load if the bucket is open enough.
    /// Returns (kg, materialIndex) released, or ValueNone.
    member this.DumpTick() =
        if this.BucketAngle > Tuning.DumpAngle && Array.sum bucketLoad > 1e-6 then
            // Release the heaviest material first (close enough to pouring).
            let mutable best = 0

            for i in 1..4 do
                if bucketLoad.[i] > bucketLoad.[best] then
                    best <- i

            let released = min Tuning.DumpRatePerTick bucketLoad.[best]
            bucketLoad.[best] <- bucketLoad.[best] - released
            ValueSome(struct (released, best))
        else
            ValueNone

    /// Push the payload mass into the bucket body's inertia (change-guarded).
    member _.RefreshLoadInertia() =
        let total = float32 (Array.sum bucketLoad)

        if MathF.Abs(total - lastInertiaMass) > 4.0f then
            lastInertiaMass <- total
            let shape = BepuPhysics.Collidables.Box(0.5f, 0.4f, 0.6f)
            let inertia = shape.ComputeInertia(Tuning.u17Masses.Bucket + total)
            let mutable bucketRef = simulation.Bodies.[bucket]

            if not bucketRef.Awake then
                bucketRef.Awake <- true

            bucketRef.LocalInertia <- inertia

    /// World-space velocity of the cutting edge.
    member this.CuttingEdgeVelocity =
        let bucketRef = simulation.Bodies.[bucket]
        let offset = this.BucketTipPosition - bucketRef.Pose.Position
        bucketRef.Velocity.Linear + Vector3.Cross(bucketRef.Velocity.Angular, offset)

    /// True when some cylinder is commanded hard but barely moving — the
    /// relief-valve squeal observable.
    member _.StallActive =
        let mutable stalled = false

        for i in 0..2 do
            if MathF.Abs laggedAxes.[i] > 0.4f then
                let parentRef = simulation.Bodies.[jointParents.[i]]
                let childRef = simulation.Bodies.[jointChildren.[i]]

                let axisWorld = Vector3.Transform(motorAxes.[i], parentRef.Pose.Orientation)

                let relative =
                    Vector3.Dot(childRef.Velocity.Angular - parentRef.Velocity.Angular, axisWorld)

                let angle = jointAngle jointParents.[i] jointChildren.[i] motorAxes.[i]
                let joint = joints.[i]

                let atLimit =
                    (angle >= joint.MaxAngle - 0.05f && laggedAxes.[i] > 0.0f)
                    || (angle <= joint.MinAngle + 0.05f && laggedAxes.[i] < 0.0f)

                if not atLimit && MathF.Abs relative < 0.03f then
                    stalled <- true

        stalled

    /// Drive the machine one tick. Axes in `input` are raw -1..1; shaping
    /// (deadband → curve → valve lag) happens here so replays stay identical.
    member this.Step(input: InputFrame, dt: float32, surfaceHeight: float32 -> float32 -> float32) =
        // 1. Shape inputs.
        laggedAxes.[Hydraulics.Boom] <-
            Hydraulics.lag laggedAxes.[Hydraulics.Boom] (Hydraulics.shapeAxis input.Boom) dt

        laggedAxes.[Hydraulics.Stick] <-
            Hydraulics.lag laggedAxes.[Hydraulics.Stick] (Hydraulics.shapeAxis input.Stick) dt

        laggedAxes.[Hydraulics.Bucket] <-
            Hydraulics.lag laggedAxes.[Hydraulics.Bucket] (Hydraulics.shapeAxis input.Bucket) dt

        laggedAxes.[Hydraulics.Swing] <-
            Hydraulics.lag laggedAxes.[Hydraulics.Swing] (Hydraulics.shapeAxis input.Swing) dt

        laggedAxes.[Hydraulics.TrackL] <-
            Hydraulics.lag laggedAxes.[Hydraulics.TrackL] (Hydraulics.shapeAxis input.LeftTrack) dt

        laggedAxes.[Hydraulics.TrackR] <-
            Hydraulics.lag laggedAxes.[Hydraulics.TrackR] (Hydraulics.shapeAxis input.RightTrack) dt

        // 2. Flow sharing across circuits.
        Hydraulics.allocate spec laggedAxes circuitOf qMax grantedFlow grantedScale circuitDemand

        // 3. Cylinder joints → motor commands.
        let relief circuit =
            snd spec.Circuits.[min circuit (spec.Circuits.Length - 1)] * 1.0e6f

        for i in 0..2 do
            let joint = joints.[i]
            let axis = laggedAxes.[i]
            let angle = jointAngle jointParents.[i] jointChildren.[i] motorAxes.[i]
            let mutable direction = MathF.Sign axis |> float32

            // Software stroke limits: never command past the stop.
            if (angle >= joint.MaxAngle && direction > 0.0f)
               || (angle <= joint.MinAngle && direction < 0.0f) then
                direction <- 0.0f

            let velocity =
                if direction = 0.0f then
                    0.0f
                else
                    direction
                    * Linkage.angularVelocity joint angle grantedFlow.[i] direction

            // Holding (axis 0) still gets the full cap: hydraulic check
            // valves hold a dead stick against gravity.
            let capDirection = if direction = 0.0f then 1.0f else direction
            let cap = Linkage.torqueCap joint angle capDirection (relief joint.Circuit)
            retune i motors.[i] motorAxes.[i] velocity cap

        // 4. Swing (plain motor with its own relief-ish cap + brake).
        let swingAxis = laggedAxes.[Hydraulics.Swing]

        retune
            3
            swingMotor
            Vector3.UnitY
            (swingAxis * Tuning.u17SwingMaxVel * grantedScale.[Hydraulics.Swing])
            Tuning.u17SwingTorque

        // 5. Tracks: velocity-servo tractive impulses at each track's ground
        // contact (Phase 0 verdict: BEPU has no conveyor surface velocity, so
        // drive is applied in Step, capped at the tractive limit).
        let mutable chassisRef = simulation.Bodies.[chassis]
        let pose = chassisRef.Pose
        let forward = Vector3.Transform(Vector3.UnitX, pose.Orientation)
        let forwardFlat = Vector3.Normalize(Vector3(forward.X, 0.0f, forward.Z))

        for side in 0..1 do
            let axis =
                laggedAxes.[if side = 0 then Hydraulics.TrackL else Hydraulics.TrackR]

            if MathF.Abs axis > 0.0f then
                let lateral = Vector3.Cross(Vector3.UnitY, forwardFlat)
                let offsetLocal = Vector3(0.0f, -0.25f, (if side = 0 then -0.55f else 0.55f))
                let offsetWorld = Vector3.Transform(offsetLocal, pose.Orientation)
                let contact = pose.Position + offsetWorld
                let ground = surfaceHeight contact.X contact.Z

                // Only drive when that track is actually near the ground.
                if contact.Y - ground < 0.35f then
                    if not chassisRef.Awake then
                        chassisRef.Awake <- true

                    let velocity = chassisRef.Velocity.Linear
                    let angular = chassisRef.Velocity.Angular
                    let pointVelocity = velocity + Vector3.Cross(angular, offsetWorld)
                    let currentSpeed = Vector3.Dot(pointVelocity, forwardFlat)

                    let target =
                        axis * Tuning.u17TrackMaxSpeed * grantedScale.[if side = 0 then 4 else 5]

                    let force =
                        Math.Clamp(
                            (target - currentSpeed) * Tuning.u17TrackGain,
                            -Tuning.u17TrackMaxForce,
                            Tuning.u17TrackMaxForce
                        )

                    let impulse = forwardFlat * (force * dt)
                    chassisRef.ApplyImpulse(impulse, offsetWorld)

                    // Lateral slip damping at the same point: tracks grip
                    // sideways much harder than they roll forward.
                    let lateralSpeed = Vector3.Dot(pointVelocity, lateral)

                    let lateralForce =
                        Math.Clamp(-lateralSpeed * Tuning.u17TrackGain, -Tuning.u17TrackMaxForce, Tuning.u17TrackMaxForce)

                    chassisRef.ApplyImpulse(lateral * (lateralForce * dt), offsetWorld)

    /// Write the five part poses into the snapshot.
    member this.FillSnapshot(snapshot: RenderSnapshot) =
        let parts = partsScratch
        snapshot.MachinePartCount <- parts.Length

        for i in 0 .. parts.Length - 1 do
            let bodyRef = simulation.Bodies.[parts.[i]]
            snapshot.MachinePositions.[i] <- bodyRef.Pose.Position
            snapshot.MachineOrientations.[i] <- bodyRef.Pose.Orientation

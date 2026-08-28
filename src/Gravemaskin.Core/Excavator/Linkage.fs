namespace Gravemaskin

open System

/// Analytic cylinder/linkage math. Cylinders are NOT rigid bodies (closed
/// constraint loops at 20:1 mass ratios fight iterative solvers): each
/// revolute joint is driven by an AngularAxisMotor whose velocity target and
/// torque cap come from the cylinder triangle computed here.
///
///        anchorA (on parent, Ra from pivot)
///        /‖
///       / ‖ cylinder length L(θ)
///  pivot  ‖
///       \ ‖
///        anchorB (on child, Rb from pivot)
///
/// Moment arm = Ra·Rb·sin(θcyl)/L — the standard triangle result.
[<RequireQualifiedAccess>]
module Linkage =

    let pistonArea (joint: CylinderJoint) =
        MathF.PI * joint.Bore * joint.Bore / 4.0f

    let annulusArea (joint: CylinderJoint) =
        MathF.PI * (joint.Bore * joint.Bore - joint.Rod * joint.Rod) / 4.0f

    /// Cylinder-triangle moment arm at a joint angle (m). Clamped away from
    /// zero so a degenerate pose can't divide the velocity target to ∞.
    let momentArm (joint: CylinderJoint) (angle: float32) =
        let cylAngle = angle + joint.AngleOffset

        let length =
            MathF.Sqrt(
                joint.Ra * joint.Ra + joint.Rb * joint.Rb
                - 2.0f * joint.Ra * joint.Rb * MathF.Cos cylAngle
            )
            |> max 0.05f

        joint.Ra * joint.Rb * MathF.Abs(MathF.Sin cylAngle) / length |> max 0.03f

    /// Torque cap (N·m) for a signed drive direction at reliefPressure (Pa).
    let torqueCap (joint: CylinderJoint) (angle: float32) (direction: float32) (reliefPressure: float32) =
        let extending = (direction > 0.0f) = joint.ExtendPositive

        let area =
            if extending then
                pistonArea joint
            else
                annulusArea joint

        reliefPressure * area * momentArm joint angle

    /// Angular velocity (rad/s) produced by a flow (m³/s) at this pose.
    let angularVelocity (joint: CylinderJoint) (angle: float32) (flow: float32) (direction: float32) =
        let extending = (direction > 0.0f) = joint.ExtendPositive

        let area =
            if extending then
                pistonArea joint
            else
                annulusArea joint

        flow / area / momentArm joint angle

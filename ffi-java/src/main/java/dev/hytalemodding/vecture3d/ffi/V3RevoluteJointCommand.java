package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

/**
 * A hinge between two bodies. Anchors are in meters relative to each body origin, and normalized
 * local frame rotations map the frame's z-axis to the hinge axis. The frames also define zero angle.
 * Angles and motor speed use radians and radians per second. The spring seeks targetAngle, whereas
 * the velocity motor seeks motorSpeed with at most maxMotorTorque in newton meters.
 *
 * <p>The target is within [-pi, pi], and ordered limits are within [-0.99*pi, 0.99*pi], even when
 * disabled. The target may lie outside the limits. Hertz, damping ratio and torque are nonnegative.
 * Connected bodies do not collide unless COLLIDE_CONNECTED_FLAG is set.
 */
public record V3RevoluteJointCommand(
    V3JointHandle handle,
    V3BodyHandle bodyA,
    V3BodyHandle bodyB,
    float localAnchorAX,
    float localAnchorAY,
    float localAnchorAZ,
    float localRotationAX,
    float localRotationAY,
    float localRotationAZ,
    float localRotationAW,
    float localAnchorBX,
    float localAnchorBY,
    float localAnchorBZ,
    float localRotationBX,
    float localRotationBY,
    float localRotationBZ,
    float localRotationBW,
    float targetAngle,
    float hertz,
    float dampingRatio,
    float lowerAngle,
    float upperAngle,
    float maxMotorTorque,
    float motorSpeed,
    int flags
) {
    public static final int ENABLE_SPRING_FLAG = 1;
    public static final int ENABLE_LIMIT_FLAG = 1 << 1;
    public static final int ENABLE_MOTOR_FLAG = 1 << 2;
    public static final int COLLIDE_CONNECTED_FLAG = 1 << 3;
    private static final int ALLOWED_FLAGS = ENABLE_SPRING_FLAG | ENABLE_LIMIT_FLAG
        | ENABLE_MOTOR_FLAG | COLLIDE_CONNECTED_FLAG;
    private static final float PI = (float) Math.PI;
    private static final float LIMIT_ANGLE = 0.99f * PI;

    public V3RevoluteJointCommand {
        Objects.requireNonNull(handle, "handle");
        Objects.requireNonNull(bodyA, "bodyA");
        Objects.requireNonNull(bodyB, "bodyB");
        if ((flags & ~ALLOWED_FLAGS) != 0) {
            throw new IllegalArgumentException("flags contain unsupported bits");
        }
        if (!V3BoxBodyCommand.allFinite(localAnchorAX, localAnchorAY, localAnchorAZ,
            localAnchorBX, localAnchorBY, localAnchorBZ, targetAngle, hertz, dampingRatio,
            lowerAngle, upperAngle, maxMotorTorque, motorSpeed)) {
            throw new IllegalArgumentException("joint values must be finite");
        }
        V3BoxBodyCommand.requireNormalizedQuaternion(localRotationAX, localRotationAY, localRotationAZ, localRotationAW);
        V3BoxBodyCommand.requireNormalizedQuaternion(localRotationBX, localRotationBY, localRotationBZ, localRotationBW);
        if (targetAngle < -PI || targetAngle > PI || lowerAngle < -LIMIT_ANGLE
            || upperAngle > LIMIT_ANGLE || lowerAngle > upperAngle) {
            throw new IllegalArgumentException("joint angles exceed the native bounds or are unordered");
        }
        if (hertz < 0 || dampingRatio < 0 || maxMotorTorque < 0) {
            throw new IllegalArgumentException("spring and motor limits cannot be negative");
        }
        if (bodyA.logicalId() == bodyB.logicalId()) {
            throw new IllegalArgumentException("a revolute joint requires two logical bodies");
        }
    }
}

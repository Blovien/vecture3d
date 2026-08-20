package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

public record V3DistanceJointCommand(
    V3JointHandle handle,
    V3BodyHandle bodyA,
    V3BodyHandle bodyB,
    float localAnchorAX,
    float localAnchorAY,
    float localAnchorAZ,
    float localAnchorBX,
    float localAnchorBY,
    float localAnchorBZ,
    float restLength,
    float minimumLength,
    float maximumLength,
    float hertz,
    float dampingRatio,
    float lowerSpringForce,
    float upperSpringForce,
    int flags
) {
    public static final int ENABLE_SPRING_FLAG = 1;
    public static final int ENABLE_LIMIT_FLAG = 1 << 1;

    private static final int ALLOWED_FLAGS = ENABLE_SPRING_FLAG | ENABLE_LIMIT_FLAG;

    public V3DistanceJointCommand {
        Objects.requireNonNull(handle, "handle");
        Objects.requireNonNull(bodyA, "bodyA");
        Objects.requireNonNull(bodyB, "bodyB");
        if ((flags & ~ALLOWED_FLAGS) != 0) {
            throw new IllegalArgumentException("flags contain unsupported bits");
        }
        if (!V3BoxBodyCommand.allFinite(
            localAnchorAX,
            localAnchorAY,
            localAnchorAZ,
            localAnchorBX,
            localAnchorBY,
            localAnchorBZ,
            restLength,
            minimumLength,
            maximumLength,
            hertz,
            dampingRatio,
            lowerSpringForce,
            upperSpringForce
        )) {
            throw new IllegalArgumentException("joint values must be finite");
        }
        if (restLength <= 0.0f || minimumLength <= 0.0f
            || maximumLength < minimumLength
            || restLength < minimumLength
            || restLength > maximumLength) {
            throw new IllegalArgumentException("joint lengths are inconsistent");
        }
        if (hertz < 0.0f || dampingRatio < 0.0f) {
            throw new IllegalArgumentException("spring damping values cannot be negative");
        }
        if (lowerSpringForce > 0.0f || upperSpringForce < 0.0f) {
            throw new IllegalArgumentException("spring force limits are inconsistent");
        }
        if (bodyA.logicalId() == bodyB.logicalId()) {
            throw new IllegalArgumentException("a distance joint requires two logical bodies");
        }
    }
}

package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

/**
 * Pose, velocity and damping for BlockGrid and sphere body creation.
 *
 * <p>The handle carries the logical identity and generation the new body takes, in the same scheme
 * as every other body operation, so removals, wrenches, kinematic targets and queries address it
 * afterwards.
 */
public record V3BodyDefinition(
    V3BodyHandle handle,
    V3BoxBodyCommand.Kind kind,
    double positionX,
    double positionY,
    double positionZ,
    float rotationX,
    float rotationY,
    float rotationZ,
    float rotationW,
    float linearVelocityX,
    float linearVelocityY,
    float linearVelocityZ,
    float angularVelocityX,
    float angularVelocityY,
    float angularVelocityZ,
    float linearDamping,
    float angularDamping,
    int flags
) {
    private static final int ALLOWED_FLAGS = V3BoxBodyCommand.ENABLE_SLEEP_FLAG
        | V3BoxBodyCommand.INITIAL_AWAKE_FLAG
        | V3BoxBodyCommand.DISABLE_COLLISION_FLAG
        | V3BoxBodyCommand.BULLET_FLAG;
    private static final float MAX_DAMPING = 10.0f;

    public V3BodyDefinition {
        Objects.requireNonNull(handle, "handle");
        Objects.requireNonNull(kind, "kind");
        if ((flags & ~ALLOWED_FLAGS) != 0) {
            throw new IllegalArgumentException("flags contain unsupported bits");
        }
        if ((flags & V3BoxBodyCommand.BULLET_FLAG) != 0 && kind != V3BoxBodyCommand.Kind.DYNAMIC) {
            throw new IllegalArgumentException("bullet bodies must be dynamic");
        }
        if (!Double.isFinite(positionX) || !Double.isFinite(positionY) || !Double.isFinite(positionZ)
            || !V3BoxBodyCommand.allFinite(
                linearVelocityX,
                linearVelocityY,
                linearVelocityZ,
                angularVelocityX,
                angularVelocityY,
                angularVelocityZ,
                linearDamping,
                angularDamping
            )) {
            throw new IllegalArgumentException("body values must be finite");
        }
        V3BoxBodyCommand.requireNormalizedQuaternion(rotationX, rotationY, rotationZ, rotationW);
        if (linearDamping < 0.0f || linearDamping > MAX_DAMPING
            || angularDamping < 0.0f || angularDamping > MAX_DAMPING) {
            throw new IllegalArgumentException("damping must be between zero and ten");
        }
    }

    /** A body at rest with an identity rotation and no damping. */
    public static V3BodyDefinition at(
        V3BodyHandle handle,
        V3BoxBodyCommand.Kind kind,
        double positionX,
        double positionY,
        double positionZ,
        int flags
    ) {
        return new V3BodyDefinition(
            handle,
            kind,
            positionX,
            positionY,
            positionZ,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            flags
        );
    }
}

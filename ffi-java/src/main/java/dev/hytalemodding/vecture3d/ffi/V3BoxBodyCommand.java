package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

public record V3BoxBodyCommand(
    V3BodyHandle handle,
    Kind kind,
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
    float halfExtentX,
    float halfExtentY,
    float halfExtentZ,
    float density,
    float friction,
    float linearDamping,
    float angularDamping,
    int flags
) {
    public static final int ENABLE_SLEEP_FLAG = 1;
    public static final int INITIAL_AWAKE_FLAG = 1 << 1;
    public static final int DISABLE_COLLISION_FLAG = 1 << 2;

    private static final int ALLOWED_FLAGS = ENABLE_SLEEP_FLAG | INITIAL_AWAKE_FLAG | DISABLE_COLLISION_FLAG;
    private static final float MAX_DAMPING = 10.0f;

    public V3BoxBodyCommand {
        Objects.requireNonNull(handle, "handle");
        Objects.requireNonNull(kind, "kind");
        if ((flags & ~ALLOWED_FLAGS) != 0) {
            throw new IllegalArgumentException("flags contain unsupported bits");
        }
        if (!Double.isFinite(positionX) || !Double.isFinite(positionY) || !Double.isFinite(positionZ)
            || !allFinite(
                linearVelocityX,
                linearVelocityY,
                linearVelocityZ,
                angularVelocityX,
                angularVelocityY,
                angularVelocityZ,
                halfExtentX,
                halfExtentY,
                halfExtentZ,
                density,
                friction,
                linearDamping,
                angularDamping
            )) {
            throw new IllegalArgumentException("body values must be finite");
        }
        requireNormalizedQuaternion(rotationX, rotationY, rotationZ, rotationW);
        if (halfExtentX <= 0.0f || halfExtentY <= 0.0f || halfExtentZ <= 0.0f) {
            throw new IllegalArgumentException("half extents must be positive");
        }
        if ((kind == Kind.DYNAMIC && density <= 0.0f)
                || (kind != Kind.DYNAMIC && density != 0.0f)) {
            throw new IllegalArgumentException("density does not match body kind");
        }
        if (friction < 0.0f || friction > 1.0f) {
            throw new IllegalArgumentException("friction must be between zero and one");
        }
        if (linearDamping < 0.0f || linearDamping > MAX_DAMPING
            || angularDamping < 0.0f || angularDamping > MAX_DAMPING) {
            throw new IllegalArgumentException("damping must be between zero and ten");
        }
    }

    static void requireNormalizedQuaternion(float x, float y, float z, float w) {
        if (!allFinite(x, y, z, w)) {
            throw new IllegalArgumentException("rotation must be finite");
        }
        float lengthSquared = x * x + y * y + z * z + w * w;
        float tolerance = 20.0f * Math.ulp(1.0f);
        if (!Float.isFinite(lengthSquared)
            || lengthSquared <= 1.0f - tolerance
            || lengthSquared >= 1.0f + tolerance) {
            throw new IllegalArgumentException("rotation must be normalized");
        }
    }

    static boolean allFinite(float... values) {
        for (float value : values) {
            if (!Float.isFinite(value)) {
                return false;
            }
        }
        return true;
    }

    public enum Kind {
        STATIC(0),
        KINEMATIC(1),
        DYNAMIC(2);

        private final int nativeValue;

        Kind(int nativeValue) {
            this.nativeValue = nativeValue;
        }

        int nativeValue() {
            return nativeValue;
        }
    }
}

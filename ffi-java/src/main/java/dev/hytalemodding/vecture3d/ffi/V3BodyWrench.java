package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

/**
 * One force and torque command reapplied before every admitted fixed step.
 */
public record V3BodyWrench(
    V3BodyHandle handle,
    float forceX,
    float forceY,
    float forceZ,
    float torqueX,
    float torqueY,
    float torqueZ,
    boolean wake
) {
    private static final float MAX_FORCE = 1_000_000.0f;
    private static final float MAX_TORQUE = 1_000_000.0f;

    public V3BodyWrench {
        Objects.requireNonNull(handle, "handle");
        if (!V3BoxBodyCommand.allFinite(forceX, forceY, forceZ, torqueX, torqueY, torqueZ)) {
            throw new IllegalArgumentException("wrench values must be finite");
        }
        if (!withinMagnitude(forceX, forceY, forceZ, MAX_FORCE)
            || !withinMagnitude(torqueX, torqueY, torqueZ, MAX_TORQUE)) {
            throw new IllegalArgumentException("wrench exceeds the native magnitude limit");
        }
    }

    private static boolean withinMagnitude(float x, float y, float z, float maximum) {
        double squaredMagnitude = (double) x * x + (double) y * y + (double) z * z;
        return squaredMagnitude <= (double) maximum * maximum;
    }
}

package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

/**
 * One target pose applied over the complete admitted fixed-step batch.
 */
public record V3KinematicTarget(
    V3BodyHandle handle,
    double positionX,
    double positionY,
    double positionZ,
    float rotationX,
    float rotationY,
    float rotationZ,
    float rotationW,
    boolean wake
) {
    public V3KinematicTarget {
        Objects.requireNonNull(handle, "handle");
        if (!Double.isFinite(positionX) || !Double.isFinite(positionY) || !Double.isFinite(positionZ)) {
            throw new IllegalArgumentException("target position must be finite");
        }
        V3BoxBodyCommand.requireNormalizedQuaternion(rotationX, rotationY, rotationZ, rotationW);
    }
}

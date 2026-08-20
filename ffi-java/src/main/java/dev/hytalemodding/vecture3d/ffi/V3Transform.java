package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

/**
 * One post-frame dynamic-body transform with double-precision world position.
 */
public record V3Transform(
    V3BodyHandle handle,
    boolean awake,
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
    float localCenterX,
    float localCenterY,
    float localCenterZ
) {
    public V3Transform {
        Objects.requireNonNull(handle, "handle");
        if (!Double.isFinite(positionX) || !Double.isFinite(positionY) || !Double.isFinite(positionZ)
            || !V3BoxBodyCommand.allFinite(
                linearVelocityX,
                linearVelocityY,
                linearVelocityZ,
                angularVelocityX,
                angularVelocityY,
                angularVelocityZ,
                localCenterX,
                localCenterY,
                localCenterZ
            )) {
            throw new IllegalArgumentException("transform values must be finite");
        }
        V3BoxBodyCommand.requireNormalizedQuaternion(rotationX, rotationY, rotationZ, rotationW);
    }
}

package dev.hytalemodding.vecture3d.ffi;

/**
 * Explicit mass, local center and symmetric inertia tensor.
 */
public record V3MassProperties(
    float mass,
    float centerX,
    float centerY,
    float centerZ,
    float inertiaXX,
    float inertiaYY,
    float inertiaZZ,
    float inertiaXY,
    float inertiaXZ,
    float inertiaYZ
) {
    public V3MassProperties {
        float leadingMinor = inertiaXX * inertiaYY - inertiaXY * inertiaXY;
        float determinant = inertiaXX * inertiaYY * inertiaZZ
            + 2.0f * inertiaXY * inertiaXZ * inertiaYZ
            - inertiaXX * inertiaYZ * inertiaYZ
            - inertiaYY * inertiaXZ * inertiaXZ
            - inertiaZZ * inertiaXY * inertiaXY;
        if (!V3BoxBodyCommand.allFinite(
            mass,
            centerX,
            centerY,
            centerZ,
            inertiaXX,
            inertiaYY,
            inertiaZZ,
            inertiaXY,
            inertiaXZ,
            inertiaYZ,
            leadingMinor,
            determinant
        ) || mass <= 0.0f || inertiaXX <= 0.0f || leadingMinor <= 0.0f || determinant <= 0.0f) {
            throw new IllegalArgumentException("mass and inertia must be finite and positive definite");
        }
    }
}

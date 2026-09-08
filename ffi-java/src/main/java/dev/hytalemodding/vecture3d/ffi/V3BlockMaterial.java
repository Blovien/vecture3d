package dev.hytalemodding.vecture3d.ffi;

/**
 * One BlockGrid surface material.
 *
 * <p>Friction, restitution and the material ID reach Box3D; the material ID is 64 bits wide on both
 * sides, so nothing is truncated. Box3D takes density per shape rather than per material, so the
 * cook uses the density of material zero for every shape the cooked data backs when that density is
 * positive and 1.0 otherwise, and ignores the density of every other material. Bond strength,
 * compressive factor and flags are accepted and ignored, reserved for fracture.
 */
public record V3BlockMaterial(
    long materialId,
    float density,
    float friction,
    float restitution,
    float bondStrength,
    float compressiveFactor,
    int flags
) {
    public V3BlockMaterial {
        if (!V3BoxBodyCommand.allFinite(density, friction, restitution, bondStrength, compressiveFactor)) {
            throw new IllegalArgumentException("material values must be finite");
        }
        if (density < 0.0f) {
            throw new IllegalArgumentException("density cannot be negative");
        }
        if (friction < 0.0f || friction > 1.0f) {
            throw new IllegalArgumentException("friction must be between zero and one");
        }
        if (restitution < 0.0f || restitution > 1.0f) {
            throw new IllegalArgumentException("restitution must be between zero and one");
        }
    }

    /** A material with no fracture properties, which this slice reserves and ignores. */
    public static V3BlockMaterial of(long materialId, float density, float friction, float restitution) {
        return new V3BlockMaterial(materialId, density, friction, restitution, 0.0f, 0.0f, 0);
    }
}

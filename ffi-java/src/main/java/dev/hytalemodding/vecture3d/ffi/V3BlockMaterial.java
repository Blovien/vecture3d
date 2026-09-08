package dev.hytalemodding.vecture3d.ffi;

/**
 * One BlockGrid surface material. Box3D uses friction, restitution, and the full 64-bit material ID.
 *
 * <p>Shape density comes from material zero: its density is used when positive, otherwise 1.0 is used.
 * Other materials' densities are ignored. Bond strength, compressive factor, and flags are reserved
 * for fracture and currently ignored.
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

    /**
     * Creates a material with fracture properties and flags set to zero.
     */
    public static V3BlockMaterial of(long materialId, float density, float friction, float restitution) {
        return new V3BlockMaterial(materialId, density, friction, restitution, 0.0f, 0.0f, 0);
    }
}

package dev.hytalemodding.vecture3d.ffi;

/**
 * One collision box in the cell selected by {@code ownerCellIndex} in the cook's cell list.
 * Boxes may be supplied in any order and are grouped by cell.
 *
 * <p>The center and half extents are cell local. Both center minus half extent and center plus
 * half extent must lie within [0, 1] on every axis. A full cube has center {@code (0.5, 0.5, 0.5)}
 * and half extents {@code (0.5, 0.5, 0.5)}.
 *
 * <p>The feature ID is reserved for fracture and ignored. Reported identity uses the cell's feature ID.
 */
public record V3BlockBox(
    int ownerCellIndex,
    int materialIndex,
    long featureId,
    float centerX,
    float centerY,
    float centerZ,
    float halfExtentX,
    float halfExtentY,
    float halfExtentZ
) {
    public V3BlockBox {
        if (ownerCellIndex < 0) {
            throw new IllegalArgumentException("ownerCellIndex cannot be negative");
        }
        if (materialIndex < 0) {
            throw new IllegalArgumentException("materialIndex cannot be negative");
        }
        if (!V3BoxBodyCommand.allFinite(centerX, centerY, centerZ, halfExtentX, halfExtentY, halfExtentZ)) {
            throw new IllegalArgumentException("box values must be finite");
        }
        if (halfExtentX <= 0.0f || halfExtentY <= 0.0f || halfExtentZ <= 0.0f) {
            throw new IllegalArgumentException("half extents must be positive");
        }
        requireInsideCell(centerX, halfExtentX);
        requireInsideCell(centerY, halfExtentY);
        requireInsideCell(centerZ, halfExtentZ);
    }

    /** A box that fills the cell its owner index names. */
    public static V3BlockBox fullCube(int ownerCellIndex, int materialIndex, long featureId) {
        return new V3BlockBox(ownerCellIndex, materialIndex, featureId, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f);
    }

    private static void requireInsideCell(float center, float halfExtent) {
        if (center - halfExtent < 0.0f || center + halfExtent > 1.0f) {
            throw new IllegalArgumentException("box bounds must stay inside the unit cell");
        }
    }
}

package dev.hytalemodding.vecture3d.ffi;

/**
 * One collision box inside the cell its owner index names.
 *
 * <p>The owner index addresses the cook's cell list. Boxes may be supplied in any order; the cook
 * groups them by owner. The center and the half extents are cell-local: the box covers center minus
 * half extent to center plus half extent, and both bounds must stay inside the unit cube of the
 * owning cell, so a full cube is a center of {@code (0.5, 0.5, 0.5)} with half extents of
 * {@code (0.5, 0.5, 0.5)}. The feature ID is accepted and ignored, reserved for per-box fracture
 * identity; a cell reports its own feature ID.
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

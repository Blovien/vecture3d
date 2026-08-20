package dev.hytalemodding.vecture3d.ffi;

/**
 * Local terrain collider and its stable feature identity.
 */
public record V3TerrainBox(
    long featureId,
    float centerX,
    float centerY,
    float centerZ,
    float halfExtentX,
    float halfExtentY,
    float halfExtentZ,
    float friction
) {
    public V3TerrainBox {
        if (featureId == 0) {
            throw new IllegalArgumentException("featureId cannot be zero");
        }
        if (!V3BoxBodyCommand.allFinite(
            centerX,
            centerY,
            centerZ,
            halfExtentX,
            halfExtentY,
            halfExtentZ,
            friction
        )) {
            throw new IllegalArgumentException("terrain box values must be finite");
        }
        if (halfExtentX <= 0.0f || halfExtentY <= 0.0f || halfExtentZ <= 0.0f) {
            throw new IllegalArgumentException("half extents must be positive");
        }
        if (friction < 0.0f || friction > 1.0f) {
            throw new IllegalArgumentException("friction must be between zero and one");
        }
    }
}

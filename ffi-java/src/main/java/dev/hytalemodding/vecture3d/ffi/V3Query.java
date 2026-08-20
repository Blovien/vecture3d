package dev.hytalemodding.vecture3d.ffi;

public record V3Query(
    long queryId,
    double originX,
    double originY,
    double originZ,
    float halfExtentX,
    float halfExtentY,
    float halfExtentZ,
    float translationX,
    float translationY,
    float translationZ
) {
    public V3Query {
        if (queryId == 0) {
            throw new IllegalArgumentException("queryId cannot be zero");
        }
        if (!Double.isFinite(originX) || !Double.isFinite(originY) || !Double.isFinite(originZ)
            || !V3BoxBodyCommand.allFinite(
                halfExtentX,
                halfExtentY,
                halfExtentZ,
                translationX,
                translationY,
                translationZ
            )) {
            throw new IllegalArgumentException("query values must be finite");
        }
        if (halfExtentX <= 0.0f || halfExtentY <= 0.0f || halfExtentZ <= 0.0f) {
            throw new IllegalArgumentException("query half extents must be positive");
        }
    }
}

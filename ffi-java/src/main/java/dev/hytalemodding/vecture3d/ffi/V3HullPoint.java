package dev.hytalemodding.vecture3d.ffi;

/**
 * One local point used to construct a native convex hull.
 */
public record V3HullPoint(float x, float y, float z) {
    public V3HullPoint {
        if (!Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)) {
            throw new IllegalArgumentException("hull coordinates must be finite");
        }
    }
}

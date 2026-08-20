package dev.hytalemodding.vecture3d.ffi;

/**
 * A stable logical body identity where negative Java IDs retain valid native unsigned values.
 */
public record V3BodyHandle(long logicalId, int generation) {
    public V3BodyHandle {
        if (logicalId == 0) {
            throw new IllegalArgumentException("logicalId cannot be zero");
        }
        if (generation <= 0) {
            throw new IllegalArgumentException("generation must be positive");
        }
    }
}

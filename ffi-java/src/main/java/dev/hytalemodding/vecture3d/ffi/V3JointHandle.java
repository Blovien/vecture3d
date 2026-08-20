package dev.hytalemodding.vecture3d.ffi;

/**
 * A stable logical joint identity where negative Java IDs retain valid native unsigned values.
 */
public record V3JointHandle(long logicalId, int generation) {
    public V3JointHandle {
        if (logicalId == 0) {
            throw new IllegalArgumentException("logicalId cannot be zero");
        }
        if (generation <= 0) {
            throw new IllegalArgumentException("generation must be positive");
        }
    }
}

package dev.hytalemodding.vecture3d.ffi;

/**
 * One occupied BlockGrid cell.
 *
 * <p>The coordinates are integer cell coordinates in BlockGrid-local space, so the cell spans
 * {@code [x, x + 1]} on that axis. The feature ID becomes the block's user data and does not have to
 * be unique. The material index selects the cell material and must address the cook's material list.
 */
public record V3BlockCell(int x, int y, int z, int materialIndex, long featureId) {
    public V3BlockCell {
        if (materialIndex < 0) {
            throw new IllegalArgumentException("materialIndex cannot be negative");
        }
    }
}

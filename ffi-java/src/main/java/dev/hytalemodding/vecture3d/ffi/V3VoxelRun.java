package dev.hytalemodding.vecture3d.ffi;

/**
 * A local axis-aligned voxel box packed into bits.
 */
public record V3VoxelRun(int packed) {
    private static final int FIELD_MASK = 0x1f;
    private static final int USED_BITS_MASK = 0x3fff_ffff;
    private static final int SECTION_SIZE = 32;

    public V3VoxelRun {
        if ((packed & ~USED_BITS_MASK) != 0) {
            throw new IllegalArgumentException("packed uses only the low thirty bits");
        }
        requireInsideSection(field(packed, 0), field(packed, 15) + 1, "x");
        requireInsideSection(field(packed, 5), field(packed, 20) + 1, "y");
        requireInsideSection(field(packed, 10), field(packed, 25) + 1, "z");
    }

    private static int field(int packed, int shift) {
        return packed >>> shift & FIELD_MASK;
    }

    private static void requireInsideSection(int start, int size, String axis) {
        if (start + size > SECTION_SIZE) {
            throw new IllegalArgumentException("voxel run exceeds its section along " + axis);
        }
    }
}

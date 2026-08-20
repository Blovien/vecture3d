package dev.hytalemodding.vecture3d.ffi;

import java.util.List;
import java.util.Objects;

/**
 * One static terrain section with immutable collision children.
 */
public record V3TerrainSectionCommand(
    V3BodyHandle handle,
    double originX,
    double originY,
    double originZ,
    List<V3VoxelRun> voxelRuns,
    List<V3TerrainBox> detailBoxes
) {
    private static final int MAX_CHILDREN = 65_534;

    public V3TerrainSectionCommand {
        Objects.requireNonNull(handle, "handle");
        Objects.requireNonNull(voxelRuns, "voxelRuns");
        Objects.requireNonNull(detailBoxes, "detailBoxes");
        if (!Double.isFinite(originX) || !Double.isFinite(originY) || !Double.isFinite(originZ)) {
            throw new IllegalArgumentException("terrain origin must be finite");
        }
        if (voxelRuns.isEmpty() && detailBoxes.isEmpty()) {
            throw new IllegalArgumentException("terrain section requires at least one child");
        }
        if (voxelRuns.size() > MAX_CHILDREN || detailBoxes.size() > MAX_CHILDREN - voxelRuns.size()) {
            throw new IllegalArgumentException("terrain section exceeds the child limit");
        }
        voxelRuns = List.copyOf(voxelRuns);
        detailBoxes = List.copyOf(detailBoxes);
    }
}

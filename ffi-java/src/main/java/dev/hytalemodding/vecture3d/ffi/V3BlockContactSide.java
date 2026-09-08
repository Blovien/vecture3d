package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

/** One contact participant, with logical cell identity when it is a BlockGrid. */
public record V3BlockContactSide(
    V3BodyHandle body,
    boolean blockGrid,
    int cellX,
    int cellY,
    int cellZ,
    int cellBoxIndex,
    int materialIndex,
    long userMaterialId,
    long userData
) {
    public V3BlockContactSide {
        Objects.requireNonNull(body, "body");
        if (cellBoxIndex < 0 || materialIndex < 0) {
            throw new IllegalArgumentException("contact identity indices must be non-negative");
        }
        if (!blockGrid && (cellX != 0 || cellY != 0 || cellZ != 0 || cellBoxIndex != 0
            || materialIndex != 0 || userMaterialId != 0L || userData != 0L)) {
            throw new IllegalArgumentException("a non-grid contact side has no cell identity");
        }
    }
}

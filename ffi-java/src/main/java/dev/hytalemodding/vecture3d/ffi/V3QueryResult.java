package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

public record V3QueryResult(
    long queryId,
    Status status,
    long hitLogicalId,
    float fraction
) {
    public V3QueryResult {
        Objects.requireNonNull(status, "status");
        if (queryId == 0) {
            throw new IllegalArgumentException("queryId cannot be zero");
        }
        if (!Float.isFinite(fraction)) {
            throw new IllegalArgumentException("fraction must be finite");
        }
        switch (status) {
            case NO_HIT -> {
                if (hitLogicalId != 0 || fraction != 1.0f) {
                    throw new IllegalArgumentException("a no-hit result requires no obstacle and fraction one");
                }
            }
            case IMMEDIATE_BLOCK -> {
                if (hitLogicalId == 0 || fraction != 0.0f) {
                    throw new IllegalArgumentException("an immediate block requires an obstacle and fraction zero");
                }
            }
            case CAST_HIT -> {
                if (hitLogicalId == 0 || fraction <= 0.0f || fraction > 1.0f) {
                    throw new IllegalArgumentException("a cast hit requires an obstacle and a positive bounded fraction");
                }
            }
        }
    }

    public enum Status {
        NO_HIT(0),
        IMMEDIATE_BLOCK(1),
        CAST_HIT(2);

        private final int nativeValue;

        Status(int nativeValue) {
            this.nativeValue = nativeValue;
        }

        int nativeValue() {
            return nativeValue;
        }

        static Status fromNative(int value) {
            return switch (value) {
                case 0 -> NO_HIT;
                case 1 -> IMMEDIATE_BLOCK;
                case 2 -> CAST_HIT;
                default -> throw new IllegalStateException("unknown native query status " + value);
            };
        }
    }
}

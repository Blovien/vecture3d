package dev.hytalemodding.vecture3d.ffi;

/**
 * Native frame counts and timings. All timing values are milliseconds.
 */
public record V3StepStats(
    int outputCount,
    int bodyCount,
    int shapeCount,
    int contactCount,
    float stepMilliseconds,
    int mutationBatchCount,
    int createdBodyCount,
    int destroyedBodyCount,
    int queryOutputCount,
    int queryCount,
    int fixedStepCount,
    int jointCount,
    float pairMilliseconds,
    float collideMilliseconds,
    float solveMilliseconds,
    int staticTreeHeight,
    int dynamicTreeHeight,
    int satCallCount,
    int satCacheHitCount,
    int graphOverflowConstraintCount,
    int heapMovePairCount
) {
    private static final int MAX_BODIES = 4_096;
    private static final int MAX_JOINTS = 4_096;
    private static final int MAX_QUERIES = 1_024;

    public V3StepStats {
        requireNonNegative(
            outputCount,
            bodyCount,
            shapeCount,
            contactCount,
            mutationBatchCount,
            createdBodyCount,
            destroyedBodyCount,
            queryOutputCount,
            queryCount,
            jointCount,
            staticTreeHeight,
            dynamicTreeHeight,
            satCallCount,
            satCacheHitCount,
            graphOverflowConstraintCount,
            heapMovePairCount
        );
        if (fixedStepCount < 0 || fixedStepCount > 4) {
            throw new IllegalArgumentException("fixedStepCount must be between zero and four");
        }
        if (outputCount > bodyCount) {
            throw new IllegalArgumentException("outputCount cannot exceed bodyCount");
        }
        if (bodyCount > MAX_BODIES || jointCount > MAX_JOINTS) {
            throw new IllegalArgumentException("body or joint count exceeds the native limit");
        }
        if (queryCount > MAX_QUERIES) {
            throw new IllegalArgumentException("queryCount exceeds the native limit");
        }
        if (queryOutputCount != queryCount) {
            throw new IllegalArgumentException("query output count must match query count");
        }
        requireNonNegativeFinite(stepMilliseconds, "stepMilliseconds");
        requireNonNegativeFinite(pairMilliseconds, "pairMilliseconds");
        requireNonNegativeFinite(collideMilliseconds, "collideMilliseconds");
        requireNonNegativeFinite(solveMilliseconds, "solveMilliseconds");
    }

    private static void requireNonNegative(int... values) {
        for (int value : values) {
            if (value < 0) {
                throw new IllegalArgumentException("native counts must be non-negative");
            }
        }
    }

    private static void requireNonNegativeFinite(float value, String name) {
        if (!Float.isFinite(value) || value < 0.0f) {
            throw new IllegalArgumentException(name + " must be finite and non-negative");
        }
    }
}

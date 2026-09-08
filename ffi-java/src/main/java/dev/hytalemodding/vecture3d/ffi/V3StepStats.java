package dev.hytalemodding.vecture3d.ffi;

/**
 * Native frame counts and timings. All timing values are milliseconds summed over the call.
 * Event loss is a lower bound: truncation also covers transitions that native recovery cannot count.
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
    int heapMovePairCount,
    long blockGridCandidateHitboxPairCount,
    long blockGridTouchingPairCount,
    long blockGridContactCount,
    long blockGridProjectileSweepCount,
    long blockGridCapExhaustionCount,
    long blockGridReplacementPublishedCount,
    long blockGridScratchPeakBytes,
    long blockGridContactReductionCount,
    int blockContactEventCount,
    long blockContactEventDroppedCount,
    boolean blockContactEventsTruncated
) {
    private static final int MAX_BODIES = 4_096;
    private static final int MAX_JOINTS = 4_096;
    private static final int MAX_QUERIES = 1_024;

    /** Existing construction defaults the appended event fields to zero. */
    public V3StepStats(
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
        int heapMovePairCount,
        long blockGridCandidateHitboxPairCount,
        long blockGridTouchingPairCount,
        long blockGridContactCount,
        long blockGridProjectileSweepCount,
        long blockGridCapExhaustionCount,
        long blockGridReplacementPublishedCount,
        long blockGridScratchPeakBytes,
        long blockGridContactReductionCount
    ) {
        this(
            outputCount,
            bodyCount,
            shapeCount,
            contactCount,
            stepMilliseconds,
            mutationBatchCount,
            createdBodyCount,
            destroyedBodyCount,
            queryOutputCount,
            queryCount,
            fixedStepCount,
            jointCount,
            pairMilliseconds,
            collideMilliseconds,
            solveMilliseconds,
            staticTreeHeight,
            dynamicTreeHeight,
            satCallCount,
            satCacheHitCount,
            graphOverflowConstraintCount,
            heapMovePairCount,
            blockGridCandidateHitboxPairCount,
            blockGridTouchingPairCount,
            blockGridContactCount,
            blockGridProjectileSweepCount,
            blockGridCapExhaustionCount,
            blockGridReplacementPublishedCount,
            blockGridScratchPeakBytes,
            blockGridContactReductionCount,
            0,
            0L,
            false
        );
    }

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
            heapMovePairCount,
            blockContactEventCount
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
        if (blockContactEventCount > 3_072 || blockContactEventDroppedCount < 0L
            || blockContactEventDroppedCount > 0xffff_ffffL) {
            throw new IllegalArgumentException("contact event counts exceed the native bounds");
        }
        requireNonNegativeFinite(stepMilliseconds, "stepMilliseconds");
        requireNonNegativeFinite(pairMilliseconds, "pairMilliseconds");
        requireNonNegativeFinite(collideMilliseconds, "collideMilliseconds");
        requireNonNegativeFinite(solveMilliseconds, "solveMilliseconds");
        // BlockGrid pair counters. Each count is the sum over the fixed steps of one call; the scratch
        // figure is the largest single step of that call rather than a sum.
        requireNonNegative(
            blockGridCandidateHitboxPairCount,
            blockGridTouchingPairCount,
            blockGridContactCount,
            blockGridProjectileSweepCount,
            blockGridCapExhaustionCount,
            blockGridReplacementPublishedCount,
            blockGridScratchPeakBytes,
            blockGridContactReductionCount
        );
    }

    private static void requireNonNegative(long... values) {
        for (long value : values) {
            if (value < 0) {
                throw new IllegalArgumentException("native counts must be non-negative");
            }
        }
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

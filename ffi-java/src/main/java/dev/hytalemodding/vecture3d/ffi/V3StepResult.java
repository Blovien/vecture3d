package dev.hytalemodding.vecture3d.ffi;

import java.util.List;
import java.util.Objects;

/**
 * One detached immutable copy of the native frame outputs.
 */
public record V3StepResult(
    List<V3Transform> transforms,
    List<V3QueryResult> queryResults,
    V3StepStats stats
) {
    public V3StepResult {
        Objects.requireNonNull(transforms, "transforms");
        Objects.requireNonNull(queryResults, "queryResults");
        Objects.requireNonNull(stats, "stats");
        transforms = List.copyOf(transforms);
        queryResults = List.copyOf(queryResults);
        if (transforms.size() != stats.outputCount()) {
            throw new IllegalArgumentException("transform count does not match stats");
        }
        if (queryResults.size() != stats.queryOutputCount()) {
            throw new IllegalArgumentException("query result count does not match stats");
        }
    }
}

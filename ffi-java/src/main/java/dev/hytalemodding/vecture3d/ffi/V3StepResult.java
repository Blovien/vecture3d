package dev.hytalemodding.vecture3d.ffi;

import java.util.List;
import java.util.Objects;

/**
 * One detached immutable copy of the native frame outputs. Contact events are ordered by fixed
 * step, then END, BEGIN and HIT. Check the stats for truncation before treating the batch as complete.
 */
public record V3StepResult(
    List<V3Transform> transforms,
    List<V3QueryResult> queryResults,
    V3StepStats stats,
    List<V3BlockContactEvent> blockContactEvents
) {
    public V3StepResult(List<V3Transform> transforms, List<V3QueryResult> queryResults, V3StepStats stats) {
        this(transforms, queryResults, stats, List.of());
    }

    public V3StepResult {
        Objects.requireNonNull(transforms, "transforms");
        Objects.requireNonNull(queryResults, "queryResults");
        Objects.requireNonNull(stats, "stats");
        Objects.requireNonNull(blockContactEvents, "blockContactEvents");
        transforms = List.copyOf(transforms);
        queryResults = List.copyOf(queryResults);
        blockContactEvents = List.copyOf(blockContactEvents);
        if (blockContactEvents.size() != stats.blockContactEventCount()) {
            throw new IllegalArgumentException("contact event count does not match stats");
        }
        for (V3BlockContactEvent event : blockContactEvents) {
            if (event.fixedStepIndex() >= stats.fixedStepCount()) {
                throw new IllegalArgumentException("contact event fixed step is outside the frame");
            }
        }
        if (transforms.size() != stats.outputCount()) {
            throw new IllegalArgumentException("transform count does not match stats");
        }
        if (queryResults.size() != stats.queryOutputCount()) {
            throw new IllegalArgumentException("query result count does not match stats");
        }
    }
}

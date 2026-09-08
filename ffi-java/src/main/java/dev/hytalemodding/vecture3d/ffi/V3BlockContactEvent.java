package dev.hytalemodding.vecture3d.ffi;

import java.util.Objects;

/**
 * A detached contact transition. The normal points from side A to side B, and the impulse is the
 * native normal impulse along that direction. Positions use the ABI's float world coordinates.
 * End events retain the last known payload, including a body generation that may no longer exist.
 */
public record V3BlockContactEvent(
    Kind kind,
    int fixedStepIndex,
    V3BlockContactSide sideA,
    V3BlockContactSide sideB,
    float pointX,
    float pointY,
    float pointZ,
    float normalX,
    float normalY,
    float normalZ,
    float impulseX,
    float impulseY,
    float impulseZ,
    float relativeNormalSpeed
) {
    public V3BlockContactEvent {
        Objects.requireNonNull(kind, "kind");
        Objects.requireNonNull(sideA, "sideA");
        Objects.requireNonNull(sideB, "sideB");
        if (fixedStepIndex < 0 || fixedStepIndex >= 4) {
            throw new IllegalArgumentException("fixedStepIndex must be between zero and three");
        }
        if (!sideA.blockGrid() && !sideB.blockGrid()) {
            throw new IllegalArgumentException("a BlockGrid contact requires at least one grid side");
        }
        if (!V3BoxBodyCommand.allFinite(
            pointX, pointY, pointZ, normalX, normalY, normalZ,
            impulseX, impulseY, impulseZ, relativeNormalSpeed
        ) || relativeNormalSpeed < 0.0f) {
            throw new IllegalArgumentException("contact values must be finite and approach speed non-negative");
        }
    }

    public enum Kind {
        BEGIN, END, HIT;

        static Kind fromNative(int flags) {
            return switch (flags) {
                case 1 -> BEGIN;
                case 2 -> END;
                case 4 -> HIT;
                default -> throw new IllegalStateException("unknown native contact kind " + flags);
            };
        }
    }
}

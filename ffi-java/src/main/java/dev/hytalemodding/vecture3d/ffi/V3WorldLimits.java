package dev.hytalemodding.vecture3d.ffi;

/**
 * World motion limits: maximum linear speed in meters per second, maximum angular speed in
 * radians per second, and the Projectile sweep candidate cap. Zero selects the engine default
 * for each speed limit. A zero candidate cap means unlimited. Sleeping is enabled by default.
 *
 * <p>Speed limits alone do not guarantee collision safety. BlockGrid speculative admission predicts
 * free motion over the current step. Later contact or joint impulses can change that motion, and
 * rotational paths are not certified. Test the chosen speeds and timestep against the game's geometry.
 *
 * <p>The bullet flag enables the native Projectile sweep for dynamic bodies against eligible geometry,
 * subject to the candidate cap. Bullet versus bullet sweeps are excluded.
 */
public record V3WorldLimits(
    float maximumLinearSpeed,
    float maximumAngularSpeed,
    int projectileCandidateCap,
    boolean enableSleep
) {
    /** Engine defaults for every limit. */
    public static final V3WorldLimits DEFAULTS = new V3WorldLimits(0.0f, 0.0f, 0);

    /**
     * Creates motion limits with sleeping enabled.
     */
    public V3WorldLimits(float maximumLinearSpeed, float maximumAngularSpeed, int projectileCandidateCap) {
        this(maximumLinearSpeed, maximumAngularSpeed, projectileCandidateCap, true);
    }

    public V3WorldLimits {
        if (!Float.isFinite(maximumLinearSpeed) || maximumLinearSpeed < 0.0f
            || !Float.isFinite(maximumAngularSpeed) || maximumAngularSpeed < 0.0f) {
            throw new IllegalArgumentException("speed limits must be finite and non-negative");
        }
        if (projectileCandidateCap < 0) {
            throw new IllegalArgumentException("projectile candidate cap must be non-negative");
        }
    }
}

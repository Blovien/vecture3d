package dev.hytalemodding.vecture3d.ffi;

/**
 * World motion limits: the maximum linear speed in meters per second, the maximum angular speed in
 * radians per second and the Projectile sweep candidate cap. Zero selects the engine default for each
 * one, and a zero candidate cap means unlimited. Sleeping is enabled by default.
 *
 * <p>BlockGrid hull motion is resolved at step boundaries. A hull allowed to travel more than about one cell per step
 * can pass clean through one-cell-thick geometry, because the step that carries it across leaves no
 * overlap for the solver to see; a fast enough spin does the same with a far corner. One cell per step
 * is the safe bound: measured against a one-cell-thick BlockGrid wall with four sub-steps a BlockGrid
 * hull still stops at two cells per step and passes clean through at three, but that margin belongs to
 * the scene, not to the engine. Bound the maximum linear speed with the smallest cell size the game
 * builds from divided by the step length. The game owns the trade between reach and speed.
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

    /** Retains the original constructor and enables sleeping. */
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

package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class V3WorldLimitsTest {
    // The bridge steps a fixed 1/60 s, so the engine default angular clamp is 0.25 pi per step
    private static final float DEFAULT_ANGULAR_CLAMP = (float) (0.25 * Math.PI * 60.0);
    private static final float SPIN_ABOVE_EVERY_LIMIT = 400.0f;

    @AfterEach
    void limitWorldsReturnToTheNativeLifetimeBaseline() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void aWorldCreatedWithoutLimitsReportsTheEngineDefaults() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3WorldLimits limits = world.limits();

            assertEquals(400.0f, limits.maximumLinearSpeed(), 0.0f);
            assertEquals(0.0f, limits.maximumAngularSpeed(), 0.0f);
            assertEquals(0, limits.projectileCandidateCap());
            assertTrue(limits.enableSleep());
        }
    }

    @Test
    void aWorldCreatedWithLimitsClampsASpinAboveTheAngularLimit() {
        V3WorldLimits limits = new V3WorldLimits(120.0f, 6.0f, 32);
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0, limits)) {
            assertEquals(limits, world.limits());

            V3Transform spun = spinOneStep(world, 2_601);

            assertEquals(6.0f, angularSpeed(spun), 1.0e-4f);
        }
    }

    @Test
    void theDefaultAngularLimitLeavesBox3DsPerStepClampInPlace() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3Transform spun = spinOneStep(world, 2_602);

            assertEquals(DEFAULT_ANGULAR_CLAMP, angularSpeed(spun), 1.0e-3f);
        }
    }

    @Test
    void aRuntimeAngularLimitAppliesFromTheNextStep() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            world.setLimits(new V3WorldLimits(0.0f, 2.5f, 0));
            assertEquals(new V3WorldLimits(400.0f, 2.5f, 0), world.limits());

            V3Transform spun = spinOneStep(world, 2_603);

            assertEquals(2.5f, angularSpeed(spun), 1.0e-4f);
            assertTrue(angularSpeed(spun) < DEFAULT_ANGULAR_CLAMP);
        }
    }

    @Test
    void sleepingCanBeDisabledAndReenabledAtRuntime() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            world.setLimits(new V3WorldLimits(0.0f, 0.0f, 0, false));
            assertFalse(world.limits().enableSleep());

            world.setLimits(new V3WorldLimits(0.0f, 0.0f, 0, true));
            assertTrue(world.limits().enableSleep());
        }
    }

    @Test
    void limitsMustBeFiniteAndNonNegative() {
        assertThrows(IllegalArgumentException.class, () -> new V3WorldLimits(-1.0f, 0.0f, 0));
        assertThrows(IllegalArgumentException.class, () -> new V3WorldLimits(0.0f, Float.NaN, 0));
        assertThrows(IllegalArgumentException.class, () -> new V3WorldLimits(0.0f, 0.0f, -1));
    }

    // The angular clamp is a world setting, so this test observes it on a hull body.
    private static V3Transform spinOneStep(V3World world, long logicalId) {
        V3BoxBodyCommand body = V3TestSupport.box(
            logicalId,
            1,
            V3BoxBodyCommand.Kind.DYNAMIC,
            0.0,
            0.0,
            0.0,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            SPIN_ABOVE_EVERY_LIMIT,
            0.0f,
            V3BoxBodyCommand.INITIAL_AWAKE_FLAG
        );
        world.replaceBoxBodies(List.of(), List.of(body));
        return V3TestSupport.transform(world.step(1, List.of(), List.of(), List.of()), logicalId);
    }

    private static float angularSpeed(V3Transform transform) {
        float x = transform.angularVelocityX();
        float y = transform.angularVelocityY();
        float z = transform.angularVelocityZ();
        return (float) Math.sqrt(x * x + y * y + z * z);
    }
}

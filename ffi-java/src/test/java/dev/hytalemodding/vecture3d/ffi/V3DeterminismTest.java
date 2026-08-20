package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class V3DeterminismTest {
    @AfterEach
    void deterministicWorldsReturnToTheNativeLifetimeBaseline() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void identicalFreshWorldsReturnIdenticalFarOriginTransformBatches() {
        List<V3Transform> first = simulateFarScene();
        List<V3Transform> second = simulateFarScene();

        assertEquals(first, second);
        assertEquals(20_000_000.25, first.get(0).positionX(), 0.0);
        assertEquals(20_000_000.75, first.get(1).positionX(), 0.0);
        assertEquals(0.5, first.get(1).positionX() - first.get(0).positionX(), 0.0);
    }

    private static List<V3Transform> simulateFarScene() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, -9.81, 0.0)) {
            V3BoxBodyCommand floor = V3TestSupport.box(
                3_000,
                1,
                V3BoxBodyCommand.Kind.STATIC,
                20_000_000.5,
                -0.5,
                0.0
            );
            V3BoxBodyCommand first = V3TestSupport.box(
                3_001,
                1,
                V3BoxBodyCommand.Kind.DYNAMIC,
                20_000_000.25,
                3.0,
                -2.0
            );
            V3BoxBodyCommand second = V3TestSupport.box(
                3_002,
                1,
                V3BoxBodyCommand.Kind.DYNAMIC,
                20_000_000.75,
                5.0,
                2.0
            );
            world.replaceBoxBodies(List.of(), List.of(floor, first, second));

            V3StepResult result = null;
            for (int frame = 0; frame < 30; frame++) {
                result = world.step(1, List.of(), List.of(), List.of());
            }
            return result.transforms();
        }
    }
}

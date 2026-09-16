package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

class V3DeterminismTest {
    @Test
    void contactPointsAndIdentitiesSurviveFarOriginTranslation() {
        List<V3BlockContactEvent> near = contactAt(0.0);
        assertFalse(near.isEmpty());
        for (double origin : new double[] {10_000_000.0, -10_000_000.0, 20_000_000.0, -20_000_000.0}) {
            List<V3BlockContactEvent> far = contactAt(origin);
            assertEquals(near.size(), far.size(), "event count at " + origin);
            for (int index = 0; index < near.size(); index++) {
                V3BlockContactEvent expected = near.get(index);
                V3BlockContactEvent actual = far.get(index);
                assertEquals(expected.kind(), actual.kind());
                assertEquals(expected.fixedStepIndex(), actual.fixedStepIndex());
                assertEquals(expected.sideA(), actual.sideA());
                assertEquals(expected.sideB(), actual.sideB());
                assertEquals(expected.pointX(), actual.pointX() - origin, 1e-6, "local contact X at " + origin);
                assertEquals(expected.pointY(), actual.pointY() - origin, 1e-6, "local contact Y at " + origin);
                assertEquals(expected.pointZ(), actual.pointZ() - origin, 1e-6, "local contact Z at " + origin);
            }
        }
    }

    private static List<V3BlockContactEvent> contactAt(double origin) {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0.0, 0.0, 0.0);
             V3CookedGrid grid = library.cookBlockGrid(
                 List.of(V3TestSupport.BLOCK_MATERIAL),
                 List.of(new V3BlockCell(0, 0, 0, 0, 42L)),
                 List.of(V3BlockBox.fullCube(0, 0, 42L)))) {
            world.attachBlockGrid(V3BodyDefinition.at(new V3BodyHandle(1L, 1),
                V3BoxBodyCommand.Kind.STATIC, origin, origin, origin, 0), grid);
            world.replaceBoxBodies(List.of(), List.of(V3TestSupport.box(2L, 1,
                V3BoxBodyCommand.Kind.DYNAMIC, origin + 0.5, origin + 1.45, origin + 0.5)));
            return world.step(1, List.of(), List.of(), List.of()).blockContactEvents();
        }
    }

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

    @Test
    void identicalCookedBlockGridsGiveIdenticalTransformsInTwoWorlds() {
        List<V3Transform> first = simulateBlockGridScene();
        List<V3Transform> second = simulateBlockGridScene();

        assertEquals(first, second);
        assertEquals(1, first.size());
        assertEquals(0.0, first.get(0).positionY(), 0.05);
    }

    private static List<V3Transform> simulateBlockGridScene() {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0.0, -9.81, 0.0);
             V3CookedGrid terrain = V3TestSupport.flatTerrainSection().cook(library);
             V3CookedGrid hull = V3TestSupport.hollowHull(false).cook(library)) {
            world.attachBlockGrid(
                V3BodyDefinition.at(new V3BodyHandle(4_001L, 1), V3BoxBodyCommand.Kind.STATIC, 0.0, -1.0, 0.0, 0),
                terrain
            );
            world.attachBlockGrid(
                V3BodyDefinition.at(
                    new V3BodyHandle(4_002L, 1),
                    V3BoxBodyCommand.Kind.DYNAMIC,
                    -2.75,
                    3.0,
                    -3.25,
                    V3BoxBodyCommand.INITIAL_AWAKE_FLAG
                ),
                hull
            );

            V3StepResult result = null;
            for (int frame = 0; frame < 60; frame++) {
                result = world.step(4, List.of(), List.of(), List.of());
            }
            return result.transforms();
        }
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
            V3TestSupport.attachCenteredUnitGrids(world, List.of(floor, first, second));

            V3StepResult result = null;
            for (int frame = 0; frame < 30; frame++) {
                result = world.step(1, List.of(), List.of(), List.of());
            }
            return result.transforms();
        }
    }
}

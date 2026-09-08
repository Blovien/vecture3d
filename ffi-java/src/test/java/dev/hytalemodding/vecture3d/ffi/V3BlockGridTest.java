package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class V3BlockGridTest {
    private static final V3BodyHandle TERRAIN = new V3BodyHandle(8_001L, 1);
    private static final V3BodyHandle HULL = new V3BodyHandle(8_002L, 1);
    private static final V3BodyHandle FRAGMENTED_TERRAIN = new V3BodyHandle(8_003L, 1);
    private static final V3BodyHandle FRAGMENTED_BODY = new V3BodyHandle(8_004L, 1);

    @Test
    void rayCastAtMovingDynamicHullReturnsLogicalCell() {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0.0, 0.0, 0.0);
             V3CookedGrid hull = V3TestSupport.hollowHull(false).cook(library)) {
            V3BodyDefinition movingHull = new V3BodyDefinition(
                HULL,
                V3BoxBodyCommand.Kind.DYNAMIC,
                4.0,
                0.0,
                0.0,
                0.0f,
                0.0f,
                0.0f,
                1.0f,
                0.5f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                V3BoxBodyCommand.INITIAL_AWAKE_FLAG
            );
            world.attachBlockGrid(movingHull, hull);

            V3StepResult step = world.step(
                1,
                List.of(),
                List.of(),
                List.of(V3Query.ray(46L, 0.0, 0.5, 0.5, 10.0f, 0.0f, 0.0f))
            );
            assertTrue(V3TestSupport.transform(step, HULL.logicalId()).positionX() > 4.0);
            V3QueryResult hit = step.queryResults().getFirst();
            assertEquals(V3QueryResult.Status.CAST_HIT, hit.status());
            assertEquals(HULL.logicalId(), hit.hitLogicalId());
            assertTrue(hit.blockGrid());
            assertEquals(0, hit.cellX());
            assertEquals(0, hit.cellY());
            assertEquals(0, hit.cellZ());
            assertEquals(0, hit.cellBoxIndex());
            assertEquals(0, hit.materialIndex());
            assertEquals(V3TestSupport.BLOCK_MATERIAL.materialId(), hit.userMaterialId());
            assertEquals(900_000L, hit.userData());
            assertEquals(-1.0f, hit.normalX(), 1.0e-5f);
        }
    }

    @AfterEach
    void blockGridWorldsReturnToTheNativeLifetimeBaseline() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void cookedTerrainAndHullCollideAfterTheirCookingHandlesAreClosed() {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0.0, -9.81, 0.0)) {
            V3CookedGrid terrain = V3TestSupport.flatTerrainSection().cook(library);
            V3CookedGrid hull = V3TestSupport.hollowHull(true).cook(library);

            V3BodyHandle terrainHandle = world.attachBlockGrid(
                V3BodyDefinition.at(TERRAIN, V3BoxBodyCommand.Kind.STATIC, 0.0, -1.0, 0.0, 0),
                terrain
            );
            V3BodyHandle hullHandle = world.attachBlockGrid(
                V3BodyDefinition.at(
                    HULL,
                    V3BoxBodyCommand.Kind.DYNAMIC,
                    -3.0,
                    3.0,
                    -3.0,
                    V3BoxBodyCommand.INITIAL_AWAKE_FLAG
                ),
                hull
            );
            assertEquals(TERRAIN, terrainHandle);
            assertEquals(HULL, hullHandle);

            // Each shape holds a reference of its own, so the cooking handles are free right away.
            terrain.close();
            hull.close();
            assertTrue(terrain.isClosed());
            assertTrue(hull.isClosed());
            // Closing is idempotent.
            hull.close();

            V3StepResult result = null;
            long touchingPairs = 0;
            long contacts = 0;
            for (int frame = 0; frame < 60; frame++) {
                result = world.step(4, List.of(), List.of(), List.of());
                touchingPairs += result.stats().blockGridTouchingPairCount();
                contacts += result.stats().blockGridContactCount();
            }

            assertEquals(1, result.transforms().size());
            V3Transform landed = V3TestSupport.transform(result, HULL.logicalId());
            assertEquals(0.0, landed.positionY(), 0.05);
            assertTrue(touchingPairs > 0, "expected touching pairs while the hull rests");
            assertTrue(contacts > 0, "expected BlockGrid contacts while the hull rests");
            assertTrue(result.stats().blockGridTouchingPairCount() > 0);
            assertTrue(result.stats().blockGridContactCount() > 0);
            assertTrue(result.stats().blockGridCandidateHitboxPairCount() >= result.stats().blockGridTouchingPairCount());
            assertTrue(result.stats().blockGridScratchPeakBytes() > 0);
            // This fixture performs no projectile sweeps or grid replacements, so these counts are zero.
            assertEquals(0, result.stats().blockGridProjectileSweepCount());
            assertEquals(0, result.stats().blockGridCapExhaustionCount());
            assertEquals(0, result.stats().blockGridReplacementPublishedCount());

            // The attached body is an ordinary body, so the ordinary removal path retires it.
            world.replaceBoxBodies(List.of(hullHandle), List.of());
            V3StepResult afterRemoval = world.step(1, List.of(), List.of(), List.of());
            assertTrue(afterRemoval.transforms().isEmpty());
            assertEquals(0, afterRemoval.stats().blockGridContactCount());
        }
    }

    @Test
    void aDynamicBlockGridSleepsOnTerrainAndWorldDisableWakesIt() {
        V3NativeLibrary library = V3TestSupport.library();
        V3BodyHandle terrainHandle = new V3BodyHandle(8_005L, 1);
        V3BodyHandle bodyHandle = new V3BodyHandle(8_006L, 1);
        try (V3World world = library.createWorld(0.0, -9.81, 0.0);
             V3CookedGrid terrain = V3TestSupport.flatTerrainSection().cook(library);
             V3CookedGrid body = V3TestSupport.hollowHull(false).cook(library)) {
            world.attachBlockGrid(
                V3BodyDefinition.at(terrainHandle, V3BoxBodyCommand.Kind.STATIC, 0.0, -1.0, 0.0, 0),
                terrain
            );
            world.attachBlockGrid(
                V3BodyDefinition.at(
                    bodyHandle,
                    V3BoxBodyCommand.Kind.DYNAMIC,
                    0.0,
                    3.0,
                    0.0,
                    V3BoxBodyCommand.ENABLE_SLEEP_FLAG | V3BoxBodyCommand.INITIAL_AWAKE_FLAG
                ),
                body
            );

            V3Transform settled = null;
            for (int step = 0; step < 180; step++) {
                settled = V3TestSupport.transform(world.step(1, List.of(), List.of(), List.of()), bodyHandle.logicalId());
                if (!settled.awake()) {
                    break;
                }
            }
            assertTrue(settled != null);
            assertFalse(settled.awake());
            assertEquals(0.0, settled.positionY(), 0.05);

            world.setLimits(new V3WorldLimits(0.0f, 0.0f, 0, false));
            assertFalse(world.limits().enableSleep());
            V3Transform woken = V3TestSupport.transform(
                world.step(1, List.of(), List.of(), List.of()),
                bodyHandle.logicalId()
            );
            assertTrue(woken.awake());

            world.setLimits(new V3WorldLimits(0.0f, 0.0f, 0, true));
            assertTrue(world.limits().enableSleep());
            V3Transform resettled = woken;
            for (int step = 0; step < 60; step++) {
                resettled = V3TestSupport.transform(
                    world.step(1, List.of(), List.of(), List.of()),
                    bodyHandle.logicalId()
                );
            }
            assertFalse(resettled.awake());
        }
    }

    @Test
    void contactReductionCountAccumulatesAcrossFixedSteps() {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0.0, -10.0, 0.0);
             V3CookedGrid terrain = V3TestSupport.fragmentedTerrainSection().cook(library);
             V3CookedGrid body = V3TestSupport.fragmentedLandingBody().cook(library)) {
            world.attachBlockGrid(
                V3BodyDefinition.at(FRAGMENTED_TERRAIN, V3BoxBodyCommand.Kind.STATIC, 0.0, 0.0, 0.0, 0),
                terrain
            );
            world.attachBlockGrid(
                V3BodyDefinition.at(
                    FRAGMENTED_BODY,
                    V3BoxBodyCommand.Kind.DYNAMIC,
                    0.0,
                    1.0,
                    0.0,
                    V3BoxBodyCommand.INITIAL_AWAKE_FLAG
                ),
                body
            );

            int consecutiveReducedSteps = 0;
            for (int step = 0; step < 120 && consecutiveReducedSteps < 8; step++) {
                V3StepResult result = world.step(1, List.of(), List.of(), List.of());
                consecutiveReducedSteps = result.stats().blockGridContactReductionCount() == 1
                    ? consecutiveReducedSteps + 1
                    : 0;
            }
            assertEquals(8, consecutiveReducedSteps, "expected a stable reduced contact");

            V3StepResult accumulated = world.step(4, List.of(), List.of(), List.of());
            assertEquals(4, accumulated.stats().fixedStepCount());
            assertEquals(4, accumulated.stats().blockGridContactReductionCount());
        }
    }

    @Test
    void oneCookedHandleBacksBodiesInTwoWorldsAndTakesAMassOverride() {
        V3NativeLibrary library = V3TestSupport.library();
        V3MassProperties override = new V3MassProperties(
            250.0f,
            0.25f,
            0.5f,
            -0.25f,
            400.0f,
            500.0f,
            600.0f,
            0.0f,
            0.0f,
            0.0f
        );
        try (V3CookedGrid hull = V3TestSupport.hollowHull(false).cook(library);
             V3World first = library.createWorld(0.0, 0.0, 0.0);
             V3World second = library.createWorld(0.0, 0.0, 0.0)) {
            V3BodyDefinition body = V3BodyDefinition.at(
                HULL,
                V3BoxBodyCommand.Kind.DYNAMIC,
                0.0,
                0.0,
                0.0,
                V3BoxBodyCommand.INITIAL_AWAKE_FLAG
            );
            assertEquals(HULL, first.attachBlockGrid(body, hull, override));
            assertEquals(HULL, second.attachBlockGrid(body, hull));

            V3Transform overridden = V3TestSupport.transform(
                first.step(1, List.of(), List.of(), List.of()),
                HULL.logicalId()
            );
            assertEquals(override.centerX(), overridden.localCenterX(), 0.0f);
            assertEquals(override.centerY(), overridden.localCenterY(), 0.0f);
            assertEquals(override.centerZ(), overridden.localCenterZ(), 0.0f);

            V3Transform derived = V3TestSupport.transform(
                second.step(1, List.of(), List.of(), List.of()),
                HULL.logicalId()
            );
            assertFalse(
                derived.localCenterX() == override.centerX() && derived.localCenterZ() == override.centerZ(),
                "a null override must leave the mass Box3D derives from the geometry"
            );

            // A logical identity is claimed once per world, exactly as for every other body.
            assertThrows(IllegalArgumentException.class, () -> first.attachBlockGrid(body, hull));
        }
    }

    @Test
    void cookRejectsInputThatDoesNotAddressItsOwnArrays() {
        V3NativeLibrary library = V3TestSupport.library();
        List<V3BlockMaterial> materials = List.of(V3TestSupport.BLOCK_MATERIAL);
        List<V3BlockCell> cells = List.of(new V3BlockCell(0, 0, 0, 0, 1L), new V3BlockCell(1, 0, 0, 0, 2L));

        assertThrows(
            IllegalArgumentException.class,
            () -> library.cookBlockGrid(materials, cells, List.of(V3BlockBox.fullCube(2, 0, 1L)))
        );
        assertThrows(
            IllegalArgumentException.class,
            () -> library.cookBlockGrid(materials, cells, List.of(V3BlockBox.fullCube(0, 1, 1L)))
        );
        assertThrows(IllegalArgumentException.class, () -> library.cookBlockGrid(materials, cells, List.of()));
        assertThrows(
            IllegalArgumentException.class,
            () -> library.cookBlockGrid(materials, List.of(new V3BlockCell(0, 0, 0, 1, 1L)), List.of(V3BlockBox.fullCube(0, 0, 1L)))
        );

        // A box has to stay inside the unit cube of the cell that owns it.
        assertThrows(
            IllegalArgumentException.class,
            () -> new V3BlockBox(0, 0, 1L, 0.5f, 0.5f, 0.5f, 0.5f, 0.75f, 0.5f)
        );

        // Two cells at one coordinate are refused by the native cook, not by the record.
        V3Exception duplicate = assertThrows(
            V3Exception.class,
            () -> library.cookBlockGrid(
                materials,
                List.of(new V3BlockCell(0, 0, 0, 0, 1L), new V3BlockCell(0, 0, 0, 0, 2L)),
                List.of(V3BlockBox.fullCube(0, 0, 1L), V3BlockBox.fullCube(1, 0, 2L))
            )
        );
        assertEquals(V3Exception.Kind.NATIVE_STATUS, duplicate.kind());
        assertEquals("cookBlockGrid", duplicate.operation());
    }

    @Test
    void aClosedCookedHandleCannotBeAttached() {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0.0, -9.81, 0.0)) {
            V3CookedGrid hull = V3TestSupport.hollowHull(false).cook(library);
            hull.close();
            V3BodyDefinition body = V3BodyDefinition.at(HULL, V3BoxBodyCommand.Kind.DYNAMIC, 0.0, 0.0, 0.0, 0);
            assertThrows(IllegalStateException.class, () -> world.attachBlockGrid(body, hull));
        }
    }
}

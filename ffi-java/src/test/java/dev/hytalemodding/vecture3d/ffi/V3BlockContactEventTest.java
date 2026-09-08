package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class V3BlockContactEventTest {
    private static final V3BodyHandle TERRAIN = new V3BodyHandle(7_001L, 1);
    private static final V3BodyHandle HULL = new V3BodyHandle(7_002L, 1);
    private static final long MATERIAL_ID = 0x1234_5678_9abc_def0L;
    private static final long TERRAIN_DATA = 0xfedc_ba98_7654_3210L;
    private static final long HULL_DATA = 0x7654_3210_abcd_ef01L;

    @AfterEach
    void worldsAreReleased() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void twoStepsRetainTheFirstBeginWithDetachedIdentity() {
        V3NativeLibrary library = V3TestSupport.library();
        V3StepResult retained;
        try (V3World world = library.createWorld(0.0, -10.0, 0.0)) {
            attachTerrain(library, world);
            attachHull(library, world, HULL, 1.0);
            retained = world.step(2, List.of(), List.of(), List.of());
            assertFalse(retained.blockContactEvents().isEmpty());
            assertEquals(List.of(-2, 0), retained.blockContactEvents().stream()
                .map(event -> event.sideA().body().equals(TERRAIN) ? event.sideA().cellX() : event.sideB().cellX())
                .distinct().sorted().toList());
            assertEquals(retained.blockContactEvents().size(), retained.stats().blockContactEventCount());
            assertEquals(0L, retained.stats().blockContactEventDroppedCount());
            assertFalse(retained.stats().blockContactEventsTruncated());
            for (V3BlockContactEvent event : retained.blockContactEvents()) {
                assertEquals(V3BlockContactEvent.Kind.BEGIN, event.kind());
                assertEquals(0, event.fixedStepIndex(), "the begin occurred before the final native step");
                assertIdentity(event, HULL);
            }
            assertThrows(UnsupportedOperationException.class, () -> retained.blockContactEvents().clear());

            V3StepResult empty = world.step(0, List.of(), List.of(), List.of());
            assertTrue(empty.blockContactEvents().isEmpty());
            assertEquals(0, empty.stats().blockContactEventCount());
            assertEquals(0.0f, empty.stats().stepMilliseconds());
            assertEquals(0.0f, empty.stats().pairMilliseconds());
            assertEquals(0.0f, empty.stats().collideMilliseconds());
            assertEquals(0.0f, empty.stats().solveMilliseconds());
        }
        // Neither later calls nor world teardown can invalidate this detached result.
        assertFalse(retained.blockContactEvents().isEmpty());
        retained.blockContactEvents().forEach(event -> assertIdentity(event, HULL));
    }

    @Test
    void anEndKeepsTheOldGenerationAndTheReplacementCanReportAHit() {
        V3NativeLibrary library = V3TestSupport.library();
        V3BodyHandle replacement = new V3BodyHandle(HULL.logicalId(), 2);
        try (V3World world = library.createWorld(0.0, -10.0, 0.0)) {
            attachTerrain(library, world);
            attachHull(library, world, HULL, 1.0);
            assertFalse(world.step(2, List.of(), List.of(), List.of()).blockContactEvents().isEmpty());
            world.replaceBoxBodies(List.of(HULL), List.of());
            attachHull(library, world, replacement, 4.0);
            assertTrue(world.step(0, List.of(), List.of(), List.of()).blockContactEvents().isEmpty());

            V3StepResult ended = world.step(2, List.of(), List.of(), List.of());
            List<V3BlockContactEvent> ends = ended.blockContactEvents().stream()
                .filter(event -> event.kind() == V3BlockContactEvent.Kind.END)
                .toList();
            assertFalse(ends.isEmpty());
            ends.forEach(event -> assertIdentity(event, HULL));

            V3BlockContactEvent hit = null;
            for (int frame = 0; frame < 90 && hit == null; frame++) {
                hit = world.step(2, List.of(), List.of(), List.of()).blockContactEvents().stream()
                    .filter(event -> event.kind() == V3BlockContactEvent.Kind.HIT)
                    .findFirst().orElse(null);
            }
            assertTrue(hit != null, "the replacement must fall onto the terrain and report a hit");
            assertIdentity(hit, replacement);
            assertTrue(hit.relativeNormalSpeed() > 0.0f);
            float normalImpulse = hit.impulseX() * hit.normalX()
                + hit.impulseY() * hit.normalY() + hit.impulseZ() * hit.normalZ();
            assertTrue(normalImpulse > 0.0f);
        }
    }

    @Test
    void existingResultConstructorsDefaultToNoEvents() {
        V3StepStats stats = new V3StepStats(
            0, 0, 0, 0, 0.0f, 0, 0, 0, 0, 0, 0, 0, 0.0f, 0.0f, 0.0f,
            0, 0, 0, 0, 0, 0, 0L, 0L, 0L, 0L, 0L, 0L, 0L, 0L
        );
        V3StepResult result = new V3StepResult(List.of(), List.of(), stats);
        assertTrue(result.blockContactEvents().isEmpty());
        assertEquals(0, stats.blockContactEventCount());
        assertEquals(0L, stats.blockContactEventDroppedCount());
        assertFalse(stats.blockContactEventsTruncated());
    }

    private static void assertIdentity(V3BlockContactEvent event, V3BodyHandle hull) {
        V3BlockContactSide terrainSide = event.sideA().body().equals(TERRAIN) ? event.sideA() : event.sideB();
        V3BlockContactSide hullSide = event.sideA().body().equals(hull) ? event.sideA() : event.sideB();
        assertEquals(TERRAIN, terrainSide.body());
        assertEquals(hull, hullSide.body());
        assertTrue(terrainSide.blockGrid());
        assertTrue(hullSide.blockGrid());
        assertTrue(terrainSide.cellX() == -2 || terrainSide.cellX() == 0);
        assertEquals(0, terrainSide.cellY());
        assertEquals(3, terrainSide.cellZ());
        assertEquals(1, terrainSide.cellBoxIndex());
        assertEquals(1, terrainSide.materialIndex());
        assertEquals(MATERIAL_ID, terrainSide.userMaterialId());
        assertEquals(TERRAIN_DATA, terrainSide.userData());
        assertEquals(terrainSide.cellX() + 7, hullSide.cellX());
        assertEquals(0, hullSide.cellY());
        assertEquals(-4, hullSide.cellZ());
        assertEquals(0, hullSide.cellBoxIndex());
        assertEquals(0, hullSide.materialIndex());
        assertEquals(79L, hullSide.userMaterialId());
        assertEquals(HULL_DATA, hullSide.userData());
        assertEquals(event.sideA().body().equals(TERRAIN) ? 1.0f : -1.0f, event.normalY(), 1.0e-5f);
    }

    private static void attachTerrain(V3NativeLibrary library, V3World world) {
        List<V3BlockMaterial> materials = List.of(
            V3BlockMaterial.of(11L, 1.0f, 0.0f, 0.0f),
            V3BlockMaterial.of(MATERIAL_ID, 1.0f, 0.0f, 0.0f)
        );
        List<V3BlockBox> boxes = List.of(
            new V3BlockBox(0, 1, 0L, 0.5f, 0.25f, 0.5f, 0.5f, 0.25f, 0.5f),
            new V3BlockBox(0, 1, 0L, 0.5f, 0.75f, 0.5f, 0.5f, 0.25f, 0.5f),
            new V3BlockBox(1, 1, 0L, 0.5f, 0.25f, 0.5f, 0.5f, 0.25f, 0.5f),
            new V3BlockBox(1, 1, 0L, 0.5f, 0.75f, 0.5f, 0.5f, 0.25f, 0.5f)
        );
        try (V3CookedGrid grid = library.cookBlockGrid(
            materials, List.of(
                new V3BlockCell(-2, 0, 3, 1, TERRAIN_DATA), new V3BlockCell(0, 0, 3, 1, TERRAIN_DATA)
            ), boxes
        )) {
            world.attachBlockGrid(V3BodyDefinition.at(TERRAIN, V3BoxBodyCommand.Kind.STATIC, 0.0, 0.0, 0.0, 0), grid);
        }
    }

    private static void attachHull(V3NativeLibrary library, V3World world, V3BodyHandle handle, double y) {
        try (V3CookedGrid grid = library.cookBlockGrid(
            List.of(V3BlockMaterial.of(79L, 1.0f, 0.0f, 0.0f)),
            List.of(new V3BlockCell(5, 0, -4, 0, HULL_DATA), new V3BlockCell(7, 0, -4, 0, HULL_DATA)),
            List.of(V3BlockBox.fullCube(0, 0, 0L), V3BlockBox.fullCube(1, 0, 0L))
        )) {
            world.attachBlockGrid(V3BodyDefinition.at(
                handle, V3BoxBodyCommand.Kind.DYNAMIC, -7.0, y, 7.0,
                V3BoxBodyCommand.INITIAL_AWAKE_FLAG | V3BoxBodyCommand.ENABLE_SLEEP_FLAG
            ), grid);
        }
    }
}

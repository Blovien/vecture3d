package dev.hytalemodding.vecture3d.ffi;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class V3ProjectileTest {
    private static final V3BodyHandle PROJECTILE = new V3BodyHandle(7_101L, 1);
    private static final V3BodyHandle WALL = new V3BodyHandle(7_102L, 1);
    private static final float RADIUS = 0.12f;
    private static final float SPEED = 120.0f;
    private static final long MATERIAL = 71L;
    private static final long CELL_DATA = 7_123L;

    @AfterEach
    void worldsAreReleased() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void bulletFlagsAreAcceptedOnlyForDynamicDefinitions() {
        int bullet = V3BoxBodyCommand.BULLET_FLAG;
        assertEquals(bullet, V3BodyDefinition.at(PROJECTILE, V3BoxBodyCommand.Kind.DYNAMIC, 0, 0, 0, bullet).flags());
        assertEquals(bullet, V3TestSupport.box(7_103L, 1, V3BoxBodyCommand.Kind.DYNAMIC,
            0, 0, 0, 0, 0, 0, 0, 0, 0, bullet).flags());
        for (V3BoxBodyCommand.Kind kind : new V3BoxBodyCommand.Kind[] {
            V3BoxBodyCommand.Kind.STATIC, V3BoxBodyCommand.Kind.KINEMATIC
        }) {
            assertThrows(IllegalArgumentException.class, () -> V3BodyDefinition.at(PROJECTILE, kind, 0, 0, 0, bullet));
            assertThrows(IllegalArgumentException.class, () -> V3TestSupport.box(7_104L, 1, kind,
                0, 0, 0, 0, 0, 0, 0, 0, 0, bullet));
        }
    }

    @Test
    void aBulletSphereStopsAndReportsItsCellAtStaticTerrainAndADynamicHull() {
        V3NativeLibrary library = V3TestSupport.library();
        for (V3BoxBodyCommand.Kind wallKind : new V3BoxBodyCommand.Kind[] {
            V3BoxBodyCommand.Kind.STATIC, V3BoxBodyCommand.Kind.DYNAMIC
        }) {
            try (V3World world = library.createWorld(0, 0, 0, new V3WorldLimits(240, 6, 0, false))) {
                try (V3CookedGrid grid = library.cookBlockGrid(
                    List.of(V3BlockMaterial.of(MATERIAL, 1, 0, 0)),
                    List.of(new V3BlockCell(0, 6, 9, 0, CELL_DATA)),
                    List.of(V3BlockBox.fullCube(0, 0, 0))
                )) {
                    world.attachBlockGrid(V3BodyDefinition.at(WALL, wallKind, 0, 0, 0,
                        V3BoxBodyCommand.INITIAL_AWAKE_FLAG), grid);
                }
                assertEquals(PROJECTILE, world.createSphereBody(projectile(PROJECTILE, 6.5, 9.5), RADIUS, 1, 0));
                V3StepResult first = world.step(1, List.of(), List.of(), List.of());
                V3Transform sphere = V3TestSupport.transform(first, PROJECTILE.logicalId());
                // Without collision, 120 m/s for 1/60 s would take x=-1 to x=1, across the near face.
                assertEquals(-RADIUS, sphere.positionX(), 0.01, wallKind + " analytical near-face impact");
                assertTrue(first.stats().blockGridProjectileSweepCount() > 0);
                assertEquals(0, first.stats().blockGridCapExhaustionCount());

                List<V3BlockContactEvent> events = new ArrayList<>(first.blockContactEvents());
                V3StepResult last = first;
                for (int step = 0; step < 4; step++) {
                    last = world.step(1, List.of(), List.of(), List.of());
                    events.addAll(last.blockContactEvents());
                }
                V3BlockContactEvent hit = events.stream().filter(event -> event.kind() == V3BlockContactEvent.Kind.HIT)
                    .findFirst().orElseThrow(() -> new AssertionError(wallKind + " sphere must report a real hit"));
                V3BlockContactSide wallSide = hit.sideA().body().equals(WALL) ? hit.sideA() : hit.sideB();
                V3BlockContactSide sphereSide = hit.sideA().body().equals(PROJECTILE) ? hit.sideA() : hit.sideB();
                assertEquals(WALL, wallSide.body());
                assertTrue(wallSide.blockGrid());
                assertEquals(0, wallSide.cellX());
                assertEquals(6, wallSide.cellY());
                assertEquals(9, wallSide.cellZ());
                assertEquals(0, wallSide.cellBoxIndex());
                assertEquals(MATERIAL, wallSide.userMaterialId());
                assertEquals(CELL_DATA, wallSide.userData());
                assertEquals(PROJECTILE, sphereSide.body());
                assertFalse(sphereSide.blockGrid());
                assertTrue(hit.relativeNormalSpeed() > 100);
                assertTrue(hit.impulseX() * hit.normalX() + hit.impulseY() * hit.normalY()
                    + hit.impulseZ() * hit.normalZ() > 0);

                sphere = V3TestSupport.transform(last, PROJECTILE.logicalId());
                double wallX = 0;
                float wallSpeed = 0;
                if (wallKind == V3BoxBodyCommand.Kind.DYNAMIC) {
                    V3Transform wall = V3TestSupport.transform(last, WALL.logicalId());
                    wallX = wall.positionX();
                    wallSpeed = wall.linearVelocityX();
                }
                assertEquals(-RADIUS, sphere.positionX() - wallX, 0.02, "the sphere remains outside the near face");
                assertEquals(wallSpeed, sphere.linearVelocityX(), 1.0f, "the impact stops relative forward motion");
            }
        }
    }

    @Test
    void aCandidateCapReportsTheSafeHoldAndPreservesVelocity() {
        Shot unlimited = shootAtCellCorner(0);
        assertEquals(0, unlimited.exhaustions());
        assertEquals(-RADIUS, unlimited.x(), 0.01);
        int exactCap = 0;
        Shot exact = null;
        for (int cap = 1; cap <= 8; cap++) {
            Shot shot = shootAtCellCorner(cap);
            if (shot.exhaustions() == 0) {
                exactCap = cap;
                exact = shot;
                break;
            }
        }
        assertTrue(exactCap > 1, "the shared corner must require multiple candidate evaluations; got " + exactCap);
        assertEquals(unlimited.x(), exact.x(), 1.0e-9, "a completed search at the cap is not exhaustion");
        Shot held = shootAtCellCorner(exactCap - 1);
        assertTrue(held.exhaustions() > 0);
        assertTrue(held.sweeps() > 0);
        assertEquals(SPEED, held.velocityX(), 0.0f, "a budget hold preserves velocity");
        assertTrue(held.x() <= unlimited.x() + 1.0e-6 && held.x() >= -1.0);
        // The native bound adds 0.02 m of speculative distance to this sphere's reach.
        assertEquals(-RADIUS - 0.02, held.x(), 1.0e-6, "the conservative entry bound holds before impact");
    }

    @Test
    void sphereCreationRetainsGenerationRulesAndRuntimeSpeedLimits() {
        V3NativeLibrary library = V3TestSupport.library();
        V3WorldLimits initial = new V3WorldLimits(30, 6, 2, false);
        try (V3World world = library.createWorld(0, 0, 0, initial)) {
            assertEquals(initial, world.limits());
            V3BodyDefinition body = projectile(PROJECTILE, 0, 0);
            assertThrows(IllegalArgumentException.class, () -> world.createSphereBody(body, 0, 1, 0));
            assertThrows(IllegalArgumentException.class, () -> world.createSphereBody(body, RADIUS, 0, 0));
            assertThrows(IllegalArgumentException.class, () -> world.createSphereBody(body, RADIUS, 1, Float.NaN));
            assertEquals(0, world.step(0, List.of(), List.of(), List.of()).stats().bodyCount());
            assertEquals(PROJECTILE, world.createSphereBody(body, RADIUS, 1, 0));
            assertThrows(IllegalArgumentException.class, () -> world.createSphereBody(body, RADIUS, 1, 0));
            V3StepResult first = world.step(1, List.of(), List.of(), List.of());
            assertEquals(1, first.stats().bodyCount());
            V3Transform sphere = V3TestSupport.transform(first, PROJECTILE.logicalId());
            assertEquals(30, sphere.linearVelocityX(), 1.0e-4f);
            assertEquals(6, sphere.angularVelocityY(), 1.0e-4f);
            V3WorldLimits updated = new V3WorldLimits(20, 3, 1, true);
            world.setLimits(updated);
            assertEquals(updated, world.limits());
            sphere = V3TestSupport.transform(world.step(1, List.of(), List.of(), List.of()), PROJECTILE.logicalId());
            assertEquals(20, sphere.linearVelocityX(), 1.0e-4f);
            assertEquals(3, sphere.angularVelocityY(), 1.0e-4f);

            world.replaceBoxBodies(List.of(PROJECTILE), List.of());
            assertThrows(IllegalArgumentException.class, () -> world.createSphereBody(body, RADIUS, 1, 0));
            V3BodyHandle next = new V3BodyHandle(PROJECTILE.logicalId(), 2);
            assertEquals(next, world.createSphereBody(projectile(next, 0, 0), RADIUS, 1, 0));
            assertEquals(1, world.step(0, List.of(), List.of(), List.of()).stats().bodyCount());
        }
    }

    private static V3BodyDefinition projectile(V3BodyHandle handle, double y, double z) {
        return new V3BodyDefinition(handle, V3BoxBodyCommand.Kind.DYNAMIC,
            -1, y, z, 0, 0, 0, 1, SPEED, 0, 0, 0, 10, 0, 0, 0,
            V3BoxBodyCommand.BULLET_FLAG | V3BoxBodyCommand.INITIAL_AWAKE_FLAG);
    }

    private static Shot shootAtCellCorner(int cap) {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0, 0, 0, new V3WorldLimits(240, 6, 0, false))) {
            V3WorldLimits limits = new V3WorldLimits(240, 6, cap, false);
            world.setLimits(limits);
            assertEquals(limits, world.limits());
            List<V3BlockCell> cells = new ArrayList<>();
            List<V3BlockBox> boxes = new ArrayList<>();
            for (int y = 0; y < 2; y++) {
                for (int z = 0; z < 2; z++) {
                    int index = cells.size();
                    float depth = 0.5f + 0.02f * index;
                    cells.add(new V3BlockCell(0, y, z, 0, CELL_DATA + index));
                    boxes.add(new V3BlockBox(index, 0, 0, depth / 2, 0.5f, 0.5f, depth / 2, 0.5f, 0.5f));
                }
            }
            try (V3CookedGrid grid = library.cookBlockGrid(
                List.of(V3BlockMaterial.of(MATERIAL, 1, 0, 0)), cells, boxes
            )) {
                world.attachBlockGrid(V3BodyDefinition.at(WALL, V3BoxBodyCommand.Kind.STATIC, 0, 0, 0, 0), grid);
            }
            world.createSphereBody(projectile(PROJECTILE, 1, 1), RADIUS, 1, 0);
            V3StepResult result = world.step(1, List.of(), List.of(), List.of());
            V3Transform sphere = V3TestSupport.transform(result, PROJECTILE.logicalId());
            return new Shot(sphere.positionX(), sphere.linearVelocityX(), result.stats().blockGridProjectileSweepCount(),
                result.stats().blockGridCapExhaustionCount());
        }
    }

    private record Shot(double x, float velocityX, long sweeps, long exhaustions) {
    }
}

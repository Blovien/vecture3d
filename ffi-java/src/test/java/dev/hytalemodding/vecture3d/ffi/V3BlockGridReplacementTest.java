package dev.hytalemodding.vecture3d.ffi;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class V3BlockGridReplacementTest {
    private static final V3BodyHandle HULL = new V3BodyHandle(7_201L, 1);
    private static final V3BodyHandle TERRAIN = new V3BodyHandle(7_202L, 1);
    private static final long MATERIAL = 72L;

    @AfterEach
    void worldsAreReleased() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void derivedMassChangesWithVolumeWhileAnExplicitOverrideIsPreserved() {
        V3NativeLibrary library = V3TestSupport.library();
        for (boolean explicit : new boolean[] {false, true}) {
            try (V3World world = library.createWorld(0, 0, 0)) {
                V3MassProperties override = explicit ? new V3MassProperties(4, 0.5f, 0.5f, 0.5f,
                    2, 2, 2, 0, 0, 0) : null;
                try (V3CookedGrid grid = grid(library, 1, 0)) {
                    world.attachBlockGrid(body(HULL, V3BoxBodyCommand.Kind.DYNAMIC, 0), grid, override);
                }
                V3Transform first = pulse(world);
                // F * dt / mass: 120 * (1/60) / 1 = 2, or 0.5 with the mass override of 4.
                assertEquals(explicit ? 0.5f : 2.0f, first.linearVelocityX(), 1.0e-5f);
                try (V3CookedGrid revision = grid(library, 9, 0, 1)) {
                    world.replaceBlockGrid(HULL, revision);
                }
                V3StepResult changed = world.step(0, List.of(), List.of(), List.of());
                V3Transform immediate = V3TestSupport.transform(changed, HULL.logicalId());
                assertEquals(first.handle(), immediate.handle());
                assertEquals(first.positionX(), immediate.positionX(), 1.0e-8);
                assertEquals(first.linearVelocityX(), immediate.linearVelocityX(), 1.0e-6f);
                assertEquals(explicit ? 0.5f : 1.0f, immediate.localCenterX(), 1.0e-6f);
                V3Transform second = pulse(world);
                // Replacement retains density 1, despite the new cook's density 9: volume 2 gives mass 2.
                assertEquals(explicit ? 0.5f : 1.0f, second.linearVelocityX() - first.linearVelocityX(), 1.0e-5f);
                assertEquals(explicit ? 0.5f : 1.0f, second.localCenterX(), 1.0e-6f);
                world.replaceBoxBodies(List.of(HULL), List.of());
            }
        }
    }

    @Test
    void restingHullLosesRemovedCellContactsAndRebuildsRetainedSupport() {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0, -9.81, 0)) {
            try (V3CookedGrid geometry = grid(library, 1, 0, 2)) {
                world.attachBlockGrid(body(TERRAIN, V3BoxBodyCommand.Kind.STATIC, -1), geometry);
                world.attachBlockGrid(body(HULL, V3BoxBodyCommand.Kind.DYNAMIC, 0), geometry);
            }
            List<V3BlockContactEvent> beforeEvents = new ArrayList<>();
            V3StepResult resting = null;
            for (int step = 0; step < 120; step++) {
                resting = world.step(1, List.of(), List.of(), List.of());
                beforeEvents.addAll(resting.blockContactEvents());
            }
            assertTrue(hasEvent(beforeEvents, V3BlockContactEvent.Kind.BEGIN, 0));
            assertTrue(hasEvent(beforeEvents, V3BlockContactEvent.Kind.BEGIN, 2));
            assertEquals(0, V3TestSupport.transform(resting, HULL.logicalId()).positionY(), 0.03);
            try (V3CookedGrid revision = grid(library, 1, 0)) {
                world.replaceBlockGrid(HULL, revision);
            }
            List<V3BlockContactEvent> afterEvents = new ArrayList<>();
            long replacements = 0;
            for (int step = 0; step < 120; step++) {
                resting = world.step(1, List.of(), List.of(), List.of());
                replacements += resting.stats().blockGridReplacementPublishedCount();
                afterEvents.addAll(resting.blockContactEvents());
                assertEquals(0, V3TestSupport.transform(resting, HULL.logicalId()).positionY(), 0.03,
                    "the retained support must remain under the hull after replacement");
            }
            assertEquals(1, replacements);
            assertTrue(hasEvent(afterEvents, V3BlockContactEvent.Kind.END, 2), "removed cell must end");
            assertFalse(hasEvent(afterEvents, V3BlockContactEvent.Kind.BEGIN, 2), "removed cell cannot begin again");
            assertTrue(hasEvent(afterEvents, V3BlockContactEvent.Kind.BEGIN, 0), "retained support rebuilds");
            assertTrue(resting.stats().blockGridContactCount() > 0);
            V3Transform transform = V3TestSupport.transform(resting, HULL.logicalId());
            assertEquals(HULL, transform.handle());
            assertEquals(0.5f, transform.localCenterX(), 1.0e-5f);
            V3StepResult queries = world.step(0, List.of(), List.of(), List.of(
                V3Query.ray(1, -1, 0.5, 0.5, 5, 0, 0),
                V3Query.ray(2, 1.5, 0.5, 0.5, 2, 0, 0)));
            assertEquals(HULL.logicalId(), queries.queryResults().getFirst().hitLogicalId());
            assertEquals(0, queries.queryResults().getFirst().cellX());
            assertEquals(V3QueryResult.Status.NO_HIT, queries.queryResults().get(1).status());
        }
    }

    @Test
    void rejectedReplacementPreservesIdentityAndChecksHandleLifetimes() throws InterruptedException {
        V3NativeLibrary library = V3TestSupport.library();
        try (V3World world = library.createWorld(0, 0, 0);
             V3CookedGrid revision = grid(library, 1, 0, 1)) {
            world.createSphereBody(body(HULL, V3BoxBodyCommand.Kind.DYNAMIC, 0), 0.5f, 1, 0);
            V3Transform before = V3TestSupport.transform(world.step(0, List.of(), List.of(), List.of()), HULL.logicalId());
            V3Exception failure = assertThrows(V3Exception.class, () -> world.replaceBlockGrid(HULL, revision));
            assertEquals(V3Exception.Kind.NATIVE_STATUS, failure.kind());
            assertEquals("replaceBlockGrid", failure.operation());
            assertEquals(before, V3TestSupport.transform(world.step(0, List.of(), List.of(), List.of()), HULL.logicalId()));
            world.replaceBoxBodies(List.of(HULL), List.of());
            assertThrows(IllegalArgumentException.class, () -> world.replaceBlockGrid(HULL, revision));
            V3BodyHandle next = new V3BodyHandle(HULL.logicalId(), 2);
            world.attachBlockGrid(body(next, V3BoxBodyCommand.Kind.DYNAMIC, 0), revision);
            assertThrows(IllegalArgumentException.class, () -> world.replaceBlockGrid(HULL, revision));
            V3CookedGrid closed = grid(library, 1, 0);
            closed.close();
            assertThrows(IllegalStateException.class, () -> world.replaceBlockGrid(next, closed));
            AtomicReference<Throwable> crossThread = new AtomicReference<>();
            Thread thread = Thread.ofPlatform().start(() -> {
                try {
                    world.replaceBlockGrid(next, revision);
                } catch (Throwable threadFailure) {
                    crossThread.set(threadFailure);
                }
            });
            thread.join();
            assertInstanceOf(IllegalStateException.class, crossThread.get());
            world.replaceBlockGrid(next, revision);
            world.replaceBlockGrid(next, revision);
            assertEquals(next, V3TestSupport.transform(world.step(1, List.of(), List.of(), List.of()), next.logicalId()).handle());
        }
    }

    private static V3Transform pulse(V3World world) {
        return V3TestSupport.transform(world.step(1, List.of(), List.of(
            new V3BodyWrench(HULL, 120, 0, 0, 0, 0, 0, true)), List.of()), HULL.logicalId());
    }

    private static boolean hasEvent(List<V3BlockContactEvent> events, V3BlockContactEvent.Kind kind, int cellX) {
        return events.stream().filter(event -> event.kind() == kind).anyMatch(event -> {
            V3BlockContactSide side = event.sideA().body().equals(HULL) ? event.sideA() : event.sideB();
            return side.body().equals(HULL) && side.blockGrid() && side.cellX() == cellX
                && side.cellY() == 0 && side.cellZ() == 0 && side.cellBoxIndex() == 0
                && side.userMaterialId() == MATERIAL && side.userData() == 7_200L + cellX;
        });
    }

    private static V3BodyDefinition body(V3BodyHandle handle, V3BoxBodyCommand.Kind kind, double y) {
        return V3BodyDefinition.at(handle, kind, 0, y, 0, V3BoxBodyCommand.INITIAL_AWAKE_FLAG);
    }

    private static V3CookedGrid grid(V3NativeLibrary library, float density, int... xs) {
        List<V3BlockCell> cells = new ArrayList<>();
        List<V3BlockBox> boxes = new ArrayList<>();
        for (int x : xs) {
            boxes.add(V3BlockBox.fullCube(cells.size(), 0, 0));
            cells.add(new V3BlockCell(x, 0, 0, 0, 7_200L + x));
        }
        return library.cookBlockGrid(List.of(V3BlockMaterial.of(MATERIAL, density, 0.5f, 0)), cells, boxes);
    }
}

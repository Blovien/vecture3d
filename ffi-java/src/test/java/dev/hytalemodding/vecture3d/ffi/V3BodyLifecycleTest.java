package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class V3BodyLifecycleTest {
    @Test
    void distinctIdsDoNotExhaustBodyCapacityOrResetGenerations() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            for (long id = 1; id <= 12_288; id++) {
                V3BoxBodyCommand body = V3TestSupport.box(id, 1, V3BoxBodyCommand.Kind.STATIC, 0.0, 0.0, 0.0);
                world.replaceBoxBodies(List.of(), List.of(body));
                world.replaceBoxBodies(List.of(body.handle()), List.of());
            }
            V3BoxBodyCommand stale = V3TestSupport.box(1L, 1, V3BoxBodyCommand.Kind.DYNAMIC, 0.0, 0.0, 0.0);
            assertThrows(IllegalArgumentException.class, () -> world.replaceBoxBodies(List.of(), List.of(stale)));
            V3BoxBodyCommand reused = V3TestSupport.box(1L, 2, V3BoxBodyCommand.Kind.DYNAMIC, 0.0, 0.0, 0.0);
            world.replaceBoxBodies(List.of(), List.of(reused));
            assertThrows(IllegalArgumentException.class, () -> world.replaceBoxBodies(List.of(stale.handle()), List.of()));
            assertEquals(reused.handle(), world.step(0, List.of(), List.of(), List.of()).transforms().getFirst().handle());
        }
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }
}

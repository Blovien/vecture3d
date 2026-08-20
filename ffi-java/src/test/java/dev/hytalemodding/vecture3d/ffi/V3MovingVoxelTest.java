package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class V3MovingVoxelTest {
    @AfterEach
    void movingVoxelWorldsReturnToTheNativeLifetimeBaseline() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void nonOriginChildrenAndExplicitCenterRemainInTheBodyLocalFrame() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3BoxBodyCommand body = V3TestSupport.box(
                1_301,
                1,
                V3BoxBodyCommand.Kind.DYNAMIC,
                96.0,
                64.0,
                -32.0
            );
            List<V3TerrainBox> boxes = List.of(
                V3TestSupport.terrainBox(7_201, 0.5f, 0.5f, 0.5f),
                V3TestSupport.terrainBox(7_202, 2.5f, 0.5f, 0.5f)
            );
            V3MassProperties mass = new V3MassProperties(
                2.0f,
                1.5f,
                0.5f,
                0.5f,
                1.0f,
                1.0f,
                1.0f,
                0.0f,
                0.0f,
                0.0f
            );

            world.createVoxelGroup(body, boxes, mass);
            V3Transform transform = V3TestSupport.transform(world.step(1, List.of(), List.of(), List.of()), 1_301);

            assertEquals(96.0, transform.positionX(), 0.0);
            assertEquals(64.0, transform.positionY(), 0.0);
            assertEquals(-32.0, transform.positionZ(), 0.0);
            assertEquals(1.5f, transform.localCenterX(), 1.0e-5f);
            assertEquals(0.5f, transform.localCenterY(), 1.0e-5f);
            assertEquals(0.5f, transform.localCenterZ(), 1.0e-5f);
        }
    }

    @Test
    void offCenterMassPreservesTheRequestedBodyOriginVelocity() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3BoxBodyCommand body = V3TestSupport.box(
                1_401,
                1,
                V3BoxBodyCommand.Kind.DYNAMIC,
                0.0,
                8.0,
                0.0,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                2.0f,
                V3BoxBodyCommand.INITIAL_AWAKE_FLAG
            );
            List<V3TerrainBox> boxes = List.of(
                V3TestSupport.terrainBox(7_301, 0.5f, 0.5f, 0.5f),
                V3TestSupport.terrainBox(7_302, 2.5f, 0.5f, 0.5f)
            );
            V3MassProperties mass = new V3MassProperties(
                2.0f,
                1.5f,
                0.5f,
                0.5f,
                1.0f,
                1.0f,
                1.0f,
                0.0f,
                0.0f,
                0.0f
            );

            world.createVoxelGroup(body, boxes, mass);
            V3Transform transform = V3TestSupport.transform(world.step(0, List.of(), List.of(), List.of()), 1_401);

            assertEquals(-1.0f, transform.linearVelocityX(), 1.0e-5f);
            assertEquals(3.0f, transform.linearVelocityY(), 1.0e-5f);
            assertEquals(0.0f, transform.linearVelocityZ(), 1.0e-5f);
            assertEquals(2.0f, transform.angularVelocityZ(), 1.0e-5f);
        }
    }

    @Test
    void explicitInertiaControlsAngularAcceleration() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3BoxBodyCommand body = V3TestSupport.box(
                1_501,
                1,
                V3BoxBodyCommand.Kind.DYNAMIC,
                0.0,
                8.0,
                0.0
            );
            world.createVoxelGroup(
                body,
                List.of(V3TestSupport.terrainBox(7_401, 0.5f, 0.5f, 0.5f)),
                new V3MassProperties(6.0f, 0.5f, 0.5f, 0.5f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f)
            );

            V3StepResult result = world.step(
                1,
                List.of(),
                List.of(new V3BodyWrench(body.handle(), 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true)),
                List.of()
            );

            assertEquals(1.0 / 60.0, V3TestSupport.transform(result, 1_501).angularVelocityX(), 2.0e-4);
        }
    }
}

package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class V3MovingBlockGridTest {
    @AfterEach
    void movingBlockGridWorldsReturnToTheNativeLifetimeBaseline() {
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

            try (V3CookedGrid grid = twoCells()) {
                world.attachBlockGrid(V3TestSupport.bodyDefinition(body), grid, mass);
            }
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

            try (V3CookedGrid grid = twoCells()) {
                world.attachBlockGrid(V3TestSupport.bodyDefinition(body), grid, mass);
            }
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
            try (V3CookedGrid grid = V3TestSupport.library().cookBlockGrid(
                List.of(V3TestSupport.BLOCK_MATERIAL), List.of(new V3BlockCell(0, 0, 0, 0, 7_401)),
                List.of(V3BlockBox.fullCube(0, 0, 7_401)))) {
                world.attachBlockGrid(V3TestSupport.bodyDefinition(body), grid,
                    new V3MassProperties(6.0f, 0.5f, 0.5f, 0.5f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f));
            }

            V3StepResult result = world.step(
                1,
                List.of(),
                List.of(new V3BodyWrench(body.handle(), 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true)),
                List.of()
            );

            assertEquals(1.0 / 60.0, V3TestSupport.transform(result, 1_501).angularVelocityX(), 2.0e-4);
        }
    }

    private static V3CookedGrid twoCells() {
        return V3TestSupport.library().cookBlockGrid(List.of(V3TestSupport.BLOCK_MATERIAL),
            List.of(new V3BlockCell(0, 0, 0, 0, 1), new V3BlockCell(2, 0, 0, 0, 2)),
            List.of(V3BlockBox.fullCube(0, 0, 1), V3BlockBox.fullCube(1, 0, 2)));
    }
}

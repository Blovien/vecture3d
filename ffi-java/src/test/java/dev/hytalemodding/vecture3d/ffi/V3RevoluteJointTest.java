package dev.hytalemodding.vecture3d.ffi;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertThrows;

class V3RevoluteJointTest {
    private static final V3BodyHandle HULL = new V3BodyHandle(7_301, 1);
    private static final V3BodyHandle TURRET = new V3BodyHandle(7_302, 1);
    private static final long JOINT_ID = 7_310;
    private static final float FRAME = (float) Math.sqrt(0.5);

    @AfterEach
    void worldsAreReleased() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void turretFollowsTranslatingHullAndMotorPivotsAroundItsVerticalHinge() {
        try (V3World world = assembly(2)) {
            world.replaceRevoluteJoints(List.of(), List.of(hinge(1, 2, 1, 0,
                V3RevoluteJointCommand.ENABLE_MOTOR_FLAG)));
            V3StepResult result = null;
            for (int step = 1; step <= 60; step++) {
                result = move(world, step / 60.0);
                assertAnchors(result, 2);
            }
            V3Transform turret = V3TestSupport.transform(result, TURRET.logicalId());
            assertEquals(1, V3TestSupport.transform(result, HULL.logicalId()).positionX(), 1.0e-5);
            assertEquals(1, turret.angularVelocityY(), 0.03f);
            assertEquals(0, turret.angularVelocityX(), 0.01f);
            assertEquals(0, turret.angularVelocityZ(), 0.01f);
            assertEquals(1, angle(turret), 0.06);
            assertEquals(1, result.stats().jointCount());
        }
    }

    @Test
    void replacementReversesMotorAndSpringSeeksItsAngleWithinLimits() {
        try (V3World world = assembly(2)) {
            world.replaceRevoluteJoints(List.of(), List.of(hinge(1, 2, 1, 0,
                V3RevoluteJointCommand.ENABLE_MOTOR_FLAG)));
            V3StepResult result = null;
            for (int step = 0; step < 30; step++) {
                result = move(world, 0);
            }
            double forward = angle(V3TestSupport.transform(result, TURRET.logicalId()));
            assertEquals(0.5, forward, 0.06);
            world.replaceRevoluteJoints(List.of(new V3JointHandle(JOINT_ID, 1)),
                List.of(hinge(2, 2, -1, 0, V3RevoluteJointCommand.ENABLE_MOTOR_FLAG)));
            for (int step = 0; step < 30; step++) {
                result = move(world, 0);
                assertAnchors(result, 2);
            }
            V3Transform reversed = V3TestSupport.transform(result, TURRET.logicalId());
            assertEquals(-1, reversed.angularVelocityY(), 0.03);
            assertEquals(forward - 0.5, angle(reversed), 0.06);

            world.replaceRevoluteJoints(List.of(new V3JointHandle(JOINT_ID, 2)),
                List.of(hinge(3, 2, 0, 0.7f, V3RevoluteJointCommand.ENABLE_SPRING_FLAG
                    | V3RevoluteJointCommand.ENABLE_LIMIT_FLAG)));
            for (int step = 0; step < 120; step++) {
                result = move(world, 0);
                assertAnchors(result, 2);
            }
            V3Transform positioned = V3TestSupport.transform(result, TURRET.logicalId());
            assertEquals(0.7, angle(positioned), 0.03);
            assertEquals(0, positioned.angularVelocityY(), 0.03);

            // The spring may seek outside the allowed interval; the enabled limit must stop it.
            world.replaceRevoluteJoints(List.of(new V3JointHandle(JOINT_ID, 3)),
                List.of(hinge(4, 2, 0, 2, V3RevoluteJointCommand.ENABLE_SPRING_FLAG
                    | V3RevoluteJointCommand.ENABLE_LIMIT_FLAG)));
            for (int step = 0; step < 120; step++) {
                result = move(world, 0);
            }
            assertEquals(1.2, angle(V3TestSupport.transform(result, TURRET.logicalId())), 0.04);
        }
    }

    @Test
    void connectedCollisionFlagControlsRealHullTurretContacts() {
        for (boolean collide : new boolean[] {false, true}) {
            try (V3World world = assembly(0.9)) {
                world.replaceRevoluteJoints(List.of(), List.of(hinge(1, 0.9f, 0, 0,
                    collide ? V3RevoluteJointCommand.COLLIDE_CONNECTED_FLAG : 0)));
                long contactCount = 0;
                boolean began = false;
                for (int step = 0; step < 10; step++) {
                    V3StepResult result = move(world, 0);
                    contactCount += result.stats().blockGridContactCount();
                    began |= result.blockContactEvents().stream().anyMatch(event ->
                        event.kind() == V3BlockContactEvent.Kind.BEGIN
                            && ((event.sideA().body().equals(HULL) && event.sideB().body().equals(TURRET))
                                || (event.sideA().body().equals(TURRET) && event.sideB().body().equals(HULL))));
                }
                assertEquals(collide, contactCount > 0, "real connected contact count=" + contactCount);
                assertEquals(collide, began, "connected BEGIN event");
            }
        }
    }

    @Test
    void jointKindsShareGenerationsAndRejectedBatchesPreserveBodyProtection() {
        try (V3World world = assembly(2)) {
            V3JointHandle first = new V3JointHandle(JOINT_ID, 1);
            world.replaceRevoluteJoints(List.of(), List.of(hinge(1, 2, 0, 0, 0)));
            V3StepResult before = world.step(0, List.of(), List.of(), List.of());
            assertThrows(IllegalArgumentException.class, () -> world.replaceRevoluteJoints(List.of(first),
                List.of(hinge(2, 2, 0, 0, 0), hinge(2, 2, 0, 0, 0))));
            assertThrows(IllegalArgumentException.class, () -> world.replaceBoxBodies(List.of(TURRET), List.of()));
            V3StepResult rejected = world.step(0, List.of(), List.of(), List.of());
            assertEquals(before.transforms(), rejected.transforms());
            assertEquals(before.stats().jointCount(), rejected.stats().jointCount());
            assertEquals(before.stats().mutationBatchCount(), rejected.stats().mutationBatchCount());
            V3JointHandle second = new V3JointHandle(JOINT_ID, 2);
            world.replaceDistanceJoints(List.of(first), List.of(new V3DistanceJointCommand(second, HULL, TURRET,
                0, 0, 0, 0, 0, 0, 2, 1, 3, 0, 0, 0, 0, 0)));
            assertThrows(IllegalArgumentException.class, () -> world.replaceRevoluteJoints(List.of(first), List.of()));
            assertThrows(IllegalArgumentException.class, () -> world.replaceRevoluteJoints(List.of(second),
                List.of(hinge(2, 2, 0, 0, 0))));
            world.replaceRevoluteJoints(List.of(second), List.of(hinge(3, 2, 0, 0, 0)));
            assertThrows(IllegalArgumentException.class, () -> world.replaceBoxBodies(List.of(HULL), List.of()));
            world.replaceDistanceJoints(List.of(new V3JointHandle(JOINT_ID, 3)), List.of());
            world.replaceBoxBodies(List.of(HULL, TURRET), List.of());
            V3StepResult empty = world.step(0, List.of(), List.of(), List.of());
            assertEquals(0, empty.stats().jointCount());
            assertEquals(0, empty.stats().bodyCount());
        }
    }

    @Test
    void malformedMotorSettingsAreRejectedBeforeReplacement() {
        assertThrows(IllegalArgumentException.class, () -> hinge(1, 2, Float.NaN, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> hinge(1, 2, 0, Float.POSITIVE_INFINITY, 0));
        assertThrows(IllegalArgumentException.class, () -> hinge(1, 2, 0, 3.2f, 0));
        assertThrows(IllegalArgumentException.class, () -> hinge(1, 2, 0, -3.2f, 0));
        assertThrows(IllegalArgumentException.class, () -> hinge(1, 2, 0, 0, 16));
        // Native bounds include both endpoints. A spring target outside the limits is still valid.
        assertEquals((float) Math.PI, hinge(1, 2, 0, (float) Math.PI, 0).targetAngle());
        assertEquals(-(float) Math.PI, hinge(1, 2, 0, -(float) Math.PI, 0).targetAngle());
    }

    private static V3World assembly(double turretY) {
        V3NativeLibrary library = V3TestSupport.library();
        V3World world = library.createWorld(0, 0, 0);
        try (V3CookedGrid grid = library.cookBlockGrid(
            List.of(V3BlockMaterial.of(73, 1, 0.5f, 0)),
            List.of(new V3BlockCell(0, 0, 0, 0, 73)),
            List.of(V3BlockBox.fullCube(0, 0, 0)))) {
            world.attachBlockGrid(V3BodyDefinition.at(HULL, V3BoxBodyCommand.Kind.KINEMATIC,
                0, 0, 0, V3BoxBodyCommand.INITIAL_AWAKE_FLAG), grid);
            world.attachBlockGrid(V3BodyDefinition.at(TURRET, V3BoxBodyCommand.Kind.DYNAMIC,
                0, turretY, 0, V3BoxBodyCommand.INITIAL_AWAKE_FLAG), grid);
            return world;
        } catch (RuntimeException | Error failure) {
            world.close();
            throw failure;
        }
    }

    private static V3RevoluteJointCommand hinge(int generation, float turretY, float speed,
        float target, int flags) {
        return new V3RevoluteJointCommand(new V3JointHandle(JOINT_ID, generation), HULL, TURRET,
            0.5f, turretY + 0.5f, 0.5f, -FRAME, 0, 0, FRAME,
            0.5f, 0.5f, 0.5f, -FRAME, 0, 0, FRAME,
            target, 4, 1, -1.2f, 1.2f, 100, speed, flags);
    }

    private static V3StepResult move(V3World world, double x) {
        return world.step(1, List.of(new V3KinematicTarget(HULL, x, 0, 0, 0, 0, 0, 1, true)),
            List.of(), List.of());
    }

    private static double angle(V3Transform turret) {
        return 2 * Math.atan2(turret.rotationY(), turret.rotationW());
    }

    private static void assertAnchors(V3StepResult result, double turretY) {
        V3Transform hull = V3TestSupport.transform(result, HULL.logicalId());
        V3Transform turret = V3TestSupport.transform(result, TURRET.logicalId());
        double[] a = worldAnchor(hull, 0.5, turretY + 0.5, 0.5);
        double[] b = worldAnchor(turret, 0.5, 0.5, 0.5);
        double distance = Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2)
            + Math.pow(a[2] - b[2], 2));
        // At 60 Hz, one translation step is 1/60 m. The attachment must remain within 2 cm.
        assertTrue(distance < 0.02, "hinge anchor separation=" + distance);
        assertEquals(0, turret.rotationX(), 0.005);
        assertEquals(0, turret.rotationZ(), 0.005);
    }

    private static double[] worldAnchor(V3Transform pose, double x, double y, double z) {
        // Rotate using q*v*q^-1, expressed as v + 2*w*(q.xyz cross v) + 2*(q.xyz cross (q.xyz cross v)).
        double qx = pose.rotationX(), qy = pose.rotationY(), qz = pose.rotationZ(), qw = pose.rotationW();
        double cx = qy * z - qz * y, cy = qz * x - qx * z, cz = qx * y - qy * x;
        return new double[] {pose.positionX() + x + 2 * (qw * cx + qy * cz - qz * cy),
            pose.positionY() + y + 2 * (qw * cy + qz * cx - qx * cz),
            pose.positionZ() + z + 2 * (qw * cz + qx * cy - qy * cx)};
    }
}

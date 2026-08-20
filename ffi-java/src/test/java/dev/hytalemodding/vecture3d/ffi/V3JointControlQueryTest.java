package dev.hytalemodding.vecture3d.ffi;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class V3JointControlQueryTest {
    @AfterEach
    void controlledWorldsReturnToTheNativeLifetimeBaseline() {
        assertEquals(0, V3TestSupport.library().activeWorldCount());
    }

    @Test
    void distanceJointKeepsForcedBodiesInsideItsHardTravelLimits() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3BoxBodyCommand anchor = V3TestSupport.box(
                2_001,
                1,
                V3BoxBodyCommand.Kind.KINEMATIC,
                0.0,
                0.0,
                0.0
            );
            V3BoxBodyCommand body = V3TestSupport.box(
                2_002,
                1,
                V3BoxBodyCommand.Kind.DYNAMIC,
                4.0,
                0.0,
                0.0
            );
            world.replaceBoxBodies(List.of(), List.of(anchor, body));
            world.replaceDistanceJoints(List.of(), List.of(distanceJoint(2_100, anchor.handle(), body.handle())));

            V3StepResult result = null;
            for (int batch = 0; batch < 60; batch++) {
                result = world.step(
                    4,
                    List.of(),
                    List.of(new V3BodyWrench(body.handle(), 600.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, true)),
                    List.of()
                );
            }

            double distance = Math.abs(
                V3TestSupport.transform(result, body.handle().logicalId()).positionX()
                    - V3TestSupport.transform(result, anchor.handle().logicalId()).positionX()
            );
            assertTrue(distance >= 2.95 && distance <= 5.05, "distance=" + distance);
            assertEquals(1, result.stats().jointCount());
        }
    }

    @Test
    void fullBatchTargetAndWrenchReachTheExpectedPoseAndVelocity() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3BoxBodyCommand kinematic = V3TestSupport.box(
                2_201,
                1,
                V3BoxBodyCommand.Kind.KINEMATIC,
                0.0,
                0.0,
                0.0
            );
            V3BoxBodyCommand dynamic = V3TestSupport.box(
                2_202,
                1,
                V3BoxBodyCommand.Kind.DYNAMIC,
                0.0,
                4.0,
                0.0
            );
            world.replaceBoxBodies(List.of(), List.of(kinematic, dynamic));

            V3StepResult result = world.step(
                4,
                List.of(new V3KinematicTarget(
                    kinematic.handle(),
                    2.0,
                    0.0,
                    0.0,
                    0.0f,
                    0.0f,
                    0.0f,
                    1.0f,
                    true
                )),
                List.of(new V3BodyWrench(dynamic.handle(), 6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, true)),
                List.of()
            );

            assertEquals(2.0, V3TestSupport.transform(result, 2_201).positionX(), 1.0e-4);
            assertTrue(V3TestSupport.transform(result, 2_201).positionX() <= 2.0);
            assertEquals(0.4f, V3TestSupport.transform(result, 2_202).linearVelocityX(), 0.025f);
            assertEquals(4, result.stats().fixedStepCount());
        }
    }

    @Test
    void queriesAreSortedBoundedAndChooseTheLowestDeterministicObstacle() {
        List<V3QueryResult> first = runQueryScene(false);
        List<V3QueryResult> second = runQueryScene(true);

        assertEquals(first, second);
        assertEquals(List.of(10L, 20L, 30L), first.stream().map(V3QueryResult::queryId).toList());
        assertEquals(V3QueryResult.Status.IMMEDIATE_BLOCK, first.get(0).status());
        assertEquals(2_301, first.get(0).hitLogicalId());
        assertEquals(V3QueryResult.Status.CAST_HIT, first.get(1).status());
        assertEquals(2_301, first.get(1).hitLogicalId());
        assertTrue(first.get(1).fraction() > 0.0f && first.get(1).fraction() < 1.0f);
        assertEquals(V3QueryResult.Status.NO_HIT, first.get(2).status());
    }

    @Test
    void oversizedOrMalformedFrameLeavesTransformsAndCountersUnchanged() {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3BoxBodyCommand dynamic = V3TestSupport.box(
                2_401,
                1,
                V3BoxBodyCommand.Kind.DYNAMIC,
                0.0,
                0.0,
                0.0
            );
            world.replaceBoxBodies(List.of(), List.of(dynamic));
            V3StepResult before = world.step(0, List.of(), List.of(), List.of());
            List<V3Query> tooManyQueries = new ArrayList<>(1_025);
            for (int index = 0; index < 1_025; index++) {
                tooManyQueries.add(new V3Query(
                    index + 1L,
                    100.0 + index,
                    0.0,
                    0.0,
                    0.1f,
                    0.1f,
                    0.1f,
                    0.0f,
                    0.0f,
                    0.0f
                ));
            }

            assertThrows(
                IllegalArgumentException.class,
                () -> world.step(0, List.of(), List.of(), tooManyQueries)
            );
            assertThrows(
                IllegalArgumentException.class,
                () -> world.step(
                    1,
                    List.of(new V3KinematicTarget(
                        dynamic.handle(),
                        1.0,
                        0.0,
                        0.0,
                        0.0f,
                        0.0f,
                        0.0f,
                        1.0f,
                        true
                    )),
                    List.of(),
                    List.of()
                )
            );
            V3StepResult after = world.step(0, List.of(), List.of(), List.of());
            assertEquals(before.transforms(), after.transforms());
            assertEquals(before.stats().mutationBatchCount(), after.stats().mutationBatchCount());
            assertEquals(before.stats().fixedStepCount(), after.stats().fixedStepCount());
        }
    }

    private static List<V3QueryResult> runQueryScene(boolean reverseCreationOrder) {
        try (V3World world = V3TestSupport.library().createWorld(0.0, 0.0, 0.0)) {
            V3BoxBodyCommand lower = V3TestSupport.box(
                2_301,
                1,
                V3BoxBodyCommand.Kind.STATIC,
                3.0,
                0.0,
                0.0
            );
            V3BoxBodyCommand higher = V3TestSupport.box(
                2_302,
                1,
                V3BoxBodyCommand.Kind.STATIC,
                3.0,
                0.0,
                0.0
            );
            world.replaceBoxBodies(
                List.of(),
                reverseCreationOrder ? List.of(lower, higher) : List.of(higher, lower)
            );
            return world.step(0, List.of(), List.of(), List.of(
                new V3Query(30, -10.0, 0.0, 0.0, 0.25f, 0.25f, 0.25f, 1.0f, 0.0f, 0.0f),
                new V3Query(10, 3.0, 0.0, 0.0, 0.25f, 0.25f, 0.25f, 0.0f, 0.0f, 0.0f),
                new V3Query(20, 0.0, 0.0, 0.0, 0.25f, 0.25f, 0.25f, 6.0f, 0.0f, 0.0f)
            )).queryResults();
        }
    }

    private static V3DistanceJointCommand distanceJoint(
        long logicalId,
        V3BodyHandle bodyA,
        V3BodyHandle bodyB
    ) {
        return new V3DistanceJointCommand(
            new V3JointHandle(logicalId, 1),
            bodyA,
            bodyB,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            4.0f,
            3.0f,
            5.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            V3DistanceJointCommand.ENABLE_LIMIT_FLAG
        );
    }
}

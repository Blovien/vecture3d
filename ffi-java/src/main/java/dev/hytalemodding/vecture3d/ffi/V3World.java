package dev.hytalemodding.vecture3d.ffi;

import java.lang.foreign.Arena;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import dev.hytalemodding.vecture3d.ffi.generated.V3Abi;
import dev.hytalemodding.vecture3d.ffi.generated.v3_body_handle;
import dev.hytalemodding.vecture3d.ffi.generated.v3_body_wrench;
import dev.hytalemodding.vecture3d.ffi.generated.v3_box_body_command;
import dev.hytalemodding.vecture3d.ffi.generated.v3_distance_joint_command;
import dev.hytalemodding.vecture3d.ffi.generated.v3_joint_handle;
import dev.hytalemodding.vecture3d.ffi.generated.v3_kinematic_target;
import dev.hytalemodding.vecture3d.ffi.generated.v3_mass_properties;
import dev.hytalemodding.vecture3d.ffi.generated.v3_query;
import dev.hytalemodding.vecture3d.ffi.generated.v3_query_result;
import dev.hytalemodding.vecture3d.ffi.generated.v3_step_stats;
import dev.hytalemodding.vecture3d.ffi.generated.v3_terrain_box;
import dev.hytalemodding.vecture3d.ffi.generated.v3_terrain_section_command;
import dev.hytalemodding.vecture3d.ffi.generated.v3_transform;
import dev.hytalemodding.vecture3d.ffi.generated.v3_voxel_run;

import static java.lang.foreign.ValueLayout.JAVA_FLOAT;

public final class V3World implements AutoCloseable {
    private static final int MAX_BODIES = 4_096;
    private static final int MAX_JOINTS = 4_096;
    private static final int MAX_TERRAIN_CHILDREN = 65_534;
    private static final int MAX_VOXEL_RUNS = 262_144;
    private static final int MAX_TERRAIN_BOXES = 262_144;
    private static final int MAX_HULL_POINTS = 64;
    private static final int MIN_HULL_POINTS = 4;
    private static final int MAX_QUERIES = 1_024;
    private static final int MAX_FIXED_STEPS = 4;
    private static final int WAKE_FLAG = 1;

    private final Thread ownerThread;
    private final Arena arena;
    private final Map<Long, BodyState> bodies = new HashMap<>();
    private final Map<Long, JointState> joints = new HashMap<>();

    private final NativeBuffer bodyHandleBuffer;
    private final NativeBuffer boxCommandBuffer;
    private final NativeBuffer terrainSectionBuffer;
    private final NativeBuffer voxelRunBuffer;
    private final NativeBuffer terrainBoxBuffer;
    private final NativeBuffer hullPointBuffer;
    private final NativeBuffer massBuffer;
    private final NativeBuffer jointHandleBuffer;
    private final NativeBuffer jointCommandBuffer;
    private final NativeBuffer targetBuffer;
    private final NativeBuffer wrenchBuffer;
    private final NativeBuffer queryBuffer;
    private final NativeBuffer transformBuffer;
    private final NativeBuffer queryResultBuffer;
    private final MemorySegment statsBuffer;

    private MemorySegment nativeWorld;
    private int activeBodyCount;
    private int activeJointCount;
    private boolean closed;

    private V3World(MemorySegment nativeWorld, Arena arena) {
        this.nativeWorld = nativeWorld;
        this.arena = arena;
        ownerThread = Thread.currentThread();
        bodyHandleBuffer = new NativeBuffer(v3_body_handle.layout());
        boxCommandBuffer = new NativeBuffer(v3_box_body_command.layout());
        terrainSectionBuffer = new NativeBuffer(v3_terrain_section_command.layout());
        voxelRunBuffer = new NativeBuffer(v3_voxel_run.layout());
        terrainBoxBuffer = new NativeBuffer(v3_terrain_box.layout());
        hullPointBuffer = new NativeBuffer(JAVA_FLOAT);
        massBuffer = new NativeBuffer(v3_mass_properties.layout());
        jointHandleBuffer = new NativeBuffer(v3_joint_handle.layout());
        jointCommandBuffer = new NativeBuffer(v3_distance_joint_command.layout());
        targetBuffer = new NativeBuffer(v3_kinematic_target.layout());
        wrenchBuffer = new NativeBuffer(v3_body_wrench.layout());
        queryBuffer = new NativeBuffer(v3_query.layout());
        transformBuffer = new NativeBuffer(v3_transform.layout());
        queryResultBuffer = new NativeBuffer(v3_query_result.layout());
        statsBuffer = v3_step_stats.allocate(arena);
    }

    static V3World create(MemorySegment nativeWorld) {
        Arena arena = Arena.ofConfined();
        try {
            return new V3World(nativeWorld, arena);
        } catch (RuntimeException | Error failure) {
            arena.close();
            throw failure;
        }
    }

    public void replaceBoxBodies(List<V3BodyHandle> removals, List<V3BoxBodyCommand> creations) {
        requireOpenOwner();
        List<V3BodyHandle> removalValues = snapshot(removals, "removals");
        List<V3BoxBodyCommand> creationValues = snapshot(creations, "creations");
        BodyPlan plan = validateBodyReplacement(
            removalValues,
            creationValues.stream().map(value -> new BodyCreation(value.handle(), value.kind())).toList()
        );

        MemorySegment nativeRemovals = bodyHandleBuffer.ensure(removalValues.size());
        MemorySegment nativeCreations = boxCommandBuffer.ensure(creationValues.size());
        writeBodyHandles(nativeRemovals, removalValues);
        for (int index = 0; index < creationValues.size(); index++) {
            writeBoxCommand(v3_box_body_command.asSlice(nativeCreations, index), creationValues.get(index));
        }

        int status = V3Abi.v3_world_replace_box_bodies(
            nativeWorld,
            nativeRemovals,
            removalValues.size(),
            nativeCreations,
            creationValues.size()
        );
        V3NativeLibrary.requireSuccess("replaceBoxBodies", status, detail(removalValues.size(), creationValues.size()));
        applyBodyPlan(plan);
    }

    public void replaceTerrainSections(List<V3BodyHandle> removals, List<V3TerrainSectionCommand> creations) {
        requireOpenOwner();
        List<V3BodyHandle> removalValues = snapshot(removals, "removals");
        List<V3TerrainSectionCommand> creationValues = snapshot(creations, "creations");
        int voxelRunCount = 0;
        int detailBoxCount = 0;
        for (V3TerrainSectionCommand creation : creationValues) {
            voxelRunCount = addBounded(voxelRunCount, creation.voxelRuns().size(), MAX_VOXEL_RUNS, "voxel runs");
            detailBoxCount = addBounded(
                detailBoxCount,
                creation.detailBoxes().size(),
                MAX_TERRAIN_BOXES,
                "detail boxes"
            );
        }
        BodyPlan plan = validateBodyReplacement(
            removalValues,
            creationValues.stream()
                .map(value -> new BodyCreation(value.handle(), V3BoxBodyCommand.Kind.STATIC))
                .toList()
        );

        MemorySegment nativeRemovals = bodyHandleBuffer.ensure(removalValues.size());
        MemorySegment nativeSections = terrainSectionBuffer.ensure(creationValues.size());
        MemorySegment nativeVoxelRuns = voxelRunBuffer.ensure(voxelRunCount);
        MemorySegment nativeDetailBoxes = terrainBoxBuffer.ensure(detailBoxCount);
        writeBodyHandles(nativeRemovals, removalValues);
        int voxelRunOffset = 0;
        int detailBoxOffset = 0;
        for (int sectionIndex = 0; sectionIndex < creationValues.size(); sectionIndex++) {
            V3TerrainSectionCommand section = creationValues.get(sectionIndex);
            MemorySegment target = v3_terrain_section_command.asSlice(nativeSections, sectionIndex);
            writeTerrainSection(target, section, voxelRunOffset, detailBoxOffset);
            for (V3VoxelRun run : section.voxelRuns()) {
                MemorySegment runTarget = v3_voxel_run.asSlice(nativeVoxelRuns, voxelRunOffset++);
                v3_voxel_run.packed(runTarget, run.packed());
            }
            for (V3TerrainBox box : section.detailBoxes()) {
                writeTerrainBox(v3_terrain_box.asSlice(nativeDetailBoxes, detailBoxOffset++), box);
            }
        }

        int status = V3Abi.v3_world_replace_terrain_sections(
            nativeWorld,
            nativeRemovals,
            removalValues.size(),
            nativeSections,
            creationValues.size(),
            nativeVoxelRuns,
            voxelRunCount,
            nativeDetailBoxes,
            detailBoxCount
        );
        V3NativeLibrary.requireSuccess("replaceTerrainSections", status, detail(voxelRunCount, detailBoxCount));
        applyBodyPlan(plan);
    }

    public void createHullBody(V3BoxBodyCommand body, List<V3HullPoint> points) {
        requireOpenOwner();
        Objects.requireNonNull(body, "body");
        List<V3HullPoint> pointValues = snapshot(points, "points");
        if (pointValues.size() < MIN_HULL_POINTS || pointValues.size() > MAX_HULL_POINTS) {
            throw new IllegalArgumentException("points must contain between 4 and 64 entries");
        }
        BodyPlan plan = validateBodyReplacement(List.of(), List.of(new BodyCreation(body.handle(), body.kind())));

        MemorySegment nativeBody = boxCommandBuffer.ensure(1);
        MemorySegment nativePoints = hullPointBuffer.ensure(pointValues.size() * 3);
        writeBoxCommand(v3_box_body_command.asSlice(nativeBody, 0), body);
        for (int index = 0; index < pointValues.size(); index++) {
            V3HullPoint point = pointValues.get(index);
            nativePoints.setAtIndex(JAVA_FLOAT, index * 3L, point.x());
            nativePoints.setAtIndex(JAVA_FLOAT, index * 3L + 1, point.y());
            nativePoints.setAtIndex(JAVA_FLOAT, index * 3L + 2, point.z());
        }

        int status = V3Abi.v3_world_create_hull_body(nativeWorld, nativeBody, nativePoints, pointValues.size());
        V3NativeLibrary.requireSuccess("createHullBody", status, pointValues.size());
        applyBodyPlan(plan);
    }

    public void createVoxelGroup(V3BoxBodyCommand body, List<V3TerrainBox> boxes, V3MassProperties mass) {
        requireOpenOwner();
        Objects.requireNonNull(body, "body");
        List<V3TerrainBox> boxValues = snapshot(boxes, "boxes");
        Objects.requireNonNull(mass, "mass");
        if (body.kind() != V3BoxBodyCommand.Kind.DYNAMIC) {
            throw new IllegalArgumentException("voxel group body must be dynamic");
        }
        if (boxValues.isEmpty() || boxValues.size() > MAX_TERRAIN_CHILDREN) {
            throw new IllegalArgumentException("boxes must contain between 1 and 65,534 entries");
        }
        BodyPlan plan = validateBodyReplacement(List.of(), List.of(new BodyCreation(body.handle(), body.kind())));

        MemorySegment nativeBody = boxCommandBuffer.ensure(1);
        MemorySegment nativeBoxes = terrainBoxBuffer.ensure(boxValues.size());
        MemorySegment nativeMass = massBuffer.ensure(1);
        writeBoxCommand(v3_box_body_command.asSlice(nativeBody, 0), body);
        for (int index = 0; index < boxValues.size(); index++) {
            writeTerrainBox(v3_terrain_box.asSlice(nativeBoxes, index), boxValues.get(index));
        }
        writeMass(v3_mass_properties.asSlice(nativeMass, 0), mass);

        int status = V3Abi.v3_world_create_voxel_group(
            nativeWorld,
            nativeBody,
            nativeBoxes,
            boxValues.size(),
            nativeMass
        );
        V3NativeLibrary.requireSuccess("createVoxelGroup", status, boxValues.size());
        applyBodyPlan(plan);
    }

    public void replaceDistanceJoints(List<V3JointHandle> removals, List<V3DistanceJointCommand> creations) {
        requireOpenOwner();
        List<V3JointHandle> removalValues = snapshot(removals, "removals");
        List<V3DistanceJointCommand> creationValues = snapshot(creations, "creations");
        JointPlan plan = validateJointReplacement(removalValues, creationValues);

        MemorySegment nativeRemovals = jointHandleBuffer.ensure(removalValues.size());
        MemorySegment nativeCreations = jointCommandBuffer.ensure(creationValues.size());
        for (int index = 0; index < removalValues.size(); index++) {
            writeJointHandle(v3_joint_handle.asSlice(nativeRemovals, index), removalValues.get(index));
        }
        for (int index = 0; index < creationValues.size(); index++) {
            writeJointCommand(v3_distance_joint_command.asSlice(nativeCreations, index), creationValues.get(index));
        }

        int status = V3Abi.v3_world_replace_distance_joints(
            nativeWorld,
            nativeRemovals,
            removalValues.size(),
            nativeCreations,
            creationValues.size()
        );
        V3NativeLibrary.requireSuccess(
            "replaceDistanceJoints",
            status,
            detail(removalValues.size(), creationValues.size())
        );
        applyJointPlan(plan);
    }

    public V3StepResult step(
        int fixedStepCount,
        List<V3KinematicTarget> targets,
        List<V3BodyWrench> wrenches,
        List<V3Query> queries
    ) {
        requireOpenOwner();
        List<V3KinematicTarget> targetValues = snapshot(targets, "targets");
        List<V3BodyWrench> wrenchValues = snapshot(wrenches, "wrenches");
        List<V3Query> queryValues = snapshot(queries, "queries");
        int movableCount = validateFrame(fixedStepCount, targetValues, wrenchValues, queryValues);

        MemorySegment nativeTargets = targetBuffer.ensure(targetValues.size());
        MemorySegment nativeWrenches = wrenchBuffer.ensure(wrenchValues.size());
        MemorySegment nativeQueries = queryBuffer.ensure(queryValues.size());
        MemorySegment nativeTransforms = transformBuffer.ensure(movableCount);
        MemorySegment nativeQueryResults = queryResultBuffer.ensure(queryValues.size());
        for (int index = 0; index < targetValues.size(); index++) {
            writeTarget(v3_kinematic_target.asSlice(nativeTargets, index), targetValues.get(index));
        }
        for (int index = 0; index < wrenchValues.size(); index++) {
            writeWrench(v3_body_wrench.asSlice(nativeWrenches, index), wrenchValues.get(index));
        }
        for (int index = 0; index < queryValues.size(); index++) {
            writeQuery(v3_query.asSlice(nativeQueries, index), queryValues.get(index));
        }

        int status = V3Abi.v3_world_step_and_read(
            nativeWorld,
            nativeTargets,
            targetValues.size(),
            nativeWrenches,
            wrenchValues.size(),
            nativeQueries,
            queryValues.size(),
            fixedStepCount,
            nativeTransforms,
            movableCount,
            nativeQueryResults,
            queryValues.size(),
            statsBuffer
        );
        V3NativeLibrary.requireSuccess("step", status, detail(fixedStepCount, queryValues.size()));

        V3StepStats stats = readStats(statsBuffer);
        if (stats.outputCount() > movableCount || stats.queryOutputCount() > queryValues.size()) {
            throw new IllegalStateException("native output count exceeds admitted capacity");
        }
        List<V3Transform> transforms = new ArrayList<>(stats.outputCount());
        for (int index = 0; index < stats.outputCount(); index++) {
            transforms.add(readTransform(v3_transform.asSlice(nativeTransforms, index)));
        }
        List<V3QueryResult> queryResults = new ArrayList<>(stats.queryOutputCount());
        for (int index = 0; index < stats.queryOutputCount(); index++) {
            queryResults.add(readQueryResult(v3_query_result.asSlice(nativeQueryResults, index)));
        }
        return new V3StepResult(transforms, queryResults, stats);
    }

    @Override
    public void close() {
        requireOwnerThread();
        if (closed) {
            return;
        }
        try {
            // Native teardown must run while every pointer backed by the confined arena is still valid.
            V3Abi.v3_world_destroy(nativeWorld);
        } finally {
            nativeWorld = MemorySegment.NULL;
            closed = true;
            arena.close();
        }
    }

    private BodyPlan validateBodyReplacement(List<V3BodyHandle> removals, List<BodyCreation> creations) {
        if (removals.size() > MAX_BODIES || creations.size() > MAX_BODIES) {
            throw new IllegalArgumentException("body batch exceeds the native limit");
        }
        Set<Long> removalIds = new HashSet<>();
        for (V3BodyHandle removal : removals) {
            if (!removalIds.add(removal.logicalId())) {
                throw new IllegalArgumentException("body removals contain a duplicate logical ID");
            }
            requireActiveBody(removal);
            if (hasAttachedJoint(removal)) {
                throw new IllegalArgumentException("body has an attached joint");
            }
        }

        Set<Long> creationIds = new HashSet<>();
        int newEntries = 0;
        for (BodyCreation creation : creations) {
            V3BodyHandle handle = creation.handle();
            if (!creationIds.add(handle.logicalId())) {
                throw new IllegalArgumentException("body creations contain a duplicate logical ID");
            }
            BodyState state = bodies.get(handle.logicalId());
            if (state == null) {
                if (handle.generation() != 1) {
                    throw new IllegalArgumentException("a new body must start at generation one");
                }
                newEntries++;
                continue;
            }
            if (state.active() && !removalIds.contains(handle.logicalId())) {
                throw new IllegalArgumentException("body logical ID is already active");
            }
            requireNextGeneration(state.generation(), handle.generation(), "body");
        }

        int finalCount = activeBodyCount - removals.size() + creations.size();
        if (finalCount > MAX_BODIES || bodies.size() + newEntries > MAX_BODIES) {
            throw new IllegalArgumentException("body replacement exceeds the native limit");
        }
        if (activeBodyCount > MAX_BODIES - creations.size()) {
            throw new IllegalArgumentException("body replacement exceeds the transient native peak");
        }
        return new BodyPlan(removals, creations, finalCount);
    }

    private JointPlan validateJointReplacement(
        List<V3JointHandle> removals,
        List<V3DistanceJointCommand> creations
    ) {
        if (removals.size() > MAX_JOINTS || creations.size() > MAX_JOINTS) {
            throw new IllegalArgumentException("joint batch exceeds the native limit");
        }
        Set<Long> removalIds = new HashSet<>();
        for (V3JointHandle removal : removals) {
            if (!removalIds.add(removal.logicalId())) {
                throw new IllegalArgumentException("joint removals contain a duplicate logical ID");
            }
            JointState state = joints.get(removal.logicalId());
            if (state == null || !state.active() || state.generation() != removal.generation()) {
                throw new IllegalArgumentException("joint removal handle is stale");
            }
        }

        Set<Long> creationIds = new HashSet<>();
        int newEntries = 0;
        for (V3DistanceJointCommand creation : creations) {
            requireJointBody(creation.bodyA());
            requireJointBody(creation.bodyB());
            V3JointHandle handle = creation.handle();
            if (!creationIds.add(handle.logicalId())) {
                throw new IllegalArgumentException("joint creations contain a duplicate logical ID");
            }
            JointState state = joints.get(handle.logicalId());
            if (state == null) {
                if (handle.generation() != 1) {
                    throw new IllegalArgumentException("a new joint must start at generation one");
                }
                newEntries++;
                continue;
            }
            if (state.active() && !removalIds.contains(handle.logicalId())) {
                throw new IllegalArgumentException("joint logical ID is already active");
            }
            requireNextGeneration(state.generation(), handle.generation(), "joint");
        }

        int finalCount = activeJointCount - removals.size() + creations.size();
        if (finalCount > MAX_JOINTS || joints.size() + newEntries > MAX_JOINTS) {
            throw new IllegalArgumentException("joint replacement exceeds the native limit");
        }
        if (activeJointCount > MAX_JOINTS - creations.size()) {
            throw new IllegalArgumentException("joint replacement exceeds the transient native peak");
        }
        return new JointPlan(removals, creations, finalCount);
    }

    private int validateFrame(
        int fixedStepCount,
        List<V3KinematicTarget> targets,
        List<V3BodyWrench> wrenches,
        List<V3Query> queries
    ) {
        if (fixedStepCount < 0 || fixedStepCount > MAX_FIXED_STEPS) {
            throw new IllegalArgumentException("fixedStepCount must be between zero and four");
        }
        if (targets.size() > MAX_BODIES || wrenches.size() > MAX_BODIES || queries.size() > MAX_QUERIES) {
            throw new IllegalArgumentException("frame input exceeds the native limit");
        }

        Set<Long> targetIds = new HashSet<>();
        for (V3KinematicTarget target : targets) {
            if (fixedStepCount == 0) {
                throw new IllegalArgumentException("a target requires at least one fixed step");
            }
            if (!targetIds.add(target.handle().logicalId())) {
                throw new IllegalArgumentException("targets contain a duplicate logical ID");
            }
            BodyState state = requireActiveBody(target.handle());
            if (state.kind() != V3BoxBodyCommand.Kind.KINEMATIC) {
                throw new IllegalArgumentException("target body must be kinematic");
            }
        }

        Set<Long> wrenchIds = new HashSet<>();
        for (V3BodyWrench wrench : wrenches) {
            if (fixedStepCount == 0) {
                throw new IllegalArgumentException("a wrench requires at least one fixed step");
            }
            if (!wrenchIds.add(wrench.handle().logicalId())) {
                throw new IllegalArgumentException("wrenches contain a duplicate logical ID");
            }
            BodyState state = requireActiveBody(wrench.handle());
            if (state.kind() != V3BoxBodyCommand.Kind.DYNAMIC) {
                throw new IllegalArgumentException("wrench body must be dynamic");
            }
        }

        Set<Long> queryIds = new HashSet<>();
        for (V3Query query : queries) {
            if (!queryIds.add(query.queryId())) {
                throw new IllegalArgumentException("queries contain a duplicate query ID");
            }
        }

        int movableCount = 0;
        for (BodyState state : bodies.values()) {
            if (state.active() && state.kind() != V3BoxBodyCommand.Kind.STATIC) {
                movableCount++;
            }
        }
        return movableCount;
    }

    private BodyState requireActiveBody(V3BodyHandle handle) {
        BodyState state = bodies.get(handle.logicalId());
        if (state == null || !state.active() || state.generation() != handle.generation()) {
            throw new IllegalArgumentException("body handle is stale");
        }
        return state;
    }

    private void requireJointBody(V3BodyHandle handle) {
        BodyState state = requireActiveBody(handle);
        if (state.kind() == V3BoxBodyCommand.Kind.STATIC) {
            throw new IllegalArgumentException("distance joint bodies cannot be static");
        }
    }

    private boolean hasAttachedJoint(V3BodyHandle body) {
        for (JointState joint : joints.values()) {
            if (joint.active() && (joint.bodyA().equals(body) || joint.bodyB().equals(body))) {
                return true;
            }
        }
        return false;
    }

    private static void requireNextGeneration(int current, int requested, String owner) {
        if (current == Integer.MAX_VALUE) {
            throw new IllegalArgumentException(owner + " generation is exhausted");
        }
        if (requested != current + 1) {
            throw new IllegalArgumentException(owner + " generation must advance by exactly one");
        }
    }

    private void applyBodyPlan(BodyPlan plan) {
        for (V3BodyHandle removal : plan.removals()) {
            bodies.computeIfPresent(removal.logicalId(), (_, previous) -> new BodyState(previous.generation(), previous.kind(), false));
        }
        for (BodyCreation creation : plan.creations()) {
            V3BodyHandle handle = creation.handle();
            bodies.put(handle.logicalId(), new BodyState(handle.generation(), creation.kind(), true));
        }
        activeBodyCount = plan.finalCount();
    }

    private void applyJointPlan(JointPlan plan) {
        for (V3JointHandle removal : plan.removals()) {
            joints.computeIfPresent(
                removal.logicalId(),
                    (_, previous) -> new JointState(previous.generation(), previous.bodyA(), previous.bodyB(), false)
            );
        }
        for (V3DistanceJointCommand creation : plan.creations()) {
            V3JointHandle handle = creation.handle();
            joints.put(
                handle.logicalId(),
                new JointState(handle.generation(), creation.bodyA(), creation.bodyB(), true)
            );
        }
        activeJointCount = plan.finalCount();
    }

    private void requireOpenOwner() {
        requireOwnerThread();
        if (closed) {
            throw new IllegalStateException("V3World is closed");
        }
    }

    private void requireOwnerThread() {
        if (Thread.currentThread() != ownerThread) {
            throw new IllegalStateException("V3World may only be used by its owner thread");
        }
    }

    private static <T> List<T> snapshot(List<T> values, String name) {
        return List.copyOf(Objects.requireNonNull(values, name));
    }

    private static int addBounded(int current, int increment, int maximum, String name) {
        if (increment > maximum - current) {
            throw new IllegalArgumentException(name + " exceed the native batch limit");
        }
        return current + increment;
    }

    private static long detail(int first, int second) {
        return (long) first + second;
    }

    private static void writeBodyHandles(MemorySegment target, List<V3BodyHandle> values) {
        for (int index = 0; index < values.size(); index++) {
            writeBodyHandle(v3_body_handle.asSlice(target, index), values.get(index));
        }
    }

    private static void writeBodyHandle(MemorySegment target, V3BodyHandle value) {
        v3_body_handle.logical_id(target, value.logicalId());
        v3_body_handle.generation(target, value.generation());
        v3_body_handle.reserved(target, 0);
    }

    private static void writeBoxCommand(MemorySegment target, V3BoxBodyCommand value) {
        v3_box_body_command.logical_id(target, value.handle().logicalId());
        v3_box_body_command.kind(target, value.kind().nativeValue());
        v3_box_body_command.generation(target, value.handle().generation());
        v3_box_body_command.position_x(target, value.positionX());
        v3_box_body_command.position_y(target, value.positionY());
        v3_box_body_command.position_z(target, value.positionZ());
        v3_box_body_command.rotation_x(target, value.rotationX());
        v3_box_body_command.rotation_y(target, value.rotationY());
        v3_box_body_command.rotation_z(target, value.rotationZ());
        v3_box_body_command.rotation_w(target, value.rotationW());
        v3_box_body_command.linear_velocity_x(target, value.linearVelocityX());
        v3_box_body_command.linear_velocity_y(target, value.linearVelocityY());
        v3_box_body_command.linear_velocity_z(target, value.linearVelocityZ());
        v3_box_body_command.angular_velocity_x(target, value.angularVelocityX());
        v3_box_body_command.angular_velocity_y(target, value.angularVelocityY());
        v3_box_body_command.angular_velocity_z(target, value.angularVelocityZ());
        v3_box_body_command.half_extent_x(target, value.halfExtentX());
        v3_box_body_command.half_extent_y(target, value.halfExtentY());
        v3_box_body_command.half_extent_z(target, value.halfExtentZ());
        v3_box_body_command.density(target, value.density());
        v3_box_body_command.friction(target, value.friction());
        v3_box_body_command.linear_damping(target, value.linearDamping());
        v3_box_body_command.angular_damping(target, value.angularDamping());
        v3_box_body_command.flags(target, value.flags());
    }

    private static void writeTerrainSection(
        MemorySegment target,
        V3TerrainSectionCommand value,
        int voxelRunOffset,
        int detailBoxOffset
    ) {
        v3_terrain_section_command.logical_id(target, value.handle().logicalId());
        v3_terrain_section_command.generation(target, value.handle().generation());
        v3_terrain_section_command.voxel_run_offset(target, voxelRunOffset);
        v3_terrain_section_command.voxel_run_count(target, value.voxelRuns().size());
        v3_terrain_section_command.detail_box_offset(target, detailBoxOffset);
        v3_terrain_section_command.detail_box_count(target, value.detailBoxes().size());
        v3_terrain_section_command.reserved(target, 0);
        v3_terrain_section_command.origin_x(target, value.originX());
        v3_terrain_section_command.origin_y(target, value.originY());
        v3_terrain_section_command.origin_z(target, value.originZ());
    }

    private static void writeTerrainBox(MemorySegment target, V3TerrainBox value) {
        v3_terrain_box.center_x(target, value.centerX());
        v3_terrain_box.center_y(target, value.centerY());
        v3_terrain_box.center_z(target, value.centerZ());
        v3_terrain_box.half_extent_x(target, value.halfExtentX());
        v3_terrain_box.half_extent_y(target, value.halfExtentY());
        v3_terrain_box.half_extent_z(target, value.halfExtentZ());
        v3_terrain_box.friction(target, value.friction());
        v3_terrain_box.reserved(target, 0);
        v3_terrain_box.feature_id(target, value.featureId());
    }

    private static void writeMass(MemorySegment target, V3MassProperties value) {
        v3_mass_properties.mass(target, value.mass());
        v3_mass_properties.center_x(target, value.centerX());
        v3_mass_properties.center_y(target, value.centerY());
        v3_mass_properties.center_z(target, value.centerZ());
        v3_mass_properties.inertia_xx(target, value.inertiaXX());
        v3_mass_properties.inertia_yy(target, value.inertiaYY());
        v3_mass_properties.inertia_zz(target, value.inertiaZZ());
        v3_mass_properties.inertia_xy(target, value.inertiaXY());
        v3_mass_properties.inertia_xz(target, value.inertiaXZ());
        v3_mass_properties.inertia_yz(target, value.inertiaYZ());
    }

    private static void writeJointHandle(MemorySegment target, V3JointHandle value) {
        v3_joint_handle.logical_id(target, value.logicalId());
        v3_joint_handle.generation(target, value.generation());
        v3_joint_handle.reserved(target, 0);
    }

    private static void writeJointCommand(MemorySegment target, V3DistanceJointCommand value) {
        v3_distance_joint_command.logical_id(target, value.handle().logicalId());
        v3_distance_joint_command.generation(target, value.handle().generation());
        v3_distance_joint_command.reserved(target, 0);
        writeBodyHandle(v3_distance_joint_command.body_a(target), value.bodyA());
        writeBodyHandle(v3_distance_joint_command.body_b(target), value.bodyB());
        v3_distance_joint_command.local_anchor_a_x(target, value.localAnchorAX());
        v3_distance_joint_command.local_anchor_a_y(target, value.localAnchorAY());
        v3_distance_joint_command.local_anchor_a_z(target, value.localAnchorAZ());
        v3_distance_joint_command.local_anchor_b_x(target, value.localAnchorBX());
        v3_distance_joint_command.local_anchor_b_y(target, value.localAnchorBY());
        v3_distance_joint_command.local_anchor_b_z(target, value.localAnchorBZ());
        v3_distance_joint_command.rest_length(target, value.restLength());
        v3_distance_joint_command.minimum_length(target, value.minimumLength());
        v3_distance_joint_command.maximum_length(target, value.maximumLength());
        v3_distance_joint_command.hertz(target, value.hertz());
        v3_distance_joint_command.damping_ratio(target, value.dampingRatio());
        v3_distance_joint_command.lower_spring_force(target, value.lowerSpringForce());
        v3_distance_joint_command.upper_spring_force(target, value.upperSpringForce());
        v3_distance_joint_command.flags(target, value.flags());
    }

    private static void writeTarget(MemorySegment target, V3KinematicTarget value) {
        v3_kinematic_target.logical_id(target, value.handle().logicalId());
        v3_kinematic_target.generation(target, value.handle().generation());
        v3_kinematic_target.flags(target, value.wake() ? WAKE_FLAG : 0);
        v3_kinematic_target.position_x(target, value.positionX());
        v3_kinematic_target.position_y(target, value.positionY());
        v3_kinematic_target.position_z(target, value.positionZ());
        v3_kinematic_target.rotation_x(target, value.rotationX());
        v3_kinematic_target.rotation_y(target, value.rotationY());
        v3_kinematic_target.rotation_z(target, value.rotationZ());
        v3_kinematic_target.rotation_w(target, value.rotationW());
    }

    private static void writeWrench(MemorySegment target, V3BodyWrench value) {
        v3_body_wrench.logical_id(target, value.handle().logicalId());
        v3_body_wrench.generation(target, value.handle().generation());
        v3_body_wrench.flags(target, value.wake() ? V3Abi.V3_WRENCH_WAKE() : 0);
        v3_body_wrench.force_x(target, value.forceX());
        v3_body_wrench.force_y(target, value.forceY());
        v3_body_wrench.force_z(target, value.forceZ());
        v3_body_wrench.torque_x(target, value.torqueX());
        v3_body_wrench.torque_y(target, value.torqueY());
        v3_body_wrench.torque_z(target, value.torqueZ());
    }

    private static void writeQuery(MemorySegment target, V3Query value) {
        v3_query.query_id(target, value.queryId());
        v3_query.origin_x(target, value.originX());
        v3_query.origin_y(target, value.originY());
        v3_query.origin_z(target, value.originZ());
        v3_query.half_extent_x(target, value.halfExtentX());
        v3_query.half_extent_y(target, value.halfExtentY());
        v3_query.half_extent_z(target, value.halfExtentZ());
        v3_query.translation_x(target, value.translationX());
        v3_query.translation_y(target, value.translationY());
        v3_query.translation_z(target, value.translationZ());
        v3_query.reserved(target, 0);
    }

    private static V3Transform readTransform(MemorySegment source) {
        return new V3Transform(
            new V3BodyHandle(v3_transform.logical_id(source), v3_transform.generation(source)),
            (v3_transform.flags(source) & WAKE_FLAG) != 0,
            v3_transform.position_x(source),
            v3_transform.position_y(source),
            v3_transform.position_z(source),
            v3_transform.rotation_x(source),
            v3_transform.rotation_y(source),
            v3_transform.rotation_z(source),
            v3_transform.rotation_w(source),
            v3_transform.linear_velocity_x(source),
            v3_transform.linear_velocity_y(source),
            v3_transform.linear_velocity_z(source),
            v3_transform.angular_velocity_x(source),
            v3_transform.angular_velocity_y(source),
            v3_transform.angular_velocity_z(source),
            v3_transform.local_center_x(source),
            v3_transform.local_center_y(source),
            v3_transform.local_center_z(source)
        );
    }

    private static V3QueryResult readQueryResult(MemorySegment source) {
        return new V3QueryResult(
            v3_query_result.query_id(source),
            V3QueryResult.Status.fromNative(v3_query_result.status(source)),
            v3_query_result.hit_logical_id(source),
            v3_query_result.fraction(source)
        );
    }

    private static V3StepStats readStats(MemorySegment source) {
        return new V3StepStats(
            v3_step_stats.output_count(source),
            v3_step_stats.body_count(source),
            v3_step_stats.shape_count(source),
            v3_step_stats.contact_count(source),
            v3_step_stats.step_milliseconds(source),
            v3_step_stats.mutation_batch_count(source),
            v3_step_stats.created_body_count(source),
            v3_step_stats.destroyed_body_count(source),
            v3_step_stats.query_output_count(source),
            v3_step_stats.query_count(source),
            v3_step_stats.fixed_step_count(source),
            v3_step_stats.joint_count(source),
            v3_step_stats.pair_milliseconds(source),
            v3_step_stats.collide_milliseconds(source),
            v3_step_stats.solve_milliseconds(source),
            v3_step_stats.static_tree_height(source),
            v3_step_stats.dynamic_tree_height(source),
            v3_step_stats.sat_call_count(source),
            v3_step_stats.sat_cache_hit_count(source),
            v3_step_stats.graph_overflow_constraint_count(source),
            v3_step_stats.heap_move_pair_count(source)
        );
    }

    private final class NativeBuffer {
        private final MemoryLayout elementLayout;
        private MemorySegment segment = MemorySegment.NULL;
        private int capacity;

        private NativeBuffer(MemoryLayout elementLayout) {
            this.elementLayout = elementLayout;
        }

        private MemorySegment ensure(int required) {
            if (required == 0) {
                return MemorySegment.NULL;
            }
            if (required > capacity) {
                int nextCapacity = capacity == 0 ? required : Math.max(required, capacity * 2);
                segment = arena.allocate(MemoryLayout.sequenceLayout(nextCapacity, elementLayout));
                capacity = nextCapacity;
            }
            return segment;
        }
    }

    private record BodyState(int generation, V3BoxBodyCommand.Kind kind, boolean active) {
    }

    private record BodyCreation(V3BodyHandle handle, V3BoxBodyCommand.Kind kind) {
    }

    private record BodyPlan(List<V3BodyHandle> removals, List<BodyCreation> creations, int finalCount) {
    }

    private record JointState(int generation, V3BodyHandle bodyA, V3BodyHandle bodyB, boolean active) {
    }

    private record JointPlan(
        List<V3JointHandle> removals,
        List<V3DistanceJointCommand> creations,
        int finalCount
    ) {
    }
}

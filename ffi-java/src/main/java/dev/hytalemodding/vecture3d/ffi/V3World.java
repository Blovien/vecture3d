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
import dev.hytalemodding.vecture3d.ffi.generated.v3_block_contact_event;
import dev.hytalemodding.vecture3d.ffi.generated.v3_block_contact_side;
import dev.hytalemodding.vecture3d.ffi.generated.v3_body_definition;
import dev.hytalemodding.vecture3d.ffi.generated.v3_body_handle;
import dev.hytalemodding.vecture3d.ffi.generated.v3_body_wrench;
import dev.hytalemodding.vecture3d.ffi.generated.v3_box_body_command;
import dev.hytalemodding.vecture3d.ffi.generated.v3_distance_joint_command;
import dev.hytalemodding.vecture3d.ffi.generated.v3_joint_handle;
import dev.hytalemodding.vecture3d.ffi.generated.v3_kinematic_target;
import dev.hytalemodding.vecture3d.ffi.generated.v3_mass_properties;
import dev.hytalemodding.vecture3d.ffi.generated.v3_query;
import dev.hytalemodding.vecture3d.ffi.generated.v3_query_result;
import dev.hytalemodding.vecture3d.ffi.generated.v3_revolute_joint_command;
import dev.hytalemodding.vecture3d.ffi.generated.v3_step_stats;
import dev.hytalemodding.vecture3d.ffi.generated.v3_transform;
import dev.hytalemodding.vecture3d.ffi.generated.v3_world_limits;

import static java.lang.foreign.ValueLayout.JAVA_FLOAT;
import static java.lang.foreign.ValueLayout.JAVA_INT;

public final class V3World implements AutoCloseable {
    private static final int MAX_BODIES = 4_096;
    private static final int MAX_JOINTS = 4_096;
    private static final int MAX_HULL_POINTS = 64;
    private static final int MIN_HULL_POINTS = 4;
    private static final int MAX_QUERIES = 1_024;
    private static final int MAX_FIXED_STEPS = 4;
    private static final int WAKE_FLAG = 1;

    private final Thread ownerThread;
    private final Arena arena;
    private final Map<Long, BodyState> bodies = new HashMap<>();
    private final Map<Long, Integer> bodyGenerations = new HashMap<>();
    private final Map<Long, JointState> joints = new HashMap<>();

    private final NativeBuffer bodyHandleBuffer;
    private final NativeBuffer boxCommandBuffer;
    private final NativeBuffer hullPointBuffer;
    private final NativeBuffer massBuffer;
    private final NativeBuffer jointHandleBuffer;
    private final NativeBuffer jointCommandBuffer;
    private final NativeBuffer revoluteCommandBuffer;
    private final NativeBuffer targetBuffer;
    private final NativeBuffer wrenchBuffer;
    private final NativeBuffer queryBuffer;
    private final NativeBuffer transformBuffer;
    private final NativeBuffer queryResultBuffer;
    private final NativeBuffer blockContactEventBuffer;
    private final MemorySegment blockContactEventCountBuffer;
    private final NativeBuffer bodyDefinitionBuffer;
    private final MemorySegment statsBuffer;
    private final MemorySegment limitsBuffer;
    private final MemorySegment attachedHandleBuffer;

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
        hullPointBuffer = new NativeBuffer(JAVA_FLOAT);
        massBuffer = new NativeBuffer(v3_mass_properties.layout());
        jointHandleBuffer = new NativeBuffer(v3_joint_handle.layout());
        jointCommandBuffer = new NativeBuffer(v3_distance_joint_command.layout());
        revoluteCommandBuffer = new NativeBuffer(v3_revolute_joint_command.layout());
        targetBuffer = new NativeBuffer(v3_kinematic_target.layout());
        wrenchBuffer = new NativeBuffer(v3_body_wrench.layout());
        queryBuffer = new NativeBuffer(v3_query.layout());
        transformBuffer = new NativeBuffer(v3_transform.layout());
        queryResultBuffer = new NativeBuffer(v3_query_result.layout());
        blockContactEventBuffer = new NativeBuffer(v3_block_contact_event.layout());
        blockContactEventCountBuffer = arena.allocate(JAVA_INT);
        bodyDefinitionBuffer = new NativeBuffer(v3_body_definition.layout());
        statsBuffer = v3_step_stats.allocate(arena);
        limitsBuffer = v3_world_limits.allocate(arena);
        attachedHandleBuffer = v3_body_handle.allocate(arena);
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

    /**
     * Atomically replaces bodies. At most 4,096 bodies may be resident, and pending creations
     * coexist with removals until commit, so the same limit also applies to that temporary peak.
     * Distinct logical IDs have no fixed lifetime limit. Their last generations are retained until
     * world close to reject stale reuse. New IDs start at one, and reuse advances by exactly one.
     */
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

    /**
     * Creates a sphere centered at the body origin.
     */
    public V3BodyHandle createSphereBody(V3BodyDefinition body, float radius, float density, float friction) {
        requireOpenOwner();
        Objects.requireNonNull(body, "body");
        if (!V3BoxBodyCommand.allFinite(radius, density, friction) || radius <= 0.0f) {
            throw new IllegalArgumentException("sphere values must be finite and radius positive");
        }
        if ((body.kind() == V3BoxBodyCommand.Kind.DYNAMIC && density <= 0.0f)
            || (body.kind() != V3BoxBodyCommand.Kind.DYNAMIC && density != 0.0f)) {
            throw new IllegalArgumentException("density does not match body kind");
        }
        if (friction < 0.0f || friction > 1.0f) {
            throw new IllegalArgumentException("friction must be between zero and one");
        }
        BodyPlan plan = validateBodyReplacement(List.of(), List.of(new BodyCreation(body.handle(), body.kind())));
        MemorySegment nativeBody = bodyDefinitionBuffer.ensure(1);
        writeBodyDefinition(v3_body_definition.asSlice(nativeBody, 0), body);
        int status = V3Abi.v3_world_create_sphere_body(
            nativeWorld, nativeBody, radius, density, friction, attachedHandleBuffer
        );
        V3NativeLibrary.requireSuccess("createSphereBody", status, body.handle().logicalId());
        V3BodyHandle created = new V3BodyHandle(
            v3_body_handle.logical_id(attachedHandleBuffer), v3_body_handle.generation(attachedHandleBuffer)
        );
        if (!created.equals(body.handle())) {
            throw new IllegalStateException("native creation returned an unexpected body handle");
        }
        applyBodyPlan(plan);
        return created;
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

    /**
     * Creates a body from the definition and attaches cooked BlockGrid geometry to it.
     *
     * <p>The shape retains the cooked data, so the caller may close the cooking handle after success.
     * A non-null mass override installs explicit mass data on the new body. A null override uses
     * mass derived from the geometry.
     */
    public V3BodyHandle attachBlockGrid(V3BodyDefinition body, V3CookedGrid grid) {
        return attachBlockGrid(body, grid, null);
    }

    /** @see #attachBlockGrid(V3BodyDefinition, V3CookedGrid) */
    public V3BodyHandle attachBlockGrid(V3BodyDefinition body, V3CookedGrid grid, V3MassProperties massOverride) {
        requireOpenOwner();
        Objects.requireNonNull(body, "body");
        Objects.requireNonNull(grid, "grid");
        BodyPlan plan = validateBodyReplacement(List.of(), List.of(new BodyCreation(body.handle(), body.kind())));

        MemorySegment nativeBody = bodyDefinitionBuffer.ensure(1);
        writeBodyDefinition(v3_body_definition.asSlice(nativeBody, 0), body);
        MemorySegment nativeMass = MemorySegment.NULL;
        if (massOverride != null) {
            nativeMass = massBuffer.ensure(1);
            writeMass(v3_mass_properties.asSlice(nativeMass, 0), massOverride);
        }

        int status = V3Abi.v3_world_attach_block_grid(
            nativeWorld,
            nativeBody,
            grid.handle(),
            nativeMass,
            attachedHandleBuffer
        );
        V3NativeLibrary.requireSuccess("attachBlockGrid", status, body.handle().logicalId());
        V3BodyHandle attached = new V3BodyHandle(
            v3_body_handle.logical_id(attachedHandleBuffer),
            v3_body_handle.generation(attachedHandleBuffer)
        );
        if (!attached.equals(body.handle())) {
            throw new IllegalStateException("native attachment published an unexpected body handle");
        }
        applyBodyPlan(plan);
        return attached;
    }

    /**
     * Replaces an attached BlockGrid while preserving the body handle and pose.
     *
     * <p>The shape retains the new geometry, so the caller may close grid after success.
     * Mass derived from geometry is recomputed using the existing shape density. If the derived
     * center of mass moves, linear velocity is adjusted by angular velocity crossed with the
     * center displacement in world coordinates. Explicit mass overrides remain unchanged.
     * Contacts rebuild, so retained cells may emit end and begin events. A rejected call leaves
     * the geometry and body registry unchanged.
     */
    public void replaceBlockGrid(V3BodyHandle body, V3CookedGrid grid) {
        requireOpenOwner();
        Objects.requireNonNull(body, "body");
        Objects.requireNonNull(grid, "grid");
        requireActiveBody(body);
        MemorySegment nativeBody = bodyHandleBuffer.ensure(1);
        writeBodyHandle(v3_body_handle.asSlice(nativeBody, 0), body);
        int status = V3Abi.v3_world_replace_block_grid(nativeWorld, nativeBody, grid.handle());
        V3NativeLibrary.requireSuccess("replaceBlockGrid", status, body.logicalId());
    }

    /**
     * Atomically replaces joints, sharing logical IDs and generations with distance joints.
     * Participants must be active dynamic or kinematic bodies. Either operation can remove either
     * joint kind, and a failed batch preserves the previous joints and body relationships.
     */
    public void replaceRevoluteJoints(List<V3JointHandle> removals, List<V3RevoluteJointCommand> creations) {
        requireOpenOwner();
        List<V3JointHandle> removalValues = snapshot(removals, "removals");
        List<V3RevoluteJointCommand> creationValues = snapshot(creations, "creations");
        JointPlan plan = validateJointReplacement(removalValues, creationValues.stream()
            .map(value -> new JointCreation(value.handle(), value.bodyA(), value.bodyB())).toList());
        MemorySegment nativeRemovals = jointHandleBuffer.ensure(removalValues.size());
        MemorySegment nativeCreations = revoluteCommandBuffer.ensure(creationValues.size());
        for (int index = 0; index < removalValues.size(); index++) {
            writeJointHandle(v3_joint_handle.asSlice(nativeRemovals, index), removalValues.get(index));
        }
        for (int index = 0; index < creationValues.size(); index++) {
            writeRevoluteCommand(v3_revolute_joint_command.asSlice(nativeCreations, index), creationValues.get(index));
        }
        int status = V3Abi.v3_world_replace_revolute_joints(nativeWorld, nativeRemovals, removalValues.size(),
            nativeCreations, creationValues.size());
        V3NativeLibrary.requireSuccess("replaceRevoluteJoints", status,
            detail(removalValues.size(), creationValues.size()));
        applyJointPlan(plan);
    }

    public void replaceDistanceJoints(List<V3JointHandle> removals, List<V3DistanceJointCommand> creations) {
        requireOpenOwner();
        List<V3JointHandle> removalValues = snapshot(removals, "removals");
        List<V3DistanceJointCommand> creationValues = snapshot(creations, "creations");
        JointPlan plan = validateJointReplacement(removalValues, creationValues.stream()
            .map(value -> new JointCreation(value.handle(), value.bodyA(), value.bodyB())).toList());

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

    /**
     * Replaces the world motion limits. The new limits apply from the next step.
     */
    public void setLimits(V3WorldLimits limits) {
        requireOpenOwner();
        Objects.requireNonNull(limits, "limits");
        writeLimits(limitsBuffer, limits);
        V3NativeLibrary.requireSuccess("setLimits", V3Abi.v3_world_set_limits(nativeWorld, limitsBuffer), 0);
    }

    /**
     * Reads the world motion limits as the engine resolved them, so a default maximum linear speed
     * reads back as the concrete engine value rather than the zero sentinel.
     */
    public V3WorldLimits limits() {
        requireOpenOwner();
        V3NativeLibrary.requireSuccess("limits", V3Abi.v3_world_get_limits(nativeWorld, limitsBuffer), 0);
        return new V3WorldLimits(
            v3_world_limits.maximum_linear_speed(limitsBuffer),
            v3_world_limits.maximum_angular_speed(limitsBuffer),
            v3_world_limits.projectile_candidate_cap(limitsBuffer),
            (v3_world_limits.flags(limitsBuffer) & V3Abi.V3_WORLD_DISABLE_SLEEP()) == 0
        );
    }

    static void writeLimits(MemorySegment target, V3WorldLimits value) {
        v3_world_limits.maximum_linear_speed(target, value.maximumLinearSpeed());
        v3_world_limits.maximum_angular_speed(target, value.maximumAngularSpeed());
        v3_world_limits.projectile_candidate_cap(target, value.projectileCandidateCap());
        v3_world_limits.flags(target, value.enableSleep() ? 0 : V3Abi.V3_WORLD_DISABLE_SLEEP());
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
        return new V3StepResult(transforms, queryResults, stats, readBlockContactEvents(stats.blockContactEventCount()));
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
            bodies.clear();
            bodyGenerations.clear();
            joints.clear();
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
        for (BodyCreation creation : creations) {
            V3BodyHandle handle = creation.handle();
            if (!creationIds.add(handle.logicalId())) {
                throw new IllegalArgumentException("body creations contain a duplicate logical ID");
            }
            Integer previousGeneration = bodyGenerations.get(handle.logicalId());
            if (previousGeneration == null) {
                if (handle.generation() != 1) {
                    throw new IllegalArgumentException("a new body must start at generation one");
                }
                continue;
            }
            if (bodies.containsKey(handle.logicalId()) && !removalIds.contains(handle.logicalId())) {
                throw new IllegalArgumentException("body logical ID is already active");
            }
            requireNextGeneration(previousGeneration, handle.generation(), "body");
        }

        int finalCount = activeBodyCount - removals.size() + creations.size();
        if (finalCount > MAX_BODIES) {
            throw new IllegalArgumentException("body replacement exceeds the native limit");
        }
        if (activeBodyCount > MAX_BODIES - creations.size()) {
            throw new IllegalArgumentException("body replacement exceeds the transient native peak");
        }
        return new BodyPlan(removals, creations, finalCount);
    }

    private JointPlan validateJointReplacement(
        List<V3JointHandle> removals,
        List<JointCreation> creations
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
        for (JointCreation creation : creations) {
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
            if (state.kind() != V3BoxBodyCommand.Kind.STATIC) {
                movableCount++;
            }
        }
        return movableCount;
    }

    private BodyState requireActiveBody(V3BodyHandle handle) {
        BodyState state = bodies.get(handle.logicalId());
        if (state == null || state.generation() != handle.generation()) {
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
            bodies.remove(removal.logicalId());
        }
        for (BodyCreation creation : plan.creations()) {
            V3BodyHandle handle = creation.handle();
            bodies.put(handle.logicalId(), new BodyState(handle.generation(), creation.kind()));
            bodyGenerations.put(handle.logicalId(), handle.generation());
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
        for (JointCreation creation : plan.creations()) {
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

    private static void writeBodyDefinition(MemorySegment target, V3BodyDefinition value) {
        writeBodyHandle(v3_body_definition.handle(target), value.handle());
        v3_body_definition.kind(target, value.kind().nativeValue());
        v3_body_definition.flags(target, value.flags());
        v3_body_definition.position_x(target, value.positionX());
        v3_body_definition.position_y(target, value.positionY());
        v3_body_definition.position_z(target, value.positionZ());
        v3_body_definition.rotation_x(target, value.rotationX());
        v3_body_definition.rotation_y(target, value.rotationY());
        v3_body_definition.rotation_z(target, value.rotationZ());
        v3_body_definition.rotation_w(target, value.rotationW());
        v3_body_definition.linear_velocity_x(target, value.linearVelocityX());
        v3_body_definition.linear_velocity_y(target, value.linearVelocityY());
        v3_body_definition.linear_velocity_z(target, value.linearVelocityZ());
        v3_body_definition.angular_velocity_x(target, value.angularVelocityX());
        v3_body_definition.angular_velocity_y(target, value.angularVelocityY());
        v3_body_definition.angular_velocity_z(target, value.angularVelocityZ());
        v3_body_definition.linear_damping(target, value.linearDamping());
        v3_body_definition.angular_damping(target, value.angularDamping());
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

    private static void writeRevoluteCommand(MemorySegment target, V3RevoluteJointCommand value) {
        v3_revolute_joint_command.logical_id(target, value.handle().logicalId());
        v3_revolute_joint_command.generation(target, value.handle().generation());
        v3_revolute_joint_command.flags(target, value.flags());
        writeBodyHandle(v3_revolute_joint_command.body_a(target), value.bodyA());
        writeBodyHandle(v3_revolute_joint_command.body_b(target), value.bodyB());
        v3_revolute_joint_command.local_anchor_a_x(target, value.localAnchorAX());
        v3_revolute_joint_command.local_anchor_a_y(target, value.localAnchorAY());
        v3_revolute_joint_command.local_anchor_a_z(target, value.localAnchorAZ());
        v3_revolute_joint_command.local_rotation_a_x(target, value.localRotationAX());
        v3_revolute_joint_command.local_rotation_a_y(target, value.localRotationAY());
        v3_revolute_joint_command.local_rotation_a_z(target, value.localRotationAZ());
        v3_revolute_joint_command.local_rotation_a_w(target, value.localRotationAW());
        v3_revolute_joint_command.local_anchor_b_x(target, value.localAnchorBX());
        v3_revolute_joint_command.local_anchor_b_y(target, value.localAnchorBY());
        v3_revolute_joint_command.local_anchor_b_z(target, value.localAnchorBZ());
        v3_revolute_joint_command.local_rotation_b_x(target, value.localRotationBX());
        v3_revolute_joint_command.local_rotation_b_y(target, value.localRotationBY());
        v3_revolute_joint_command.local_rotation_b_z(target, value.localRotationBZ());
        v3_revolute_joint_command.local_rotation_b_w(target, value.localRotationBW());
        v3_revolute_joint_command.target_angle(target, value.targetAngle());
        v3_revolute_joint_command.hertz(target, value.hertz());
        v3_revolute_joint_command.damping_ratio(target, value.dampingRatio());
        v3_revolute_joint_command.lower_angle(target, value.lowerAngle());
        v3_revolute_joint_command.upper_angle(target, value.upperAngle());
        v3_revolute_joint_command.max_motor_torque(target, value.maxMotorTorque());
        v3_revolute_joint_command.motor_speed(target, value.motorSpeed());
        v3_revolute_joint_command.reserved0(target, 0);
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
        v3_query.kind(target, value.kind().nativeValue());
        v3_query.flags(target, 0);
        v3_query.category_bits(target, value.categoryBits());
        v3_query.mask_bits(target, value.maskBits());
        v3_query.point_count(target, value.points().size());
        v3_query.radius(target, value.radius());
        MemorySegment pointX = v3_query.point_x(target);
        MemorySegment pointY = v3_query.point_y(target);
        MemorySegment pointZ = v3_query.point_z(target);
        pointX.fill((byte) 0);
        pointY.fill((byte) 0);
        pointZ.fill((byte) 0);
        for (int index = 0; index < value.points().size(); index++) {
            V3HullPoint point = value.points().get(index);
            pointX.setAtIndex(JAVA_FLOAT, index, point.x());
            pointY.setAtIndex(JAVA_FLOAT, index, point.y());
            pointZ.setAtIndex(JAVA_FLOAT, index, point.z());
        }
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
            v3_query_result.fraction(source),
            v3_query_result.point_x(source),
            v3_query_result.point_y(source),
            v3_query_result.point_z(source),
            v3_query_result.normal_x(source),
            v3_query_result.normal_y(source),
            v3_query_result.normal_z(source),
            (v3_query_result.flags(source) & V3Abi.V3_QUERY_RESULT_BLOCK_GRID()) != 0,
            v3_query_result.cell_x(source),
            v3_query_result.cell_y(source),
            v3_query_result.cell_z(source),
            v3_query_result.cell_box_index(source),
            v3_query_result.material_index(source),
            v3_query_result.user_material_id(source),
            v3_query_result.user_data(source)
        );
    }

    private List<V3BlockContactEvent> readBlockContactEvents(int count) {
        if (count == 0) {
            return List.of();
        }
        MemorySegment events = blockContactEventBuffer.ensure(count);
        int status = V3Abi.v3_world_get_block_contact_events(nativeWorld, events, count, blockContactEventCountBuffer);
        V3NativeLibrary.requireSuccess("getBlockContactEvents", status, count);
        if (blockContactEventCountBuffer.get(JAVA_INT, 0) != count) {
            throw new IllegalStateException("native event count does not match step stats");
        }
        List<V3BlockContactEvent> result = new ArrayList<>(count);
        for (int index = 0; index < count; index++) {
            MemorySegment event = v3_block_contact_event.asSlice(events, index);
            result.add(new V3BlockContactEvent(
                V3BlockContactEvent.Kind.fromNative(v3_block_contact_event.flags(event)),
                v3_block_contact_event.fixed_step_index(event),
                readBlockContactSide(v3_block_contact_event.body_a(event), v3_block_contact_event.side_a(event)),
                readBlockContactSide(v3_block_contact_event.body_b(event), v3_block_contact_event.side_b(event)),
                v3_block_contact_event.point_x(event),
                v3_block_contact_event.point_y(event),
                v3_block_contact_event.point_z(event),
                v3_block_contact_event.normal_x(event),
                v3_block_contact_event.normal_y(event),
                v3_block_contact_event.normal_z(event),
                v3_block_contact_event.impulse_x(event),
                v3_block_contact_event.impulse_y(event),
                v3_block_contact_event.impulse_z(event),
                v3_block_contact_event.relative_normal_speed(event)
            ));
        }
        return result;
    }

    private static V3BlockContactSide readBlockContactSide(MemorySegment body, MemorySegment side) {
        int flags = v3_block_contact_side.flags(side);
        if ((flags & ~1) != 0) {
            throw new IllegalStateException("unknown native contact side flags " + flags);
        }
        return new V3BlockContactSide(
            new V3BodyHandle(v3_body_handle.logical_id(body), v3_body_handle.generation(body)),
            (flags & 1) != 0,
            v3_block_contact_side.cell_x(side),
            v3_block_contact_side.cell_y(side),
            v3_block_contact_side.cell_z(side),
            v3_block_contact_side.cell_box_index(side),
            v3_block_contact_side.material_index(side),
            v3_block_contact_side.user_material_id(side),
            v3_block_contact_side.user_data(side)
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
            v3_step_stats.heap_move_pair_count(source),
            v3_step_stats.block_grid_candidate_hitbox_pair_count(source),
            v3_step_stats.block_grid_touching_pair_count(source),
            v3_step_stats.block_grid_contact_count(source),
            v3_step_stats.block_grid_projectile_sweep_count(source),
            v3_step_stats.block_grid_cap_exhaustion_count(source),
            v3_step_stats.block_grid_replacement_published_count(source),
            v3_step_stats.block_grid_scratch_peak_bytes(source),
            v3_step_stats.block_grid_contact_reduction_count(source),
            v3_step_stats.block_contact_event_count(source),
            Integer.toUnsignedLong(v3_step_stats.block_contact_event_dropped_count(source)),
            (v3_step_stats.block_contact_event_flags(source) & 1) != 0
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

    private record BodyState(int generation, V3BoxBodyCommand.Kind kind) {
    }

    private record BodyCreation(V3BodyHandle handle, V3BoxBodyCommand.Kind kind) {
    }

    private record BodyPlan(List<V3BodyHandle> removals, List<BodyCreation> creations, int finalCount) {
    }

    private record JointState(int generation, V3BodyHandle bodyA, V3BodyHandle bodyB, boolean active) {
    }

    private record JointCreation(V3JointHandle handle, V3BodyHandle bodyA, V3BodyHandle bodyB) {
    }

    private record JointPlan(
        List<V3JointHandle> removals,
        List<JointCreation> creations,
        int finalCount
    ) {
    }
}

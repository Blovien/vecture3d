// SPDX-License-Identifier: MIT

#include "internal.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ENSURE( condition )                                                                                                      \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( !( condition ) )                                                                                                    \
		{                                                                                                                        \
			fprintf( stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__, #condition );                                  \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

static v3_box_body_command make_dynamic_box( uint64_t logical_id, double position_x, float velocity_x )
{
	return (v3_box_body_command){
		.logical_id = logical_id,
		.kind = V3_DYNAMIC_BODY,
		.generation = 1,
		.position_x = position_x,
		.rotation_w = 1.0f,
		.linear_velocity_x = velocity_x,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
		.density = 1.0f,
		.friction = 0.5f,
		.flags = V3_BODY_INITIAL_AWAKE,
	};
}

static v3_body_handle body_handle( const v3_box_body_command* body )
{
	return (v3_body_handle){ .logical_id = body->logical_id, .generation = body->generation };
}

static int attach_centered_block_bodies( v3_world* world, const v3_box_body_command* commands, uint32_t count )
{
	v3_block_material material = { .material_id = 1, .density = 1, .friction = 0.5f };
	v3_block_cell cells[8];
	v3_block_box boxes[8];
	uint32_t index = 0;
	// Eight partial cells form exactly [-0.5, 0.5]^3. Explicit mass preserves the ordinary cube dynamics.
	for ( int x = -1; x <= 0; ++x )
	{
		for ( int y = -1; y <= 0; ++y )
		{
			for ( int z = -1; z <= 0; ++z )
			{
				cells[index] = (v3_block_cell){ .x = x, .y = y, .z = z, .feature_id = index + 1 };
				boxes[index] = (v3_block_box){ .owner_cell_index = index,
											   .feature_id = index + 1,
											   .center_x = x == -1 ? 0.75f : 0.25f,
											   .center_y = y == -1 ? 0.75f : 0.25f,
											   .center_z = z == -1 ? 0.75f : 0.25f,
											   .half_extent_x = 0.25f,
											   .half_extent_y = 0.25f,
											   .half_extent_z = 0.25f };
				++index;
			}
		}
	}
	v3_cooked_grid* grid = NULL;
	ENSURE( v3_cook_block_grid( &material, 1, cells, 8, boxes, 8, &grid ) == V3_OK );
	for ( uint32_t i = 0; i < count; ++i )
	{
		const v3_box_body_command* command = commands + i;
		v3_body_definition body = { .handle = body_handle( command ),
									.kind = command->kind,
									.flags = command->flags,
									.position_x = command->position_x,
									.position_y = command->position_y,
									.position_z = command->position_z,
									.rotation_x = command->rotation_x,
									.rotation_y = command->rotation_y,
									.rotation_z = command->rotation_z,
									.rotation_w = command->rotation_w,
									.linear_velocity_x = command->linear_velocity_x,
									.linear_velocity_y = command->linear_velocity_y,
									.linear_velocity_z = command->linear_velocity_z,
									.angular_velocity_x = command->angular_velocity_x,
									.angular_velocity_y = command->angular_velocity_y,
									.angular_velocity_z = command->angular_velocity_z,
									.linear_damping = command->linear_damping,
									.angular_damping = command->angular_damping };
		v3_body_handle handle;
		v3_mass_properties mass = { .mass = 1, .inertia_xx = 1.0f / 6.0f, .inertia_yy = 1.0f / 6.0f, .inertia_zz = 1.0f / 6.0f };
		v3_status status =
			v3_world_attach_block_grid( world, &body, grid, command->kind == V3_DYNAMIC_BODY ? &mass : NULL, &handle );
		if ( status != V3_OK )
		{
			v3_destroy_cooked_grid( grid );
			ENSURE( status == V3_OK );
		}
	}
	v3_destroy_cooked_grid( grid );
	return 0;
}

static v3_distance_joint_command make_coupler( uint64_t logical_id, const v3_box_body_command* body_a,
											   const v3_box_body_command* body_b )
{
	return (v3_distance_joint_command){
		.logical_id = logical_id,
		.generation = 1,
		.body_a = body_handle( body_a ),
		.body_b = body_handle( body_b ),
		.local_anchor_a_x = 0.5f,
		.local_anchor_b_x = -0.5f,
		.rest_length = 2.0f,
		.minimum_length = 1.8f,
		.maximum_length = 2.2f,
		.hertz = 3.0f,
		.damping_ratio = 0.7f,
		.lower_spring_force = -200.0f,
		.upper_spring_force = 200.0f,
		.flags = V3_JOINT_ENABLE_SPRING | V3_JOINT_ENABLE_LIMIT,
	};
}

static int test_invalid_replacement_preserves_accepted_joint_and_bodies( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[2] = {
		make_dynamic_box( UINT64_C( 310 ), -1.5, 0.0f ),
		make_dynamic_box( UINT64_C( 311 ), 1.5, 0.0f ),
	};
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
	v3_distance_joint_command accepted = make_coupler( UINT64_C( 410 ), bodies, bodies + 1 );
	ENSURE( v3_world_replace_distance_joints( world, NULL, 0, &accepted, 1 ) == V3_OK );

	b3Counters before = b3World_GetCounters( world->world_id );
	uint32_t before_mutation_count = world->mutation_batch_count;
	b3Pos before_a = b3Body_GetPosition( world->body_entries[0].body_id );
	b3Pos before_b = b3Body_GetPosition( world->body_entries[1].body_id );
	v3_distance_joint_command malformed = accepted;
	malformed.generation = 2;
	malformed.hertz = NAN;
	v3_joint_handle removal = { .logical_id = accepted.logical_id, .generation = accepted.generation };
	ENSURE( v3_world_replace_distance_joints( world, &removal, 1, &malformed, 1 ) == V3_NON_FINITE );

	b3Counters after = b3World_GetCounters( world->world_id );
	ENSURE( after.bodyCount == before.bodyCount );
	ENSURE( after.jointCount == before.jointCount );
	ENSURE( world->mutation_batch_count == before_mutation_count );
	ENSURE( b3Body_GetPosition( world->body_entries[0].body_id ).x == before_a.x );
	ENSURE( b3Body_GetPosition( world->body_entries[1].body_id ).x == before_b.x );
	ENSURE( v3_world_replace_distance_joints( world, &removal, 1, NULL, 0 ) == V3_OK );
	ENSURE( b3World_GetCounters( world->world_id ).jointCount == 0 );

	v3_world_destroy( world );
	return 0;
}

static int test_distance_limit_and_attached_body_guard( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[2] = {
		make_dynamic_box( UINT64_C( 300 ), -1.5, -4.0f ),
		make_dynamic_box( UINT64_C( 301 ), 1.5, 4.0f ),
	};
	ENSURE( attach_centered_block_bodies( world, bodies, 2 ) == 0 );
	v3_distance_joint_command coupler = make_coupler( UINT64_C( 400 ), bodies, bodies + 1 );
	ENSURE( v3_world_replace_distance_joints( world, NULL, 0, &coupler, 1 ) == V3_OK );

	for ( uint32_t step = 0; step < UINT32_C( 240 ); ++step )
	{
		b3World_Step( world->world_id, 1.0f / 60.0f, 4 );
	}
	b3Pos position_a = b3Body_GetPosition( world->body_entries[0].body_id );
	b3Pos position_b = b3Body_GetPosition( world->body_entries[1].body_id );
	double anchor_distance = fabs( position_b.x - position_a.x ) - 1.0;
	ENSURE( anchor_distance >= 1.75 && anchor_distance <= 2.25 );
	ENSURE( b3World_GetCounters( world->world_id ).jointCount == 1 );

	v3_body_handle attached_body = body_handle( bodies );
	ENSURE( v3_world_replace_box_bodies( world, &attached_body, 1, NULL, 0 ) == V3_JOINT_ATTACHED );
	v3_joint_handle joint = { .logical_id = coupler.logical_id, .generation = coupler.generation };
	ENSURE( v3_world_replace_distance_joints( world, &joint, 1, NULL, 0 ) == V3_OK );
	v3_body_handle removals[2] = { body_handle( bodies ), body_handle( bodies + 1 ) };
	ENSURE( v3_world_replace_box_bodies( world, removals, 2, NULL, 0 ) == V3_OK );

	v3_world_destroy( world );
	return 0;
}

static v3_revolute_joint_command make_hinge( uint64_t id, const v3_box_body_command* a, const v3_box_body_command* b )
{
	return (v3_revolute_joint_command){
		.logical_id = id,
		.generation = 1,
		.body_a = body_handle( a ),
		.body_b = body_handle( b ),
		.local_rotation_a_w = 1.0f,
		.local_rotation_b_w = 1.0f,
		.lower_angle = -1.0f,
		.upper_angle = 1.0f,
		.max_motor_torque = 20.0f,
		.motor_speed = 0.5f,
	};
}

// Observe the downstream Box3D public contract, including distinct frames and disabled settings.
static int test_revolute_definition_and_replacement( void )
{
	v3_world* world = v3_world_create( 0, 0, 0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[] = { make_dynamic_box( 600, -2, 0 ), make_dynamic_box( 601, 2, 0 ) };
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
	v3_revolute_joint_command hinge = make_hinge( 700, bodies, bodies + 1 );
	hinge.local_anchor_a_x = 1;
	hinge.local_anchor_a_y = 2;
	hinge.local_anchor_a_z = 3;
	hinge.local_anchor_b_x = -4;
	hinge.local_anchor_b_y = -5;
	hinge.local_anchor_b_z = -6;
	hinge.local_rotation_a_x = 0.5f;
	hinge.local_rotation_a_y = -0.5f;
	hinge.local_rotation_a_z = 0.5f;
	hinge.local_rotation_a_w = -0.5f;
	hinge.local_rotation_b_x = -0.5f;
	hinge.local_rotation_b_y = 0.5f;
	hinge.local_rotation_b_z = 0.5f;
	hinge.local_rotation_b_w = 0.5f;
	hinge.target_angle = 0.3f;
	hinge.hertz = 2.5f;
	hinge.damping_ratio = 0.7f;
	for ( int enabled = 1; enabled >= 0; --enabled )
	{
		hinge.flags =
			enabled ? V3_JOINT_ENABLE_SPRING | V3_JOINT_ENABLE_LIMIT | V3_JOINT_ENABLE_MOTOR | V3_JOINT_COLLIDE_CONNECTED : 0;
		v3_joint_handle old = { 700, 1, 0 };
		ENSURE( v3_world_replace_revolute_joints( world, enabled ? NULL : &old, enabled ? 0 : 1, &hinge, 1 ) == V3_OK );
		b3JointId joint = world->joint_entries[0].joint_id;
		ENSURE( b3Joint_GetType( joint ) == b3_revoluteJoint );
		b3Transform a = b3Joint_GetLocalFrameA( joint ), b = b3Joint_GetLocalFrameB( joint );
		ENSURE( a.p.x == 1 && a.p.y == 2 && a.p.z == 3 );
		ENSURE( b.p.x == -4 && b.p.y == -5 && b.p.z == -6 );
		ENSURE( a.q.v.x == 0.5f && a.q.v.y == -0.5f && a.q.v.z == 0.5f && a.q.s == -0.5f );
		ENSURE( b.q.v.x == -0.5f && b.q.v.y == 0.5f && b.q.v.z == 0.5f && b.q.s == 0.5f );
		ENSURE( b3RevoluteJoint_GetTargetAngle( joint ) == 0.3f );
		ENSURE( b3RevoluteJoint_GetSpringHertz( joint ) == 2.5f );
		ENSURE( b3RevoluteJoint_GetSpringDampingRatio( joint ) == 0.7f );
		ENSURE( b3RevoluteJoint_GetLowerLimit( joint ) == -1.0f );
		ENSURE( b3RevoluteJoint_GetUpperLimit( joint ) == 1.0f );
		ENSURE( b3RevoluteJoint_GetMaxMotorTorque( joint ) == 20.0f );
		ENSURE( b3RevoluteJoint_GetMotorSpeed( joint ) == 0.5f );
		ENSURE( b3RevoluteJoint_IsSpringEnabled( joint ) == (bool)enabled );
		ENSURE( b3RevoluteJoint_IsLimitEnabled( joint ) == (bool)enabled );
		ENSURE( b3RevoluteJoint_IsMotorEnabled( joint ) == (bool)enabled );
		ENSURE( b3Joint_GetCollideConnected( joint ) == (bool)enabled );
		hinge.generation++;
	}
	v3_world_destroy( world );
	return 0;
}

// The second creation must fail after one real native allocation, before accepted joints are removed.
static int test_revolute_failed_batch_preserves_both_kinds( void )
{
	v3_world* world = v3_world_create( 0, 0, 0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[] = { make_dynamic_box( 600, -2, 0 ), make_dynamic_box( 601, 2, 0 ), make_dynamic_box( 602, 5, 0 ),
									 make_dynamic_box( 603, 8, 0 ) };
	ENSURE( attach_centered_block_bodies( world, bodies, 4 ) == 0 );
	v3_revolute_joint_command hinge = make_hinge( 700, bodies, bodies + 1 );
	v3_distance_joint_command coupler = make_coupler( 701, bodies, bodies + 1 );
	ENSURE( v3_world_replace_revolute_joints( world, NULL, 0, &hinge, 1 ) == V3_OK );
	ENSURE( v3_world_replace_distance_joints( world, NULL, 0, &coupler, 1 ) == V3_OK );
	b3JointId old_hinge = world->joint_entries[0].joint_id, old_coupler = world->joint_entries[1].joint_id;
	v3_transform transforms[4];
	v3_step_stats before, after;
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 0, transforms, 4, NULL, 0, &before ) == V3_OK );
	v3_joint_handle removals[] = { { 700, 1, 0 }, { 701, 1, 0 } };
	v3_revolute_joint_command next[] = { make_hinge( 700, bodies + 2, bodies + 3 ), make_hinge( 701, bodies, bodies + 2 ) };
	next[0].generation = next[1].generation = 2;
	v3_test_fail_after( V3_TEST_FAULT_CREATE_JOINT, 1 );
	ENSURE( v3_world_replace_revolute_joints( world, removals, 2, next, 2 ) == V3_NATIVE_FAILURE );
	ENSURE( b3Joint_IsValid( old_hinge ) && b3Joint_IsValid( old_coupler ) );
	ENSURE( b3RevoluteJoint_GetMotorSpeed( old_hinge ) == 0.5f );
	ENSURE( b3DistanceJoint_GetLength( old_coupler ) == 2.0f );
	ENSURE( b3World_GetCounters( world->world_id ).jointCount == 2 );
	ENSURE( b3Body_GetJointCount( world->body_entries[0].body_id ) == 2 );
	ENSURE( b3Body_GetJointCount( world->body_entries[1].body_id ) == 2 );
	ENSURE( b3Body_GetJointCount( world->body_entries[2].body_id ) == 0 );
	ENSURE( b3Body_GetJointCount( world->body_entries[3].body_id ) == 0 );
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 0, transforms, 4, NULL, 0, &after ) == V3_OK );
	ENSURE( before.joint_count == after.joint_count && before.mutation_batch_count == after.mutation_batch_count );
	v3_body_handle attached = body_handle( bodies ), detached = body_handle( bodies + 3 );
	ENSURE( v3_world_replace_box_bodies( world, &attached, 1, NULL, 0 ) == V3_JOINT_ATTACHED );
	ENSURE( v3_world_replace_box_bodies( world, &detached, 1, NULL, 0 ) == V3_OK );
	// The failed batch must not consume generations. Both kinds can now be removed by this operation.
	ENSURE( v3_world_replace_revolute_joints( world, removals, 2, NULL, 0 ) == V3_OK );
	hinge.generation = 2;
	ENSURE( v3_world_replace_revolute_joints( world, NULL, 0, &hinge, 1 ) == V3_OK );
	v3_world_destroy( world );
	return 0;
}

// A transformed point is p + v + 2*w*(q.xyz cross v) + 2*q.xyz cross (q.xyz cross v).
// This independent quaternion formula does not use the native joint frame or angle accessors.
static void world_anchor( const v3_transform* t, double x, double y, double z, double out[3] )
{
	double qx = t->rotation_x, qy = t->rotation_y, qz = t->rotation_z, w = t->rotation_w;
	double cx = qy * z - qz * y, cy = qz * x - qx * z, cz = qx * y - qy * x;
	out[0] = t->position_x + x + 2 * ( w * cx + qy * cz - qz * cy );
	out[1] = t->position_y + y + 2 * ( w * cy + qz * cx - qx * cz );
	out[2] = t->position_z + z + 2 * ( w * cz + qx * cy - qy * cx );
}

static int test_revolute_moving_anchors_motor_spring_and_limits( void )
{
	v3_world* world = v3_world_create( 0, 0, 0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[] = { make_dynamic_box( 600, 0, 0 ), make_dynamic_box( 601, 0, 0 ) };
	bodies[0].kind = V3_KINEMATIC_BODY;
	bodies[0].density = 0;
	bodies[1].position_y = 2;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
	v3_revolute_joint_command hinge = make_hinge( 700, bodies, bodies + 1 );
	hinge.local_anchor_a_x = hinge.local_anchor_b_x = 0.25f;
	hinge.local_anchor_a_z = hinge.local_anchor_b_z = -0.5f;
	hinge.local_anchor_a_y = 2;
	// A -90 degree frame rotation around x makes local z the world y hinge axis.
	hinge.local_rotation_a_x = hinge.local_rotation_b_x = -sqrtf( 0.5f );
	hinge.local_rotation_a_w = hinge.local_rotation_b_w = sqrtf( 0.5f );
	hinge.flags = V3_JOINT_ENABLE_MOTOR;
	hinge.motor_speed = 0.8f;
	hinge.max_motor_torque = 100;
	v3_transform transforms[2];
	v3_step_stats stats;
	int total_steps = 0;
	for ( int phase = 0; phase < 4; ++phase )
	{
		if ( phase == 1 )
			hinge.motor_speed = -0.8f;
		if ( phase == 2 )
		{
			hinge.flags = V3_JOINT_ENABLE_SPRING;
			hinge.target_angle = 0.6f;
			hinge.hertz = 3;
			hinge.damping_ratio = 1;
		}
		if ( phase == 3 )
		{
			hinge.flags = V3_JOINT_ENABLE_MOTOR | V3_JOINT_ENABLE_LIMIT;
			hinge.motor_speed = 0.8f;
			hinge.lower_angle = -0.2f;
			hinge.upper_angle = 0.4f;
		}
		v3_joint_handle old = { 700, hinge.generation - 1, 0 };
		ENSURE( v3_world_replace_revolute_joints( world, phase == 0 ? NULL : &old, phase == 0 ? 0 : 1, &hinge, 1 ) == V3_OK );
		int steps = phase == 2 ? 180 : 60;
		for ( int i = 0; i < steps; ++i )
		{
			++total_steps;
			v3_kinematic_target target = {
				.logical_id = 600, .generation = 1, .flags = V3_TARGET_WAKE, .position_x = total_steps / 120.0, .rotation_w = 1 };
			ENSURE( v3_world_step_and_read( world, &target, 1, NULL, 0, NULL, 0, 1, transforms, 2, NULL, 0, &stats ) == V3_OK );
			double a[3], b[3];
			world_anchor( transforms, 0.25, 2, -0.5, a );
			world_anchor( transforms + 1, 0.25, 0, -0.5, b );
			// 3 cm allows transient solver error at 60 Hz, far below the 3 m hull travel.
			ENSURE( fabs( a[0] - b[0] ) < 0.03 && fabs( a[1] - b[1] ) < 0.03 && fabs( a[2] - b[2] ) < 0.03 );
			ENSURE( fabs( transforms[1].rotation_x ) < 0.005 && fabs( transforms[1].rotation_z ) < 0.005 );
		}
		double angle = 2 * atan2( transforms[1].rotation_y, transforms[1].rotation_w );
		// One second at +/-0.8 rad/s gives 0.8, then zero; spring and limit have separate targets.
		double expected[] = { 0.8, 0, 0.6, 0.4 };
		ENSURE( fabs( angle - expected[phase] ) < 0.06 );
		hinge.generation++;
	}
	ENSURE( fabs( transforms[0].position_x - 3.0 ) < 0.001 );
	v3_world_destroy( world );
	return 0;
}

static int test_revolute_validation_and_shared_generations( void )
{
	v3_world* world = v3_world_create( 0, 0, 0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[] = { make_dynamic_box( 600, -2, 0 ), make_dynamic_box( 601, 2, 0 ),
									 make_dynamic_box( 602, 5, 0 ) };
	bodies[2].kind = V3_STATIC_BODY;
	bodies[2].density = 0;
	ENSURE( attach_centered_block_bodies( world, bodies, 3 ) == 0 );
	v3_distance_joint_command distance = make_coupler( 700, bodies, bodies + 1 );
	ENSURE( v3_world_replace_distance_joints( world, NULL, 0, &distance, 1 ) == V3_OK );
	v3_revolute_joint_command hinge = make_hinge( 700, bodies, bodies + 1 );
	ENSURE( v3_world_replace_revolute_joints( world, NULL, 0, &hinge, 1 ) == V3_DUPLICATE_ID );
	v3_joint_handle old = { 700, 1, 0 };
	ENSURE( v3_world_replace_revolute_joints( world, &old, 1, &hinge, 1 ) == V3_INVALID_GENERATION );
	hinge.generation = 2;
	ENSURE( v3_world_replace_revolute_joints( world, &old, 1, &hinge, 1 ) == V3_OK );
	ENSURE( v3_world_replace_distance_joints( world, &old, 1, NULL, 0 ) == V3_STALE_HANDLE );
	old.generation = 2;
	v3_revolute_joint_command valid = hinge;
	valid.generation = 3;
	b3JointId accepted = world->joint_entries[0].joint_id;
	uint32_t mutations = world->mutation_batch_count;

#define REJECT( field, value, expected )                                                                                         \
	do                                                                                                                           \
	{                                                                                                                            \
		v3_revolute_joint_command bad = valid;                                                                                   \
		bad.field = ( value );                                                                                                   \
		ENSURE( v3_world_replace_revolute_joints( world, &old, 1, &bad, 1 ) == ( expected ) );                                   \
	}                                                                                                                            \
	while ( 0 )
	REJECT( logical_id, 0, V3_INVALID_ARGUMENT );
	REJECT( generation, 0, V3_INVALID_GENERATION );
	REJECT( generation, UINT32_MAX, V3_INVALID_GENERATION );
	REJECT( generation, 4, V3_INVALID_GENERATION );
	REJECT( reserved0, 1, V3_INVALID_ARGUMENT );
	REJECT( flags, 16, V3_INVALID_ARGUMENT );
	REJECT( body_a.reserved, 1, V3_INVALID_ARGUMENT );
	REJECT( body_b.reserved, 1, V3_INVALID_ARGUMENT );
	REJECT( body_a.logical_id, 0, V3_INVALID_ARGUMENT );
	REJECT( body_b.logical_id, 999, V3_STALE_HANDLE );
	REJECT( body_a.generation, 0, V3_INVALID_ARGUMENT );
	REJECT( body_b.generation, UINT32_MAX, V3_INVALID_ARGUMENT );
	REJECT( body_a.generation, 2, V3_STALE_HANDLE );
	REJECT( body_b.generation, 2, V3_STALE_HANDLE );
	REJECT( body_b.logical_id, 600, V3_INVALID_ARGUMENT );
	REJECT( body_b.logical_id, 602, V3_INVALID_ARGUMENT );
	REJECT( local_rotation_a_w, 0, V3_INVALID_QUATERNION );
	REJECT( local_rotation_b_w, 1.01f, V3_INVALID_QUATERNION );
	REJECT( local_rotation_a_x, INFINITY, V3_INVALID_QUATERNION );
	REJECT( local_rotation_b_y, NAN, V3_INVALID_QUATERNION );
	REJECT( target_angle, nextafterf( B3_PI, INFINITY ), V3_INVALID_ARGUMENT );
	REJECT( target_angle, nextafterf( -B3_PI, -INFINITY ), V3_INVALID_ARGUMENT );
	REJECT( lower_angle, nextafterf( -0.99f * B3_PI, -INFINITY ), V3_INVALID_ARGUMENT );
	REJECT( upper_angle, nextafterf( 0.99f * B3_PI, INFINITY ), V3_INVALID_ARGUMENT );
	REJECT( lower_angle, 1.1f, V3_INVALID_ARGUMENT );
	REJECT( max_motor_torque, -0.01f, V3_INVALID_ARGUMENT );
	REJECT( hertz, -0.01f, V3_INVALID_DAMPING );
	REJECT( damping_ratio, -0.01f, V3_INVALID_DAMPING );
#undef REJECT
	const size_t finite_fields[] = {
		offsetof( v3_revolute_joint_command, local_anchor_a_x ), offsetof( v3_revolute_joint_command, local_anchor_a_y ),
		offsetof( v3_revolute_joint_command, local_anchor_a_z ), offsetof( v3_revolute_joint_command, local_anchor_b_x ),
		offsetof( v3_revolute_joint_command, local_anchor_b_y ), offsetof( v3_revolute_joint_command, local_anchor_b_z ),
		offsetof( v3_revolute_joint_command, target_angle ),	 offsetof( v3_revolute_joint_command, hertz ),
		offsetof( v3_revolute_joint_command, damping_ratio ),	 offsetof( v3_revolute_joint_command, lower_angle ),
		offsetof( v3_revolute_joint_command, upper_angle ),		 offsetof( v3_revolute_joint_command, max_motor_torque ),
		offsetof( v3_revolute_joint_command, motor_speed ),
	};
	for ( size_t i = 0; i < sizeof( finite_fields ) / sizeof( finite_fields[0] ); ++i )
	{
		v3_revolute_joint_command bad = valid;
		float invalid = i % 2 ? INFINITY : NAN;
		memcpy( (char*)&bad + finite_fields[i], &invalid, sizeof( invalid ) );
		ENSURE( v3_world_replace_revolute_joints( world, &old, 1, &bad, 1 ) == V3_NON_FINITE );
	}
	ENSURE( v3_world_replace_revolute_joints( NULL, NULL, 0, NULL, 0 ) == V3_INVALID_ARGUMENT );
	ENSURE( v3_world_replace_revolute_joints( world, NULL, 1, NULL, 0 ) == V3_INVALID_ARGUMENT );
	ENSURE( v3_world_replace_revolute_joints( world, NULL, 0, NULL, 1 ) == V3_INVALID_ARGUMENT );
	ENSURE( v3_world_replace_revolute_joints( world, &old, 4097, NULL, 0 ) == V3_LIMIT_EXCEEDED );
	ENSURE( v3_world_replace_revolute_joints( world, NULL, 0, &valid, 4097 ) == V3_LIMIT_EXCEEDED );
	v3_joint_handle malformed = old;
	malformed.reserved = 1;
	ENSURE( v3_world_replace_revolute_joints( world, &malformed, 1, NULL, 0 ) == V3_INVALID_ARGUMENT );
	v3_joint_handle duplicate[] = { old, old };
	ENSURE( v3_world_replace_revolute_joints( world, duplicate, 2, NULL, 0 ) == V3_DUPLICATE_ID );
	v3_revolute_joint_command batch[] = { valid, valid };
	ENSURE( v3_world_replace_revolute_joints( world, &old, 1, batch, 2 ) == V3_DUPLICATE_ID );
	batch[1].logical_id = 701;
	batch[1].generation = 1;
	batch[1].motor_speed = NAN;
	// A queued fault remains armed after invalid command 2, proving no command reached native creation.
	v3_test_fail_after( V3_TEST_FAULT_CREATE_JOINT, 0 );
	ENSURE( v3_world_replace_revolute_joints( world, &old, 1, batch, 2 ) == V3_NON_FINITE );
	ENSURE( v3_world_replace_revolute_joints( world, &old, 1, &valid, 1 ) == V3_NATIVE_FAILURE );
	ENSURE( world->mutation_batch_count == mutations && b3Joint_IsValid( accepted ) );
	ENSURE( b3World_GetCounters( world->world_id ).jointCount == 1 );
	v3_body_handle attached = body_handle( bodies );
	ENSURE( v3_world_replace_box_bodies( world, &attached, 1, NULL, 0 ) == V3_JOINT_ATTACHED );
	// Exact native bounds and zero settings are accepted, even without enable flags.
	valid.target_angle = B3_PI;
	valid.lower_angle = -0.99f * B3_PI;
	valid.upper_angle = 0.99f * B3_PI;
	valid.max_motor_torque = valid.hertz = valid.damping_ratio = 0;
	ENSURE( v3_world_replace_revolute_joints( world, &old, 1, &valid, 1 ) == V3_OK );
	old.generation = 3;
	distance.generation = 4;
	distance.flags |= V3_JOINT_ENABLE_MOTOR;
	ENSURE( v3_world_replace_distance_joints( world, &old, 1, &distance, 1 ) == V3_INVALID_ARGUMENT );
	distance.flags = V3_JOINT_COLLIDE_CONNECTED;
	ENSURE( v3_world_replace_distance_joints( world, &old, 1, &distance, 1 ) == V3_INVALID_ARGUMENT );
	distance.flags = 0;
	ENSURE( v3_world_replace_distance_joints( world, &old, 1, &distance, 1 ) == V3_OK );
	old.generation = 4;
	ENSURE( v3_world_replace_revolute_joints( world, &old, 1, NULL, 0 ) == V3_OK );
	ENSURE( v3_world_replace_box_bodies( world, &attached, 1, NULL, 0 ) == V3_OK );
	valid.logical_id = 701;
	valid.generation = 1;
	ENSURE( v3_world_replace_revolute_joints( world, NULL, 0, &valid, 1 ) == V3_STALE_HANDLE );
	v3_world_destroy( world );
	return 0;
}

static int test_revolute_connected_collision_and_kinematic_participants( void )
{
	for ( int collide = 0; collide <= 1; ++collide )
	{
		v3_world* world = v3_world_create( 0, 0, 0, NULL );
		ENSURE( world != NULL );
		v3_box_body_command bodies[] = { make_dynamic_box( 600, 0, 0 ), make_dynamic_box( 601, 0.75, 0 ) };
		ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
		v3_revolute_joint_command hinge = make_hinge( 700, bodies, bodies + 1 );
		hinge.local_anchor_a_x = 0.375f;
		hinge.local_anchor_b_x = -0.375f;
		hinge.flags = collide ? V3_JOINT_COLLIDE_CONNECTED : 0;
		ENSURE( v3_world_replace_revolute_joints( world, NULL, 0, &hinge, 1 ) == V3_OK );
		v3_transform transforms[2];
		v3_step_stats stats;
		for ( int step = 0; step < 10; ++step )
		{
			ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, transforms, 2, NULL, 0, &stats ) == V3_OK );
		}
		// Overlapping unit boxes provide a positive collision control after broad phase discovery.
		ENSURE( collide ? stats.contact_count > 0 : stats.contact_count == 0 );
		v3_world_destroy( world );
	}
	// Preserve distance's participant contract: two kinematic bodies may retain a joint,
	// although neither can be moved by its constraint impulses.
	v3_world* world = v3_world_create( 0, 0, 0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[] = { make_dynamic_box( 600, 0, 0 ), make_dynamic_box( 601, 2, 0 ) };
	bodies[0].kind = bodies[1].kind = V3_KINEMATIC_BODY;
	bodies[0].density = bodies[1].density = 0;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
	v3_revolute_joint_command hinge = make_hinge( 700, bodies, bodies + 1 );
	ENSURE( v3_world_replace_revolute_joints( world, NULL, 0, &hinge, 1 ) == V3_OK );
	v3_joint_handle removal = { 700, 1, 0 };
	ENSURE( v3_world_replace_revolute_joints( world, &removal, 1, NULL, 0 ) == V3_OK );
	v3_body_handle removals[] = { body_handle( bodies ), body_handle( bodies + 1 ) };
	ENSURE( v3_world_replace_box_bodies( world, removals, 2, NULL, 0 ) == V3_OK );
	v3_world_destroy( world );
	return 0;
}

int main( void )
{
	uint32_t baseline = v3_active_world_count();
	ENSURE( test_invalid_replacement_preserves_accepted_joint_and_bodies() == 0 );
	ENSURE( test_distance_limit_and_attached_body_guard() == 0 );
	ENSURE( test_revolute_definition_and_replacement() == 0 );
	ENSURE( test_revolute_failed_batch_preserves_both_kinds() == 0 );
	ENSURE( test_revolute_moving_anchors_motor_spring_and_limits() == 0 );
	ENSURE( test_revolute_validation_and_shared_generations() == 0 );
	ENSURE( test_revolute_connected_collision_and_kinematic_participants() == 0 );
	ENSURE( v3_active_world_count() == baseline );
	puts( "v3 joint tests passed" );
	return 0;
}

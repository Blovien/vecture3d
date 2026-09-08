// SPDX-License-Identifier: MIT

#include "vecture3d/abi.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// Independent C layout contract agreed for the Java/native record.
_Static_assert( sizeof( v3_revolute_joint_command ) == 136, "revolute command size" );
_Static_assert( _Alignof( v3_revolute_joint_command ) == 8, "revolute command alignment" );
#define REVOLUTE_FIELD( field, offset, size )                                                                                    \
	_Static_assert( offsetof( v3_revolute_joint_command, field ) == offset, "revolute " #field " offset" );                      \
	_Static_assert( sizeof( ( (v3_revolute_joint_command*)0 )->field ) == size, "revolute " #field " size" )
REVOLUTE_FIELD( logical_id, 0, 8 );
REVOLUTE_FIELD( generation, 8, 4 );
REVOLUTE_FIELD( flags, 12, 4 );
REVOLUTE_FIELD( body_a, 16, 16 );
REVOLUTE_FIELD( body_b, 32, 16 );
REVOLUTE_FIELD( local_anchor_a_x, 48, 4 );
REVOLUTE_FIELD( local_anchor_a_y, 52, 4 );
REVOLUTE_FIELD( local_anchor_a_z, 56, 4 );
REVOLUTE_FIELD( local_rotation_a_x, 60, 4 );
REVOLUTE_FIELD( local_rotation_a_y, 64, 4 );
REVOLUTE_FIELD( local_rotation_a_z, 68, 4 );
REVOLUTE_FIELD( local_rotation_a_w, 72, 4 );
REVOLUTE_FIELD( local_anchor_b_x, 76, 4 );
REVOLUTE_FIELD( local_anchor_b_y, 80, 4 );
REVOLUTE_FIELD( local_anchor_b_z, 84, 4 );
REVOLUTE_FIELD( local_rotation_b_x, 88, 4 );
REVOLUTE_FIELD( local_rotation_b_y, 92, 4 );
REVOLUTE_FIELD( local_rotation_b_z, 96, 4 );
REVOLUTE_FIELD( local_rotation_b_w, 100, 4 );
REVOLUTE_FIELD( target_angle, 104, 4 );
REVOLUTE_FIELD( hertz, 108, 4 );
REVOLUTE_FIELD( damping_ratio, 112, 4 );
REVOLUTE_FIELD( lower_angle, 116, 4 );
REVOLUTE_FIELD( upper_angle, 120, 4 );
REVOLUTE_FIELD( max_motor_torque, 124, 4 );
REVOLUTE_FIELD( motor_speed, 128, 4 );
REVOLUTE_FIELD( reserved0, 132, 4 );
#undef REVOLUTE_FIELD

#define V3_TEST_BODY_LIMIT UINT32_C( 4096 )

_Static_assert( sizeof( v3_step_stats ) == 168, "v3_step_stats size" );
_Static_assert( offsetof( v3_step_stats, block_contact_event_count ) == 152, "v3_step_stats event count offset" );
_Static_assert( offsetof( v3_step_stats, block_contact_event_dropped_count ) == 156, "v3_step_stats dropped offset" );
_Static_assert( offsetof( v3_step_stats, block_contact_event_flags ) == 160, "v3_step_stats event flags offset" );
_Static_assert( sizeof( v3_block_contact_side ) == 40, "v3_block_contact_side size" );
_Static_assert( offsetof( v3_block_contact_side, cell_x ) == 0, "v3_block_contact_side cell offset" );
_Static_assert( offsetof( v3_block_contact_side, material_index ) == 16, "v3_block_contact_side material offset" );
_Static_assert( offsetof( v3_block_contact_side, flags ) == 20, "v3_block_contact_side flags offset" );
_Static_assert( offsetof( v3_block_contact_side, user_material_id ) == 24, "v3_block_contact_side material ID offset" );
_Static_assert( offsetof( v3_block_contact_side, user_data ) == 32, "v3_block_contact_side data offset" );
_Static_assert( sizeof( v3_block_contact_event ) == 176, "v3_block_contact_event size" );
_Static_assert( offsetof( v3_block_contact_event, body_a ) == 0, "v3_block_contact_event body offset" );
_Static_assert( offsetof( v3_block_contact_event, feature_id_a ) == 32, "v3_block_contact_event feature offset" );
_Static_assert( offsetof( v3_block_contact_event, point_x ) == 48, "v3_block_contact_event point offset" );
_Static_assert( offsetof( v3_block_contact_event, normal_x ) == 60, "v3_block_contact_event normal offset" );
_Static_assert( offsetof( v3_block_contact_event, impulse_x ) == 72, "v3_block_contact_event impulse offset" );
_Static_assert( offsetof( v3_block_contact_event, relative_normal_speed ) == 84, "v3_block_contact_event speed offset" );
_Static_assert( offsetof( v3_block_contact_event, flags ) == 88, "v3_block_contact_event flags offset" );
_Static_assert( offsetof( v3_block_contact_event, fixed_step_index ) == 92, "v3_block_contact_event step offset" );
_Static_assert( offsetof( v3_block_contact_event, side_a ) == 96, "v3_block_contact_event side A offset" );
_Static_assert( offsetof( v3_block_contact_event, side_b ) == 136, "v3_block_contact_event side B offset" );

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

static v3_box_body_command make_static_box( uint64_t logical_id, uint32_t generation )
{
	return (v3_box_body_command){
		.logical_id = logical_id,
		.kind = V3_STATIC_BODY,
		.generation = generation,
		.rotation_w = 1.0f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
		.friction = 0.5f,
	};
}

static v3_body_handle make_handle( uint64_t logical_id, uint32_t generation )
{
	return (v3_body_handle){
		.logical_id = logical_id,
		.generation = generation,
	};
}

static int test_world_sleep_limits_are_consistent_and_atomic( void )
{
	v3_world_limits enabled = { 0.0f, 0.0f, 0, 0 };
	v3_world_limits disabled = { 120.0f, 6.0f, 32, V3_WORLD_DISABLE_SLEEP };
	v3_world_limits actual = { 0 };

	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, &disabled );
	ENSURE( world != NULL );
	ENSURE( v3_world_get_limits( world, &actual ) == V3_OK );
	ENSURE( actual.flags == V3_WORLD_DISABLE_SLEEP );

	ENSURE( v3_world_set_limits( world, &enabled ) == V3_OK );
	ENSURE( v3_world_get_limits( world, &actual ) == V3_OK );
	ENSURE( actual.flags == 0 );

	ENSURE( v3_world_set_limits( world, &disabled ) == V3_OK );
	ENSURE( v3_world_get_limits( world, &actual ) == V3_OK );
	ENSURE( actual.maximum_linear_speed == disabled.maximum_linear_speed );
	ENSURE( actual.maximum_angular_speed == disabled.maximum_angular_speed );
	ENSURE( actual.projectile_candidate_cap == disabled.projectile_candidate_cap );
	ENSURE( actual.flags == V3_WORLD_DISABLE_SLEEP );

	v3_world_limits invalid = disabled;
	invalid.flags = UINT32_C( 2 );
	ENSURE( v3_world_set_limits( world, &invalid ) == V3_INVALID_ARGUMENT );
	ENSURE( v3_world_get_limits( world, &actual ) == V3_OK );
	ENSURE( actual.maximum_linear_speed == disabled.maximum_linear_speed );
	ENSURE( actual.maximum_angular_speed == disabled.maximum_angular_speed );
	ENSURE( actual.projectile_candidate_cap == disabled.projectile_candidate_cap );
	ENSURE( actual.flags == V3_WORLD_DISABLE_SLEEP );
	ENSURE( v3_world_create( 0.0, -9.81, 0.0, &invalid ) == NULL );

	v3_world_destroy( world );
	return 0;
}

static int test_native_linear_speed_limit_bounds_motion( void )
{
	v3_world_limits limits = { 12.0f, 0.0f, 0, 0 };
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, &limits );
	ENSURE( world != NULL );

	v3_box_body_command body = {
		.logical_id = UINT64_C( 901 ),
		.kind = V3_DYNAMIC_BODY,
		.generation = 1,
		.rotation_w = 1.0f,
		.linear_velocity_x = 120.0f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
		.density = 1.0f,
		.friction = 0.5f,
		.flags = V3_BODY_INITIAL_AWAKE,
	};
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &body, 1 ) == V3_OK );

	v3_transform transform = { 0 };
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.output_count == 1 );
	ENSURE( transform.position_x > 0.0 && transform.position_x <= limits.maximum_linear_speed / 60.0 + 1.0e-4 );

	v3_world_destroy( world );
	return 0;
}

static int test_world_lifetime( void )
{
	uint32_t baseline = v3_active_world_count();
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );
	ENSURE( v3_active_world_count() == baseline + 1u );

	v3_world_destroy( world );
	ENSURE( v3_active_world_count() == baseline );
	v3_world_destroy( NULL );
	ENSURE( v3_active_world_count() == baseline );
	return 0;
}

static int test_empty_block_contact_getter_validates_outputs( void )
{
	uint32_t count = UINT32_MAX;
	ENSURE( v3_world_get_block_contact_events( NULL, NULL, 0, &count ) == V3_INVALID_ARGUMENT );
	ENSURE( count == UINT32_MAX );
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );
	ENSURE( v3_world_get_block_contact_events( world, NULL, 0, NULL ) == V3_INVALID_ARGUMENT );
	ENSURE( v3_world_get_block_contact_events( world, NULL, 0, &count ) == V3_OK );
	ENSURE( count == 0 );
	v3_world_destroy( world );
	return 0;
}

static int test_atomic_validation( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );

	v3_box_body_command batch[2] = {
		make_static_box( UINT64_C( 101 ), UINT32_C( 1 ) ),
		make_static_box( UINT64_C( 102 ), UINT32_C( 1 ) ),
	};
	batch[1].rotation_w = 0.0f;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, batch, 2 ) == V3_INVALID_QUATERNION );

	v3_box_body_command accepted = make_static_box( UINT64_C( 101 ), UINT32_C( 1 ) );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &accepted, 1 ) == V3_OK );
	v3_body_handle removal = make_handle( accepted.logical_id, accepted.generation );
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, NULL, 0 ) == V3_OK );

	v3_box_body_command invalid = make_static_box( UINT64_C( 103 ), UINT32_C( 1 ) );
	invalid.position_x = NAN;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &invalid, 1 ) == V3_NON_FINITE );
	invalid = make_static_box( UINT64_C( 103 ), UINT32_C( 1 ) );
	invalid.half_extent_x = 0.0f;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &invalid, 1 ) == V3_INVALID_DIMENSION );
	invalid = make_static_box( UINT64_C( 103 ), UINT32_C( 1 ) );
	invalid.kind = V3_DYNAMIC_BODY;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &invalid, 1 ) == V3_INVALID_DENSITY );
	invalid = make_static_box( UINT64_C( 103 ), UINT32_C( 1 ) );
	invalid.friction = -0.1f;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &invalid, 1 ) == V3_INVALID_FRICTION );
	invalid = make_static_box( UINT64_C( 103 ), UINT32_C( 1 ) );
	invalid.linear_damping = 10.1f;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &invalid, 1 ) == V3_INVALID_DAMPING );

	v3_world_destroy( world );
	return 0;
}

static int test_duplicate_ids( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );

	v3_box_body_command duplicates[2] = {
		make_static_box( UINT64_C( 201 ), UINT32_C( 1 ) ),
		make_static_box( UINT64_C( 201 ), UINT32_C( 1 ) ),
	};
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, duplicates, 2 ) == V3_DUPLICATE_ID );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, duplicates, 1 ) == V3_OK );

	v3_body_handle removals[2] = {
		make_handle( UINT64_C( 201 ), UINT32_C( 1 ) ),
		make_handle( UINT64_C( 201 ), UINT32_C( 1 ) ),
	};
	ENSURE( v3_world_replace_box_bodies( world, removals, 2, NULL, 0 ) == V3_DUPLICATE_ID );
	ENSURE( v3_world_replace_box_bodies( world, removals, 1, NULL, 0 ) == V3_OK );

	v3_world_destroy( world );
	return 0;
}

static int test_generations_and_stale_removals( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );

	v3_box_body_command command = make_static_box( UINT64_C( 301 ), UINT32_C( 2 ) );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &command, 1 ) == V3_INVALID_GENERATION );
	command.generation = 1;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &command, 1 ) == V3_OK );

	v3_body_handle removal = make_handle( command.logical_id, UINT32_C( 2 ) );
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, NULL, 0 ) == V3_STALE_HANDLE );
	removal.generation = 1;
	command.generation = 3;
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, &command, 1 ) == V3_INVALID_GENERATION );
	command.generation = 2;
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, &command, 1 ) == V3_OK );

	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, NULL, 0 ) == V3_STALE_HANDLE );
	removal.generation = 2;
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, NULL, 0 ) == V3_OK );

	v3_world_destroy( world );
	return 0;
}

static int test_exact_body_and_logical_entry_limits( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );

	v3_box_body_command* creations = calloc( V3_TEST_BODY_LIMIT, sizeof( *creations ) );
	v3_body_handle* removals = calloc( V3_TEST_BODY_LIMIT, sizeof( *removals ) );
	ENSURE( creations != NULL );
	ENSURE( removals != NULL );
	for ( uint32_t index = 0; index < V3_TEST_BODY_LIMIT; ++index )
	{
		uint64_t logical_id = UINT64_C( 10000 ) + index;
		creations[index] = make_static_box( logical_id, UINT32_C( 1 ) );
		creations[index].position_x = 2.0 * index;
		removals[index] = make_handle( logical_id, UINT32_C( 1 ) );
	}

	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, creations, V3_TEST_BODY_LIMIT ) == V3_OK );
	ENSURE( v3_world_replace_box_bodies( world, removals, V3_TEST_BODY_LIMIT, NULL, 0 ) == V3_OK );

	v3_box_body_command new_id = make_static_box( UINT64_C( 20000 ), UINT32_C( 1 ) );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &new_id, 1 ) == V3_LIMIT_EXCEEDED );

	free( removals );
	free( creations );
	v3_world_destroy( world );
	return 0;
}

static int test_transient_peak_limit( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );

	v3_box_body_command initial = make_static_box( UINT64_C( 40001 ), UINT32_C( 1 ) );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &initial, 1 ) == V3_OK );

	v3_box_body_command* creations = calloc( V3_TEST_BODY_LIMIT, sizeof( *creations ) );
	ENSURE( creations != NULL );
	creations[0] = make_static_box( initial.logical_id, UINT32_C( 2 ) );
	for ( uint32_t index = 1; index < V3_TEST_BODY_LIMIT; ++index )
	{
		creations[index] = make_static_box( UINT64_C( 40001 ) + index, UINT32_C( 1 ) );
	}
	v3_body_handle removal = make_handle( initial.logical_id, UINT32_C( 1 ) );
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, creations, V3_TEST_BODY_LIMIT ) == V3_PEAK_LIMIT_EXCEEDED );
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, NULL, 0 ) == V3_OK );

	free( creations );
	v3_world_destroy( world );
	return 0;
}

static int test_replacement_churn( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );

	v3_box_body_command command = make_static_box( UINT64_C( 50001 ), UINT32_C( 1 ) );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &command, 1 ) == V3_OK );
	for ( uint32_t cycle = 0; cycle < UINT32_C( 1000 ); ++cycle )
	{
		v3_body_handle removal = make_handle( command.logical_id, command.generation );
		command.generation += 1;
		ENSURE( v3_world_replace_box_bodies( world, &removal, 1, &command, 1 ) == V3_OK );
	}

	v3_body_handle removal = make_handle( command.logical_id, command.generation );
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, NULL, 0 ) == V3_OK );
	v3_world_destroy( world );
	return 0;
}

int main( void )
{
	uint32_t baseline = v3_active_world_count();
	ENSURE( test_world_sleep_limits_are_consistent_and_atomic() == 0 );
	ENSURE( test_native_linear_speed_limit_bounds_motion() == 0 );
	ENSURE( test_world_lifetime() == 0 );
	ENSURE( test_empty_block_contact_getter_validates_outputs() == 0 );
	ENSURE( test_atomic_validation() == 0 );
	ENSURE( test_duplicate_ids() == 0 );
	ENSURE( test_generations_and_stale_removals() == 0 );
	ENSURE( test_exact_body_and_logical_entry_limits() == 0 );
	ENSURE( test_transient_peak_limit() == 0 );
	ENSURE( test_replacement_churn() == 0 );
	ENSURE( v3_active_world_count() == baseline );
	puts( "v3 world tests passed" );
	return 0;
}

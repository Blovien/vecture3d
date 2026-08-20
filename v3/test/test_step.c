// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define V3_TEST_DENSE_CHILD_COUNT UINT32_C( 32 )
#define V3_TEST_TARGET_WAKE UINT32_C( 1 )

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

static v3_box_body_command make_box( uint64_t logical_id, uint32_t kind, double x, double y, double z )
{
	return (v3_box_body_command){
		.logical_id = logical_id,
		.kind = kind,
		.generation = 1,
		.position_x = x,
		.position_y = y,
		.position_z = z,
		.rotation_w = 1.0f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
		.density = kind == V3_DYNAMIC_BODY ? 1.0f : 0.0f,
		.friction = 0.5f,
		.flags = V3_BODY_INITIAL_AWAKE,
	};
}

static const v3_transform* find_transform( const v3_transform* transforms, uint32_t count, uint64_t logical_id )
{
	for ( uint32_t index = 0; index < count; ++index )
	{
		if ( transforms[index].logical_id == logical_id )
		{
			return transforms + index;
		}
	}
	return NULL;
}

static v3_kinematic_target make_target( const v3_box_body_command* body, double x )
{
	return (v3_kinematic_target){
		.logical_id = body->logical_id,
		.generation = body->generation,
		.flags = V3_TEST_TARGET_WAKE,
		.position_x = x,
		.position_y = body->position_y,
		.position_z = body->position_z,
		.rotation_w = 1.0f,
	};
}

static v3_body_wrench make_wrench( const v3_box_body_command* body, float force_x )
{
	return (v3_body_wrench){
		.logical_id = body->logical_id,
		.generation = body->generation,
		.flags = V3_WRENCH_WAKE,
		.force_x = force_x,
	};
}

static v3_query make_query( uint64_t query_id, double origin_x, float translation_x )
{
	return (v3_query){
		.query_id = query_id,
		.origin_x = origin_x,
		.half_extent_x = 0.25f,
		.half_extent_y = 0.25f,
		.half_extent_z = 0.25f,
		.translation_x = translation_x,
	};
}

static int read_transforms( v3_world* world, v3_transform* transforms, uint32_t capacity, v3_step_stats* stats )
{
	return (int)v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 0, transforms, capacity, NULL, 0, stats );
}

static int ensure_transforms_match( v3_world* world, const v3_transform* expected, uint32_t count )
{
	v3_transform actual[4] = { 0 };
	v3_step_stats stats = { 0 };
	ENSURE( count <= 4 );
	ENSURE( read_transforms( world, actual, count, &stats ) == V3_OK );
	ENSURE( stats.output_count == count );
	ENSURE( memcmp( actual, expected, count * sizeof( *actual ) ) == 0 );
	return 0;
}

static int test_exact_fixed_step_counts( void )
{
	for ( uint32_t fixed_steps = 0; fixed_steps <= 4; ++fixed_steps )
	{
		v3_world* world = v3_world_create( 0.0, 0.0, 0.0 );
		ENSURE( world != NULL );
		v3_box_body_command body = make_box( UINT64_C( 1000 ) + fixed_steps, V3_DYNAMIC_BODY, 0.0, 0.0, 0.0 );
		body.linear_velocity_x = 1.0f;
		ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &body, 1 ) == V3_OK );

		v3_transform transform = { 0 };
		v3_step_stats stats = { 0 };
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, fixed_steps, &transform, 1, NULL, 0, &stats ) ==
				V3_OK );
		ENSURE( stats.fixed_step_count == fixed_steps );
		ENSURE( stats.output_count == 1 );
		ENSURE( fabs( transform.position_x - (double)fixed_steps / 60.0 ) < 1.0e-5 );

		v3_world_destroy( world );
	}
	return 0;
}

static int test_target_duration_and_wrench_reapplication( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command bodies[2] = {
		make_box( UINT64_C( 1100 ), V3_KINEMATIC_BODY, 0.0, 0.0, 0.0 ),
		make_box( UINT64_C( 1101 ), V3_DYNAMIC_BODY, 0.0, 4.0, 0.0 ),
	};
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
	v3_kinematic_target target = make_target( bodies, 2.0 );
	v3_body_wrench wrench = make_wrench( bodies + 1, 6.0f );
	v3_transform transforms[2] = { 0 };
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, &target, 1, &wrench, 1, NULL, 0, 4, transforms, 2, NULL, 0, &stats ) == V3_OK );

	const v3_transform* kinematic = find_transform( transforms, stats.output_count, bodies[0].logical_id );
	const v3_transform* dynamic = find_transform( transforms, stats.output_count, bodies[1].logical_id );
	ENSURE( kinematic != NULL && dynamic != NULL );
	ENSURE( fabs( kinematic->position_x - 2.0 ) < 1.0e-4 );
	ENSURE( kinematic->position_x <= 2.0 );
	ENSURE( fabsf( dynamic->linear_velocity_x - 0.4f ) < 0.025f );
	ENSURE( stats.fixed_step_count == 4 );

	v3_world_destroy( world );
	return 0;
}

static int test_rejected_frames_do_not_mutate_state_or_outputs( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command bodies[3] = {
		make_box( UINT64_C( 1200 ), V3_KINEMATIC_BODY, 0.0, 0.0, 0.0 ),
		make_box( UINT64_C( 1201 ), V3_DYNAMIC_BODY, 0.0, 2.0, 0.0 ),
		make_box( UINT64_C( 1202 ), V3_DYNAMIC_BODY, 2.0, 2.0, 0.0 ),
	};
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 3 ) == V3_OK );
	v3_transform baseline[3] = { 0 };
	v3_step_stats baseline_stats = { 0 };
	ENSURE( read_transforms( world, baseline, 3, &baseline_stats ) == V3_OK );

	v3_kinematic_target target = make_target( bodies, 5.0 );
	v3_body_wrench wrenches[2] = { make_wrench( bodies + 1, 6.0f ), make_wrench( bodies + 2, 6.0f ) };
	wrenches[1].force_x = NAN;
	v3_query valid_query = make_query( UINT64_C( 1 ), 10.0, 1.0f );
	v3_query malformed_query = valid_query;
	malformed_query.half_extent_x = 0.0f;
	v3_transform transforms[3];
	v3_query_result query_result;
	v3_step_stats stats;
	unsigned char transform_sentinel[sizeof( transforms )];
	unsigned char query_sentinel[sizeof( query_result )];
	unsigned char stats_sentinel[sizeof( stats )];

#define RESET_SENTINELS()                                                                                                        \
	do                                                                                                                           \
	{                                                                                                                            \
		memset( transforms, 0xA5, sizeof( transforms ) );                                                                        \
		memset( &query_result, 0x5A, sizeof( query_result ) );                                                                   \
		memset( &stats, 0x3C, sizeof( stats ) );                                                                                 \
		memcpy( transform_sentinel, transforms, sizeof( transforms ) );                                                          \
		memcpy( query_sentinel, &query_result, sizeof( query_result ) );                                                         \
		memcpy( stats_sentinel, &stats, sizeof( stats ) );                                                                       \
	}                                                                                                                            \
	while ( 0 )

#define ENSURE_SENTINELS_AND_STATE()                                                                                             \
	do                                                                                                                           \
	{                                                                                                                            \
		ENSURE( memcmp( transforms, transform_sentinel, sizeof( transforms ) ) == 0 );                                           \
		ENSURE( memcmp( &query_result, query_sentinel, sizeof( query_result ) ) == 0 );                                          \
		ENSURE( memcmp( &stats, stats_sentinel, sizeof( stats ) ) == 0 );                                                        \
		ENSURE( world->mutation_batch_count == baseline_stats.mutation_batch_count );                                            \
		ENSURE( world->created_body_count == baseline_stats.created_body_count );                                                \
		ENSURE( world->destroyed_body_count == baseline_stats.destroyed_body_count );                                            \
		ENSURE( ensure_transforms_match( world, baseline, 3 ) == 0 );                                                            \
	}                                                                                                                            \
	while ( 0 )

	RESET_SENTINELS();
	ENSURE( v3_world_step_and_read( world, &target, 1, wrenches, 2, NULL, 0, 1, transforms, 3, &query_result, 1, &stats ) ==
			V3_NON_FINITE );
	ENSURE_SENTINELS_AND_STATE();

	v3_kinematic_target malformed_target = target;
	malformed_target.rotation_w = 2.0f;
	RESET_SENTINELS();
	ENSURE( v3_world_step_and_read( world, &malformed_target, 1, NULL, 0, NULL, 0, 1, transforms, 3, &query_result, 1, &stats ) ==
			V3_INVALID_QUATERNION );
	ENSURE_SENTINELS_AND_STATE();

	RESET_SENTINELS();
	ENSURE( v3_world_step_and_read( world, &target, 1, NULL, 0, &malformed_query, 1, 1, transforms, 3, &query_result, 1,
									&stats ) == V3_INVALID_DIMENSION );
	ENSURE_SENTINELS_AND_STATE();

	RESET_SENTINELS();
	ENSURE( v3_world_step_and_read( world, &target, 1, NULL, 0, NULL, 0, 1, transforms, 2, &query_result, 1, &stats ) ==
			V3_OUTPUT_TOO_SMALL );
	ENSURE_SENTINELS_AND_STATE();

	RESET_SENTINELS();
	ENSURE( v3_world_step_and_read( world, &target, 1, NULL, 0, &valid_query, 1, 1, transforms, 3, NULL, 0, &stats ) ==
			V3_OUTPUT_TOO_SMALL );
	ENSURE( memcmp( transforms, transform_sentinel, sizeof( transforms ) ) == 0 );
	ENSURE( memcmp( &stats, stats_sentinel, sizeof( stats ) ) == 0 );
	ENSURE( world->mutation_batch_count == baseline_stats.mutation_batch_count );
	ENSURE( world->created_body_count == baseline_stats.created_body_count );
	ENSURE( world->destroyed_body_count == baseline_stats.destroyed_body_count );
	ENSURE( ensure_transforms_match( world, baseline, 3 ) == 0 );

#undef ENSURE_SENTINELS_AND_STATE
#undef RESET_SENTINELS
	v3_world_destroy( world );
	return 0;
}

static int run_query_scene( bool reverse_creation_order, v3_query_result output[3] )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command lower = make_box( UINT64_C( 1301 ), V3_STATIC_BODY, 3.0, 0.0, 0.0 );
	v3_box_body_command higher = make_box( UINT64_C( 1302 ), V3_STATIC_BODY, 3.0, 0.0, 0.0 );
	v3_box_body_command obstacles[2] = { reverse_creation_order ? lower : higher, reverse_creation_order ? higher : lower };
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, obstacles, 2 ) == V3_OK );

	v3_query queries[3] = {
		make_query( UINT64_C( 30 ), -10.0, 1.0f ),
		make_query( UINT64_C( 10 ), 3.0, 0.0f ),
		make_query( UINT64_C( 20 ), 0.0, 6.0f ),
	};
	v3_query_result results[4];
	memset( results, 0x6B, sizeof( results ) );
	v3_query_result untouched = results[3];
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, queries, 3, 0, NULL, 0, results, 4, &stats ) == V3_OK );
	ENSURE( stats.query_count == 3 && stats.query_output_count == 3 );
	ENSURE( results[0].query_id == 10 && results[0].status == V3_QUERY_IMMEDIATE_BLOCK );
	ENSURE( results[0].hit_logical_id == lower.logical_id && results[0].fraction == 0.0f );
	ENSURE( results[1].query_id == 20 && results[1].status == V3_QUERY_CAST_HIT );
	ENSURE( results[1].hit_logical_id == lower.logical_id );
	ENSURE( results[1].fraction > 0.0f && results[1].fraction < 1.0f );
	ENSURE( results[2].query_id == 30 && results[2].status == V3_QUERY_NO_HIT );
	ENSURE( results[2].hit_logical_id == 0 && results[2].fraction == 1.0f );
	ENSURE( memcmp( results + 3, &untouched, sizeof( untouched ) ) == 0 );
	memcpy( output, results, 3 * sizeof( *output ) );

	v3_world_destroy( world );
	return 0;
}

static int test_queries_are_sorted_bounded_and_deterministic( void )
{
	v3_query_result first[3] = { 0 };
	v3_query_result second[3] = { 0 };
	ENSURE( run_query_scene( false, first ) == 0 );
	ENSURE( run_query_scene( true, second ) == 0 );
	ENSURE( memcmp( first, second, sizeof( first ) ) == 0 );
	return 0;
}

static int simulate_far_scene( v3_transform output[2] )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command bodies[3] = {
		make_box( UINT64_C( 1400 ), V3_STATIC_BODY, 20000000.5, -0.5, 0.0 ),
		make_box( UINT64_C( 1401 ), V3_DYNAMIC_BODY, 20000000.25, 3.0, -2.0 ),
		make_box( UINT64_C( 1402 ), V3_DYNAMIC_BODY, 20000000.75, 5.0, 2.0 ),
	};
	bodies[0].half_extent_x = 5.0f;
	bodies[0].half_extent_z = 5.0f;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 3 ) == V3_OK );

	v3_transform transforms[4];
	v3_step_stats stats = { 0 };
	memset( transforms, 0x4D, sizeof( transforms ) );
	for ( uint32_t frame = 0; frame < 30; ++frame )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, transforms, 4, NULL, 0, &stats ) == V3_OK );
	}
	ENSURE( stats.output_count == 2 );
	ENSURE( transforms[0].position_x == 20000000.25 );
	ENSURE( transforms[1].position_x == 20000000.75 );
	ENSURE( transforms[1].position_x - transforms[0].position_x == 0.5 );
	v3_transform untouched;
	memset( &untouched, 0x4D, sizeof( untouched ) );
	ENSURE( memcmp( transforms + 2, &untouched, sizeof( untouched ) ) == 0 );
	ENSURE( memcmp( transforms + 3, &untouched, sizeof( untouched ) ) == 0 );
	memcpy( output, transforms, 2 * sizeof( *output ) );

	v3_world_destroy( world );
	return 0;
}

int main( void )
{
	uint32_t baseline = v3_active_world_count();
	ENSURE( test_exact_fixed_step_counts() == 0 );
	ENSURE( test_target_duration_and_wrench_reapplication() == 0 );
	ENSURE( test_rejected_frames_do_not_mutate_state_or_outputs() == 0 );
	ENSURE( test_queries_are_sorted_bounded_and_deterministic() == 0 );
	ENSURE( v3_active_world_count() == baseline );
	puts( "v3 step tests passed" );
	return 0;
}

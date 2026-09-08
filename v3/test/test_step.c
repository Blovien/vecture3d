// SPDX-License-Identifier: MIT

#include "v3_internal.h"
#include "vecture3d/block_grid.h"

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

static int test_default_world_allows_eligible_body_to_sleep( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );

	v3_box_body_command body = make_box( UINT64_C( 1050 ), V3_DYNAMIC_BODY, 0.0, 0.0, 0.0 );
	body.flags |= V3_BODY_ENABLE_SLEEP;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &body, 1 ) == V3_OK );

	v3_transform transform = { 0 };
	v3_step_stats stats = { 0 };
	for ( uint32_t step = 0; step < 60; ++step )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	}

	ENSURE( stats.output_count == 1 );
	ENSURE( transform.logical_id == body.logical_id );
	ENSURE( ( transform.flags & V3_TRANSFORM_AWAKE ) == 0 );

	v3_world_destroy( world );
	return 0;
}

static int test_sleep_configuration_controls_and_runtime_wake( void )
{
	v3_world_limits disabled = { 0.0f, 0.0f, 0, V3_WORLD_DISABLE_SLEEP };
	v3_world_limits enabled = { 0.0f, 0.0f, 0, 0 };
	v3_world_limits observed = { 0 };
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, &disabled );
	ENSURE( world != NULL );

	v3_box_body_command body = make_box( UINT64_C( 1060 ), V3_DYNAMIC_BODY, 0.0, 0.0, 0.0 );
	body.flags |= V3_BODY_ENABLE_SLEEP;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &body, 1 ) == V3_OK );

	v3_transform transform = { 0 };
	v3_step_stats stats = { 0 };
	for ( uint32_t step = 0; step < 60; ++step )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	}
	ENSURE( ( transform.flags & V3_TRANSFORM_AWAKE ) != 0 );

	ENSURE( v3_world_set_limits( world, &enabled ) == V3_OK );
	ENSURE( v3_world_get_limits( world, &observed ) == V3_OK );
	ENSURE( observed.flags == 0 );
	for ( uint32_t step = 0; step < 60; ++step )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	}
	ENSURE( ( transform.flags & V3_TRANSFORM_AWAKE ) == 0 );

	ENSURE( v3_world_set_limits( world, &disabled ) == V3_OK );
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( ( transform.flags & V3_TRANSFORM_AWAKE ) != 0 );

	ENSURE( v3_world_set_limits( world, &enabled ) == V3_OK );
	for ( uint32_t step = 0; step < 60; ++step )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	}
	ENSURE( ( transform.flags & V3_TRANSFORM_AWAKE ) == 0 );

	v3_world_destroy( world );
	return 0;
}

static int test_body_sleep_flag_is_independent( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[2] = {
		make_box( UINT64_C( 1070 ), V3_DYNAMIC_BODY, 0.0, 0.0, 0.0 ),
		make_box( UINT64_C( 1071 ), V3_DYNAMIC_BODY, 2.0, 0.0, 0.0 ),
	};
	bodies[0].flags |= V3_BODY_ENABLE_SLEEP;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );

	v3_transform transforms[2] = { 0 };
	v3_step_stats stats = { 0 };
	for ( uint32_t step = 0; step < 60; ++step )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, transforms, 2, NULL, 0, &stats ) == V3_OK );
	}

	const v3_transform* eligible = find_transform( transforms, stats.output_count, bodies[0].logical_id );
	const v3_transform* disabled = find_transform( transforms, stats.output_count, bodies[1].logical_id );
	ENSURE( eligible != NULL && disabled != NULL );
	ENSURE( ( eligible->flags & V3_TRANSFORM_AWAKE ) == 0 );
	ENSURE( ( disabled->flags & V3_TRANSFORM_AWAKE ) != 0 );

	v3_world_destroy( world );
	return 0;
}

static int test_wrench_wake_respects_settled_support_and_isolation( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[4] = {
		make_box( UINT64_C( 1080 ), V3_STATIC_BODY, 0.0, -0.5, 0.0 ),
		make_box( UINT64_C( 1081 ), V3_STATIC_BODY, 5.0, -0.5, 0.0 ),
		make_box( UINT64_C( 1082 ), V3_DYNAMIC_BODY, 0.0, 0.5, 0.0 ),
		make_box( UINT64_C( 1083 ), V3_DYNAMIC_BODY, 5.0, 0.5, 0.0 ),
	};
	bodies[2].flags |= V3_BODY_ENABLE_SLEEP;
	bodies[3].flags |= V3_BODY_ENABLE_SLEEP;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 4 ) == V3_OK );

	v3_transform transforms[2] = { 0 };
	v3_step_stats stats = { 0 };
	for ( uint32_t step = 0; step < 180; ++step )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, transforms, 2, NULL, 0, &stats ) == V3_OK );
	}
	const v3_transform* cargo = find_transform( transforms, stats.output_count, bodies[2].logical_id );
	const v3_transform* isolated = find_transform( transforms, stats.output_count, bodies[3].logical_id );
	ENSURE( cargo != NULL && isolated != NULL );
	ENSURE( fabs( cargo->position_y - 0.5 ) < 0.05 );
	ENSURE( fabs( isolated->position_y - 0.5 ) < 0.05 );
	ENSURE( ( cargo->flags & V3_TRANSFORM_AWAKE ) == 0 );
	ENSURE( ( isolated->flags & V3_TRANSFORM_AWAKE ) == 0 );

	v3_body_wrench no_wake = make_wrench( bodies + 3, 40.0f );
	no_wake.flags = 0;
	ENSURE( v3_world_step_and_read( world, NULL, 0, &no_wake, 1, NULL, 0, 1, transforms, 2, NULL, 0, &stats ) == V3_OK );
	isolated = find_transform( transforms, stats.output_count, bodies[3].logical_id );
	ENSURE( isolated != NULL );
	ENSURE( ( isolated->flags & V3_TRANSFORM_AWAKE ) == 0 );

	v3_body_wrench wake = make_wrench( bodies + 2, 40.0f );
	ENSURE( v3_world_step_and_read( world, NULL, 0, &wake, 1, NULL, 0, 1, transforms, 2, NULL, 0, &stats ) == V3_OK );
	cargo = find_transform( transforms, stats.output_count, bodies[2].logical_id );
	isolated = find_transform( transforms, stats.output_count, bodies[3].logical_id );
	ENSURE( cargo != NULL && isolated != NULL );
	ENSURE( ( cargo->flags & V3_TRANSFORM_AWAKE ) != 0 );
	ENSURE( ( isolated->flags & V3_TRANSFORM_AWAKE ) == 0 );

	v3_world_destroy( world );
	return 0;
}

static int test_target_wake_respects_settled_support_and_isolation( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[5] = {
		make_box( UINT64_C( 1090 ), V3_STATIC_BODY, 0.0, -0.5, 0.0 ),
		make_box( UINT64_C( 1091 ), V3_KINEMATIC_BODY, 0.0, 0.5, 0.0 ),
		make_box( UINT64_C( 1092 ), V3_DYNAMIC_BODY, 0.0, 1.5, 0.0 ),
		make_box( UINT64_C( 1093 ), V3_STATIC_BODY, 5.0, -0.5, 0.0 ),
		make_box( UINT64_C( 1094 ), V3_DYNAMIC_BODY, 5.0, 0.5, 0.0 ),
	};
	bodies[1].flags |= V3_BODY_ENABLE_SLEEP;
	bodies[2].flags |= V3_BODY_ENABLE_SLEEP;
	bodies[4].flags |= V3_BODY_ENABLE_SLEEP;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 5 ) == V3_OK );

	v3_transform transforms[3] = { 0 };
	v3_step_stats stats = { 0 };
	for ( uint32_t step = 0; step < 180; ++step )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, transforms, 3, NULL, 0, &stats ) == V3_OK );
	}
	const v3_transform* deck = find_transform( transforms, stats.output_count, bodies[1].logical_id );
	const v3_transform* cargo = find_transform( transforms, stats.output_count, bodies[2].logical_id );
	const v3_transform* isolated = find_transform( transforms, stats.output_count, bodies[4].logical_id );
	ENSURE( deck != NULL && cargo != NULL && isolated != NULL );
	ENSURE( fabs( deck->position_y - 0.5 ) < 0.05 );
	ENSURE( fabs( cargo->position_y - 1.5 ) < 0.05 );
	ENSURE( fabs( isolated->position_y - 0.5 ) < 0.05 );
	ENSURE( ( deck->flags & V3_TRANSFORM_AWAKE ) == 0 );
	ENSURE( ( cargo->flags & V3_TRANSFORM_AWAKE ) == 0 );
	ENSURE( ( isolated->flags & V3_TRANSFORM_AWAKE ) == 0 );

	v3_kinematic_target no_wake = make_target( bodies + 1, 1.0 );
	no_wake.flags = 0;
	ENSURE( v3_world_step_and_read( world, &no_wake, 1, NULL, 0, NULL, 0, 1, transforms, 3, NULL, 0, &stats ) == V3_OK );
	deck = find_transform( transforms, stats.output_count, bodies[1].logical_id );
	cargo = find_transform( transforms, stats.output_count, bodies[2].logical_id );
	isolated = find_transform( transforms, stats.output_count, bodies[4].logical_id );
	ENSURE( deck != NULL && cargo != NULL && isolated != NULL );
	ENSURE( fabs( deck->position_x ) < 0.05 );
	ENSURE( ( deck->flags & V3_TRANSFORM_AWAKE ) == 0 );
	ENSURE( ( cargo->flags & V3_TRANSFORM_AWAKE ) == 0 );
	ENSURE( ( isolated->flags & V3_TRANSFORM_AWAKE ) == 0 );

	v3_kinematic_target wake = make_target( bodies + 1, 1.0 );
	ENSURE( v3_world_step_and_read( world, &wake, 1, NULL, 0, NULL, 0, 1, transforms, 3, NULL, 0, &stats ) == V3_OK );
	deck = find_transform( transforms, stats.output_count, bodies[1].logical_id );
	cargo = find_transform( transforms, stats.output_count, bodies[2].logical_id );
	isolated = find_transform( transforms, stats.output_count, bodies[4].logical_id );
	ENSURE( deck != NULL && cargo != NULL && isolated != NULL );
	ENSURE( ( deck->flags & V3_TRANSFORM_AWAKE ) != 0 );
	ENSURE( ( cargo->flags & V3_TRANSFORM_AWAKE ) != 0 );
	ENSURE( ( isolated->flags & V3_TRANSFORM_AWAKE ) == 0 );

	v3_world_destroy( world );
	return 0;
}

static int test_exact_fixed_step_counts( void )
{
	for ( uint32_t fixed_steps = 0; fixed_steps <= 4; ++fixed_steps )
	{
		v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
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
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
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
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
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
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
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

static v3_cooked_grid* cook_single_cell_grid( uint64_t material_id, uint64_t user_data );

static int test_zero_translation_block_grid_overlap_is_an_immediate_block( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_cooked_grid* grid = cook_single_cell_grid( 1350, 1350 );
	ENSURE( grid != NULL );
	v3_body_definition body = { .handle = { .logical_id = 1350, .generation = 1 }, .kind = V3_STATIC_BODY, .rotation_w = 1.0f };
	v3_body_handle handle;
	ENSURE( v3_world_attach_block_grid( world, &body, grid, NULL, &handle ) == V3_OK );
	v3_destroy_cooked_grid( grid );

	v3_query query = make_query( UINT64_C( 1351 ), 0.5, 0.0f );
	query.origin_y = 0.5;
	query.origin_z = 0.5;
	v3_query_result result = { 0 };
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, &query, 1, 0, NULL, 0, &result, 1, &stats ) == V3_OK );
	ENSURE( result.status == V3_QUERY_IMMEDIATE_BLOCK );
	ENSURE( result.hit_logical_id == body.handle.logical_id );
	ENSURE( result.fraction == 0.0f );

	v3_world_destroy( world );
	return 0;
}

static int test_block_grid_query_kinds_return_cell_identity( void )
{
	v3_block_material material = { .material_id = UINT64_C( 4242 ), .density = 1.0f, .friction = 0.5f };
	v3_block_cell cell = { .feature_id = UINT64_C( 2468 ) };
	v3_block_box box = {
		.center_x = 0.5f,
		.center_y = 0.5f,
		.center_z = 0.5f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
	};
	v3_cooked_grid* grid = NULL;
	ENSURE( v3_cook_block_grid( &material, 1, &cell, 1, &box, 1, &grid ) == V3_OK );

	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_body_definition body = {
		.handle = { .logical_id = UINT64_C( 4600 ), .generation = 1 },
		.kind = V3_STATIC_BODY,
		.position_x = 4.0,
		.rotation_w = 1.0f,
	};
	v3_body_handle attached = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &body, grid, NULL, &attached ) == V3_OK );
	v3_destroy_cooked_grid( grid );

	v3_query queries[5] = { 0 };
	for ( int index = 0; index < 5; ++index )
	{
		queries[index].query_id = (uint64_t)( 100 + index );
		queries[index].kind = (uint32_t)( V3_QUERY_RAY + index );
		queries[index].origin_y = 0.5;
		queries[index].origin_z = 0.5;
		queries[index].translation_x = 10.0f;
		queries[index].category_bits = UINT64_C( 7 );
		queries[index].mask_bits = UINT64_C( 7 );
	}
	queries[1].point_count = 1;
	queries[1].radius = 0.2f;
	queries[2].point_count = 2;
	queries[2].radius = 0.15f;
	queries[2].point_y[0] = -0.1f;
	queries[2].point_y[1] = 0.1f;
	queries[3].point_count = 1;
	queries[3].half_extent_x = 0.2f;
	queries[3].half_extent_y = 0.2f;
	queries[3].half_extent_z = 0.2f;
	queries[4].point_count = 4;
	queries[4].point_x[0] = -0.2f;
	queries[4].point_x[1] = 0.2f;
	queries[4].point_y[2] = 0.2f;
	queries[4].point_z[3] = 0.2f;

	v3_query_result results[5] = { 0 };
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, queries, 5, 0, NULL, 0, results, 5, &stats ) == V3_OK );
	ENSURE( stats.query_output_count == 5 );
	for ( int index = 0; index < 5; ++index )
	{
		ENSURE( results[index].query_id == (uint64_t)( 100 + index ) );
		ENSURE( results[index].status == V3_QUERY_CAST_HIT );
		ENSURE( results[index].hit_logical_id == body.handle.logical_id );
		ENSURE( results[index].fraction > 0.0f && results[index].fraction < 1.0f );
		ENSURE( fabsf( results[index].normal_x + 1.0f ) < 1.0e-4f );
		ENSURE( results[index].flags == V3_QUERY_RESULT_BLOCK_GRID );
		ENSURE( results[index].cell_x == 0 && results[index].cell_y == 0 && results[index].cell_z == 0 );
		ENSURE( results[index].cell_box_index == 0 && results[index].material_index == 0 );
		ENSURE( results[index].user_material_id == material.material_id && results[index].user_data == cell.feature_id );
	}

	v3_world_destroy( world );
	return 0;
}

static int test_two_step_call_retains_first_step_block_contact_begin( void )
{
	v3_block_material material = { .material_id = UINT64_C( 4701 ), .density = 1.0f, .friction = 0.5f };
	v3_block_cell cell = { .feature_id = UINT64_C( 4702 ) };
	v3_block_box box = {
		.center_x = 0.5f,
		.center_y = 0.5f,
		.center_z = 0.5f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
	};
	v3_cooked_grid* grid = NULL;
	ENSURE( v3_cook_block_grid( &material, 1, &cell, 1, &box, 1, &grid ) == V3_OK );

	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_body_definition terrain = {
		.handle = { .logical_id = UINT64_C( 4703 ), .generation = 1 },
		.kind = V3_STATIC_BODY,
		.rotation_w = 1.0f,
	};
	v3_body_handle terrain_handle = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &terrain, grid, NULL, &terrain_handle ) == V3_OK );
	v3_destroy_cooked_grid( grid );

	v3_box_body_command body = make_box( UINT64_C( 4704 ), V3_DYNAMIC_BODY, 0.5, 1.45, 0.5 );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &body, 1 ) == V3_OK );
	v3_transform transform = { 0 };
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 2, &transform, 1, NULL, 0, &stats ) == V3_OK );

	v3BlockContactEvents final_native_step = v3World_GetBlockContactEvents( world->world_id );
	ENSURE( final_native_step.beginCount == 0 );
	ENSURE( stats.block_contact_event_count > 0 );
	ENSURE( stats.block_contact_event_count <= 32 );
	ENSURE( stats.block_contact_event_dropped_count == 0 );
	ENSURE( stats.block_contact_event_flags == 0 );

	v3_block_contact_event events[32] = { 0 };
	uint32_t required = 0;
	ENSURE( v3_world_get_block_contact_events( world, NULL, 0, &required ) == V3_OUTPUT_TOO_SMALL );
	ENSURE( required == stats.block_contact_event_count );
	ENSURE( v3_world_get_block_contact_events( world, events, required, &required ) == V3_OK );
	for ( uint32_t index = 0; index < required; ++index )
	{
		const v3_block_contact_event* event = events + index;
		ENSURE( event->flags == V3_BLOCK_CONTACT_BEGIN && event->fixed_step_index == 0 );
		ENSURE( event->feature_id_a == 0 && event->feature_id_b == 0 );
		const v3_block_contact_side* grid_side =
			event->body_a.logical_id == terrain.handle.logical_id ? &event->side_a : &event->side_b;
		ENSURE( grid_side->flags == V3_BLOCK_CONTACT_SIDE_GRID );
		ENSURE( grid_side->cell_x == 0 && grid_side->cell_y == 0 && grid_side->cell_z == 0 );
		ENSURE( grid_side->cell_box_index == 0 && grid_side->material_index == 0 );
		ENSURE( grid_side->user_material_id == material.material_id && grid_side->user_data == cell.feature_id );
	}

	v3_block_contact_event retry[32] = { 0 };
	uint32_t retry_count = 0;
	ENSURE( v3_world_get_block_contact_events( world, retry, required - 1, &retry_count ) == V3_OUTPUT_TOO_SMALL );
	ENSURE( retry_count == required );
	ENSURE( v3_world_get_block_contact_events( world, retry, retry_count, &retry_count ) == V3_OK );
	ENSURE( memcmp( events, retry, required * sizeof( *events ) ) == 0 );

	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 5, &transform, 1, NULL, 0, &stats ) == V3_LIMIT_EXCEEDED );
	ENSURE( v3_world_get_block_contact_events( world, retry, 32, &retry_count ) == V3_OK );
	ENSURE( retry_count == required && memcmp( events, retry, required * sizeof( *events ) ) == 0 );
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, NULL, 0, NULL, 0, &stats ) == V3_OUTPUT_TOO_SMALL );
	ENSURE( v3_world_get_block_contact_events( world, retry, 32, &retry_count ) == V3_OK );
	ENSURE( retry_count == required && memcmp( events, retry, required * sizeof( *events ) ) == 0 );

	v3_body_handle removal = { .logical_id = body.logical_id, .generation = 1 };
	v3_box_body_command replacement = body;
	replacement.generation = 2;
	replacement.position_x = 10.0;
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, &replacement, 1 ) == V3_OK );
	removal.generation = 2;
	replacement.generation = 3;
	replacement.position_x = body.position_x;
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, &replacement, 1 ) == V3_OK );

	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 0, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.block_contact_event_count == 0 && stats.block_contact_event_dropped_count == 0 );
	ENSURE( stats.step_milliseconds == 0.0f && stats.pair_milliseconds == 0.0f && stats.collide_milliseconds == 0.0f &&
			stats.solve_milliseconds == 0.0f );
	ENSURE( v3_world_get_block_contact_events( world, NULL, 0, &required ) == V3_OK && required == 0 );

	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 4, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.block_contact_event_count > 1 && stats.block_contact_event_count <= 32 );
	ENSURE( v3_world_get_block_contact_events( world, events, 32, &required ) == V3_OK );
	bool found_old_end = false;
	bool found_new_begin = false;
	uint32_t first_begin = required;
	uint32_t last_end = 0;
	for ( uint32_t index = 0; index < required; ++index )
	{
		const v3_block_contact_event* event = events + index;
		ENSURE( event->fixed_step_index < 4 );
		if ( event->flags == V3_BLOCK_CONTACT_END )
		{
			last_end = index;
			const v3_body_handle* participant = event->body_a.logical_id == body.logical_id ? &event->body_a : &event->body_b;
			if ( participant->generation == 1 )
			{
				const v3_block_contact_side* grid_side =
					event->body_a.logical_id == terrain.handle.logical_id ? &event->side_a : &event->side_b;
				ENSURE( grid_side->flags == V3_BLOCK_CONTACT_SIDE_GRID );
				ENSURE( grid_side->cell_x == 0 && grid_side->cell_y == 0 && grid_side->cell_z == 0 );
				ENSURE( grid_side->cell_box_index == 0 && grid_side->material_index == 0 );
				ENSURE( grid_side->user_material_id == material.material_id && grid_side->user_data == cell.feature_id );
				found_old_end = true;
			}
		}
		else if ( event->flags == V3_BLOCK_CONTACT_BEGIN )
		{
			if ( first_begin == required )
			{
				first_begin = index;
			}
			const v3_body_handle* participant = event->body_a.logical_id == body.logical_id ? &event->body_a : &event->body_b;
			found_new_begin = found_new_begin || participant->generation == 3;
		}
	}
	ENSURE( found_old_end && found_new_begin );
	ENSURE( last_end < first_begin );

	v3_world_destroy( world );
	return 0;
}

static v3_cooked_grid* cook_single_cell_grid( uint64_t material_id, uint64_t user_data )
{
	v3_block_material material = { .material_id = material_id, .density = 1.0f, .friction = 0.5f };
	v3_block_cell cell = { .feature_id = user_data };
	v3_block_box box = {
		.center_x = 0.5f,
		.center_y = 0.5f,
		.center_z = 0.5f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
	};
	v3_cooked_grid* grid = NULL;
	return v3_cook_block_grid( &material, 1, &cell, 1, &box, 1, &grid ) == V3_OK ? grid : NULL;
}

static int test_one_step_calls_report_hits_for_both_block_grid_sides( void )
{
	v3_cooked_grid* terrain_grid = cook_single_cell_grid( UINT64_C( 4801 ), UINT64_C( 4802 ) );
	v3_cooked_grid* falling_grid = cook_single_cell_grid( UINT64_C( 4803 ), UINT64_C( 4804 ) );
	ENSURE( terrain_grid != NULL && falling_grid != NULL );
	v3_world* world = v3_world_create( 0.0, -10.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_body_definition terrain = {
		.handle = { .logical_id = UINT64_C( 4805 ), .generation = 1 },
		.kind = V3_STATIC_BODY,
		.rotation_w = 1.0f,
	};
	v3_body_definition falling = {
		.handle = { .logical_id = UINT64_C( 4806 ), .generation = 1 },
		.kind = V3_DYNAMIC_BODY,
		.position_y = 2.0,
		.rotation_w = 1.0f,
		.flags = V3_BODY_INITIAL_AWAKE,
	};
	v3_body_handle attached = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &terrain, terrain_grid, NULL, &attached ) == V3_OK );
	ENSURE( v3_world_attach_block_grid( world, &falling, falling_grid, NULL, &attached ) == V3_OK );
	v3_destroy_cooked_grid( terrain_grid );
	v3_destroy_cooked_grid( falling_grid );

	v3_block_contact_event events[32] = { 0 };
	bool found_begin = false;
	bool found_hit = false;
	for ( uint32_t step = 0; step < 180 && ( !found_begin || !found_hit ); ++step )
	{
		v3_transform transform = { 0 };
		v3_step_stats stats = { 0 };
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
		ENSURE( stats.block_contact_event_count <= 32 );
		uint32_t count = 0;
		ENSURE( v3_world_get_block_contact_events( world, events, 32, &count ) == V3_OK );
		for ( uint32_t index = 0; index < count; ++index )
		{
			const v3_block_contact_event* event = events + index;
			ENSURE( event->fixed_step_index == 0 );
			ENSURE( event->side_a.flags == V3_BLOCK_CONTACT_SIDE_GRID );
			ENSURE( event->side_b.flags == V3_BLOCK_CONTACT_SIDE_GRID );
			ENSURE( event->side_a.cell_x == 0 && event->side_a.cell_y == 0 && event->side_a.cell_z == 0 );
			ENSURE( event->side_b.cell_x == 0 && event->side_b.cell_y == 0 && event->side_b.cell_z == 0 );
			ENSURE( event->side_a.cell_box_index == 0 && event->side_b.cell_box_index == 0 );
			ENSURE( event->side_a.material_index == 0 && event->side_b.material_index == 0 );
			ENSURE(
				( event->side_a.user_material_id == UINT64_C( 4801 ) && event->side_b.user_material_id == UINT64_C( 4803 ) ) ||
				( event->side_a.user_material_id == UINT64_C( 4803 ) && event->side_b.user_material_id == UINT64_C( 4801 ) ) );
			ENSURE( ( event->side_a.user_data == UINT64_C( 4802 ) && event->side_b.user_data == UINT64_C( 4804 ) ) ||
					( event->side_a.user_data == UINT64_C( 4804 ) && event->side_b.user_data == UINT64_C( 4802 ) ) );
			found_begin = found_begin || event->flags == V3_BLOCK_CONTACT_BEGIN;
			if ( event->flags == V3_BLOCK_CONTACT_HIT )
			{
				found_hit = true;
				ENSURE( event->relative_normal_speed > 0.0f );
				ENSURE( event->impulse_x * event->normal_x + event->impulse_y * event->normal_y +
							event->impulse_z * event->normal_z >=
						0.0f );
			}
		}
	}
	ENSURE( found_begin && found_hit );
	v3_world_destroy( world );
	return 0;
}

static int test_profile_times_sum_inside_the_public_fixed_step_loop( void )
{
	b3Profile profiles[4] = {
		{ .step = 1.0f, .pairs = 10.0f, .collide = 100.0f, .solve = 1000.0f },
		{ .step = 2.0f, .pairs = 20.0f, .collide = 200.0f, .solve = 2000.0f },
		{ .step = 3.0f, .pairs = 30.0f, .collide = 300.0f, .solve = 3000.0f },
		{ .step = 4.0f, .pairs = 40.0f, .collide = 400.0f, .solve = 4000.0f },
	};
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_test_set_step_profiles( profiles, 4 );
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 4, NULL, 0, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.step_milliseconds == 10.0f );
	ENSURE( stats.pair_milliseconds == 100.0f );
	ENSURE( stats.collide_milliseconds == 1000.0f );
	ENSURE( stats.solve_milliseconds == 10000.0f );
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 0, NULL, 0, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.step_milliseconds == 0.0f && stats.pair_milliseconds == 0.0f && stats.collide_milliseconds == 0.0f &&
			stats.solve_milliseconds == 0.0f );
	v3_test_set_step_profiles( NULL, 0 );
	v3_world_destroy( world );
	return 0;
}

static int test_four_step_call_reports_later_step_index( void )
{
	v3_cooked_grid* grid = cook_single_cell_grid( UINT64_C( 4851 ), UINT64_C( 4852 ) );
	ENSURE( grid != NULL );
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_body_definition terrain = {
		.handle = { .logical_id = UINT64_C( 4853 ), .generation = 1 },
		.kind = V3_STATIC_BODY,
		.rotation_w = 1.0f,
	};
	v3_body_handle attached = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &terrain, grid, NULL, &attached ) == V3_OK );
	v3_destroy_cooked_grid( grid );
	v3_box_body_command body = make_box( UINT64_C( 4854 ), V3_DYNAMIC_BODY, 0.5, 2.0, 0.5 );
	body.linear_velocity_y = -10.0f;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &body, 1 ) == V3_OK );

	v3_transform transform = { 0 };
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 4, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.block_contact_event_count > 0 && stats.block_contact_event_count <= 32 );
	v3_block_contact_event events[32] = { 0 };
	uint32_t count = 0;
	ENSURE( v3_world_get_block_contact_events( world, events, 32, &count ) == V3_OK );
	bool found_later_event = false;
	for ( uint32_t index = 0; index < count; ++index )
	{
		ENSURE( events[index].fixed_step_index < 4 );
		found_later_event = found_later_event || events[index].fixed_step_index > 0;
	}
	ENSURE( found_later_event );
	v3_world_destroy( world );
	return 0;
}

static int test_native_event_overflow_stays_visible_across_two_steps( void )
{
	enum
	{
		contact_body_count = 300,
	};
	v3_block_material material = { .material_id = UINT64_C( 4901 ), .density = 1.0f, .friction = 0.5f };
	v3_block_cell cells[contact_body_count];
	v3_block_box boxes[contact_body_count];
	v3_box_body_command bodies[contact_body_count];
	for ( int index = 0; index < contact_body_count; ++index )
	{
		cells[index] = (v3_block_cell){ .x = index, .feature_id = UINT64_C( 5000 ) + (uint64_t)index };
		boxes[index] = (v3_block_box){
			.center_x = 0.5f,
			.center_y = 0.5f,
			.center_z = 0.5f,
			.half_extent_x = 0.5f,
			.half_extent_y = 0.5f,
			.half_extent_z = 0.5f,
			.owner_cell_index = (uint32_t)index,
		};
		bodies[index] = make_box( UINT64_C( 6000 ) + (uint64_t)index, V3_DYNAMIC_BODY, (double)index + 0.5, 1.15, 0.5 );
		bodies[index].half_extent_x = 0.2f;
		bodies[index].half_extent_y = 0.2f;
		bodies[index].half_extent_z = 0.2f;
	}
	v3_cooked_grid* grid = NULL;
	ENSURE( v3_cook_block_grid( &material, 1, cells, contact_body_count, boxes, contact_body_count, &grid ) == V3_OK );
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_body_definition terrain = {
		.handle = { .logical_id = UINT64_C( 4902 ), .generation = 1 },
		.kind = V3_STATIC_BODY,
		.rotation_w = 1.0f,
	};
	v3_body_handle attached = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &terrain, grid, NULL, &attached ) == V3_OK );
	v3_destroy_cooked_grid( grid );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, contact_body_count ) == V3_OK );

	v3_transform transforms[contact_body_count];
	v3_step_stats stats = { 0 };
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 2, transforms, contact_body_count, NULL, 0, &stats ) ==
			V3_OK );
	ENSURE( stats.block_contact_event_count <= V3_MAX_BLOCK_CONTACT_EVENTS );
	ENSURE( stats.block_contact_event_dropped_count > 0 );
	ENSURE( stats.block_contact_event_flags == V3_BLOCK_CONTACT_EVENTS_TRUNCATED );
	v3_block_contact_event events[V3_MAX_BLOCK_CONTACT_EVENTS];
	uint32_t count = 0;
	ENSURE( v3_world_get_block_contact_events( world, events, V3_MAX_BLOCK_CONTACT_EVENTS, &count ) == V3_OK );
	ENSURE( count == stats.block_contact_event_count );
	v3_world_destroy( world );
	return 0;
}

int main( void )
{
	uint32_t baseline = v3_active_world_count();
	ENSURE( test_default_world_allows_eligible_body_to_sleep() == 0 );
	ENSURE( test_sleep_configuration_controls_and_runtime_wake() == 0 );
	ENSURE( test_body_sleep_flag_is_independent() == 0 );
	ENSURE( test_wrench_wake_respects_settled_support_and_isolation() == 0 );
	ENSURE( test_target_wake_respects_settled_support_and_isolation() == 0 );
	ENSURE( test_exact_fixed_step_counts() == 0 );
	ENSURE( test_target_duration_and_wrench_reapplication() == 0 );
	ENSURE( test_rejected_frames_do_not_mutate_state_or_outputs() == 0 );
	ENSURE( test_queries_are_sorted_bounded_and_deterministic() == 0 );
	ENSURE( test_zero_translation_block_grid_overlap_is_an_immediate_block() == 0 );
	ENSURE( test_block_grid_query_kinds_return_cell_identity() == 0 );
	ENSURE( test_two_step_call_retains_first_step_block_contact_begin() == 0 );
	ENSURE( test_one_step_calls_report_hits_for_both_block_grid_sides() == 0 );
	ENSURE( test_profile_times_sum_inside_the_public_fixed_step_loop() == 0 );
	ENSURE( test_four_step_call_reports_later_step_index() == 0 );
	ENSURE( test_native_event_overflow_stays_visible_across_two_steps() == 0 );
	ENSURE( v3_active_world_count() == baseline );
	puts( "v3 step tests passed" );
	return 0;
}

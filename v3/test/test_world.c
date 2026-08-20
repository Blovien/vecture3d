// SPDX-License-Identifier: MIT

#include "v3_abi.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define V3_TEST_BODY_LIMIT UINT32_C( 4096 )

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

static int test_world_lifetime( void )
{
	uint32_t baseline = v3_active_world_count();
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0 );
	ENSURE( world != NULL );
	ENSURE( v3_active_world_count() == baseline + 1u );

	v3_world_destroy( world );
	ENSURE( v3_active_world_count() == baseline );
	v3_world_destroy( NULL );
	ENSURE( v3_active_world_count() == baseline );
	return 0;
}

static int test_atomic_validation( void )
{
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0 );
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
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0 );
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
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0 );
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
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0 );
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
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0 );
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
	v3_world* world = v3_world_create( 0.0, -9.81, 0.0 );
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
	ENSURE( test_world_lifetime() == 0 );
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

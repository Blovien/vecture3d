// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>

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

static int ensure_empty_world( const v3_world* world )
{
	ENSURE( world->body_entry_count == 0 );
	ENSURE( world->active_body_count == 0 );
	ENSURE( world->mutation_batch_count == 0 );
	ENSURE( world->created_body_count == 0 );
	ENSURE( world->destroyed_body_count == 0 );
	ENSURE( b3World_GetCounters( world->world_id ).bodyCount == 0 );
	return 0;
}

static int test_world_allocation_failure_does_not_publish_world( void )
{
	uint32_t baseline = v3_active_world_count_internal();
	v3_test_fail_after( V3_TEST_FAULT_WORLD_CALLOC, 0 );
	ENSURE( v3_world_create_internal( 0.0, -9.81, 0.0 ) == NULL );
	ENSURE( v3_active_world_count_internal() == baseline );
	return 0;
}

static int test_entry_allocation_failure_does_not_publish_logical_id( void )
{
	v3_world* world = v3_world_create_internal( 0.0, -9.81, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command command = make_static_box( UINT64_C( 60001 ), UINT32_C( 1 ) );

	v3_test_fail_after( V3_TEST_FAULT_BODY_ENTRIES_REALLOC, 0 );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, &command, 1 ) == V3_OUT_OF_MEMORY );
	ENSURE( ensure_empty_world( world ) == 0 );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, &command, 1 ) == V3_OK );

	v3_world_destroy_internal( world );
	return 0;
}

static int test_pending_allocation_failure_does_not_publish_logical_id( void )
{
	v3_world* world = v3_world_create_internal( 0.0, -9.81, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command initial = make_static_box( UINT64_C( 61001 ), UINT32_C( 1 ) );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, &initial, 1 ) == V3_OK );
	v3_body_handle removal = make_handle( initial.logical_id, initial.generation );
	ENSURE( v3_world_replace_box_bodies_internal( world, &removal, 1, NULL, 0 ) == V3_OK );

	uint32_t entry_count = world->body_entry_count;
	uint32_t mutation_count = world->mutation_batch_count;
	uint32_t created_count = world->created_body_count;
	uint32_t destroyed_count = world->destroyed_body_count;
	v3_box_body_command command = make_static_box( UINT64_C( 61002 ), UINT32_C( 1 ) );
	v3_test_fail_after( V3_TEST_FAULT_PENDING_CALLOC, 0 );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, &command, 1 ) == V3_OUT_OF_MEMORY );
	ENSURE( world->body_entry_count == entry_count );
	ENSURE( world->active_body_count == 0 );
	ENSURE( world->mutation_batch_count == mutation_count );
	ENSURE( world->created_body_count == created_count );
	ENSURE( world->destroyed_body_count == destroyed_count );
	ENSURE( b3World_GetCounters( world->world_id ).bodyCount == 0 );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, &command, 1 ) == V3_OK );

	v3_world_destroy_internal( world );
	return 0;
}

static int test_body_creation_failure_rolls_back_pending_body( void )
{
	v3_world* world = v3_world_create_internal( 0.0, -9.81, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command commands[2] = {
		make_static_box( UINT64_C( 62001 ), UINT32_C( 1 ) ),
		make_static_box( UINT64_C( 62002 ), UINT32_C( 1 ) ),
	};

	v3_test_fail_after( V3_TEST_FAULT_CREATE_BODY, 1 );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, commands, 2 ) == V3_NATIVE_FAILURE );
	ENSURE( ensure_empty_world( world ) == 0 );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, commands, 2 ) == V3_OK );

	v3_world_destroy_internal( world );
	return 0;
}

static int test_shape_creation_failure_rolls_back_current_and_pending_bodies( void )
{
	v3_world* world = v3_world_create_internal( 0.0, -9.81, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command commands[2] = {
		make_static_box( UINT64_C( 63001 ), UINT32_C( 1 ) ),
		make_static_box( UINT64_C( 63002 ), UINT32_C( 1 ) ),
	};

	v3_test_fail_after( V3_TEST_FAULT_CREATE_SHAPE, 1 );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, commands, 2 ) == V3_NATIVE_FAILURE );
	ENSURE( ensure_empty_world( world ) == 0 );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, commands, 2 ) == V3_OK );

	v3_world_destroy_internal( world );
	return 0;
}

static int test_mutation_counters_saturate_at_signed_maximum( void )
{
	v3_world* world = v3_world_create_internal( 0.0, -9.81, 0.0 );
	ENSURE( world != NULL );
	world->mutation_batch_count = (uint32_t)INT32_MAX - 1u;
	world->created_body_count = (uint32_t)INT32_MAX - 1u;
	world->destroyed_body_count = (uint32_t)INT32_MAX - 1u;

	v3_box_body_command command = make_static_box( UINT64_C( 64001 ), UINT32_C( 1 ) );
	ENSURE( v3_world_replace_box_bodies_internal( world, NULL, 0, &command, 1 ) == V3_OK );
	v3_body_handle removal = make_handle( command.logical_id, command.generation );
	ENSURE( v3_world_replace_box_bodies_internal( world, &removal, 1, NULL, 0 ) == V3_OK );
	ENSURE( world->mutation_batch_count == (uint32_t)INT32_MAX );
	ENSURE( world->created_body_count == (uint32_t)INT32_MAX );
	ENSURE( world->destroyed_body_count == (uint32_t)INT32_MAX );

	v3_world_destroy_internal( world );
	return 0;
}

int main( void )
{
	uint32_t baseline = v3_active_world_count_internal();
	ENSURE( test_world_allocation_failure_does_not_publish_world() == 0 );
	ENSURE( test_entry_allocation_failure_does_not_publish_logical_id() == 0 );
	ENSURE( test_pending_allocation_failure_does_not_publish_logical_id() == 0 );
	ENSURE( test_body_creation_failure_rolls_back_pending_body() == 0 );
	ENSURE( test_shape_creation_failure_rolls_back_current_and_pending_bodies() == 0 );
	ENSURE( test_mutation_counters_saturate_at_signed_maximum() == 0 );
	ENSURE( v3_active_world_count_internal() == baseline );
	puts( "v3 internal world tests passed" );
	return 0;
}

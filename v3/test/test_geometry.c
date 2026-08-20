// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define V3_TEST_DEFAULT_FRICTION 0.7f
#define V3_TEST_FIXED_STEP ( 1.0f / 60.0f )

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

static v3_box_body_command make_static_box( uint64_t logical_id, double position_x, double position_y, double position_z )
{
	return (v3_box_body_command){
		.logical_id = logical_id,
		.kind = V3_STATIC_BODY,
		.generation = 1,
		.position_x = position_x,
		.position_y = position_y,
		.position_z = position_z,
		.rotation_w = 1.0f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
		.friction = V3_TEST_DEFAULT_FRICTION,
	};
}

static v3_box_body_command make_dynamic_box( uint64_t logical_id, double position_x, double position_y, double position_z )
{
	v3_box_body_command command = make_static_box( logical_id, position_x, position_y, position_z );
	command.kind = V3_DYNAMIC_BODY;
	command.density = 1.0f;
	command.flags = V3_BODY_INITIAL_AWAKE;
	return command;
}

static v3_body_handle make_handle( uint64_t logical_id, uint32_t generation )
{
	return (v3_body_handle){ .logical_id = logical_id, .generation = generation };
}

static v3_terrain_box make_detail_box( uint64_t feature_id, float center_x, float center_y, float center_z )
{
	return (v3_terrain_box){
		.center_x = center_x,
		.center_y = center_y,
		.center_z = center_z,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
		.friction = V3_TEST_DEFAULT_FRICTION,
		.feature_id = feature_id,
	};
}

static v3_terrain_section_command make_section( uint64_t logical_id, uint32_t generation, uint32_t detail_box_offset,
												uint32_t detail_box_count )
{
	return (v3_terrain_section_command){
		.logical_id = logical_id,
		.generation = generation,
		.detail_box_offset = detail_box_offset,
		.detail_box_count = detail_box_count,
	};
}

static uint32_t pack_voxel_run( uint32_t x, uint32_t y, uint32_t z, uint32_t width, uint32_t height, uint32_t depth )
{
	return x | ( y << 5u ) | ( z << 10u ) | ( ( width - 1u ) << 15u ) | ( ( height - 1u ) << 20u ) | ( ( depth - 1u ) << 25u );
}

static v3_body_entry* find_entry( v3_world* world, uint64_t logical_id )
{
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		if ( world->body_entries[index].logical_id == logical_id )
		{
			return world->body_entries + index;
		}
	}

	return NULL;
}

static int test_terrain_payload_is_retained_after_caller_overwrite( void )
{
	v3_world* world = v3_world_create( 0.0, -32.0, 0.0 );
	ENSURE( world != NULL );

	v3_box_body_command dynamic = make_dynamic_box( UINT64_C( 1001 ), 0.5, 1.5, 0.5 );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &dynamic, 1 ) == V3_OK );

	v3_voxel_run runs[1] = { { .packed = pack_voxel_run( 0, 0, 0, 1, 1, 1 ) } };
	v3_terrain_box details[1] = { make_detail_box( UINT64_C( 7001 ), 0.5f, 0.5f, 0.5f ) };
	v3_terrain_section_command section = make_section( UINT64_C( 1002 ), 1, 0, 1 );
	section.voxel_run_count = 1;
	ENSURE( v3_world_replace_terrain_sections( world, NULL, 0, &section, 1, runs, 1, details, 1 ) == V3_OK );

	memset( runs, 0xA5, sizeof( runs ) );
	memset( details, 0, sizeof( details ) );

	v3_body_entry* dynamic_entry = find_entry( world, dynamic.logical_id );
	v3_body_entry* terrain_entry = find_entry( world, section.logical_id );
	ENSURE( dynamic_entry != NULL && terrain_entry != NULL );
	ENSURE( b3Shape_GetType( terrain_entry->shape_id ) == b3_voxelShape );

	b3World_Step( world->world_id, V3_TEST_FIXED_STEP, 4 );
	ENSURE( b3Body_GetContactCapacity( dynamic_entry->body_id ) > 0 );

	v3_world_destroy( world );
	return 0;
}

static int test_streamed_voxel_pairs_with_resting_dynamic_body( void )
{
	v3_world* world = v3_world_create( 0.0, -32.0, 0.0 );
	ENSURE( world != NULL );

	v3_box_body_command bodies[2] = {
		make_static_box( UINT64_C( 1201 ), 0.0, 0.5, 0.0 ),
		make_dynamic_box( UINT64_C( 1202 ), 0.0, 4.0, 0.0 ),
	};
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
	v3_body_entry* dynamic_entry = find_entry( world, bodies[1].logical_id );
	ENSURE( dynamic_entry != NULL );
	for ( int step = 0; step < 240; ++step )
	{
		b3World_Step( world->world_id, V3_TEST_FIXED_STEP, 4 );
	}
	int before_contact_capacity = b3Body_GetContactCapacity( dynamic_entry->body_id );
	ENSURE( before_contact_capacity > 0 );

	v3_voxel_run run = { .packed = pack_voxel_run( 0, 0, 0, 1, 1, 1 ) };
	v3_terrain_section_command section = make_section( UINT64_C( 1203 ), 1, 0, 0 );
	section.voxel_run_count = 1;
	ENSURE( v3_world_replace_terrain_sections( world, NULL, 0, &section, 1, &run, 1, NULL, 0 ) == V3_OK );
	b3World_Step( world->world_id, V3_TEST_FIXED_STEP, 4 );
	ENSURE( b3Body_GetContactCapacity( dynamic_entry->body_id ) > before_contact_capacity );

	v3_world_destroy( world );
	return 0;
}

int main( void )
{
	ENSURE( test_terrain_payload_is_retained_after_caller_overwrite() == 0 );
	ENSURE( test_streamed_voxel_pairs_with_resting_dynamic_body() == 0 );
	puts( "v3 geometry tests passed" );
	return 0;
}

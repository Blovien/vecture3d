// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <math.h>
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
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0 );
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
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0 );
	ENSURE( world != NULL );
	v3_box_body_command bodies[2] = {
		make_dynamic_box( UINT64_C( 300 ), -1.5, -4.0f ),
		make_dynamic_box( UINT64_C( 301 ), 1.5, 4.0f ),
	};
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
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
	ENSURE( v3_world_replace_terrain_sections( world, &attached_body, 1, NULL, 0, NULL, 0, NULL, 0 ) == V3_JOINT_ATTACHED );
	v3_joint_handle joint = { .logical_id = coupler.logical_id, .generation = coupler.generation };
	ENSURE( v3_world_replace_distance_joints( world, &joint, 1, NULL, 0 ) == V3_OK );
	v3_body_handle removals[2] = { body_handle( bodies ), body_handle( bodies + 1 ) };
	ENSURE( v3_world_replace_box_bodies( world, removals, 2, NULL, 0 ) == V3_OK );

	v3_world_destroy( world );
	return 0;
}

int main( void )
{
	uint32_t baseline = v3_active_world_count();
	ENSURE( test_invalid_replacement_preserves_accepted_joint_and_bodies() == 0 );
	ENSURE( test_distance_limit_and_attached_body_guard() == 0 );
	ENSURE( v3_active_world_count() == baseline );
	puts( "v3 joint tests passed" );
	return 0;
}

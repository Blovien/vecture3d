// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

typedef struct v3_pending_body
{
	uint64_t logical_id;
	uint32_t generation;
	b3BodyId body_id;
	b3ShapeId shape_id;
	uint32_t kind;
} v3_pending_body;

// TODO: prolly just enfore double precision inside box3d part
_Static_assert( sizeof( b3Pos ) == 24, "V3 and Box3D must share double-precision world positions" );

static uint32_t v3_saturating_add( uint32_t value, uint32_t increment )
{
	return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

static bool v3_is_normalized_quaternion( float x, float y, float z, float w )
{
	if ( !isfinite( x ) || !isfinite( y ) || !isfinite( z ) || !isfinite( w ) )
	{
		return false;
	}

	float length_squared = x * x + y * y + z * z + w * w;
	return isfinite( length_squared ) && 1.0f - 20.0f * FLT_EPSILON < length_squared &&
		   length_squared < 1.0f + 20.0f * FLT_EPSILON;
}

static int v3_find_body_entry( const v3_world* world, uint64_t logical_id )
{
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		if ( world->body_entries[index].logical_id == logical_id )
		{
			return (int)index;
		}
	}

	return -1;
}

static bool v3_removes_logical_id( const v3_body_handle* removals, uint32_t removal_count, uint64_t logical_id )
{
	for ( uint32_t index = 0; index < removal_count; ++index )
	{
		if ( removals[index].logical_id == logical_id )
		{
			return true;
		}
	}

	return false;
}

static v3_status v3_validate_removals( const v3_world* world, const v3_body_handle* removals, uint32_t removal_count )
{
	for ( uint32_t index = 0; index < removal_count; ++index )
	{
		const v3_body_handle* removal = removals + index;
		if ( removal->logical_id == 0 || removal->generation == 0 || removal->generation > INT32_MAX || removal->reserved != 0 )
		{
			return V3_INVALID_ARGUMENT;
		}

		for ( uint32_t previous = 0; previous < index; ++previous )
		{
			if ( removals[previous].logical_id == removal->logical_id )
			{
				return V3_DUPLICATE_ID;
			}
		}

		int entry_index = v3_find_body_entry( world, removal->logical_id );
		if ( entry_index < 0 || !world->body_entries[entry_index].is_active ||
			 world->body_entries[entry_index].generation != removal->generation )
		{
			return V3_STALE_HANDLE;
		}
	}

	return V3_OK;
}

static v3_status v3_validate_box( const v3_box_body_command* command )
{
	if ( command->logical_id == 0 ||
		 ( command->kind != V3_STATIC_BODY && command->kind != V3_KINEMATIC_BODY && command->kind != V3_DYNAMIC_BODY ) )
	{
		return V3_INVALID_ARGUMENT;
	}

	if ( command->generation == 0 || command->generation > INT32_MAX )
	{
		return V3_INVALID_GENERATION;
	}

	if ( ( command->flags & ~( V3_BODY_ENABLE_SLEEP | V3_BODY_INITIAL_AWAKE | V3_BODY_DISABLE_COLLISION ) ) != 0 )
	{
		return V3_INVALID_ARGUMENT;
	}

	if ( !isfinite( command->position_x ) || !isfinite( command->position_y ) || !isfinite( command->position_z ) ||
		 !isfinite( command->linear_velocity_x ) || !isfinite( command->linear_velocity_y ) ||
		 !isfinite( command->linear_velocity_z ) || !isfinite( command->angular_velocity_x ) ||
		 !isfinite( command->angular_velocity_y ) || !isfinite( command->angular_velocity_z ) ||
		 !isfinite( command->half_extent_x ) || !isfinite( command->half_extent_y ) || !isfinite( command->half_extent_z ) ||
		 !isfinite( command->density ) || !isfinite( command->friction ) || !isfinite( command->linear_damping ) ||
		 !isfinite( command->angular_damping ) )
	{
		return V3_NON_FINITE;
	}

	if ( !v3_is_normalized_quaternion( command->rotation_x, command->rotation_y, command->rotation_z, command->rotation_w ) )
	{
		return V3_INVALID_QUATERNION;
	}

	if ( command->half_extent_x <= 0.0f || command->half_extent_y <= 0.0f || command->half_extent_z <= 0.0f )
	{
		return V3_INVALID_DIMENSION;
	}

	if ( command->density < 0.0f || ( command->kind == V3_DYNAMIC_BODY && command->density <= 0.0f ) ||
		 ( command->kind != V3_DYNAMIC_BODY && command->density != 0.0f ) )
	{
		return V3_INVALID_DENSITY;
	}

	if ( command->friction < 0.0f || command->friction > 1.0f )
	{
		return V3_INVALID_FRICTION;
	}

	if ( command->linear_damping < 0.0f || command->linear_damping > V3_MAX_DAMPING || command->angular_damping < 0.0f ||
		 command->angular_damping > V3_MAX_DAMPING )
	{
		return V3_INVALID_DAMPING;
	}

	return V3_OK;
}

static v3_status v3_validate_creations( const v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
										const v3_box_body_command* creations, uint32_t creation_count, uint32_t* new_entry_count )
{
	*new_entry_count = 0;
	for ( uint32_t index = 0; index < creation_count; ++index )
	{
		const v3_box_body_command* creation = creations + index;
		v3_status status = v3_validate_box( creation );
		if ( status != V3_OK )
		{
			return status;
		}

		for ( uint32_t previous = 0; previous < index; ++previous )
		{
			if ( creations[previous].logical_id == creation->logical_id )
			{
				return V3_DUPLICATE_ID;
			}
		}

		int entry_index = v3_find_body_entry( world, creation->logical_id );
		if ( entry_index < 0 )
		{
			if ( creation->generation != 1 )
			{
				return V3_INVALID_GENERATION;
			}

			*new_entry_count += 1;
			continue;
		}

		const v3_body_entry* entry = world->body_entries + entry_index;
		if ( entry->is_active && !v3_removes_logical_id( removals, removal_count, creation->logical_id ) )
		{
			return V3_DUPLICATE_ID;
		}

		if ( entry->generation == INT32_MAX )
		{
			return V3_GENERATION_EXHAUSTED;
		}

		if ( creation->generation != entry->generation + 1u )
		{
			return V3_INVALID_GENERATION;
		}
	}

	return V3_OK;
}

static v3_status v3_reserve_body_entries( v3_world* world, uint32_t required_capacity )
{
	if ( required_capacity <= world->body_entry_capacity )
	{
		return V3_OK;
	}

	uint32_t new_capacity = world->body_entry_capacity == 0 ? UINT32_C( 16 ) : world->body_entry_capacity;
	while ( new_capacity < required_capacity )
	{
		new_capacity = new_capacity > V3_MAX_LOGICAL_BODY_IDS / 2u ? V3_MAX_LOGICAL_BODY_IDS : new_capacity * 2u;
	}

	v3_body_entry* entries = realloc( world->body_entries, (size_t)new_capacity * sizeof( *entries ) );
	if ( entries == NULL )
	{
		return V3_OUT_OF_MEMORY;
	}

	world->body_entries = entries;
	world->body_entry_capacity = new_capacity;
	return V3_OK;
}

static void v3_destroy_body_entry( v3_body_entry* entry )
{
	b3DestroyBody( entry->body_id );
	entry->is_active = false;
}

static void v3_destroy_pending_bodies( v3_pending_body* pending, uint32_t created_count )
{
	while ( created_count > 0 )
	{
		b3DestroyBody( pending[--created_count].body_id );
	}
}

v3_world* v3_world_create_internal( double gravity_x, double gravity_y, double gravity_z )
{
	v3_world* world = calloc( 1, sizeof( *world ) );
	if ( world == NULL )
	{
		return NULL;
	}

	b3WorldDef definition = b3DefaultWorldDef();
	definition.gravity = (b3Vec3){ (float)gravity_x, (float)gravity_y, (float)gravity_z };

	{
        // TODO: double check, but in theory island sleeping shouldn't account for this setting
        definition.enableSleep = false;
	}

	definition.workerCount = 1;
	world->world_id = b3CreateWorld( &definition );
	if ( B3_IS_NULL( world->world_id ) )
	{
		free( world );
		return NULL;
	}

	return world;
}

v3_status v3_world_replace_box_bodies_internal( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
												const v3_box_body_command* creations, uint32_t creation_count )
{
	v3_status status = v3_validate_removals( world, removals, removal_count );
	if ( status != V3_OK )
	{
		return status;
	}

	uint32_t new_entry_count = 0;
	status = v3_validate_creations( world, removals, removal_count, creations, creation_count, &new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}

	uint32_t final_body_count = world->active_body_count - removal_count + creation_count;
	if ( final_body_count > V3_MAX_BODIES_PER_BATCH || world->body_entry_count + new_entry_count > V3_MAX_LOGICAL_BODY_IDS )
	{
		return V3_LIMIT_EXCEEDED;
	}

	// Pending bodies coexist with accepted removals until every native creation succeeds.
	if ( world->active_body_count > V3_MAX_BODIES_PER_BATCH - creation_count )
	{
		return V3_PEAK_LIMIT_EXCEEDED;
	}

	status = v3_reserve_body_entries( world, world->body_entry_count + new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}

	v3_pending_body* pending = NULL;
	if ( creation_count > 0 )
	{
		pending = calloc( creation_count, sizeof( *pending ) );
		if ( pending == NULL )
		{
			return V3_OUT_OF_MEMORY;
		}
	}

	uint32_t created_count = 0;
	for ( uint32_t index = 0; index < creation_count; ++index )
	{
		const v3_box_body_command* command = creations + index;
		b3BodyDef body_definition = b3DefaultBodyDef();
		body_definition.type = (b3BodyType)command->kind;
		body_definition.position = (b3Pos){ command->position_x, command->position_y, command->position_z };
		body_definition.rotation =
			(b3Quat){ { command->rotation_x, command->rotation_y, command->rotation_z }, command->rotation_w };
		body_definition.linearVelocity =
			(b3Vec3){ command->linear_velocity_x, command->linear_velocity_y, command->linear_velocity_z };
		body_definition.angularVelocity =
			(b3Vec3){ command->angular_velocity_x, command->angular_velocity_y, command->angular_velocity_z };
		body_definition.linearDamping = command->linear_damping;
		body_definition.angularDamping = command->angular_damping;
		body_definition.enableSleep = ( command->flags & V3_BODY_ENABLE_SLEEP ) != 0;
		body_definition.isAwake = ( command->flags & V3_BODY_INITIAL_AWAKE ) != 0;

		b3BodyId body_id = b3CreateBody( world->world_id, &body_definition );
		if ( B3_IS_NULL( body_id ) )
		{
			status = V3_NATIVE_FAILURE;
			break;
		}

		b3ShapeDef shape_definition = b3DefaultShapeDef();
		shape_definition.density = command->density;
		shape_definition.baseMaterial.friction = command->friction;
		shape_definition.filter.categoryBits = command->kind == V3_STATIC_BODY		? V3_STATIC_CATEGORY
											   : command->kind == V3_KINEMATIC_BODY ? V3_KINEMATIC_CATEGORY
																					: V3_DYNAMIC_CATEGORY;
		shape_definition.filter.maskBits = ( command->flags & V3_BODY_DISABLE_COLLISION ) != 0 ? 0u : V3_ALL_BODY_CATEGORIES;
		b3BoxHull box = b3MakeBoxHull( command->half_extent_x, command->half_extent_y, command->half_extent_z );
		b3ShapeId shape_id = b3CreateHullShape( body_id, &shape_definition, &box.base );
		if ( B3_IS_NULL( shape_id ) )
		{
			b3DestroyBody( body_id );
			status = V3_NATIVE_FAILURE;
			break;
		}

		pending[created_count++] = (v3_pending_body){
			.logical_id = command->logical_id,
			.generation = command->generation,
			.body_id = body_id,
			.shape_id = shape_id,
			.kind = command->kind,
		};
	}

	if ( status != V3_OK )
	{
		v3_destroy_pending_bodies( pending, created_count );
		free( pending );
		return status;
	}

	for ( uint32_t index = 0; index < removal_count; ++index )
	{
		int entry_index = v3_find_body_entry( world, removals[index].logical_id );
		v3_destroy_body_entry( world->body_entries + entry_index );
	}

	// Logical generations become visible only after all native bodies and shapes exist
	for ( uint32_t index = 0; index < creation_count; ++index )
	{
		const v3_pending_body* created = pending + index;
		int entry_index = v3_find_body_entry( world, created->logical_id );
		if ( entry_index < 0 )
		{
			entry_index = (int)world->body_entry_count++;
		}

		world->body_entries[entry_index] = (v3_body_entry){
			.logical_id = created->logical_id,
			.generation = created->generation,
			.body_id = created->body_id,
			.shape_id = created->shape_id,
			.kind = created->kind,
			.is_active = true,
		};
	}

	world->active_body_count = final_body_count;
	if ( removal_count > 0 || creation_count > 0 )
	{
		world->mutation_batch_count = v3_saturating_add( world->mutation_batch_count, 1 );
		world->created_body_count = v3_saturating_add( world->created_body_count, creation_count );
		world->destroyed_body_count = v3_saturating_add( world->destroyed_body_count, removal_count );
	}

	free( pending );
	return V3_OK;
}

void v3_world_destroy_internal( v3_world* world )
{
	if ( world == NULL )
	{
		return;
	}

	b3DestroyWorld( world->world_id );
	free( world->body_entries );
	free( world );
}

uint32_t v3_active_world_count_internal( void )
{
	return (uint32_t)b3GetWorldCount();
}

// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

typedef struct v3_pending_joint
{
	uint64_t logical_id;
	uint32_t generation;
	b3JointId joint_id;
	uint64_t body_a_logical_id;
	uint32_t body_a_generation;
	uint64_t body_b_logical_id;
	uint32_t body_b_generation;
} v3_pending_joint;

static int v3_joint_find_body_entry( const v3_world* world, uint64_t logical_id )
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

static int v3_joint_find_entry( const v3_world* world, uint64_t logical_id )
{
	for ( uint32_t index = 0; index < world->joint_entry_count; ++index )
	{
		if ( world->joint_entries[index].logical_id == logical_id )
		{
			return (int)index;
		}
	}
	return -1;
}

static bool v3_joint_removes_logical_id( const v3_joint_handle* removals, uint32_t removal_count, uint64_t logical_id )
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

static bool v3_joint_has_attached_body( const v3_world* world, uint64_t logical_id, uint32_t generation )
{
	for ( uint32_t index = 0; index < world->joint_entry_count; ++index )
	{
		const v3_joint_entry* joint = world->joint_entries + index;
		if ( joint->is_active && ( ( joint->body_a_logical_id == logical_id && joint->body_a_generation == generation ) ||
								   ( joint->body_b_logical_id == logical_id && joint->body_b_generation == generation ) ) )
		{
			return true;
		}
	}
	return false;
}

v3_status v3_joint_validate_body_removals_internal( const v3_world* world, const v3_body_handle* removals,
													uint32_t removal_count )
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

		int entry_index = v3_joint_find_body_entry( world, removal->logical_id );
		if ( entry_index < 0 || !world->body_entries[entry_index].is_active ||
			 world->body_entries[entry_index].generation != removal->generation )
		{
			return V3_STALE_HANDLE;
		}
		if ( v3_joint_has_attached_body( world, removal->logical_id, removal->generation ) )
		{
			return V3_JOINT_ATTACHED;
		}
	}
	return V3_OK;
}

v3_status v3_world_replace_box_bodies_joint_aware_internal( v3_world* world, const v3_body_handle* removals,
															uint32_t removal_count, const v3_box_body_command* creations,
															uint32_t creation_count )
{
	v3_status status = v3_joint_validate_body_removals_internal( world, removals, removal_count );
	return status == V3_OK
			   ? v3_world_replace_box_bodies_geometry_aware_internal( world, removals, removal_count, creations, creation_count )
			   : status;
}

static v3_status v3_joint_validate_removals( const v3_world* world, const v3_joint_handle* removals, uint32_t removal_count )
{
	for ( uint32_t index = 0; index < removal_count; ++index )
	{
		const v3_joint_handle* removal = removals + index;
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

		int entry_index = v3_joint_find_entry( world, removal->logical_id );
		if ( entry_index < 0 || !world->joint_entries[entry_index].is_active ||
			 world->joint_entries[entry_index].generation != removal->generation )
		{
			return V3_STALE_HANDLE;
		}
	}
	return V3_OK;
}

static v3_status v3_joint_validate_body( const v3_world* world, const v3_body_handle* handle )
{
	if ( handle->logical_id == 0 || handle->generation == 0 || handle->generation > INT32_MAX || handle->reserved != 0 )
	{
		return V3_INVALID_ARGUMENT;
	}
	int entry_index = v3_joint_find_body_entry( world, handle->logical_id );
	if ( entry_index < 0 || !world->body_entries[entry_index].is_active ||
		 world->body_entries[entry_index].generation != handle->generation )
	{
		return V3_STALE_HANDLE;
	}
	return world->body_entries[entry_index].kind == V3_STATIC_BODY ? V3_INVALID_ARGUMENT : V3_OK;
}

static v3_status v3_joint_validate_command( const v3_world* world, const v3_distance_joint_command* command )
{
	if ( command->logical_id == 0 || command->reserved != 0 ||
		 ( command->flags & ~( V3_JOINT_ENABLE_SPRING | V3_JOINT_ENABLE_LIMIT ) ) != 0 )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( command->generation == 0 || command->generation > INT32_MAX )
	{
		return V3_INVALID_GENERATION;
	}
	if ( !isfinite( command->local_anchor_a_x ) || !isfinite( command->local_anchor_a_y ) ||
		 !isfinite( command->local_anchor_a_z ) || !isfinite( command->local_anchor_b_x ) ||
		 !isfinite( command->local_anchor_b_y ) || !isfinite( command->local_anchor_b_z ) || !isfinite( command->rest_length ) ||
		 !isfinite( command->minimum_length ) || !isfinite( command->maximum_length ) || !isfinite( command->hertz ) ||
		 !isfinite( command->damping_ratio ) || !isfinite( command->lower_spring_force ) ||
		 !isfinite( command->upper_spring_force ) )
	{
		return V3_NON_FINITE;
	}
	if ( command->rest_length <= 0.0f || command->minimum_length <= 0.0f || command->maximum_length < command->minimum_length ||
		 command->rest_length < command->minimum_length || command->rest_length > command->maximum_length )
	{
		return V3_INVALID_DIMENSION;
	}
	if ( command->hertz < 0.0f || command->damping_ratio < 0.0f )
	{
		return V3_INVALID_DAMPING;
	}
	if ( command->lower_spring_force > 0.0f || command->upper_spring_force < 0.0f ||
		 command->lower_spring_force > command->upper_spring_force || command->body_a.logical_id == command->body_b.logical_id )
	{
		return V3_INVALID_ARGUMENT;
	}

	v3_status status = v3_joint_validate_body( world, &command->body_a );
	return status == V3_OK ? v3_joint_validate_body( world, &command->body_b ) : status;
}

static v3_status v3_joint_validate_creations( const v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
											  const v3_distance_joint_command* creations, uint32_t creation_count,
											  uint32_t* new_entry_count )
{
	*new_entry_count = 0;
	for ( uint32_t index = 0; index < creation_count; ++index )
	{
		const v3_distance_joint_command* creation = creations + index;
		v3_status status = v3_joint_validate_command( world, creation );
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

		int entry_index = v3_joint_find_entry( world, creation->logical_id );
		if ( entry_index < 0 )
		{
			if ( creation->generation != 1 )
			{
				return V3_INVALID_GENERATION;
			}
			*new_entry_count += 1;
			continue;
		}

		const v3_joint_entry* entry = world->joint_entries + entry_index;
		if ( entry->is_active && !v3_joint_removes_logical_id( removals, removal_count, creation->logical_id ) )
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

static v3_status v3_joint_reserve_entries( v3_world* world, uint32_t required_capacity )
{
	if ( required_capacity <= world->joint_entry_capacity )
	{
		return V3_OK;
	}
	uint32_t new_capacity = world->joint_entry_capacity == 0 ? UINT32_C( 16 ) : world->joint_entry_capacity;
	while ( new_capacity < required_capacity )
	{
		new_capacity = new_capacity > V3_MAX_LOGICAL_JOINT_IDS / 2u ? V3_MAX_LOGICAL_JOINT_IDS : new_capacity * 2u;
	}
	v3_joint_entry* entries = realloc( world->joint_entries, (size_t)new_capacity * sizeof( *entries ) );
	if ( entries == NULL )
	{
		return V3_OUT_OF_MEMORY;
	}
	world->joint_entries = entries;
	world->joint_entry_capacity = new_capacity;
	return V3_OK;
}

static uint32_t v3_joint_saturating_add( uint32_t value, uint32_t increment )
{
	const uint32_t maximum = (uint32_t)INT32_MAX;
	return value >= maximum || maximum - value < increment ? maximum : value + increment;
}

v3_status v3_world_replace_distance_joints_internal( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
													 const v3_distance_joint_command* creations, uint32_t creation_count )
{
	v3_status status = v3_joint_validate_removals( world, removals, removal_count );
	if ( status != V3_OK )
	{
		return status;
	}
	uint32_t new_entry_count = 0;
	status = v3_joint_validate_creations( world, removals, removal_count, creations, creation_count, &new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}

	uint32_t final_joint_count = world->active_joint_count - removal_count + creation_count;
	if ( final_joint_count > V3_MAX_JOINTS_PER_BATCH || world->joint_entry_count + new_entry_count > V3_MAX_LOGICAL_JOINT_IDS )
	{
		return V3_LIMIT_EXCEEDED;
	}
	if ( world->active_joint_count > V3_MAX_JOINTS_PER_BATCH - creation_count )
	{
		return V3_PEAK_LIMIT_EXCEEDED;
	}
	status = v3_joint_reserve_entries( world, world->joint_entry_count + new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}

	v3_pending_joint* pending = NULL;
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
		const v3_distance_joint_command* command = creations + index;
		int body_a_index = v3_joint_find_body_entry( world, command->body_a.logical_id );
		int body_b_index = v3_joint_find_body_entry( world, command->body_b.logical_id );
		b3DistanceJointDef definition = b3DefaultDistanceJointDef();
		definition.base.bodyIdA = world->body_entries[body_a_index].body_id;
		definition.base.bodyIdB = world->body_entries[body_b_index].body_id;
		definition.base.localFrameA = b3Transform_identity;
		definition.base.localFrameA.p =
			(b3Vec3){ command->local_anchor_a_x, command->local_anchor_a_y, command->local_anchor_a_z };
		definition.base.localFrameB = b3Transform_identity;
		definition.base.localFrameB.p =
			(b3Vec3){ command->local_anchor_b_x, command->local_anchor_b_y, command->local_anchor_b_z };
		definition.base.collideConnected = false;
		definition.length = command->rest_length;
		definition.enableSpring = ( command->flags & V3_JOINT_ENABLE_SPRING ) != 0;
		definition.lowerSpringForce = command->lower_spring_force;
		definition.upperSpringForce = command->upper_spring_force;
		definition.hertz = command->hertz;
		definition.dampingRatio = command->damping_ratio;
		definition.enableLimit = ( command->flags & V3_JOINT_ENABLE_LIMIT ) != 0;
		definition.minLength = command->minimum_length;
		definition.maxLength = command->maximum_length;
		definition.enableMotor = false;
		b3JointId joint_id = b3CreateDistanceJoint( world->world_id, &definition );
		if ( B3_IS_NULL( joint_id ) )
		{
			status = V3_NATIVE_FAILURE;
			break;
		}
		pending[created_count++] = (v3_pending_joint){
			.logical_id = command->logical_id,
			.generation = command->generation,
			.joint_id = joint_id,
			.body_a_logical_id = command->body_a.logical_id,
			.body_a_generation = command->body_a.generation,
			.body_b_logical_id = command->body_b.logical_id,
			.body_b_generation = command->body_b.generation,
		};
	}

	if ( status != V3_OK )
	{
		while ( created_count > 0 )
		{
			b3DestroyJoint( pending[--created_count].joint_id, true );
		}
		free( pending );
		return status;
	}

	for ( uint32_t index = 0; index < removal_count; ++index )
	{
		int entry_index = v3_joint_find_entry( world, removals[index].logical_id );
		v3_joint_entry* entry = world->joint_entries + entry_index;
		b3DestroyJoint( entry->joint_id, true );
		entry->is_active = false;
	}
	for ( uint32_t index = 0; index < creation_count; ++index )
	{
		const v3_pending_joint* created = pending + index;
		int entry_index = v3_joint_find_entry( world, created->logical_id );
		if ( entry_index < 0 )
		{
			entry_index = (int)world->joint_entry_count++;
		}
		world->joint_entries[entry_index] = (v3_joint_entry){
			.logical_id = created->logical_id,
			.generation = created->generation,
			.joint_id = created->joint_id,
			.body_a_logical_id = created->body_a_logical_id,
			.body_a_generation = created->body_a_generation,
			.body_b_logical_id = created->body_b_logical_id,
			.body_b_generation = created->body_b_generation,
			.is_active = true,
		};
	}

	world->active_joint_count = final_joint_count;
	if ( removal_count > 0 || creation_count > 0 )
	{
		world->mutation_batch_count = v3_joint_saturating_add( world->mutation_batch_count, 1 );
	}
	free( pending );
	return V3_OK;
}

void v3_joint_destroy_state_internal( v3_world* world )
{
	if ( world == NULL )
	{
		return;
	}
	free( world->joint_entries );
	world->joint_entries = NULL;
}

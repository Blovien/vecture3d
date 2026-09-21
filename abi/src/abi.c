// SPDX-License-Identifier: MIT

#include "internal.h"

#include <float.h>
#include <math.h>

uint32_t v3_abi_version( void )
{
	return V3_ABI_VERSION;
}

uint64_t v3_abi_schema_hash( void )
{
	return V3_ABI_SCHEMA_HASH;
}

static v3_status v3_validate_limits( const v3_world_limits* limits )
{
	if ( ( limits->flags & ~V3_WORLD_DISABLE_SLEEP ) != 0 )
	{
		return V3_INVALID_ARGUMENT;
	}

	if ( !isfinite( limits->maximum_linear_speed ) || limits->maximum_linear_speed < 0.0f ||
		 !isfinite( limits->maximum_angular_speed ) || limits->maximum_angular_speed < 0.0f ||
		 limits->projectile_candidate_cap < 0 )
	{
		return V3_NON_FINITE;
	}

	return V3_OK;
}

v3_world* v3_world_create( double gravity_x, double gravity_y, double gravity_z, const v3_world_limits* limits )
{
	if ( !isfinite( gravity_x ) || !isfinite( gravity_y ) || !isfinite( gravity_z ) || fabs( gravity_x ) > FLT_MAX ||
		 fabs( gravity_y ) > FLT_MAX || fabs( gravity_z ) > FLT_MAX )
	{
		return NULL;
	}

	if ( limits != NULL && v3_validate_limits( limits ) != V3_OK )
	{
		return NULL;
	}

	return v3_world_create_internal( gravity_x, gravity_y, gravity_z, limits );
}

v3_status v3_world_set_limits( v3_world* world, const v3_world_limits* limits )
{
	if ( world == NULL || limits == NULL )
	{
		return V3_INVALID_ARGUMENT;
	}

	v3_status status = v3_validate_limits( limits );
	if ( status != V3_OK )
	{
		return status;
	}

	v3_world_set_limits_internal( world, limits );
	return V3_OK;
}

v3_status v3_world_get_limits( const v3_world* world, v3_world_limits* limits )
{
	if ( world == NULL || limits == NULL )
	{
		return V3_INVALID_ARGUMENT;
	}

	v3_world_get_limits_internal( world, limits );
	return V3_OK;
}

v3_status v3_world_replace_box_bodies( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
									   const v3_box_body_command* creations, uint32_t creation_count )
{
	if ( world == NULL || ( removal_count > 0 && removals == NULL ) || ( creation_count > 0 && creations == NULL ) )
	{
		return V3_INVALID_ARGUMENT;
	}

	if ( removal_count > V3_MAX_BODIES_PER_BATCH || creation_count > V3_MAX_BODIES_PER_BATCH )
	{
		return V3_LIMIT_EXCEEDED;
	}

	return v3_world_replace_box_bodies_joint_aware_internal( world, removals, removal_count, creations, creation_count );
}

v3_status v3_world_create_hull_body( v3_world* world, const v3_box_body_command* command, const float* point_xyz,
									 uint32_t point_count )
{
	if ( world == NULL || command == NULL || point_xyz == NULL || point_count < 4u || point_count > V3_MAX_HULL_POINTS )
	{
		return V3_INVALID_ARGUMENT;
	}

	return v3_world_create_hull_body_internal( world, command, point_xyz, point_count );
}

v3_status v3_world_create_sphere_body( v3_world* world, const v3_body_definition* body, float radius, float density,
									   float friction, v3_body_handle* out )
{
	if ( world == NULL || body == NULL || out == NULL )
	{
		return V3_INVALID_ARGUMENT;
	}

	return v3_world_create_sphere_body_internal( world, body, radius, density, friction, out );
}

v3_status v3_cook_block_grid( const v3_block_material* materials, uint32_t material_count, const v3_block_cell* cells,
							  uint32_t cell_count, const v3_block_box* boxes, uint32_t box_count, v3_cooked_grid** out )
{
	if ( out == NULL )
	{
		return V3_INVALID_ARGUMENT;
	}

	// Every failure publishes no handle, so a caller that ignores the status still owns nothing.
	*out = NULL;
	if ( materials == NULL || cells == NULL || boxes == NULL || material_count == 0 || cell_count == 0 || box_count == 0 )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( material_count > V3_MAX_BLOCK_GRID_ITEMS || cell_count > V3_MAX_BLOCK_GRID_ITEMS || box_count > V3_MAX_BLOCK_GRID_ITEMS )
	{
		return V3_LIMIT_EXCEEDED;
	}

	return v3_cook_block_grid_internal( materials, material_count, cells, cell_count, boxes, box_count, out );
}

v3_status v3_world_attach_block_grid( v3_world* world, const v3_body_definition* body, const v3_cooked_grid* grid,
									  const v3_mass_properties* mass_override, v3_body_handle* out )
{
	if ( world == NULL || body == NULL || grid == NULL || out == NULL )
	{
		return V3_INVALID_ARGUMENT;
	}

	return v3_world_attach_block_grid_internal( world, body, grid, mass_override, out );
}

v3_status v3_world_replace_block_grid( v3_world* world, const v3_body_handle* body, const v3_cooked_grid* grid )
{
	if ( world == NULL || body == NULL || grid == NULL )
	{
		return V3_INVALID_ARGUMENT;
	}
	return v3_world_replace_block_grid_internal( world, body, grid );
}

void v3_destroy_cooked_grid( v3_cooked_grid* grid )
{
	v3_destroy_cooked_grid_internal( grid );
}

v3_status v3_world_replace_distance_joints( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
											const v3_distance_joint_command* creations, uint32_t creation_count )
{
	if ( world == NULL || ( removal_count > 0 && removals == NULL ) || ( creation_count > 0 && creations == NULL ) )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( removal_count > V3_MAX_JOINTS_PER_BATCH || creation_count > V3_MAX_JOINTS_PER_BATCH )
	{
		return V3_LIMIT_EXCEEDED;
	}
	return v3_world_replace_distance_joints_internal( world, removals, removal_count, creations, creation_count );
}

v3_status v3_world_replace_revolute_joints( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
											const v3_revolute_joint_command* creations, uint32_t creation_count )
{
	if ( world == NULL || ( removal_count > 0 && removals == NULL ) || ( creation_count > 0 && creations == NULL ) )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( removal_count > V3_MAX_JOINTS_PER_BATCH || creation_count > V3_MAX_JOINTS_PER_BATCH )
	{
		return V3_LIMIT_EXCEEDED;
	}
	return v3_world_replace_revolute_joints_internal( world, removals, removal_count, creations, creation_count );
}

v3_status v3_world_step_and_read( v3_world* world, const v3_kinematic_target* targets, uint32_t target_count,
								  const v3_body_wrench* wrenches, uint32_t wrench_count, const v3_query* queries,
								  uint32_t query_count, uint32_t fixed_step_count, v3_transform* transforms,
								  uint32_t transform_capacity, v3_query_result* query_results, uint32_t query_result_capacity,
								  v3_step_stats* stats )
{
	if ( world == NULL || stats == NULL || ( target_count > 0 && targets == NULL ) || ( wrench_count > 0 && wrenches == NULL ) ||
		 ( query_count > 0 && queries == NULL ) )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( target_count > V3_MAX_BODIES_PER_BATCH || wrench_count > V3_MAX_WRENCHES_PER_BATCH ||
		 query_count > V3_MAX_QUERIES_PER_BATCH || fixed_step_count > V3_MAX_FIXED_STEPS )
	{
		return V3_LIMIT_EXCEEDED;
	}
	return v3_world_step_and_read_internal( world, targets, target_count, wrenches, wrench_count, queries, query_count,
											fixed_step_count, transforms, transform_capacity, query_results,
											query_result_capacity, stats );
}

v3_status v3_world_get_block_contact_events( const v3_world* world, v3_block_contact_event* events, uint32_t event_capacity,
											 uint32_t* event_count )
{
	if ( world == NULL || event_count == NULL )
	{
		return V3_INVALID_ARGUMENT;
	}
	return v3_world_get_block_contact_events_internal( world, events, event_capacity, event_count );
}

void v3_world_destroy( v3_world* world )
{
	v3_joint_destroy_state_internal( world );
	v3_world_destroy_internal( world );
}

uint32_t v3_active_world_count( void )
{
	return v3_active_world_count_internal();
}

// SPDX-License-Identifier: MIT

#include "v3_internal.h"

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

v3_world* v3_world_create( double gravity_x, double gravity_y, double gravity_z )
{
	if ( !isfinite( gravity_x ) || !isfinite( gravity_y ) || !isfinite( gravity_z ) || fabs( gravity_x ) > FLT_MAX ||
		 fabs( gravity_y ) > FLT_MAX || fabs( gravity_z ) > FLT_MAX )
	{
		return NULL;
	}

	return v3_world_create_internal( gravity_x, gravity_y, gravity_z );
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

v3_status v3_world_replace_terrain_sections( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
											 const v3_terrain_section_command* sections, uint32_t section_count,
											 const v3_voxel_run* voxel_runs, uint32_t voxel_run_count,
											 const v3_terrain_box* detail_boxes, uint32_t detail_box_count )
{
	if ( world == NULL || ( removal_count > 0 && removals == NULL ) || ( section_count > 0 && sections == NULL ) ||
		 ( voxel_run_count > 0 && voxel_runs == NULL ) || ( detail_box_count > 0 && detail_boxes == NULL ) )
	{
		return V3_INVALID_ARGUMENT;
	}

	if ( removal_count > V3_MAX_BODIES_PER_BATCH || section_count > V3_MAX_BODIES_PER_BATCH ||
		 voxel_run_count > V3_MAX_VOXEL_RUNS_PER_BATCH || detail_box_count > V3_MAX_TERRAIN_DETAIL_BOXES_PER_BATCH )
	{
		return V3_LIMIT_EXCEEDED;
	}
	v3_status status = v3_joint_validate_body_removals_internal( world, removals, removal_count );
	if ( status != V3_OK )
	{
		return status;
	}

	return v3_world_replace_terrain_sections_internal( world, removals, removal_count, sections, section_count, voxel_runs,
													   voxel_run_count, detail_boxes, detail_box_count );
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

v3_status v3_world_create_voxel_group( v3_world* world, const v3_box_body_command* command, const v3_terrain_box* boxes,
									   uint32_t box_count, const v3_mass_properties* mass_properties )
{
	if ( world == NULL || command == NULL || boxes == NULL || mass_properties == NULL || box_count == 0 ||
		 box_count > V3_MAX_VOXEL_CHILDREN_PER_SECTION )
	{
		return V3_INVALID_ARGUMENT;
	}

	return v3_world_create_voxel_group_internal( world, command, boxes, box_count, mass_properties );
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

void v3_world_destroy( v3_world* world )
{
	v3_geometry_destroy_owned_payloads_internal( world );
	v3_joint_destroy_state_internal( world );
	v3_world_destroy_internal( world );
}

uint32_t v3_active_world_count( void )
{
	return v3_active_world_count_internal();
}

// SPDX-License-Identifier: MIT

#ifndef V3_INTERNAL_H
#define V3_INTERNAL_H

#include "v3_abi.h"

#include <box3d/box3d.h>
#include <stdbool.h>
#include <stdint.h>

#define V3_MAX_BODIES_PER_BATCH UINT32_C( 4096 )
#define V3_MAX_LOGICAL_BODY_IDS UINT32_C( 4096 )
#define V3_MAX_VOXEL_CHILDREN_PER_SECTION UINT32_C( 65534 )
#define V3_MAX_VOXEL_RUNS_PER_BATCH UINT32_C( 262144 )
#define V3_MAX_TERRAIN_DETAIL_BOXES_PER_BATCH UINT32_C( 262144 )
#define V3_VOXEL_SECTION_SIZE UINT32_C( 32 )
#define V3_VOXEL_RUN_USED_BITS UINT32_C( 0x3fffffff )
#define V3_MAX_HULL_POINTS UINT32_C( 64 )
#define V3_MAX_JOINTS_PER_BATCH UINT32_C( 4096 )
#define V3_MAX_LOGICAL_JOINT_IDS UINT32_C( 4096 )
#define V3_MAX_WRENCHES_PER_BATCH UINT32_C( 4096 )
#define V3_MAX_QUERIES_PER_BATCH UINT32_C( 1024 )
#define V3_MAX_FIXED_STEPS UINT32_C( 4 )
#define V3_MAX_DAMPING 10.0f
#define V3_MAX_FORCE_MAGNITUDE 1000000.0f
#define V3_MAX_TORQUE_MAGNITUDE 1000000.0f
#define V3_DEFAULT_VOXEL_FRICTION 0.7f
#define V3_FIXED_TIME_STEP ( 1.0f / 60.0f )
#define V3_SUB_STEP_COUNT 4
#define V3_TRANSFORM_AWAKE UINT32_C( 1 )
#define V3_TARGET_WAKE UINT32_C( 1 )
#define V3_GRAPH_OVERFLOW_INDEX UINT32_C( 23 )

#define V3_STATIC_CATEGORY ( UINT64_C( 1 ) << 0u )
#define V3_KINEMATIC_CATEGORY ( UINT64_C( 1 ) << 1u )
#define V3_DYNAMIC_CATEGORY ( UINT64_C( 1 ) << 2u )
#define V3_ALL_BODY_CATEGORIES ( V3_STATIC_CATEGORY | V3_KINEMATIC_CATEGORY | V3_DYNAMIC_CATEGORY )

typedef struct v3_body_entry
{
	uint64_t logical_id;
	uint32_t generation;
	b3BodyId body_id;
	b3ShapeId shape_id;
	b3CompoundData* owned_voxel;
	b3MassData mass_data;
	uint32_t kind;
	bool has_explicit_mass_data;
	bool is_active;
} v3_body_entry;

typedef struct v3_joint_entry
{
	uint64_t logical_id;
	uint32_t generation;
	b3JointId joint_id;
	uint64_t body_a_logical_id;
	uint32_t body_a_generation;
	uint64_t body_b_logical_id;
	uint32_t body_b_generation;
	bool is_active;
} v3_joint_entry;

struct v3_world
{
	b3WorldId world_id;
	v3_body_entry* body_entries;
	uint32_t body_entry_count;
	uint32_t body_entry_capacity;
	uint32_t active_body_count;
	v3_joint_entry* joint_entries;
	uint32_t joint_entry_count;
	uint32_t joint_entry_capacity;
	uint32_t active_joint_count;
	uint32_t mutation_batch_count;
	uint32_t created_body_count;
	uint32_t destroyed_body_count;
};

v3_world* v3_world_create_internal( double gravity_x, double gravity_y, double gravity_z );
v3_status v3_world_replace_box_bodies_internal( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
												const v3_box_body_command* creations, uint32_t creation_count );
v3_status v3_world_replace_box_bodies_geometry_aware_internal( v3_world* world, const v3_body_handle* removals,
															   uint32_t removal_count, const v3_box_body_command* creations,
															   uint32_t creation_count );
v3_status v3_world_replace_box_bodies_joint_aware_internal( v3_world* world, const v3_body_handle* removals,
															uint32_t removal_count, const v3_box_body_command* creations,
															uint32_t creation_count );
v3_status v3_joint_validate_body_removals_internal( const v3_world* world, const v3_body_handle* removals,
													uint32_t removal_count );
v3_status v3_world_replace_terrain_sections_internal( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
													  const v3_terrain_section_command* sections, uint32_t section_count,
													  const v3_voxel_run* voxel_runs, uint32_t voxel_run_count,
													  const v3_terrain_box* detail_boxes, uint32_t detail_box_count );
v3_status v3_world_create_hull_body_internal( v3_world* world, const v3_box_body_command* command, const float* point_xyz,
											  uint32_t point_count );
v3_status v3_world_create_voxel_group_internal( v3_world* world, const v3_box_body_command* command, const v3_terrain_box* boxes,
												uint32_t box_count, const v3_mass_properties* mass_properties );
void v3_geometry_destroy_owned_payloads_internal( v3_world* world );
v3_status v3_world_replace_distance_joints_internal( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
													 const v3_distance_joint_command* creations, uint32_t creation_count );
v3_status v3_world_step_and_read_internal( v3_world* world, const v3_kinematic_target* targets, uint32_t target_count,
										   const v3_body_wrench* wrenches, uint32_t wrench_count, const v3_query* queries,
										   uint32_t query_count, uint32_t fixed_step_count, v3_transform* transforms,
										   uint32_t transform_capacity, v3_query_result* query_results,
										   uint32_t query_result_capacity, v3_step_stats* stats );
void v3_joint_destroy_state_internal( v3_world* world );
void v3_world_destroy_internal( v3_world* world );
uint32_t v3_active_world_count_internal( void );

typedef enum v3_test_fault
{
	V3_TEST_FAULT_NONE,
	V3_TEST_FAULT_WORLD_CALLOC,
	V3_TEST_FAULT_BODY_ENTRIES_REALLOC,
	V3_TEST_FAULT_PENDING_CALLOC,
	V3_TEST_FAULT_CREATE_BODY,
	V3_TEST_FAULT_CREATE_SHAPE,
} v3_test_fault;

#if defined( V3_TESTING )
void v3_test_fail_after( v3_test_fault fault, uint32_t successful_calls_before_failure );
#endif

#endif

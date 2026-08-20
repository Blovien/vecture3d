// SPDX-License-Identifier: MIT

#ifndef V3_ABI_H
#define V3_ABI_H

// clang-format off
#include <stddef.h>
#include <stdint.h>
#include "v3_abi_manifest.h"
// clang-format on

#if defined( _WIN32 )
#define V3_API __declspec( dllexport )
#else
#define V3_API __attribute__( ( visibility( "default" ) ) )
#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef uint32_t v3_status;
typedef struct v3_world v3_world;

#define V3_OK UINT32_C( 0 )
#define V3_INVALID_ARGUMENT UINT32_C( 1 )
#define V3_LIMIT_EXCEEDED UINT32_C( 2 )
#define V3_DUPLICATE_ID UINT32_C( 3 )
#define V3_OUT_OF_MEMORY UINT32_C( 4 )
#define V3_NATIVE_FAILURE UINT32_C( 5 )
#define V3_OUTPUT_TOO_SMALL UINT32_C( 6 )
#define V3_NON_FINITE UINT32_C( 7 )
#define V3_INVALID_DIMENSION UINT32_C( 8 )
#define V3_INVALID_DENSITY UINT32_C( 9 )
#define V3_INVALID_FRICTION UINT32_C( 10 )
#define V3_STALE_HANDLE UINT32_C( 11 )
#define V3_INVALID_GENERATION UINT32_C( 12 )
#define V3_PEAK_LIMIT_EXCEEDED UINT32_C( 13 )
#define V3_GENERATION_EXHAUSTED UINT32_C( 14 )
#define V3_INVALID_DAMPING UINT32_C( 15 )
#define V3_INVALID_QUATERNION UINT32_C( 16 )
#define V3_INVALID_TARGET UINT32_C( 17 )
#define V3_JOINT_ATTACHED UINT32_C( 18 )
#define V3_INVALID_WRENCH UINT32_C( 19 )
#define V3_INVALID_MATERIAL UINT32_C( 20 )
#define V3_INVALID_CELL UINT32_C( 21 )
#define V3_DUPLICATE_CELL UINT32_C( 22 )
#define V3_INVALID_FEATURE UINT32_C( 23 )
#define V3_DUPLICATE_FEATURE UINT32_C( 24 )
#define V3_INVALID_OWNER UINT32_C( 25 )
#define V3_INVALID_BLOCK_FIELD UINT32_C( 26 )
#define V3_PENDING_FRACTURE_PLAN UINT32_C( 27 )

// Fracture assigned definitions for later
#define V3_FRACTURE_PLAN_NOT_BOUND UINT32_C( 28 )
#define V3_FRACTURE_PLAN_STALE UINT32_C( 29 )
#define V3_FRACTURE_PLAN_CONSUMED UINT32_C( 30 )

#define V3_NO_FRACTURE UINT32_C( 35 )
// end

#define V3_CAPACITY_RESERVATION_FAILED UINT32_C( 31 )
#define V3_SOURCE_REVISION_MISMATCH UINT32_C( 32 )
#define V3_INVALID_FRACTURE_TRIGGER UINT32_C( 33 )
#define V3_WORLD_POISONED UINT32_C( 34 )


#define V3_STATIC_BODY UINT32_C( 0 )
#define V3_KINEMATIC_BODY UINT32_C( 1 )
#define V3_DYNAMIC_BODY UINT32_C( 2 )

#define V3_BODY_ENABLE_SLEEP UINT32_C( 1 )
#define V3_BODY_INITIAL_AWAKE UINT32_C( 2 )
#define V3_BODY_DISABLE_COLLISION UINT32_C( 4 )

#define V3_JOINT_ENABLE_SPRING UINT32_C( 1 )
#define V3_JOINT_ENABLE_LIMIT UINT32_C( 2 )

#define V3_WRENCH_WAKE UINT32_C( 1 )

#define V3_QUERY_NO_HIT UINT32_C( 0 )
#define V3_QUERY_IMMEDIATE_BLOCK UINT32_C( 1 )
#define V3_QUERY_CAST_HIT UINT32_C( 2 )

typedef struct v3_box_body_command
{
	uint64_t logical_id;
	uint32_t kind;
	uint32_t generation;
	double position_x;
	double position_y;
	double position_z;
	float rotation_x;
	float rotation_y;
	float rotation_z;
	float rotation_w;
	float linear_velocity_x;
	float linear_velocity_y;
	float linear_velocity_z;
	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;
	float half_extent_x;
	float half_extent_y;
	float half_extent_z;
	float density;
	float friction;
	float linear_damping;
	float angular_damping;
	uint32_t flags;
} v3_box_body_command;

typedef struct v3_body_handle
{
	uint64_t logical_id;
	uint32_t generation;
	uint32_t reserved;
} v3_body_handle;

typedef struct v3_terrain_section_command
{
	uint64_t logical_id;
	uint32_t generation;
	uint32_t voxel_run_offset;
	uint32_t voxel_run_count;
	uint32_t detail_box_offset;
	uint32_t detail_box_count;
	uint32_t reserved;
	double origin_x;
	double origin_y;
	double origin_z;
} v3_terrain_section_command;

typedef struct v3_voxel_run
{
	uint32_t packed;
} v3_voxel_run;

typedef struct v3_terrain_box
{
	float center_x;
	float center_y;
	float center_z;
	float half_extent_x;
	float half_extent_y;
	float half_extent_z;
	float friction;
	uint32_t reserved;
	uint64_t feature_id;
} v3_terrain_box;

typedef struct v3_mass_properties
{
	float mass;
	float center_x;
	float center_y;
	float center_z;
	float inertia_xx;
	float inertia_yy;
	float inertia_zz;
	float inertia_xy;
	float inertia_xz;
	float inertia_yz;
} v3_mass_properties;

typedef struct v3_distance_joint_command
{
	uint64_t logical_id;
	uint32_t generation;
	uint32_t reserved;
	v3_body_handle body_a;
	v3_body_handle body_b;
	float local_anchor_a_x;
	float local_anchor_a_y;
	float local_anchor_a_z;
	float local_anchor_b_x;
	float local_anchor_b_y;
	float local_anchor_b_z;
	float rest_length;
	float minimum_length;
	float maximum_length;
	float hertz;
	float damping_ratio;
	float lower_spring_force;
	float upper_spring_force;
	uint32_t flags;
} v3_distance_joint_command;

typedef struct v3_joint_handle
{
	uint64_t logical_id;
	uint32_t generation;
	uint32_t reserved;
} v3_joint_handle;

typedef struct v3_kinematic_target
{
	uint64_t logical_id;
	uint32_t generation;
	uint32_t flags;
	double position_x;
	double position_y;
	double position_z;
	float rotation_x;
	float rotation_y;
	float rotation_z;
	float rotation_w;
} v3_kinematic_target;

typedef struct v3_body_wrench
{
	uint64_t logical_id;
	uint32_t generation;
	uint32_t flags;
	float force_x;
	float force_y;
	float force_z;
	float torque_x;
	float torque_y;
	float torque_z;
} v3_body_wrench;

typedef struct v3_query
{
	uint64_t query_id;
	double origin_x;
	double origin_y;
	double origin_z;
	float half_extent_x;
	float half_extent_y;
	float half_extent_z;
	float translation_x;
	float translation_y;
	float translation_z;
	uint32_t reserved;
} v3_query;

typedef struct v3_query_result
{
	uint64_t query_id;
	uint32_t status;
	uint32_t reserved;
	uint64_t hit_logical_id;
	float fraction;
	float reserved2;
} v3_query_result;

typedef struct v3_transform
{
	uint64_t logical_id;
	uint32_t flags;
	uint32_t generation;
	double position_x;
	double position_y;
	double position_z;
	float rotation_x;
	float rotation_y;
	float rotation_z;
	float rotation_w;
	float linear_velocity_x;
	float linear_velocity_y;
	float linear_velocity_z;
	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;
	float local_center_x;
	float local_center_y;
	float local_center_z;
	uint32_t reserved;
} v3_transform;

typedef struct v3_step_stats
{
	uint32_t output_count;
	uint32_t body_count;
	uint32_t shape_count;
	uint32_t contact_count;
	float step_milliseconds;
	uint32_t mutation_batch_count;
	uint32_t created_body_count;
	uint32_t destroyed_body_count;
	uint32_t query_output_count;
	uint32_t query_count;
	uint32_t fixed_step_count;
	uint32_t joint_count;
	float pair_milliseconds;
	float collide_milliseconds;
	float solve_milliseconds;
	uint32_t static_tree_height;
	uint32_t dynamic_tree_height;
	uint32_t sat_call_count;
	uint32_t sat_cache_hit_count;
	uint32_t graph_overflow_constraint_count;
	uint32_t heap_move_pair_count;
} v3_step_stats;

// These records reserve approved schema layouts; milestone 1 exposes no operations for them.
typedef struct v3_body_definition
{
	v3_body_handle handle;
	uint32_t kind;
	uint32_t flags;
	double position_x;
	double position_y;
	double position_z;
	float rotation_x;
	float rotation_y;
	float rotation_z;
	float rotation_w;
	float linear_velocity_x;
	float linear_velocity_y;
	float linear_velocity_z;
	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;
	float linear_damping;
	float angular_damping;
} v3_body_definition;

typedef struct v3_block_material
{
	uint64_t material_id;
	float density;
	float friction;
	float restitution;
	float bond_strength;
	float compressive_factor;
	uint32_t flags;
} v3_block_material;

typedef struct v3_block_cell
{
	int32_t x;
	int32_t y;
	int32_t z;
	uint32_t material_index;
	uint64_t feature_id;
	uint32_t flags;
	uint32_t reserved;
} v3_block_cell;

typedef struct v3_block_box
{
	float center_x;
	float center_y;
	float center_z;
	float half_extent_x;
	float half_extent_y;
	float half_extent_z;
	uint32_t owner_cell_index;
	uint32_t material_index;
	uint64_t feature_id;
} v3_block_box;

typedef struct v3_block_field_command
{
	v3_body_definition body;
	uint64_t source_revision;
	uint32_t material_offset;
	uint32_t material_count;
	uint32_t cell_offset;
	uint32_t cell_count;
	uint32_t box_offset;
	uint32_t box_count;
} v3_block_field_command;

typedef struct v3_block_contact_event
{
	v3_body_handle body_a;
	v3_body_handle body_b;
	uint64_t feature_id_a;
	uint64_t feature_id_b;
	float point_x;
	float point_y;
	float point_z;
	float normal_x;
	float normal_y;
	float normal_z;
	float impulse_x;
	float impulse_y;
	float impulse_z;
	float relative_normal_speed;
	uint32_t flags;
	uint32_t fixed_step_index;
} v3_block_contact_event;

typedef struct v3_fracture_plan_handle
{
	uint64_t logical_id;
	uint32_t generation;
	uint32_t reserved;
} v3_fracture_plan_handle;

typedef struct v3_fracture_trigger
{
	v3_body_handle target;
	uint64_t expected_source_revision;
	uint64_t feature_id_a;
	uint64_t feature_id_b;
	uint32_t kind;
	uint32_t flags;
	float point_x;
	float point_y;
	float point_z;
	float impulse_x;
	float impulse_y;
	float impulse_z;
	float radius;
	float reserved;
} v3_fracture_trigger;

typedef struct v3_fracture_plan_summary
{
	v3_fracture_plan_handle plan;
	uint64_t frozen_world_epoch;
	uint32_t field_mutation_count;
	uint32_t removed_cell_count;
	uint32_t fragment_count;
	uint32_t fragment_cell_count;
	uint32_t dust_cell_count;
	uint32_t required_fragment_assignment_count;
} v3_fracture_plan_summary;

typedef struct v3_field_revision
{
	v3_body_handle body;
	uint64_t source_revision;
} v3_field_revision;

typedef struct v3_removed_cell
{
	v3_body_handle source_body;
	uint64_t feature_id;
} v3_removed_cell;

typedef struct v3_fragment_assignment
{
	uint32_t fragment_index;
	uint32_t reserved;
	v3_body_handle body;
} v3_fragment_assignment;

typedef struct v3_fragment_descriptor
{
	uint32_t fragment_index;
	uint32_t flags;
	v3_body_handle parent;
	uint64_t source_revision;
	uint32_t cell_offset;
	uint32_t cell_count;
	double position_x;
	double position_y;
	double position_z;
	float rotation_x;
	float rotation_y;
	float rotation_z;
	float rotation_w;
	float linear_velocity_x;
	float linear_velocity_y;
	float linear_velocity_z;
	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;
	float local_center_x;
	float local_center_y;
	float local_center_z;
	float mass;
	float inertia_xx;
	float inertia_yy;
	float inertia_zz;
	float inertia_xy;
	float inertia_xz;
	float inertia_yz;
	uint32_t reserved0;
	uint32_t reserved1;
} v3_fragment_descriptor;

V3_API uint32_t v3_abi_version( void );
V3_API uint64_t v3_abi_schema_hash( void );
V3_API v3_world* v3_world_create( double gravity_x, double gravity_y, double gravity_z );
V3_API v3_status v3_world_replace_box_bodies( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
											  const v3_box_body_command* creations, uint32_t creation_count );
V3_API v3_status v3_world_replace_terrain_sections( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
													const v3_terrain_section_command* sections, uint32_t section_count,
													const v3_voxel_run* voxel_runs, uint32_t voxel_run_count,
													const v3_terrain_box* detail_boxes, uint32_t detail_box_count );
V3_API v3_status v3_world_create_hull_body( v3_world* world, const v3_box_body_command* command, const float* point_xyz,
											uint32_t point_count );
V3_API v3_status v3_world_create_voxel_group( v3_world* world, const v3_box_body_command* command, const v3_terrain_box* boxes,
											  uint32_t box_count, const v3_mass_properties* mass_properties );
V3_API v3_status v3_world_replace_distance_joints( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
												   const v3_distance_joint_command* creations, uint32_t creation_count );
V3_API v3_status v3_world_step_and_read( v3_world* world, const v3_kinematic_target* targets, uint32_t target_count,
										 const v3_body_wrench* wrenches, uint32_t wrench_count, const v3_query* queries,
										 uint32_t query_count, uint32_t fixed_step_count, v3_transform* transforms,
										 uint32_t transform_capacity, v3_query_result* query_results,
										 uint32_t query_result_capacity, v3_step_stats* stats );
V3_API void v3_world_destroy( v3_world* world );
V3_API uint32_t v3_active_world_count( void );

#ifdef __cplusplus
}
#endif

#endif

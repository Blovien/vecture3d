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
// Immutable cooked BlockGrid geometry. One handle owns one reference to the cooked data, is
// independent of any world, and may back shapes in several worlds at once.
typedef struct v3_cooked_grid v3_cooked_grid;

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
#define V3_INVALID_BLOCK_GRID UINT32_C( 26 )
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
/// Enable continuous projectile collision for a dynamic body. Static and kinematic bodies reject
/// this flag. Sweeps cover static, kinematic and non-bullet dynamic bodies, not other bullets.
#define V3_BODY_BULLET UINT32_C( 8 )

#define V3_WORLD_DISABLE_SLEEP UINT32_C( 1 )

#define V3_JOINT_ENABLE_SPRING UINT32_C( 1 )
#define V3_JOINT_ENABLE_LIMIT UINT32_C( 2 )
#define V3_JOINT_ENABLE_MOTOR UINT32_C( 4 )
#define V3_JOINT_COLLIDE_CONNECTED UINT32_C( 8 )

#define V3_WRENCH_WAKE UINT32_C( 1 )

#define V3_QUERY_NO_HIT UINT32_C( 0 )
#define V3_QUERY_IMMEDIATE_BLOCK UINT32_C( 1 )
#define V3_QUERY_CAST_HIT UINT32_C( 2 )

#define V3_QUERY_LEGACY_BOX UINT32_C( 0 )
#define V3_QUERY_RAY UINT32_C( 1 )
#define V3_QUERY_SPHERE UINT32_C( 2 )
#define V3_QUERY_CAPSULE UINT32_C( 3 )
#define V3_QUERY_BOX UINT32_C( 4 )
#define V3_QUERY_HULL UINT32_C( 5 )
#define V3_QUERY_MAX_POINTS UINT32_C( 64 )
#define V3_QUERY_RESULT_BLOCK_GRID UINT32_C( 1 )

#define V3_BLOCK_CONTACT_BEGIN UINT32_C( 1 )
#define V3_BLOCK_CONTACT_END UINT32_C( 2 )
#define V3_BLOCK_CONTACT_HIT UINT32_C( 4 )
#define V3_BLOCK_CONTACT_KIND_MASK ( V3_BLOCK_CONTACT_BEGIN | V3_BLOCK_CONTACT_END | V3_BLOCK_CONTACT_HIT )
#define V3_BLOCK_CONTACT_SIDE_GRID UINT32_C( 1 )
#define V3_BLOCK_CONTACT_EVENTS_TRUNCATED UINT32_C( 1 )
// Four fixed steps times the native bound of 256 events for each of END, BEGIN and HIT.
#define V3_BLOCK_CONTACT_EVENT_CAPACITY UINT32_C( 3072 )

// World motion limits and configuration. Explicit layout: four 4 byte fields at offsets 0, 4, 8 and 12,
// size 16, alignment 4, no implicit padding.
//
// Motion is resolved at step boundaries. A body allowed to travel more than about one cell per step
// can pass clean through one-cell-thick geometry, because the step that carries it across leaves no
// overlap for the solver to see; a fast enough spin does the same with a far corner. One cell per step
// is the safe bound: measured against a one-cell-thick BlockGrid wall with four sub-steps a BlockGrid
// hull still stops at two cells per step and passes clean through at three, but that margin belongs to
// the scene, not to the engine. Bound maximum_linear_speed with the smallest cell size the game builds
// from divided by the step length. The game owns the trade between reach and speed; the engine only
// reports the consequence.
typedef struct v3_world_limits
{
	// Meters per second. Zero selects the engine default (400 m/s).
	float maximum_linear_speed;
	// Radians per second. Zero selects the engine default, the per-step rotation clamp.
	float maximum_angular_speed;
	// Maximum Projectile sweep candidates per sweep. Zero means unlimited.
	int32_t projectile_candidate_cap;
	// V3_WORLD_DISABLE_SLEEP disables island sleeping. Zero enables sleeping.
	uint32_t flags;
} v3_world_limits;

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

/// Revolute constraint between two distinct dynamic or kinematic bodies, using the shared
/// distance/revolute logical joint identity. Local anchors are in meters from each body origin.
/// Local rotations are normalized quaternions in xyzw order. Their squared norm must differ
/// from 1 by less than 20*FLT_EPSILON.
/// Each frame's local z-axis is the hinge axis. Rotation of frame B relative to frame A
/// about that axis defines the signed angle in radians, using the right hand rule.
/// All floats must be finite. target_angle is the spring position target in [-pi, pi];
/// motor_speed is the velocity target in radians per second. hertz, damping_ratio and
/// max_motor_torque (newton meters) must be nonnegative. lower_angle <= upper_angle and both
/// must lie in [-0.99*pi, 0.99*pi], even when limits are disabled. The spring target need
/// not lie within the limits. flags accepts SPRING, LIMIT, MOTOR and COLLIDE_CONNECTED;
/// a clear flag disables that feature. Connected collision is disabled by default.
/// reserved0 must be zero. generation is in [1, INT32_MAX].
typedef struct v3_revolute_joint_command
{
	uint64_t logical_id;
	uint32_t generation;
	uint32_t flags;
	v3_body_handle body_a;
	v3_body_handle body_b;
	float local_anchor_a_x;
	float local_anchor_a_y;
	float local_anchor_a_z;
	float local_rotation_a_x;
	float local_rotation_a_y;
	float local_rotation_a_z;
	float local_rotation_a_w;
	float local_anchor_b_x;
	float local_anchor_b_y;
	float local_anchor_b_z;
	float local_rotation_b_x;
	float local_rotation_b_y;
	float local_rotation_b_z;
	float local_rotation_b_w;
	float target_angle;
	float hertz;
	float damping_ratio;
	float lower_angle;
	float upper_angle;
	float max_motor_torque;
	float motor_speed;
	uint32_t reserved0;
} v3_revolute_joint_command;

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
	uint32_t kind;
	uint32_t flags;
	uint64_t category_bits;
	uint64_t mask_bits;
	uint32_t point_count;
	float radius;
	float point_x[V3_QUERY_MAX_POINTS];
	float point_y[V3_QUERY_MAX_POINTS];
	float point_z[V3_QUERY_MAX_POINTS];
} v3_query;

typedef struct v3_query_result
{
	uint64_t query_id;
	uint32_t status;
	uint32_t reserved;
	uint64_t hit_logical_id;
	float fraction;
	float reserved2;
	double point_x;
	double point_y;
	double point_z;
	float normal_x;
	float normal_y;
	float normal_z;
	uint32_t flags;
	int32_t cell_x;
	int32_t cell_y;
	int32_t cell_z;
	int32_t cell_box_index;
	uint32_t material_index;
	uint32_t reserved3;
	uint64_t user_material_id;
	uint64_t user_data;
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
	// BlockGrid pair counters, appended when the pair pass landed. Box3D rebuilds them from zero on
	// every fixed step, so each count below is the sum over the fixed steps this call ran and the
	// scratch figure is the maximum over those steps.
	// Sum. Candidate hitbox pairs the pair traversal enumerated.
	uint64_t block_grid_candidate_hitbox_pair_count;
	// Sum. Candidate hitbox pairs that produced a manifold, which is not the number of contacts.
	uint64_t block_grid_touching_pair_count;
	// Sum. BlockGrid pair contacts the pass selected.
	uint64_t block_grid_contact_count;
	// Sum. Reserved until the Projectile sweep lands with world configuration; reads zero.
	uint64_t block_grid_projectile_sweep_count;
	// Sum. Reserved until the Projectile candidate cap lands with world configuration; reads zero.
	uint64_t block_grid_cap_exhaustion_count;
	// Sum. BlockGrid revisions published since the previous step.
	uint64_t block_grid_replacement_published_count;
	// Peak, not a sum. Largest scratch byte count the pair pass took from the step arena in any one
	// fixed step of this call.
	uint64_t block_grid_scratch_peak_bytes;
	// Sum. Pair updates that reduced a fragmented contact to the bounded support subset.
	uint64_t block_grid_contact_reduction_count;
	// Number of BlockGrid contact begin, hit, and end events retained for this call, in fixed-step
	// order. The event records are copied with v3_world_get_block_contact_events.
	uint32_t block_contact_event_count;
	// Saturating count of native or bridge events that could not be retained for this call.
	uint32_t block_contact_event_dropped_count;
	// V3_BLOCK_CONTACT_EVENTS_TRUNCATED is set when the event batch is incomplete.
	uint32_t block_contact_event_flags;
} v3_step_stats;

// v3_body_definition, v3_block_material, v3_block_cell and v3_block_box carry the BlockGrid cook and
// attach payload. Every record below them still reserves an approved schema layout and has no
// operation of its own.

/// Pose, velocity and damping for v3_world_attach_block_grid and v3_world_create_sphere_body.
/// handle supplies the logical identity and generation used by removals, wrenches, kinematic
/// targets and queries after creation.
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

// One BlockGrid surface material. friction, restitution and material_id are used: material_id is
// carried through unchanged as the Box3D user material identifier, which is also 64 bits wide, so
// nothing is truncated. Box3D takes density per shape rather than per material, so the cook uses
// material zero's density as the density of every shape the cooked data backs when that density is
// positive and 1.0 otherwise; the density of every other material is ignored. bond_strength,
// compressive_factor and flags are accepted and ignored, reserved for fracture.
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

// One occupied cell. x, y and z are integer cell coordinates in BlockGrid-local space, so the cell
// spans [x, x + 1] on that axis. feature_id becomes the block user data and does not have to be
// unique. material_index selects the cell material and must be below material_count. flags and
// reserved must be zero.
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

// One collision box inside the cell that owner_cell_index names, which is an index into the cell
// array of the same cook. Boxes may arrive in any order; the cook groups them by owner. The center
// and the half extents are cell-local: the box covers center minus half extent to center plus half
// extent, and both bounds must stay inside the unit cube [0, 1] of the owning cell, so a full cube
// is a center of (0.5, 0.5, 0.5) with half extents of (0.5, 0.5, 0.5) and the cook places it at the
// owning cell coordinate. material_index selects the box material. feature_id is accepted and
// ignored, reserved for per-box fracture identity; a cell reports its own feature_id.
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

typedef struct v3_block_grid_command
{
	v3_body_definition body;
	uint64_t source_revision;
	uint32_t material_offset;
	uint32_t material_count;
	uint32_t cell_offset;
	uint32_t cell_count;
	uint32_t box_offset;
	uint32_t box_count;
} v3_block_grid_command;

typedef struct v3_block_contact_side
{
	// Logical cell coordinates and the original Cell Box index supplied by the caller.
	// All identity fields are zero unless V3_BLOCK_CONTACT_SIDE_GRID is set.
	int32_t cell_x;
	int32_t cell_y;
	int32_t cell_z;
	int32_t cell_box_index;
	uint32_t material_index;
	uint32_t flags;
	uint64_t user_material_id;
	uint64_t user_data;
} v3_block_contact_side;

typedef struct v3_block_contact_event
{
	// Side order follows the native contact. The normal points from A to B.
	v3_body_handle body_a;
	v3_body_handle body_b;
	// Reserved fracture feature identifiers. They are zero until a later interface defines a source.
	uint64_t feature_id_a;
	uint64_t feature_id_b;
	// Native double precision world positions are converted to this reserved float representation.
	float point_x;
	float point_y;
	float point_z;
	float normal_x;
	float normal_y;
	float normal_z;
	// Native scalar normal impulse multiplied by the A-to-B normal.
	float impulse_x;
	float impulse_y;
	float impulse_z;
	// Positive native approach speed in meters per second.
	float relative_normal_speed;
	// Exactly one V3_BLOCK_CONTACT_BEGIN, END or HIT flag.
	uint32_t flags;
	// Zero-based fixed step index within the successful step-and-read call.
	uint32_t fixed_step_index;
	v3_block_contact_side side_a;
	v3_block_contact_side side_b;
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
// A NULL limits pointer selects the engine defaults for every limit.
V3_API v3_world* v3_world_create( double gravity_x, double gravity_y, double gravity_z, const v3_world_limits* limits );
V3_API v3_status v3_world_set_limits( v3_world* world, const v3_world_limits* limits );
V3_API v3_status v3_world_get_limits( v3_world* world, v3_world_limits* limits );
V3_API v3_status v3_world_replace_box_bodies( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
											  const v3_box_body_command* creations, uint32_t creation_count );
V3_API v3_status v3_world_create_hull_body( v3_world* world, const v3_box_body_command* command, const float* point_xyz,
											uint32_t point_count );
/// Create a body with one sphere centered at its local origin. radius must be finite and positive,
/// density finite and positive for dynamic bodies or zero otherwise, and friction finite in [0, 1].
/// V3_BODY_BULLET is valid only for dynamic bodies. body is read only during this call. On success, *out receives body->handle
/// and the world owns the body until removal or world destruction. Failure leaves *out unchanged and creates no body. Removed
/// handles become stale and reuse requires a new generation.
V3_API v3_status v3_world_create_sphere_body( v3_world* world, const v3_body_definition* body, float radius, float density,
											  float friction, v3_body_handle* out );
// Cook immutable BlockGrid geometry without touching a world. On success *out owns one reference to
// the cooked data and the caller destroys it with v3_destroy_cooked_grid; the handle is independent
// of any world and the same handle may back shapes in several worlds. Every array is read during
// the call only, and every count must be greater than zero. cells and boxes are matched by
// owner_cell_index, which indexes cells and must be in range; boxes may arrive in any order. Any
// failure writes NULL through out and cooks nothing: V3_INVALID_ARGUMENT for a missing pointer, a
// zero count or a reserved field that is not zero, V3_INVALID_MATERIAL for an unusable material or
// a material index out of range, V3_INVALID_OWNER for an owner index out of range, V3_NON_FINITE
// for a value that is not finite, V3_INVALID_CELL for a cell with no box or a coordinate the cook
// cannot represent, V3_DUPLICATE_CELL for two cells at one coordinate, V3_INVALID_DIMENSION for a
// box outside its own unit cell, V3_LIMIT_EXCEEDED past a cook limit and V3_OUT_OF_MEMORY when the
// allocator cannot satisfy it.
V3_API v3_status v3_cook_block_grid( const v3_block_material* materials, uint32_t material_count, const v3_block_cell* cells,
									 uint32_t cell_count, const v3_block_box* boxes, uint32_t box_count, v3_cooked_grid** out );
// Create a static, kinematic or dynamic body from body and attach cooked geometry to it. The shape
// takes its own reference on the cooked data, so the caller may destroy grid as soon as this
// returns. A non-NULL mass_override installs explicit mass data on the new body and must be finite
// with a positive mass and a positive definite inertia tensor; a NULL override leaves the mass Box3D
// derives from the geometry. The new body handle is written through out and is addressable by every
// other body operation.
V3_API v3_status v3_world_attach_block_grid( v3_world* world, const v3_body_definition* body, v3_cooked_grid* grid,
											 const v3_mass_properties* mass_override, v3_body_handle* out );
/// Replace the geometry of an active BlockGrid body atomically between steps. The world and grid
/// must be live handles. Success retains grid independently of the caller and preserves logical,
/// body and shape identities, pose, filters and event flags. Derived mass uses the existing shape
/// density; explicit mass overrides and their cached center remain unchanged. Angular velocity
/// is preserved. A derived center shift adjusts linear velocity by angular velocity cross the
/// world center displacement, following native mass recomputation.
/// Contacts are rebuilt: retained cells may emit END then BEGIN while support is restored.
/// Success wakes the body and advances the mutation and native replacement counters once.
/// Failure preserves geometry and observable body state. Returns V3_INVALID_ARGUMENT for NULL
/// pointers or malformed handles, V3_STALE_HANDLE for an inactive or mismatched generation,
/// V3_INVALID_BLOCK_GRID for the wrong shape type or missing cooked data, V3_OUT_OF_MEMORY for
/// reservation failure, and V3_NATIVE_FAILURE if the native world is locked.
V3_API v3_status v3_world_replace_block_grid( v3_world* world, const v3_body_handle* body, v3_cooked_grid* grid );
// Release the handle's one reference to its cooked data. NULL is accepted. Cooked bytes survive
// until the last reference, from any shape or any world, is gone.
V3_API void v3_destroy_cooked_grid( v3_cooked_grid* grid );
V3_API v3_status v3_world_replace_distance_joints( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
												   const v3_distance_joint_command* creations, uint32_t creation_count );
/// Atomically remove joints of either kind and create revolute joints between steps.
/// Arrays may be NULL only when their count is zero. Each count and the active and temporary
/// joint totals are limited to 4096, as is the number of logical joint IDs ever admitted.
/// New IDs start at generation 1; reuse advances the previous generation by exactly one,
/// including changes between distance and revolute. Active IDs must be removed in this batch.
/// Complete validation precedes mutation. Failed validation or native creation preserves
/// accepted joints, body relationships and mutation counters. Success prevents removal of
/// either attached body until its joints are removed. World ownership and threading rules
/// are the same as for distance replacement.
/// Returns V3_INVALID_ARGUMENT for malformed pointers, IDs, flags, reserved fields, negative
/// torque, static or identical participants, or invalid angle bounds; V3_NON_FINITE for
/// nonfinite anchors or settings; V3_INVALID_QUATERNION for invalid local rotations;
/// V3_INVALID_DAMPING for negative hertz or damping; V3_STALE_HANDLE for stale handles;
/// V3_DUPLICATE_ID, V3_INVALID_GENERATION or V3_GENERATION_EXHAUSTED for identity violations;
/// V3_LIMIT_EXCEEDED or V3_PEAK_LIMIT_EXCEEDED for capacity violations; V3_OUT_OF_MEMORY
/// for reservation failure; and V3_NATIVE_FAILURE when native creation returns no joint.
V3_API v3_status v3_world_replace_revolute_joints( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
												   const v3_revolute_joint_command* creations, uint32_t creation_count );
V3_API v3_status v3_world_step_and_read( v3_world* world, const v3_kinematic_target* targets, uint32_t target_count,
										 const v3_body_wrench* wrenches, uint32_t wrench_count, const v3_query* queries,
										 uint32_t query_count, uint32_t fixed_step_count, v3_transform* transforms,
										 uint32_t transform_capacity, v3_query_result* query_results,
										 uint32_t query_result_capacity, v3_step_stats* stats );
// Copy the completed call's BlockGrid event batch without advancing simulation. Events are ordered
// by fixed step and then END, BEGIN and HIT, preserving native order within each kind. Use
// block_contact_event_count from the last successful step stats as the required capacity. event_count
// receives the required size on success or V3_OUTPUT_TOO_SMALL, so an undersized destination may be
// retried unchanged.
V3_API v3_status v3_world_get_block_contact_events( const v3_world* world, v3_block_contact_event* events,
													uint32_t event_capacity, uint32_t* event_count );
V3_API void v3_world_destroy( v3_world* world );
V3_API uint32_t v3_active_world_count( void );

#ifdef __cplusplus
}
#endif

#endif

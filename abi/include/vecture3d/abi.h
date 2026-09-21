// SPDX-License-Identifier: MIT

#ifndef V3_ABI_H
#define V3_ABI_H

// clang-format off
#include <stddef.h>
#include <stdint.h>
#include "abi_manifest.h"
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

// Speed limits do not guarantee collision safety. Speculative admission predicts free motion,
// which later contact or joint impulses can change. Rotational paths are not certified.
// Test speeds and timestep against the game's geometry.
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

/// Revolute joint between distinct dynamic or kinematic bodies. IDs are shared with distance joints.
/// All floats must be finite. Anchors use meters from the body origin. Frame rotations use xyzw
/// quaternions with |squared norm - 1| < 20*FLT_EPSILON. Each frame's local z-axis is the hinge axis.
/// The signed angle measures frame B relative to A in radians using the right hand rule.
typedef struct v3_revolute_joint_command
{
	uint64_t logical_id;
	// [1, INT32_MAX].
	uint32_t generation;
	// V3_JOINT_* flags. Unset features, including connected collision, are disabled.
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
	// Spring target in [-pi, pi], independent of the limits.
	float target_angle;
	// Nonnegative spring frequency and damping.
	float hertz;
	float damping_ratio;
	// Ordered bounds within [-0.99*pi, 0.99*pi], even when limits are disabled.
	float lower_angle;
	float upper_angle;
	// Nonnegative, in newton meters.
	float max_motor_torque;
	// Motor target in radians per second.
	float motor_speed;
	// Must be zero.
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
	// BlockGrid counters are reset each fixed step. This call sums the counts across its fixed
	// steps and reports the maximum scratch use across those steps.
	// Sum. Candidate hitbox pairs the pair traversal enumerated.
	uint64_t block_grid_candidate_hitbox_pair_count;
	// Sum. Candidate hitbox pairs that produced a manifold, which is not the number of contacts.
	uint64_t block_grid_touching_pair_count;
	// Sum. BlockGrid pair contacts the pass selected.
	uint64_t block_grid_contact_count;
	// Sum. Fast convex shape sweeps against BlockGrid geometry, including bodies without the bullet flag.
	uint64_t block_grid_projectile_sweep_count;
	// Sum. Sweeps against BlockGrid geometry that exhausted the candidate cap.
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

/// Body creation parameters. handle supplies the new body's logical ID and generation.
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

// One BlockGrid surface material. Box3D uses friction, restitution, and the full 64-bit material_id.
// Shape density comes from material zero: its density is used when positive, otherwise 1.0 is used.
// Other materials' densities are ignored. bond_strength, compressive_factor, and flags are reserved
// for fracture and currently ignored.
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

// Collision box in owner_cell_index, with cell-local bounds inside [0, 1] on every axis.
// Bounds are center +/- half extent. Boxes may arrive in any order. material_index selects
// the material. feature_id is ignored: reported identity uses the cell's feature_id.
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
	// Absolute world position, preserving native double precision at large world offsets.
	double point_x;
	double point_y;
	double point_z;
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
V3_API v3_status v3_world_get_limits( const v3_world* world, v3_world_limits* limits );
V3_API v3_status v3_world_replace_box_bodies( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
											  const v3_box_body_command* creations, uint32_t creation_count );
V3_API v3_status v3_world_create_hull_body( v3_world* world, const v3_box_body_command* command, const float* point_xyz,
											uint32_t point_count );
/// Create a sphere at the body origin. radius must be positive and friction in [0, 1], both finite.
/// density must be finite and positive for dynamic bodies, zero otherwise. Only dynamic bodies
/// accept V3_BODY_BULLET. body is read during this call. Success writes body->handle to *out.
/// Failure leaves *out unchanged and creates no body. Reusing a removed ID requires a new generation.
V3_API v3_status v3_world_create_sphere_body( v3_world* world, const v3_body_definition* body, float radius, float density,
											  float friction, v3_body_handle* out );
// Cook geometry independently of any world. Arrays are read only during this call. Require finite
// floats, nonzero counts, valid material and owner_cell_index values, unique cells, and a box per cell.
// Cell flags and reserved fields must be zero. On success, *out owns one reference, released with
// v3_destroy_cooked_grid. Failure sets *out to NULL when out is non-NULL and creates no handle.
V3_API v3_status v3_cook_block_grid( const v3_block_material* materials, uint32_t material_count, const v3_block_cell* cells,
									 uint32_t cell_count, const v3_block_box* boxes, uint32_t box_count, v3_cooked_grid** out );
// Create body and attach grid, retaining its geometry. The caller may then destroy grid.
// A NULL mass_override derives mass from geometry. An override must be finite, with positive mass
// and a positive definite inertia tensor. Success writes the new body handle to *out.
V3_API v3_status v3_world_attach_block_grid( v3_world* world, const v3_body_definition* body, const v3_cooked_grid* grid,
											 const v3_mass_properties* mass_override, v3_body_handle* out );
/// Replace geometry between steps using live world, body, and grid handles. Success retains grid
/// and preserves body and shape IDs, pose, filters, event flags, and angular velocity.
/// Recompute derived mass at the existing density. A center shift adds angular velocity crossed
/// with world center displacement to linear velocity. Explicit mass and cached center are unchanged.
/// Rebuild contacts, which may emit END then BEGIN for retained cells. Wake the body and increment
/// mutation and replacement counters once. Failure preserves geometry and observable body state.
V3_API v3_status v3_world_replace_block_grid( v3_world* world, const v3_body_handle* body, const v3_cooked_grid* grid );
// Release the handle's one reference to its cooked data. NULL is accepted. Cooked bytes survive
// until the last reference, from any shape or any world, is gone.
V3_API void v3_destroy_cooked_grid( v3_cooked_grid* grid );
V3_API v3_status v3_world_replace_distance_joints( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
												   const v3_distance_joint_command* creations, uint32_t creation_count );
/// Atomically remove joints of either kind and create revolute joints between steps.
/// Arrays may be NULL only for zero counts. Each count, active/temporary joint total, and lifetime
/// logical ID total is limited to 4096. New IDs use generation 1. Reuse increments it by exactly one,
/// including when switching joint kinds. Active IDs must be removed in this batch before reuse.
/// Validation and creation failures preserve joints, body relationships, and mutation counters.
/// Attached bodies cannot be removed until their joints are removed. Distance-joint threading rules apply.
V3_API v3_status v3_world_replace_revolute_joints( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
												   const v3_revolute_joint_command* creations, uint32_t creation_count );
V3_API v3_status v3_world_step_and_read( v3_world* world, const v3_kinematic_target* targets, uint32_t target_count,
										 const v3_body_wrench* wrenches, uint32_t wrench_count, const v3_query* queries,
										 uint32_t query_count, uint32_t fixed_step_count, v3_transform* transforms,
										 uint32_t transform_capacity, v3_query_result* query_results,
										 uint32_t query_result_capacity, v3_step_stats* stats );
// Copy the last successful step call's events in fixed-step order, then END, BEGIN, HIT order,
// preserving native order within each kind. block_contact_event_count gives the required capacity.
// *event_count receives that size on success or V3_OUTPUT_TOO_SMALL. An undersized copy can be retried
// without changing the batch or advancing simulation.
V3_API v3_status v3_world_get_block_contact_events( const v3_world* world, v3_block_contact_event* events,
													uint32_t event_capacity, uint32_t* event_count );
V3_API void v3_world_destroy( v3_world* world );
V3_API uint32_t v3_active_world_count( void );

#ifdef __cplusplus
}
#endif

#endif

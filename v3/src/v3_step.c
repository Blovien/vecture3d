// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <float.h>
#include <limits.h>
#include <math.h>

typedef struct v3_query_callback_context
{
	const v3_world* world;
	bool hit;
	uint64_t hit_logical_id;
	float fraction;
} v3_query_callback_context;

static int v3_step_find_body_entry( const v3_world* world, uint64_t logical_id )
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

static int v3_step_find_shape_entry( const v3_world* world, b3ShapeId shape_id )
{
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		const v3_body_entry* entry = world->body_entries + index;
		if ( entry->is_active &&
			 ( B3_ID_EQUALS( entry->shape_id, shape_id ) || B3_ID_EQUALS( entry->body_id, b3Shape_GetBody( shape_id ) ) ) )
		{
			return (int)index;
		}
	}
	return -1;
}

static bool v3_step_is_normalized_quaternion( float x, float y, float z, float w )
{
	if ( !isfinite( x ) || !isfinite( y ) || !isfinite( z ) || !isfinite( w ) )
	{
		return false;
	}
	float length_squared = x * x + y * y + z * z + w * w;
	return isfinite( length_squared ) && 1.0f - 20.0f * FLT_EPSILON < length_squared &&
		   length_squared < 1.0f + 20.0f * FLT_EPSILON;
}

static v3_status v3_step_validate_target( const v3_world* world, const v3_kinematic_target* target, uint32_t fixed_step_count )
{
	if ( target->logical_id == 0 || target->generation == 0 || target->generation > INT32_MAX ||
		 ( target->flags & ~V3_TARGET_WAKE ) != 0 || fixed_step_count == 0 )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( !isfinite( target->position_x ) || !isfinite( target->position_y ) || !isfinite( target->position_z ) )
	{
		return V3_NON_FINITE;
	}
	if ( !v3_step_is_normalized_quaternion( target->rotation_x, target->rotation_y, target->rotation_z, target->rotation_w ) )
	{
		return V3_INVALID_QUATERNION;
	}
	int entry_index = v3_step_find_body_entry( world, target->logical_id );
	if ( entry_index < 0 || !world->body_entries[entry_index].is_active ||
		 world->body_entries[entry_index].generation != target->generation )
	{
		return V3_STALE_HANDLE;
	}
	return world->body_entries[entry_index].kind == V3_KINEMATIC_BODY ? V3_OK : V3_INVALID_TARGET;
}

static v3_status v3_step_validate_targets( const v3_world* world, const v3_kinematic_target* targets, uint32_t target_count,
										   uint32_t fixed_step_count )
{
	for ( uint32_t index = 0; index < target_count; ++index )
	{
		v3_status status = v3_step_validate_target( world, targets + index, fixed_step_count );
		if ( status != V3_OK )
		{
			return status;
		}
		for ( uint32_t previous = 0; previous < index; ++previous )
		{
			if ( targets[previous].logical_id == targets[index].logical_id )
			{
				return V3_DUPLICATE_ID;
			}
		}
	}
	return V3_OK;
}

static bool v3_step_vector_within_magnitude( float x, float y, float z, float maximum )
{
	double squared_magnitude = (double)x * x + (double)y * y + (double)z * z;
	return squared_magnitude <= (double)maximum * maximum;
}

static v3_status v3_step_validate_wrench( const v3_world* world, const v3_body_wrench* wrench, uint32_t fixed_step_count )
{
	if ( wrench->logical_id == 0 || wrench->generation == 0 || wrench->generation > INT32_MAX ||
		 ( wrench->flags & ~V3_WRENCH_WAKE ) != 0 || fixed_step_count == 0 )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( !isfinite( wrench->force_x ) || !isfinite( wrench->force_y ) || !isfinite( wrench->force_z ) ||
		 !isfinite( wrench->torque_x ) || !isfinite( wrench->torque_y ) || !isfinite( wrench->torque_z ) )
	{
		return V3_NON_FINITE;
	}
	if ( !v3_step_vector_within_magnitude( wrench->force_x, wrench->force_y, wrench->force_z, V3_MAX_FORCE_MAGNITUDE ) ||
		 !v3_step_vector_within_magnitude( wrench->torque_x, wrench->torque_y, wrench->torque_z, V3_MAX_TORQUE_MAGNITUDE ) )
	{
		return V3_INVALID_WRENCH;
	}
	int entry_index = v3_step_find_body_entry( world, wrench->logical_id );
	if ( entry_index < 0 || !world->body_entries[entry_index].is_active ||
		 world->body_entries[entry_index].generation != wrench->generation )
	{
		return V3_STALE_HANDLE;
	}
	return world->body_entries[entry_index].kind == V3_DYNAMIC_BODY ? V3_OK : V3_INVALID_WRENCH;
}

static v3_status v3_step_validate_wrenches( const v3_world* world, const v3_body_wrench* wrenches, uint32_t wrench_count,
											uint32_t fixed_step_count )
{
	for ( uint32_t index = 0; index < wrench_count; ++index )
	{
		v3_status status = v3_step_validate_wrench( world, wrenches + index, fixed_step_count );
		if ( status != V3_OK )
		{
			return status;
		}
		for ( uint32_t previous = 0; previous < index; ++previous )
		{
			if ( wrenches[previous].logical_id == wrenches[index].logical_id )
			{
				return V3_DUPLICATE_ID;
			}
		}
	}
	return V3_OK;
}

static v3_status v3_step_validate_query( const v3_query* query )
{
	if ( query->query_id == 0 || query->reserved != 0 )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( !isfinite( query->origin_x ) || !isfinite( query->origin_y ) || !isfinite( query->origin_z ) ||
		 !isfinite( query->half_extent_x ) || !isfinite( query->half_extent_y ) || !isfinite( query->half_extent_z ) ||
		 !isfinite( query->translation_x ) || !isfinite( query->translation_y ) || !isfinite( query->translation_z ) )
	{
		return V3_NON_FINITE;
	}
	return query->half_extent_x > 0.0f && query->half_extent_y > 0.0f && query->half_extent_z > 0.0f ? V3_OK
																									 : V3_INVALID_DIMENSION;
}

static v3_status v3_step_validate_queries( const v3_query* queries, uint32_t query_count )
{
	for ( uint32_t index = 0; index < query_count; ++index )
	{
		v3_status status = v3_step_validate_query( queries + index );
		if ( status != V3_OK )
		{
			return status;
		}
		for ( uint32_t previous = 0; previous < index; ++previous )
		{
			if ( queries[previous].query_id == queries[index].query_id )
			{
				return V3_DUPLICATE_ID;
			}
		}
	}
	return V3_OK;
}

static bool v3_step_overlap_callback( b3ShapeId shape_id, void* context )
{
	v3_query_callback_context* callback = context;
	int entry_index = v3_step_find_shape_entry( callback->world, shape_id );
	if ( entry_index >= 0 && callback->world->body_entries[entry_index].kind == V3_STATIC_BODY )
	{
		uint64_t logical_id = callback->world->body_entries[entry_index].logical_id;
		if ( !callback->hit || logical_id < callback->hit_logical_id )
		{
			callback->hit = true;
			callback->hit_logical_id = logical_id;
		}
		callback->fraction = 0.0f;
	}
	return true;
}

static float v3_step_cast_callback( b3ShapeId shape_id, b3Pos point, b3Vec3 normal, float fraction, uint64_t user_material_id,
									int triangle_index, int child_index, void* context )
{
	(void)point;
	(void)normal;
	(void)user_material_id;
	(void)triangle_index;
	(void)child_index;
	v3_query_callback_context* callback = context;
	int entry_index = v3_step_find_shape_entry( callback->world, shape_id );
	if ( entry_index >= 0 && callback->world->body_entries[entry_index].kind == V3_STATIC_BODY )
	{
		uint64_t logical_id = callback->world->body_entries[entry_index].logical_id;
		if ( !callback->hit || fraction < callback->fraction ||
			 ( fraction == callback->fraction && logical_id < callback->hit_logical_id ) )
		{
			callback->hit = true;
			callback->hit_logical_id = logical_id;
			callback->fraction = fraction;
		}
	}
	return 1.0f;
}

static v3_query_result v3_step_run_query( const v3_world* world, const v3_query* query )
{
	b3BoxHull box = b3MakeBoxHull( query->half_extent_x, query->half_extent_y, query->half_extent_z );
	b3ShapeProxy proxy = { .points = box.boxPoints, .count = 8, .radius = 0.0f };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.maskBits = V3_STATIC_CATEGORY;
	filter.categoryBits = V3_STATIC_CATEGORY;
	v3_query_callback_context callback = {
		.world = world,
		.fraction = 1.0f,
	};
	b3Pos origin = { query->origin_x, query->origin_y, query->origin_z };
	b3World_OverlapShape( world->world_id, origin, &proxy, filter, v3_step_overlap_callback, &callback );
	if ( callback.hit )
	{
		return (v3_query_result){
			.query_id = query->query_id,
			.status = V3_QUERY_IMMEDIATE_BLOCK,
			.hit_logical_id = callback.hit_logical_id,
			.fraction = 0.0f,
		};
	}

	b3Vec3 translation = { query->translation_x, query->translation_y, query->translation_z };
	b3World_CastShape( world->world_id, origin, &proxy, translation, filter, v3_step_cast_callback, &callback );
	if ( callback.hit && callback.fraction <= 0.0f )
	{
		return (v3_query_result){
			.query_id = query->query_id,
			.status = V3_QUERY_IMMEDIATE_BLOCK,
			.hit_logical_id = callback.hit_logical_id,
			.fraction = 0.0f,
		};
	}
	return (v3_query_result){
		.query_id = query->query_id,
		.status = callback.hit ? V3_QUERY_CAST_HIT : V3_QUERY_NO_HIT,
		.hit_logical_id = callback.hit ? callback.hit_logical_id : 0,
		.fraction = callback.hit ? callback.fraction : 1.0f,
	};
}

static void v3_step_sort_query_results( v3_query_result* results, uint32_t count )
{
	for ( uint32_t index = 1; index < count; ++index )
	{
		v3_query_result value = results[index];
		uint32_t previous = index;
		while ( previous > 0 && results[previous - 1].query_id > value.query_id )
		{
			results[previous] = results[previous - 1];
			previous -= 1;
		}
		results[previous] = value;
	}
}

v3_status v3_world_step_and_read_internal( v3_world* world, const v3_kinematic_target* targets, uint32_t target_count,
										   const v3_body_wrench* wrenches, uint32_t wrench_count, const v3_query* queries,
										   uint32_t query_count, uint32_t fixed_step_count, v3_transform* transforms,
										   uint32_t transform_capacity, v3_query_result* query_results,
										   uint32_t query_result_capacity, v3_step_stats* stats )
{
	v3_status status = v3_step_validate_targets( world, targets, target_count, fixed_step_count );
	if ( status != V3_OK )
	{
		return status;
	}
	status = v3_step_validate_wrenches( world, wrenches, wrench_count, fixed_step_count );
	if ( status != V3_OK )
	{
		return status;
	}
	status = v3_step_validate_queries( queries, query_count );
	if ( status != V3_OK )
	{
		return status;
	}

	uint32_t movable_count = 0;
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		const v3_body_entry* entry = world->body_entries + index;
		if ( entry->is_active && entry->kind != V3_STATIC_BODY )
		{
			movable_count += 1;
		}
	}
	if ( movable_count > transform_capacity || ( movable_count > 0 && transforms == NULL ) ||
		 query_count > query_result_capacity || ( query_count > 0 && query_results == NULL ) )
	{
		return V3_OUTPUT_TOO_SMALL;
	}

	float target_duration = (float)fixed_step_count * V3_FIXED_TIME_STEP;
	for ( uint32_t index = 0; index < target_count; ++index )
	{
		const v3_kinematic_target* target = targets + index;
		int entry_index = v3_step_find_body_entry( world, target->logical_id );
		b3WorldTransform transform = {
			.p = { target->position_x, target->position_y, target->position_z },
			.q = { { target->rotation_x, target->rotation_y, target->rotation_z }, target->rotation_w },
		};
		b3Body_SetTargetTransform( world->body_entries[entry_index].body_id, transform, target_duration,
								   ( target->flags & V3_TARGET_WAKE ) != 0 );
	}

	for ( uint32_t step_index = 0; step_index < fixed_step_count; ++step_index )
	{
		for ( uint32_t wrench_index = 0; wrench_index < wrench_count; ++wrench_index )
		{
			const v3_body_wrench* wrench = wrenches + wrench_index;
			int entry_index = v3_step_find_body_entry( world, wrench->logical_id );
			b3BodyId body_id = world->body_entries[entry_index].body_id;
			bool wake = ( wrench->flags & V3_WRENCH_WAKE ) != 0;
			b3Body_ApplyForceToCenter( body_id, (b3Vec3){ wrench->force_x, wrench->force_y, wrench->force_z }, wake );
			b3Body_ApplyTorque( body_id, (b3Vec3){ wrench->torque_x, wrench->torque_y, wrench->torque_z }, wake );
		}
		b3World_Step( world->world_id, V3_FIXED_TIME_STEP, V3_SUB_STEP_COUNT );
	}

	uint32_t output_index = 0;
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		const v3_body_entry* body = world->body_entries + index;
		if ( !body->is_active || body->kind == V3_STATIC_BODY )
		{
			continue;
		}
		b3WorldTransform transform = b3Body_GetTransform( body->body_id );
		b3Vec3 linear_velocity = b3Body_GetLinearVelocity( body->body_id );
		b3Vec3 angular_velocity = b3Body_GetAngularVelocity( body->body_id );
		b3MassData mass_data = body->has_explicit_mass_data ? body->mass_data : b3Body_GetMassData( body->body_id );
		transforms[output_index++] = (v3_transform){
			.logical_id = body->logical_id,
			.flags = b3Body_IsAwake( body->body_id ) ? V3_TRANSFORM_AWAKE : 0u,
			.generation = body->generation,
			.position_x = transform.p.x,
			.position_y = transform.p.y,
			.position_z = transform.p.z,
			.rotation_x = transform.q.v.x,
			.rotation_y = transform.q.v.y,
			.rotation_z = transform.q.v.z,
			.rotation_w = transform.q.s,
			.linear_velocity_x = linear_velocity.x,
			.linear_velocity_y = linear_velocity.y,
			.linear_velocity_z = linear_velocity.z,
			.angular_velocity_x = angular_velocity.x,
			.angular_velocity_y = angular_velocity.y,
			.angular_velocity_z = angular_velocity.z,
			.local_center_x = mass_data.center.x,
			.local_center_y = mass_data.center.y,
			.local_center_z = mass_data.center.z,
		};
	}

	for ( uint32_t index = 0; index < query_count; ++index )
	{
		query_results[index] = v3_step_run_query( world, queries + index );
	}
	v3_step_sort_query_results( query_results, query_count );
	b3Counters counters = b3World_GetCounters( world->world_id );
	b3Profile profile = b3World_GetProfile( world->world_id );
	*stats = (v3_step_stats){
		.output_count = output_index,
		.body_count = (uint32_t)counters.bodyCount,
		.shape_count = (uint32_t)counters.shapeCount,
		.contact_count = (uint32_t)counters.contactCount,
		.step_milliseconds = profile.step,
		.mutation_batch_count = world->mutation_batch_count,
		.created_body_count = world->created_body_count,
		.destroyed_body_count = world->destroyed_body_count,
		.query_output_count = query_count,
		.query_count = query_count,
		.fixed_step_count = fixed_step_count,
		.joint_count = (uint32_t)counters.jointCount,
		.pair_milliseconds = profile.pairs,
		.collide_milliseconds = profile.collide,
		.solve_milliseconds = profile.solve,
		.static_tree_height = (uint32_t)counters.staticTreeHeight,
		.dynamic_tree_height = (uint32_t)counters.treeHeight,
		.sat_call_count = (uint32_t)counters.satCallCount,
		.sat_cache_hit_count = (uint32_t)counters.satCacheHitCount,
		.graph_overflow_constraint_count = (uint32_t)counters.colorCounts[V3_GRAPH_OVERFLOW_INDEX],
		.heap_move_pair_count = (uint32_t)counters.heapMovePairCount,
	};
	return V3_OK;
}

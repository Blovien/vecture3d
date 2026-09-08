// SPDX-License-Identifier: MIT

#include "v3_internal.h"
#include "vecture3d/block_grid.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#if defined( V3_TESTING )
static struct
{
	b3Profile profiles[V3_MAX_FIXED_STEPS];
	uint32_t count;
	uint32_t index;
} v3_test_profile_source;

void v3_test_set_step_profiles( const b3Profile* profiles, uint32_t count )
{
	memset( &v3_test_profile_source, 0, sizeof( v3_test_profile_source ) );
	if ( profiles != NULL && count > 0 && count <= V3_MAX_FIXED_STEPS )
	{
		memcpy( v3_test_profile_source.profiles, profiles, count * sizeof( *profiles ) );
		v3_test_profile_source.count = count;
	}
}
#endif

typedef struct v3_query_callback_context
{
	const v3_world* world;
	bool hit;
	uint64_t hit_logical_id;
	float fraction;
} v3_query_callback_context;

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

static b3Profile v3_step_get_profile( b3WorldId world_id )
{
#if defined( V3_TESTING )
	if ( v3_test_profile_source.index < v3_test_profile_source.count )
	{
		return v3_test_profile_source.profiles[v3_test_profile_source.index++];
	}
#endif
	return b3World_GetProfile( world_id );
}

static bool v3_step_find_body_identity( const v3_world* world, b3BodyId body_id, v3_body_identity* result )
{
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		const v3_body_entry* entry = world->body_entries + index;
		if ( entry->is_active && B3_ID_EQUALS( entry->body_id, body_id ) )
		{
			*result = (v3_body_identity){
				.body_id = entry->body_id,
				.logical_id = entry->logical_id,
				.generation = entry->generation,
			};
			return true;
		}
	}

	const v3_block_contact_storage* storage = world->block_contacts;
	for ( uint32_t index = 0; index < storage->previous_body_identity_count; ++index )
	{
		const v3_body_identity* identity = storage->previous_body_identities + index;
		if ( B3_ID_EQUALS( identity->body_id, body_id ) )
		{
			*result = *identity;
			return true;
		}
	}
	return false;
}

static v3_block_contact_side v3_step_translate_contact_side( const v3BlockContactSide* source )
{
	if ( !source->isBlockGrid )
	{
		return (v3_block_contact_side){ 0 };
	}
	return (v3_block_contact_side){
		.cell_x = source->cellX,
		.cell_y = source->cellY,
		.cell_z = source->cellZ,
		.cell_box_index = source->subHitboxIndex,
		.material_index = source->materialIndex,
		.flags = V3_BLOCK_CONTACT_SIDE_GRID,
		.user_material_id = source->userMaterialId,
		.user_data = source->userData,
	};
}

static bool v3_step_translate_contact_event( const v3_world* world, const v3BlockContactEvent* source, uint32_t kind,
											 uint32_t fixed_step_index, v3_block_contact_event* event )
{
	v3_body_identity body_a;
	if ( !v3_step_find_body_identity( world, source->sideA.bodyId, &body_a ) )
	{
		return false;
	}
	v3_body_identity body_b;
	if ( !v3_step_find_body_identity( world, source->sideB.bodyId, &body_b ) )
	{
		return false;
	}

	*event = (v3_block_contact_event){
		.body_a = { .logical_id = body_a.logical_id, .generation = body_a.generation },
		.body_b = { .logical_id = body_b.logical_id, .generation = body_b.generation },
		.point_x = (float)source->point.x,
		.point_y = (float)source->point.y,
		.point_z = (float)source->point.z,
		.normal_x = source->normal.x,
		.normal_y = source->normal.y,
		.normal_z = source->normal.z,
		.impulse_x = source->normalImpulse * source->normal.x,
		.impulse_y = source->normalImpulse * source->normal.y,
		.impulse_z = source->normalImpulse * source->normal.z,
		.relative_normal_speed = source->approachSpeed,
		.flags = kind,
		.fixed_step_index = fixed_step_index,
		.side_a = v3_step_translate_contact_side( &source->sideA ),
		.side_b = v3_step_translate_contact_side( &source->sideB ),
	};
	return true;
}

static void v3_step_add_dropped_events( v3_block_contact_storage* storage, uint32_t count )
{
	if ( count > 0 )
	{
		storage->dropped_count = v3_geometry_saturating_add_internal( storage->dropped_count, count );
		storage->flags |= V3_BLOCK_CONTACT_EVENTS_TRUNCATED;
	}
}

static void v3_step_append_contact_events( const v3_world* world, const v3BlockContactEvent* sources, int count, uint32_t kind,
										   uint32_t fixed_step_index )
{
	v3_block_contact_storage* storage = world->block_contacts;
	for ( int index = 0; index < count; ++index )
	{
		if ( storage->event_count == V3_MAX_BLOCK_CONTACT_EVENTS )
		{
			v3_step_add_dropped_events( storage, 1 );
			continue;
		}
		v3_block_contact_event event;
		if ( !v3_step_translate_contact_event( world, sources + index, kind, fixed_step_index, &event ) )
		{
			v3_step_add_dropped_events( storage, 1 );
			continue;
		}
		storage->events[storage->event_count++] = event;
	}
}

static void v3_step_collect_contact_events( const v3_world* world, uint32_t fixed_step_index )
{
	v3_block_contact_storage* storage = world->block_contacts;
	v3BlockContactEvents events = v3World_GetBlockContactEvents( world->world_id );
	if ( events.truncated )
	{
		storage->flags |= V3_BLOCK_CONTACT_EVENTS_TRUNCATED;
	}
	v3_step_add_dropped_events( storage, events.droppedEndCount );
	v3_step_add_dropped_events( storage, events.droppedBeginCount );
	v3_step_add_dropped_events( storage, events.droppedHitCount );
	v3_step_append_contact_events( world, events.endEvents, events.endCount, V3_BLOCK_CONTACT_END, fixed_step_index );
	v3_step_append_contact_events( world, events.beginEvents, events.beginCount, V3_BLOCK_CONTACT_BEGIN, fixed_step_index );
	v3_step_append_contact_events( world, events.hitEvents, events.hitCount, V3_BLOCK_CONTACT_HIT, fixed_step_index );
}

static void v3_step_refresh_body_identities( v3_world* world )
{
	v3_block_contact_storage* storage = world->block_contacts;
	storage->previous_body_identity_count = 0;
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		const v3_body_entry* entry = world->body_entries + index;
		if ( entry->is_active )
		{
			storage->previous_body_identities[storage->previous_body_identity_count++] = (v3_body_identity){
				.body_id = entry->body_id,
				.logical_id = entry->logical_id,
				.generation = entry->generation,
			};
		}
	}
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
	if ( !v3_geometry_is_normalized_quaternion_internal( target->rotation_x, target->rotation_y, target->rotation_z,
														 target->rotation_w ) )
	{
		return V3_INVALID_QUATERNION;
	}
	int entry_index = v3_geometry_find_body_entry_internal( world, target->logical_id );
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
	int entry_index = v3_geometry_find_body_entry_internal( world, wrench->logical_id );
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
	if ( query->query_id == 0 || query->reserved != 0 || query->flags != 0 || query->kind > V3_QUERY_HULL )
	{
		return V3_INVALID_ARGUMENT;
	}
	if ( !isfinite( query->origin_x ) || !isfinite( query->origin_y ) || !isfinite( query->origin_z ) ||
		 !isfinite( query->half_extent_x ) || !isfinite( query->half_extent_y ) || !isfinite( query->half_extent_z ) ||
		 !isfinite( query->translation_x ) || !isfinite( query->translation_y ) || !isfinite( query->translation_z ) )
	{
		return V3_NON_FINITE;
	}
	if ( !isfinite( query->radius ) || query->radius < 0.0f || query->point_count > V3_QUERY_MAX_POINTS )
	{
		return V3_INVALID_DIMENSION;
	}
	for ( uint32_t index = 0; index < query->point_count; ++index )
	{
		if ( !isfinite( query->point_x[index] ) || !isfinite( query->point_y[index] ) || !isfinite( query->point_z[index] ) )
		{
			return V3_NON_FINITE;
		}
	}
	if ( query->kind == V3_QUERY_LEGACY_BOX || query->kind == V3_QUERY_BOX )
	{
		if ( query->half_extent_x <= 0.0f || query->half_extent_y <= 0.0f || query->half_extent_z <= 0.0f )
		{
			return V3_INVALID_DIMENSION;
		}
	}
	if ( query->kind == V3_QUERY_RAY )
	{
		return query->point_count == 0 && query->radius == 0.0f ? V3_OK : V3_INVALID_ARGUMENT;
	}
	if ( query->kind == V3_QUERY_SPHERE )
	{
		return query->point_count == 1 && query->radius > 0.0f ? V3_OK : V3_INVALID_DIMENSION;
	}
	if ( query->kind == V3_QUERY_CAPSULE )
	{
		return query->point_count == 2 && query->radius > 0.0f ? V3_OK : V3_INVALID_DIMENSION;
	}
	if ( query->kind == V3_QUERY_BOX )
	{
		return query->point_count == 1 && query->radius == 0.0f ? V3_OK : V3_INVALID_ARGUMENT;
	}
	if ( query->kind == V3_QUERY_HULL )
	{
		return query->point_count >= 4 ? V3_OK : V3_INVALID_DIMENSION;
	}
	return query->point_count == 0 && query->radius == 0.0f ? V3_OK : V3_INVALID_ARGUMENT;
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

static v3_query_result v3_step_run_legacy_query( const v3_world* world, const v3_query* query )
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

static v3_query_result v3_step_make_cast_result( const v3_world* world, const v3_query* query, v3ClosestCastResult cast )
{
	if ( cast.hit == false )
	{
		return (v3_query_result){ .query_id = query->query_id, .status = V3_QUERY_NO_HIT, .fraction = 1.0f };
	}
	int entry_index = v3_step_find_shape_entry( world, cast.shapeId );
	if ( entry_index < 0 )
	{
		return (v3_query_result){ .query_id = query->query_id, .status = V3_QUERY_NO_HIT, .fraction = 1.0f };
	}
	return (v3_query_result){
		.query_id = query->query_id,
		.status = cast.fraction == 0.0f ? V3_QUERY_IMMEDIATE_BLOCK : V3_QUERY_CAST_HIT,
		.hit_logical_id = world->body_entries[entry_index].logical_id,
		.fraction = cast.fraction,
		.point_x = cast.point.x,
		.point_y = cast.point.y,
		.point_z = cast.point.z,
		.normal_x = cast.normal.x,
		.normal_y = cast.normal.y,
		.normal_z = cast.normal.z,
		.flags = cast.isBlockGrid ? V3_QUERY_RESULT_BLOCK_GRID : 0,
		.cell_x = cast.cellX,
		.cell_y = cast.cellY,
		.cell_z = cast.cellZ,
		.cell_box_index = cast.cellBoxIndex,
		.material_index = cast.materialIndex,
		.user_material_id = cast.userMaterialId,
		.user_data = cast.userData,
	};
}

static v3_query_result v3_step_run_query( const v3_world* world, const v3_query* query )
{
	if ( query->kind == V3_QUERY_LEGACY_BOX )
	{
		return v3_step_run_legacy_query( world, query );
	}

	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.categoryBits = query->category_bits == 0 && query->mask_bits == 0 ? V3_ALL_BODY_CATEGORIES : query->category_bits;
	filter.maskBits = query->category_bits == 0 && query->mask_bits == 0 ? V3_ALL_BODY_CATEGORIES : query->mask_bits;
	b3Pos origin = { query->origin_x, query->origin_y, query->origin_z };
	b3Vec3 translation = { query->translation_x, query->translation_y, query->translation_z };
	v3ClosestCastResult cast = { 0 };
	if ( query->kind == V3_QUERY_RAY )
	{
		cast = v3World_CastRayClosest( world->world_id, origin, translation, filter );
	}
	else if ( query->kind == V3_QUERY_SPHERE )
	{
		b3Sphere sphere = { .center = { query->point_x[0], query->point_y[0], query->point_z[0] }, .radius = query->radius };
		cast = v3World_CastSphereClosest( world->world_id, origin, &sphere, translation, filter );
	}
	else if ( query->kind == V3_QUERY_CAPSULE )
	{
		b3Capsule capsule = {
			.center1 = { query->point_x[0], query->point_y[0], query->point_z[0] },
			.center2 = { query->point_x[1], query->point_y[1], query->point_z[1] },
			.radius = query->radius,
		};
		cast = v3World_CastCapsuleClosest( world->world_id, origin, &capsule, translation, filter );
	}
	else if ( query->kind == V3_QUERY_BOX )
	{
		b3Vec3 center = { query->point_x[0], query->point_y[0], query->point_z[0] };
		b3Vec3 half_extent = { query->half_extent_x, query->half_extent_y, query->half_extent_z };
		cast = v3World_CastBoxClosest( world->world_id, origin, center, half_extent, translation, filter );
	}
	else
	{
		b3Vec3 points[V3_QUERY_MAX_POINTS];
		for ( uint32_t index = 0; index < query->point_count; ++index )
		{
			points[index] = (b3Vec3){ query->point_x[index], query->point_y[index], query->point_z[index] };
		}
		b3ShapeProxy proxy = { .points = points, .count = (int)query->point_count, .radius = query->radius };
		cast = v3World_CastShapeClosest( world->world_id, origin, &proxy, translation, filter );
	}
	return v3_step_make_cast_result( world, query, cast );
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

	v3_block_contact_storage* contact_storage = world->block_contacts;
	contact_storage->event_count = 0;
	contact_storage->dropped_count = 0;
	contact_storage->flags = 0;

	float target_duration = (float)fixed_step_count * V3_FIXED_TIME_STEP;
	for ( uint32_t index = 0; index < target_count; ++index )
	{
		const v3_kinematic_target* target = targets + index;
		int entry_index = v3_geometry_find_body_entry_internal( world, target->logical_id );
		b3WorldTransform transform = {
			.p = { target->position_x, target->position_y, target->position_z },
			.q = { { target->rotation_x, target->rotation_y, target->rotation_z }, target->rotation_w },
		};
		b3Body_SetTargetTransform( world->body_entries[entry_index].body_id, transform, target_duration,
								   ( target->flags & V3_TARGET_WAKE ) != 0 );
	}

	// Box3D rebuilds the BlockGrid pair counters from zero on every fixed step, so one call reports
	// the sum over its steps and, for the scratch figure, the largest single step.
	v3BlockGridPairCounters block_grid = { 0 };
	b3Profile profile = { 0 };
	for ( uint32_t step_index = 0; step_index < fixed_step_count; ++step_index )
	{
		for ( uint32_t wrench_index = 0; wrench_index < wrench_count; ++wrench_index )
		{
			const v3_body_wrench* wrench = wrenches + wrench_index;
			int entry_index = v3_geometry_find_body_entry_internal( world, wrench->logical_id );
			b3BodyId body_id = world->body_entries[entry_index].body_id;
			bool wake = ( wrench->flags & V3_WRENCH_WAKE ) != 0;
			b3Body_ApplyForceToCenter( body_id, (b3Vec3){ wrench->force_x, wrench->force_y, wrench->force_z }, wake );
			b3Body_ApplyTorque( body_id, (b3Vec3){ wrench->torque_x, wrench->torque_y, wrench->torque_z }, wake );
		}
		b3World_Step( world->world_id, V3_FIXED_TIME_STEP, V3_SUB_STEP_COUNT );
		b3Profile step_profile = v3_step_get_profile( world->world_id );
		profile.step += step_profile.step;
		profile.pairs += step_profile.pairs;
		profile.collide += step_profile.collide;
		profile.solve += step_profile.solve;
		v3_step_collect_contact_events( world, step_index );
		v3_step_refresh_body_identities( world );

		v3BlockGridPairCounters step_counters = v3World_GetBlockGridPairCounters( world->world_id );
		block_grid.candidateHitboxPairCount += step_counters.candidateHitboxPairCount;
		block_grid.touchingPairCount += step_counters.touchingPairCount;
		block_grid.contactCount += step_counters.contactCount;
		block_grid.projectileSweepCount += step_counters.projectileSweepCount;
		block_grid.capExhaustionCount += step_counters.capExhaustionCount;
		block_grid.replacementPublishedCount += step_counters.replacementPublishedCount;
		block_grid.contactReductionCount += step_counters.contactReductionCount;
		if ( step_counters.scratchPeakBytes > block_grid.scratchPeakBytes )
		{
			block_grid.scratchPeakBytes = step_counters.scratchPeakBytes;
		}
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
		.block_grid_candidate_hitbox_pair_count = block_grid.candidateHitboxPairCount,
		.block_grid_touching_pair_count = block_grid.touchingPairCount,
		.block_grid_contact_count = block_grid.contactCount,
		.block_grid_projectile_sweep_count = block_grid.projectileSweepCount,
		.block_grid_cap_exhaustion_count = block_grid.capExhaustionCount,
		.block_grid_replacement_published_count = block_grid.replacementPublishedCount,
		.block_grid_scratch_peak_bytes = block_grid.scratchPeakBytes,
		.block_grid_contact_reduction_count = block_grid.contactReductionCount,
		.block_contact_event_count = contact_storage->event_count,
		.block_contact_event_dropped_count = contact_storage->dropped_count,
		.block_contact_event_flags = contact_storage->flags,
	};
	return V3_OK;
}

v3_status v3_world_get_block_contact_events_internal( const v3_world* world, v3_block_contact_event* events,
													  uint32_t event_capacity, uint32_t* event_count )
{
	const v3_block_contact_storage* storage = world->block_contacts;
	*event_count = storage->event_count;
	if ( event_capacity < storage->event_count || ( storage->event_count > 0 && events == NULL ) )
	{
		return V3_OUTPUT_TOO_SMALL;
	}
	if ( storage->event_count > 0 )
	{
		memcpy( events, storage->events, storage->event_count * sizeof( *events ) );
	}
	return V3_OK;
}

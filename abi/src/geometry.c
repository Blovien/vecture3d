// SPDX-License-Identifier: MIT

#include "internal.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

static bool v3_geometry_is_finite_positive( float value )
{
	return isfinite( value ) && value > 0.0f;
}

uint32_t v3_geometry_saturating_add_internal( uint32_t value, uint32_t increment )
{
	const uint32_t maximum = (uint32_t)INT32_MAX;
	return value >= maximum || maximum - value < increment ? maximum : value + increment;
}

bool v3_geometry_is_normalized_quaternion_internal( float x, float y, float z, float w )
{
	if ( !isfinite( x ) || !isfinite( y ) || !isfinite( z ) || !isfinite( w ) )
	{
		return false;
	}

	float length_squared = x * x + y * y + z * z + w * w;
	return isfinite( length_squared ) && 1.0f - 20.0f * FLT_EPSILON < length_squared &&
		   length_squared < 1.0f + 20.0f * FLT_EPSILON;
}

static v3_status v3_geometry_validate_body_command( const v3_box_body_command* command )
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

	if ( !v3_body_flags_are_valid( command->kind, command->flags ) )
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

	if ( !v3_geometry_is_normalized_quaternion_internal( command->rotation_x, command->rotation_y, command->rotation_z,
														 command->rotation_w ) )
	{
		return V3_INVALID_QUATERNION;
	}

	if ( !v3_geometry_is_finite_positive( command->half_extent_x ) || !v3_geometry_is_finite_positive( command->half_extent_y ) ||
		 !v3_geometry_is_finite_positive( command->half_extent_z ) )
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

static v3_status v3_geometry_validate_single_creation( const v3_world* world, const v3_box_body_command* command,
													   uint32_t* new_entry_count )
{
	v3_status status = v3_geometry_validate_body_command( command );
	if ( status != V3_OK )
	{
		return status;
	}

	return v3_geometry_validate_body_generation_internal( world, command->logical_id, command->generation, false,
														  new_entry_count );
}

static b3BodyDef v3_geometry_make_body_definition( const v3_box_body_command* command )
{
	b3BodyDef definition = b3DefaultBodyDef();
	definition.type = (b3BodyType)command->kind;
	definition.position = (b3Pos){ command->position_x, command->position_y, command->position_z };
	definition.rotation = (b3Quat){ { command->rotation_x, command->rotation_y, command->rotation_z }, command->rotation_w };
	definition.linearVelocity = (b3Vec3){ command->linear_velocity_x, command->linear_velocity_y, command->linear_velocity_z };
	definition.angularVelocity =
		(b3Vec3){ command->angular_velocity_x, command->angular_velocity_y, command->angular_velocity_z };
	definition.linearDamping = command->linear_damping;
	definition.angularDamping = command->angular_damping;
	definition.enableSleep = ( command->flags & V3_BODY_ENABLE_SLEEP ) != 0;
	definition.isAwake = ( command->flags & V3_BODY_INITIAL_AWAKE ) != 0;
	definition.isBullet = ( command->flags & V3_BODY_BULLET ) != 0;
	return definition;
}

static b3ShapeDef v3_geometry_make_shape_definition( const v3_box_body_command* command )
{
	b3ShapeDef definition = b3DefaultShapeDef();
	definition.density = command->density;
	definition.baseMaterial.friction = command->friction;
	definition.enableContactEvents = true;
	definition.enableHitEvents = true;
	definition.filter.categoryBits = command->kind == V3_STATIC_BODY	  ? V3_STATIC_CATEGORY
									 : command->kind == V3_KINEMATIC_BODY ? V3_KINEMATIC_CATEGORY
																		  : V3_DYNAMIC_CATEGORY;
	definition.filter.maskBits = ( command->flags & V3_BODY_DISABLE_COLLISION ) != 0 ? 0u : V3_ALL_BODY_CATEGORIES;
	return definition;
}

static b3BodyId v3_geometry_create_body( b3WorldId world_id, const b3BodyDef* definition )
{
#if defined( V3_TESTING )
	if ( v3_test_should_fail_internal( V3_TEST_FAULT_CREATE_BODY ) )
	{
		return b3_nullBodyId;
	}
#endif
	return b3CreateBody( world_id, definition );
}

static b3ShapeId v3_geometry_create_sphere_shape( b3BodyId body_id, const b3ShapeDef* definition, const b3Sphere* sphere )
{
#if defined( V3_TESTING )
	if ( v3_test_should_fail_internal( V3_TEST_FAULT_CREATE_SHAPE ) )
	{
		return b3_nullShapeId;
	}
#endif
	return b3CreateSphereShape( body_id, definition, sphere );
}

static v3_box_body_command v3_geometry_make_sphere_command( const v3_body_definition* body, float radius, float density,
															float friction )
{
	return (v3_box_body_command){
		.logical_id = body->handle.logical_id,
		.kind = body->kind,
		.generation = body->handle.generation,
		.position_x = body->position_x,
		.position_y = body->position_y,
		.position_z = body->position_z,
		.rotation_x = body->rotation_x,
		.rotation_y = body->rotation_y,
		.rotation_z = body->rotation_z,
		.rotation_w = body->rotation_w,
		.linear_velocity_x = body->linear_velocity_x,
		.linear_velocity_y = body->linear_velocity_y,
		.linear_velocity_z = body->linear_velocity_z,
		.angular_velocity_x = body->angular_velocity_x,
		.angular_velocity_y = body->angular_velocity_y,
		.angular_velocity_z = body->angular_velocity_z,
		.half_extent_x = radius,
		.half_extent_y = radius,
		.half_extent_z = radius,
		.density = density,
		.friction = friction,
		.linear_damping = body->linear_damping,
		.angular_damping = body->angular_damping,
		.flags = body->flags,
	};
}

static void v3_geometry_publish_body( v3_world* world, const v3_body_entry* created )
{
	v3_geometry_publish_body_internal( world, created );
	world->active_body_count += 1u;
	world->mutation_batch_count = v3_geometry_saturating_add_internal( world->mutation_batch_count, 1 );
	world->created_body_count = v3_geometry_saturating_add_internal( world->created_body_count, 1 );
}

v3_status v3_world_create_hull_body_internal( v3_world* world, const v3_box_body_command* command, const float* point_xyz,
											  uint32_t point_count )
{
	uint32_t new_entry_count = 0;
	v3_status status = v3_geometry_validate_single_creation( world, command, &new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}
	if ( world->active_body_count >= V3_MAX_BODIES_PER_BATCH )
	{
		return V3_LIMIT_EXCEEDED;
	}

	b3Vec3 points[V3_MAX_HULL_POINTS];
	for ( uint32_t index = 0; index < point_count; ++index )
	{
		float x = point_xyz[index * 3u];
		float y = point_xyz[index * 3u + 1u];
		float z = point_xyz[index * 3u + 2u];
		if ( !isfinite( x ) || !isfinite( y ) || !isfinite( z ) )
		{
			return V3_NON_FINITE;
		}
		points[index] = (b3Vec3){ x, y, z };
	}

	b3HullData* hull = b3CreateHull( points, (int)point_count, (int)point_count );
	if ( hull == NULL )
	{
		return V3_INVALID_DIMENSION;
	}

	status = v3_geometry_reserve_body_state_internal( world, world->active_body_count + 1u, new_entry_count );
	if ( status != V3_OK )
	{
		b3DestroyHull( hull );
		return status;
	}

	b3BodyDef body_definition = v3_geometry_make_body_definition( command );
	b3BodyId body_id = b3CreateBody( world->world_id, &body_definition );
	if ( B3_IS_NULL( body_id ) )
	{
		b3DestroyHull( hull );
		return V3_NATIVE_FAILURE;
	}

	b3ShapeDef shape_definition = v3_geometry_make_shape_definition( command );
	b3ShapeId shape_id = b3CreateHullShape( body_id, &shape_definition, hull );
	b3DestroyHull( hull );
	if ( B3_IS_NULL( shape_id ) )
	{
		b3DestroyBody( body_id );
		return V3_NATIVE_FAILURE;
	}

	v3_body_entry created = {
		.logical_id = command->logical_id,
		.generation = command->generation,
		.body_id = body_id,
		.shape_id = shape_id,
		.kind = command->kind,
	};
	v3_geometry_publish_body( world, &created );
	return V3_OK;
}

v3_status v3_world_create_sphere_body_internal( v3_world* world, const v3_body_definition* body, float radius, float density,
												float friction, v3_body_handle* out )
{
	if ( body->handle.reserved != 0 )
	{
		return V3_INVALID_ARGUMENT;
	}

	v3_box_body_command command = v3_geometry_make_sphere_command( body, radius, density, friction );
	uint32_t new_entry_count = 0;
	v3_status status = v3_geometry_validate_single_creation( world, &command, &new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}
	if ( world->active_body_count >= V3_MAX_BODIES_PER_BATCH )
	{
		return V3_LIMIT_EXCEEDED;
	}

	status = v3_geometry_reserve_body_state_internal( world, world->active_body_count + 1u, new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}

	b3BodyDef body_definition = v3_geometry_make_body_definition( &command );
	b3BodyId body_id = v3_geometry_create_body( world->world_id, &body_definition );
	if ( B3_IS_NULL( body_id ) )
	{
		return V3_NATIVE_FAILURE;
	}

	b3ShapeDef shape_definition = v3_geometry_make_shape_definition( &command );
	b3Sphere sphere = { .center = { 0.0f, 0.0f, 0.0f }, .radius = radius };
	b3ShapeId shape_id = v3_geometry_create_sphere_shape( body_id, &shape_definition, &sphere );
	if ( B3_IS_NULL( shape_id ) )
	{
		b3DestroyBody( body_id );
		return V3_NATIVE_FAILURE;
	}

	v3_body_entry created = {
		.logical_id = command.logical_id,
		.generation = command.generation,
		.body_id = body_id,
		.shape_id = shape_id,
		.kind = command.kind,
	};
	v3_geometry_publish_body( world, &created );
	*out = body->handle;
	return V3_OK;
}

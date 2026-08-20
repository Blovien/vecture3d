// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

typedef struct v3_pending_geometry_body
{
	uint64_t logical_id;
	uint32_t generation;
	b3BodyId body_id;
	b3ShapeId shape_id;
	b3CompoundData* owned_voxel;
	b3MassData mass_data;
	uint32_t kind;
	bool has_explicit_mass_data;
} v3_pending_geometry_body;

static bool v3_geometry_is_finite_positive( float value )
{
	return isfinite( value ) && value > 0.0f;
}

static uint32_t v3_geometry_saturating_add( uint32_t value, uint32_t increment )
{
	const uint32_t maximum = (uint32_t)INT32_MAX;
	return value >= maximum || maximum - value < increment ? maximum : value + increment;
}

static int v3_geometry_find_body_entry( const v3_world* world, uint64_t logical_id )
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

static bool v3_geometry_removes_logical_id( const v3_body_handle* removals, uint32_t removal_count, uint64_t logical_id )
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

static v3_status v3_geometry_validate_removals( const v3_world* world, const v3_body_handle* removals, uint32_t removal_count )
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

		int entry_index = v3_geometry_find_body_entry( world, removal->logical_id );
		if ( entry_index < 0 || !world->body_entries[entry_index].is_active ||
			 world->body_entries[entry_index].generation != removal->generation )
		{
			return V3_STALE_HANDLE;
		}
	}

	return V3_OK;
}

static bool v3_geometry_is_normalized_quaternion( float x, float y, float z, float w )
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

	if ( ( command->flags & ~( V3_BODY_ENABLE_SLEEP | V3_BODY_INITIAL_AWAKE | V3_BODY_DISABLE_COLLISION ) ) != 0 )
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

	if ( !v3_geometry_is_normalized_quaternion( command->rotation_x, command->rotation_y, command->rotation_z,
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

	int entry_index = v3_geometry_find_body_entry( world, command->logical_id );
	if ( entry_index < 0 )
	{
		if ( command->generation != 1 )
		{
			return V3_INVALID_GENERATION;
		}
		*new_entry_count = 1;
		return V3_OK;
	}

	*new_entry_count = 0;
	const v3_body_entry* entry = world->body_entries + entry_index;
	if ( entry->is_active )
	{
		return V3_DUPLICATE_ID;
	}
	if ( entry->generation == INT32_MAX )
	{
		return V3_GENERATION_EXHAUSTED;
	}
	return command->generation == entry->generation + 1u ? V3_OK : V3_INVALID_GENERATION;
}

static v3_status v3_geometry_validate_terrain_sections( const v3_world* world, const v3_body_handle* removals,
														uint32_t removal_count, const v3_terrain_section_command* sections,
														uint32_t section_count, const v3_voxel_run* voxel_runs,
														uint32_t voxel_run_count, const v3_terrain_box* detail_boxes,
														uint32_t detail_box_count, uint32_t* new_entry_count )
{
	*new_entry_count = 0;
	uint32_t expected_voxel_run_offset = 0;
	uint32_t expected_detail_box_offset = 0;
	for ( uint32_t index = 0; index < section_count; ++index )
	{
		const v3_terrain_section_command* section = sections + index;
		if ( section->voxel_run_count > V3_MAX_VOXEL_CHILDREN_PER_SECTION ||
			 section->detail_box_count > V3_MAX_VOXEL_CHILDREN_PER_SECTION - section->voxel_run_count )
		{
			return V3_INVALID_ARGUMENT;
		}

		uint32_t child_count = section->voxel_run_count + section->detail_box_count;
		if ( section->logical_id == 0 || section->generation == 0 || section->generation > INT32_MAX || section->reserved != 0 ||
			 child_count == 0 || section->voxel_run_offset != expected_voxel_run_offset ||
			 section->detail_box_offset != expected_detail_box_offset || expected_voxel_run_offset > voxel_run_count ||
			 section->voxel_run_count > voxel_run_count - expected_voxel_run_offset ||
			 expected_detail_box_offset > detail_box_count ||
			 section->detail_box_count > detail_box_count - expected_detail_box_offset )
		{
			return V3_INVALID_ARGUMENT;
		}
		if ( !isfinite( section->origin_x ) || !isfinite( section->origin_y ) || !isfinite( section->origin_z ) )
		{
			return V3_NON_FINITE;
		}

		for ( uint32_t previous = 0; previous < index; ++previous )
		{
			if ( sections[previous].logical_id == section->logical_id )
			{
				return V3_DUPLICATE_ID;
			}
		}

		int entry_index = v3_geometry_find_body_entry( world, section->logical_id );
		if ( entry_index < 0 )
		{
			if ( section->generation != 1 )
			{
				return V3_INVALID_GENERATION;
			}
			*new_entry_count += 1;
		}
		else
		{
			const v3_body_entry* entry = world->body_entries + entry_index;
			if ( entry->is_active && !v3_geometry_removes_logical_id( removals, removal_count, section->logical_id ) )
			{
				return V3_DUPLICATE_ID;
			}
			if ( entry->generation == INT32_MAX )
			{
				return V3_GENERATION_EXHAUSTED;
			}
			if ( section->generation != entry->generation + 1u )
			{
				return V3_INVALID_GENERATION;
			}
		}

		for ( uint32_t child = 0; child < section->voxel_run_count; ++child )
		{
			uint32_t packed = voxel_runs[section->voxel_run_offset + child].packed;
			uint32_t x = packed & 31u;
			uint32_t y = packed >> 5u & 31u;
			uint32_t z = packed >> 10u & 31u;
			uint32_t width = ( packed >> 15u & 31u ) + 1u;
			uint32_t height = ( packed >> 20u & 31u ) + 1u;
			uint32_t depth = ( packed >> 25u & 31u ) + 1u;
			if ( ( packed & ~V3_VOXEL_RUN_USED_BITS ) != 0 || x + width > V3_VOXEL_SECTION_SIZE ||
				 y + height > V3_VOXEL_SECTION_SIZE || z + depth > V3_VOXEL_SECTION_SIZE )
			{
				return V3_INVALID_ARGUMENT;
			}
		}

		for ( uint32_t child = 0; child < section->detail_box_count; ++child )
		{
			const v3_terrain_box* box = detail_boxes + section->detail_box_offset + child;
			if ( box->reserved != 0 || box->feature_id == 0 )
			{
				return V3_INVALID_ARGUMENT;
			}
			if ( !isfinite( box->center_x ) || !isfinite( box->center_y ) || !isfinite( box->center_z ) ||
				 !isfinite( box->half_extent_x ) || !isfinite( box->half_extent_y ) || !isfinite( box->half_extent_z ) ||
				 !isfinite( box->friction ) )
			{
				return V3_NON_FINITE;
			}
			if ( !v3_geometry_is_finite_positive( box->half_extent_x ) || !v3_geometry_is_finite_positive( box->half_extent_y ) ||
				 !v3_geometry_is_finite_positive( box->half_extent_z ) )
			{
				return V3_INVALID_DIMENSION;
			}
			if ( box->friction < 0.0f || box->friction > 1.0f )
			{
				return V3_INVALID_FRICTION;
			}
		}

		expected_voxel_run_offset += section->voxel_run_count;
		expected_detail_box_offset += section->detail_box_count;
	}

	return expected_voxel_run_offset == voxel_run_count && expected_detail_box_offset == detail_box_count ? V3_OK
																										  : V3_INVALID_ARGUMENT;
}

static v3_status v3_geometry_reserve_body_entries( v3_world* world, uint32_t required_capacity )
{
	if ( required_capacity <= world->body_entry_capacity )
	{
		return V3_OK;
	}

	uint32_t new_capacity = world->body_entry_capacity == 0 ? UINT32_C( 16 ) : world->body_entry_capacity;
	while ( new_capacity < required_capacity )
	{
		new_capacity = new_capacity > V3_MAX_LOGICAL_BODY_IDS / 2u ? V3_MAX_LOGICAL_BODY_IDS : new_capacity * 2u;
	}

	v3_body_entry* entries = realloc( world->body_entries, (size_t)new_capacity * sizeof( *entries ) );
	if ( entries == NULL )
	{
		return V3_OUT_OF_MEMORY;
	}

	world->body_entries = entries;
	world->body_entry_capacity = new_capacity;
	return V3_OK;
}

static void v3_geometry_destroy_body_entry( v3_body_entry* entry )
{
	// Voxel shapes borrow their compound, so the body and shape must die before the payload.
	b3DestroyBody( entry->body_id );
	if ( entry->owned_voxel != NULL )
	{
		b3DestroyCompound( entry->owned_voxel );
		entry->owned_voxel = NULL;
	}
	entry->is_active = false;
}

static void v3_geometry_destroy_pending_body( v3_pending_geometry_body* pending )
{
	b3DestroyBody( pending->body_id );
	if ( pending->owned_voxel != NULL )
	{
		b3DestroyCompound( pending->owned_voxel );
		pending->owned_voxel = NULL;
	}
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
	return definition;
}

static b3ShapeDef v3_geometry_make_shape_definition( const v3_box_body_command* command )
{
	b3ShapeDef definition = b3DefaultShapeDef();
	definition.density = command->density;
	definition.baseMaterial.friction = command->friction;
	definition.filter.categoryBits = command->kind == V3_STATIC_BODY	  ? V3_STATIC_CATEGORY
									 : command->kind == V3_KINEMATIC_BODY ? V3_KINEMATIC_CATEGORY
																		  : V3_DYNAMIC_CATEGORY;
	definition.filter.maskBits = ( command->flags & V3_BODY_DISABLE_COLLISION ) != 0 ? 0u : V3_ALL_BODY_CATEGORIES;
	return definition;
}

static void v3_geometry_publish_body( v3_world* world, const v3_pending_geometry_body* created )
{
	int entry_index = v3_geometry_find_body_entry( world, created->logical_id );
	if ( entry_index < 0 )
	{
		entry_index = (int)world->body_entry_count++;
	}

	world->body_entries[entry_index] = (v3_body_entry){
		.logical_id = created->logical_id,
		.generation = created->generation,
		.body_id = created->body_id,
		.shape_id = created->shape_id,
		.owned_voxel = created->owned_voxel,
		.mass_data = created->mass_data,
		.kind = created->kind,
		.has_explicit_mass_data = created->has_explicit_mass_data,
		.is_active = true,
	};
}

v3_status v3_world_replace_box_bodies_geometry_aware_internal( v3_world* world, const v3_body_handle* removals,
															   uint32_t removal_count, const v3_box_body_command* creations,
															   uint32_t creation_count )
{
	// Successful box replacement can overwrite a logical entry, so retain compound owners until native teardown finishes.
	b3CompoundData* removed_voxels[V3_MAX_BODIES_PER_BATCH] = { 0 };
	for ( uint32_t index = 0; index < removal_count; ++index )
	{
		int entry_index = v3_geometry_find_body_entry( world, removals[index].logical_id );
		if ( entry_index >= 0 )
		{
			removed_voxels[index] = world->body_entries[entry_index].owned_voxel;
		}
	}

	v3_status status = v3_world_replace_box_bodies_internal( world, removals, removal_count, creations, creation_count );
	if ( status != V3_OK )
	{
		return status;
	}

	for ( uint32_t index = 0; index < removal_count; ++index )
	{
		if ( removed_voxels[index] == NULL )
		{
			continue;
		}

		int entry_index = v3_geometry_find_body_entry( world, removals[index].logical_id );
		if ( entry_index >= 0 && world->body_entries[entry_index].owned_voxel == removed_voxels[index] )
		{
			world->body_entries[entry_index].owned_voxel = NULL;
		}
		b3DestroyCompound( removed_voxels[index] );
	}

	return V3_OK;
}

void v3_geometry_destroy_owned_payloads_internal( v3_world* world )
{
	if ( world == NULL )
	{
		return;
	}

	// Voxel shapes borrow their compounds, so public world teardown destroys each body before releasing its payload.
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		v3_body_entry* entry = world->body_entries + index;
		if ( entry->owned_voxel == NULL )
		{
			continue;
		}

		if ( entry->is_active )
		{
			b3DestroyBody( entry->body_id );
			entry->is_active = false;
		}
		b3DestroyCompound( entry->owned_voxel );
		entry->owned_voxel = NULL;
	}
}

v3_status v3_world_replace_terrain_sections_internal( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
													  const v3_terrain_section_command* sections, uint32_t section_count,
													  const v3_voxel_run* voxel_runs, uint32_t voxel_run_count,
													  const v3_terrain_box* detail_boxes, uint32_t detail_box_count )
{
	v3_status status = v3_geometry_validate_removals( world, removals, removal_count );
	if ( status != V3_OK )
	{
		return status;
	}

	uint32_t new_entry_count = 0;
	status = v3_geometry_validate_terrain_sections( world, removals, removal_count, sections, section_count, voxel_runs,
													voxel_run_count, detail_boxes, detail_box_count, &new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}

	uint32_t final_body_count = world->active_body_count - removal_count + section_count;
	if ( final_body_count > V3_MAX_BODIES_PER_BATCH || world->body_entry_count + new_entry_count > V3_MAX_LOGICAL_BODY_IDS )
	{
		return V3_LIMIT_EXCEEDED;
	}
	if ( world->active_body_count > V3_MAX_BODIES_PER_BATCH - section_count )
	{
		return V3_PEAK_LIMIT_EXCEEDED;
	}

	status = v3_geometry_reserve_body_entries( world, world->body_entry_count + new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}

	v3_pending_geometry_body* pending = NULL;
	if ( section_count > 0 )
	{
		pending = calloc( section_count, sizeof( *pending ) );
		if ( pending == NULL )
		{
			return V3_OUT_OF_MEMORY;
		}
	}

	uint32_t created_count = 0;
	for ( uint32_t section_index = 0; section_index < section_count; ++section_index )
	{
		const v3_terrain_section_command* section = sections + section_index;
		uint32_t child_count = section->voxel_run_count + section->detail_box_count;
		b3BoxHull* hulls = calloc( child_count, sizeof( *hulls ) );
		b3CompoundHullDef* hull_definitions = calloc( child_count, sizeof( *hull_definitions ) );
		if ( hulls == NULL || hull_definitions == NULL )
		{
			free( hull_definitions );
			free( hulls );
			status = V3_OUT_OF_MEMORY;
			break;
		}

		for ( uint32_t child_index = 0; child_index < section->voxel_run_count; ++child_index )
		{
			uint32_t packed = voxel_runs[section->voxel_run_offset + child_index].packed;
			uint32_t x = packed & 31u;
			uint32_t y = packed >> 5u & 31u;
			uint32_t z = packed >> 10u & 31u;
			uint32_t width = ( packed >> 15u & 31u ) + 1u;
			uint32_t height = ( packed >> 20u & 31u ) + 1u;
			uint32_t depth = ( packed >> 25u & 31u ) + 1u;
			hulls[child_index] = b3MakeBoxHull( 0.5f * (float)width, 0.5f * (float)height, 0.5f * (float)depth );
			b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
			material.friction = V3_DEFAULT_VOXEL_FRICTION;
			uint64_t feature_id = section->logical_id ^ ( (uint64_t)packed << 32u ) ^ packed;
			material.userMaterialId = feature_id == 0 ? 1 : feature_id;
			hull_definitions[child_index] = (b3CompoundHullDef){
				.hull = &hulls[child_index].base,
				.transform =
					{
						.p = { (float)x + 0.5f * (float)width, (float)y + 0.5f * (float)height, (float)z + 0.5f * (float)depth },
						.q = { { 0.0f, 0.0f, 0.0f }, 1.0f },
					},
				.material = material,
			};
		}

		for ( uint32_t detail_index = 0; detail_index < section->detail_box_count; ++detail_index )
		{
			uint32_t child_index = section->voxel_run_count + detail_index;
			const v3_terrain_box* box = detail_boxes + section->detail_box_offset + detail_index;
			hulls[child_index] = b3MakeBoxHull( box->half_extent_x, box->half_extent_y, box->half_extent_z );
			b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
			material.friction = box->friction;
			material.userMaterialId = box->feature_id;
			hull_definitions[child_index] = (b3CompoundHullDef){
				.hull = &hulls[child_index].base,
				.transform =
					{
						.p = { box->center_x, box->center_y, box->center_z },
						.q = { { 0.0f, 0.0f, 0.0f }, 1.0f },
					},
				.material = material,
			};
		}

		b3CompoundDef compound_definition = {
			.hulls = hull_definitions,
			.hullCount = (int)child_count,
		};
		b3CompoundData* voxel = b3CreateCompound( &compound_definition );
		free( hull_definitions );
		free( hulls );
		if ( voxel == NULL )
		{
			status = V3_NATIVE_FAILURE;
			break;
		}

		b3BodyDef body_definition = b3DefaultBodyDef();
		body_definition.type = b3_staticBody;
		body_definition.position = (b3Pos){ section->origin_x, section->origin_y, section->origin_z };
		b3BodyId body_id = b3CreateBody( world->world_id, &body_definition );
		if ( B3_IS_NULL( body_id ) )
		{
			b3DestroyCompound( voxel );
			status = V3_NATIVE_FAILURE;
			break;
		}

		b3ShapeDef shape_definition = b3DefaultShapeDef();
		shape_definition.filter.categoryBits = V3_STATIC_CATEGORY;
		shape_definition.filter.maskBits = V3_ALL_BODY_CATEGORIES;
		// Streamed voxels can appear under resting bodies, so pair them during publication.
		shape_definition.invokeContactCreation = true;
		b3ShapeId shape_id = b3CreateVoxelShape( body_id, &shape_definition, voxel );
		if ( B3_IS_NULL( shape_id ) )
		{
			b3DestroyBody( body_id );
			b3DestroyCompound( voxel );
			status = V3_NATIVE_FAILURE;
			break;
		}

		pending[created_count++] = (v3_pending_geometry_body){
			.logical_id = section->logical_id,
			.generation = section->generation,
			.body_id = body_id,
			.shape_id = shape_id,
			.owned_voxel = voxel,
			.kind = V3_STATIC_BODY,
		};
	}

	if ( status != V3_OK )
	{
		while ( created_count > 0 )
		{
			v3_geometry_destroy_pending_body( pending + --created_count );
		}
		free( pending );
		return status;
	}

	for ( uint32_t index = 0; index < removal_count; ++index )
	{
		int entry_index = v3_geometry_find_body_entry( world, removals[index].logical_id );
		v3_geometry_destroy_body_entry( world->body_entries + entry_index );
	}

	for ( uint32_t index = 0; index < section_count; ++index )
	{
		v3_geometry_publish_body( world, pending + index );
	}

	world->active_body_count = final_body_count;
	if ( removal_count > 0 || section_count > 0 )
	{
		world->mutation_batch_count = v3_geometry_saturating_add( world->mutation_batch_count, 1 );
		world->created_body_count = v3_geometry_saturating_add( world->created_body_count, section_count );
		world->destroyed_body_count = v3_geometry_saturating_add( world->destroyed_body_count, removal_count );
	}

	free( pending );
	return V3_OK;
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
	if ( world->active_body_count >= V3_MAX_BODIES_PER_BATCH ||
		 world->body_entry_count + new_entry_count > V3_MAX_LOGICAL_BODY_IDS )
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

	status = v3_geometry_reserve_body_entries( world, world->body_entry_count + new_entry_count );
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

	v3_pending_geometry_body created = {
		.logical_id = command->logical_id,
		.generation = command->generation,
		.body_id = body_id,
		.shape_id = shape_id,
		.kind = command->kind,
	};
	v3_geometry_publish_body( world, &created );
	world->active_body_count += 1u;
	world->mutation_batch_count = v3_geometry_saturating_add( world->mutation_batch_count, 1 );
	world->created_body_count = v3_geometry_saturating_add( world->created_body_count, 1 );
	return V3_OK;
}

v3_status v3_world_create_voxel_group_internal( v3_world* world, const v3_box_body_command* command, const v3_terrain_box* boxes,
												uint32_t box_count, const v3_mass_properties* mass_properties )
{
	if ( command->kind != V3_DYNAMIC_BODY )
	{
		return V3_INVALID_ARGUMENT;
	}

	uint32_t new_entry_count = 0;
	v3_status status = v3_geometry_validate_single_creation( world, command, &new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}
	if ( world->active_body_count >= V3_MAX_BODIES_PER_BATCH ||
		 world->body_entry_count + new_entry_count > V3_MAX_LOGICAL_BODY_IDS )
	{
		return V3_LIMIT_EXCEEDED;
	}

	float leading_inertia_minor =
		mass_properties->inertia_xx * mass_properties->inertia_yy - mass_properties->inertia_xy * mass_properties->inertia_xy;
	float inertia_determinant = mass_properties->inertia_xx * mass_properties->inertia_yy * mass_properties->inertia_zz +
								2.0f * mass_properties->inertia_xy * mass_properties->inertia_xz * mass_properties->inertia_yz -
								mass_properties->inertia_xx * mass_properties->inertia_yz * mass_properties->inertia_yz -
								mass_properties->inertia_yy * mass_properties->inertia_xz * mass_properties->inertia_xz -
								mass_properties->inertia_zz * mass_properties->inertia_xy * mass_properties->inertia_xy;
	if ( !isfinite( mass_properties->mass ) || mass_properties->mass <= 0.0f || !isfinite( mass_properties->center_x ) ||
		 !isfinite( mass_properties->center_y ) || !isfinite( mass_properties->center_z ) ||
		 !isfinite( mass_properties->inertia_xx ) || !isfinite( mass_properties->inertia_yy ) ||
		 !isfinite( mass_properties->inertia_zz ) || !isfinite( mass_properties->inertia_xy ) ||
		 !isfinite( mass_properties->inertia_xz ) || !isfinite( mass_properties->inertia_yz ) ||
		 !isfinite( leading_inertia_minor ) || !isfinite( inertia_determinant ) || mass_properties->inertia_xx <= 0.0f ||
		 leading_inertia_minor <= 0.0f || inertia_determinant <= 0.0f )
	{
		return V3_INVALID_DENSITY;
	}

	for ( uint32_t index = 0; index < box_count; ++index )
	{
		const v3_terrain_box* box = boxes + index;
		if ( box->reserved != 0 || box->feature_id == 0 )
		{
			return V3_INVALID_ARGUMENT;
		}
		if ( !isfinite( box->center_x ) || !isfinite( box->center_y ) || !isfinite( box->center_z ) ||
			 !v3_geometry_is_finite_positive( box->half_extent_x ) || !v3_geometry_is_finite_positive( box->half_extent_y ) ||
			 !v3_geometry_is_finite_positive( box->half_extent_z ) )
		{
			return V3_INVALID_DIMENSION;
		}
		if ( !isfinite( box->friction ) || box->friction < 0.0f || box->friction > 1.0f )
		{
			return V3_INVALID_FRICTION;
		}
	}

	status = v3_geometry_reserve_body_entries( world, world->body_entry_count + new_entry_count );
	if ( status != V3_OK )
	{
		return status;
	}

	b3BodyDef body_definition = v3_geometry_make_body_definition( command );
	body_definition.type = b3_dynamicBody;
	b3BodyId body_id = b3CreateBody( world->world_id, &body_definition );
	if ( B3_IS_NULL( body_id ) )
	{
		return V3_NATIVE_FAILURE;
	}

	b3ShapeId first_shape_id = b3_nullShapeId;
	for ( uint32_t index = 0; index < box_count; ++index )
	{
		const v3_terrain_box* box = boxes + index;
		b3BoxHull hull = b3MakeBoxHull( box->half_extent_x, box->half_extent_y, box->half_extent_z );
		b3ShapeDef shape_definition = b3DefaultShapeDef();
		// Install explicit all-child mass data only after the complete collision shell exists.
		shape_definition.density = 0.0f;
		shape_definition.baseMaterial.friction = box->friction;
		shape_definition.baseMaterial.userMaterialId = box->feature_id;
		shape_definition.filter.categoryBits = V3_DYNAMIC_CATEGORY;
		shape_definition.filter.maskBits = ( command->flags & V3_BODY_DISABLE_COLLISION ) != 0 ? 0u : V3_ALL_BODY_CATEGORIES;
		b3Transform transform = {
			.p = { box->center_x, box->center_y, box->center_z },
			.q = { { 0.0f, 0.0f, 0.0f }, 1.0f },
		};
		b3ShapeId shape_id =
			b3CreateTransformedHullShape( body_id, &shape_definition, &hull.base, transform, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
		if ( B3_IS_NULL( shape_id ) )
		{
			b3DestroyBody( body_id );
			return V3_NATIVE_FAILURE;
		}
		if ( index == 0 )
		{
			first_shape_id = shape_id;
		}
	}

	b3MassData mass_data = {
		.mass = mass_properties->mass,
		.center = { mass_properties->center_x, mass_properties->center_y, mass_properties->center_z },
		.inertia =
			{
				.cx = { mass_properties->inertia_xx, mass_properties->inertia_xy, mass_properties->inertia_xz },
				.cy = { mass_properties->inertia_xy, mass_properties->inertia_yy, mass_properties->inertia_yz },
				.cz = { mass_properties->inertia_xz, mass_properties->inertia_yz, mass_properties->inertia_zz },
			},
	};
	b3Body_SetMassData( body_id, mass_data );

	v3_pending_geometry_body created = {
		.logical_id = command->logical_id,
		.generation = command->generation,
		.body_id = body_id,
		.shape_id = first_shape_id,
		.mass_data = mass_data,
		.kind = command->kind,
		.has_explicit_mass_data = true,
	};
	v3_geometry_publish_body( world, &created );
	world->active_body_count += 1u;
	world->mutation_batch_count = v3_geometry_saturating_add( world->mutation_batch_count, 1 );
	world->created_body_count = v3_geometry_saturating_add( world->created_body_count, 1 );
	return V3_OK;
}

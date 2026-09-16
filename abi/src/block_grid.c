// SPDX-License-Identifier: MIT

#include "internal.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <vecture3d/block_grid.h>

// Each cooking handle retains one reference to immutable cooked data. At cook time, material zero
// supplies the density used when creating shapes from this data.
struct v3_cooked_grid
{
	v3BlockGridData* data;
	float shape_density;
};

// Translation arrays live only for the duration of one cook, because the cooker copies everything
// it is given before returning.
typedef struct v3_block_grid_input
{
	b3SurfaceMaterial* surfaces;
	v3BlockGridBlock* blocks;
	v3BlockGridBox* boxes;
	uint32_t* box_counts;
} v3_block_grid_input;

static void v3_block_grid_free_input( v3_block_grid_input* input )
{
	free( input->surfaces );
	free( input->blocks );
	free( input->boxes );
	free( input->box_counts );
	*input = (v3_block_grid_input){ 0 };
}

static v3_status v3_block_grid_map_cook_status( v3BlockGridCookStatus status )
{
	switch ( status )
	{
		case v3_blockGridCookOk:
			return V3_OK;
		case v3_blockGridCookInvalidDefinition:
			return V3_INVALID_ARGUMENT;
		case v3_blockGridCookLimitExceeded:
			return V3_LIMIT_EXCEEDED;
		case v3_blockGridCookOutOfMemory:
			return V3_OUT_OF_MEMORY;
		case v3_blockGridCookInvalidMaterial:
			return V3_INVALID_MATERIAL;
		case v3_blockGridCookInvalidBlock:
			return V3_INVALID_CELL;
		case v3_blockGridCookDuplicateBlock:
			return V3_DUPLICATE_CELL;
		case v3_blockGridCookInvalidBox:
			return V3_INVALID_DIMENSION;
	}

	return V3_NATIVE_FAILURE;
}

static v3_status v3_block_grid_validate_materials( const v3_block_material* materials, uint32_t material_count )
{
	for ( uint32_t index = 0; index < material_count; ++index )
	{
		const v3_block_material* material = materials + index;
		if ( !isfinite( material->density ) || !isfinite( material->friction ) || !isfinite( material->restitution ) ||
			 !isfinite( material->bond_strength ) || !isfinite( material->compressive_factor ) )
		{
			return V3_NON_FINITE;
		}
		// Reserved fracture fields do not affect cooking; the float fields must still be finite.
		if ( material->density < 0.0f || material->friction < 0.0f || material->friction > 1.0f || material->restitution < 0.0f ||
			 material->restitution > 1.0f )
		{
			return V3_INVALID_MATERIAL;
		}
	}

	return V3_OK;
}

static v3_status v3_block_grid_validate_cells( const v3_block_cell* cells, uint32_t cell_count, uint32_t material_count )
{
	for ( uint32_t index = 0; index < cell_count; ++index )
	{
		const v3_block_cell* cell = cells + index;
		if ( cell->flags != 0 || cell->reserved != 0 )
		{
			return V3_INVALID_ARGUMENT;
		}
		if ( cell->material_index >= material_count )
		{
			return V3_INVALID_MATERIAL;
		}
	}

	return V3_OK;
}

static v3_status v3_block_grid_validate_boxes( const v3_block_box* boxes, uint32_t box_count, uint32_t cell_count,
											   uint32_t material_count )
{
	for ( uint32_t index = 0; index < box_count; ++index )
	{
		const v3_block_box* box = boxes + index;
		if ( box->owner_cell_index >= cell_count )
		{
			return V3_INVALID_OWNER;
		}
		if ( box->material_index >= material_count )
		{
			return V3_INVALID_MATERIAL;
		}
		if ( !isfinite( box->center_x ) || !isfinite( box->center_y ) || !isfinite( box->center_z ) ||
			 !isfinite( box->half_extent_x ) || !isfinite( box->half_extent_y ) || !isfinite( box->half_extent_z ) )
		{
			return V3_NON_FINITE;
		}
		if ( box->half_extent_x <= 0.0f || box->half_extent_y <= 0.0f || box->half_extent_z <= 0.0f )
		{
			return V3_INVALID_DIMENSION;
		}
	}

	return V3_OK;
}

// Boxes name their owning cell and may arrive in any order, so one counting pass sizes every cell's
// run and a second pass fills it. Input order is preserved inside a run, which is the order that
// identifies a box within its block.
static void v3_block_grid_group_boxes( const v3_block_cell* cells, uint32_t cell_count, const v3_block_box* boxes,
									   uint32_t box_count, v3_block_grid_input* input )
{
	for ( uint32_t index = 0; index < box_count; ++index )
	{
		input->box_counts[boxes[index].owner_cell_index] += 1u;
	}

	// Turn the per-cell counts into run starts so the fill pass below can index the mutable boxes
	// array this function owns instead of writing through the block's const view of it.
	uint32_t offset = 0;
	for ( uint32_t index = 0; index < cell_count; ++index )
	{
		const v3_block_cell* cell = cells + index;
		uint32_t run_size = input->box_counts[index];
		input->box_counts[index] = offset;
		input->blocks[index] = (v3BlockGridBlock){
			.x = cell->x,
			.y = cell->y,
			.z = cell->z,
			.userData = cell->feature_id,
			.boxes = input->boxes + offset,
			.boxCount = 0,
		};
		offset += run_size;
	}

	for ( uint32_t index = 0; index < box_count; ++index )
	{
		const v3_block_box* box = boxes + index;
		v3BlockGridBlock* block = input->blocks + box->owner_cell_index;
		// Appending at the run start keeps the input order of the boxes inside each cell.
		uint32_t write_index = input->box_counts[box->owner_cell_index] + (uint32_t)block->boxCount;
		// The center and the half extents are cell-local, so the bounds are the unit-cell AABB the
		// cooker expects and it adds the owning cell coordinate itself.
		input->boxes[write_index] = (v3BlockGridBox){
			.bounds =
				{
					.lowerBound = { box->center_x - box->half_extent_x, box->center_y - box->half_extent_y,
									box->center_z - box->half_extent_z },
					.upperBound = { box->center_x + box->half_extent_x, box->center_y + box->half_extent_y,
									box->center_z + box->half_extent_z },
				},
			.materialIndex = box->material_index,
		};
		block->boxCount += 1;
	}
}

static v3_status v3_block_grid_build_input( const v3_block_material* materials, uint32_t material_count,
											const v3_block_cell* cells, uint32_t cell_count, const v3_block_box* boxes,
											uint32_t box_count, v3_block_grid_input* input )
{
	input->surfaces = calloc( material_count, sizeof( *input->surfaces ) );
	input->blocks = calloc( cell_count, sizeof( *input->blocks ) );
	input->boxes = calloc( box_count, sizeof( *input->boxes ) );
	input->box_counts = calloc( cell_count, sizeof( *input->box_counts ) );
	if ( input->surfaces == NULL || input->blocks == NULL || input->boxes == NULL || input->box_counts == NULL )
	{
		v3_block_grid_free_input( input );
		return V3_OUT_OF_MEMORY;
	}

	for ( uint32_t index = 0; index < material_count; ++index )
	{
		const v3_block_material* material = materials + index;
		b3SurfaceMaterial surface = b3DefaultSurfaceMaterial();
		surface.friction = material->friction;
		surface.restitution = material->restitution;
		// Both identifiers are 64 bits wide, so nothing is truncated.
		surface.userMaterialId = material->material_id;
		input->surfaces[index] = surface;
	}

	v3_block_grid_group_boxes( cells, cell_count, boxes, box_count, input );
	return V3_OK;
}

v3_status v3_cook_block_grid_internal( const v3_block_material* materials, uint32_t material_count, const v3_block_cell* cells,
									   uint32_t cell_count, const v3_block_box* boxes, uint32_t box_count, v3_cooked_grid** out )
{
	v3_status status = v3_block_grid_validate_materials( materials, material_count );
	if ( status != V3_OK )
	{
		return status;
	}
	status = v3_block_grid_validate_cells( cells, cell_count, material_count );
	if ( status != V3_OK )
	{
		return status;
	}
	status = v3_block_grid_validate_boxes( boxes, box_count, cell_count, material_count );
	if ( status != V3_OK )
	{
		return status;
	}

	v3_cooked_grid* grid = calloc( 1, sizeof( *grid ) );
	if ( grid == NULL )
	{
		return V3_OUT_OF_MEMORY;
	}

	v3_block_grid_input input = { 0 };
	status = v3_block_grid_build_input( materials, material_count, cells, cell_count, boxes, box_count, &input );
	if ( status != V3_OK )
	{
		free( grid );
		return status;
	}

	v3BlockGridCookDef definition = {
		.materials = input.surfaces,
		.materialCount = (int)material_count,
		.blocks = input.blocks,
		.blockCount = (int)cell_count,
	};
	v3BlockGridCookResult result = v3CookBlockGrid( &definition );
	v3_block_grid_free_input( &input );
	if ( result.status != v3_blockGridCookOk )
	{
		free( grid );
		return v3_block_grid_map_cook_status( result.status );
	}

	grid->data = result.data;
	grid->shape_density = materials[0].density > 0.0f ? materials[0].density : 1.0f;
	*out = grid;
	return V3_OK;
}

void v3_destroy_cooked_grid_internal( v3_cooked_grid* grid )
{
	if ( grid == NULL )
	{
		return;
	}

	v3DestroyBlockGridData( grid->data );
	free( grid );
}

static v3_status v3_block_grid_validate_body( const v3_body_definition* body )
{
	if ( body->handle.logical_id == 0 || body->handle.reserved != 0 ||
		 ( body->kind != V3_STATIC_BODY && body->kind != V3_KINEMATIC_BODY && body->kind != V3_DYNAMIC_BODY ) )
	{
		return V3_INVALID_ARGUMENT;
	}

	if ( body->handle.generation == 0 || body->handle.generation > INT32_MAX )
	{
		return V3_INVALID_GENERATION;
	}

	if ( !v3_body_flags_are_valid( body->kind, body->flags ) )
	{
		return V3_INVALID_ARGUMENT;
	}

	if ( !isfinite( body->position_x ) || !isfinite( body->position_y ) || !isfinite( body->position_z ) ||
		 !isfinite( body->linear_velocity_x ) || !isfinite( body->linear_velocity_y ) || !isfinite( body->linear_velocity_z ) ||
		 !isfinite( body->angular_velocity_x ) || !isfinite( body->angular_velocity_y ) ||
		 !isfinite( body->angular_velocity_z ) || !isfinite( body->linear_damping ) || !isfinite( body->angular_damping ) )
	{
		return V3_NON_FINITE;
	}

	if ( !v3_geometry_is_normalized_quaternion_internal( body->rotation_x, body->rotation_y, body->rotation_z,
														 body->rotation_w ) )
	{
		return V3_INVALID_QUATERNION;
	}

	if ( body->linear_damping < 0.0f || body->linear_damping > V3_MAX_DAMPING || body->angular_damping < 0.0f ||
		 body->angular_damping > V3_MAX_DAMPING )
	{
		return V3_INVALID_DAMPING;
	}

	return V3_OK;
}

static v3_status v3_block_grid_validate_mass( const v3_mass_properties* mass )
{
	float leading_minor = mass->inertia_xx * mass->inertia_yy - mass->inertia_xy * mass->inertia_xy;
	float determinant =
		mass->inertia_xx * mass->inertia_yy * mass->inertia_zz + 2.0f * mass->inertia_xy * mass->inertia_xz * mass->inertia_yz -
		mass->inertia_xx * mass->inertia_yz * mass->inertia_yz - mass->inertia_yy * mass->inertia_xz * mass->inertia_xz -
		mass->inertia_zz * mass->inertia_xy * mass->inertia_xy;
	if ( !isfinite( mass->mass ) || mass->mass <= 0.0f || !isfinite( mass->center_x ) || !isfinite( mass->center_y ) ||
		 !isfinite( mass->center_z ) || !isfinite( mass->inertia_xx ) || !isfinite( mass->inertia_yy ) ||
		 !isfinite( mass->inertia_zz ) || !isfinite( mass->inertia_xy ) || !isfinite( mass->inertia_xz ) ||
		 !isfinite( mass->inertia_yz ) || !isfinite( leading_minor ) || !isfinite( determinant ) || mass->inertia_xx <= 0.0f ||
		 leading_minor <= 0.0f || determinant <= 0.0f )
	{
		return V3_INVALID_DENSITY;
	}

	return V3_OK;
}

static v3_status v3_block_grid_validate_identity( const v3_world* world, const v3_body_definition* body,
												  uint32_t* new_entry_count )
{
	return v3_geometry_validate_body_generation_internal( world, body->handle.logical_id, body->handle.generation, false,
														  new_entry_count );
}

v3_status v3_world_attach_block_grid_internal( v3_world* world, const v3_body_definition* body, v3_cooked_grid* grid,
											   const v3_mass_properties* mass_override, v3_body_handle* out )
{
	if ( grid->data == NULL )
	{
		return V3_INVALID_BLOCK_GRID;
	}

	v3_status status = v3_block_grid_validate_body( body );
	if ( status != V3_OK )
	{
		return status;
	}
	if ( mass_override != NULL )
	{
		status = v3_block_grid_validate_mass( mass_override );
		if ( status != V3_OK )
		{
			return status;
		}
	}

	uint32_t new_entry_count = 0;
	status = v3_block_grid_validate_identity( world, body, &new_entry_count );
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

	b3BodyDef body_definition = b3DefaultBodyDef();
	body_definition.type = (b3BodyType)body->kind;
	body_definition.position = (b3Pos){ body->position_x, body->position_y, body->position_z };
	body_definition.rotation = (b3Quat){ { body->rotation_x, body->rotation_y, body->rotation_z }, body->rotation_w };
	body_definition.linearVelocity = (b3Vec3){ body->linear_velocity_x, body->linear_velocity_y, body->linear_velocity_z };
	body_definition.angularVelocity = (b3Vec3){ body->angular_velocity_x, body->angular_velocity_y, body->angular_velocity_z };
	body_definition.linearDamping = body->linear_damping;
	body_definition.angularDamping = body->angular_damping;
	body_definition.enableSleep = ( body->flags & V3_BODY_ENABLE_SLEEP ) != 0;
	body_definition.isAwake = ( body->flags & V3_BODY_INITIAL_AWAKE ) != 0;
	body_definition.isBullet = ( body->flags & V3_BODY_BULLET ) != 0;

	b3BodyId body_id = b3CreateBody( world->world_id, &body_definition );
	if ( B3_IS_NULL( body_id ) )
	{
		return V3_NATIVE_FAILURE;
	}

	b3ShapeDef shape_definition = b3DefaultShapeDef();
	shape_definition.density = grid->shape_density;
	shape_definition.enableContactEvents = true;
	shape_definition.enableHitEvents = true;
	shape_definition.filter.categoryBits = body->kind == V3_STATIC_BODY		 ? V3_STATIC_CATEGORY
										   : body->kind == V3_KINEMATIC_BODY ? V3_KINEMATIC_CATEGORY
																			 : V3_DYNAMIC_CATEGORY;
	shape_definition.filter.maskBits = ( body->flags & V3_BODY_DISABLE_COLLISION ) != 0 ? 0u : V3_ALL_BODY_CATEGORIES;
	// A streamed BlockGrid can appear under a body that is already resting, so pair it on creation.
	shape_definition.invokeContactCreation = true;

	// The shape takes a reference of its own, so the caller keeps the one its handle owns.
	b3ShapeId shape_id = v3CreateBlockGridShape( body_id, &shape_definition, grid->data );
	if ( B3_IS_NULL( shape_id ) )
	{
		b3DestroyBody( body_id );
		return V3_NATIVE_FAILURE;
	}

	b3MassData mass_data = { 0 };
	if ( mass_override != NULL )
	{
		mass_data = (b3MassData){
			.mass = mass_override->mass,
			.center = { mass_override->center_x, mass_override->center_y, mass_override->center_z },
			.inertia =
				{
					.cx = { mass_override->inertia_xx, mass_override->inertia_xy, mass_override->inertia_xz },
					.cy = { mass_override->inertia_xy, mass_override->inertia_yy, mass_override->inertia_yz },
					.cz = { mass_override->inertia_xz, mass_override->inertia_yz, mass_override->inertia_zz },
				},
		};
		b3Body_SetMassData( body_id, mass_data );
	}

	v3_body_entry created = {
		.logical_id = body->handle.logical_id,
		.generation = body->handle.generation,
		.body_id = body_id,
		.shape_id = shape_id,
		.mass_data = mass_data,
		.kind = body->kind,
		.has_explicit_mass_data = mass_override != NULL,
		.is_active = true,
	};

	v3_geometry_publish_body_internal( world, &created );
	world->active_body_count += 1u;
	world->mutation_batch_count = v3_geometry_saturating_add_internal( world->mutation_batch_count, 1 );
	world->created_body_count = v3_geometry_saturating_add_internal( world->created_body_count, 1 );
	*out = body->handle;
	return V3_OK;
}

v3_status v3_world_replace_block_grid_internal( v3_world* world, const v3_body_handle* body, v3_cooked_grid* grid )
{
	if ( body->logical_id == 0 || body->generation == 0 || body->generation > INT32_MAX || body->reserved != 0 )
	{
		return V3_INVALID_ARGUMENT;
	}

	int index = v3_geometry_find_body_entry_internal( world, body->logical_id );
	if ( index < 0 || !world->body_entries[index].is_active || world->body_entries[index].generation != body->generation )
	{
		return V3_STALE_HANDLE;
	}
	if ( grid->data == NULL )
	{
		return V3_INVALID_BLOCK_GRID;
	}

	const v3_body_entry* entry = world->body_entries + index;
	// Native publication validates the shape and world before reserving storage. Keeping the
	// override flag here also preserves the cached mass data used by transform readback.
	v3BlockGridReplaceStatus status = v3ReplaceBlockGridShape( entry->shape_id, grid->data, !entry->has_explicit_mass_data );
	switch ( status )
	{
		case v3_blockGridReplaceOk:
			world->mutation_batch_count = v3_geometry_saturating_add_internal( world->mutation_batch_count, 1 );
			return V3_OK;
		case v3_blockGridReplaceInvalidShape:
			return V3_STALE_HANDLE;
		case v3_blockGridReplaceWrongShapeType:
		case v3_blockGridReplaceInvalidData:
			return V3_INVALID_BLOCK_GRID;
		case v3_blockGridReplaceOutOfMemory:
			return V3_OUT_OF_MEMORY;
		case v3_blockGridReplaceWorldLocked:
			return V3_NATIVE_FAILURE;
	}
	return V3_NATIVE_FAILURE;
}

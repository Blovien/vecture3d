// SPDX-License-Identifier: MIT

#include "internal.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define V3_TEST_DEFAULT_FRICTION 0.7f

#define ENSURE( condition )                                                                                                      \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( !( condition ) )                                                                                                    \
		{                                                                                                                        \
			fprintf( stderr, "%s:%d: assertion failed: %s\n", __FILE__, __LINE__, #condition );                                  \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

static v3_box_body_command make_static_box( uint64_t logical_id, double position_x, double position_y, double position_z )
{
	return (v3_box_body_command){
		.logical_id = logical_id,
		.kind = V3_STATIC_BODY,
		.generation = 1,
		.position_x = position_x,
		.position_y = position_y,
		.position_z = position_z,
		.rotation_w = 1.0f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
		.friction = V3_TEST_DEFAULT_FRICTION,
	};
}

static v3_box_body_command make_dynamic_box( uint64_t logical_id, double position_x, double position_y, double position_z )
{
	v3_box_body_command command = make_static_box( logical_id, position_x, position_y, position_z );
	command.kind = V3_DYNAMIC_BODY;
	command.density = 1.0f;
	command.flags = V3_BODY_INITIAL_AWAKE;
	return command;
}

static v3_body_entry* find_entry( v3_world* world, uint64_t logical_id )
{
	for ( uint32_t index = 0; index < world->body_entry_count; ++index )
	{
		if ( world->body_entries[index].logical_id == logical_id )
		{
			return world->body_entries + index;
		}
	}

	return NULL;
}

#define V3_TEST_TERRAIN_HALF_SPAN 8
#define V3_TEST_TERRAIN_CELLS ( 16 * 16 )
#define V3_TEST_HULL_SPAN 6
#define V3_TEST_HULL_HEIGHT 4
#define V3_TEST_HULL_CELLS ( V3_TEST_HULL_SPAN * V3_TEST_HULL_HEIGHT * V3_TEST_HULL_SPAN )

static const v3_block_material v3_test_block_material = {
	.material_id = UINT64_C( 4242 ),
	.density = 1.0f,
	.friction = 0.5f,
	.restitution = 0.0f,
	// Reserved for fracture and ignored by the cook.
	.bond_strength = 12.0f,
	.compressive_factor = 3.0f,
	.flags = 0,
};

// One full cube in the cell that owner names. Cell-local bounds run from zero to one on each axis.
static v3_block_box make_full_cube_box( uint32_t owner, uint64_t feature_id )
{
	return (v3_block_box){
		.center_x = 0.5f,
		.center_y = 0.5f,
		.center_z = 0.5f,
		.half_extent_x = 0.5f,
		.half_extent_y = 0.5f,
		.half_extent_z = 0.5f,
		.owner_cell_index = owner,
		.material_index = 0,
		.feature_id = feature_id,
	};
}

static uint32_t build_flat_terrain( v3_block_cell* cells, v3_block_box* boxes )
{
	uint32_t count = 0;
	for ( int x = -V3_TEST_TERRAIN_HALF_SPAN; x < V3_TEST_TERRAIN_HALF_SPAN; ++x )
	{
		for ( int z = -V3_TEST_TERRAIN_HALF_SPAN; z < V3_TEST_TERRAIN_HALF_SPAN; ++z )
		{
			cells[count] = (v3_block_cell){ .x = x, .y = 0, .z = z, .material_index = 0, .feature_id = count + 1u };
			boxes[count] = make_full_cube_box( count, count + 1u );
			count += 1u;
		}
	}
	return count;
}

// A hollow shell: every cell on a face of the 6x4x6 box is filled and the interior is empty.
static uint32_t build_hollow_hull( v3_block_cell* cells, v3_block_box* boxes, bool reverse_box_order )
{
	uint32_t count = 0;
	for ( int x = 0; x < V3_TEST_HULL_SPAN; ++x )
	{
		for ( int y = 0; y < V3_TEST_HULL_HEIGHT; ++y )
		{
			for ( int z = 0; z < V3_TEST_HULL_SPAN; ++z )
			{
				bool shell = x == 0 || x == V3_TEST_HULL_SPAN - 1 || y == 0 || y == V3_TEST_HULL_HEIGHT - 1 || z == 0 ||
							 z == V3_TEST_HULL_SPAN - 1;
				if ( shell == false )
				{
					continue;
				}
				cells[count] = (v3_block_cell){
					.x = x,
					.y = y,
					.z = z,
					.material_index = 0,
					.feature_id = UINT64_C( 900000 ) + count,
				};
				count += 1u;
			}
		}
	}

	// Boxes name their owning cell, so a reversed array must cook to the same geometry.
	for ( uint32_t index = 0; index < count; ++index )
	{
		uint32_t owner = reverse_box_order ? count - 1u - index : index;
		boxes[index] = make_full_cube_box( owner, cells[owner].feature_id );
	}
	return count;
}

static v3_body_definition make_block_grid_body( uint64_t logical_id, uint32_t kind, double x, double y, double z )
{
	return (v3_body_definition){
		.handle = { .logical_id = logical_id, .generation = 1 },
		.kind = kind,
		.flags = kind == V3_DYNAMIC_BODY ? V3_BODY_INITIAL_AWAKE : 0u,
		.position_x = x,
		.position_y = y,
		.position_z = z,
		.rotation_w = 1.0f,
	};
}

static int test_cook_rejects_bad_input_without_publishing_a_handle( void )
{
	v3_block_material material = v3_test_block_material;
	v3_block_cell cells[2] = {
		{ .x = 0, .y = 0, .z = 0, .material_index = 0, .feature_id = 1 },
		{ .x = 1, .y = 0, .z = 0, .material_index = 0, .feature_id = 2 },
	};
	v3_block_box boxes[2] = { make_full_cube_box( 0, 1 ), make_full_cube_box( 1, 2 ) };

	ENSURE( v3_cook_block_grid( &material, 1, cells, 2, boxes, 2, NULL ) == V3_INVALID_ARGUMENT );

	v3_cooked_grid* grid = (v3_cooked_grid*)&material;
	ENSURE( v3_cook_block_grid( NULL, 1, cells, 2, boxes, 2, &grid ) == V3_INVALID_ARGUMENT && grid == NULL );

	grid = (v3_cooked_grid*)&material;
	ENSURE( v3_cook_block_grid( &material, 0, cells, 2, boxes, 2, &grid ) == V3_INVALID_ARGUMENT && grid == NULL );

	v3_block_box out_of_range = boxes[1];
	out_of_range.owner_cell_index = 2;
	v3_block_box owner_boxes[2] = { boxes[0], out_of_range };
	ENSURE( v3_cook_block_grid( &material, 1, cells, 2, owner_boxes, 2, &grid ) == V3_INVALID_OWNER && grid == NULL );

	v3_block_box bad_material = boxes[1];
	bad_material.material_index = 1;
	v3_block_box material_boxes[2] = { boxes[0], bad_material };
	ENSURE( v3_cook_block_grid( &material, 1, cells, 2, material_boxes, 2, &grid ) == V3_INVALID_MATERIAL && grid == NULL );

	v3_block_cell reserved_cell = cells[1];
	reserved_cell.reserved = 1;
	v3_block_cell reserved_cells[2] = { cells[0], reserved_cell };
	ENSURE( v3_cook_block_grid( &material, 1, reserved_cells, 2, boxes, 2, &grid ) == V3_INVALID_ARGUMENT && grid == NULL );

	v3_block_cell duplicate_cells[2] = { cells[0], cells[0] };
	ENSURE( v3_cook_block_grid( &material, 1, duplicate_cells, 2, boxes, 2, &grid ) == V3_DUPLICATE_CELL && grid == NULL );

	// The bounds of a box must stay inside the unit cube of the cell that owns it.
	v3_block_box oversized = boxes[1];
	oversized.half_extent_y = 0.75f;
	v3_block_box oversized_boxes[2] = { boxes[0], oversized };
	ENSURE( v3_cook_block_grid( &material, 1, cells, 2, oversized_boxes, 2, &grid ) == V3_INVALID_DIMENSION && grid == NULL );

	v3_block_material bad_friction = material;
	bad_friction.friction = 2.0f;
	ENSURE( v3_cook_block_grid( &bad_friction, 1, cells, 2, boxes, 2, &grid ) == V3_INVALID_MATERIAL && grid == NULL );

	// A cell with no box of its own leaves the cooker with an empty block.
	v3_block_box single_owner[2] = { make_full_cube_box( 0, 1 ), make_full_cube_box( 0, 2 ) };
	ENSURE( v3_cook_block_grid( &material, 1, cells, 2, single_owner, 2, &grid ) == V3_INVALID_CELL && grid == NULL );

	// NULL is accepted so a caller can release unconditionally.
	v3_destroy_cooked_grid( NULL );
	return 0;
}

static int test_bullet_flag_reaches_every_dynamic_creation_path( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );

	v3_box_body_command box = make_dynamic_box( UINT64_C( 4501 ), 0.0, 0.0, 0.0 );
	box.flags |= V3_BODY_BULLET;
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &box, 1 ) == V3_OK );

	float hull_points[] = {
		-0.5f, -0.5f, -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, 0.5f, -0.5f, -0.5f, -0.5f, 0.5f,
	};
	v3_box_body_command hull = make_dynamic_box( UINT64_C( 4502 ), 3.0, 0.0, 0.0 );
	hull.flags |= V3_BODY_BULLET;
	ENSURE( v3_world_create_hull_body( world, &hull, hull_points, 4 ) == V3_OK );

	v3_block_cell grid_cell = { .x = 0, .y = 0, .z = 0, .feature_id = UINT64_C( 4504 ) };
	v3_block_box grid_box = make_full_cube_box( 0, UINT64_C( 4504 ) );
	v3_cooked_grid* grid = NULL;
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, &grid_cell, 1, &grid_box, 1, &grid ) == V3_OK );
	v3_body_definition grid_body = make_block_grid_body( UINT64_C( 4504 ), V3_DYNAMIC_BODY, 9.0, 0.0, 0.0 );
	grid_body.flags |= V3_BODY_BULLET;
	v3_body_handle grid_handle = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &grid_body, grid, NULL, &grid_handle ) == V3_OK );
	v3_destroy_cooked_grid( grid );

	v3_body_definition sphere_body = make_block_grid_body( UINT64_C( 4505 ), V3_DYNAMIC_BODY, 12.0, 0.0, 0.0 );
	sphere_body.flags |= V3_BODY_BULLET;
	v3_body_handle sphere_handle = { 0 };
	ENSURE( v3_world_create_sphere_body( world, &sphere_body, 0.5f, 1.0f, 0.3f, &sphere_handle ) == V3_OK );

	v3_body_entry* box_entry = find_entry( world, box.logical_id );
	v3_body_entry* hull_entry = find_entry( world, hull.logical_id );
	v3_body_entry* grid_entry = find_entry( world, grid_body.handle.logical_id );
	v3_body_entry* sphere_entry = find_entry( world, sphere_body.handle.logical_id );
	ENSURE( box_entry != NULL && box_entry->is_active && b3Body_IsBullet( box_entry->body_id ) );
	ENSURE( hull_entry != NULL && hull_entry->is_active && b3Body_IsBullet( hull_entry->body_id ) );
	ENSURE( grid_entry != NULL && grid_entry->is_active && b3Body_IsBullet( grid_entry->body_id ) );
	ENSURE( sphere_entry != NULL && sphere_entry->is_active && b3Body_IsBullet( sphere_entry->body_id ) );
	ENSURE( sphere_handle.logical_id == sphere_body.handle.logical_id && sphere_handle.generation == 1 );
	ENSURE( b3Shape_GetType( sphere_entry->shape_id ) == b3_sphereShape );
	ENSURE( b3Shape_AreContactEventsEnabled( sphere_entry->shape_id ) );
	ENSURE( b3Shape_AreHitEventsEnabled( sphere_entry->shape_id ) );

	v3_world_destroy( world );
	return 0;
}

static int test_sphere_lifecycle_validation_and_failure_atomicity( void )
{
	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );

	v3_body_definition body = make_block_grid_body( UINT64_C( 4601 ), V3_DYNAMIC_BODY, 0.0, 0.0, 0.0 );
	body.flags |= V3_BODY_BULLET;
	v3_body_handle untouched = { .logical_id = UINT64_C( 99 ), .generation = 77, .reserved = 88 };
	v3_body_handle output = untouched;

	ENSURE( v3_world_create_sphere_body( world, &body, 0.0f, 1.0f, 0.3f, &output ) == V3_INVALID_DIMENSION );
	ENSURE( memcmp( &output, &untouched, sizeof( output ) ) == 0 );
	ENSURE( world->active_body_count == 0 && world->created_body_count == 0 &&
			b3World_GetCounters( world->world_id ).bodyCount == 0 );

	body.handle.reserved = 1;
	output = untouched;
	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 1.0f, 0.3f, &output ) == V3_INVALID_ARGUMENT );
	ENSURE( memcmp( &output, &untouched, sizeof( output ) ) == 0 );
	body.handle.reserved = 0;

	body.kind = V3_STATIC_BODY;
	output = untouched;
	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 0.0f, 0.3f, &output ) == V3_INVALID_ARGUMENT );
	ENSURE( memcmp( &output, &untouched, sizeof( output ) ) == 0 );
	body.kind = V3_KINEMATIC_BODY;
	output = untouched;
	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 0.0f, 0.3f, &output ) == V3_INVALID_ARGUMENT );
	ENSURE( memcmp( &output, &untouched, sizeof( output ) ) == 0 );
	body.kind = V3_DYNAMIC_BODY;
	body.flags |= UINT32_C( 1 ) << 31u;
	output = untouched;
	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 1.0f, 0.3f, &output ) == V3_INVALID_ARGUMENT );
	ENSURE( memcmp( &output, &untouched, sizeof( output ) ) == 0 );
	body.flags &= ~( UINT32_C( 1 ) << 31u );

	v3_test_fail_after( V3_TEST_FAULT_CREATE_BODY, 0 );
	output = untouched;
	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 1.0f, 0.3f, &output ) == V3_NATIVE_FAILURE );
	ENSURE( memcmp( &output, &untouched, sizeof( output ) ) == 0 );
	ENSURE( world->body_entry_count == 0 && world->active_body_count == 0 && world->mutation_batch_count == 0 );
	ENSURE( world->created_body_count == 0 && world->destroyed_body_count == 0 );
	ENSURE( b3World_GetCounters( world->world_id ).bodyCount == 0 );

	v3_test_fail_after( V3_TEST_FAULT_CREATE_SHAPE, 0 );
	output = untouched;
	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 1.0f, 0.3f, &output ) == V3_NATIVE_FAILURE );
	ENSURE( memcmp( &output, &untouched, sizeof( output ) ) == 0 );
	ENSURE( world->body_entry_count == 0 && world->active_body_count == 0 && world->mutation_batch_count == 0 );
	ENSURE( world->created_body_count == 0 && world->destroyed_body_count == 0 );
	ENSURE( b3World_GetCounters( world->world_id ).bodyCount == 0 );

	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 1.0f, 0.3f, &output ) == V3_OK );
	ENSURE( output.logical_id == body.handle.logical_id && output.generation == body.handle.generation && output.reserved == 0 );
	v3_body_entry* entry = find_entry( world, body.handle.logical_id );
	ENSURE( entry != NULL && entry->is_active );
	ENSURE( b3Shape_GetType( entry->shape_id ) == b3_sphereShape );
	ENSURE( b3Shape_GetSphere( entry->shape_id ).radius == 0.5f );
	ENSURE( b3Body_IsBullet( entry->body_id ) );
	ENSURE( world->active_body_count == 1 && world->created_body_count == 1 );

	v3_body_handle removal = output;
	ENSURE( v3_world_replace_box_bodies( world, &removal, 1, NULL, 0 ) == V3_OK );
	ENSURE( world->active_body_count == 0 && world->destroyed_body_count == 1 );

	body.handle.generation = 2;
	output = untouched;
	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 1.0f, 0.3f, &output ) == V3_OK );
	ENSURE( output.logical_id == body.handle.logical_id && output.generation == 2 && output.reserved == 0 );

	output = untouched;
	ENSURE( v3_world_create_sphere_body( world, &body, 0.5f, 1.0f, 0.3f, &output ) == V3_DUPLICATE_ID );
	ENSURE( memcmp( &output, &untouched, sizeof( output ) ) == 0 );
	ENSURE( world->active_body_count == 1 && world->created_body_count == 2 && world->destroyed_body_count == 1 );

	v3_world_destroy( world );
	return 0;
}

static int test_block_grid_hull_lands_on_a_block_grid_terrain( void )
{
	static v3_block_cell terrain_cells[V3_TEST_TERRAIN_CELLS];
	static v3_block_box terrain_boxes[V3_TEST_TERRAIN_CELLS];
	static v3_block_cell hull_cells[V3_TEST_HULL_CELLS];
	static v3_block_box hull_boxes[V3_TEST_HULL_CELLS];

	uint32_t terrain_count = build_flat_terrain( terrain_cells, terrain_boxes );
	uint32_t hull_count = build_hollow_hull( hull_cells, hull_boxes, true );
	ENSURE( terrain_count == V3_TEST_TERRAIN_CELLS );
	ENSURE( hull_count == 112 );

	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	ENSURE( world != NULL );

	v3_cooked_grid* terrain = NULL;
	v3_cooked_grid* hull = NULL;
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, terrain_cells, terrain_count, terrain_boxes, terrain_count,
								&terrain ) == V3_OK );
	ENSURE( terrain != NULL );
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, hull_cells, hull_count, hull_boxes, hull_count, &hull ) == V3_OK );
	ENSURE( hull != NULL );

	v3_body_definition terrain_body = make_block_grid_body( UINT64_C( 5001 ), V3_STATIC_BODY, 0.0, -1.0, 0.0 );
	v3_body_definition hull_body = make_block_grid_body( UINT64_C( 5002 ), V3_DYNAMIC_BODY, -3.0, 3.0, -3.0 );
	v3_body_handle terrain_handle = { 0 };
	v3_body_handle hull_handle = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &terrain_body, terrain, NULL, &terrain_handle ) == V3_OK );
	ENSURE( v3_world_attach_block_grid( world, &hull_body, hull, NULL, &hull_handle ) == V3_OK );
	ENSURE( terrain_handle.logical_id == UINT64_C( 5001 ) && terrain_handle.generation == 1 );
	ENSURE( hull_handle.logical_id == UINT64_C( 5002 ) && hull_handle.generation == 1 );

	// The shapes hold their own references, so the cooking handles are free immediately.
	v3_destroy_cooked_grid( terrain );
	v3_destroy_cooked_grid( hull );

	v3_transform transforms[2];
	v3_step_stats stats = { 0 };
	uint64_t total_touching = 0;
	uint64_t total_contacts = 0;
	double resting_y = 0.0;
	for ( int frame = 0; frame < 60; ++frame )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 4, transforms, 2, NULL, 0, &stats ) == V3_OK );
		total_touching += stats.block_grid_touching_pair_count;
		total_contacts += stats.block_grid_contact_count;
		resting_y = transforms[0].position_y;
	}

	ENSURE( stats.output_count == 1 );
	ENSURE( total_touching > 0 && total_contacts > 0 );
	ENSURE( stats.block_grid_touching_pair_count > 0 && stats.block_grid_contact_count > 0 );
	// This fixture performs no projectile sweeps or grid replacements, so these counts are zero.
	ENSURE( stats.block_grid_projectile_sweep_count == 0 && stats.block_grid_cap_exhaustion_count == 0 );
	ENSURE( stats.block_grid_replacement_published_count == 0 );
	ENSURE( resting_y > -0.05 && resting_y < 0.05 );

	v3_body_entry* hull_entry = find_entry( world, UINT64_C( 5002 ) );
	ENSURE( hull_entry != NULL && hull_entry->is_active );
	ENSURE( b3Shape_GetType( hull_entry->shape_id ) == v3_blockGridShape );

	// The handle is an ordinary body handle, so the ordinary removal path retires it.
	ENSURE( v3_world_replace_box_bodies( world, &hull_handle, 1, NULL, 0 ) == V3_OK );
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, transforms, 2, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.output_count == 0 );

	v3_world_destroy( world );
	return 0;
}

static int test_attached_block_grid_takes_the_mass_override( void )
{
	static v3_block_cell hull_cells[V3_TEST_HULL_CELLS];
	static v3_block_box hull_boxes[V3_TEST_HULL_CELLS];
	uint32_t hull_count = build_hollow_hull( hull_cells, hull_boxes, false );

	v3_world* world = v3_world_create( 0.0, 0.0, 0.0, NULL );
	ENSURE( world != NULL );

	v3_cooked_grid* hull = NULL;
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, hull_cells, hull_count, hull_boxes, hull_count, &hull ) == V3_OK );

	v3_body_definition body = make_block_grid_body( UINT64_C( 6001 ), V3_DYNAMIC_BODY, 4.0, 5.0, 6.0 );
	body.linear_velocity_x = 2.0f;
	body.angular_velocity_z = 3.0f;
	v3_mass_properties mass = {
		.mass = 250.0f,
		.center_x = 0.25f,
		.center_y = 0.5f,
		.center_z = -0.25f,
		.inertia_xx = 400.0f,
		.inertia_yy = 500.0f,
		.inertia_zz = 600.0f,
	};
	v3_body_handle handle = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &body, hull, &mass, &handle ) == V3_OK );

	v3_body_entry* entry = find_entry( world, UINT64_C( 6001 ) );
	ENSURE( entry != NULL );
	b3MassData installed = b3Body_GetMassData( entry->body_id );
	ENSURE( installed.mass == mass.mass );
	ENSURE( installed.center.x == mass.center_x && installed.center.y == mass.center_y && installed.center.z == mass.center_z );

	ENSURE( installed.inertia.cx.x == 400 && installed.inertia.cy.y == 500 && installed.inertia.cz.z == 600 );
	ENSURE( installed.inertia.cx.y == 0 && installed.inertia.cx.z == 0 && installed.inertia.cy.z == 0 );
	v3_transform transform;
	v3_step_stats stats;
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 0, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( transform.position_x == 4 && transform.position_y == 5 && transform.position_z == 6 );
	ENSURE( transform.local_center_x == 0.25f && transform.local_center_y == 0.5f && transform.local_center_z == -0.25f );
	// The input is origin velocity. COM velocity adds omega cross local center.
	ENSURE( transform.linear_velocity_x == 0.5f && transform.linear_velocity_y == 0.75f && transform.linear_velocity_z == 0 );
	ENSURE( transform.angular_velocity_z == 3 );

	// A second body may share the same cooked data, and an invalid override changes nothing.
	v3_body_definition shared = make_block_grid_body( UINT64_C( 6002 ), V3_KINEMATIC_BODY, 20.0, 0.0, 0.0 );
	v3_mass_properties invalid = mass;
	invalid.mass = -1.0f;
	v3_body_handle shared_handle = { 0 };
	ENSURE( v3_world_attach_block_grid( world, &shared, hull, &invalid, &shared_handle ) == V3_INVALID_DENSITY );
	ENSURE( v3_world_attach_block_grid( world, &shared, hull, NULL, &shared_handle ) == V3_OK );
	ENSURE( shared_handle.logical_id == UINT64_C( 6002 ) );

	// A duplicate identity is refused exactly as it is for every other body operation.
	ENSURE( v3_world_attach_block_grid( world, &shared, hull, NULL, &shared_handle ) == V3_DUPLICATE_ID );

	v3_destroy_cooked_grid( hull );
	v3_world_destroy( world );
	return 0;
}

static double simulate_hull_drop( bool reverse_box_order )
{
	static v3_block_cell terrain_cells[V3_TEST_TERRAIN_CELLS];
	static v3_block_box terrain_boxes[V3_TEST_TERRAIN_CELLS];
	static v3_block_cell hull_cells[V3_TEST_HULL_CELLS];
	static v3_block_box hull_boxes[V3_TEST_HULL_CELLS];

	uint32_t terrain_count = build_flat_terrain( terrain_cells, terrain_boxes );
	uint32_t hull_count = build_hollow_hull( hull_cells, hull_boxes, reverse_box_order );

	v3_world* world = v3_world_create( 0.0, -9.81, 0.0, NULL );
	v3_cooked_grid* terrain = NULL;
	v3_cooked_grid* hull = NULL;
	v3_cook_block_grid( &v3_test_block_material, 1, terrain_cells, terrain_count, terrain_boxes, terrain_count, &terrain );
	v3_cook_block_grid( &v3_test_block_material, 1, hull_cells, hull_count, hull_boxes, hull_count, &hull );

	v3_body_definition terrain_body = make_block_grid_body( UINT64_C( 7001 ), V3_STATIC_BODY, 0.0, -1.0, 0.0 );
	v3_body_definition hull_body = make_block_grid_body( UINT64_C( 7002 ), V3_DYNAMIC_BODY, -2.75, 3.0, -3.25 );
	v3_body_handle handle = { 0 };
	v3_world_attach_block_grid( world, &terrain_body, terrain, NULL, &handle );
	v3_world_attach_block_grid( world, &hull_body, hull, NULL, &handle );
	v3_destroy_cooked_grid( terrain );
	v3_destroy_cooked_grid( hull );

	v3_transform transforms[2];
	v3_step_stats stats = { 0 };
	for ( int frame = 0; frame < 60; ++frame )
	{
		v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 4, transforms, 2, NULL, 0, &stats );
	}
	double position_y = transforms[0].position_y;
	v3_world_destroy( world );
	return position_y;
}

static int test_box_input_order_does_not_change_the_cooked_geometry( void )
{
	// Boxes carry their owning cell, so the shim groups them and the input order is irrelevant.
	double ordered = simulate_hull_drop( false );
	double reversed = simulate_hull_drop( true );
	ENSURE( ordered == reversed );
	return 0;
}

static int test_terrain_payload_is_retained_after_caller_overwrite( void )
{
	int bytes_before = b3GetByteCount();
	v3_world* world = v3_world_create( 0.0, -32.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command dynamic = make_dynamic_box( UINT64_C( 1001 ), 0.5, 1.5, 0.5 );
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, &dynamic, 1 ) == V3_OK );

	// The old overlapping payload described this same unit cube. One Cell Box describes its union.
	v3_block_material material = v3_test_block_material;
	material.material_id = 1002;
	material.friction = V3_TEST_DEFAULT_FRICTION;
	v3_block_cell cell = { .feature_id = 7001 };
	v3_block_box box = make_full_cube_box( 0, 7001 );
	v3_cooked_grid* grid = NULL;
	ENSURE( v3_cook_block_grid( &material, 1, &cell, 1, &box, 1, &grid ) == V3_OK );
	memset( &material, 0xA5, sizeof( material ) );
	memset( &cell, 0xA5, sizeof( cell ) );
	memset( &box, 0, sizeof( box ) );
	v3_body_definition terrain = make_block_grid_body( 1002, V3_STATIC_BODY, 0, 0, 0 );
	v3_body_handle handle;
	ENSURE( v3_world_attach_block_grid( world, &terrain, grid, NULL, &handle ) == V3_OK );
	v3_destroy_cooked_grid( grid );

	v3_query query = { .query_id = 1003,
					   .kind = V3_QUERY_RAY,
					   .origin_x = 0.5,
					   .origin_y = 3,
					   .origin_z = 0.5,
					   .translation_y = -4,
					   .category_bits = V3_ALL_BODY_CATEGORIES,
					   .mask_bits = V3_STATIC_CATEGORY };
	v3_query_result result;
	v3_transform transform;
	v3_step_stats stats;
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, &query, 1, 1, &transform, 1, &result, 1, &stats ) == V3_OK );
	ENSURE( result.status == V3_QUERY_CAST_HIT && result.hit_logical_id == 1002 );
	ENSURE( result.user_material_id == 1002 && result.user_data == 7001 );
	ENSURE( result.cell_x == 0 && result.cell_y == 0 && result.cell_z == 0 );
	ENSURE( fabsf( result.fraction - 0.5f ) < 0.001f );
	v3_block_contact_event events[16];
	uint32_t count;
	ENSURE( v3_world_get_block_contact_events( world, events, 16, &count ) == V3_OK );
	bool contact = false;
	for ( uint32_t i = 0; i < count; ++i )
	{
		const v3_block_contact_event* event = events + i;
		const v3_block_contact_side* side = event->body_a.logical_id == 1002 ? &event->side_a : &event->side_b;
		if ( event->flags == V3_BLOCK_CONTACT_BEGIN && ( event->body_a.logical_id == 1002 || event->body_b.logical_id == 1002 ) )
		{
			ENSURE( side->user_material_id == 1002 && side->user_data == 7001 );
			contact = true;
		}
	}
	ENSURE( contact );
	// Reusing the logical entry must release the shape's retained cook reference exactly once.
	v3_box_body_command replacement = make_static_box( 1002, 5, 0, 0 );
	replacement.generation = 2;
	ENSURE( v3_world_replace_box_bodies( world, &handle, 1, &replacement, 1 ) == V3_OK );
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, &query, 1, 0, &transform, 1, &result, 1, &stats ) == V3_OK );
	ENSURE( result.status == V3_QUERY_NO_HIT );
	v3_world_destroy( world );
	ENSURE( b3GetByteCount() == bytes_before );
	return 0;
}

static int test_streamed_block_grid_pairs_with_resting_dynamic_body( void )
{
	v3_world* world = v3_world_create( 0.0, -32.0, 0.0, NULL );
	ENSURE( world != NULL );
	v3_box_body_command bodies[2] = {
		make_static_box( UINT64_C( 1201 ), 0.0, 0.5, 0.0 ),
		make_dynamic_box( UINT64_C( 1202 ), 0.0, 4.0, 0.0 ),
	};
	ENSURE( v3_world_replace_box_bodies( world, NULL, 0, bodies, 2 ) == V3_OK );
	v3_transform transform;
	v3_step_stats stats;
	for ( int step = 0; step < 240; ++step )
	{
		ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	}
	ENSURE( fabs( transform.position_y - 1.5 ) < 0.05 );
	v3_block_cell cell = { .feature_id = 1203 };
	v3_block_box box = make_full_cube_box( 0, 1203 );
	v3_cooked_grid* grid = NULL;
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, &cell, 1, &box, 1, &grid ) == V3_OK );
	v3_body_definition terrain = make_block_grid_body( 1203, V3_STATIC_BODY, 0, 0, 0 );
	v3_body_handle handle;
	ENSURE( v3_world_attach_block_grid( world, &terrain, grid, NULL, &handle ) == V3_OK );
	v3_destroy_cooked_grid( grid );
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	v3_block_contact_event events[16];
	uint32_t count;
	ENSURE( v3_world_get_block_contact_events( world, events, 16, &count ) == V3_OK );
	bool discovered = false;
	for ( uint32_t i = 0; i < count; ++i )
	{
		const v3_block_contact_event* event = events + i;
		if ( event->flags == V3_BLOCK_CONTACT_BEGIN &&
			 ( ( event->body_a.logical_id == 1202 && event->body_b.logical_id == 1203 ) ||
			   ( event->body_a.logical_id == 1203 && event->body_b.logical_id == 1202 ) ) )
		{
			discovered = true;
		}
	}
	ENSURE( discovered );
	v3_world_destroy( world );
	return 0;
}

static int test_replace_block_grid_derives_mass_and_retains_identity( void )
{
	int bytes_before = b3GetByteCount();
	v3_world* world = v3_world_create( 0, 0, 0, NULL );
	ENSURE( world != NULL );
	v3_block_cell cells[2] = { { .feature_id = 1 }, { .x = 2, .feature_id = 2 } };
	v3_block_box boxes[2] = { make_full_cube_box( 0, 1 ), make_full_cube_box( 1, 2 ) };
	v3_block_material material = v3_test_block_material;
	material.density = 3;
	v3_cooked_grid* old = NULL;
	v3_cooked_grid* next = NULL;
	ENSURE( v3_cook_block_grid( &material, 1, cells, 1, boxes, 1, &old ) == V3_OK );
	material.density = 19;
	ENSURE( v3_cook_block_grid( &material, 1, cells, 2, boxes, 2, &next ) == V3_OK );
	v3_body_definition body = make_block_grid_body( 7201, V3_DYNAMIC_BODY, 4, 5, 6 );
	body.linear_velocity_x = 2;
	body.angular_velocity_z = 3;
	v3_body_handle handle;
	ENSURE( v3_world_attach_block_grid( world, &body, old, NULL, &handle ) == V3_OK );
	v3_body_entry before = *find_entry( world, handle.logical_id );
	b3Vec3 velocity = b3Body_GetLinearVelocity( before.body_id );
	ENSURE( b3Body_GetMass( before.body_id ) == 3 );
	ENSURE( v3_world_replace_block_grid( world, &handle, next ) == V3_OK );
	v3_body_entry after = *find_entry( world, handle.logical_id );
	ENSURE( B3_ID_EQUALS( before.body_id, after.body_id ) && B3_ID_EQUALS( before.shape_id, after.shape_id ) );
	ENSURE( after.logical_id == handle.logical_id && after.generation == handle.generation );
	b3MassData mass = b3Body_GetMassData( after.body_id );
	// Two disjoint unit cubes at density 3 have mass 6 and center (1.5, 0.5, 0.5).
	ENSURE( mass.mass == 6 && mass.center.x == 1.5f && mass.center.y == 0.5f && mass.center.z == 0.5f );
	b3Vec3 shifted = b3Body_GetLinearVelocity( after.body_id );
	ENSURE( shifted.x == velocity.x && shifted.y == velocity.y + 3 && shifted.z == velocity.z );
	ENSURE( b3Body_GetAngularVelocity( after.body_id ).z == 3 );
	b3Pos position = b3Body_GetPosition( after.body_id );
	ENSURE( position.x == 4 && position.y == 5 && position.z == 6 );
	ENSURE( world->mutation_batch_count == 2 && world->created_body_count == 1 && world->destroyed_body_count == 0 );
	ENSURE( v3_world_replace_block_grid( world, &handle, next ) == V3_OK );
	ENSURE( world->mutation_batch_count == 3 && b3Body_GetMass( after.body_id ) == 6 );
	v3_destroy_cooked_grid( next );
	v3_destroy_cooked_grid( old );
	ENSURE( b3Shape_GetAABB( after.shape_id ).upperBound.x >= 7 );
	v3_world_destroy( world );
	ENSURE( b3GetByteCount() == bytes_before );
	return 0;
}

// A vertical ray through cell 2 distinguishes the two revisions without advancing simulation.
static int check_replacement_query( v3_world* world, uint64_t logical_id, bool hit )
{
	v3_query query = { .query_id = 72,
					   .kind = V3_QUERY_RAY,
					   .origin_x = 2.5,
					   .origin_y = 3,
					   .origin_z = 0.5,
					   .translation_y = -4,
					   .category_bits = 7,
					   .mask_bits = 7 };
	v3_query_result result;
	v3_step_stats stats;
	v3_transform transform;
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, &query, 1, 0, &transform, 1, &result, 1, &stats ) == V3_OK );
	ENSURE( result.status == ( hit ? V3_QUERY_CAST_HIT : V3_QUERY_NO_HIT ) );
	if ( hit )
	{
		ENSURE( result.hit_logical_id == logical_id && result.cell_x == 2 && result.user_data == 2 );
	}
	return 0;
}

static int replacement_allocation_calls;
static void* reject_replacement_allocation( int32_t size, int32_t alignment )
{
	(void)size;
	(void)alignment;
	replacement_allocation_calls += 1;
	return NULL;
}

static int test_replace_block_grid_rejects_invalid_and_failed_publications( void )
{
	int bytes_before = b3GetByteCount();
	v3_world* world = v3_world_create( 0, 0, 0, NULL );
	ENSURE( world != NULL );
	v3_block_cell cells[2] = { { .feature_id = 1 }, { .x = 2, .feature_id = 2 } };
	v3_block_box boxes[2] = { make_full_cube_box( 0, 1 ), make_full_cube_box( 1, 2 ) };
	v3_cooked_grid* old = NULL;
	v3_cooked_grid* next = NULL;
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, cells, 1, boxes, 1, &old ) == V3_OK );
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, cells, 2, boxes, 2, &next ) == V3_OK );
	v3_body_definition body = make_block_grid_body( 7202, V3_DYNAMIC_BODY, 0, 0, 0 );
	v3_body_handle handle;
	ENSURE( v3_world_attach_block_grid( world, &body, old, NULL, &handle ) == V3_OK );
	v3_body_definition sphere = make_block_grid_body( 7203, V3_STATIC_BODY, 20, 0, 0 );
	v3_body_handle wrong_type;
	ENSURE( v3_world_create_sphere_body( world, &sphere, 1, 0, 0.5f, &wrong_type ) == V3_OK );
	v3_body_entry before = *find_entry( world, handle.logical_id );
	uint32_t mutations = world->mutation_batch_count;
	ENSURE( check_replacement_query( world, handle.logical_id, false ) == 0 );
	ENSURE( v3_world_replace_block_grid( NULL, &handle, next ) == V3_INVALID_ARGUMENT );
	ENSURE( v3_world_replace_block_grid( world, NULL, next ) == V3_INVALID_ARGUMENT );
	ENSURE( v3_world_replace_block_grid( world, &handle, NULL ) == V3_INVALID_ARGUMENT );
	v3_body_handle invalid[] = { { 0, 1, 0 }, { 7202, 0, 0 }, { 7202, UINT32_MAX, 0 }, { 7202, 1, 1 } };
	for ( unsigned i = 0; i < sizeof( invalid ) / sizeof( invalid[0] ); ++i )
	{
		ENSURE( v3_world_replace_block_grid( world, invalid + i, next ) == V3_INVALID_ARGUMENT );
	}
	v3_body_handle stale[] = { { 9999, 1, 0 }, { 7202, 2, 0 } };
	for ( unsigned i = 0; i < sizeof( stale ) / sizeof( stale[0] ); ++i )
	{
		ENSURE( v3_world_replace_block_grid( world, stale + i, next ) == V3_STALE_HANDLE );
	}
	ENSURE( v3_world_replace_block_grid( world, &wrong_type, next ) == V3_INVALID_BLOCK_GRID );

	// Use the native trying allocator, restoring it before assertions or cleanup.
	replacement_allocation_calls = 0;
	b3SetAllocator( reject_replacement_allocation, free );
	v3_status failed = v3_world_replace_block_grid( world, &handle, next );
	b3SetAllocator( NULL, NULL );
	ENSURE( failed == V3_OUT_OF_MEMORY && replacement_allocation_calls == 1 );
	ENSURE( world->mutation_batch_count == mutations );
	ENSURE( memcmp( &before, find_entry( world, handle.logical_id ), sizeof( before ) ) == 0 );
	ENSURE( b3Body_GetMass( before.body_id ) == 1 );
	ENSURE( b3Body_GetLocalCenter( before.body_id ).x == 0.5f );
	ENSURE( b3Body_GetPosition( before.body_id ).x == 0 && b3Body_GetLinearVelocity( before.body_id ).x == 0 );
	ENSURE( check_replacement_query( world, handle.logical_id, false ) == 0 );
	v3_step_stats stats;
	v3_transform transform;
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.block_grid_replacement_published_count == 0 );
	ENSURE( v3_world_replace_block_grid( world, &handle, next ) == V3_OK );
	ENSURE( check_replacement_query( world, handle.logical_id, true ) == 0 );
	ENSURE( v3_world_replace_block_grid( world, &handle, old ) == V3_OK );
	ENSURE( check_replacement_query( world, handle.logical_id, false ) == 0 );
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 1, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( stats.block_grid_replacement_published_count == 2 );
	ENSURE( v3_world_replace_box_bodies( world, &handle, 1, NULL, 0 ) == V3_OK );
	ENSURE( v3_world_replace_block_grid( world, &handle, next ) == V3_STALE_HANDLE );
	body.handle.generation = 2;
	v3_body_handle renewed;
	ENSURE( v3_world_attach_block_grid( world, &body, old, NULL, &renewed ) == V3_OK );
	ENSURE( v3_world_replace_block_grid( world, &handle, next ) == V3_STALE_HANDLE );
	ENSURE( v3_world_replace_block_grid( world, &renewed, next ) == V3_OK );
	v3_destroy_cooked_grid( old );
	v3_destroy_cooked_grid( next );
	ENSURE( check_replacement_query( world, renewed.logical_id, true ) == 0 );
	v3_world_destroy( world );
	ENSURE( b3GetByteCount() == bytes_before );
	return 0;
}

static int test_replace_block_grid_preserves_explicit_mass_and_cached_center( void )
{
	int bytes_before = b3GetByteCount();
	v3_world* world = v3_world_create( 0, 0, 0, NULL );
	ENSURE( world != NULL );
	v3_block_cell cells[2] = { { .feature_id = 1 }, { .x = 2, .feature_id = 2 } };
	v3_block_box boxes[2] = { make_full_cube_box( 0, 1 ), make_full_cube_box( 1, 2 ) };
	v3_cooked_grid* old = NULL;
	v3_cooked_grid* next = NULL;
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, cells, 1, boxes, 1, &old ) == V3_OK );
	ENSURE( v3_cook_block_grid( &v3_test_block_material, 1, cells, 2, boxes, 2, &next ) == V3_OK );
	v3_body_definition body = make_block_grid_body( 7204, V3_DYNAMIC_BODY, 0, 0, 0 );
	body.angular_velocity_z = 2;
	v3_mass_properties override = { .mass = 4,
									.center_x = 0.25f,
									.center_y = 0.5f,
									.center_z = -0.25f,
									.inertia_xx = 8,
									.inertia_yy = 9,
									.inertia_zz = 10,
									.inertia_xy = 0.5f };
	v3_body_handle handle;
	ENSURE( v3_world_attach_block_grid( world, &body, old, &override, &handle ) == V3_OK );
	v3_body_entry* entry = find_entry( world, handle.logical_id );
	b3MassData before = b3Body_GetMassData( entry->body_id );
	b3Vec3 velocity = b3Body_GetLinearVelocity( entry->body_id );
	for ( int i = 0; i < 8; ++i )
	{
		ENSURE( v3_world_replace_block_grid( world, &handle, i % 2 == 0 ? next : old ) == V3_OK );
	}
	v3_destroy_cooked_grid( old );
	v3_destroy_cooked_grid( next );
	b3MassData after = b3Body_GetMassData( entry->body_id );
	ENSURE( after.mass == before.mass && after.center.x == before.center.x && after.center.y == before.center.y &&
			after.center.z == before.center.z );
	ENSURE( memcmp( &after.inertia, &before.inertia, sizeof( before.inertia ) ) == 0 );
	ENSURE( entry->has_explicit_mass_data && memcmp( &entry->mass_data, &before, sizeof( before ) ) == 0 );
	b3Vec3 after_velocity = b3Body_GetLinearVelocity( entry->body_id );
	ENSURE( after_velocity.x == velocity.x && after_velocity.y == velocity.y && after_velocity.z == velocity.z );
	v3_transform transform;
	v3_step_stats stats;
	ENSURE( v3_world_step_and_read( world, NULL, 0, NULL, 0, NULL, 0, 0, &transform, 1, NULL, 0, &stats ) == V3_OK );
	ENSURE( transform.local_center_x == override.center_x && transform.local_center_y == override.center_y &&
			transform.local_center_z == override.center_z );
	v3_world_destroy( world );
	ENSURE( b3GetByteCount() == bytes_before );
	return 0;
}

int main( void )
{
	ENSURE( test_replace_block_grid_rejects_invalid_and_failed_publications() == 0 );
	ENSURE( test_replace_block_grid_preserves_explicit_mass_and_cached_center() == 0 );
	ENSURE( test_replace_block_grid_derives_mass_and_retains_identity() == 0 );
	ENSURE( test_bullet_flag_reaches_every_dynamic_creation_path() == 0 );
	ENSURE( test_sphere_lifecycle_validation_and_failure_atomicity() == 0 );
	ENSURE( test_terrain_payload_is_retained_after_caller_overwrite() == 0 );
	ENSURE( test_streamed_block_grid_pairs_with_resting_dynamic_body() == 0 );
	ENSURE( test_cook_rejects_bad_input_without_publishing_a_handle() == 0 );
	ENSURE( test_block_grid_hull_lands_on_a_block_grid_terrain() == 0 );
	ENSURE( test_attached_block_grid_takes_the_mass_override() == 0 );
	ENSURE( test_box_input_order_does_not_change_the_cooked_geometry() == 0 );
	puts( "v3 geometry tests passed" );
	return 0;
}

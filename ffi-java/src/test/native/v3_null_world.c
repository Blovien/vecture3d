// SPDX-License-Identifier: MIT

#include "v3_abi.h"

uint32_t v3_abi_version( void )
{
	return V3_ABI_VERSION;
}

uint64_t v3_abi_schema_hash( void )
{
	return V3_ABI_SCHEMA_HASH;
}

v3_world* v3_world_create( double gravity_x, double gravity_y, double gravity_z, const v3_world_limits* limits )
{
	(void)gravity_x;
	(void)gravity_y;
	(void)gravity_z;
	(void)limits;
	return NULL;
}

v3_status v3_world_set_limits( v3_world* world, const v3_world_limits* limits )
{
	(void)world;
	(void)limits;
	return V3_INVALID_ARGUMENT;
}

v3_status v3_world_get_limits( v3_world* world, v3_world_limits* limits )
{
	(void)world;
	(void)limits;
	return V3_INVALID_ARGUMENT;
}

v3_status v3_world_get_block_contact_events( const v3_world* world, v3_block_contact_event* events, uint32_t event_capacity,
											 uint32_t* event_count )
{
	(void)world;
	(void)events;
	(void)event_capacity;
	(void)event_count;
	return V3_INVALID_ARGUMENT;
}

v3_status v3_world_create_sphere_body( v3_world* world, const v3_body_definition* body, float radius, float density,
									   float friction, v3_body_handle* out )
{
	(void)world;
	(void)body;
	(void)radius;
	(void)density;
	(void)friction;
	(void)out;
	return V3_INVALID_ARGUMENT;
}

uint32_t v3_active_world_count( void )
{
	return 0;
}

v3_status v3_world_replace_block_grid( v3_world* world, const v3_body_handle* body, v3_cooked_grid* grid )
{
	(void)world;
	(void)body;
	(void)grid;
	return V3_INVALID_ARGUMENT;
}

v3_status v3_world_replace_revolute_joints( v3_world* world, const v3_joint_handle* removals, uint32_t removal_count,
											const v3_revolute_joint_command* creations, uint32_t creation_count )
{
	(void)world;
	(void)removals;
	(void)removal_count;
	(void)creations;
	(void)creation_count;
	return V3_INVALID_ARGUMENT;
}

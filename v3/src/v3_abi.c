// SPDX-License-Identifier: MIT

#include "v3_internal.h"

#include <float.h>
#include <math.h>

uint32_t v3_abi_version( void )
{
	return V3_ABI_VERSION;
}

uint64_t v3_abi_schema_hash( void )
{
	return V3_ABI_SCHEMA_HASH;
}

v3_world* v3_world_create( double gravity_x, double gravity_y, double gravity_z )
{
	if ( !isfinite( gravity_x ) || !isfinite( gravity_y ) || !isfinite( gravity_z ) || fabs( gravity_x ) > FLT_MAX ||
		 fabs( gravity_y ) > FLT_MAX || fabs( gravity_z ) > FLT_MAX )
	{
		return NULL;
	}

	return v3_world_create_internal( gravity_x, gravity_y, gravity_z );
}

v3_status v3_world_replace_box_bodies( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
									   const v3_box_body_command* creations, uint32_t creation_count )
{
	if ( world == NULL || ( removal_count > 0 && removals == NULL ) || ( creation_count > 0 && creations == NULL ) )
	{
		return V3_INVALID_ARGUMENT;
	}

	if ( removal_count > V3_MAX_BODIES_PER_BATCH || creation_count > V3_MAX_BODIES_PER_BATCH )
	{
		return V3_LIMIT_EXCEEDED;
	}

	return v3_world_replace_box_bodies_internal( world, removals, removal_count, creations, creation_count );
}

void v3_world_destroy( v3_world* world )
{
	v3_world_destroy_internal( world );
}

uint32_t v3_active_world_count( void )
{
	return v3_active_world_count_internal();
}

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

v3_world* v3_world_create( double gravity_x, double gravity_y, double gravity_z )
{
	(void)gravity_x;
	(void)gravity_y;
	(void)gravity_z;
	return NULL;
}

uint32_t v3_active_world_count( void )
{
	return 0;
}

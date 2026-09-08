// SPDX-License-Identifier: MIT

#include <stdint.h>

uint32_t v3_abi_version( void )
{
	return 1;
}

uint64_t v3_abi_schema_hash( void )
{
	return UINT64_C( 0 );
}

uint32_t v3_active_world_count( void )
{
	return 0;
}

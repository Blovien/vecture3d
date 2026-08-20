// SPDX-License-Identifier: MIT

#ifndef V3_INTERNAL_H
#define V3_INTERNAL_H

#include "v3_abi.h"

#include <box3d/box3d.h>
#include <stdbool.h>
#include <stdint.h>

#define V3_MAX_BODIES_PER_BATCH UINT32_C( 4096 )
#define V3_MAX_LOGICAL_BODY_IDS UINT32_C( 4096 )
#define V3_MAX_DAMPING 10.0f

#define V3_STATIC_CATEGORY ( UINT64_C( 1 ) << 0u )
#define V3_KINEMATIC_CATEGORY ( UINT64_C( 1 ) << 1u )
#define V3_DYNAMIC_CATEGORY ( UINT64_C( 1 ) << 2u )
#define V3_ALL_BODY_CATEGORIES ( V3_STATIC_CATEGORY | V3_KINEMATIC_CATEGORY | V3_DYNAMIC_CATEGORY )

typedef struct v3_body_entry
{
	uint64_t logical_id;
	uint32_t generation;
	b3BodyId body_id;
	b3ShapeId shape_id;
	uint32_t kind;
	bool is_active;
} v3_body_entry;

struct v3_world
{
	b3WorldId world_id;
	v3_body_entry* body_entries;
	uint32_t body_entry_count;
	uint32_t body_entry_capacity;
	uint32_t active_body_count;
	uint32_t mutation_batch_count;
	uint32_t created_body_count;
	uint32_t destroyed_body_count;
};

v3_world* v3_world_create_internal( double gravity_x, double gravity_y, double gravity_z );
v3_status v3_world_replace_box_bodies_internal( v3_world* world, const v3_body_handle* removals, uint32_t removal_count,
												const v3_box_body_command* creations, uint32_t creation_count );
void v3_world_destroy_internal( v3_world* world );
uint32_t v3_active_world_count_internal( void );

typedef enum v3_test_fault
{
	V3_TEST_FAULT_NONE,
	V3_TEST_FAULT_WORLD_CALLOC,
	V3_TEST_FAULT_BODY_ENTRIES_REALLOC,
	V3_TEST_FAULT_PENDING_CALLOC,
	V3_TEST_FAULT_CREATE_BODY,
	V3_TEST_FAULT_CREATE_SHAPE,
} v3_test_fault;

#if defined( V3_TESTING )
void v3_test_fail_after( v3_test_fault fault, uint32_t successful_calls_before_failure );
#endif

#endif

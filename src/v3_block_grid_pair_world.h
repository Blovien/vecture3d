// SPDX-License-Identifier: MIT

#pragma once

#include "arena_allocator.h"
#include "v3_block_grid_pair_update.h"

typedef struct b3World b3World;

// Rebuilds every overlapping awake BlockGrid pair contact, one pair at a time and independently:
// a pair that cannot build geometry ends the step separated and the rest are unaffected
// The arena is owned by the caller and may be reused after its surrounding collision pass
void v3BlockGridPairUpdateAwakeContacts( b3World* world, b3Arena arena, float timeStep, int subStepCount );

// Recompute broad admission from this step before discovering or retaining pair contacts.
void v3BlockGridPairPrepareStep( b3World* world, float timeStep, int subStepCount );

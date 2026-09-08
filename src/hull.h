// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

#include <stddef.h>

uint64_t b3HashHullData( const b3HullData* hull );
bool b3CompareHullData( const b3HullData* hull1, const b3HullData* hull2 );

// All box hull paths use the same lower bound for thin axes
static inline b3Vec3 b3ClampBoxHalfExtent( b3Vec3 halfExtent )
{
	float minimum = 0.2f * B3_LINEAR_SLOP;
	return b3Max( (b3Vec3){ minimum, minimum, minimum }, halfExtent );
}

// Map keyed by hull content. The world hull database uses the value as a reference count,
// while compound baking stores uses the value as a byte offset. Implementation is in hull.c.
#define NAME b3HullMap
#define KEY_TY const b3HullData*
#define VAL_TY int
#define HEADER_MODE
#include "verstable.h"

// Total map allocation in bytes, excluding the stored hull data
size_t b3HullMapByteCount( b3HullMap* map );

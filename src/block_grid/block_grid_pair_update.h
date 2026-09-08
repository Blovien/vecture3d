// SPDX-License-Identifier: MIT

#pragma once

#include "block_grid_contact.h"
#include "block_grid_pair.h"

typedef struct b3Shape b3Shape;
typedef struct b3World b3World;

typedef struct v3BlockGridPairUpdateScratch
{
	// Query buffers must be clean at entry and remain clean on every outcome
	v3BlockGridPairTraversalScratch traversal;
	void* memory;
	size_t byteCapacity;
} v3BlockGridPairUpdateScratch;

typedef struct v3BlockGridPairUpdateInput
{
	b3Contact* contact;
	const b3Shape* shapeA;
	const b3Shape* shapeB;
	uint64_t sourceEpochA;
	uint64_t sourceEpochB;
	b3Vec3 localCenterA;
	b3Vec3 localCenterB;
	b3WorldTransform transformA;
	b3WorldTransform transformB;
	float admissionDistance;
} v3BlockGridPairUpdateInput;

// What one updated pair contributed to the step counters
typedef struct v3BlockGridPairUpdateResult
{
	uint64_t candidateHitboxPairCount;
	uint64_t touchingPairCount;
	uint64_t reductionCount;
} v3BlockGridPairUpdateResult;

// Covers reducer scratch, current and previous reduced patches, the retained impulses lifted out of
// the pair storage before it is overwritten, point match candidates, and the previous SAT cache lookup
// Query bitmap storage is owned separately by v3BlockGridPairUpdateScratch
size_t v3BlockGridPairUpdateScratchByteCount( void );

// Rebuilds the pair's manifolds in its contact. If no geometry can be produced,
// the contact ends the step separated with no manifolds. Other contacts are unchanged.
v3BlockGridPairUpdateResult v3BlockGridPairUpdateContact( b3World* world, const v3BlockGridPairUpdateInput* input,
														  v3BlockGridPairUpdateScratch* scratch );

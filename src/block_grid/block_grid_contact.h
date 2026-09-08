// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct b3Contact b3Contact;
typedef struct b3ContactMaterial b3ContactMaterial;
typedef struct b3Shape b3Shape;
typedef struct b3World b3World;
typedef struct v3BlockGridPairState v3BlockGridPairState;

enum
{
	v3_blockGridPairChildIndex = B3_MAX_CHILD_SHAPES - 1,
};

static_assert( v3_blockGridPairChildIndex > UINT16_MAX, "BlockGrid pair marker overlaps a hitbox index" );

// Unset marks a contact not yet updated this step. An update that cannot build
// manifolds leaves the contact separated.
typedef enum v3BlockGridPairOutcome
{
	v3_blockGridPairOutcomeUnset,
	v3_blockGridPairOutcomeSeparated,
	v3_blockGridPairOutcomeTouching,
} v3BlockGridPairOutcome;

typedef struct v3BlockGridPairPatch
{
	// One packed A-B hitbox pair for each point in the matching manifold, the way a mesh
	// manifold point names its triangle: this is where the point's surface materials come from
	uint32_t hitboxPairKeys[4];

	uint32_t supportRegionKey;
	uint8_t directedSatKey;
	uint8_t reserved;
} v3BlockGridPairPatch;

typedef struct v3BlockGridPairControl
{
	// This pointer owns the allocation, while the contact manifold pointer only views its manifold span
	v3BlockGridPairState* state;
	uint64_t sourceEpochA;
	uint64_t sourceEpochB;
	uint16_t correlationPolicy;
	uint8_t lastOutcome;
	uint8_t reserved;
} v3BlockGridPairControl;

// One retained manifold's warm-start impulses, lifted into caller scratch before the update
// overwrites the pair's storage in place
typedef struct v3BlockGridPairRetainedPatch
{
	b3Vec3 frictionImpulse;
	b3Vec3 rollingImpulse;
	float twistImpulse;
	float normalImpulse[4];
} v3BlockGridPairRetainedPatch;

static_assert( sizeof( v3BlockGridPairPatch ) == 24, "BlockGrid pair patch layout changed" );
static_assert( sizeof( v3BlockGridPairControl ) == 32, "BlockGrid pair control layout changed" );

// Reserves manifold capacity rounded up to a power of two so changes in manifold
// count can reuse the allocation while they fit that capacity.
v3BlockGridPairState* v3BlockGridPairTryAllocate( b3World* world, int manifoldCount );

// Restores a snapshot allocation with its recorded capacity.
v3BlockGridPairState* v3BlockGridPairTryRestore( b3World* world, uint32_t capacityBytes, int manifoldCount );

// Recomputes the layout for manifoldCount and clears the region to be written.
// Keeps the allocation address and byte count. Returns false if the count does not fit.
bool v3BlockGridPairReuseState( v3BlockGridPairState* state, int manifoldCount );

// Clears all logical contact data while retaining this pair's allocation.
bool v3BlockGridPairRetainCapacityForReplacement( b3Contact* contact );

bool v3BlockGridPairFreeState( b3World* world, v3BlockGridPairState* state );

b3Manifold* v3BlockGridPairManifolds( v3BlockGridPairState* state );
v3BlockGridPairPatch* v3BlockGridPairPatches( v3BlockGridPairState* state );
// One resolved Box3D contact material per manifold, in manifold order
b3ContactMaterial* v3BlockGridPairContactMaterials( v3BlockGridPairState* state );
int v3BlockGridPairManifoldCount( const v3BlockGridPairState* state );
uint32_t v3BlockGridPairStateByteCount( const v3BlockGridPairState* state );
uint32_t v3BlockGridPairCapacityBytes( const v3BlockGridPairState* state );
bool v3BlockGridPairStateBelongsToWorld( const v3BlockGridPairState* state, const b3World* world );

bool v3BlockGridPairInitializeContact( b3Contact* contact, uint64_t sourceEpochA, uint64_t sourceEpochB );
// The caller validates world ownership and remains responsible for the returned state
v3BlockGridPairState* v3BlockGridPairExchangeState( b3Contact* contact, v3BlockGridPairState* state );
// Releases pair storage and clears the contact manifolds.
bool v3BlockGridPairClear( b3World* world, b3Contact* contact );
bool v3BlockGridPairContactIsValid( const b3Contact* contact );
// Returns the stable hitbox and local feature identity, or UINT64_MAX when the point is invalid
uint64_t v3BlockGridPairPointKey( const b3Contact* contact, int manifoldIndex, int pointIndex );
// Identifies the two hitboxes at a manifold point so the caller can read their materials.
bool v3BlockGridPairPointHitboxes( const b3Contact* contact, int manifoldIndex, int pointIndex, int* hitboxIndexA,
								   int* hitboxIndexB );
// Ordered hitbox material indices into the two shape material tables, false when either hitbox or
// its material index is out of range. Runtime callback results are never part of this identity.
bool v3BlockGridPairMaterialPairKey( const b3Shape* shapeA, int hitboxIndexA, const b3Shape* shapeB, int hitboxIndexB,
									 uint32_t* materialPairKey );
// Checks everything a snapshot restores before the pair state becomes visible to a step
bool v3BlockGridPairSnapshotIsValid( const b3World* world, const b3Contact* contact );

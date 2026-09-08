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

// Unset marks a contact the pair update has not reached this step; an update that cannot build a
// manifold, for any reason, leaves the contact separated the way an ordinary Box3D contact does
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

// Sizes the owner with power-of-two manifold headroom so a resting pair whose patch count drifts
// keeps rebuilding into the same block instead of taking a fresh one every step
v3BlockGridPairState* v3BlockGridPairTryAllocate( b3World* world, int manifoldCount );

// Restores a snapshot allocation without recomputing its warmed capacity.
v3BlockGridPairState* v3BlockGridPairTryRestore( b3World* world, uint32_t capacityBytes, int manifoldCount );

// Relays out a retained owner for a new manifold count and clears the region the update is about to
// write, keeping the owner's address and allocation size; false when the count does not fit
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
// Releases the pair storage and leaves the contact with no manifolds, the way an ordinary Box3D
// contact drops its manifold when it stops touching
bool v3BlockGridPairClear( b3World* world, b3Contact* contact );
bool v3BlockGridPairContactIsValid( const b3Contact* contact );
// Returns the stable hitbox and local feature identity, or UINT64_MAX when the point is invalid
uint64_t v3BlockGridPairPointKey( const b3Contact* contact, int manifoldIndex, int pointIndex );
// Names the two struck hitboxes behind one manifold point, so a caller can read their surface
// materials the way a mesh caller reads a manifold point's triangle
bool v3BlockGridPairPointHitboxes( const b3Contact* contact, int manifoldIndex, int pointIndex, int* hitboxIndexA,
								   int* hitboxIndexB );
// Ordered hitbox material indices into the two shape material tables, false when either hitbox or
// its material index is out of range. Runtime callback results are never part of this identity.
bool v3BlockGridPairMaterialPairKey( const b3Shape* shapeA, int hitboxIndexA, const b3Shape* shapeB, int hitboxIndexB,
									 uint32_t* materialPairKey );
// Checks everything a snapshot restores before the pair state becomes visible to a step
bool v3BlockGridPairSnapshotIsValid( const b3World* world, const b3Contact* contact );

// SPDX-License-Identifier: MIT

#include "v3_block_grid_contact.h"

#include "contact.h"
#include "core.h"
#include "physics_world.h"
#include "shape.h"
#include "v3_block_grid_pair.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define V3_BLOCK_GRID_PAIR_STATE_VERSION 1

struct v3BlockGridPairState
{
	uint32_t byteCount;
	uint16_t manifoldCount;
	uint16_t worldId;
	uint16_t worldGeneration;
	uint16_t version;
	uint32_t reserved;
};

typedef struct v3BlockGridPairLayout
{
	size_t cursor;
	uint32_t manifoldOffset;
	uint32_t patchOffset;
	uint32_t materialOffset;
	bool valid;
} v3BlockGridPairLayout;

_Static_assert( sizeof( v3BlockGridPairState ) == 16, "BlockGrid pair state header changed" );

static void v3BlockGridPairReserve( v3BlockGridPairLayout* layout, size_t count, size_t elementSize, size_t alignment,
									uint32_t* offset )
{
	if ( layout->valid == false )
	{
		return;
	}

	if ( count > SIZE_MAX / elementSize || layout->cursor > SIZE_MAX - ( alignment - 1 ) )
	{
		layout->valid = false;
		return;
	}

	size_t bytes = count * elementSize;
	size_t aligned = ( layout->cursor + alignment - 1 ) & ~( alignment - 1 );
	if ( aligned > UINT32_MAX || bytes > UINT32_MAX || aligned > UINT32_MAX - bytes )
	{
		layout->valid = false;
		return;
	}

	*offset = (uint32_t)aligned;
	layout->cursor = aligned + bytes;
}

static v3BlockGridPairLayout v3BlockGridPairPlanLayout( int manifoldCount )
{
	v3BlockGridPairLayout layout = { .cursor = sizeof( v3BlockGridPairState ), .valid = true };
	if ( manifoldCount < 0 || manifoldCount > UINT16_MAX )
	{
		layout.valid = false;
		return layout;
	}
	if ( manifoldCount == 0 )
	{
		return layout;
	}

	v3BlockGridPairReserve( &layout, (size_t)manifoldCount, sizeof( b3Manifold ), _Alignof( b3Manifold ),
							&layout.manifoldOffset );
	v3BlockGridPairReserve( &layout, (size_t)manifoldCount, sizeof( v3BlockGridPairPatch ), _Alignof( v3BlockGridPairPatch ),
							&layout.patchOffset );
	v3BlockGridPairReserve( &layout, (size_t)manifoldCount, sizeof( b3ContactMaterial ), _Alignof( b3ContactMaterial ),
							&layout.materialOffset );

	if ( layout.cursor > (size_t)INT_MAX - ( B3_ALIGNMENT - 1 ) )
	{
		layout.valid = false;
	}

	return layout;
}

static int v3BlockGridPairManifoldCapacity( int manifoldCount )
{
	int capacity = 1;
	while ( capacity < manifoldCount && capacity < v3_blockGridPairStandardMaxRegions )
	{
		capacity *= 2;
	}
	return capacity > v3_blockGridPairStandardMaxRegions ? v3_blockGridPairStandardMaxRegions : capacity;
}

static uint32_t v3BlockGridPairAllocationByteCount( size_t byteCount )
{
	return (uint32_t)( ( byteCount + B3_ALIGNMENT - 1 ) & ~(size_t)( B3_ALIGNMENT - 1 ) );
}

static bool v3BlockGridPairCapacityIsSupported( uint32_t capacityBytes )
{
	for ( int capacity = 1; capacity < v3_blockGridPairStandardMaxRegions; capacity *= 2 )
	{
		v3BlockGridPairLayout layout = v3BlockGridPairPlanLayout( capacity );
		if ( layout.valid && layout.cursor == capacityBytes )
		{
			return true;
		}
	}

	v3BlockGridPairLayout maximum = v3BlockGridPairPlanLayout( v3_blockGridPairStandardMaxRegions );
	return maximum.valid && maximum.cursor == capacityBytes;
}

static bool v3BlockGridPairStateIsValid( const v3BlockGridPairState* state )
{
	if ( state == NULL || state->version != V3_BLOCK_GRID_PAIR_STATE_VERSION || state->reserved != 0 )
	{
		return false;
	}

	// A retained owner keeps the byte count it was allocated with, so a later rebuild with fewer
	// manifolds may occupy less of it than the allocation holds
	v3BlockGridPairLayout layout = v3BlockGridPairPlanLayout( state->manifoldCount );
	return layout.valid && layout.cursor <= state->byteCount && v3BlockGridPairCapacityIsSupported( state->byteCount );
}

bool v3BlockGridPairStateBelongsToWorld( const v3BlockGridPairState* state, const b3World* world )
{
	return world != NULL && v3BlockGridPairStateIsValid( state ) && state->worldId == world->worldId &&
		   state->worldGeneration == world->generation;
}

static v3BlockGridPairState* v3BlockGridPairTryAllocateExact( b3World* world, uint32_t capacityBytes, int manifoldCount )
{
	if ( world == NULL )
	{
		return NULL;
	}

	v3BlockGridPairLayout layout = v3BlockGridPairPlanLayout( manifoldCount );
	if ( layout.valid == false || layout.cursor > capacityBytes || v3BlockGridPairCapacityIsSupported( capacityBytes ) == false )
	{
		return NULL;
	}

	uint64_t allocationByteCount = v3BlockGridPairAllocationByteCount( capacityBytes );
	b3LockMutex( world->contactAllocatorMutex );
	world->blockGridPairBytes += allocationByteCount;
	b3UnlockMutex( world->contactAllocatorMutex );

	v3BlockGridPairState* state = b3TryAlloc( capacityBytes );
	if ( state == NULL )
	{
		b3LockMutex( world->contactAllocatorMutex );
		B3_ASSERT( world->blockGridPairBytes >= allocationByteCount );
		world->blockGridPairBytes -= allocationByteCount;
		b3UnlockMutex( world->contactAllocatorMutex );
		return NULL;
	}

	memset( state, 0, capacityBytes );
	state->byteCount = capacityBytes;
	state->manifoldCount = (uint16_t)manifoldCount;
	state->worldId = world->worldId;
	state->worldGeneration = world->generation;
	state->version = V3_BLOCK_GRID_PAIR_STATE_VERSION;
	return state;
}

v3BlockGridPairState* v3BlockGridPairTryAllocate( b3World* world, int manifoldCount )
{
	v3BlockGridPairLayout layout = v3BlockGridPairPlanLayout( manifoldCount );
	if ( layout.valid == false || manifoldCount == 0 )
	{
		return NULL;
	}

	// Manifold counts drift by a patch or two while a hull rests, so the allocation has headroom.
	v3BlockGridPairLayout capacityLayout = v3BlockGridPairPlanLayout( v3BlockGridPairManifoldCapacity( manifoldCount ) );
	if ( capacityLayout.valid == false || capacityLayout.cursor < layout.cursor )
	{
		capacityLayout = layout;
	}
	return v3BlockGridPairTryAllocateExact( world, (uint32_t)capacityLayout.cursor, manifoldCount );
}

v3BlockGridPairState* v3BlockGridPairTryRestore( b3World* world, uint32_t capacityBytes, int manifoldCount )
{
	return v3BlockGridPairTryAllocateExact( world, capacityBytes, manifoldCount );
}

bool v3BlockGridPairReuseState( v3BlockGridPairState* state, int manifoldCount )
{
	if ( v3BlockGridPairStateIsValid( state ) == false )
	{
		return false;
	}

	v3BlockGridPairLayout layout = v3BlockGridPairPlanLayout( manifoldCount );
	if ( layout.valid == false || layout.cursor > state->byteCount )
	{
		return false;
	}

	uint32_t byteCount = state->byteCount;
	uint16_t worldId = state->worldId;
	uint16_t worldGeneration = state->worldGeneration;
	memset( state, 0, layout.cursor );
	state->byteCount = byteCount;
	state->manifoldCount = (uint16_t)manifoldCount;
	state->worldId = worldId;
	state->worldGeneration = worldGeneration;
	state->version = V3_BLOCK_GRID_PAIR_STATE_VERSION;
	return true;
}

bool v3BlockGridPairFreeState( b3World* world, v3BlockGridPairState* state )
{
	if ( world == NULL || v3BlockGridPairStateBelongsToWorld( state, world ) == false )
	{
		return false;
	}

	size_t byteCount = state->byteCount;
	uint64_t allocationByteCount = v3BlockGridPairAllocationByteCount( byteCount );
	b3LockMutex( world->contactAllocatorMutex );
	if ( world->blockGridPairBytes < allocationByteCount )
	{
		b3UnlockMutex( world->contactAllocatorMutex );
		return false;
	}
	world->blockGridPairBytes -= allocationByteCount;
	b3UnlockMutex( world->contactAllocatorMutex );
	b3Free( state, byteCount );
	return true;
}

b3Manifold* v3BlockGridPairManifolds( v3BlockGridPairState* state )
{
	if ( v3BlockGridPairStateIsValid( state ) == false || state->manifoldCount == 0 )
	{
		return NULL;
	}

	v3BlockGridPairLayout layout = v3BlockGridPairPlanLayout( state->manifoldCount );
	return (b3Manifold*)( (uint8_t*)state + layout.manifoldOffset );
}

v3BlockGridPairPatch* v3BlockGridPairPatches( v3BlockGridPairState* state )
{
	if ( v3BlockGridPairStateIsValid( state ) == false || state->manifoldCount == 0 )
	{
		return NULL;
	}

	v3BlockGridPairLayout layout = v3BlockGridPairPlanLayout( state->manifoldCount );
	return (v3BlockGridPairPatch*)( (uint8_t*)state + layout.patchOffset );
}

b3ContactMaterial* v3BlockGridPairContactMaterials( v3BlockGridPairState* state )
{
	if ( v3BlockGridPairStateIsValid( state ) == false || state->manifoldCount == 0 )
	{
		return NULL;
	}

	v3BlockGridPairLayout layout = v3BlockGridPairPlanLayout( state->manifoldCount );
	return (b3ContactMaterial*)( (uint8_t*)state + layout.materialOffset );
}

int v3BlockGridPairManifoldCount( const v3BlockGridPairState* state )
{
	return state == NULL ? 0 : state->manifoldCount;
}

uint32_t v3BlockGridPairStateByteCount( const v3BlockGridPairState* state )
{
	return v3BlockGridPairStateIsValid( state ) ? v3BlockGridPairAllocationByteCount( state->byteCount ) : 0;
}

uint32_t v3BlockGridPairCapacityBytes( const v3BlockGridPairState* state )
{
	return v3BlockGridPairStateIsValid( state ) ? state->byteCount : 0;
}

bool v3BlockGridPairRetainCapacityForReplacement( b3Contact* contact )
{
	if ( contact == NULL || contact->kind != v3_blockGridPairContactKind || v3BlockGridPairContactIsValid( contact ) == false )
	{
		return false;
	}

	v3BlockGridPairState* state = contact->blockGridPair.state;
	if ( state != NULL )
	{
		uint32_t byteCount = state->byteCount;
		uint16_t worldId = state->worldId;
		uint16_t worldGeneration = state->worldGeneration;
		memset( state, 0, byteCount );
		state->byteCount = byteCount;
		state->worldId = worldId;
		state->worldGeneration = worldGeneration;
		state->version = V3_BLOCK_GRID_PAIR_STATE_VERSION;
	}

	contact->manifolds = NULL;
	contact->manifoldCount = 0;
	contact->blockGridPair = (v3BlockGridPairControl){
		.state = state,
		.sourceEpochA = 1,
		.sourceEpochB = 1,
		.correlationPolicy = v3_blockGridPairCorrelationPolicyVersion,
		.lastOutcome = v3_blockGridPairOutcomeUnset,
	};
	return true;
}

bool v3BlockGridPairInitializeContact( b3Contact* contact, uint64_t sourceEpochA, uint64_t sourceEpochB )
{
	if ( contact == NULL || contact->manifolds != NULL || contact->manifoldCount != 0 || contact->kind != b3_convexContactKind ||
		 contact->reserved != 0 )
	{
		return false;
	}

	contact->kind = v3_blockGridPairContactKind;
	contact->blockGridPair = (v3BlockGridPairControl){
		.sourceEpochA = sourceEpochA,
		.sourceEpochB = sourceEpochB,
	};
	return true;
}

bool v3BlockGridPairContactIsValid( const b3Contact* contact )
{
	if ( contact == NULL || contact->kind != v3_blockGridPairContactKind )
	{
		return false;
	}

	const v3BlockGridPairState* state = contact->blockGridPair.state;
	if ( state == NULL )
	{
		return contact->manifolds == NULL && contact->manifoldCount == 0 && contact->blockGridPair.reserved == 0 &&
			   contact->blockGridPair.lastOutcome <= v3_blockGridPairOutcomeTouching;
	}

	return contact->blockGridPair.reserved == 0 && contact->blockGridPair.lastOutcome <= v3_blockGridPairOutcomeTouching &&
		   v3BlockGridPairStateIsValid( state ) && contact->manifoldCount == state->manifoldCount &&
		   contact->manifolds == v3BlockGridPairManifolds( contact->blockGridPair.state );
}

uint64_t v3BlockGridPairPointKey( const b3Contact* contact, int manifoldIndex, int pointIndex )
{
	if ( v3BlockGridPairContactIsValid( contact ) == false || contact->blockGridPair.state == NULL || manifoldIndex < 0 ||
		 manifoldIndex >= contact->manifoldCount )
	{
		return UINT64_MAX;
	}

	const b3Manifold* manifold = contact->manifolds + manifoldIndex;
	if ( pointIndex < 0 || pointIndex >= manifold->pointCount )
	{
		return UINT64_MAX;
	}

	const v3BlockGridPairPatch* patches = v3BlockGridPairPatches( contact->blockGridPair.state );
	return (uint64_t)patches[manifoldIndex].hitboxPairKeys[pointIndex] << 32 | manifold->points[pointIndex].featureId;
}

bool v3BlockGridPairPointHitboxes( const b3Contact* contact, int manifoldIndex, int pointIndex, int* hitboxIndexA,
								   int* hitboxIndexB )
{
	uint64_t pointKey = v3BlockGridPairPointKey( contact, manifoldIndex, pointIndex );
	if ( pointKey == UINT64_MAX || hitboxIndexA == NULL || hitboxIndexB == NULL )
	{
		return false;
	}

	uint32_t hitboxPairKey = (uint32_t)( pointKey >> 32 );
	*hitboxIndexA = (int)( hitboxPairKey >> 16 );
	*hitboxIndexB = (int)( hitboxPairKey & UINT16_MAX );
	return true;
}

v3BlockGridPairState* v3BlockGridPairExchangeState( b3Contact* contact, v3BlockGridPairState* state )
{
	B3_ASSERT( contact != NULL && contact->kind == v3_blockGridPairContactKind );
	B3_ASSERT( state == NULL || v3BlockGridPairStateIsValid( state ) );
	v3BlockGridPairState* previousState = contact->blockGridPair.state;
	contact->blockGridPair.state = state;
	contact->manifolds = v3BlockGridPairManifolds( state );
	contact->manifoldCount = (uint16_t)v3BlockGridPairManifoldCount( state );
	return previousState;
}

bool v3BlockGridPairClear( b3World* world, b3Contact* contact )
{
	if ( world == NULL || contact == NULL || contact->kind != v3_blockGridPairContactKind ||
		 v3BlockGridPairContactIsValid( contact ) == false )
	{
		return false;
	}

	v3BlockGridPairState* oldState = contact->blockGridPair.state;
	if ( oldState == NULL )
	{
		return true;
	}
	if ( v3BlockGridPairFreeState( world, oldState ) == false )
	{
		return false;
	}

	v3BlockGridPairState* replacedState = v3BlockGridPairExchangeState( contact, NULL );
	B3_ASSERT( replacedState == oldState );
	return replacedState == oldState;
}

bool v3BlockGridPairMaterialPairKey( const b3Shape* shapeA, int hitboxIndexA, const b3Shape* shapeB, int hitboxIndexB,
									 uint32_t* materialPairKey )
{
	if ( shapeA == NULL || shapeB == NULL || shapeA->type != v3_blockGridShape || shapeB->type != v3_blockGridShape ||
		 shapeA->blockGrid == NULL || shapeB->blockGrid == NULL || hitboxIndexA < 0 ||
		 hitboxIndexA >= v3BlockGrid_GetHitboxCount( shapeA->blockGrid ) || hitboxIndexB < 0 ||
		 hitboxIndexB >= v3BlockGrid_GetHitboxCount( shapeB->blockGrid ) )
	{
		return false;
	}

	uint32_t materialIndexA = v3BlockGrid_GetHitboxMaterial( shapeA->blockGrid, hitboxIndexA );
	uint32_t materialIndexB = v3BlockGrid_GetHitboxMaterial( shapeB->blockGrid, hitboxIndexB );
	if ( materialIndexA > UINT16_MAX || materialIndexA >= (uint32_t)shapeA->materialCount || materialIndexB > UINT16_MAX ||
		 materialIndexB >= (uint32_t)shapeB->materialCount )
	{
		return false;
	}

	*materialPairKey = materialIndexA << 16 | materialIndexB;
	return true;
}

static bool v3BlockGridPairSnapshotScalarIsValid( float value, bool nonNegative )
{
	if ( b3IsValidFloat( value ) == false || ( nonNegative && value < 0.0f ) )
	{
		return false;
	}

	// Non-negative coefficients use one representation for zero so replay hashes do not depend on its sign
	return nonNegative == false || value != 0.0f || signbit( value ) == 0;
}

static bool v3BlockGridPairSnapshotMaterialIsValid( const b3ContactMaterial* material )
{
	return v3BlockGridPairSnapshotScalarIsValid( material->friction, true ) &&
		   v3BlockGridPairSnapshotScalarIsValid( material->restitution, true ) &&
		   v3BlockGridPairSnapshotScalarIsValid( material->rollingResistance, true ) &&
		   b3IsValidVec3( material->tangentVelocity );
}

// Patches are written in the reducer's region order, so a snapshot that arrives in any other order
// did not come from a step this build could have produced
static int v3BlockGridPairCompareSnapshotPatches( const v3BlockGridPairPatch* patchA, uint32_t materialPairKeyA,
												  const v3BlockGridPairPatch* patchB, uint32_t materialPairKeyB )
{
	if ( patchA->directedSatKey != patchB->directedSatKey )
	{
		return patchA->directedSatKey < patchB->directedSatKey ? -1 : 1;
	}
	if ( materialPairKeyA != materialPairKeyB )
	{
		return materialPairKeyA < materialPairKeyB ? -1 : 1;
	}
	return ( patchA->supportRegionKey > patchB->supportRegionKey ) - ( patchA->supportRegionKey < patchB->supportRegionKey );
}

bool v3BlockGridPairSnapshotIsValid( const b3World* world, const b3Contact* contact )
{
	if ( world == NULL || v3BlockGridPairContactIsValid( contact ) == false || contact->shapeIdA < 0 ||
		 contact->shapeIdA >= world->shapes.count || contact->shapeIdB < 0 || contact->shapeIdB >= world->shapes.count ||
		 b3IsValidQuat( contact->cachedRotationA ) == false || b3IsValidQuat( contact->cachedRotationB ) == false ||
		 b3IsValidTransform( contact->cachedRelativePose ) == false ||
		 v3BlockGridPairSnapshotScalarIsValid( contact->friction, true ) == false ||
		 v3BlockGridPairSnapshotScalarIsValid( contact->restitution, true ) == false ||
		 v3BlockGridPairSnapshotScalarIsValid( contact->rollingResistance, true ) == false ||
		 b3IsValidVec3( contact->tangentVelocity ) == false || contact->blockGridPair.sourceEpochA == 0 ||
		 contact->blockGridPair.sourceEpochB == 0 ||
		 contact->blockGridPair.correlationPolicy != v3_blockGridPairCorrelationPolicyVersion )
	{
		return false;
	}

	const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
	if ( shapeA->id != contact->shapeIdA || shapeB->id != contact->shapeIdB )
	{
		return false;
	}

	v3BlockGridPairState* state = contact->blockGridPair.state;
	if ( contact->blockGridPair.lastOutcome == v3_blockGridPairOutcomeUnset )
	{
		uint32_t persistentFlags = b3_contactEnableContactEvents | b3_contactStaticFlag | b3_contactRecycleFlag |
								   b3_simEnablePreSolveEvents | b3_enableSpeculativePoints;
		if ( contact->manifoldCount != 0 || contact->manifolds != NULL || contact->blockGridPair.sourceEpochA != 1 ||
			 contact->blockGridPair.sourceEpochB != 1 || ( contact->flags & ~persistentFlags ) != 0 ||
			 contact->colorIndex != B3_NULL_INDEX || contact->islandId != B3_NULL_INDEX ||
			 contact->islandIndex != B3_NULL_INDEX || contact->bodySimIndexA != B3_NULL_INDEX ||
			 contact->bodySimIndexB != B3_NULL_INDEX || contact->cachedRotationA.v.x != 0.0f ||
			 contact->cachedRotationA.v.y != 0.0f || contact->cachedRotationA.v.z != 0.0f || contact->cachedRotationA.s != 1.0f ||
			 contact->cachedRotationB.v.x != 0.0f || contact->cachedRotationB.v.y != 0.0f ||
			 contact->cachedRotationB.v.z != 0.0f || contact->cachedRotationB.s != 1.0f ||
			 contact->cachedRelativePose.p.x != 0.0 || contact->cachedRelativePose.p.y != 0.0 ||
			 contact->cachedRelativePose.p.z != 0.0 || contact->cachedRelativePose.q.v.x != 0.0f ||
			 contact->cachedRelativePose.q.v.y != 0.0f || contact->cachedRelativePose.q.v.z != 0.0f ||
			 contact->cachedRelativePose.q.s != 1.0f || contact->friction != 0.0f || contact->restitution != 0.0f ||
			 contact->rollingResistance != 0.0f || contact->tangentVelocity.x != 0.0f || contact->tangentVelocity.y != 0.0f ||
			 contact->tangentVelocity.z != 0.0f || contact->setIndex < 0 || contact->setIndex >= world->solverSets.count )
		{
			return false;
		}

		const b3SolverSet* set = b3Array_Get( world->solverSets, contact->setIndex );
		if ( contact->localIndex < 0 || contact->localIndex >= set->contactIndices.count ||
			 set->contactIndices.data[contact->localIndex] != contact->contactId )
		{
			return false;
		}
		return state == NULL ||
			   ( v3BlockGridPairStateBelongsToWorld( state, world ) && v3BlockGridPairManifoldCount( state ) == 0 );
	}

	if ( state == NULL )
	{
		return contact->blockGridPair.lastOutcome == v3_blockGridPairOutcomeSeparated;
	}
	if ( v3BlockGridPairStateBelongsToWorld( state, world ) == false ||
		 contact->blockGridPair.lastOutcome != v3_blockGridPairOutcomeTouching )
	{
		return false;
	}

	int manifoldCount = v3BlockGridPairManifoldCount( state );
	if ( manifoldCount > v3_blockGridPairStandardMaxRegions )
	{
		return false;
	}

	const b3Manifold* manifolds = v3BlockGridPairManifolds( state );
	const v3BlockGridPairPatch* patches = v3BlockGridPairPatches( state );
	const b3ContactMaterial* materials = v3BlockGridPairContactMaterials( state );
	uint32_t previousMaterialPairKey = 0;
	for ( int i = 0; i < manifoldCount; ++i )
	{
		const b3Manifold* manifold = manifolds + i;
		const v3BlockGridPairPatch* patch = patches + i;
		uint32_t materialPairKey;
		if ( manifold->pointCount <= 0 || manifold->pointCount > B3_MAX_MANIFOLD_POINTS ||
			 b3IsValidVec3( manifold->normal ) == false || b3IsNormalized( manifold->normal ) == false ||
			 v3BlockGridPairSnapshotScalarIsValid( manifold->twistImpulse, false ) == false ||
			 b3IsValidVec3( manifold->frictionImpulse ) == false || b3IsValidVec3( manifold->rollingImpulse ) == false ||
			 patch->directedSatKey >= v3_blockGridPairSatKeyCount || patch->reserved != 0 ||
			 v3BlockGridPairSnapshotMaterialIsValid( materials + i ) == false ||
			 v3BlockGridPairMaterialPairKey( shapeA, (int)( patch->hitboxPairKeys[0] >> 16 ), shapeB,
											 (int)( patch->hitboxPairKeys[0] & UINT16_MAX ), &materialPairKey ) == false )
		{
			return false;
		}

		if ( i > 0 &&
			 v3BlockGridPairCompareSnapshotPatches( patches + i - 1, previousMaterialPairKey, patch, materialPairKey ) >= 0 )
		{
			return false;
		}
		previousMaterialPairKey = materialPairKey;

		for ( int j = 0; j < manifold->pointCount; ++j )
		{
			const b3ManifoldPoint* point = manifold->points + j;
			uint32_t pointMaterialPairKey;
			uint64_t pointKey = v3BlockGridPairPointKey( contact, i, j );
			if ( b3IsValidVec3( point->anchorA ) == false || b3IsValidVec3( point->anchorB ) == false ||
				 v3BlockGridPairSnapshotScalarIsValid( point->separation, false ) == false ||
				 v3BlockGridPairSnapshotScalarIsValid( point->baseSeparation, false ) == false ||
				 v3BlockGridPairSnapshotScalarIsValid( point->normalImpulse, true ) == false ||
				 v3BlockGridPairSnapshotScalarIsValid( point->totalNormalImpulse, true ) == false ||
				 v3BlockGridPairSnapshotScalarIsValid( point->normalVelocity, false ) == false ||
				 point->triangleIndex != B3_NULL_INDEX || pointKey == UINT64_MAX ||
				 v3BlockGridPairMaterialPairKey( shapeA, (int)( patch->hitboxPairKeys[j] >> 16 ), shapeB,
												 (int)( patch->hitboxPairKeys[j] & UINT16_MAX ),
												 &pointMaterialPairKey ) == false ||
				 pointMaterialPairKey != materialPairKey )
			{
				return false;
			}
			// Emission follows geometry rather than identity. Keys must still be distinct.
			for ( int k = 0; k < j; ++k )
			{
				if ( v3BlockGridPairPointKey( contact, i, k ) == pointKey )
				{
					return false;
				}
			}
		}
		for ( int j = manifold->pointCount; j < B3_MAX_MANIFOLD_POINTS; ++j )
		{
			if ( patch->hitboxPairKeys[j] != 0 )
			{
				return false;
			}
		}
	}

	return true;
}

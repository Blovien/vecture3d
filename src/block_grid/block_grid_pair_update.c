// SPDX-License-Identifier: MIT

#include "block_grid_pair_update.h"

#include "body.h"
#include "contact.h"
#include "hull.h"
#include "manifold.h"
#include "physics_world.h"
#include "shape.h"
#include "block_grid_internal.h"

#include <string.h>

typedef struct v3BlockGridPairPointCandidate
{
	uint16_t currentOrdinal;
	uint16_t previousOrdinal;
} v3BlockGridPairPointCandidate;

_Static_assert( sizeof( v3BlockGridPairPointCandidate ) == sizeof( uint32_t ),
				"point candidate scratch footprint must remain four bytes" );

typedef struct v3BlockGridPairUpdateScratchLayout
{
	size_t builderByteCount;
	size_t reducedPatchByteCount;
	size_t previousPatchByteCount;
	size_t retainedPatchByteCount;
	size_t previousCacheByteCount;
	size_t pointCandidateByteCount;
	size_t byteCount;
} v3BlockGridPairUpdateScratchLayout;

typedef struct v3BlockGridPairUpdateBuffers
{
	void* builderMemory;
	size_t builderByteCapacity;
	v3BlockGridPairReducedPatch* reducedPatches;
	v3BlockGridPairReducedPatch* previousPatches;
	v3BlockGridPairRetainedPatch* retainedPatches;
	struct v3BlockGridPairCacheRecord* previousCaches;
	v3BlockGridPairPointCandidate* pointCandidates;
} v3BlockGridPairUpdateBuffers;

// Exact penetration beyond this depth means the pair cannot produce a usable manifold this step
static const float v3_blockGridPairMaxPenetration = 1.0f;

typedef struct v3BlockGridPairCacheRecord
{
	uint32_t hitboxPairKey;
	b3SATCache cache;
} v3BlockGridPairCacheRecord;

typedef struct v3BlockGridPairCandidateContext
{
	const b3Shape* shapeA;
	const b3Shape* shapeB;
	b3Transform transformBtoA;
	v3BlockGridPairBuilder* builder;
	const v3BlockGridPairCacheRecord* previousCaches;
	int previousCacheCount;
	float admissionDistance;
} v3BlockGridPairCandidateContext;

static bool v3BlockGridPairAddScratchBytes( size_t* total, size_t byteCount, size_t alignment )
{
	if ( alignment == 0 || ( alignment & ( alignment - 1 ) ) != 0 || *total > SIZE_MAX - ( alignment - 1 ) )
	{
		return false;
	}

	size_t aligned = ( *total + alignment - 1 ) & ~( alignment - 1 );
	if ( aligned > SIZE_MAX - byteCount )
	{
		return false;
	}

	*total = aligned + byteCount;
	return true;
}

static v3BlockGridPairUpdateScratchLayout v3BlockGridPairPlanUpdateScratch( void )
{
	v3BlockGridPairUpdateScratchLayout layout = { 0 };
	layout.builderByteCount = v3BlockGridPairBuildScratchByteCount();
	if ( layout.builderByteCount == 0 )
	{
		return layout;
	}
	layout.reducedPatchByteCount = (size_t)v3_blockGridPairStandardMaxRegions * sizeof( v3BlockGridPairReducedPatch );
	layout.previousPatchByteCount = (size_t)v3_blockGridPairStandardMaxRegions * sizeof( v3BlockGridPairReducedPatch );
	layout.retainedPatchByteCount = (size_t)v3_blockGridPairStandardMaxRegions * sizeof( v3BlockGridPairRetainedPatch );
	layout.previousCacheByteCount = (size_t)v3_blockGridPairStandardMaxPoints * sizeof( v3BlockGridPairCacheRecord );
	layout.pointCandidateByteCount =
		(size_t)v3_blockGridPairStandardMaxPoints * v3_blockGridPairStandardMaxPoints * sizeof( v3BlockGridPairPointCandidate );

	// The leading slack covers an arbitrarily aligned caller buffer
	size_t patchAlignment = _Alignof( v3BlockGridPairReducedPatch );
	size_t retainedAlignment = _Alignof( v3BlockGridPairRetainedPatch );
	size_t cacheAlignment = _Alignof( v3BlockGridPairCacheRecord );
	size_t candidateAlignment = _Alignof( uint32_t );
	size_t maximumAlignment = patchAlignment > cacheAlignment ? patchAlignment : cacheAlignment;
	maximumAlignment = maximumAlignment > candidateAlignment ? maximumAlignment : candidateAlignment;
	maximumAlignment = maximumAlignment > retainedAlignment ? maximumAlignment : retainedAlignment;
	size_t total = maximumAlignment - 1;
	if ( v3BlockGridPairAddScratchBytes( &total, layout.builderByteCount, 1 ) == false ||
		 v3BlockGridPairAddScratchBytes( &total, layout.reducedPatchByteCount, patchAlignment ) == false ||
		 v3BlockGridPairAddScratchBytes( &total, layout.previousPatchByteCount, patchAlignment ) == false ||
		 v3BlockGridPairAddScratchBytes( &total, layout.retainedPatchByteCount, retainedAlignment ) == false ||
		 v3BlockGridPairAddScratchBytes( &total, layout.previousCacheByteCount, cacheAlignment ) == false ||
		 v3BlockGridPairAddScratchBytes( &total, layout.pointCandidateByteCount, candidateAlignment ) == false )
	{
		return layout;
	}
	layout.byteCount = total;
	return layout;
}

size_t v3BlockGridPairUpdateScratchByteCount( void )
{
	return v3BlockGridPairPlanUpdateScratch().byteCount;
}

static bool v3BlockGridPairTakeScratch( uintptr_t* cursor, size_t* remaining, size_t byteCount, size_t alignment, void** result )
{
	if ( alignment == 0 || ( alignment & ( alignment - 1 ) ) != 0 || *cursor > UINTPTR_MAX - ( alignment - 1 ) )
	{
		return false;
	}

	uintptr_t aligned = ( *cursor + alignment - 1 ) & ~(uintptr_t)( alignment - 1 );
	size_t padding = (size_t)( aligned - *cursor );
	if ( padding > *remaining || byteCount > *remaining - padding || aligned > UINTPTR_MAX - byteCount )
	{
		return false;
	}

	*result = (void*)aligned;
	*cursor = aligned + byteCount;
	*remaining -= padding + byteCount;
	return true;
}

static bool v3BlockGridPairPartitionUpdateScratch( v3BlockGridPairUpdateScratch* scratch,
												   const v3BlockGridPairUpdateScratchLayout* layout,
												   v3BlockGridPairUpdateBuffers* buffers )
{
	if ( scratch == NULL || scratch->memory == NULL || layout->byteCount == 0 || scratch->byteCapacity < layout->byteCount )
	{
		return false;
	}

	uintptr_t cursor = (uintptr_t)scratch->memory;
	size_t remaining = scratch->byteCapacity;
	void* patches;
	void* previousPatches;
	void* retainedPatches;
	void* previousCaches;
	void* pointCandidates;
	if ( v3BlockGridPairTakeScratch( &cursor, &remaining, layout->builderByteCount, 1, &buffers->builderMemory ) == false ||
		 v3BlockGridPairTakeScratch( &cursor, &remaining, layout->reducedPatchByteCount, _Alignof( v3BlockGridPairReducedPatch ),
									 &patches ) == false ||
		 v3BlockGridPairTakeScratch( &cursor, &remaining, layout->previousPatchByteCount, _Alignof( v3BlockGridPairReducedPatch ),
									 &previousPatches ) == false ||
		 v3BlockGridPairTakeScratch( &cursor, &remaining, layout->retainedPatchByteCount,
									 _Alignof( v3BlockGridPairRetainedPatch ), &retainedPatches ) == false ||
		 v3BlockGridPairTakeScratch( &cursor, &remaining, layout->previousCacheByteCount, _Alignof( v3BlockGridPairCacheRecord ),
									 &previousCaches ) == false ||
		 v3BlockGridPairTakeScratch( &cursor, &remaining, layout->pointCandidateByteCount, _Alignof( uint32_t ),
									 &pointCandidates ) == false )
	{
		return false;
	}

	buffers->builderByteCapacity = layout->builderByteCount;
	buffers->reducedPatches = patches;
	buffers->previousPatches = previousPatches;
	buffers->retainedPatches = retainedPatches;
	buffers->previousCaches = previousCaches;
	buffers->pointCandidates = pointCandidates;
	return true;
}

static const b3SATCache* v3BlockGridPairFindPreviousCache( const v3BlockGridPairCandidateContext* context, int hitboxIndexA,
														   int hitboxIndexB )
{
	uint32_t key = v3BlockGridPairHitboxPairKey( hitboxIndexA, hitboxIndexB );
	int begin = 0;
	int end = context->previousCacheCount;
	while ( begin < end )
	{
		int middle = begin + ( end - begin ) / 2;
		const v3BlockGridPairCacheRecord* record = context->previousCaches + middle;
		if ( record->hitboxPairKey < key )
		{
			begin = middle + 1;
		}
		else
		{
			end = middle;
		}
	}
	return begin < context->previousCacheCount && context->previousCaches[begin].hitboxPairKey == key
			   ? &context->previousCaches[begin].cache
			   : NULL;
}

// A hitbox is a box hull, so it rolls on the hull convention Box3D already uses for convex shapes
static float v3BlockGridPairHitboxRollingRadius( const v3BlockGridData* grid, int hitboxIndex )
{
	b3Vec3 halfExtent = b3ClampBoxHalfExtent( v3BlockGridHitboxHalfExtent( grid, hitboxIndex ) );
	return 0.25f * b3MinFloat( halfExtent.x, b3MinFloat( halfExtent.y, halfExtent.z ) );
}

// Resolves one patch through Box3D's own surface material path, reading the struck hitboxes'
// materials out of the shape material tables exactly as b3UpdateConvexContact reads a shape's
static bool v3BlockGridPairResolvePatchMaterial( const b3World* world, const b3Shape* shapeA, b3Quat rotationA,
												 const b3Shape* shapeB, b3Quat rotationB, uint32_t hitboxPairKey,
												 b3ContactMaterial* contactMaterial )
{
	int hitboxIndexA = (int)( hitboxPairKey >> 16 );
	int hitboxIndexB = (int)( hitboxPairKey & UINT16_MAX );
	uint32_t materialPairKey;
	if ( v3BlockGridPairMaterialPairKey( shapeA, hitboxIndexA, shapeB, hitboxIndexB, &materialPairKey ) == false )
	{
		return false;
	}

	const b3SurfaceMaterial* materialA = b3GetShapeMaterials( shapeA ) + ( materialPairKey >> 16 );
	const b3SurfaceMaterial* materialB = b3GetShapeMaterials( shapeB ) + ( materialPairKey & UINT16_MAX );
	float effectiveRadius = 0.0f;
	if ( materialA->rollingResistance > 0.0f || materialB->rollingResistance > 0.0f )
	{
		effectiveRadius = b3MaxFloat( v3BlockGridPairHitboxRollingRadius( shapeA->blockGrid, hitboxIndexA ),
									  v3BlockGridPairHitboxRollingRadius( shapeB->blockGrid, hitboxIndexB ) );
	}
	return b3ResolveContactMaterial( world, materialA, rotationA, materialB, rotationB, effectiveRadius, contactMaterial );
}

static bool v3BlockGridPairCandidate( int hitboxIndexA, int hitboxIndexB, void* contextPointer )
{
	v3BlockGridPairCandidateContext* context = contextPointer;
	uint32_t materialPairKey;
	if ( v3BlockGridPairMaterialPairKey( context->shapeA, hitboxIndexA, context->shapeB, hitboxIndexB, &materialPairKey ) ==
		 false )
	{
		return false;
	}

	v3BlockGridPairManifold manifold;
	const b3SATCache* previousCache = v3BlockGridPairFindPreviousCache( context, hitboxIndexA, hitboxIndexB );
	v3BlockGridPairManifoldStatus manifoldStatus =
		v3BlockGridPairCollideHitboxes( context->shapeA->blockGrid, hitboxIndexA, context->shapeB->blockGrid, hitboxIndexB,
										context->transformBtoA, previousCache, &manifold, context->admissionDistance );
	if ( manifoldStatus == v3_blockGridPairManifoldInvalid )
	{
		return false;
	}
	if ( manifoldStatus == v3_blockGridPairManifoldEmpty )
	{
		return true;
	}

	for ( int i = 0; i < manifold.pointCount; ++i )
	{
		if ( manifold.points[i].separation < -v3_blockGridPairMaxPenetration )
		{
			return false;
		}
	}

	return v3BlockGridPairBuildAdd( context->builder, hitboxIndexA, hitboxIndexB, materialPairKey, &manifold ) ==
		   v3_blockGridPairBuildCollecting;
}

static b3FeaturePair v3BlockGridPairFeaturePairFromId( uint32_t featureId )
{
	return (b3FeaturePair){
		.owner1 = (uint8_t)( featureId >> 24 ),
		.index1 = (uint8_t)( featureId >> 16 ),
		.owner2 = (uint8_t)( featureId >> 8 ),
		.index2 = (uint8_t)featureId,
	};
}

static bool v3BlockGridPairMakePreviousCache( const v3BlockGridPairPatch* patch, const b3ManifoldPoint* point,
											  v3BlockGridPairCacheRecord* record )
{
	b3SATCache cache = { .separation = point->separation };
	if ( patch->directedSatKey < v3_blockGridPairBoxFaceCount )
	{
		cache.type = b3_faceAxisA;
		cache.indexA = patch->directedSatKey;
	}
	else if ( patch->directedSatKey < v3_blockGridPairFaceSatKeyCount )
	{
		cache.type = b3_faceAxisB;
		cache.indexB = (uint8_t)( ( patch->directedSatKey - v3_blockGridPairBoxFaceCount ) ^ 1 );
	}
	else if ( patch->directedSatKey < v3_blockGridPairSatKeyCount )
	{
		b3FeaturePair pair = v3BlockGridPairFeaturePairFromId( point->featureId );
		if ( pair.owner1 != b3_featureShapeA || pair.owner2 != b3_featureShapeB || pair.index1 >= v3_blockGridPairBoxEdgeCount ||
			 pair.index2 >= v3_blockGridPairBoxEdgeCount || ( pair.index1 & 1 ) != 0 || ( pair.index2 & 1 ) != 0 )
		{
			return false;
		}
		cache.type = b3_edgePairAxis;
		cache.indexA = pair.index1;
		cache.indexB = pair.index2;
	}
	else
	{
		return false;
	}

	record->cache = cache;
	return true;
}

static bool v3BlockGridPairAddPreviousCache( v3BlockGridPairCacheRecord* records, int* count, v3BlockGridPairCacheRecord record )
{
	int index = 0;
	while ( index < *count && records[index].hitboxPairKey < record.hitboxPairKey )
	{
		index += 1;
	}
	if ( index < *count && records[index].hitboxPairKey == record.hitboxPairKey )
	{
		const b3SATCache* previous = &records[index].cache;
		return previous->type == record.cache.type && previous->indexA == record.cache.indexA &&
			   previous->indexB == record.cache.indexB;
	}
	if ( *count == v3_blockGridPairStandardMaxPoints )
	{
		return false;
	}

	memmove( records + index + 1, records + index, (size_t)( *count - index ) * sizeof( records[0] ) );
	records[index] = record;
	*count += 1;
	return true;
}

// SAT caches and impulses cannot cross a source change or a missing transform cache
static v3BlockGridPairState* v3BlockGridPairReusableState( const v3BlockGridPairUpdateInput* input )
{
	const b3Contact* contact = input->contact;
	if ( contact->blockGridPair.state == NULL || contact->blockGridPair.sourceEpochA != input->sourceEpochA ||
		 contact->blockGridPair.sourceEpochB != input->sourceEpochB || ( contact->flags & b3_relativeTransformValid ) == 0 )
	{
		return NULL;
	}
	return contact->blockGridPair.state;
}

static bool v3BlockGridPairBuildPreviousCaches( const v3BlockGridPairUpdateInput* input, v3BlockGridPairCacheRecord* records,
												int* count )
{
	*count = 0;
	v3BlockGridPairState* state = v3BlockGridPairReusableState( input );
	if ( state == NULL )
	{
		return true;
	}

	b3Manifold* manifolds = v3BlockGridPairManifolds( state );
	v3BlockGridPairPatch* patches = v3BlockGridPairPatches( state );
	int manifoldCount = v3BlockGridPairManifoldCount( state );
	for ( int i = 0; i < manifoldCount; ++i )
	{
		const b3Manifold* manifold = manifolds + i;
		const v3BlockGridPairPatch* patch = patches + i;
		if ( manifold->pointCount <= 0 || manifold->pointCount > B3_MAX_MANIFOLD_POINTS || patch->reserved != 0 )
		{
			return false;
		}
		for ( int j = 0; j < manifold->pointCount; ++j )
		{
			v3BlockGridPairCacheRecord record = { .hitboxPairKey = patch->hitboxPairKeys[j] };
			if ( v3BlockGridPairMakePreviousCache( patch, manifold->points + j, &record ) == false ||
				 v3BlockGridPairAddPreviousCache( records, count, record ) == false )
			{
				return false;
			}
		}
	}
	return true;
}

static bool v3BlockGridPairBuildPreviousPatch( const v3BlockGridPairUpdateInput* input, v3BlockGridPairState* state,
											   int patchIndex, v3BlockGridPairReducedPatch* reduced,
											   v3BlockGridPairRetainedPatch* retained )
{
	b3Quat rotationA = input->contact->cachedRotationA;
	b3Vec3 localCenterA = input->localCenterA;
	int manifoldCount = v3BlockGridPairManifoldCount( state );
	if ( state == NULL || reduced == NULL || patchIndex < 0 || patchIndex >= manifoldCount ||
		 b3IsValidQuat( rotationA ) == false || b3IsValidVec3( localCenterA ) == false )
	{
		return false;
	}

	b3Manifold* manifolds = v3BlockGridPairManifolds( state );
	v3BlockGridPairPatch* patches = v3BlockGridPairPatches( state );
	const b3Manifold* manifold = manifolds + patchIndex;
	const v3BlockGridPairPatch* patch = patches + patchIndex;
	uint32_t materialPairKey;
	if ( manifold->pointCount <= 0 || manifold->pointCount > B3_MAX_MANIFOLD_POINTS ||
		 patch->directedSatKey >= v3_blockGridPairSatKeyCount || patch->reserved != 0 ||
		 v3BlockGridPairMaterialPairKey( input->shapeA, (int)( patch->hitboxPairKeys[0] >> 16 ), input->shapeB,
										 (int)( patch->hitboxPairKeys[0] & UINT16_MAX ), &materialPairKey ) == false )
	{
		return false;
	}

	*reduced = (v3BlockGridPairReducedPatch){
		.materialPairKey = materialPairKey,
		.supportRegionKey = patch->supportRegionKey,
		.directedSatKey = patch->directedSatKey,
		.pointCount = (uint8_t)manifold->pointCount,
	};

	memcpy( reduced->hitboxPairKeys, patch->hitboxPairKeys, sizeof( reduced->hitboxPairKeys ) );
	reduced->normal = b3InvRotateVector( rotationA, manifold->normal );
	b3Vec3 centerOffset = b3RotateVector( rotationA, localCenterA );
	*retained = (v3BlockGridPairRetainedPatch){
		.frictionImpulse = manifold->frictionImpulse,
		.rollingImpulse = manifold->rollingImpulse,
		.twistImpulse = manifold->twistImpulse,
	};
	for ( int i = 0; i < manifold->pointCount; ++i )
	{
		const b3ManifoldPoint* source = manifold->points + i;
		reduced->points[i] = (b3LocalManifoldPoint){
			.point = b3InvRotateVector( rotationA, b3Add( source->anchorA, centerOffset ) ),
			.separation = source->separation,
			.pair = v3BlockGridPairFeaturePairFromId( source->featureId ),
			.triangleIndex = B3_NULL_INDEX,
		};
		retained->normalImpulse[i] = source->normalImpulse;
	}
	return true;
}

// Lifts the retained manifolds into caller scratch before the update overwrites the pair storage,
// so the rebuild can write in place and still warm start from what the previous step left behind
static int v3BlockGridPairCaptureRetainedPatches( const v3BlockGridPairUpdateInput* input,
												  v3BlockGridPairReducedPatch* reducedPatches,
												  v3BlockGridPairRetainedPatch* retainedPatches )
{
	v3BlockGridPairState* state = v3BlockGridPairReusableState( input );
	int patchCount = v3BlockGridPairManifoldCount( state );
	for ( int i = 0; i < patchCount; ++i )
	{
		if ( v3BlockGridPairBuildPreviousPatch( input, state, i, reducedPatches + i, retainedPatches + i ) == false )
		{
			return 0;
		}
	}
	return patchCount;
}

typedef struct v3BlockGridPairPointReference
{
	uint16_t patchIndex;
	uint8_t pointIndex;
	uint8_t reserved;
} v3BlockGridPairPointReference;

typedef struct v3BlockGridPairCurrentCorrelationView
{
	b3Manifold* manifolds;
	const v3BlockGridPairReducedPatch* reducedPatches;
	int patchCount;
} v3BlockGridPairCurrentCorrelationView;

typedef struct v3BlockGridPairPreviousCorrelationView
{
	const v3BlockGridPairRetainedPatch* retainedPatches;
	const v3BlockGridPairReducedPatch* reducedPatches;
	int patchCount;
} v3BlockGridPairPreviousCorrelationView;

typedef struct v3BlockGridPairPointReferences
{
	v3BlockGridPairPointReference values[v3_blockGridPairStandardMaxPoints];
	int count;
} v3BlockGridPairPointReferences;

typedef struct v3BlockGridPairPointCandidateContext
{
	const v3BlockGridPairCurrentCorrelationView* current;
	const v3BlockGridPairPreviousCorrelationView* previous;
	const v3BlockGridPairPointReferences* currentReferences;
	const v3BlockGridPairPointReferences* previousReferences;
} v3BlockGridPairPointCandidateContext;

typedef struct v3BlockGridPairAcceptedPointMatch
{
	uint16_t currentOrdinal;
	uint16_t previousOrdinal;
	float maximumAnchorDistanceSquared;
} v3BlockGridPairAcceptedPointMatch;

typedef struct v3BlockGridPairPatchCandidate
{
	float summedGeometricError;
	uint16_t currentPatch;
	uint16_t previousPatch;
	uint16_t pointMatchCount;
	uint16_t reserved;
} v3BlockGridPairPatchCandidate;

typedef struct v3BlockGridPairAcceptedPointMatches
{
	v3BlockGridPairAcceptedPointMatch values[v3_blockGridPairStandardMaxPoints];
	int count;
} v3BlockGridPairAcceptedPointMatches;

typedef struct v3BlockGridPairPatchCandidates
{
	v3BlockGridPairPatchCandidate values[v3_blockGridPairStandardMaxPoints];
	int count;
} v3BlockGridPairPatchCandidates;

static bool v3BlockGridPairBuildPointReferences( const v3BlockGridPairReducedPatch* patches, int patchCount,
												 v3BlockGridPairPointReferences* references )
{
	references->count = 0;
	for ( int patchIndex = 0; patchIndex < patchCount; ++patchIndex )
	{
		int patchPointCount = patches[patchIndex].pointCount;
		if ( patchPointCount <= 0 || patchPointCount > B3_MAX_MANIFOLD_POINTS )
		{
			return false;
		}
		for ( int pointIndex = 0; pointIndex < patchPointCount; ++pointIndex )
		{
			if ( references->count == v3_blockGridPairStandardMaxPoints )
			{
				return false;
			}
			references->values[references->count++] = (v3BlockGridPairPointReference){
				.patchIndex = (uint16_t)patchIndex,
				.pointIndex = (uint8_t)pointIndex,
			};
		}
	}
	return true;
}

static int v3BlockGridPairComparePointCandidates( v3BlockGridPairPointCandidate candidateA,
												  v3BlockGridPairPointCandidate candidateB,
												  const v3BlockGridPairPointCandidateContext* context )
{
	const v3BlockGridPairPointReference* currentReferenceA = context->currentReferences->values + candidateA.currentOrdinal;
	const v3BlockGridPairPointReference* previousReferenceA = context->previousReferences->values + candidateA.previousOrdinal;
	const v3BlockGridPairPointReference* currentReferenceB = context->currentReferences->values + candidateB.currentOrdinal;
	const v3BlockGridPairPointReference* previousReferenceB = context->previousReferences->values + candidateB.previousOrdinal;
	const v3BlockGridPairReducedPatch* currentPatchA = context->current->reducedPatches + currentReferenceA->patchIndex;
	const v3BlockGridPairReducedPatch* previousPatchA = context->previous->reducedPatches + previousReferenceA->patchIndex;
	const v3BlockGridPairReducedPatch* currentPatchB = context->current->reducedPatches + currentReferenceB->patchIndex;
	const v3BlockGridPairReducedPatch* previousPatchB = context->previous->reducedPatches + previousReferenceB->patchIndex;
	v3BlockGridPairPointMatchMetrics metricsA = v3BlockGridPairMeasureReducedPointMatch(
		currentPatchA, currentReferenceA->pointIndex, previousPatchA, previousReferenceA->pointIndex );
	v3BlockGridPairPointMatchMetrics metricsB = v3BlockGridPairMeasureReducedPointMatch(
		currentPatchB, currentReferenceB->pointIndex, previousPatchB, previousReferenceB->pointIndex );
	if ( metricsA.maximumAnchorDistanceSquared != metricsB.maximumAnchorDistanceSquared )
	{
		return metricsA.maximumAnchorDistanceSquared < metricsB.maximumAnchorDistanceSquared ? -1 : 1;
	}
	if ( metricsA.summedAnchorDistanceSquared != metricsB.summedAnchorDistanceSquared )
	{
		return metricsA.summedAnchorDistanceSquared < metricsB.summedAnchorDistanceSquared ? -1 : 1;
	}
	if ( metricsA.normalDot != metricsB.normalDot )
	{
		return metricsA.normalDot > metricsB.normalDot ? -1 : 1;
	}

	uint32_t currentHitboxA = currentPatchA->hitboxPairKeys[currentReferenceA->pointIndex];
	uint32_t previousHitboxA = previousPatchA->hitboxPairKeys[previousReferenceA->pointIndex];
	uint32_t currentHitboxB = currentPatchB->hitboxPairKeys[currentReferenceB->pointIndex];
	uint32_t previousHitboxB = previousPatchB->hitboxPairKeys[previousReferenceB->pointIndex];
	bool sameHitboxA = currentHitboxA == previousHitboxA;
	bool sameHitboxB = currentHitboxB == previousHitboxB;
	if ( sameHitboxA != sameHitboxB )
	{
		return sameHitboxA ? -1 : 1;
	}
	if ( currentHitboxA != currentHitboxB )
	{
		return currentHitboxA < currentHitboxB ? -1 : 1;
	}
	uint32_t currentFeatureA = b3MakeFeatureId( currentPatchA->points[currentReferenceA->pointIndex].pair );
	uint32_t currentFeatureB = b3MakeFeatureId( currentPatchB->points[currentReferenceB->pointIndex].pair );
	if ( currentFeatureA != currentFeatureB )
	{
		return currentFeatureA < currentFeatureB ? -1 : 1;
	}
	if ( previousHitboxA != previousHitboxB )
	{
		return previousHitboxA < previousHitboxB ? -1 : 1;
	}
	uint32_t previousFeatureA = b3MakeFeatureId( previousPatchA->points[previousReferenceA->pointIndex].pair );
	uint32_t previousFeatureB = b3MakeFeatureId( previousPatchB->points[previousReferenceB->pointIndex].pair );
	if ( previousFeatureA != previousFeatureB )
	{
		return previousFeatureA < previousFeatureB ? -1 : 1;
	}
	if ( currentPatchA->supportRegionKey != currentPatchB->supportRegionKey )
	{
		return currentPatchA->supportRegionKey < currentPatchB->supportRegionKey ? -1 : 1;
	}
	if ( previousPatchA->supportRegionKey != previousPatchB->supportRegionKey )
	{
		return previousPatchA->supportRegionKey < previousPatchB->supportRegionKey ? -1 : 1;
	}
	if ( currentReferenceA->patchIndex != currentReferenceB->patchIndex )
	{
		return currentReferenceA->patchIndex < currentReferenceB->patchIndex ? -1 : 1;
	}
	if ( currentReferenceA->pointIndex != currentReferenceB->pointIndex )
	{
		return currentReferenceA->pointIndex < currentReferenceB->pointIndex ? -1 : 1;
	}
	if ( previousReferenceA->patchIndex != previousReferenceB->patchIndex )
	{
		return previousReferenceA->patchIndex < previousReferenceB->patchIndex ? -1 : 1;
	}
	B3_ASSERT( previousReferenceA->pointIndex != previousReferenceB->pointIndex );
	return previousReferenceA->pointIndex < previousReferenceB->pointIndex ? -1 : 1;
}

static void v3BlockGridPairSiftPointCandidates( v3BlockGridPairPointCandidate* candidates, int root, int count,
												const v3BlockGridPairPointCandidateContext* context )
{
	while ( 2 * root + 1 < count )
	{
		int child = 2 * root + 1;
		if ( child + 1 < count && v3BlockGridPairComparePointCandidates( candidates[child], candidates[child + 1], context ) < 0 )
		{
			child += 1;
		}
		if ( v3BlockGridPairComparePointCandidates( candidates[root], candidates[child], context ) >= 0 )
		{
			return;
		}
		v3BlockGridPairPointCandidate swap = candidates[root];
		candidates[root] = candidates[child];
		candidates[child] = swap;
		root = child;
	}
}

static void v3BlockGridPairSortPointCandidates( v3BlockGridPairPointCandidate* candidates, int count,
												const v3BlockGridPairPointCandidateContext* context )
{
	for ( int root = count / 2; root-- > 0; )
	{
		v3BlockGridPairSiftPointCandidates( candidates, root, count, context );
	}
	for ( int end = count - 1; end > 0; --end )
	{
		v3BlockGridPairPointCandidate swap = candidates[0];
		candidates[0] = candidates[end];
		candidates[end] = swap;
		v3BlockGridPairSiftPointCandidates( candidates, 0, end, context );
	}
}

static int v3BlockGridPairComparePatchTuples( const v3BlockGridPairReducedPatch* patchA, int patchOrdinalA,
											  const v3BlockGridPairReducedPatch* patchB, int patchOrdinalB )
{
	if ( patchA->directedSatKey != patchB->directedSatKey )
	{
		return patchA->directedSatKey < patchB->directedSatKey ? -1 : 1;
	}
	if ( patchA->materialPairKey != patchB->materialPairKey )
	{
		return patchA->materialPairKey < patchB->materialPairKey ? -1 : 1;
	}
	if ( patchA->supportRegionKey != patchB->supportRegionKey )
	{
		return patchA->supportRegionKey < patchB->supportRegionKey ? -1 : 1;
	}
	return ( patchOrdinalA > patchOrdinalB ) - ( patchOrdinalA < patchOrdinalB );
}

static int v3BlockGridPairComparePatchCandidates( const v3BlockGridPairPatchCandidate* candidateA,
												  const v3BlockGridPairPatchCandidate* candidateB,
												  const v3BlockGridPairCurrentCorrelationView* current,
												  const v3BlockGridPairPreviousCorrelationView* previous )
{
	if ( candidateA->pointMatchCount != candidateB->pointMatchCount )
	{
		return candidateA->pointMatchCount > candidateB->pointMatchCount ? -1 : 1;
	}
	if ( candidateA->summedGeometricError != candidateB->summedGeometricError )
	{
		return candidateA->summedGeometricError < candidateB->summedGeometricError ? -1 : 1;
	}
	int comparison =
		v3BlockGridPairComparePatchTuples( current->reducedPatches + candidateA->currentPatch, candidateA->currentPatch,
										   current->reducedPatches + candidateB->currentPatch, candidateB->currentPatch );
	if ( comparison != 0 )
	{
		return comparison;
	}
	comparison =
		v3BlockGridPairComparePatchTuples( previous->reducedPatches + candidateA->previousPatch, candidateA->previousPatch,
										   previous->reducedPatches + candidateB->previousPatch, candidateB->previousPatch );
	B3_ASSERT( comparison != 0 );
	return comparison;
}

static void v3BlockGridPairSiftPatchCandidates( v3BlockGridPairPatchCandidate* candidates, int root, int count,
												const v3BlockGridPairCurrentCorrelationView* current,
												const v3BlockGridPairPreviousCorrelationView* previous )
{
	while ( 2 * root + 1 < count )
	{
		int child = 2 * root + 1;
		if ( child + 1 < count &&
			 v3BlockGridPairComparePatchCandidates( candidates + child, candidates + child + 1, current, previous ) < 0 )
		{
			child += 1;
		}
		if ( v3BlockGridPairComparePatchCandidates( candidates + root, candidates + child, current, previous ) >= 0 )
		{
			return;
		}
		v3BlockGridPairPatchCandidate swap = candidates[root];
		candidates[root] = candidates[child];
		candidates[child] = swap;
		root = child;
	}
}

static void v3BlockGridPairSortPatchCandidates( v3BlockGridPairPatchCandidate* candidates, int count,
												const v3BlockGridPairCurrentCorrelationView* current,
												const v3BlockGridPairPreviousCorrelationView* previous )
{
	for ( int root = count / 2; root-- > 0; )
	{
		v3BlockGridPairSiftPatchCandidates( candidates, root, count, current, previous );
	}
	for ( int end = count - 1; end > 0; --end )
	{
		v3BlockGridPairPatchCandidate swap = candidates[0];
		candidates[0] = candidates[end];
		candidates[end] = swap;
		v3BlockGridPairSiftPatchCandidates( candidates, 0, end, current, previous );
	}
}

static int v3BlockGridPairCollectPointCandidates( const v3BlockGridPairCurrentCorrelationView* current,
												  const v3BlockGridPairPreviousCorrelationView* previous,
												  const v3BlockGridPairPointReferences* currentReferences,
												  const v3BlockGridPairPointReferences* previousReferences,
												  v3BlockGridPairPointCandidate* candidates )
{
	int candidateCount = 0;
	for ( int currentOrdinal = 0; currentOrdinal < currentReferences->count; ++currentOrdinal )
	{
		const v3BlockGridPairPointReference* currentReference = currentReferences->values + currentOrdinal;
		const v3BlockGridPairReducedPatch* currentPatch = current->reducedPatches + currentReference->patchIndex;
		for ( int previousOrdinal = 0; previousOrdinal < previousReferences->count; ++previousOrdinal )
		{
			const v3BlockGridPairPointReference* previousReference = previousReferences->values + previousOrdinal;
			const v3BlockGridPairReducedPatch* previousPatch = previous->reducedPatches + previousReference->patchIndex;
			if ( v3BlockGridPairReducedPointHasCompatibleGeometry( currentPatch, currentReference->pointIndex, previousPatch,
																   previousReference->pointIndex ) )
			{
				B3_ASSERT( candidateCount < v3_blockGridPairStandardMaxPoints * v3_blockGridPairStandardMaxPoints );
				candidates[candidateCount++] = (v3BlockGridPairPointCandidate){
					.currentOrdinal = (uint16_t)currentOrdinal,
					.previousOrdinal = (uint16_t)previousOrdinal,
				};
			}
		}
	}
	return candidateCount;
}

static void v3BlockGridPairClaimPointCandidates( const v3BlockGridPairCurrentCorrelationView* current,
												 const v3BlockGridPairPreviousCorrelationView* previous,
												 const v3BlockGridPairPointReferences* currentReferences,
												 const v3BlockGridPairPointReferences* previousReferences,
												 const v3BlockGridPairPointCandidate* candidates, int candidateCount,
												 v3BlockGridPairAcceptedPointMatches* accepted )
{
	bool currentClaimed[v3_blockGridPairStandardMaxPoints] = { 0 };
	bool previousClaimed[v3_blockGridPairStandardMaxPoints] = { 0 };
	accepted->count = 0;
	for ( int candidateIndex = 0; candidateIndex < candidateCount; ++candidateIndex )
	{
		const v3BlockGridPairPointCandidate* candidate = candidates + candidateIndex;
		int currentOrdinal = candidate->currentOrdinal;
		int previousOrdinal = candidate->previousOrdinal;
		if ( currentClaimed[currentOrdinal] || previousClaimed[previousOrdinal] )
		{
			continue;
		}

		const v3BlockGridPairPointReference* currentReference = currentReferences->values + currentOrdinal;
		const v3BlockGridPairPointReference* previousReference = previousReferences->values + previousOrdinal;
		b3ManifoldPoint* currentPoint = current->manifolds[currentReference->patchIndex].points + currentReference->pointIndex;
		currentPoint->normalImpulse =
			previous->retainedPatches[previousReference->patchIndex].normalImpulse[previousReference->pointIndex];
		currentPoint->persisted = true;
		currentClaimed[currentOrdinal] = true;
		previousClaimed[previousOrdinal] = true;
		v3BlockGridPairPointMatchMetrics metrics = v3BlockGridPairMeasureReducedPointMatch(
			current->reducedPatches + currentReference->patchIndex, currentReference->pointIndex,
			previous->reducedPatches + previousReference->patchIndex, previousReference->pointIndex );
		B3_ASSERT( accepted->count < v3_blockGridPairStandardMaxPoints );
		accepted->values[accepted->count++] = (v3BlockGridPairAcceptedPointMatch){
			.currentOrdinal = (uint16_t)currentOrdinal,
			.previousOrdinal = (uint16_t)previousOrdinal,
			.maximumAnchorDistanceSquared = metrics.maximumAnchorDistanceSquared,
		};
	}
}

static void v3BlockGridPairBuildPatchCandidates( const v3BlockGridPairPointReferences* currentReferences,
												 const v3BlockGridPairPointReferences* previousReferences,
												 const v3BlockGridPairAcceptedPointMatches* accepted,
												 v3BlockGridPairPatchCandidates* candidates )
{
	candidates->count = 0;
	for ( int acceptedIndex = 0; acceptedIndex < accepted->count; ++acceptedIndex )
	{
		const v3BlockGridPairAcceptedPointMatch* match = accepted->values + acceptedIndex;
		int currentPatch = currentReferences->values[match->currentOrdinal].patchIndex;
		int previousPatch = previousReferences->values[match->previousOrdinal].patchIndex;
		int patchCandidateIndex = 0;
		while ( patchCandidateIndex < candidates->count &&
				( candidates->values[patchCandidateIndex].currentPatch != currentPatch ||
				  candidates->values[patchCandidateIndex].previousPatch != previousPatch ) )
		{
			patchCandidateIndex += 1;
		}
		if ( patchCandidateIndex == candidates->count )
		{
			B3_ASSERT( candidates->count < v3_blockGridPairStandardMaxPoints );
			candidates->values[candidates->count++] = (v3BlockGridPairPatchCandidate){
				.currentPatch = (uint16_t)currentPatch,
				.previousPatch = (uint16_t)previousPatch,
			};
		}
		candidates->values[patchCandidateIndex].pointMatchCount += 1;
		candidates->values[patchCandidateIndex].summedGeometricError += match->maximumAnchorDistanceSquared;
	}
}

static void v3BlockGridPairTransferPatchImpulses( const v3BlockGridPairCurrentCorrelationView* current,
												  const v3BlockGridPairPreviousCorrelationView* previous,
												  const v3BlockGridPairPatchCandidates* candidates )
{
	bool currentPatchClaimed[v3_blockGridPairStandardMaxRegions] = { 0 };
	bool previousPatchClaimed[v3_blockGridPairStandardMaxRegions] = { 0 };
	for ( int candidateIndex = 0; candidateIndex < candidates->count; ++candidateIndex )
	{
		const v3BlockGridPairPatchCandidate* candidate = candidates->values + candidateIndex;
		if ( currentPatchClaimed[candidate->currentPatch] || previousPatchClaimed[candidate->previousPatch] )
		{
			continue;
		}
		// Box3D warm starts a rebuilt manifold from what the previous step left behind without
		// consulting the material, so a claimed patch keeps its impulses the same way
		b3Manifold* currentManifold = current->manifolds + candidate->currentPatch;
		const v3BlockGridPairRetainedPatch* retained = previous->retainedPatches + candidate->previousPatch;
		currentManifold->frictionImpulse = retained->frictionImpulse;
		currentManifold->twistImpulse = retained->twistImpulse;
		currentManifold->rollingImpulse = retained->rollingImpulse;
		currentPatchClaimed[candidate->currentPatch] = true;
		previousPatchClaimed[candidate->previousPatch] = true;
	}
}

static void v3BlockGridPairCorrelateManifolds( const v3BlockGridPairCurrentCorrelationView* current,
											   const v3BlockGridPairPreviousCorrelationView* previous,
											   v3BlockGridPairPointCandidate* pointCandidates )
{
	v3BlockGridPairPointReferences currentReferences;
	v3BlockGridPairPointReferences previousReferences;
	if ( v3BlockGridPairBuildPointReferences( current->reducedPatches, current->patchCount, &currentReferences ) == false ||
		 v3BlockGridPairBuildPointReferences( previous->reducedPatches, previous->patchCount, &previousReferences ) == false ||
		 currentReferences.count == 0 || previousReferences.count == 0 )
	{
		return;
	}

	int pointCandidateCount =
		v3BlockGridPairCollectPointCandidates( current, previous, &currentReferences, &previousReferences, pointCandidates );
	v3BlockGridPairPointCandidateContext pointContext = {
		.current = current,
		.previous = previous,
		.currentReferences = &currentReferences,
		.previousReferences = &previousReferences,
	};
	v3BlockGridPairSortPointCandidates( pointCandidates, pointCandidateCount, &pointContext );

	v3BlockGridPairAcceptedPointMatches accepted;
	v3BlockGridPairClaimPointCandidates( current, previous, &currentReferences, &previousReferences, pointCandidates,
										 pointCandidateCount, &accepted );
	v3BlockGridPairPatchCandidates patchCandidates;
	v3BlockGridPairBuildPatchCandidates( &currentReferences, &previousReferences, &accepted, &patchCandidates );
	v3BlockGridPairSortPatchCandidates( patchCandidates.values, patchCandidates.count, current, previous );
	v3BlockGridPairTransferPatchImpulses( current, previous, &patchCandidates );
}

// Writes each region's identity beside its manifold and resolves its surface materials once, so a
// failed resolution ends this pair the way b3UpdateConvexContact ends a contact whose material fails
// The lowest selected Hitbox pair remains the material representative when solve order changes,
// including the rolling radius for regions containing Hitboxes with different extents.
static bool v3BlockGridPairWritePatches( const b3World* world, v3BlockGridPairState* state,
										 const v3BlockGridPairUpdateInput* input,
										 const v3BlockGridPairReducedPatch* reducedPatches, int patchCount )
{
	v3BlockGridPairPatch* patches = v3BlockGridPairPatches( state );
	b3ContactMaterial* materials = v3BlockGridPairContactMaterials( state );
	if ( patches == NULL || materials == NULL || patchCount != v3BlockGridPairManifoldCount( state ) )
	{
		return false;
	}

	for ( int i = 0; i < patchCount; ++i )
	{
		const v3BlockGridPairReducedPatch* source = reducedPatches + i;
		if ( source->pointCount == 0 || source->pointCount > B3_MAX_MANIFOLD_POINTS ||
			 source->directedSatKey >= v3_blockGridPairSatKeyCount )
		{
			return false;
		}
		uint32_t materialKey = source->hitboxPairKeys[0];
		for ( int j = 1; j < source->pointCount; ++j )
		{
			if ( source->hitboxPairKeys[j] < materialKey )
				materialKey = source->hitboxPairKeys[j];
		}
		if ( v3BlockGridPairResolvePatchMaterial( world, input->shapeA, input->transformA.q, input->shapeB, input->transformB.q,
												  materialKey, materials + i ) == false )
		{
			return false;
		}

		patches[i] = (v3BlockGridPairPatch){
			.supportRegionKey = source->supportRegionKey,
			.directedSatKey = source->directedSatKey,
		};
		memcpy( patches[i].hitboxPairKeys, source->hitboxPairKeys, sizeof( patches[i].hitboxPairKeys ) );
	}
	return true;
}

static void v3BlockGridPairWriteManifolds( v3BlockGridPairState* state, const v3BlockGridPairReducedPatch* reducedPatches,
										   int patchCount, const v3BlockGridPairUpdateInput* input )
{
	b3Manifold* manifolds = v3BlockGridPairManifolds( state );
	b3Matrix3 matrixA = b3MakeMatrixFromQuat( input->transformA.q );
	b3Vec3 centerA = b3RotateVector( input->transformA.q, input->localCenterA );
	b3Vec3 centerB = b3RotateVector( input->transformB.q, input->localCenterB );
	b3Vec3 originDelta = b3SubPos( input->transformA.p, input->transformB.p );

	for ( int i = 0; i < patchCount; ++i )
	{
		const v3BlockGridPairReducedPatch* sourcePatch = reducedPatches + i;
		b3Manifold* manifold = manifolds + i;
		manifold->normal = b3MulMV( matrixA, sourcePatch->normal );
		manifold->pointCount = sourcePatch->pointCount;
		for ( int j = 0; j < sourcePatch->pointCount; ++j )
		{
			const b3LocalManifoldPoint* sourcePoint = sourcePatch->points + j;
			b3Vec3 anchorA = b3MulMV( matrixA, sourcePoint->point );
			manifold->points[j] = (b3ManifoldPoint){
				.anchorA = b3Sub( anchorA, centerA ),
				.anchorB = b3Sub( b3Add( anchorA, originDelta ), centerB ),
				.separation = sourcePoint->separation,
				.baseSeparation = sourcePoint->separation,
				.featureId = b3MakeFeatureId( sourcePoint->pair ),
				.triangleIndex = B3_NULL_INDEX,
			};
		}
	}
}

static bool v3BlockGridPairUpdateInputIsValid( const b3World* world, const v3BlockGridPairUpdateInput* input )
{
	return world != NULL && input != NULL && v3BlockGridPairContactIsValid( input->contact ) && input->shapeA != NULL &&
		   input->shapeB != NULL && input->shapeA->type == v3_blockGridShape && input->shapeB->type == v3_blockGridShape &&
		   input->shapeA->blockGrid != NULL && input->shapeB->blockGrid != NULL &&
		   input->contact->shapeIdA == input->shapeA->id && input->contact->shapeIdB == input->shapeB->id &&
		   input->sourceEpochA != 0 && input->sourceEpochB != 0 && b3IsValidVec3( input->localCenterA ) &&
		   b3IsValidVec3( input->localCenterB ) && b3IsValidWorldTransform( input->transformA ) &&
		   b3IsValidWorldTransform( input->transformB );
}

// An update with no usable geometry preserves contact identity and clears its manifolds.
static v3BlockGridPairUpdateResult v3BlockGridPairSeparateContact( b3World* world, const v3BlockGridPairUpdateInput* input,
																   v3BlockGridPairState* orphan,
																   v3BlockGridPairUpdateResult result )
{
	b3Contact* contact = input->contact;
	v3BlockGridPairState* owner = v3BlockGridPairExchangeState( contact, NULL );
	if ( owner != NULL && owner != orphan )
	{
		v3BlockGridPairFreeState( world, owner );
	}
	if ( orphan != NULL )
	{
		v3BlockGridPairFreeState( world, orphan );
	}
	contact->blockGridPair.sourceEpochA = input->sourceEpochA;
	contact->blockGridPair.sourceEpochB = input->sourceEpochB;
	contact->blockGridPair.correlationPolicy = v3_blockGridPairCorrelationPolicyVersion;
	contact->blockGridPair.lastOutcome = v3_blockGridPairOutcomeSeparated;
	return result;
}

// The same contact geometry must not acquire a different solve order merely
// because it was cooked into different hitboxes. Prefer the smallest rotational
// contribution to normal inverse mass, then world position and exact identity.
// This treats both dynamic bodies symmetrically and gives static bodies no weight.
static bool v3BlockGridPairOrderPoints( b3World* world, const v3BlockGridPairUpdateInput* input,
										v3BlockGridPairReducedPatch* patches, int patchCount )
{
	b3BodySim* simA = b3GetBodySim( world, world->bodies.data + input->shapeA->bodyId );
	b3BodySim* simB = b3GetBodySim( world, world->bodies.data + input->shapeB->bodyId );
	b3Matrix3 rotationA = b3MakeMatrixFromQuat( input->transformA.q );
	b3Vec3 centerA = b3RotateVector( input->transformA.q, input->localCenterA );
	b3Vec3 centerB = b3RotateVector( input->transformB.q, input->localCenterB );
	b3Vec3 originDelta = b3SubPos( input->transformA.p, input->transformB.p );
	for ( int p = 0; p < patchCount; ++p )
	{
		v3BlockGridPairReducedPatch* patch = patches + p;
		b3Vec3 normal = b3MulMV( rotationA, patch->normal );
		float scores[4];
		b3Pos positions[4];
		for ( int i = 0; i < patch->pointCount; ++i )
		{
			b3Vec3 anchor = b3MulMV( rotationA, patch->points[i].point );
			b3Vec3 a = b3Cross( b3Sub( anchor, centerA ), normal );
			b3Vec3 b = b3Cross( b3Sub( b3Add( anchor, originDelta ), centerB ), normal );
			scores[i] = b3Dot( a, b3MulMV( simA->invInertiaWorld, a ) ) + b3Dot( b, b3MulMV( simB->invInertiaWorld, b ) );
			if ( !b3IsValidFloat( scores[i] ) )
				return false;
			positions[i] = b3OffsetPos( input->transformA.p, anchor );
		}
		for ( int i = 1; i < patch->pointCount; ++i )
		{
			int j = i;
			while ( j > 0 )
			{
				float a = scores[j], b = scores[j - 1];
				b3Pos x = positions[j], y = positions[j - 1];
				uint64_t keyA = (uint64_t)patch->hitboxPairKeys[j] << 32 | b3MakeFeatureId( patch->points[j].pair );
				uint64_t keyB = (uint64_t)patch->hitboxPairKeys[j - 1] << 32 | b3MakeFeatureId( patch->points[j - 1].pair );
				bool lower;
				if ( a != b )
					lower = a < b;
				else if ( x.x != y.x )
					lower = x.x < y.x;
				else if ( x.y != y.y )
					lower = x.y < y.y;
				else if ( x.z != y.z )
					lower = x.z < y.z;
				else
					lower = keyA < keyB;
				if ( !lower )
					break;
				scores[j] = b;
				scores[j - 1] = a;
				positions[j] = y;
				positions[j - 1] = x;
				b3LocalManifoldPoint point = patch->points[j];
				patch->points[j] = patch->points[j - 1];
				patch->points[j - 1] = point;
				uint32_t key = patch->hitboxPairKeys[j];
				patch->hitboxPairKeys[j] = patch->hitboxPairKeys[j - 1];
				patch->hitboxPairKeys[j - 1] = key;
				--j;
			}
		}
	}
	return true;
}

v3BlockGridPairUpdateResult v3BlockGridPairUpdateContact( b3World* world, const v3BlockGridPairUpdateInput* input,
														  v3BlockGridPairUpdateScratch* scratch )
{
	v3BlockGridPairUpdateResult result = { 0 };
	v3BlockGridPairUpdateScratchLayout layout = v3BlockGridPairPlanUpdateScratch();
	if ( v3BlockGridPairUpdateInputIsValid( world, input ) == false || layout.byteCount == 0 )
	{
		return result;
	}

	b3Contact* contact = input->contact;
	const b3Shape* shapeA = input->shapeA;
	const b3Shape* shapeB = input->shapeB;
	v3BlockGridPairUpdateBuffers buffers;
	if ( !b3IsValidFloat( input->admissionDistance ) || input->admissionDistance < 0.0f ||
		 v3BlockGridPairPartitionUpdateScratch( scratch, &layout, &buffers ) == false )
	{
		return v3BlockGridPairSeparateContact( world, input, NULL, result );
	}

	v3BlockGridPairBuilder* builder = v3BlockGridPairBuildBegin( buffers.builderMemory, buffers.builderByteCapacity, NULL );
	int previousCacheCount = 0;
	if ( builder == NULL || v3BlockGridPairBuildPreviousCaches( input, buffers.previousCaches, &previousCacheCount ) == false )
	{
		return v3BlockGridPairSeparateContact( world, input, NULL, result );
	}

	v3BlockGridPairCandidateContext candidate = {
		.shapeA = shapeA,
		.shapeB = shapeB,
		.transformBtoA = b3InvMulWorldTransforms( input->transformA, input->transformB ),
		.builder = builder,
		.previousCaches = buffers.previousCaches,
		.previousCacheCount = previousCacheCount,
		.admissionDistance = input->admissionDistance,
	};
	b3Pos base = input->transformA.p;
	v3BlockGridPairTraversalResult traversal =
		v3BlockGridPairEnumerateCandidates( shapeA->blockGrid, b3ToRelativeTransform( input->transformA, base ),
											shapeB->blockGrid, b3ToRelativeTransform( input->transformB, base ),
											&scratch->traversal, v3BlockGridPairCandidate, &candidate, input->admissionDistance );
	result.candidateHitboxPairCount = traversal.candidatePairs;
	if ( traversal.status != v3_blockGridPairTraversalCompleted )
	{
		return v3BlockGridPairSeparateContact( world, input, NULL, result );
	}

	v3BlockGridPairBuildResult build =
		v3BlockGridPairBuildFinish( builder, buffers.reducedPatches, v3_blockGridPairStandardMaxRegions );
	result.touchingPairCount = build.touchingPairCount;
	result.reductionCount = build.reductionEngaged ? 1 : 0;
	if ( build.status != v3_blockGridPairBuildCompleted || build.patchCount == 0 )
	{
		return v3BlockGridPairSeparateContact( world, input, NULL, result );
	}

	if ( !v3BlockGridPairOrderPoints( world, input, buffers.reducedPatches, build.patchCount ) )
	{
		return v3BlockGridPairSeparateContact( world, input, NULL, result );
	}

	// The retained side moves into scratch first so the rebuild can overwrite the pair storage in place
	int previousPatchCount = v3BlockGridPairCaptureRetainedPatches( input, buffers.previousPatches, buffers.retainedPatches );

	// Reuse the allocation when the new manifold data fits its capacity.
	v3BlockGridPairState* state = contact->blockGridPair.state;
	v3BlockGridPairState* allocated = NULL;
	if ( v3BlockGridPairReuseState( state, build.patchCount ) == false )
	{
		state = v3BlockGridPairTryAllocate( world, build.patchCount );
		if ( state == NULL )
		{
			return v3BlockGridPairSeparateContact( world, input, NULL, result );
		}
		allocated = state;
	}

	if ( v3BlockGridPairWritePatches( world, state, input, buffers.reducedPatches, build.patchCount ) == false )
	{
		return v3BlockGridPairSeparateContact( world, input, allocated, result );
	}

	v3BlockGridPairWriteManifolds( state, buffers.reducedPatches, build.patchCount, input );
	v3BlockGridPairState* replaced = v3BlockGridPairExchangeState( contact, state );
	if ( replaced != NULL && replaced != state )
	{
		v3BlockGridPairFreeState( world, replaced );
	}
	if ( previousPatchCount > 0 )
	{
		v3BlockGridPairCurrentCorrelationView currentView = {
			.manifolds = v3BlockGridPairManifolds( state ),
			.reducedPatches = buffers.reducedPatches,
			.patchCount = build.patchCount,
		};
		v3BlockGridPairPreviousCorrelationView previousView = {
			.retainedPatches = buffers.retainedPatches,
			.reducedPatches = buffers.previousPatches,
			.patchCount = previousPatchCount,
		};
		v3BlockGridPairCorrelateManifolds( &currentView, &previousView, buffers.pointCandidates );
	}

	contact->blockGridPair.sourceEpochA = input->sourceEpochA;
	contact->blockGridPair.sourceEpochB = input->sourceEpochB;
	contact->blockGridPair.correlationPolicy = v3_blockGridPairCorrelationPolicyVersion;
	contact->blockGridPair.lastOutcome = v3_blockGridPairOutcomeTouching;
	return result;
}

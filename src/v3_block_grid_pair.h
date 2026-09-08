// SPDX-License-Identifier: MIT

#pragma once

#include "v3_block_grid.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// A traversal either enumerated every candidate pair from valid cooked geometry or produced nothing the update can use
typedef enum v3BlockGridPairTraversalStatus
{
	v3_blockGridPairTraversalInvalid,
	v3_blockGridPairTraversalCompleted,
} v3BlockGridPairTraversalStatus;

typedef struct v3BlockGridPairTraversalScratch
{
	v3BlockGridQueryScratch queryA;
	v3BlockGridQueryScratch queryB;
} v3BlockGridPairTraversalScratch;

typedef struct v3BlockGridPairTraversalResult
{
	v3BlockGridPairTraversalStatus status;

	// visitedGroups and markedReferences report group walks rather than distinct groups, since a
	// chosen direct query charges its window once while measuring and again while walking;
	// candidatePairs is a step counter
	uint64_t visitedGroups;
	uint64_t markedReferences;
	uint64_t candidatePairs;
} v3BlockGridPairTraversalResult;

enum
{
	v3_blockGridPairBoxFaceCount = 6,
	v3_blockGridPairFaceSatKeyCount = 12,
	v3_blockGridPairBoxEdgeCount = 24,
	v3_blockGridPairSatKeyCount = 30,
	v3_blockGridPairSatKeyInvalid = UINT8_MAX,
	v3_blockGridPairMaxFootprintPoints = 8,

	// Version one retains a valid previous axis within one Box3D linear slop
	v3_blockGridPairSatPolicyVersion = 1,

	v3_blockGridPairCorrelationPolicyVersion = 2,
	// A pair keeps at most this many support regions. Each retained region emits no more than
	// Box3D's four manifold points, so the correlation storage below is derived from this limit.
	v3_blockGridPairStandardMaxRegions = 120,
	v3_blockGridPairStandardMaxPoints = v3_blockGridPairStandardMaxRegions * B3_MAX_MANIFOLD_POINTS,

	// Touching leaves are reduced in batches while the retained solver representation stays bounded.
	v3_blockGridPairMaxTouchingPairs = 4096,
};

typedef enum v3BlockGridPairManifoldStatus
{
	v3_blockGridPairManifoldInvalid,
	v3_blockGridPairManifoldEmpty,
	v3_blockGridPairManifoldTouching,
} v3BlockGridPairManifoldStatus;

typedef struct v3BlockGridPairFootprint
{
	// The unreduced clipped area lets adjacent hitbox contacts form one support region
	b3Vec3 points[v3_blockGridPairMaxFootprintPoints];
	uint8_t pointCount;
	uint8_t reserved[3];
} v3BlockGridPairFootprint;

typedef struct v3BlockGridPairManifold
{
	b3Vec3 normal;
	b3LocalManifoldPoint points[4];
	v3BlockGridPairFootprint footprint;
	b3SATCache satCache;
	int pointCount;
	uint8_t directedSatKey;
	uint8_t retainedPreviousAxis;
	uint8_t reserved[2];
} v3BlockGridPairManifold;

// Invalid input, insufficient scratch and checked arithmetic failure leave the update without a
// manifold. Valid collection continues through fixed batches into the bounded patch output.
typedef enum v3BlockGridPairBuildStatus
{
	v3_blockGridPairBuildInvalid,
	v3_blockGridPairBuildCollecting,
	v3_blockGridPairBuildCompleted,
} v3BlockGridPairBuildStatus;

typedef struct v3BlockGridPairReducedPatch
{
	b3Vec3 normal;
	b3LocalManifoldPoint points[4];
	uint32_t hitboxPairKeys[4];
	// Ordered hitbox material indices, so one patch never mixes two surface materials
	uint32_t materialPairKey;
	// The lowest hitbox-pair key keeps the region stable across traversal order
	uint32_t supportRegionKey;
	uint8_t directedSatKey;
	uint8_t pointCount;
	uint8_t reserved[2];
} v3BlockGridPairReducedPatch;

typedef struct v3BlockGridPairBuildResult
{
	v3BlockGridPairBuildStatus status;
	uint64_t touchingPairCount;
	bool reductionEngaged;
	int patchCount;
} v3BlockGridPairBuildResult;

typedef struct v3BlockGridPairBuilder v3BlockGridPairBuilder;

// Candidate arguments always name A first, while callback order follows the selected direction
typedef bool v3BlockGridPairCandidateFcn( int hitboxIndexA, int hitboxIndexB, void* context );

v3BlockGridPairTraversalResult v3BlockGridPairEnumerateCandidates( const v3BlockGridData* gridA, b3Transform transformA,
																   const v3BlockGridData* gridB, b3Transform transformB,
																   v3BlockGridPairTraversalScratch* scratch,
																   v3BlockGridPairCandidateFcn* callback, void* context,
																   float admissionDistance );

// Runs the ordinary Box3D hull manifold for one hitbox pair and records the SAT family it used
// A cache from the same retained hitbox pair may keep its axis when exhaustive SAT stays within one linear slop
v3BlockGridPairManifoldStatus v3BlockGridPairCollideHitboxes( const v3BlockGridData* gridA, int hitboxIndexA,
															  const v3BlockGridData* gridB, int hitboxIndexB,
															  b3Transform transformBtoA, const b3SATCache* previousCache,
															  v3BlockGridPairManifold* result, float admissionDistance );

size_t v3BlockGridPairBuildScratchByteCount( void );
uint32_t v3BlockGridPairHitboxPairKey( int hitboxIndexA, int hitboxIndexB );

// Builder state remains in caller scratch and is valid until finish returns
v3BlockGridPairBuilder* v3BlockGridPairBuildBegin( void* memory, size_t byteCapacity, v3BlockGridPairBuildStatus* status );
v3BlockGridPairBuildStatus v3BlockGridPairBuildAdd( v3BlockGridPairBuilder* builder, int hitboxIndexA, int hitboxIndexB,
													uint32_t materialPairKey, const v3BlockGridPairManifold* manifold );
v3BlockGridPairBuildResult v3BlockGridPairBuildFinish( v3BlockGridPairBuilder* builder, v3BlockGridPairReducedPatch* patches,
													   int patchCapacity );

// Callers compare both source epochs before using these geometric compatibility checks
typedef struct v3BlockGridPairPointMatchMetrics
{
	float anchorDistanceSquaredA;
	float anchorDistanceSquaredB;
	float maximumAnchorDistanceSquared;
	float summedAnchorDistanceSquared;
	float normalDot;
} v3BlockGridPairPointMatchMetrics;

// Both patches must be nonnull and both point indices must be valid
v3BlockGridPairPointMatchMetrics v3BlockGridPairMeasureReducedPointMatch( const v3BlockGridPairReducedPatch* current,
																		  int currentPoint,
																		  const v3BlockGridPairReducedPatch* previous,
																		  int previousPoint );
bool v3BlockGridPairReducedPointHasCompatibleGeometry( const v3BlockGridPairReducedPatch* current, int currentPoint,
													   const v3BlockGridPairReducedPatch* previous, int previousPoint );

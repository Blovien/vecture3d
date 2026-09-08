// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "v3_block_grid.h"
#include "v3_block_grid_internal.h"
#include "vecture3d/block_grid.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( _WIN32 )
#include <malloc.h>
#endif

#define CHECK( condition )                                                                                                       \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( !( condition ) )                                                                                                    \
		{                                                                                                                        \
			fprintf( stderr, "condition false at %s:%d: %s\n", __FILE__, __LINE__, #condition );                                 \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static int allocationCount;
static int allocationFailureIndex;

static void* FailingAlloc( int32_t size, int32_t alignment )
{
	if ( allocationCount++ == allocationFailureIndex )
	{
		return NULL;
	}
#if defined( _WIN32 )
	return _aligned_malloc( (size_t)size, (size_t)alignment );
#else
	return aligned_alloc( (size_t)alignment, (size_t)size );
#endif
}

static void FailingFree( void* memory )
{
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

static bool CookStatsAreZero( v3BlockGridCookStats stats )
{
	return stats.materialCount == 0 && stats.blockCount == 0 && stats.boxCount == 0 && stats.byteCount == 0 &&
		   stats.inputBoxCount == 0 && stats.culledBoxCount == 0 && stats.mergedBoxCount == 0 &&
		   stats.localBounds.lowerBound.x == 0.0f && stats.localBounds.lowerBound.y == 0.0f &&
		   stats.localBounds.lowerBound.z == 0.0f && stats.localBounds.upperBound.x == 0.0f &&
		   stats.localBounds.upperBound.y == 0.0f && stats.localBounds.upperBound.z == 0.0f;
}

static bool CookFailedAtomically( v3BlockGridCookResult result, v3BlockGridCookStatus expectedStatus )
{
	return result.status == expectedStatus && result.data == NULL && CookStatsAreZero( result.stats );
}

static bool Vec3Equals( b3Vec3 a, b3Vec3 b )
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

static int TestCookCopiesInputArrays( void )
{
	int byteCountBeforeCook = b3GetByteCount();
	b3SurfaceMaterial materials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
	materials[0].friction = 0.125f;
	materials[0].restitution = 0.25f;
	materials[0].rollingResistance = 0.375f;
	materials[0].tangentVelocity = (b3Vec3){ 0.5f, -0.25f, 0.75f };
	materials[0].userMaterialId = UINT64_C( 0x1020304050607080 );
	materials[0].customColor = UINT32_C( 0x00123456 );
	materials[1].friction = 0.625f;
	materials[1].restitution = 0.75f;
	materials[1].rollingResistance = 0.875f;
	materials[1].tangentVelocity = (b3Vec3){ -0.5f, 0.25f, -0.75f };
	materials[1].userMaterialId = UINT64_C( 0x8070605040302010 );
	materials[1].customColor = UINT32_C( 0x00654321 );

	v3BlockGridBox positiveBoxes[1] = {
		{ .bounds = { { 0.25f, 0.0f, 0.5f }, { 0.75f, 0.5f, 1.0f } }, .materialIndex = 0 },
	};
	v3BlockGridBox negativeBoxes[1] = {
		{ .bounds = { { 0.125f, 0.25f, 0.375f }, { 0.625f, 0.75f, 0.875f } }, .materialIndex = 1 },
	};
	v3BlockGridBlock blocks[2] = {
		{ .x = 4, .y = -2, .z = 7, .userData = UINT64_C( 0x1122334455667788 ), .boxes = positiveBoxes, .boxCount = 1 },
		{ .x = -3, .y = 5, .z = -1, .userData = UINT64_C( 0x8877665544332211 ), .boxes = negativeBoxes, .boxCount = 1 },
	};
	v3BlockGridCookDef def = { .materials = materials, .materialCount = 2, .blocks = blocks, .blockCount = 2 };

	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	CHECK( result.status == v3_blockGridCookOk );
	CHECK( result.data != NULL );

	memset( materials, 0, sizeof( materials ) );
	memset( positiveBoxes, 0, sizeof( positiveBoxes ) );
	memset( negativeBoxes, 0, sizeof( negativeBoxes ) );
	memset( blocks, 0, sizeof( blocks ) );

	const b3SurfaceMaterial* cookedMaterials = v3BlockGrid_GetMaterials( result.data );
	CHECK( cookedMaterials[0].friction == 0.125f );
	CHECK( cookedMaterials[0].restitution == 0.25f );
	CHECK( cookedMaterials[0].rollingResistance == 0.375f );
	CHECK( Vec3Equals( cookedMaterials[0].tangentVelocity, (b3Vec3){ 0.5f, -0.25f, 0.75f } ) );
	CHECK( cookedMaterials[0].userMaterialId == UINT64_C( 0x1020304050607080 ) );
	CHECK( cookedMaterials[0].customColor == UINT32_C( 0x00123456 ) );
	CHECK( cookedMaterials[1].friction == 0.625f );
	CHECK( cookedMaterials[1].restitution == 0.75f );
	CHECK( cookedMaterials[1].rollingResistance == 0.875f );
	CHECK( Vec3Equals( cookedMaterials[1].tangentVelocity, (b3Vec3){ -0.5f, 0.25f, -0.75f } ) );
	CHECK( cookedMaterials[1].userMaterialId == UINT64_C( 0x8070605040302010 ) );
	CHECK( cookedMaterials[1].customColor == UINT32_C( 0x00654321 ) );

	int originX, originY, originZ, placement;
	v3BlockGrid_GetPlacement( result.data, &originX, &originY, &originZ, &placement );
	const v3BlockGridBlockData* cookedBlocks = v3BlockGridBlocks( result.data );
	CHECK( cookedBlocks[0].x + originX == -3 );
	CHECK( cookedBlocks[0].y + originY == 5 );
	CHECK( cookedBlocks[0].z + originZ == -1 );
	CHECK( cookedBlocks[0].sourceId == UINT64_C( 0x8877665544332211 ) );
	CHECK( cookedBlocks[1].x + originX == 4 );
	CHECK( cookedBlocks[1].y + originY == -2 );
	CHECK( cookedBlocks[1].z + originZ == 7 );
	CHECK( cookedBlocks[1].sourceId == UINT64_C( 0x1122334455667788 ) );

	CHECK( v3BlockGrid_GetHitboxMaterial( result.data, 0 ) == 1 );
	b3Vec3 center, halfExtent;
	v3BlockGrid_GetHitboxBounds( result.data, 0, &center, &halfExtent );
	CHECK( Vec3Equals( center, (b3Vec3){ -2.625f, 5.5f, -0.375f } ) );
	CHECK( Vec3Equals( halfExtent, (b3Vec3){ 0.25f, 0.25f, 0.25f } ) );
	CHECK( v3BlockGrid_GetHitboxMaterial( result.data, 1 ) == 0 );
	v3BlockGrid_GetHitboxBounds( result.data, 1, &center, &halfExtent );
	CHECK( Vec3Equals( center, (b3Vec3){ 4.5f, -1.75f, 7.75f } ) );
	CHECK( Vec3Equals( halfExtent, (b3Vec3){ 0.25f, 0.25f, 0.25f } ) );

	v3DestroyBlockGridData( result.data );
	CHECK( b3GetByteCount() == byteCountBeforeCook );
	return 0;
}

int main( void )
{
	CHECK( TestCookCopiesInputArrays() == 0 );

	int byteCountBeforeCook = b3GetByteCount();
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	v3BlockGridBox boxes[2] = {
		{ .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.5f, 1.0f } }, .materialIndex = 0 },
		{ .bounds = { { 0.25f, 0.5f, 0.25f }, { 0.75f, 1.0f, 0.75f } }, .materialIndex = 0 },
	};
	v3BlockGridBlock block = { .x = 2, .y = -1, .z = 4, .userData = 7, .boxes = boxes, .boxCount = 2 };
	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = &block, .blockCount = 1 };

	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	CHECK( result.status == v3_blockGridCookOk );
	CHECK( result.data != NULL );
	CHECK( result.stats.materialCount == 1 );
	CHECK( result.stats.blockCount == 1 );
	CHECK( result.stats.boxCount == 2 );
	CHECK( result.stats.byteCount > 0 );

	v3BlockGridCookStats sharedStats = v3BlockGrid_GetCookStats( result.data );
	CHECK( sharedStats.blockCount == result.stats.blockCount );
	CHECK( sharedStats.boxCount == result.stats.boxCount );
	CHECK( sharedStats.byteCount == result.stats.byteCount );
	CHECK( result.stats.localBounds.lowerBound.x == 2.0f );
	CHECK( result.stats.localBounds.lowerBound.y == -1.0f );
	CHECK( result.stats.localBounds.lowerBound.z == 4.0f );
	CHECK( result.stats.localBounds.upperBound.x == 3.0f );
	CHECK( result.stats.localBounds.upperBound.y == 0.0f );
	CHECK( result.stats.localBounds.upperBound.z == 5.0f );

	v3DestroyBlockGridData( result.data );
	CHECK( b3GetByteCount() == byteCountBeforeCook );
	v3DestroyBlockGridData( NULL );
	sharedStats = v3BlockGrid_GetCookStats( NULL );
	CHECK( CookStatsAreZero( sharedStats ) );

	result = v3CookBlockGrid( NULL );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidDefinition ) );

	const b3SurfaceMaterial* validMaterials = def.materials;
	def.materials = NULL;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidDefinition ) );
	def.materials = validMaterials;
	const v3BlockGridBlock* validBlocks = def.blocks;
	def.blocks = NULL;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidDefinition ) );
	def.blocks = validBlocks;
	def.materialCount = 0;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidDefinition ) );
	def.materialCount = 1;
	def.blockCount = 0;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidDefinition ) );
	def.blockCount = 1;
	const v3BlockGridBox* validBoxes = block.boxes;
	block.boxes = NULL;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidBlock ) );
	block.boxes = validBoxes;

	boxes[0].materialIndex = 1;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidMaterial ) );

	boxes[0].materialIndex = 0;
	b3AABB validBounds = boxes[0].bounds;
	boxes[0].bounds.upperBound.x = 1.01f;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidBox ) );
	boxes[0].bounds = validBounds;
	boxes[0].bounds.upperBound.x = boxes[0].bounds.lowerBound.x;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidBox ) );
	boxes[0].bounds = validBounds;
	boxes[0].bounds.lowerBound.x = NAN;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidBox ) );
	boxes[0].bounds = validBounds;

	int validBoxCount = block.boxCount;
	block.boxCount = 0;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidBlock ) );
	block.boxCount = 65535;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookLimitExceeded ) );
	block.boxCount = validBoxCount;
	v3BlockGridBlock overflowBlocks[2] = { block, block };
	overflowBlocks[0].boxCount = 32768;
	overflowBlocks[1].x += 1;
	overflowBlocks[1].boxCount = 32767;
	def.blocks = overflowBlocks;
	def.blockCount = 2;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookLimitExceeded ) );

	v3BlockGridBlock duplicateBlocks[2] = { block, block };
	def.blocks = duplicateBlocks;
	def.blockCount = 2;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookDuplicateBlock ) );

	def.blocks = &block;
	def.blockCount = 1;
	int validX = block.x;
	block.x = 16777215;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidBlock ) );
	block.x = validX;

	def.blockCount = 65535;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookLimitExceeded ) );
	def.blockCount = 1;

	float validFriction = material.friction;
	material.friction = INFINITY;
	result = v3CookBlockGrid( &def );
	CHECK( CookFailedAtomically( result, v3_blockGridCookInvalidMaterial ) );
	material.friction = validFriction;

	enum
	{
		projectionBoxCount = 64,
		maximumAllocationAttempts = 128,
	};
	v3BlockGridBox projectionBoxes[projectionBoxCount];
	for ( int i = 0; i < projectionBoxCount; ++i )
	{
		projectionBoxes[i] = (v3BlockGridBox){ .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
	}
	v3BlockGridBlock projectionBlock = { .boxes = projectionBoxes, .boxCount = projectionBoxCount };
	v3BlockGridCookDef projectionDef = {
		.materials = &material,
		.materialCount = 1,
		.blocks = &projectionBlock,
		.blockCount = 1,
	};
	bool reachedSuccessfulAllocation = false;
	for ( allocationFailureIndex = 0; allocationFailureIndex < maximumAllocationAttempts; ++allocationFailureIndex )
	{
		int byteCountBeforeFailure = b3GetByteCount();
		allocationCount = 0;
		b3SetAllocator( FailingAlloc, FailingFree );
		result = v3CookBlockGrid( &projectionDef );
		if ( result.status == v3_blockGridCookOk )
		{
			CHECK( result.data != NULL );
			v3DestroyBlockGridData( result.data );
			b3SetAllocator( NULL, NULL );
			CHECK( b3GetByteCount() == byteCountBeforeFailure );
			reachedSuccessfulAllocation = true;
			break;
		}
		b3SetAllocator( NULL, NULL );
		CHECK( CookFailedAtomically( result, v3_blockGridCookOutOfMemory ) );
		CHECK( b3GetByteCount() == byteCountBeforeFailure );
	}
	CHECK( reachedSuccessfulAllocation );
	CHECK( b3GetByteCount() == byteCountBeforeCook );
	return 0;
}

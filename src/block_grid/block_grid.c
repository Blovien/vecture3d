// SPDX-License-Identifier: MIT

#include "platform.h"
#include "block_grid_internal.h"

#include <math.h>

bool v3BlockGridComputeBlockRange( const v3BlockGridFrame* frame, b3AABB bounds, v3BlockGridRangeConvention convention,
								   int range[6] )
{
	const int origins[3] = { frame->originX, frame->originY, frame->originZ };
	const int counts[3] = { frame->blocksX, frame->blocksY, frame->blocksZ };
	const float lower[3] = { bounds.lowerBound.x, bounds.lowerBound.y, bounds.lowerBound.z };
	const float upper[3] = { bounds.upperBound.x, bounds.upperBound.y, bounds.upperBound.z };

	for ( int axis = 0; axis < 3; ++axis )
	{
		float frameLower = (float)origins[axis];
		float frameUpper = (float)( origins[axis] + counts[axis] );
		if ( lower[axis] <= frameLower )
		{
			range[axis] = 0;
		}
		else if ( lower[axis] >= frameUpper )
		{
			range[axis] = convention == v3_blockGridRangeTouching ? counts[axis] - 1 : counts[axis];
		}
		else
		{
			range[axis] = (int)floorf( lower[axis] ) - origins[axis];
		}

		if ( upper[axis] <= frameLower )
		{
			range[axis + 3] = convention == v3_blockGridRangeTouching ? 0 : -1;
		}
		else if ( upper[axis] >= frameUpper )
		{
			range[axis + 3] = counts[axis] - 1;
		}
		else if ( convention == v3_blockGridRangeTouching )
		{
			range[axis + 3] = (int)floorf( upper[axis] ) - origins[axis];
		}
		else
		{
			range[axis + 3] = (int)ceilf( upper[axis] ) - 1 - origins[axis];
		}
	}

	return range[0] <= range[3] && range[1] <= range[4] && range[2] <= range[5];
}

void v3BlockGrid_Retain( v3BlockGridData* grid )
{
	if ( grid == NULL )
	{
		return;
	}
	// The count is the only mutable state in a shared grid. Retaining from zero
	// means the caller resurrected a freed grid.
	int previous = b3AtomicFetchAddInt( &grid->referenceCount, 1 );
	B3_ASSERT( previous > 0 );
	B3_UNUSED( previous );
}

void v3BlockGrid_Release( v3BlockGridData* grid )
{
	if ( grid == NULL )
	{
		return;
	}
	int previous = b3AtomicFetchAddInt( &grid->referenceCount, -1 );
	B3_ASSERT( previous > 0 );
	if ( previous == 1 )
	{
		// One allocation holds every section, so this is the whole teardown.
		b3Free( grid, (size_t)grid->byteCount );
	}
}

int v3BlockGrid_GetMaterialCount( const v3BlockGridData* grid )
{
	return grid == NULL ? 0 : grid->materialCount;
}

int v3BlockGrid_GetBlockCount( const v3BlockGridData* grid )
{
	return grid == NULL ? 0 : grid->blockCount;
}

int v3BlockGrid_GetHitboxCount( const v3BlockGridData* grid )
{
	return grid == NULL ? 0 : grid->hitboxCount;
}

bool v3BlockGrid_ResolveIdentity( const v3BlockGridData* grid, b3WorldTransform transform, b3Pos point, int hitboxIndex,
								  v3BlockGridIdentity* identity )
{
	if ( grid == NULL || identity == NULL || hitboxIndex < 0 || hitboxIndex >= grid->hitboxCount )
	{
		return false;
	}

	b3Vec3 localPoint = b3InvTransformWorldPoint( transform, point );
	const float coordinate[3] = {
		localPoint.x - (float)grid->worldOriginX,
		localPoint.y - (float)grid->worldOriginY,
		localPoint.z - (float)grid->worldOriginZ,
	};
	int cellMin[3];
	int cellMax[3];
	v3BlockGrid_GetHitboxCellRange( grid, hitboxIndex, cellMin, cellMax );
	int localCell[3];
	for ( int axis = 0; axis < 3; ++axis )
	{
		localCell[axis] = b3ClampInt( (int)floorf( coordinate[axis] ), cellMin[axis], cellMax[axis] );
	}

	uint64_t userData;
	if ( v3BlockGrid_GetCellUserData( grid, localCell[0], localCell[1], localCell[2], &userData ) == false )
	{
		return false;
	}
	uint32_t materialIndex = v3BlockGrid_GetHitboxMaterial( grid, hitboxIndex );
	uint64_t sourceId = v3BlockGrid_GetHitboxSourceId( grid, hitboxIndex );
	const b3SurfaceMaterial* materials = v3BlockGrid_GetMaterials( grid );
	*identity = (v3BlockGridIdentity){
		.cellX = localCell[0] + grid->worldOriginX,
		.cellY = localCell[1] + grid->worldOriginY,
		.cellZ = localCell[2] + grid->worldOriginZ,
		.cellBoxIndex = (int)(uint32_t)sourceId - 1,
		.materialIndex = materialIndex,
		.userMaterialId = materialIndex < (uint32_t)grid->materialCount ? materials[materialIndex].userMaterialId : 0,
		.userData = userData,
	};
	return true;
}

uint64_t v3BlockGrid_GetHitboxSourceId( const v3BlockGridData* grid, int hitboxIndex )
{
	if ( grid == NULL || hitboxIndex < 0 || hitboxIndex >= grid->hitboxCount )
	{
		return 0;
	}
	return v3BlockGridHitboxes( grid )[hitboxIndex].sourceId;
}

uint32_t v3BlockGrid_GetHitboxOwnerBlock( const v3BlockGridData* grid, int hitboxIndex )
{
	if ( grid == NULL || hitboxIndex < 0 || hitboxIndex >= grid->hitboxCount )
	{
		return UINT32_MAX;
	}
	return v3BlockGridHitboxes( grid )[hitboxIndex].ownerBlockIndex;
}

uint32_t v3BlockGrid_GetHitboxMaterial( const v3BlockGridData* grid, int hitboxIndex )
{
	if ( grid == NULL || hitboxIndex < 0 || hitboxIndex >= grid->hitboxCount )
	{
		return UINT32_MAX;
	}
	return v3BlockGridHitboxes( grid )[hitboxIndex].materialIndex;
}

bool v3BlockGrid_GetCellUserData( const v3BlockGridData* grid, int localX, int localY, int localZ, uint64_t* userData )
{
	if ( grid == NULL || userData == NULL )
	{
		return false;
	}

	const v3BlockGridBlockData* blocks = v3BlockGridBlocks( grid );
	int lower = 0;
	int upper = grid->blockCount;
	while ( lower < upper )
	{
		int middle = lower + ( upper - lower ) / 2;
		const v3BlockGridBlockData* block = blocks + middle;
		bool before =
			block->x < localX || ( block->x == localX && ( block->y < localY || ( block->y == localY && block->z < localZ ) ) );
		if ( before )
		{
			lower = middle + 1;
		}
		else
		{
			upper = middle;
		}
	}
	if ( lower >= grid->blockCount )
	{
		return false;
	}
	const v3BlockGridBlockData* block = blocks + lower;
	if ( block->x != localX || block->y != localY || block->z != localZ )
	{
		return false;
	}
	*userData = block->sourceId;
	return true;
}

void v3BlockGrid_GetHitboxCellRange( const v3BlockGridData* grid, int hitboxIndex, int cellMin[3], int cellMax[3] )
{
	if ( cellMin == NULL || cellMax == NULL || grid == NULL || hitboxIndex < 0 || hitboxIndex >= grid->hitboxCount )
	{
		return;
	}
	const v3BlockGridHitboxData* hitbox = v3BlockGridHitboxes( grid ) + hitboxIndex;
	for ( int axis = 0; axis < 3; ++axis )
	{
		cellMin[axis] = hitbox->cellMin[axis];
		cellMax[axis] = hitbox->cellMax[axis];
	}
}

uint32_t v3BlockGrid_GetHitboxFaceMask( const v3BlockGridData* grid, int hitboxIndex )
{
	if ( grid == NULL || hitboxIndex < 0 || hitboxIndex >= grid->hitboxCount )
	{
		return 0;
	}
	return v3BlockGridHitboxes( grid )[hitboxIndex].faceMask;
}

void v3BlockGrid_GetHitboxBounds( const v3BlockGridData* grid, int hitboxIndex, b3Vec3* center, b3Vec3* halfExtent )
{
	if ( center == NULL || halfExtent == NULL )
	{
		return;
	}
	*center = b3Vec3_zero;
	*halfExtent = b3Vec3_zero;
	if ( grid == NULL || hitboxIndex < 0 || hitboxIndex >= grid->hitboxCount )
	{
		return;
	}
	const v3BlockGridHitboxData* hitbox = v3BlockGridHitboxes( grid ) + hitboxIndex;
	*center = b3Add( hitbox->center, v3BlockGridWorldOrigin( grid ) );
	*halfExtent = hitbox->halfExtent;
}

const b3SurfaceMaterial* v3BlockGrid_GetMaterials( const v3BlockGridData* grid )
{
	return grid == NULL ? NULL : v3BlockGridMaterials( grid );
}

b3AABB v3BlockGrid_GetBounds( const v3BlockGridData* grid )
{
	if ( grid == NULL )
	{
		return (b3AABB){ b3Vec3_zero, b3Vec3_zero };
	}
	b3Vec3 origin = v3BlockGridWorldOrigin( grid );
	return (b3AABB){ b3Add( grid->bounds.lowerBound, origin ), b3Add( grid->bounds.upperBound, origin ) };
}

int v3BlockGrid_GetQueryScratchWordCount( const v3BlockGridData* grid )
{
	// One bit per hitbox, rounded up to whole 64-bit words.
	return grid == NULL ? 0 : ( grid->hitboxCount + 63 ) / 64;
}

// Seeds a restored payload with one reference for whoever owns the clone.
// Recording normalizes the stored count to zero at intern time so a live
// reference never travels in durable bytes.
void v3BlockGridAdoptRestored( v3BlockGridData* grid )
{
	b3AtomicStoreInt( &grid->referenceCount, 1 );
}

void v3BlockGridRestorePlacement( v3BlockGridData* grid, int originX, int originY, int originZ, int placement )
{
	grid->worldOriginX = originX;
	grid->worldOriginY = originY;
	grid->worldOriginZ = originZ;
	grid->placement = placement;
}

void v3BlockGrid_GetPlacement( const v3BlockGridData* grid, int* originX, int* originY, int* originZ, int* placement )
{
	*originX = grid->worldOriginX;
	*originY = grid->worldOriginY;
	*originZ = grid->worldOriginZ;
	*placement = grid->placement;
}

bool v3BlockGridPlacementMatches( const v3BlockGridData* grid, int originX, int originY, int originZ, int placement )
{
	return grid->worldOriginX == originX && grid->worldOriginY == originY && grid->worldOriginZ == originZ &&
		   grid->placement == placement;
}

v3BlockGridStats v3BlockGrid_GetStats( const v3BlockGridData* grid )
{
	v3BlockGridStats stats = { 0 };
	if ( grid == NULL )
	{
		return stats;
	}

	stats.blockCount = grid->blockCount;
	stats.hitboxCount = grid->hitboxCount;
	stats.inputHitboxCount = grid->inputBoxCount;
	stats.culledHitboxCount = grid->culledBoxCount;
	stats.mergedHitboxCount = grid->mergedBoxCount;
	stats.groupCount = grid->direct.groupsX * grid->direct.groupsY * grid->direct.groupsZ;
	stats.bounds = v3BlockGrid_GetBounds( grid );
	stats.placement = (v3BlockGridPlacement)grid->placement;

	// Derived rather than stored, so the payload keeps its layout and its hash.
	// The CSR offsets already carry the reference total in their trailing entry.
	const uint32_t* offsets = v3BlockGridHitboxOffsets( grid );
	stats.hitboxReferenceCount = (int)offsets[stats.groupCount];

	// Segment sizes below exclude projection arrays and padding.
	// totalBytes includes the entire cooked allocation.
	stats.headerBytes = (int)sizeof( v3BlockGridData );
	stats.materialBytes = grid->materialCount * (int)sizeof( b3SurfaceMaterial );
	stats.blockBytes = grid->blockCount * (int)sizeof( v3BlockGridBlockData );
	stats.hitboxBytes = grid->hitboxCount * (int)sizeof( v3BlockGridHitboxData );
	stats.occupancyMaskBytes = stats.groupCount * (int)sizeof( uint64_t );
	stats.hitboxOffsetBytes = ( stats.groupCount + 1 ) * (int)sizeof( uint32_t );
	stats.hitboxIndexBytes = stats.hitboxReferenceCount * (int)sizeof( uint32_t );
	stats.totalBytes = grid->byteCount;

	// Occupancy counts too, not just hitbox references. A group whose blocks are
	// all interior still answers occupancy queries.
	const uint64_t* masks = v3BlockGridOccupancyMasks( grid );
	for ( int group = 0; group < stats.groupCount; ++group )
	{
		if ( masks[group] != 0 || offsets[group + 1] > offsets[group] )
		{
			stats.nonEmptyGroupCount += 1;
		}
	}
	return stats;
}

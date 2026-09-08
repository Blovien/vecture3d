// SPDX-License-Identifier: MIT
//
// Cooked BlockGrid layout, shared by the cook and query sides.
//
// One allocation holds everything:
//
//   header | materials | blocks | hitboxes | occupancy masks | hitbox ranges |
//   hitbox indices | cell ranges | cell indices | cell bounds
//
// The three trailing sections belong to the projection grid and exist only when
// a field kept one, since a field too sparse to earn it declines, so nothing may
// read them without checking cellCount first.
//
// Nothing in it is a pointer, so it hashes and compares as bytes and frees in
// one call. Sections are reached by byte offset through the accessors below.

#pragma once

#include "core.h"
#include "v3_block_grid.h"

#include <stddef.h>
#include <stdint.h>

typedef struct v3BlockGridBlockData
{
	int x, y, z;
	uint32_t materialIndex;
	uint64_t sourceId;
	uint32_t flags;

	// This is explicitly zero defined padding to avoid uninitialized memory
	// in the hash.
	uint32_t reserved;
} v3BlockGridBlockData;

typedef struct v3BlockGridHitboxData
{
	b3Vec3 center;
	b3Vec3 halfExtent;
	uint32_t ownerBlockIndex;
	uint32_t materialIndex;
	uint64_t sourceId;

	// The cells this hitbox covers, grid-local and inclusive on both ends. A merged hitbox
	// spans its whole run and an exact sub-cell box covers the one cell it sits in, so a
	// contact or a cast can still name a cell after merging has taken the one-box-per-cell
	// correspondence away. Frame axes stay under V3_BLOCK_GRID_MAX_BLOCKS_PER_AXIS, so a
	// grid-local coordinate fits sixteen bits.
	uint16_t cellMin[3];
	uint16_t cellMax[3];

	// Which of the six axis-aligned faces this hitbox has buried, one bit per face at
	// 2 * axis + positive: bit 0 is -x, bit 1 is +x, and so on up to bit 5 for +z. A face is
	// set only when the neighbouring cells cover it whole, so the narrowphase can drop a
	// shallow contact on it without searching for what covers it. The upper twenty-six bits
	// are always zero, which also keeps this word from carrying uninitialized bytes into the
	// hash the way the padding it replaces did.
	uint32_t faceMask;
} v3BlockGridHitboxData;

// The lattice window the grid covers. Sized at cook time to contain both blocks
// and any protruding box, so no hitbox falls outside the indexed region.
typedef struct v3BlockGridFrame
{
	int originX, originY, originZ;
	int blocksX, blocksY, blocksZ;
} v3BlockGridFrame;

// The Occupancy Group index: one entry per group in the frame, addressed
// arithmetically.
//
// Groups are 4x4x4 blocks so occupancy fits one uint64_t mask and counting is a
// popcount. Hitbox ranges are CSR: hitboxOffsets[slot] to [slot+1] bounds that
// group's slice of hitboxIndices, so the trailing offset entry must exist.
typedef struct v3BlockGridDirectIndex
{
	int groupsX, groupsY, groupsZ;
	uint32_t occupancyMaskOffset;
	uint32_t hitboxOffsetOffset;
	uint32_t hitboxIndexOffset;
} v3BlockGridDirectIndex;

// This grid flattens the field along one axis. It makes a 2D grid of cells.
// Each cell holds a list of the hitboxes above and below it.
//
// A ray moves from cell to cell. It reads only the cells that it crosses.
// The group walk instead reads every group in the box around the ray.
//
// Each hitbox goes into all the cells that it covers. It does not go only into
// the cell at its centre. A wide box must stay visible from all of its area.
//
// This grid is not a height field. A height field keeps one surface for each
// cell. A cell here keeps a list. Therefore caves, decks and overhangs stay
// correct.
//
// The cook selects the axis to flatten. It does not always use Y. The flattened
// axis loses its detail, so the cook flattens the thinnest axis. Terrain
// flattens Y, but a tower does not. The wrong axis was several times slower in
// tests.
//
// Cell coordinates start at zero, as the rest of the payload does. The cook
// origin must stay out of the content. If it did not, the same build in two
// places would give two different hashes.
//
// cellCount is zero when the field has no grid. Rays then use the group walk.
// The group walk is always correct, but it is slower.
typedef struct v3BlockGridProjectionIndex
{
	// The flattened axis. 0 is X, 1 is Y, and 2 is Z. The other two axes keep
	// their order. If the cook flattens Y, the cells use X and then Z.
	int projectionAxis;

	int cellsU, cellsV;
	int cellCount;

	// Cell i holds the indices from offsets[i] to offsets[i + 1]. Thus there is
	// always one more offset than there are cells.
	uint32_t cellOffsetOffset;
	uint32_t cellIndexOffset;

	// The lowest and the highest point of the contents of the cell, measured on
	// the flattened axis. If a ray stays outside these two limits, it skips the
	// cell and does not read the list. A flat ray thus ignores most of a terrain
	// field.
	uint32_t cellNearOffset;
	uint32_t cellFarOffset;
} v3BlockGridProjectionIndex;

// The cooked header, and the start of the allocation.
//
// The reference count and world origin sit ahead of
// V3_BLOCK_GRID_CONTENT_OFFSET, and everything from byteCount on is standard content.
// Once published only that leading trailer is mutable.
//
// NOTE: The same hull cooked at two positions must share one
// hash, so the origin stays out of the payload and rides with the handle.
// Payload coordinates are grid-local and the entry points translate in and out.
struct v3BlockGridData
{
	b3AtomicInt referenceCount;

	// Written after the digest is taken, so it cannot perturb identity.
	int worldOriginX, worldOriginY, worldOriginZ;

	// Caller intent, also outside the digest. Two grids differing only here are
	// the same content.
	int placement;

	// Explicit padding
	int trailerPadding;

	int byteCount;
	uint64_t hash;
	int materialCount;
	int blockCount;

	// Hitboxes the cook kept, which is the length of the hitbox section.
	int hitboxCount;

	// What reduction did, in Cell Boxes: the definition held inputBoxCount of them,
	// culledBoxCount produced no hitbox because their cell is enclosed, mergedBoxCount were
	// folded into a hitbox another cell originated, and hitboxCount is what is left. The three
	// always sum back to the input, so a reader can check the accounting rather than trust it.
	int inputBoxCount;
	int culledBoxCount;
	int mergedBoxCount;

	// Explicit padding, so the header carries no implicit bytes into the hash.
	int countPadding;

	v3BlockGridFrame frame;
	uint32_t materialOffset;
	uint32_t blockOffset;
	uint32_t hitboxOffset;
	v3BlockGridDirectIndex direct;
	v3BlockGridProjectionIndex projection;
	b3AABB bounds;
};

typedef enum v3BlockGridRangeConvention
{
	v3_blockGridRangeTouching,

	// Occupancy treats the upper boundary as outside, so summing adjacent
	// regions does not double count shared faces.
	v3_blockGridRangeHalfOpen,
} v3BlockGridRangeConvention;

typedef struct v3BlockGridQueryCost
{
	uint64_t groups;
	uint64_t references;
} v3BlockGridQueryCost;

typedef enum v3BlockGridQueryCostStatus
{
	v3_blockGridQueryCostInvalid,
	v3_blockGridQueryCostEmpty,
	v3_blockGridQueryCostReady,
	v3_blockGridQueryCostWorkRefused,
} v3BlockGridQueryCostStatus;

_Static_assert( sizeof( v3BlockGridBlockData ) == 32, "BlockGrid block bytes must not contain implicit padding" );
_Static_assert( sizeof( v3BlockGridHitboxData ) == 56, "BlockGrid hitbox layout changed" );
_Static_assert( sizeof( b3SurfaceMaterial ) == 40, "BlockGrid material layout changed" );
_Static_assert( sizeof( v3BlockGridData ) == 184, "BlockGrid packed header layout changed" );

// Where content begins. Hashing and byte comparison both start here,
// skipping the reference count and world origin.
#define V3_BLOCK_GRID_CONTENT_OFFSET offsetof( v3BlockGridData, byteCount )

_Static_assert( V3_BLOCK_GRID_CONTENT_OFFSET == 24, "BlockGrid content region moved" );

// Converts bounds to an inclusive lattice range, lower in [0..2] and upper in
// [3..5], returning false when empty. Endpoints are clipped against the frame
// first, since query bounds can span the float range.
bool v3BlockGridComputeBlockRange( const v3BlockGridFrame* frame, b3AABB bounds, v3BlockGridRangeConvention convention,
								   int range[6] );

// Counts the Occupancy Groups and hitbox references covered by one Direct-index query
// Pair traversal measures both overlap windows and makes the cheaper side its outer query
// The counts also enforce the work limit before candidate enumeration starts
v3BlockGridQueryCostStatus v3BlockGridMeasureDirectQueryCost( const v3BlockGridData* grid, b3AABB bounds, uint64_t maxGroups,
															  v3BlockGridQueryCost* cost );

// Sections are byte offsets from the header, validated at cook time.
static inline const void* v3BlockGridConstPointer( const v3BlockGridData* grid, uint32_t offset )
{
	return (const uint8_t*)grid + offset;
}

static inline void* v3BlockGridPointer( v3BlockGridData* grid, uint32_t offset )
{
	return (uint8_t*)grid + offset;
}

static inline const b3SurfaceMaterial* v3BlockGridMaterials( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->materialOffset );
}

static inline const v3BlockGridHitboxData* v3BlockGridHitboxes( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->hitboxOffset );
}

static inline const v3BlockGridBlockData* v3BlockGridBlocks( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->blockOffset );
}

static inline const uint64_t* v3BlockGridOccupancyMasks( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->direct.occupancyMaskOffset );
}

static inline const uint32_t* v3BlockGridHitboxOffsets( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->direct.hitboxOffsetOffset );
}

static inline const uint32_t* v3BlockGridHitboxIndices( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->direct.hitboxIndexOffset );
}

static inline const uint32_t* v3BlockGridProjectionCellOffsets( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->projection.cellOffsetOffset );
}

static inline const uint32_t* v3BlockGridProjectionCellIndices( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->projection.cellIndexOffset );
}

static inline const float* v3BlockGridProjectionCellNear( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->projection.cellNearOffset );
}

static inline const float* v3BlockGridProjectionCellFar( const v3BlockGridData* grid )
{
	return v3BlockGridConstPointer( grid, grid->projection.cellFarOffset );
}

// The surviving axes for a collapsed one, always in ascending order so cell
// addressing is the same on both the cook and query sides.
static inline void v3BlockGridProjectionAxes( int projectionAxis, int* u, int* v )
{
	*u = projectionAxis == 0 ? 1 : 0;
	*v = projectionAxis == 2 ? 1 : 2;
}

static inline int v3BlockGridGroupIndex( const v3BlockGridDirectIndex* direct, int gx, int gy, int gz )
{
	B3_ASSERT( 0 <= gx && gx < direct->groupsX );
	B3_ASSERT( 0 <= gy && gy < direct->groupsY );
	B3_ASSERT( 0 <= gz && gz < direct->groupsZ );
	return ( gz * direct->groupsY + gy ) * direct->groupsX + gx;
}

// Placement lives outside the payload, so world-space values are translated here
// on the way in and back on the way out. Callers never see grid-local space.
static inline b3Vec3 v3BlockGridWorldOrigin( const v3BlockGridData* grid )
{
	return (b3Vec3){ (float)grid->worldOriginX, (float)grid->worldOriginY, (float)grid->worldOriginZ };
}

static inline b3AABB v3BlockGridToLocalBounds( const v3BlockGridData* grid, b3AABB bounds )
{
	b3Vec3 origin = v3BlockGridWorldOrigin( grid );
	return (b3AABB){ b3Sub( bounds.lowerBound, origin ), b3Sub( bounds.upperBound, origin ) };
}

// Total cooked size, for callers that copy the whole payload such as recording.
static inline int v3BlockGridSnapshotByteCount( const v3BlockGridData* grid )
{
	return grid->byteCount;
}

// Seeds a restored payload with one reference. Recording restores raw bytes and
// normalizes the stored count to zero at intern time, so a live reference never
// travels in durable data.
void v3BlockGridAdoptRestored( v3BlockGridData* grid );

// Placement lives outside the content hash, so a restored payload arrives with a
// zeroed trailer and the snapshot puts it back from the shape record.
void v3BlockGridRestorePlacement( v3BlockGridData* grid, int originX, int originY, int originZ, int placement );
bool v3BlockGridPlacementMatches( const v3BlockGridData* grid, int originX, int originY, int originZ, int placement );

static inline b3Vec3 v3BlockGridHitboxLocalCenter( const v3BlockGridData* grid, int hitboxIndex )
{
	return v3BlockGridHitboxes( grid )[hitboxIndex].center;
}

static inline b3Vec3 v3BlockGridHitboxHalfExtent( const v3BlockGridData* grid, int hitboxIndex )
{
	return v3BlockGridHitboxes( grid )[hitboxIndex].halfExtent;
}

static inline b3AABB v3BlockGridHitboxBounds( const v3BlockGridHitboxData* hitbox )
{
	return (b3AABB){
		.lowerBound = b3Sub( hitbox->center, hitbox->halfExtent ),
		.upperBound = b3Add( hitbox->center, hitbox->halfExtent ),
	};
}

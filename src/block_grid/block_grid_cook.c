// SPDX-License-Identifier: MIT

#include "platform.h"
#include "block_grid_internal.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define V3_BLOCK_GRID_MAX_MATERIALS 65534
#define V3_BLOCK_GRID_MAX_BLOCKS 65534
#define V3_BLOCK_GRID_MAX_HITBOXES 65534
#define V3_BLOCK_GRID_MAX_BLOCKS_PER_AXIS 4096
#define V3_BLOCK_GRID_MAX_GROUPS ( 1 << 20 )

#define V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD 16777215

// Cells a merged hitbox may span on one axis. Merging trades boxes for a coarser bounding box,
// and both the group index and the broad phase prune on bounds, so a run that grew without a
// limit would hand every query the whole field back. Thirty-two cells is one Occupancy Group
// edge times eight, wide enough to collapse a hull face to a handful of boxes and narrow enough
// that a slab still misses most of the field.
#define V3_BLOCK_GRID_MAX_MERGE_CELLS 32

typedef struct v3BlockGridBlockSortEntry
{
	v3BlockGridBlockDef def;
	int inputIndex;
} v3BlockGridBlockSortEntry;

// Reduction state for one occupied cell.
//
// Only cells containing a single full unit cube can be culled or merged. Their faces can fully
// cover adjacent cell faces, and adjacent cubes can form a box. Other cells keep their input boxes.
// The lookup stores the state needed for run growth without reading the larger box list.
typedef struct v3BlockGridCellState
{
	// The cell's sole Cell Box, or -1 for none and -2 for more than one.
	int32_t boxIndex;

	// That box's surface material, meaningful only for a solid cell.
	uint32_t materialIndex;

	uint8_t solid;
	uint8_t enclosed;
	uint8_t claimed;
	uint8_t statePadding;
} v3BlockGridCellState;

// Cell lookup uses open addressing with a capacity that is a power of two and no deletion.
// Storage depends on block count, including for sparse fields with large empty regions.
// Each eight-byte slot packs a grid-local coordinate above its block index. Zero marks an empty slot.
#define V3_BLOCK_GRID_CELL_INDEX_SHIFT 17

_Static_assert( V3_BLOCK_GRID_MAX_BLOCKS < ( 1 << V3_BLOCK_GRID_CELL_INDEX_SHIFT ),
				"BlockGrid cell slots must hold every block index" );

typedef struct v3BlockGridHitboxSortEntry
{
	v3BlockGridBoxDef def;

	// The owner's index after blocks are sorted, not the caller's. This is what
	// gets stored, so a hit still traces back to the right block.
	uint32_t orderedOwnerBlock;

	// The buried faces of this hitbox, one bit per face, or zero for a box that is not exactly a
	// run of whole cells. Carried on the sort entry so it survives the reorder into identity order.
	uint32_t faceMask;
} v3BlockGridHitboxSortEntry;

typedef struct v3BlockGridLayout
{
	size_t cursor;
	bool valid;
	int byteCount;
	uint32_t materialOffset;
	uint32_t blockOffset;
	uint32_t hitboxOffset;
	uint32_t occupancyMaskOffset;
	uint32_t hitboxOffsetOffset;
	uint32_t hitboxIndexOffset;
	uint32_t cellOffsetOffset;
	uint32_t cellIndexOffset;
	uint32_t cellNearOffset;
	uint32_t cellFarOffset;
} v3BlockGridLayout;

// The cook owns every temporary allocation and the unfinished result.
// The public entry point can therefore leave through one cleanup path.
typedef struct v3BlockGridCook
{
	const v3BlockGridDef* def;
	v3BlockGridCreateStatus status;
	int identityCount;
	uint64_t* identities;
	size_t identityBytes;
	v3BlockGridBlockSortEntry* blocks;
	size_t blockBytes;
	uint32_t* blockRemap;
	size_t blockRemapBytes;
	v3BlockGridHitboxSortEntry* hitboxes;
	size_t hitboxBytes;

	// Hitboxes after reduction, which is what every later stage sizes and emits. The reduced
	// list is built beside the input list and copied back over it, because a run reads cells the
	// scan has not reached yet.
	int hitboxCount;
	int culledHitboxCount;
	int mergedHitboxCount;
	v3BlockGridHitboxSortEntry* reducedHitboxes;
	size_t reducedHitboxBytes;
	v3BlockGridCellState* cellStates;
	size_t cellStateBytes;
	uint64_t* cellSlots;
	size_t cellSlotBytes;
	int cellSlotMask;

	uint32_t* groupReferenceCounts;
	size_t groupReferenceBytes;
	uint32_t* groupCursors;
	size_t groupCursorBytes;
	uint32_t* cellReferenceCounts;
	size_t cellReferenceBytes;
	uint32_t* cellCursors;
	size_t cellCursorBytes;
	int projectionAxis;
	int cellsU, cellsV;
	int cellCount;
	uint64_t cellReferenceCount;
	v3BlockGridFrame frame;
	int groupsX, groupsY, groupsZ;
	int groupCount;
	uint64_t hitboxReferenceCount;
	b3AABB bounds;
	v3BlockGridLayout layout;
	v3BlockGridData* grid;
} v3BlockGridCook;

static int v3BlockGridCompareU64( const void* a, const void* b )
{
	uint64_t left = *(const uint64_t*)a;
	uint64_t right = *(const uint64_t*)b;
	return ( left > right ) - ( left < right );
}

static int v3BlockGridCompareBlocks( const void* a, const void* b )
{
	const v3BlockGridBlockSortEntry* left = a;
	const v3BlockGridBlockSortEntry* right = b;
	if ( left->def.x != right->def.x )
	{
		return ( left->def.x > right->def.x ) - ( left->def.x < right->def.x );
	}
	if ( left->def.y != right->def.y )
	{
		return ( left->def.y > right->def.y ) - ( left->def.y < right->def.y );
	}
	return ( left->def.z > right->def.z ) - ( left->def.z < right->def.z );
}

static int v3BlockGridCompareHitboxes( const void* a, const void* b )
{
	const v3BlockGridHitboxSortEntry* left = a;
	const v3BlockGridHitboxSortEntry* right = b;
	return ( left->def.sourceId > right->def.sourceId ) - ( left->def.sourceId < right->def.sourceId );
}

static bool v3BlockGridReject( v3BlockGridCook* cook, v3BlockGridCreateStatus status )
{
	cook->status = status;
	return false;
}

static bool v3BlockGridIsValidMaterial( const b3SurfaceMaterial* material )
{
	return b3IsValidFloat( material->friction ) && material->friction >= 0.0f && b3IsValidFloat( material->restitution ) &&
		   material->restitution >= 0.0f && b3IsValidFloat( material->rollingResistance ) &&
		   material->rollingResistance >= 0.0f && b3IsValidVec3( material->tangentVelocity );
}

static bool v3BlockGridIsRepresentableHitbox( const v3BlockGridBoxDef* hitbox )
{
	double minX = (double)hitbox->center.x - hitbox->halfExtent.x;
	double minY = (double)hitbox->center.y - hitbox->halfExtent.y;
	double minZ = (double)hitbox->center.z - hitbox->halfExtent.z;
	double maxX = (double)hitbox->center.x + hitbox->halfExtent.x;
	double maxY = (double)hitbox->center.y + hitbox->halfExtent.y;
	double maxZ = (double)hitbox->center.z + hitbox->halfExtent.z;
	return minX >= -V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD && minY >= -V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD &&
		   minZ >= -V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD && maxX <= V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD &&
		   maxY <= V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD && maxZ <= V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD;
}

static bool v3BlockGridAllocateInputScratch( v3BlockGridCook* cook )
{
	const v3BlockGridDef* def = cook->def;
	cook->identityCount = def->blockCount + def->boxCount;
	cook->identityBytes = (size_t)cook->identityCount * sizeof( uint64_t );
	cook->blockBytes = (size_t)def->blockCount * sizeof( v3BlockGridBlockSortEntry );
	cook->blockRemapBytes = (size_t)def->blockCount * sizeof( uint32_t );
	cook->hitboxBytes = (size_t)def->boxCount * sizeof( v3BlockGridHitboxSortEntry );
	cook->identities = b3TryAlloc( cook->identityBytes );
	cook->blocks = b3TryAlloc( cook->blockBytes );
	cook->blockRemap = b3TryAlloc( cook->blockRemapBytes );
	cook->hitboxes = b3TryAlloc( cook->hitboxBytes );
	if ( cook->identities == NULL || cook->blocks == NULL || cook->blockRemap == NULL || cook->hitboxes == NULL )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateOutOfMemory );
	}
	return true;
}

static bool v3BlockGridValidateAndCopyInput( v3BlockGridCook* cook )
{
	const v3BlockGridDef* def = cook->def;
	if ( def == NULL || def->materials == NULL || def->blocks == NULL || def->boxes == NULL || def->materialCount <= 0 ||
		 def->blockCount <= 0 || def->boxCount <= 0 )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateInvalidDefinition );
	}
	if ( def->placement != v3_blockGridSectionTile && def->placement != v3_blockGridStandaloneAggregate )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateInvalidDefinition );
	}
	if ( def->materialCount > V3_BLOCK_GRID_MAX_MATERIALS || def->blockCount > V3_BLOCK_GRID_MAX_BLOCKS ||
		 def->boxCount > V3_BLOCK_GRID_MAX_HITBOXES )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateLimitExceeded );
	}
	if ( v3BlockGridAllocateInputScratch( cook ) == false )
	{
		return false;
	}

	for ( int i = 0; i < def->materialCount; ++i )
	{
		if ( v3BlockGridIsValidMaterial( def->materials + i ) == false )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateInvalidMaterial );
		}
	}

	for ( int i = 0; i < def->blockCount; ++i )
	{
		const v3BlockGridBlockDef* block = def->blocks + i;
		if ( block->materialIndex >= (uint32_t)def->materialCount )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateInvalidMaterial );
		}
		if ( block->sourceId == 0 || block->x < -V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD ||
			 block->x >= V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD || block->y < -V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD ||
			 block->y >= V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD || block->z < -V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD ||
			 block->z >= V3_BLOCK_GRID_MAX_EXACT_BLOCK_COORD )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateInvalidBlock );
		}
		cook->identities[i] = block->sourceId;
		cook->blocks[i] = (v3BlockGridBlockSortEntry){ .def = *block, .inputIndex = i };
	}

	for ( int i = 0; i < def->boxCount; ++i )
	{
		const v3BlockGridBoxDef* hitbox = def->boxes + i;
		if ( b3IsValidVec3( hitbox->center ) == false || b3IsValidVec3( hitbox->halfExtent ) == false ||
			 hitbox->halfExtent.x <= 0.0f || hitbox->halfExtent.y <= 0.0f || hitbox->halfExtent.z <= 0.0f ||
			 hitbox->sourceId == 0 )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateInvalidHitbox );
		}
		if ( hitbox->ownerBlockIndex >= (uint32_t)def->blockCount )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateInvalidOwner );
		}
		if ( hitbox->materialIndex >= (uint32_t)def->materialCount )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateInvalidMaterial );
		}
		if ( v3BlockGridIsRepresentableHitbox( hitbox ) == false )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateLimitExceeded );
		}
		cook->identities[def->blockCount + i] = hitbox->sourceId;
		cook->hitboxes[i].def = *hitbox;
		cook->hitboxes[i].faceMask = 0;
	}
	return true;
}

// Puts the input into one fixed order, so the same field always cooks to the
// same bytes no matter how the caller arranged its arrays. That fixed order is
// what gives the content hash meaning: two captures of one assembly compare
// equal and can share an interned grid.
//
// Blocks order by position and hitboxes by stable ID, so caller array order never
// reaches the packed representation. Sorting blocks moves them, which invalidates
// every ownerBlockIndex the caller passed, so owners are remapped through
// blockRemap here.
//
// Duplicates are rejected rather than merged: two blocks at one coordinate have
// no single correct order, so there is no honest way to place them.
static bool v3BlockGridOrderInput( v3BlockGridCook* cook )
{
	const v3BlockGridDef* def = cook->def;

	qsort( cook->identities, (size_t)cook->identityCount, sizeof( uint64_t ), v3BlockGridCompareU64 );
	for ( int i = 1; i < cook->identityCount; ++i )
	{
		if ( cook->identities[i] == cook->identities[i - 1] )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateDuplicateHitbox );
		}
	}

	qsort( cook->blocks, (size_t)def->blockCount, sizeof( v3BlockGridBlockSortEntry ), v3BlockGridCompareBlocks );
	for ( int i = 0; i < def->blockCount; ++i )
	{
		if ( i > 0 && cook->blocks[i - 1].def.x == cook->blocks[i].def.x && cook->blocks[i - 1].def.y == cook->blocks[i].def.y &&
			 cook->blocks[i - 1].def.z == cook->blocks[i].def.z )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateDuplicateBlock );
		}
		cook->blockRemap[cook->blocks[i].inputIndex] = (uint32_t)i;
	}

	for ( int i = 0; i < def->boxCount; ++i )
	{
		cook->hitboxes[i].orderedOwnerBlock = cook->blockRemap[cook->hitboxes[i].def.ownerBlockIndex];
	}
	qsort( cook->hitboxes, (size_t)def->boxCount, sizeof( v3BlockGridHitboxSortEntry ), v3BlockGridCompareHitboxes );
	return true;
}

static bool v3BlockGridComputeFrame( v3BlockGridCook* cook )
{
	const v3BlockGridDef* def = cook->def;

	// The frame covers logical occupancy and any exact box that protrudes from
	// its owner block.
	b3Vec3 lower = { FLT_MAX, FLT_MAX, FLT_MAX };
	b3Vec3 upper = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
	for ( int i = 0; i < def->blockCount; ++i )
	{
		b3Vec3 blockLower = {
			(float)cook->blocks[i].def.x,
			(float)cook->blocks[i].def.y,
			(float)cook->blocks[i].def.z,
		};
		b3Vec3 blockUpper = { blockLower.x + 1.0f, blockLower.y + 1.0f, blockLower.z + 1.0f };
		lower = b3Min( lower, blockLower );
		upper = b3Max( upper, blockUpper );
	}
	for ( int i = 0; i < def->boxCount; ++i )
	{
		lower = b3Min( lower, b3Sub( cook->hitboxes[i].def.center, cook->hitboxes[i].def.halfExtent ) );
		upper = b3Max( upper, b3Add( cook->hitboxes[i].def.center, cook->hitboxes[i].def.halfExtent ) );
	}

	int originX = (int)floorf( lower.x );
	int originY = (int)floorf( lower.y );
	int originZ = (int)floorf( lower.z );
	int64_t blocksX = (int64_t)(int)ceilf( upper.x ) - originX;
	int64_t blocksY = (int64_t)(int)ceilf( upper.y ) - originY;
	int64_t blocksZ = (int64_t)(int)ceilf( upper.z ) - originZ;
	if ( blocksX <= 0 || blocksY <= 0 || blocksZ <= 0 || blocksX > V3_BLOCK_GRID_MAX_BLOCKS_PER_AXIS ||
		 blocksY > V3_BLOCK_GRID_MAX_BLOCKS_PER_AXIS || blocksZ > V3_BLOCK_GRID_MAX_BLOCKS_PER_AXIS )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateLimitExceeded );
	}

	cook->frame = (v3BlockGridFrame){
		.originX = originX,
		.originY = originY,
		.originZ = originZ,
		.blocksX = (int)blocksX,
		.blocksY = (int)blocksY,
		.blocksZ = (int)blocksZ,
	};
	cook->groupsX = ( cook->frame.blocksX + 3 ) / 4;
	cook->groupsY = ( cook->frame.blocksY + 3 ) / 4;
	cook->groupsZ = ( cook->frame.blocksZ + 3 ) / 4;
	int64_t groupCount = (int64_t)cook->groupsX * cook->groupsY * cook->groupsZ;
	if ( groupCount <= 0 || groupCount > V3_BLOCK_GRID_MAX_GROUPS )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateLimitExceeded );
	}
	cook->groupCount = (int)groupCount;
	cook->bounds = (b3AABB){ lower, upper };
	return true;
}

// Reduction works in grid-local lattice coordinates, which the frame keeps below
// V3_BLOCK_GRID_MAX_BLOCKS_PER_AXIS, so all three fit one word with room to spare.
static uint64_t v3BlockGridCellKey( int localX, int localY, int localZ )
{
	return ( ( (uint64_t)localX << 24 ) | ( (uint64_t)localY << 12 ) | (uint64_t)localZ ) + 1;
}

static uint32_t v3BlockGridCellSlotOf( uint64_t key, int mask )
{
	uint64_t mixed = key * UINT64_C( 0x9e3779b97f4a7c15 );
	return (uint32_t)( ( mixed >> 32 ) & (uint64_t)mask );
}

// The occupied cell at this lattice coordinate, or -1 where the field has none.
static int v3BlockGridFindCell( const v3BlockGridCook* cook, int localX, int localY, int localZ )
{
	if ( localX < 0 || localY < 0 || localZ < 0 || localX >= cook->frame.blocksX || localY >= cook->frame.blocksY ||
		 localZ >= cook->frame.blocksZ )
	{
		return -1;
	}
	uint64_t key = v3BlockGridCellKey( localX, localY, localZ );
	uint32_t slot = v3BlockGridCellSlotOf( key, cook->cellSlotMask );
	uint64_t entry = cook->cellSlots[slot];
	while ( entry != 0 )
	{
		if ( entry >> V3_BLOCK_GRID_CELL_INDEX_SHIFT == key )
		{
			return (int)( entry & ( ( UINT64_C( 1 ) << V3_BLOCK_GRID_CELL_INDEX_SHIFT ) - 1 ) );
		}
		slot = ( slot + 1 ) & (uint32_t)cook->cellSlotMask;
		entry = cook->cellSlots[slot];
	}
	return -1;
}

static void v3BlockGridLocalCell( const v3BlockGridCook* cook, int blockIndex, int local[3] )
{
	const v3BlockGridBlockDef* block = &cook->blocks[blockIndex].def;
	local[0] = block->x - cook->frame.originX;
	local[1] = block->y - cook->frame.originY;
	local[2] = block->z - cook->frame.originZ;
}

static bool v3BlockGridAllocateReductionScratch( v3BlockGridCook* cook )
{
	const v3BlockGridDef* def = cook->def;

	// Keep the load factor at or below three quarters without rehashing.
	int capacity = 16;
	while ( capacity < def->blockCount + def->blockCount / 3 )
	{
		capacity *= 2;
	}
	cook->cellSlotMask = capacity - 1;
	cook->cellSlotBytes = (size_t)capacity * sizeof( uint64_t );
	cook->cellStateBytes = (size_t)def->blockCount * sizeof( v3BlockGridCellState );
	cook->reducedHitboxBytes = (size_t)def->boxCount * sizeof( v3BlockGridHitboxSortEntry );
	cook->cellSlots = b3TryAlloc( cook->cellSlotBytes );
	cook->cellStates = b3TryAlloc( cook->cellStateBytes );
	cook->reducedHitboxes = b3TryAlloc( cook->reducedHitboxBytes );
	if ( cook->cellSlots == NULL || cook->cellStates == NULL || cook->reducedHitboxes == NULL )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateOutOfMemory );
	}
	memset( cook->cellSlots, 0, cook->cellSlotBytes );
	memset( cook->cellStates, 0, cook->cellStateBytes );

	for ( int i = 0; i < def->blockCount; ++i )
	{
		int local[3];
		v3BlockGridLocalCell( cook, i, local );
		uint64_t key = v3BlockGridCellKey( local[0], local[1], local[2] );
		uint32_t slot = v3BlockGridCellSlotOf( key, cook->cellSlotMask );
		while ( cook->cellSlots[slot] != 0 )
		{
			slot = ( slot + 1 ) & (uint32_t)cook->cellSlotMask;
		}
		cook->cellSlots[slot] = ( key << V3_BLOCK_GRID_CELL_INDEX_SHIFT ) | (uint64_t)i;
	}
	return true;
}

// Marks the cells whose one box fills their cell exactly. The comparison is exact rather than
// toleranced: a box that merely nearly fills its cell is a different solid, and approximating it
// would move geometry, which the cooker never does.
static void v3BlockGridClassifyCells( v3BlockGridCook* cook )
{
	const v3BlockGridDef* def = cook->def;
	for ( int i = 0; i < def->blockCount; ++i )
	{
		cook->cellStates[i].boxIndex = -1;
	}
	for ( int i = 0; i < def->boxCount; ++i )
	{
		v3BlockGridCellState* cell = cook->cellStates + cook->hitboxes[i].orderedOwnerBlock;
		cell->boxIndex = cell->boxIndex == -1 ? i : -2;
	}
	for ( int i = 0; i < def->blockCount; ++i )
	{
		v3BlockGridCellState* cell = cook->cellStates + i;
		if ( cell->boxIndex < 0 )
		{
			continue;
		}
		const v3BlockGridBoxDef* box = &cook->hitboxes[cell->boxIndex].def;
		const v3BlockGridBlockDef* block = &cook->blocks[i].def;
		bool filled = box->center.x == (float)block->x + 0.5f && box->center.y == (float)block->y + 0.5f &&
					  box->center.z == (float)block->z + 0.5f && box->halfExtent.x == 0.5f && box->halfExtent.y == 0.5f &&
					  box->halfExtent.z == 0.5f;
		cell->solid = filled ? 1 : 0;
		cell->materialIndex = box->materialIndex;
	}
}

// A solid cell whose six face neighbours are solid contributes no hitbox. Every point of its
// surface lies on a face that a neighbour's own face covers exactly, so no body outside the
// aggregate can reach it, and the cell is still a logical block, so mass does not notice.
//
// Only a full cube counts as cover. A neighbour holding sub-cell boxes may leave part of the
// shared face open, and reduction has no cheap way to prove it does not, so the conservative
// answer is to keep the face.
static void v3BlockGridCullEnclosedCells( v3BlockGridCook* cook )
{
	static const int faceOffsets[6][3] = {
		{ 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 },
	};
	for ( int i = 0; i < cook->def->blockCount; ++i )
	{
		if ( cook->cellStates[i].solid == 0 )
		{
			continue;
		}
		int local[3];
		v3BlockGridLocalCell( cook, i, local );
		bool enclosed = true;
		for ( int face = 0; face < 6 && enclosed; ++face )
		{
			int neighbour = v3BlockGridFindCell( cook, local[0] + faceOffsets[face][0], local[1] + faceOffsets[face][1],
												 local[2] + faceOffsets[face][2] );
			enclosed = neighbour >= 0 && cook->cellStates[neighbour].solid != 0;
		}
		cook->cellStates[i].enclosed = enclosed ? 1 : 0;
	}
}

// Whether this cell may join a run that carries the given material. Surface material is the only
// per-hitbox property a grid has; every other collision setting belongs to the shape the grid is
// attached to, so it is shared by construction and cannot separate two cells of one grid.
static bool v3BlockGridCellJoinsRun( const v3BlockGridCook* cook, int localX, int localY, int localZ, uint32_t materialIndex )
{
	int index = v3BlockGridFindCell( cook, localX, localY, localZ );
	if ( index < 0 )
	{
		return false;
	}
	const v3BlockGridCellState* cell = cook->cellStates + index;
	return cell->solid != 0 && cell->enclosed == 0 && cell->claimed == 0 && cell->materialIndex == materialIndex;
}

static void v3BlockGridClaimRun( v3BlockGridCook* cook, const int origin[3], const int span[3] )
{
	for ( int z = 0; z < span[2]; ++z )
	{
		for ( int y = 0; y < span[1]; ++y )
		{
			for ( int x = 0; x < span[0]; ++x )
			{
				int index = v3BlockGridFindCell( cook, origin[0] + x, origin[1] + y, origin[2] + z );
				B3_ASSERT( index >= 0 );
				cook->cellStates[index].claimed = 1;
			}
		}
	}
}

// Grows the run that starts at one cell: as far as it goes along x, then by whole rows along y,
// then by whole layers along z. Taking only complete rows and layers is what keeps the merged
// box exactly the union of its cells, with no cell added that the field does not hold.
static void v3BlockGridGrowRun( const v3BlockGridCook* cook, const int origin[3], uint32_t materialIndex, int span[3] )
{
	span[0] = 1;
	span[1] = 1;
	span[2] = 1;
	while ( span[0] < V3_BLOCK_GRID_MAX_MERGE_CELLS &&
			v3BlockGridCellJoinsRun( cook, origin[0] + span[0], origin[1], origin[2], materialIndex ) )
	{
		span[0] += 1;
	}
	while ( span[1] < V3_BLOCK_GRID_MAX_MERGE_CELLS )
	{
		bool wholeRow = true;
		for ( int x = 0; x < span[0] && wholeRow; ++x )
		{
			wholeRow = v3BlockGridCellJoinsRun( cook, origin[0] + x, origin[1] + span[1], origin[2], materialIndex );
		}
		if ( wholeRow == false )
		{
			break;
		}
		span[1] += 1;
	}
	while ( span[2] < V3_BLOCK_GRID_MAX_MERGE_CELLS )
	{
		bool wholeLayer = true;
		for ( int y = 0; y < span[1] && wholeLayer; ++y )
		{
			for ( int x = 0; x < span[0] && wholeLayer; ++x )
			{
				wholeLayer = v3BlockGridCellJoinsRun( cook, origin[0] + x, origin[1] + y, origin[2] + span[2], materialIndex );
			}
		}
		if ( wholeLayer == false )
		{
			break;
		}
		span[2] += 1;
	}
}

// Whether this cell can bury part of a neighbour's face. Only an exact unit cube can: a sub-cell
// box may leave any part of the shared face open, and a cell holding several boxes has no single
// face at all. An enclosed cell counts even though it produces no Hitbox, because what buries a
// face is the solid on the other side of it, not the box the cook chose to keep for it.
static bool v3BlockGridCellCoversFace( const v3BlockGridCook* cook, int localX, int localY, int localZ )
{
	int index = v3BlockGridFindCell( cook, localX, localY, localZ );
	return index >= 0 && cook->cellStates[index].solid != 0;
}

// The six buried faces of one run, as a bit per face at 2 * axis + positive.
//
// A run is exactly the union of its cells, so each of its faces is exactly the union of that many
// cell faces and is buried when every cell across it is solid. Deriving the answer from cell
// occupancy rather than from Hitbox overlap is what makes a merged slab and its neighbours agree:
// both read the same lattice, so neither the cell count behind a face nor the number of runs that
// happen to hold those cells can change it. The rule stays conservative in the one direction that
// matters, since a face is culled only when it is covered whole.
static uint32_t v3BlockGridRunFaceMask( const v3BlockGridCook* cook, const int origin[3], const int span[3] )
{
	uint32_t mask = 0;
	for ( int axis = 0; axis < 3; ++axis )
	{
		int tangentU = ( axis + 1 ) % 3;
		int tangentV = ( axis + 2 ) % 3;
		for ( int positive = 0; positive < 2; ++positive )
		{
			int neighbour[3] = { origin[0], origin[1], origin[2] };
			neighbour[axis] = positive ? origin[axis] + span[axis] : origin[axis] - 1;
			bool covered = true;
			for ( int u = 0; u < span[tangentU] && covered; ++u )
			{
				for ( int v = 0; v < span[tangentV] && covered; ++v )
				{
					int cell[3] = { neighbour[0], neighbour[1], neighbour[2] };
					cell[tangentU] = origin[tangentU] + u;
					cell[tangentV] = origin[tangentV] + v;
					covered = v3BlockGridCellCoversFace( cook, cell[0], cell[1], cell[2] );
				}
			}
			mask |= covered ? ( 1u << ( 2 * axis + positive ) ) : 0u;
		}
	}
	return mask;
}

// Greedily merges surviving full cells into runs in cooked block order. Other boxes remain unchanged.
// Runs grow along x, then y, then z, subject to matching material and the 32-cell size limit.
// The fixed order makes the result deterministic. Each run inherits its starting cell's unique identity.
static bool v3BlockGridMergeCells( v3BlockGridCook* cook )
{
	const v3BlockGridDef* def = cook->def;
	int emitted = 0;
	for ( int i = 0; i < def->blockCount; ++i )
	{
		const v3BlockGridCellState* cell = cook->cellStates + i;
		if ( cell->solid == 0 || cell->enclosed != 0 || cell->claimed != 0 )
		{
			continue;
		}
		int origin[3];
		v3BlockGridLocalCell( cook, i, origin );
		v3BlockGridHitboxSortEntry run = cook->hitboxes[cell->boxIndex];
		int span[3];
		v3BlockGridGrowRun( cook, origin, cell->materialIndex, span );
		v3BlockGridClaimRun( cook, origin, span );

		const v3BlockGridBlockDef* block = &cook->blocks[i].def;
		run.def.center = (b3Vec3){
			(float)block->x + 0.5f * (float)span[0],
			(float)block->y + 0.5f * (float)span[1],
			(float)block->z + 0.5f * (float)span[2],
		};
		run.def.halfExtent = (b3Vec3){ 0.5f * (float)span[0], 0.5f * (float)span[1], 0.5f * (float)span[2] };
		run.faceMask = v3BlockGridRunFaceMask( cook, origin, span );
		run.orderedOwnerBlock = (uint32_t)i;
		cook->reducedHitboxes[emitted++] = run;
		cook->mergedHitboxCount += span[0] * span[1] * span[2] - 1;
	}

	for ( int i = 0; i < def->boxCount; ++i )
	{
		if ( cook->cellStates[cook->hitboxes[i].orderedOwnerBlock].solid != 0 )
		{
			continue;
		}
		cook->reducedHitboxes[emitted++] = cook->hitboxes[i];
	}

	for ( int i = 0; i < def->blockCount; ++i )
	{
		cook->culledHitboxCount += cook->cellStates[i].enclosed;
	}

	// Every Cell Box either survives as a hitbox, is culled, or is folded into another cell's
	// run, so the three have to account for the whole input.
	B3_ASSERT( emitted + cook->culledHitboxCount + cook->mergedHitboxCount == def->boxCount );
	cook->hitboxCount = emitted;
	memcpy( cook->hitboxes, cook->reducedHitboxes, (size_t)emitted * sizeof( v3BlockGridHitboxSortEntry ) );

	// Back into stable identity order, the order the cooked payload and every query rely on.
	qsort( cook->hitboxes, (size_t)emitted, sizeof( v3BlockGridHitboxSortEntry ), v3BlockGridCompareHitboxes );
	return true;
}

// Culls enclosed geometry and greedily merges remaining full cells, preserving the exposed collision surface.
// Logical cells remain unchanged, so mass and inertia are unaffected.
static bool v3BlockGridReduceGeometry( v3BlockGridCook* cook )
{
	cook->hitboxCount = cook->def->boxCount;
	if ( v3BlockGridAllocateReductionScratch( cook ) == false )
	{
		return false;
	}
	v3BlockGridClassifyCells( cook );
	v3BlockGridCullEnclosedCells( cook );
	return v3BlockGridMergeCells( cook );
}

static bool v3BlockGridComputeHitboxRange( const v3BlockGridCook* cook, int hitboxIndex, int range[6] )
{
	const v3BlockGridBoxDef* hitbox = &cook->hitboxes[hitboxIndex].def;
	b3AABB bounds = {
		.lowerBound = b3Sub( hitbox->center, hitbox->halfExtent ),
		.upperBound = b3Add( hitbox->center, hitbox->halfExtent ),
	};
	return v3BlockGridComputeBlockRange( &cook->frame, bounds, v3_blockGridRangeTouching, range );
}

// Maximum projection storage as a multiple of hitbox storage.
// The byte limit includes cell offsets for empty cells as well as hitbox references.
#define V3_BLOCK_GRID_MAX_PROJECTION_PAYLOAD_SHARE 1

// The minimum number of hitboxes for a projection grid. A smaller field is
// already fast with the group walk. A grid would add bytes and cook time, but
// it would remove only a few tests.
#define V3_BLOCK_GRID_MIN_PROJECTION_HITBOXES 64

// Returns the covered projection cells as { u0, v0, u1, v1 }, with inclusive limits.
// The range is empty when u1 < u0. The hitbox's upper edge is open: a box ending on a
// cell edge does not occupy the next cell. Every overlapped cell receives a reference.
static void v3BlockGridProjectionFootprint( const v3BlockGridCook* cook, int hitboxIndex, int span[4] )
{
	const v3BlockGridBoxDef* hitbox = &cook->hitboxes[hitboxIndex].def;
	const float center[3] = { hitbox->center.x, hitbox->center.y, hitbox->center.z };
	const float extent[3] = { hitbox->halfExtent.x, hitbox->halfExtent.y, hitbox->halfExtent.z };
	const int origin[3] = { cook->frame.originX, cook->frame.originY, cook->frame.originZ };
	const int limit[2] = { cook->cellsU, cook->cellsV };
	int axis[2];
	v3BlockGridProjectionAxes( cook->projectionAxis, &axis[0], &axis[1] );

	for ( int i = 0; i < 2; ++i )
	{
		int a = axis[i];
		int lower = (int)floorf( center[a] - extent[a] ) - origin[a];
		int upper = (int)ceilf( center[a] + extent[a] ) - 1 - origin[a];
		span[i] = lower < 0 ? 0 : lower;
		span[i + 2] = upper >= limit[i] ? limit[i] - 1 : upper;
	}
}

// Selects the thinnest axis to flatten.
static int v3BlockGridChooseProjectionAxis( const v3BlockGridCook* cook )
{
	int blocks[3] = { cook->frame.blocksX, cook->frame.blocksY, cook->frame.blocksZ };
	int axis = 0;
	for ( int i = 1; i < 3; ++i )
	{
		if ( blocks[i] < blocks[axis] )
		{
			axis = i;
		}
	}
	return axis;
}

// Measures the size of the projection grid and decides if the field gets one.
//
// If the field does not get a grid, cellCount stays zero and all queries use
// the group walk. This function can thus refuse for any reason. The results of
// the queries do not change.
static bool v3BlockGridCountProjectionReferences( v3BlockGridCook* cook )
{
	if ( cook->hitboxCount < V3_BLOCK_GRID_MIN_PROJECTION_HITBOXES )
	{
		return true;
	}

	cook->projectionAxis = v3BlockGridChooseProjectionAxis( cook );
	int blocks[3] = { cook->frame.blocksX, cook->frame.blocksY, cook->frame.blocksZ };
	int axisU, axisV;
	v3BlockGridProjectionAxes( cook->projectionAxis, &axisU, &axisV );
	cook->cellsU = blocks[axisU];
	cook->cellsV = blocks[axisV];

	int64_t cellCount = (int64_t)cook->cellsU * cook->cellsV;
	if ( cellCount <= 0 || cellCount > V3_BLOCK_GRID_MAX_GROUPS )
	{
		return true;
	}

	// The grid pays for each cell, also for the empty ones. A field with few
	// hitboxes in a large frame thus stops here, before the count starts.
	uint64_t hitboxBytes = (uint64_t)cook->hitboxCount * sizeof( v3BlockGridHitboxData );
	uint64_t budget = hitboxBytes * V3_BLOCK_GRID_MAX_PROJECTION_PAYLOAD_SHARE;
	uint64_t fixedBytes = (uint64_t)cellCount * ( sizeof( uint32_t ) + 2 * sizeof( float ) ) + sizeof( uint32_t );
	if ( fixedBytes > budget )
	{
		return true;
	}

	cook->cellReferenceBytes = (size_t)cellCount * sizeof( uint32_t );
	cook->cellReferenceCounts = b3TryAlloc( cook->cellReferenceBytes );
	if ( cook->cellReferenceCounts == NULL )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateOutOfMemory );
	}
	memset( cook->cellReferenceCounts, 0, cook->cellReferenceBytes );

	uint64_t referenceBudget = ( budget - fixedBytes ) / sizeof( uint32_t );
	for ( int hitbox = 0; hitbox < cook->hitboxCount; ++hitbox )
	{
		int span[4];
		v3BlockGridProjectionFootprint( cook, hitbox, span );
		for ( int v = span[1]; v <= span[3]; ++v )
		{
			for ( int u = span[0]; u <= span[2]; ++u )
			{
				cook->cellReferenceCounts[(size_t)v * cook->cellsU + u] += 1;
				cook->cellReferenceCount += 1;
			}
		}
		if ( cook->cellReferenceCount > referenceBudget )
		{
			// The grid is too large. Release it and use the group walk for
			// this field. The release stays here to keep the decision in one
			// place.
			b3Free( cook->cellReferenceCounts, cook->cellReferenceBytes );
			cook->cellReferenceCounts = NULL;
			cook->cellReferenceBytes = 0;
			cook->cellReferenceCount = 0;
			return true;
		}
	}

	cook->cellCount = (int)cellCount;
	return true;
}

static bool v3BlockGridCountHitboxReferences( v3BlockGridCook* cook )
{
	cook->groupReferenceBytes = (size_t)cook->groupCount * sizeof( uint32_t );
	cook->groupReferenceCounts = b3TryAlloc( cook->groupReferenceBytes );
	if ( cook->groupReferenceCounts == NULL )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateOutOfMemory );
	}
	memset( cook->groupReferenceCounts, 0, cook->groupReferenceBytes );

	v3BlockGridDirectIndex direct = {
		.groupsX = cook->groupsX,
		.groupsY = cook->groupsY,
		.groupsZ = cook->groupsZ,
	};

	// This pass sizes compact group-to-hitbox ranges before the immutable
	// allocation is emitted. The fill pass below uses the same range helper.
	for ( int hitbox = 0; hitbox < cook->hitboxCount; ++hitbox )
	{
		int range[6];
		if ( v3BlockGridComputeHitboxRange( cook, hitbox, range ) == false )
		{
			return v3BlockGridReject( cook, v3_blockGridCreateInvalidHitbox );
		}
		for ( int gz = range[2] >> 2; gz <= range[5] >> 2; ++gz )
		{
			for ( int gy = range[1] >> 2; gy <= range[4] >> 2; ++gy )
			{
				for ( int gx = range[0] >> 2; gx <= range[3] >> 2; ++gx )
				{
					int group = v3BlockGridGroupIndex( &direct, gx, gy, gz );
					if ( cook->groupReferenceCounts[group] == UINT32_MAX || cook->hitboxReferenceCount == UINT32_MAX )
					{
						return v3BlockGridReject( cook, v3_blockGridCreateLimitExceeded );
					}
					cook->groupReferenceCounts[group] += 1;
					cook->hitboxReferenceCount += 1;
				}
			}
		}
	}
	return true;
}

static void v3BlockGridLayoutReserve( v3BlockGridLayout* layout, size_t count, size_t elementSize, size_t alignment,
									  uint32_t* offset )
{
	if ( layout->valid == false )
	{
		return;
	}
	B3_ASSERT( alignment > 0 && ( alignment & ( alignment - 1 ) ) == 0 );
	if ( count > 0 && elementSize > SIZE_MAX / count )
	{
		layout->valid = false;
		return;
	}
	size_t bytes = count * elementSize;
	if ( layout->cursor > SIZE_MAX - ( alignment - 1 ) )
	{
		layout->valid = false;
		return;
	}
	size_t aligned = ( layout->cursor + alignment - 1 ) & ~( alignment - 1 );
	if ( aligned > INT_MAX || bytes > INT_MAX || aligned > (size_t)INT_MAX - bytes )
	{
		layout->valid = false;
		return;
	}
	*offset = (uint32_t)aligned;
	layout->cursor = aligned + bytes;
}

static bool v3BlockGridPlanLayout( v3BlockGridCook* cook )
{
	v3BlockGridLayout* layout = &cook->layout;

	// Offsets are relative to the allocation base. Keeping the payload free of
	// pointers makes the same bytes usable by hashing and recording.
	*layout = (v3BlockGridLayout){ .cursor = sizeof( v3BlockGridData ), .valid = true };
	v3BlockGridLayoutReserve( layout, (size_t)cook->def->materialCount, sizeof( b3SurfaceMaterial ),
							  _Alignof( b3SurfaceMaterial ), &layout->materialOffset );
	v3BlockGridLayoutReserve( layout, (size_t)cook->def->blockCount, sizeof( v3BlockGridBlockData ),
							  _Alignof( v3BlockGridBlockData ), &layout->blockOffset );
	v3BlockGridLayoutReserve( layout, (size_t)cook->hitboxCount, sizeof( v3BlockGridHitboxData ),
							  _Alignof( v3BlockGridHitboxData ), &layout->hitboxOffset );
	v3BlockGridLayoutReserve( layout, (size_t)cook->groupCount, sizeof( uint64_t ), _Alignof( uint64_t ),
							  &layout->occupancyMaskOffset );
	v3BlockGridLayoutReserve( layout, (size_t)cook->groupCount + 1, sizeof( uint32_t ), _Alignof( uint32_t ),
							  &layout->hitboxOffsetOffset );
	v3BlockGridLayoutReserve( layout, (size_t)cook->hitboxReferenceCount, sizeof( uint32_t ), _Alignof( uint32_t ),
							  &layout->hitboxIndexOffset );

	// Reserved only if the field has a projection grid. If it does not, these
	// offsets stay zero. No function reads them, because cellCount controls
	// all access.
	if ( cook->cellCount > 0 )
	{
		v3BlockGridLayoutReserve( layout, (size_t)cook->cellCount + 1, sizeof( uint32_t ), _Alignof( uint32_t ),
								  &layout->cellOffsetOffset );
		v3BlockGridLayoutReserve( layout, (size_t)cook->cellReferenceCount, sizeof( uint32_t ), _Alignof( uint32_t ),
								  &layout->cellIndexOffset );
		v3BlockGridLayoutReserve( layout, (size_t)cook->cellCount, sizeof( float ), _Alignof( float ), &layout->cellNearOffset );
		v3BlockGridLayoutReserve( layout, (size_t)cook->cellCount, sizeof( float ), _Alignof( float ), &layout->cellFarOffset );
	}
	if ( layout->valid == false || layout->cursor > (size_t)INT_MAX - ( B3_ALIGNMENT - 1 ) )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateLimitExceeded );
	}
	layout->byteCount = (int)( ( layout->cursor + B3_ALIGNMENT - 1 ) & ~(size_t)( B3_ALIGNMENT - 1 ) );
	return true;
}

// The cells one hitbox covers, grid-local and inclusive on both ends.
//
// The upper edge is open, the same rule the projection footprint uses: a box stopping exactly on
// a cell boundary does not claim the next cell, so a run of four cells reports four and a
// sub-cell box reports the single cell it sits in. Endpoints are clipped to the frame, which
// only matters for a box the caller pushed past the lattice it declared.
static void v3BlockGridHitboxCellRange( const v3BlockGridCook* cook, const v3BlockGridBoxDef* hitbox, uint16_t cellMin[3],
										uint16_t cellMax[3] )
{
	const float center[3] = { hitbox->center.x, hitbox->center.y, hitbox->center.z };
	const float extent[3] = { hitbox->halfExtent.x, hitbox->halfExtent.y, hitbox->halfExtent.z };
	const int origin[3] = { cook->frame.originX, cook->frame.originY, cook->frame.originZ };
	const int limit[3] = { cook->frame.blocksX, cook->frame.blocksY, cook->frame.blocksZ };
	for ( int axis = 0; axis < 3; ++axis )
	{
		int lower = (int)floorf( center[axis] - extent[axis] ) - origin[axis];
		int upper = (int)ceilf( center[axis] + extent[axis] ) - 1 - origin[axis];
		lower = lower < 0 ? 0 : lower;
		upper = upper >= limit[axis] ? limit[axis] - 1 : upper;
		cellMin[axis] = (uint16_t)lower;
		cellMax[axis] = (uint16_t)( upper < lower ? lower : upper );
	}
}

// The frame origin is subtracted once here, so two translations of the same assembly
// cook to identical bytes and share one content hash.
// Callers keep working in their own space because queries translate through the frame
// origin on the way in and out.
static void v3BlockGridEmitBlocksAndHitboxes( v3BlockGridCook* cook )
{
	const b3Vec3 gridOrigin = {
		(float)cook->frame.originX,
		(float)cook->frame.originY,
		(float)cook->frame.originZ,
	};

	v3BlockGridBlockData* blocks = v3BlockGridPointer( cook->grid, cook->layout.blockOffset );
	for ( int i = 0; i < cook->def->blockCount; ++i )
	{
		blocks[i] = (v3BlockGridBlockData){
			.x = cook->blocks[i].def.x - cook->frame.originX,
			.y = cook->blocks[i].def.y - cook->frame.originY,
			.z = cook->blocks[i].def.z - cook->frame.originZ,
			.materialIndex = cook->blocks[i].def.materialIndex,
			.sourceId = cook->blocks[i].def.sourceId,
			.flags = cook->blocks[i].def.flags,
			.reserved = 0,
		};
	}

	v3BlockGridHitboxData* hitboxes = v3BlockGridPointer( cook->grid, cook->layout.hitboxOffset );
	for ( int i = 0; i < cook->hitboxCount; ++i )
	{
		hitboxes[i] = (v3BlockGridHitboxData){
			.center = b3Sub( cook->hitboxes[i].def.center, gridOrigin ),
			.halfExtent = cook->hitboxes[i].def.halfExtent,
			.ownerBlockIndex = cook->hitboxes[i].orderedOwnerBlock,
			.materialIndex = cook->hitboxes[i].def.materialIndex,
			.sourceId = cook->hitboxes[i].def.sourceId,
			.faceMask = cook->hitboxes[i].faceMask,
		};
		v3BlockGridHitboxCellRange( cook, &cook->hitboxes[i].def, hitboxes[i].cellMin, hitboxes[i].cellMax );
	}
}

// Writes the hitbox list and the two limits of each cell.
//
// Each hitbox goes into all the cells that it covers. A large box thus stays
// visible from all of them, and not only from the cell at its centre.
static bool v3BlockGridEmitProjectionIndex( v3BlockGridCook* cook )
{
	if ( cook->cellCount <= 0 )
	{
		return true;
	}

	uint32_t* cellOffsets = v3BlockGridPointer( cook->grid, cook->layout.cellOffsetOffset );
	uint32_t* cellIndices = v3BlockGridPointer( cook->grid, cook->layout.cellIndexOffset );
	float* nearBound = v3BlockGridPointer( cook->grid, cook->layout.cellNearOffset );
	float* farBound = v3BlockGridPointer( cook->grid, cook->layout.cellFarOffset );

	uint32_t running = 0;
	for ( int cell = 0; cell < cook->cellCount; ++cell )
	{
		cellOffsets[cell] = running;
		running += cook->cellReferenceCounts[cell];

		// An empty cell keeps its limits in the wrong order. No ray can be
		// inside them, so the cell needs no test for the empty condition.
		nearBound[cell] = FLT_MAX;
		farBound[cell] = -FLT_MAX;
	}
	cellOffsets[cook->cellCount] = running;

	cook->cellCursorBytes = (size_t)cook->cellCount * sizeof( uint32_t );
	cook->cellCursors = b3TryAlloc( cook->cellCursorBytes );
	if ( cook->cellCursors == NULL )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateOutOfMemory );
	}
	memcpy( cook->cellCursors, cellOffsets, cook->cellCursorBytes );

	const v3BlockGridHitboxData* hitboxes = v3BlockGridHitboxes( cook->grid );
	int collapsed = cook->projectionAxis;
	for ( int hitbox = 0; hitbox < cook->hitboxCount; ++hitbox )
	{
		int span[4];
		v3BlockGridProjectionFootprint( cook, hitbox, span );
		const float center[3] = { hitboxes[hitbox].center.x, hitboxes[hitbox].center.y, hitboxes[hitbox].center.z };
		const float extent[3] = { hitboxes[hitbox].halfExtent.x, hitboxes[hitbox].halfExtent.y, hitboxes[hitbox].halfExtent.z };
		float low = center[collapsed] - extent[collapsed];
		float high = center[collapsed] + extent[collapsed];
		for ( int v = span[1]; v <= span[3]; ++v )
		{
			for ( int u = span[0]; u <= span[2]; ++u )
			{
				size_t cell = (size_t)v * cook->cellsU + u;
				cellIndices[cook->cellCursors[cell]++] = (uint32_t)hitbox;
				if ( low < nearBound[cell] )
				{
					nearBound[cell] = low;
				}
				if ( high > farBound[cell] )
				{
					farBound[cell] = high;
				}
			}
		}
	}

	for ( int cell = 0; cell < cook->cellCount; ++cell )
	{
		// The count step and the write step use the same cells. If a cursor
		// stops before the next offset, the list does not agree with the
		// field.
		B3_ASSERT( cook->cellCursors[cell] == cellOffsets[cell + 1] );
	}
	return true;
}

static bool v3BlockGridEmitDirectIndex( v3BlockGridCook* cook )
{
	uint64_t* occupancyMasks = v3BlockGridPointer( cook->grid, cook->layout.occupancyMaskOffset );
	for ( int i = 0; i < cook->def->blockCount; ++i )
	{
		int x = cook->blocks[i].def.x - cook->frame.originX;
		int y = cook->blocks[i].def.y - cook->frame.originY;
		int z = cook->blocks[i].def.z - cook->frame.originZ;
		int group = v3BlockGridGroupIndex( &cook->grid->direct, x >> 2, y >> 2, z >> 2 );

		// Occupancy bits run x first, then y, then z inside each 4x4x4 group.
		int bit = ( ( z & 3 ) << 4 ) | ( ( y & 3 ) << 2 ) | ( x & 3 );
		occupancyMasks[group] |= (uint64_t)1 << bit;
	}

	uint32_t* hitboxOffsets = v3BlockGridPointer( cook->grid, cook->layout.hitboxOffsetOffset );
	for ( int group = 0; group < cook->groupCount; ++group )
	{
		hitboxOffsets[group + 1] = hitboxOffsets[group] + cook->groupReferenceCounts[group];
	}
	cook->groupCursorBytes = (size_t)cook->groupCount * sizeof( uint32_t );
	cook->groupCursors = b3TryAlloc( cook->groupCursorBytes );
	if ( cook->groupCursors == NULL )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateOutOfMemory );
	}
	memcpy( cook->groupCursors, hitboxOffsets, cook->groupCursorBytes );

	uint32_t* hitboxIndices = v3BlockGridPointer( cook->grid, cook->layout.hitboxIndexOffset );
	for ( int hitbox = 0; hitbox < cook->hitboxCount; ++hitbox )
	{
		int range[6];
		bool validRange = v3BlockGridComputeHitboxRange( cook, hitbox, range );
		B3_ASSERT( validRange );
		B3_UNUSED( validRange );
		for ( int gz = range[2] >> 2; gz <= range[5] >> 2; ++gz )
		{
			for ( int gy = range[1] >> 2; gy <= range[4] >> 2; ++gy )
			{
				for ( int gx = range[0] >> 2; gx <= range[3] >> 2; ++gx )
				{
					int group = v3BlockGridGroupIndex( &cook->grid->direct, gx, gy, gz );
					hitboxIndices[cook->groupCursors[group]++] = (uint32_t)hitbox;
				}
			}
		}
	}
	for ( int group = 0; group < cook->groupCount; ++group )
	{
		// Counting and filling use the same hitbox range. Matching endpoints
		// guard that invariant before the cooked grid is published.
		B3_ASSERT( cook->groupCursors[group] == hitboxOffsets[group + 1] );
	}
	return true;
}

static bool v3BlockGridEmit( v3BlockGridCook* cook )
{
	cook->grid = b3TryAlloc( (size_t)cook->layout.byteCount );
	if ( cook->grid == NULL )
	{
		return v3BlockGridReject( cook, v3_blockGridCreateOutOfMemory );
	}
	memset( cook->grid, 0, (size_t)cook->layout.byteCount );
	cook->grid->byteCount = cook->layout.byteCount;
	cook->grid->materialCount = cook->def->materialCount;
	cook->grid->blockCount = cook->def->blockCount;
	cook->grid->hitboxCount = cook->hitboxCount;
	cook->grid->inputBoxCount = cook->def->boxCount;
	cook->grid->culledBoxCount = cook->culledHitboxCount;
	cook->grid->mergedBoxCount = cook->mergedHitboxCount;
	cook->grid->countPadding = 0;
	// Payload coordinates start at zero. The caller's cook origin is stored in
	// the mutable header fields.
	cook->grid->frame = (v3BlockGridFrame){
		.originX = 0,
		.originY = 0,
		.originZ = 0,
		.blocksX = cook->frame.blocksX,
		.blocksY = cook->frame.blocksY,
		.blocksZ = cook->frame.blocksZ,
	};
	cook->grid->worldOriginX = cook->frame.originX;
	cook->grid->worldOriginY = cook->frame.originY;
	cook->grid->worldOriginZ = cook->frame.originZ;

	cook->grid->placement = (int)cook->def->placement;
	cook->grid->materialOffset = cook->layout.materialOffset;
	cook->grid->blockOffset = cook->layout.blockOffset;
	cook->grid->hitboxOffset = cook->layout.hitboxOffset;
	cook->grid->direct = (v3BlockGridDirectIndex){
		.groupsX = cook->groupsX,
		.groupsY = cook->groupsY,
		.groupsZ = cook->groupsZ,
		.occupancyMaskOffset = cook->layout.occupancyMaskOffset,
		.hitboxOffsetOffset = cook->layout.hitboxOffsetOffset,
		.hitboxIndexOffset = cook->layout.hitboxIndexOffset,
	};
	cook->grid->projection = (v3BlockGridProjectionIndex){
		.projectionAxis = cook->projectionAxis,
		.cellsU = cook->cellCount > 0 ? cook->cellsU : 0,
		.cellsV = cook->cellCount > 0 ? cook->cellsV : 0,
		.cellCount = cook->cellCount,
		.cellOffsetOffset = cook->layout.cellOffsetOffset,
		.cellIndexOffset = cook->layout.cellIndexOffset,
		.cellNearOffset = cook->layout.cellNearOffset,
		.cellFarOffset = cook->layout.cellFarOffset,
	};
	const b3Vec3 frameOrigin = {
		(float)cook->frame.originX,
		(float)cook->frame.originY,
		(float)cook->frame.originZ,
	};
	cook->grid->bounds = (b3AABB){
		b3Sub( cook->bounds.lowerBound, frameOrigin ),
		b3Sub( cook->bounds.upperBound, frameOrigin ),
	};

	// Callers assemble materials in many ways and a stray byte there would both reject
	// a valid capture and change the hash of otherwise identical content, so cooking normalizes it.
	b3SurfaceMaterial* materials = v3BlockGridPointer( cook->grid, cook->layout.materialOffset );
	memcpy( materials, cook->def->materials, (size_t)cook->def->materialCount * sizeof( b3SurfaceMaterial ) );
	for ( int i = 0; i < cook->def->materialCount; ++i )
	{
		materials[i].padding = 0;
	}
	v3BlockGridEmitBlocksAndHitboxes( cook );
	if ( v3BlockGridEmitDirectIndex( cook ) == false )
	{
		return false;
	}

	if ( v3BlockGridEmitProjectionIndex( cook ) == false )
	{
		return false;
	}

	cook->grid->referenceCount.value = 0;
	cook->grid->hash = 0;
	cook->grid->hash = b3Hash64NonZero( (const uint8_t*)cook->grid + V3_BLOCK_GRID_CONTENT_OFFSET,
										cook->grid->byteCount - (int)V3_BLOCK_GRID_CONTENT_OFFSET );
	b3AtomicStoreInt( &cook->grid->referenceCount, 1 );
	cook->status = v3_blockGridCreateOk;
	return true;
}

static void v3BlockGridFreeCook( v3BlockGridCook* cook )
{
	b3Free( cook->identities, cook->identityBytes );
	b3Free( cook->blocks, cook->blockBytes );
	b3Free( cook->blockRemap, cook->blockRemapBytes );
	b3Free( cook->hitboxes, cook->hitboxBytes );
	b3Free( cook->reducedHitboxes, cook->reducedHitboxBytes );
	b3Free( cook->cellStates, cook->cellStateBytes );
	b3Free( cook->cellSlots, cook->cellSlotBytes );
	b3Free( cook->groupReferenceCounts, cook->groupReferenceBytes );
	b3Free( cook->groupCursors, cook->groupCursorBytes );
	b3Free( cook->cellReferenceCounts, cook->cellReferenceBytes );
	b3Free( cook->cellCursors, cook->cellCursorBytes );
	if ( cook->grid != NULL )
	{
		b3Free( cook->grid, (size_t)cook->grid->byteCount );
	}
}

v3BlockGridData* v3BlockGrid_Create( const v3BlockGridDef* def, v3BlockGridCreateStatus* status )
{
	v3BlockGridCook cook = {
		.def = def,
		.status = v3_blockGridCreateInvalidDefinition,
	};
	v3BlockGridData* result = NULL;
	// Reduction runs after the frame and before every stage that sizes itself on the hitbox
	// list. The frame is unaffected by it: a merged box is the union of its cells and a culled
	// box is one cell, so both stay inside bounds the block loop already covered.
	if ( v3BlockGridValidateAndCopyInput( &cook ) == false || v3BlockGridOrderInput( &cook ) == false ||
		 v3BlockGridComputeFrame( &cook ) == false || v3BlockGridReduceGeometry( &cook ) == false ||
		 v3BlockGridCountHitboxReferences( &cook ) == false || v3BlockGridCountProjectionReferences( &cook ) == false ||
		 v3BlockGridPlanLayout( &cook ) == false || v3BlockGridEmit( &cook ) == false )
	{
		goto done;
	}

	result = cook.grid;
	cook.grid = NULL;

done:
	if ( status != NULL )
	{
		*status = cook.status;
	}
	v3BlockGridFreeCook( &cook );
	return result;
}

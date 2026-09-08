// SPDX-License-Identifier: MIT
//
// A BlockGrid is an immutable, discrete field of axis-aligned blocks (terrain, a
// hull, a build).
//
// BlockGrid holds the whole field in one allocation to avoid one shape per block.
//
// The grid is described in parallel by:
//
//   Blocks   the logical unit cubes on the integer lattice, used for occupancy.
//   Hitboxes the exact boxes collision runs against. Cooking drops enclosed full
//            cubes and merges the survivors into slabs, so hitboxes are usually
//            far fewer than blocks without changing the reachable surface. Each
//            one records the cells it covers, which is how a hit still names a
//            logical block once the one-box-per-cell correspondence is gone.
//
// Two indexes make the queries faster.
//
// Occupancy Groups collect 4x4x4 blocks behind a 64-bit mask, together with the
// hitboxes that touch them. This is one table over the full field, and it can
// answer all queries alone.
//
// Rays also use a projection grid. This grid flattens the field along its
// thinnest axis. A ray then moves from cell to cell and reads only the cells
// that it crosses. The group index instead reads the full box around the ray.
//
// A field gets no projection grid if the grid is larger than the hitbox data it
// makes faster. Its rays then use the group index. The projection grid thus
// changes the speed of a query, but never the result.
//
// Changes require cooking and publishing a new grid, then releasing the old grid.
// Identical content packs to identical bytes and one hash, allowing content interning.
// Immutable geometry can be read concurrently without locks.

#pragma once

#include "vecture3d/block_grid.h"

#include "box3d/math_functions.h"
#include "box3d/types.h"

#include <stdbool.h>
#include <stdint.h>

// One unit block at grid coordinate (x, y, z)
typedef struct v3BlockGridBlockDef
{
	int x, y, z;
	uint32_t materialIndex;

	// Caller-provided ID. Must be unique across blocks and hitboxes in this cook.
	uint64_t sourceId;

	// Flags are part of the hash calculation, so blocks with different flags are not
	// equal
	uint32_t flags;
} v3BlockGridBlockDef;

// One collision box
typedef struct v3BlockGridBoxDef
{
	b3Vec3 center;
	b3Vec3 halfExtent;

	// Indexes the caller's block array; cooking remaps it through the spatial
	// sort so a hit still traces back to a block after merging.
	uint32_t ownerBlockIndex;
	uint32_t materialIndex;

	// Shares the block namespace above, so numbering hitboxes from one would
	// collide with block one and be rejected at cook time.
	uint64_t sourceId;
} v3BlockGridBoxDef;

// How the caller means a grid to sit in the world. A section tile is one piece
// of a larger streamed field, a standalone aggregate is self-contained.
typedef enum v3BlockGridPlacement
{
	v3_blockGridSectionTile,
	v3_blockGridStandaloneAggregate,
} v3BlockGridPlacement;

typedef struct v3BlockGridDef
{
	// Cooking does not sort this array, so material order is part of content
	// identity. Give the table a stable order and remap indices before cooking.
	const b3SurfaceMaterial* materials;
	int materialCount;
	const v3BlockGridBlockDef* blocks;
	int blockCount;
	const v3BlockGridBoxDef* boxes;
	int boxCount;

	// Zero is v3_blockGridSectionTile, so an unset placement reads as terrain.
	v3BlockGridPlacement placement;
} v3BlockGridDef;

typedef enum v3BlockGridCreateStatus
{
	v3_blockGridCreateOk,
	v3_blockGridCreateInvalidDefinition,
	v3_blockGridCreateLimitExceeded,
	v3_blockGridCreateOutOfMemory,
	v3_blockGridCreateInvalidMaterial,
	v3_blockGridCreateInvalidBlock,
	v3_blockGridCreateDuplicateBlock,
	v3_blockGridCreateInvalidHitbox,
	v3_blockGridCreateDuplicateHitbox,
	v3_blockGridCreateInvalidOwner,
} v3BlockGridCreateStatus;

typedef struct v3BlockGridQueryScratch
{
	// One buffer per active query. Nested and concurrent queries need their own,
	// even against the same grid.
	//
	// Allocate zeroed once, sized by v3BlockGrid_GetQueryScratchWordCount.
	// Queries clear only the words they touched, including after an early stop,
	// so reuse needs no clearing and cost tracks the query, not the grid.
	//
	// Projected ray traversal needs no query bitmap. The fallback group walk
	// uses this buffer when the field has no projection grid.
	uint64_t* visitedHitboxes;
	int visitedWordCapacity;
} v3BlockGridQueryScratch;

// Return false to stop the query; stopping is reported separately from
// completing.
typedef bool v3BlockGridQueryFcn( int hitboxIndex, void* context );

typedef enum v3BlockGridQueryStatus
{
	v3_blockGridQueryInvalid,
	v3_blockGridQueryCompleted,
	v3_blockGridQueryStopped,
} v3BlockGridQueryStatus;

typedef enum v3BlockGridRayStatus
{
	v3_blockGridRayInvalid,
	v3_blockGridRayMiss,
	v3_blockGridRayHit,
} v3BlockGridRayStatus;

typedef struct v3BlockGridRayResult
{
	v3BlockGridRayStatus status;
	int hitboxIndex;

	// Fraction along p1 to p2. A ray starting inside a hitbox reports zero with
	// a zero normal, since there is no entry face.
	float fraction;
	b3Vec3 faceNormal;
} v3BlockGridRayResult;

// Cooked statistics for diagnostics and capacity planning.
typedef struct v3BlockGridStats
{
	int blockCount;
	int hitboxCount;

	// Reduction counts satisfy inputHitboxCount = hitboxCount + culledHitboxCount + mergedHitboxCount.
	// Culled boxes belong to enclosed cells. Merged boxes are absorbed into another cell's hitbox.
	int inputHitboxCount;
	int culledHitboxCount;
	int mergedHitboxCount;

	// Groups in the frame against groups holding anything. The ratio is what
	// decides whether a dense index suits this field.
	int groupCount;
	int nonEmptyGroupCount;

	// Size of the CSR list. A hitbox spanning several groups counts once per
	// group it reaches.
	int hitboxReferenceCount;

	// Sizes of the header, geometry and direct index sections.
	// Projection arrays and padding are included only in totalBytes.
	int headerBytes;
	int materialBytes;
	int blockBytes;
	int hitboxBytes;
	int occupancyMaskBytes;
	int hitboxOffsetBytes;
	int hitboxIndexBytes;

	// The cooked allocation size.
	int totalBytes;

	b3AABB bounds;
	v3BlockGridPlacement placement;
} v3BlockGridStats;

// Zero-initialized for a null grid.
v3BlockGridStats v3BlockGrid_GetStats( const v3BlockGridData* grid );

// Where the cooked lattice sits. Placement rides outside the content hash, so
// two grids differing only here are the same content and intern together; a
// caller that serializes a grid has to carry this alongside the bytes.
void v3BlockGrid_GetPlacement( const v3BlockGridData* grid, int* originX, int* originY, int* originZ, int* placement );

// Runtime cost of the occupancy-group narrowing ladder, accumulated across all
// queries since the last reset. invocations, groupsWalked, and refsMarked cover
// every marking pass including rays; the remaining three cover only the AABB
// candidate loop. Release builds report zeros.
typedef struct v3BlockGridQueryCounters
{
	int invocations;
	int groupsWalked;
	int refsMarked;
	int wordsSpanned;
	int bitsTested;
	int overlapsFired;
} v3BlockGridQueryCounters;

v3BlockGridQueryCounters v3BlockGrid_GetQueryCounters( void );

void v3BlockGrid_ResetQueryCounters( void );

v3BlockGridData* v3BlockGrid_Create( const v3BlockGridDef* def, v3BlockGridCreateStatus* status );
void v3BlockGrid_Retain( v3BlockGridData* grid );
void v3BlockGrid_Release( v3BlockGridData* grid );

int v3BlockGrid_GetMaterialCount( const v3BlockGridData* grid );
int v3BlockGrid_GetBlockCount( const v3BlockGridData* grid );
int v3BlockGrid_GetHitboxCount( const v3BlockGridData* grid );

typedef struct v3BlockGridIdentity
{
	int cellX, cellY, cellZ;
	int cellBoxIndex;
	uint32_t materialIndex;
	uint64_t userMaterialId;
	uint64_t userData;
} v3BlockGridIdentity;

// Resolve a private cooked hitbox at a world point to the caller's logical identity.
// The point selects one cell when cooking merged several cells into one slab.
bool v3BlockGrid_ResolveIdentity( const v3BlockGridData* grid, b3WorldTransform transform, b3Pos point, int hitboxIndex,
								  v3BlockGridIdentity* identity );

// Returns the caller's hitbox ID, preserved through cooking.
// Input IDs must be unique across blocks and hitboxes in this cook.
uint64_t v3BlockGrid_GetHitboxSourceId( const v3BlockGridData* grid, int hitboxIndex );

// The cooked block index this hitbox came from. After merging, one owner stands
// for the whole run.
uint32_t v3BlockGrid_GetHitboxOwnerBlock( const v3BlockGridData* grid, int hitboxIndex );
uint32_t v3BlockGrid_GetHitboxMaterial( const v3BlockGridData* grid, int hitboxIndex );

// Find a logical cell in logarithmic time and return its caller data.
bool v3BlockGrid_GetCellUserData( const v3BlockGridData* grid, int localX, int localY, int localZ, uint64_t* userData );

// The cells this hitbox covers, in grid-local lattice coordinates and inclusive on both ends.
// A merged hitbox reports its whole run and an exact sub-cell box reports the one cell it sits
// in, which is how a hit resolves to a logical block once merging has removed the one box per
// cell. Both arrays are left untouched for an invalid grid or index.
void v3BlockGrid_GetHitboxCellRange( const v3BlockGridData* grid, int hitboxIndex, int cellMin[3], int cellMax[3] );

// Which of the six axis-aligned faces of this hitbox the neighbouring cells cover whole, as a
// bit per face at 2 * axis + positive: bit 0 is -x, bit 1 is +x, and so on up to bit 5 for +z.
// Cooked once with the geometry and read in constant time, so a contact never searches for what
// covers a face.
uint32_t v3BlockGrid_GetHitboxFaceMask( const v3BlockGridData* grid, int hitboxIndex );

// Returns world space, with placement added back; only the stored bytes are
// placement-free.
void v3BlockGrid_GetHitboxBounds( const v3BlockGridData* grid, int hitboxIndex, b3Vec3* center, b3Vec3* halfExtent );
const b3SurfaceMaterial* v3BlockGrid_GetMaterials( const v3BlockGridData* grid );
b3AABB v3BlockGrid_GetBounds( const v3BlockGridData* grid );

// Scratch words this grid needs. Scales with hitbox count, so size per grid.
int v3BlockGrid_GetQueryScratchWordCount( const v3BlockGridData* grid );

// Reports each overlapping hitbox once, in ascending stable hitbox ID order
// so results stay identical across recooks.
// Touching a face counts as overlap, so a body resting flush on a surface stays
// a contact candidate.
v3BlockGridQueryStatus v3BlockGrid_QueryAABB( const v3BlockGridData* grid, b3AABB bounds, v3BlockGridQueryScratch* scratch,
											  v3BlockGridQueryFcn* callback, void* context );

// Finds the nearest hit between p1 and p2.
//
// Exactly equal computed fractions use the lower source ID. Distinct fractions
// always select the nearer hit, independently of the query index.
v3BlockGridRayResult v3BlockGrid_RayCast( const v3BlockGridData* grid, b3Vec3 p1, b3Vec3 p2, v3BlockGridQueryScratch* scratch );

// Counts logical blocks inside bounds from the occupancy masks alone, never
// visiting hitboxes.
int v3BlockGrid_GetOccupiedBlockCount( const v3BlockGridData* grid, b3AABB bounds );

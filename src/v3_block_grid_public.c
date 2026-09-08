// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "physics_world.h"
#include "shape.h"
#include "v3_block_grid.h"
#include "v3_block_grid_internal.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <stdlib.h>

#define V3_PUBLIC_BLOCK_GRID_MAX_ITEMS 65534

static const v3BlockGridCookStatus v3_publicBlockGridStatuses[] = {
	[v3_blockGridCreateOk] = v3_blockGridCookOk,
	[v3_blockGridCreateInvalidDefinition] = v3_blockGridCookInvalidDefinition,
	[v3_blockGridCreateLimitExceeded] = v3_blockGridCookLimitExceeded,
	[v3_blockGridCreateOutOfMemory] = v3_blockGridCookOutOfMemory,
	[v3_blockGridCreateInvalidMaterial] = v3_blockGridCookInvalidMaterial,
	[v3_blockGridCreateInvalidBlock] = v3_blockGridCookInvalidBlock,
	[v3_blockGridCreateDuplicateBlock] = v3_blockGridCookDuplicateBlock,
	[v3_blockGridCreateInvalidHitbox] = v3_blockGridCookInvalidBox,
	[v3_blockGridCreateDuplicateHitbox] = v3_blockGridCookInvalidBox,
	[v3_blockGridCreateInvalidOwner] = v3_blockGridCookInvalidBlock,
};

_Static_assert( B3_ARRAY_COUNT( v3_publicBlockGridStatuses ) == v3_blockGridCreateInvalidOwner + 1,
				"BlockGrid status map is incomplete" );

typedef struct v3PublicBlockEntry
{
	const v3BlockGridBlock* block;
} v3PublicBlockEntry;

typedef struct v3PublicBlockGridCook
{
	const v3BlockGridCookDef* def;
	v3BlockGridCookStatus status;
	v3PublicBlockEntry* orderedBlocks;
	size_t orderedBlockBytes;
	v3BlockGridBlockDef* blocks;
	size_t blockBytes;
	v3BlockGridBoxDef* boxes;
	size_t boxBytes;
	int boxCount;
} v3PublicBlockGridCook;

static bool v3PublicBlockGridReject( v3PublicBlockGridCook* cook, v3BlockGridCookStatus status )
{
	cook->status = status;
	return false;
}

static int v3PublicBlockGridCompareBlocks( const void* a, const void* b )
{
	const v3BlockGridBlock* left = ( (const v3PublicBlockEntry*)a )->block;
	const v3BlockGridBlock* right = ( (const v3PublicBlockEntry*)b )->block;
	if ( left->x != right->x )
	{
		return ( left->x > right->x ) - ( left->x < right->x );
	}
	if ( left->y != right->y )
	{
		return ( left->y > right->y ) - ( left->y < right->y );
	}
	return ( left->z > right->z ) - ( left->z < right->z );
}

static bool v3PublicBlockGridValidateDefinition( v3PublicBlockGridCook* cook )
{
	const v3BlockGridCookDef* def = cook->def;
	if ( def == NULL || def->materials == NULL || def->blocks == NULL || def->materialCount <= 0 || def->blockCount <= 0 )
	{
		return v3PublicBlockGridReject( cook, v3_blockGridCookInvalidDefinition );
	}
	if ( def->materialCount > V3_PUBLIC_BLOCK_GRID_MAX_ITEMS || def->blockCount > V3_PUBLIC_BLOCK_GRID_MAX_ITEMS )
	{
		return v3PublicBlockGridReject( cook, v3_blockGridCookLimitExceeded );
	}

	int64_t boxCount = 0;
	for ( int i = 0; i < def->blockCount; ++i )
	{
		const v3BlockGridBlock* block = def->blocks + i;
		if ( block->boxes == NULL || block->boxCount <= 0 )
		{
			return v3PublicBlockGridReject( cook, v3_blockGridCookInvalidBlock );
		}
		boxCount += block->boxCount;
		if ( boxCount > V3_PUBLIC_BLOCK_GRID_MAX_ITEMS )
		{
			return v3PublicBlockGridReject( cook, v3_blockGridCookLimitExceeded );
		}
	}
	cook->boxCount = (int)boxCount;
	return true;
}

static bool v3PublicBlockGridAllocateInput( v3PublicBlockGridCook* cook )
{
	cook->orderedBlockBytes = (size_t)cook->def->blockCount * sizeof( v3PublicBlockEntry );
	cook->blockBytes = (size_t)cook->def->blockCount * sizeof( v3BlockGridBlockDef );
	cook->boxBytes = (size_t)cook->boxCount * sizeof( v3BlockGridBoxDef );
	cook->orderedBlocks = b3TryAlloc( cook->orderedBlockBytes );
	cook->blocks = b3TryAlloc( cook->blockBytes );
	cook->boxes = b3TryAlloc( cook->boxBytes );
	if ( cook->orderedBlocks == NULL || cook->blocks == NULL || cook->boxes == NULL )
	{
		return v3PublicBlockGridReject( cook, v3_blockGridCookOutOfMemory );
	}
	return true;
}

static bool v3PublicBlockGridOrderBlocks( v3PublicBlockGridCook* cook )
{
	for ( int i = 0; i < cook->def->blockCount; ++i )
	{
		cook->orderedBlocks[i].block = cook->def->blocks + i;
	}
	qsort( cook->orderedBlocks, (size_t)cook->def->blockCount, sizeof( v3PublicBlockEntry ), v3PublicBlockGridCompareBlocks );
	for ( int i = 1; i < cook->def->blockCount; ++i )
	{
		if ( v3PublicBlockGridCompareBlocks( cook->orderedBlocks + i - 1, cook->orderedBlocks + i ) == 0 )
		{
			return v3PublicBlockGridReject( cook, v3_blockGridCookDuplicateBlock );
		}
	}
	return true;
}

static bool v3PublicBlockGridIsValidBox( const v3BlockGridBox* box )
{
	const b3AABB cell = { .lowerBound = b3Vec3_zero, .upperBound = { 1.0f, 1.0f, 1.0f } };
	if ( b3IsValidAABB( box->bounds ) == false || b3AABB_Contains( cell, box->bounds ) == false )
	{
		return false;
	}
	b3Vec3 size = b3Sub( box->bounds.upperBound, box->bounds.lowerBound );
	return size.x > 0.0f && size.y > 0.0f && size.z > 0.0f;
}

static bool v3PublicBlockGridCopyInput( v3PublicBlockGridCook* cook )
{
	int boxIndex = 0;
	for ( int blockIndex = 0; blockIndex < cook->def->blockCount; ++blockIndex )
	{
		const v3BlockGridBlock* source = cook->orderedBlocks[blockIndex].block;
		cook->blocks[blockIndex] = (v3BlockGridBlockDef){
			.x = source->x,
			.y = source->y,
			.z = source->z,
			.sourceId = (uint64_t)blockIndex + 1,
		};
		for ( int localIndex = 0; localIndex < source->boxCount; ++localIndex )
		{
			const v3BlockGridBox* box = source->boxes + localIndex;
			if ( box->materialIndex >= (uint32_t)cook->def->materialCount )
			{
				return v3PublicBlockGridReject( cook, v3_blockGridCookInvalidMaterial );
			}
			if ( v3PublicBlockGridIsValidBox( box ) == false )
			{
				return v3PublicBlockGridReject( cook, v3_blockGridCookInvalidBox );
			}
			b3Vec3 size = b3Sub( box->bounds.upperBound, box->bounds.lowerBound );
			b3Vec3 localCenter = b3MulAdd( box->bounds.lowerBound, 0.5f, size );
			cook->boxes[boxIndex++] = (v3BlockGridBoxDef){
				.center = b3Add( (b3Vec3){ (float)source->x, (float)source->y, (float)source->z }, localCenter ),
				.halfExtent = b3MulSV( 0.5f, size ),
				.ownerBlockIndex = (uint32_t)blockIndex,
				.materialIndex = box->materialIndex,
				.sourceId = ( (uint64_t)( (uint32_t)blockIndex + 1 ) << 32 ) | ( (uint32_t)localIndex + 1 ),
			};
		}
	}
	return true;
}

static v3BlockGridCookStatus v3PublicBlockGridMapStatus( v3BlockGridCreateStatus status )
{
	B3_ASSERT( 0 <= status && status < B3_ARRAY_COUNT( v3_publicBlockGridStatuses ) );
	return v3_publicBlockGridStatuses[status];
}

static void v3PublicBlockGridCopyUserData( const v3PublicBlockGridCook* cook, v3BlockGridData* data )
{
	// The internal cooker validates globally unique temporary identities. Public
	// block data may repeat, so copy it only after cooking and before publication.
	v3BlockGridBlockData* blocks = v3BlockGridPointer( data, data->blockOffset );
	for ( int i = 0; i < data->blockCount; ++i )
	{
		blocks[i].sourceId = cook->orderedBlocks[i].block->userData;
	}
	data->hash = 0;
	data->hash = b3Hash64NonZero( (const uint8_t*)data + V3_BLOCK_GRID_CONTENT_OFFSET,
								  data->byteCount - (int)V3_BLOCK_GRID_CONTENT_OFFSET );
}

static void v3PublicBlockGridFreeCook( v3PublicBlockGridCook* cook )
{
	b3Free( cook->orderedBlocks, cook->orderedBlockBytes );
	b3Free( cook->blocks, cook->blockBytes );
	b3Free( cook->boxes, cook->boxBytes );
}

v3BlockGridCookStats v3BlockGrid_GetCookStats( const v3BlockGridData* data )
{
	if ( data == NULL )
	{
		return (v3BlockGridCookStats){ 0 };
	}
	return (v3BlockGridCookStats){
		.materialCount = data->materialCount,
		.blockCount = data->blockCount,
		.boxCount = data->hitboxCount,
		.byteCount = data->byteCount,
		.localBounds = v3BlockGrid_GetBounds( data ),
		.inputBoxCount = data->inputBoxCount,
		.culledBoxCount = data->culledBoxCount,
		.mergedBoxCount = data->mergedBoxCount,
	};
}

v3BlockGridCookResult v3CookBlockGrid( const v3BlockGridCookDef* def )
{
	v3BlockGridCookResult result = { .status = v3_blockGridCookInvalidDefinition };
	v3PublicBlockGridCook cook = { .def = def, .status = v3_blockGridCookInvalidDefinition };
	if ( v3PublicBlockGridValidateDefinition( &cook ) == false || v3PublicBlockGridAllocateInput( &cook ) == false ||
		 v3PublicBlockGridOrderBlocks( &cook ) == false || v3PublicBlockGridCopyInput( &cook ) == false )
	{
		goto done;
	}

	v3BlockGridDef internalDef = {
		.materials = def->materials,
		.materialCount = def->materialCount,
		.blocks = cook.blocks,
		.blockCount = def->blockCount,
		.boxes = cook.boxes,
		.boxCount = cook.boxCount,
		.placement = v3_blockGridStandaloneAggregate,
	};
	v3BlockGridCreateStatus internalStatus = v3_blockGridCreateInvalidDefinition;
	result.data = v3BlockGrid_Create( &internalDef, &internalStatus );
	cook.status = v3PublicBlockGridMapStatus( internalStatus );
	if ( result.data != NULL )
	{
		v3PublicBlockGridCopyUserData( &cook, result.data );
		result.stats = v3BlockGrid_GetCookStats( result.data );
	}

done:
	result.status = cook.status;
	v3PublicBlockGridFreeCook( &cook );
	return result;
}

void v3DestroyBlockGridData( v3BlockGridData* data )
{
	v3BlockGrid_Release( data );
}

typedef struct v3ClosestCastContext
{
	b3World* world;
	v3ClosestCastResult result;
	bool ignoreInitialOverlap;
} v3ClosestCastContext;

static bool v3PublicCastComesFirst( const v3ClosestCastResult* candidate, const v3ClosestCastResult* current )
{
	if ( current->hit == false || candidate->fraction < current->fraction )
	{
		return true;
	}
	if ( candidate->fraction > current->fraction )
	{
		return false;
	}
	if ( candidate->shapeId.index1 != current->shapeId.index1 )
	{
		return candidate->shapeId.index1 < current->shapeId.index1;
	}
	if ( candidate->cellX != current->cellX )
	{
		return candidate->cellX < current->cellX;
	}
	if ( candidate->cellY != current->cellY )
	{
		return candidate->cellY < current->cellY;
	}
	if ( candidate->cellZ != current->cellZ )
	{
		return candidate->cellZ < current->cellZ;
	}
	return candidate->cellBoxIndex < current->cellBoxIndex;
}

static float v3PublicClosestCastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
										  int triangleIndex, int childIndex, void* context )
{
	B3_UNUSED( childIndex );
	v3ClosestCastContext* cast = context;
	if ( cast->ignoreInitialOverlap && fraction == 0.0f )
	{
		return 1.0f;
	}

	b3Shape* shape = b3Array_Get( cast->world->shapes, shapeId.index1 - 1 );
	v3ClosestCastResult candidate = {
		.bodyId = b3MakeBodyId( cast->world, shape->bodyId ),
		.shapeId = shapeId,
		.point = point,
		.normal = normal,
		.fraction = fraction,
		.userMaterialId = userMaterialId,
		.triangleIndex = triangleIndex,
		.hit = true,
	};
	if ( shape->type == v3_blockGridShape )
	{
		v3BlockGridIdentity identity;
		b3Body* body = b3Array_Get( cast->world->bodies, shape->bodyId );
		if ( v3BlockGrid_ResolveIdentity( shape->blockGrid, b3GetBodyTransformQuick( cast->world, body ), point,
										  cast->world->blockGridQueryHitboxIndex, &identity ) == false )
		{
			return 1.0f;
		}
		candidate.isBlockGrid = true;
		candidate.cellX = identity.cellX;
		candidate.cellY = identity.cellY;
		candidate.cellZ = identity.cellZ;
		candidate.cellBoxIndex = identity.cellBoxIndex;
		candidate.materialIndex = identity.materialIndex;
		candidate.userMaterialId = identity.userMaterialId;
		candidate.userData = identity.userData;
	}
	if ( v3PublicCastComesFirst( &candidate, &cast->result ) )
	{
		cast->result = candidate;
	}
	return 1.0f;
}

v3ClosestCastResult v3World_CastShapeClosest( b3WorldId worldId, b3Pos origin, const b3ShapeProxy* proxy, b3Vec3 translation,
											  b3QueryFilter filter )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL || proxy == NULL || proxy->points == NULL || proxy->count <= 0 )
	{
		return (v3ClosestCastResult){ 0 };
	}
	v3ClosestCastContext context = { .world = world };
	b3World_CastShape( worldId, origin, proxy, translation, filter, v3PublicClosestCastCallback, &context );
	return context.result;
}

v3ClosestCastResult v3World_CastRayClosest( b3WorldId worldId, b3Pos origin, b3Vec3 translation, b3QueryFilter filter )
{
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return (v3ClosestCastResult){ 0 };
	}
	v3ClosestCastContext context = { .world = world, .ignoreInitialOverlap = true };
	b3World_CastRay( worldId, origin, translation, filter, v3PublicClosestCastCallback, &context );
	return context.result;
}

v3ClosestCastResult v3World_CastSphereClosest( b3WorldId worldId, b3Pos origin, const b3Sphere* sphere, b3Vec3 translation,
											   b3QueryFilter filter )
{
	if ( sphere == NULL )
	{
		return (v3ClosestCastResult){ 0 };
	}
	b3ShapeProxy proxy = { .points = &sphere->center, .count = 1, .radius = sphere->radius };
	return v3World_CastShapeClosest( worldId, origin, &proxy, translation, filter );
}

v3ClosestCastResult v3World_CastCapsuleClosest( b3WorldId worldId, b3Pos origin, const b3Capsule* capsule, b3Vec3 translation,
												b3QueryFilter filter )
{
	if ( capsule == NULL )
	{
		return (v3ClosestCastResult){ 0 };
	}
	b3ShapeProxy proxy = { .points = &capsule->center1, .count = 2, .radius = capsule->radius };
	return v3World_CastShapeClosest( worldId, origin, &proxy, translation, filter );
}

v3ClosestCastResult v3World_CastBoxClosest( b3WorldId worldId, b3Pos origin, b3Vec3 center, b3Vec3 halfExtent, b3Vec3 translation,
											b3QueryFilter filter )
{
	b3BoxHull box = b3MakeAxisAlignedBoxHull( halfExtent, center );
	b3ShapeProxy proxy = { .points = box.boxPoints, .count = 8, .radius = 0.0f };
	return v3World_CastShapeClosest( worldId, origin, &proxy, translation, filter );
}

v3ClosestCastResult v3World_CastHullClosest( b3WorldId worldId, b3Pos origin, const b3HullData* hull, b3Vec3 translation,
											 b3QueryFilter filter )
{
	if ( hull == NULL )
	{
		return (v3ClosestCastResult){ 0 };
	}
	b3ShapeProxy proxy = { .points = b3GetHullPoints( hull ), .count = hull->vertexCount, .radius = 0.0f };
	return v3World_CastShapeClosest( worldId, origin, &proxy, translation, filter );
}

// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

#include <stdbool.h>
#include <stdint.h>

/// One axis-aligned collision box expressed in its owning unit cell.
/// Bounds may touch the cell boundary but must remain within [0, 1] on every axis.
typedef struct v3BlockGridBox
{
	b3AABB bounds;
	uint32_t materialIndex;
} v3BlockGridBox;

/// One occupied logical cell and its explicit collision boxes.
/// Block array order does not affect cooked geometry. Box array order identifies a
/// box within this block. userData is copied verbatim and does not need to be unique.
typedef struct v3BlockGridBlock
{
	int x, y, z;
	uint64_t userData;
	const v3BlockGridBox* boxes;
	int boxCount;
} v3BlockGridBlock;

/// Input for one immutable BlockGrid cook.
/// The cooker copies all referenced arrays before returning.
typedef struct v3BlockGridCookDef
{
	const b3SurfaceMaterial* materials;
	int materialCount;
	const v3BlockGridBlock* blocks;
	int blockCount;
} v3BlockGridCookDef;

/// Opaque immutable cooked BlockGrid data.
typedef struct v3BlockGridData v3BlockGridData;

typedef enum v3BlockGridCookStatus
{
	/// Cooking completed and the result owns non-null data.
	v3_blockGridCookOk,

	/// The definition, one of its arrays, or a required count is missing.
	v3_blockGridCookInvalidDefinition,

	/// A count, coordinate range, or cooked representation exceeds a supported limit.
	v3_blockGridCookLimitExceeded,

	/// The configured Box3D allocator could not satisfy the cook.
	v3_blockGridCookOutOfMemory,

	/// A surface material or a box material index is invalid.
	v3_blockGridCookInvalidMaterial,

	/// A block has an unrepresentable coordinate or has no boxes.
	v3_blockGridCookInvalidBlock,

	/// More than one block occupies the same local cell coordinate.
	v3_blockGridCookDuplicateBlock,

	/// A box is non-finite, empty, or extends outside its owning unit cell.
	v3_blockGridCookInvalidBox,
} v3BlockGridCookStatus;

/// Statistics for the immutable cooked result. Bounds use BlockGrid-local coordinates.
typedef struct v3BlockGridCookStats
{
	/// Number of copied surface materials.
	int materialCount;

	/// Number of occupied logical cells.
	int blockCount;

	/// Number of cooked collision boxes retained after culling and merging.
	int boxCount;

	/// Total bytes owned by the cooked data.
	int byteCount;

	/// Bounds of all cooked boxes in BlockGrid-local coordinates.
	b3AABB localBounds;

	/// Collision boxes the definition supplied, before culling and merging.
	int inputBoxCount;

	/// Boxes that produced no collision box because their cell is a full cube whose six
	/// face neighbours are full cubes, so nothing outside the grid can reach it.
	int culledBoxCount;

	/// Boxes folded into a merged box that another cell originated. inputBoxCount always
	/// equals boxCount plus culledBoxCount plus mergedBoxCount.
	int mergedBoxCount;
} v3BlockGridCookStats;

/// A successful result owns data and reports v3_blockGridCookOk. Every failed
/// result has data == NULL and zero statistics, so no partial ownership is published.
typedef struct v3BlockGridCookResult
{
	v3BlockGridCookStatus status;
	v3BlockGridData* data;
	v3BlockGridCookStats stats;
} v3BlockGridCookResult;

/// Cook immutable data without accessing a Box3D world.
/// Independent calls may run concurrently. The application must configure its Box3D
/// allocator before cooking, keep it unchanged until all cooked data is destroyed,
/// and provide a thread-safe allocator when cooks run concurrently. Input arrays must
/// remain read-only until their call returns.
B3_API v3BlockGridCookResult v3CookBlockGrid( const v3BlockGridCookDef* def );

/// Read statistics from immutable cooked data. This is safe concurrently with other
/// read-only uses of the same data. The caller must keep data alive for every reader.
B3_API v3BlockGridCookStats v3BlockGrid_GetCookStats( const v3BlockGridData* data );

/// Destroy the caller's one ownership of cooked data. NULL is accepted. The caller
/// must synchronize this with all read-only users and call it exactly once per
/// successful cook. Pointer identity and cooked byte layout are unspecified.
B3_API void v3DestroyBlockGridData( v3BlockGridData* data );

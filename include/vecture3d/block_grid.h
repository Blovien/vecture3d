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

/// One participant in a BlockGrid contact event. Body and shape identifiers are
/// always populated. Grid fields are zero when isBlockGrid is false.
typedef struct v3BlockContactSide
{
	b3BodyId bodyId;
	b3ShapeId shapeId;
	bool isBlockGrid;
	int cellX, cellY, cellZ;
	int subHitboxIndex;
	uint32_t materialIndex;
	uint64_t userMaterialId;
	uint64_t userData;
} v3BlockContactSide;

/// One contact transition involving at least one BlockGrid shape. The normal
/// points from sideA to sideB. End events retain the last known point and payload.
typedef struct v3BlockContactEvent
{
	v3BlockContactSide sideA, sideB;
	b3Pos point;
	b3Vec3 normal;
	float normalImpulse;
	float approachSpeed;
} v3BlockContactEvent;

/// BlockGrid contact events published by the most recent step. The arrays remain
/// valid until the next world step. Destruction and replacement may append cached
/// end events without moving the arrays. capacity is the fixed bound of each array.
/// If identity tracking overflows, truncated stays true through the first step
/// whose contact state fits again. Begins are suppressed during recovery because
/// an untracked ongoing contact cannot be distinguished from a new cell contact.
/// The dropped counters cannot count unknown transitions during that interval.
typedef struct v3BlockContactEvents
{
	const v3BlockContactEvent* beginEvents;
	const v3BlockContactEvent* hitEvents;
	const v3BlockContactEvent* endEvents;
	int beginCount, hitCount, endCount;
	uint32_t droppedBeginCount, droppedHitCount, droppedEndCount;
	int capacity;
	bool truncated;
} v3BlockContactEvents;

/// Closest cast result for ordinary Box3D shapes and BlockGrids. Grid fields are
/// zero unless isBlockGrid is true. cellBoxIndex is the original Cell Box index.
typedef struct v3ClosestCastResult
{
	b3BodyId bodyId;
	b3ShapeId shapeId;
	b3Pos point;
	b3Vec3 normal;
	float fraction;
	uint64_t userMaterialId;
	int triangleIndex;
	bool isBlockGrid;
	int cellX, cellY, cellZ;
	int cellBoxIndex;
	uint32_t materialIndex;
	uint64_t userData;
	bool hit;
} v3ClosestCastResult;

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

/// Attach immutable cooked data to a static, kinematic, or dynamic body and return
/// an ordinary Box3D shape identifier. A successful shape retains its own internal
/// ownership, so the caller may destroy its cooking handle after this call returns.
/// The same data may be reused by multiple shapes and independently synchronized
/// worlds, including concurrent read-only use. The caller must keep data alive for
/// the duration of this call.
///
/// Attachment mutates the body's world. The world must be unlocked, and the caller
/// must serialize this call with every other operation on that world. Concurrent
/// attachment to different worlds also requires a thread-safe configured allocator.
///
/// Returns b3_nullShapeId when bodyId is invalid, def or data is NULL, def requests
/// a sensor, or Box3D cannot attach the shape. Failure creates no shape and retains
/// no ownership. On success, destroying the shape, its body, or its world releases
/// the shape's internal ownership.
B3_API b3ShapeId v3CreateBlockGridShape( b3BodyId bodyId, const b3ShapeDef* def, v3BlockGridData* data );

/// Destroy the caller's one ownership of cooked data. NULL is accepted. The caller
/// must synchronize this with all read-only users and call it exactly once per
/// successful cook. Pointer identity and cooked byte layout are unspecified.
B3_API void v3DestroyBlockGridData( v3BlockGridData* data );

typedef enum v3BlockGridReplaceStatus
{
	/// The new revision is published and the shape now collides against it.
	v3_blockGridReplaceOk,

	/// shapeId does not name a live shape.
	v3_blockGridReplaceInvalidShape,

	/// The shape is live but was not created by v3CreateBlockGridShape.
	v3_blockGridReplaceWrongShapeType,

	/// data is NULL. Cooked data carries its own validity, so any successful cook
	/// may back any BlockGrid shape and there is no further compatibility rule.
	v3_blockGridReplaceInvalidData,

	/// The shape's world is stepping or otherwise mid-callback.
	v3_blockGridReplaceWorldLocked,

	/// The configured Box3D allocator could not satisfy the publication. Nothing
	/// was mutated; the previous revision is still attached.
	v3_blockGridReplaceOutOfMemory,
} v3BlockGridReplaceStatus;

/// Publish a new cooked revision behind an existing BlockGrid shape.
///
/// The caller cooks the new revision with v3CookBlockGrid, which never touches a
/// world, and calls this with the world unlocked; it must serialize this call with
/// every other operation on that world, exactly as attachment requires.
///
/// The shape identifier survives, so every handle the application already holds
/// stays valid, and so does every b3ShapeDef property the shape was created with:
/// user data, filter, density, explosion scale, name, and the event flags. Only the
/// geometry and what is derived from it change.
///
/// One publication means the swap and everything it invalidates happen inside a
/// single locked section, so no step and no query can observe a half-updated shape:
/// the shape takes its own reference on data, the previous revision's shape-held
/// reference is dropped, the shape's material table is rebuilt from the new
/// revision, the broad-phase proxy is recreated at the new bounds, every contact
/// touching the shape is destroyed so contacts on removed blocks end, the body's
/// mass data is recomputed when updateBodyMass is true, and the body is woken.
/// Contacts on blocks the new revision kept are recreated by the next step, with a
/// rebuilt warm start rather than a carried one. Republishing the revision the shape
/// already holds is a full publication with the same effects, not a no-op: contacts
/// are destroyed and rebuilt, the proxy is recreated, and the counter is incremented.
///
/// A failure changes nothing at all. The previous revision stays attached and fully
/// usable, the caller keeps its own reference on data, and stepping continues
/// exactly as it would have without the call.
///
/// The caller must keep data alive for the duration of this call. On success the
/// shape holds its own reference, so the caller may release its cooking handle with
/// v3DestroyBlockGridData as soon as the call returns; the cooked bytes are freed
/// once the last reference, from any shape or world, is gone.
B3_API v3BlockGridReplaceStatus v3ReplaceBlockGridShape( b3ShapeId shapeId, v3BlockGridData* data, bool updateBodyMass );

/// Return contact events involving a BlockGrid. Logical cells are expressed in
/// each grid's own coordinates and subHitboxIndex is the original Cell Box index.
/// Invalid world identifiers return a zero initialized result.
B3_API v3BlockContactEvents v3World_GetBlockContactEvents( b3WorldId worldId );

/// Cast a ray and return the closest ordinary shape or BlockGrid hit. As with
/// b3World_CastRayClosest, a ray that starts overlapped ignores that shape.
B3_API v3ClosestCastResult v3World_CastRayClosest( b3WorldId worldId, b3Pos origin, b3Vec3 translation, b3QueryFilter filter );

/// Cast an arbitrary convex point cloud through the world.
B3_API v3ClosestCastResult v3World_CastShapeClosest( b3WorldId worldId, b3Pos origin, const b3ShapeProxy* proxy,
													 b3Vec3 translation, b3QueryFilter filter );

/// Cast a sphere through the world. The sphere is relative to origin.
B3_API v3ClosestCastResult v3World_CastSphereClosest( b3WorldId worldId, b3Pos origin, const b3Sphere* sphere, b3Vec3 translation,
													  b3QueryFilter filter );

/// Cast a capsule through the world. The capsule is relative to origin.
B3_API v3ClosestCastResult v3World_CastCapsuleClosest( b3WorldId worldId, b3Pos origin, const b3Capsule* capsule,
													   b3Vec3 translation, b3QueryFilter filter );

/// Cast an axis-aligned box through the world. center is relative to origin.
B3_API v3ClosestCastResult v3World_CastBoxClosest( b3WorldId worldId, b3Pos origin, b3Vec3 center, b3Vec3 halfExtent,
												   b3Vec3 translation, b3QueryFilter filter );

/// Cast a convex hull through the world. Hull points are relative to origin.
B3_API v3ClosestCastResult v3World_CastHullClosest( b3WorldId worldId, b3Pos origin, const b3HullData* hull, b3Vec3 translation,
													b3QueryFilter filter );

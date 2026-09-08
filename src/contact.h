// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "arena_allocator.h"
#include "container.h"
#include "block_grid/block_grid_contact.h"

#include "box3d/collision.h"
#include "box3d/types.h"

#define B3_FORCE_GHOST_COLLISIONS 0

typedef struct b3Shape b3Shape;
typedef struct b3World b3World;

typedef union b3ContactCache
{
	b3SATCache satCache;
	b3SimplexCache simplexCache;
} b3ContactCache;

typedef struct b3TriangleCache
{
	int triangleIndex;
	b3ContactCache cache;
} b3TriangleCache;

b3DeclareArray( b3TriangleCache );

enum b3ContactFlags
{
	// Set when the solid shapes are touching.
	b3_contactTouchingFlag = 0x00000001,

	// Contact has a hit event
	b3_contactHitEventFlag = 0x00000002,

	// This contact wants contact events
	b3_contactEnableContactEvents = 0x00000004,

	// This is contact is between a dynamic and static body
	b3_contactStaticFlag = 0x00000008,

	b3_contactRecycleFlag = 0x00000010,

	// Set when the shapes are touching
	b3_simTouchingFlag = 0x00010000,

	// This contact no longer has overlapping AABBs
	b3_simDisjoint = 0x00020000,

	// This contact started touching
	b3_simStartedTouching = 0x00040000,

	// This contact stopped touching
	b3_simStoppedTouching = 0x00080000,

	// This contact has a hit event
	b3_simEnableHitEvent = 0x00100000,

	// This contact wants pre-solve events
	b3_simEnablePreSolveEvents = 0x00200000,

	// Relative transform is cached for contact recycling
	b3_relativeTransformValid = 0x00800000,

	// Enable speculative contact points
	b3_enableSpeculativePoints = 0x01000000,
};

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
typedef struct b3ContactEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b3ContactEdge;

typedef struct b3MeshContact
{
	b3Array( b3TriangleCache ) triangleCache;
	b3AABB queryBounds;
} b3MeshContact;

typedef struct b3ConvexContact
{
	b3ContactCache cache;
} b3ConvexContact;

// Runtime material values consumed by one solver manifold
typedef struct b3ContactMaterial
{
	float friction;
	float restitution;
	float rollingResistance;
	b3Vec3 tangentVelocity;
} b3ContactMaterial;

// Resolves both material callbacks and rotates local conveyor velocities into world space
// effectiveRadius follows the shape-specific rolling convention chosen by the caller
// Invalid callback output returns false without changing the output
bool b3ResolveContactMaterial( const b3World* world, const b3SurfaceMaterial* materialA, b3Quat rotationA,
							   const b3SurfaceMaterial* materialB, b3Quat rotationB, float effectiveRadius,
							   b3ContactMaterial* contactMaterial );

typedef enum b3ContactKind
{
	b3_convexContactKind,
	b3_meshContactKind,
	v3_blockGridPairContactKind,
	b3_contactKindCount,
} b3ContactKind;

static inline bool b3ContactKindUsesScalarSolver( b3ContactKind kind )
{
	return kind == b3_meshContactKind || kind == v3_blockGridPairContactKind;
}

// Represents the persistent interaction between two shapes
typedef struct b3Contact
{
	// index of simulation set stored in b3World
	// B3_NULL_INDEX when slot is free
	int setIndex;

	// index into the constraint graph color array
	// B3_NULL_INDEX for non-touching or sleeping contacts
	// B3_NULL_INDEX when slot is free
	int colorIndex;

	// contact index within set or graph color
	// B3_NULL_INDEX when slot is free
	int localIndex;

	b3ContactEdge edges[2];
	int shapeIdA;
	int shapeIdB;
	int childIndex;

	// A contact only belongs to an island if touching, otherwise B3_NULL_INDEX.
	int islandId;

	// Index into the island's contacts array for O(1) swap-removal.
	// B3_NULL_INDEX when not in an island.
	int islandIndex;

	// Back index into b3World::contacts
	int contactId;

	// These are transient and cached for improved performance. B3_NULL_INDEX for static bodies.
	int bodySimIndexA;
	int bodySimIndexB;

	// b3ContactFlags
	uint32_t flags;

	b3Manifold* manifolds;
	uint16_t manifoldCount;
	uint8_t kind;
	uint8_t reserved;

	// Cache for contact recycling.
	b3Quat cachedRotationA;
	b3Quat cachedRotationB;
	b3Transform cachedRelativePose;

	// Mixed friction and restitution
	float friction;

	// The contact kind selects exactly one owner in this union
	union
	{
		b3ConvexContact convexContact;
		b3MeshContact meshContact;
		v3BlockGridPairControl blockGridPair;
	};

	float restitution;
	float rollingResistance;
	b3Vec3 tangentVelocity;

	// This is monotonically advanced when a contact is allocated in this slot
	// Used to check for invalid b3ContactId
	uint32_t generation;
} b3Contact;

_Static_assert( v3_blockGridPairContactKind < UINT8_MAX, "Contact kind no longer fits its field" );

typedef struct b3ContactSpec
{
	int contactId;

	// Start of the global manifold constraint array
	int manifoldStart;
	uint16_t manifoldCount;
} b3ContactSpec;

b3DeclareArray( b3ContactSpec );

typedef struct b3ContactUpdateResult
{
	bool touching;
	bool completed;
} b3ContactUpdateResult;

void b3InitializeContactRegisters( void );

bool b3ContactStorageIsValid( const b3Contact* contact );
bool b3DestroyContactStorage( b3World* world, b3Contact* contact );

void b3CreateContact( b3World* world, b3Shape* shapeA, b3Shape* shapeB, int childIndex, bool createBlockGridPairs );
void b3DestroyContact( b3World* world, b3Contact* contact, bool wakeBodies );

// Ends each aggregate contact lifetime for a BlockGrid publication while retaining its pair slot.
void b3ResetBlockGridPairContactsForReplacement( b3World* world, b3Shape* shape );

b3ContactUpdateResult b3UpdateContact( b3World* world, int workerIndex, b3Contact* contact, b3Shape* shapeA, b3Vec3 localCenterA,
									   b3WorldTransform xfA, b3Shape* shapeB, b3Vec3 localCenterB, b3WorldTransform xfB,
									   bool isFast, b3Arena arena );

bool b3ComputeMeshManifolds( b3World* world, int workerIndex, b3Contact* contact, const b3Shape* shapeA, const int* materialMap,
							 b3WorldTransform xfA, const b3Shape* shapeB, b3WorldTransform xfB, bool isFast, b3Arena arena );

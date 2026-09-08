// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"

#include "algorithm.h"
#include "body.h"
#include "compound.h"
#include "constraint_graph.h"
#include "island.h"
#include "manifold.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"
#include "table.h"
#include "block_grid/block_grid.h"
#include "block_grid/block_grid_shape.h"

#include "box3d/box3d.h"

#include <limits.h>

// Contacts and determinism
// A deterministic simulation requires contacts to exist in the same order in b3Island no matter the thread count.
// The order must reproduce from run to run. This is necessary because the Gauss-Seidel constraint solver is order dependent.
//
// Creation:
// - Contacts are created using results from b3UpdateBroadPhasePairs
// - These results are ordered according to the order of the broad-phase move array
// - The move array is ordered according to the shape creation order using a bitset.
// - The island/shape/body order is determined by creation order
// - Logically contacts are only created for awake bodies, so they are immediately added to the awake contact array (serially)
//
// Island linking:
// - The awake contact array is built from the body-contact graph for all awake bodies in awake islands.
// - Awake contacts are solved in parallel and they generate contact state changes.
// - These state changes may link islands together using union find.
// - The state changes are ordered using a bit array that encompasses all contacts
// - As long as contacts are created in deterministic order, island link order is deterministic.
// - This keeps the order of contacts in islands deterministic

// Manifold functions should compute important results in local space to improve precision. However, this
// interface function takes two world transforms instead of a relative transform for these reasons:
//
// First:
// The anchors need to be computed relative to the shape origin in world space. This is necessary so the
// solver does not need to access static body transforms. Not even in constraint preparation. This approach
// has world space vectors yet retains precision.
//
// Second:
// b3ManifoldPoint::point is very useful for debugging and it is in world space.
//
// Third:
// The user may call the manifold functions directly and they should be easy to use and have easy to use
// results.
// typedef b3Manifold b3ManifoldFcn( const b3Shape* shapeA, b3Transform xfA, const b3Shape* shapeB, b3Transform xfB,
//								  b3ContactCache* cache );

static b3Contact* b3GetContactFullId( b3World* world, b3ContactId contactId )
{
	int id = contactId.index1 - 1;
	b3Contact* contact = b3Array_Get( world->contacts, id );
	B3_ASSERT( contact->contactId == id && contact->generation == contactId.generation );
	return contact;
}

b3ContactData b3Contact_GetData( b3ContactId contactId )
{
	b3World* world = b3GetWorld( contactId.world0 );
	b3Contact* contact = b3GetContactFullId( world, contactId );

	const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );

	b3ContactData data = { 0 };
	data.contactId = contactId;
	data.shapeIdA = (b3ShapeId){
		.index1 = shapeA->id + 1,
		.world0 = contactId.world0,
		.generation = shapeA->generation,
	};
	data.shapeIdB = (b3ShapeId){
		.index1 = shapeB->id + 1,
		.world0 = contactId.world0,
		.generation = shapeB->generation,
	};

	if ( contact->manifoldCount > 0 )
	{
		data.manifolds = contact->manifolds;
		data.manifoldCount = contact->manifoldCount;
	}
	else
	{
		data.manifolds = NULL;
		data.manifoldCount = 0;
	}

	return data;
}

struct b3ContactRegister
{
	// b3ManifoldFcn* fcn;
	bool supported;
	bool primary;
};

static struct b3ContactRegister s_registers[b3_shapeTypeCount][b3_shapeTypeCount];
static bool s_initialized = false;

bool b3ContactStorageIsValid( const b3Contact* contact )
{
	if ( contact == NULL || contact->kind >= b3_contactKindCount || contact->reserved != 0 ||
		 ( contact->manifoldCount == 0 ) != ( contact->manifolds == NULL ) )
	{
		return false;
	}

	b3ContactKind kind = (b3ContactKind)contact->kind;
	if ( kind == v3_blockGridPairContactKind )
	{
		return v3BlockGridPairContactIsValid( contact );
	}

	if ( kind == b3_meshContactKind )
	{
		const b3Array( b3TriangleCache )* cache = &contact->meshContact.triangleCache;
		return cache->count >= 0 && cache->capacity >= cache->count && ( cache->capacity == 0 ) == ( cache->data == NULL );
	}

	return kind == b3_convexContactKind;
}

bool b3DestroyContactStorage( b3World* world, b3Contact* contact )
{
	if ( world == NULL || b3ContactStorageIsValid( contact ) == false )
	{
		return false;
	}

	b3ContactKind kind = (b3ContactKind)contact->kind;
	if ( kind == v3_blockGridPairContactKind )
	{
		return v3BlockGridPairClear( world, contact );
	}

	b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
	contact->manifolds = NULL;
	contact->manifoldCount = 0;
	if ( kind == b3_meshContactKind )
	{
		b3Array_Destroy( contact->meshContact.triangleCache );
	}
	return true;
}

static void b3AddType( b3ShapeType type1, b3ShapeType type2 )
{
	B3_ASSERT( 0 <= type1 && type1 < b3_shapeTypeCount );
	B3_ASSERT( 0 <= type2 && type2 < b3_shapeTypeCount );

	s_registers[type1][type2].supported = true;
	s_registers[type1][type2].primary = true;

	if ( type1 != type2 )
	{
		s_registers[type2][type1].supported = true;
		s_registers[type2][type1].primary = false;
	}
}

void b3InitializeContactRegisters( void )
{
	if ( s_initialized == false )
	{
		b3AddType( b3_sphereShape, b3_sphereShape );
		b3AddType( b3_capsuleShape, b3_sphereShape );
		b3AddType( b3_capsuleShape, b3_capsuleShape );
		b3AddType( b3_compoundShape, b3_sphereShape );
		b3AddType( b3_compoundShape, b3_capsuleShape );
		b3AddType( b3_compoundShape, b3_hullShape );
		b3AddType( b3_hullShape, b3_sphereShape );
		b3AddType( b3_hullShape, b3_capsuleShape );
		b3AddType( b3_hullShape, b3_hullShape );
		b3AddType( b3_meshShape, b3_sphereShape );
		b3AddType( b3_meshShape, b3_capsuleShape );
		b3AddType( b3_meshShape, b3_hullShape );
		b3AddType( b3_heightShape, b3_sphereShape );
		b3AddType( b3_heightShape, b3_capsuleShape );
		b3AddType( b3_heightShape, b3_hullShape );

		// The grid resolves a candidate hitbox into a box hull and runs the same
		// convex manifold for sphere, capsule and hull contacts.
		b3AddType( v3_blockGridShape, b3_sphereShape );
		b3AddType( v3_blockGridShape, b3_capsuleShape );
		b3AddType( v3_blockGridShape, b3_hullShape );

		// BlockGrid pairs bypass this registration table. createBlockGridPairs controls their creation.
		s_initialized = true;
	}
}

void b3CreateContact( b3World* world, b3Shape* shapeA, b3Shape* shapeB, int childIndex, bool createBlockGridPairs )
{
	b3ShapeType typeA = shapeA->type;
	b3ShapeType typeB = shapeB->type;

	B3_ASSERT( 0 <= typeA && typeA < b3_shapeTypeCount );
	B3_ASSERT( 0 <= typeB && typeB < b3_shapeTypeCount );

	bool isBlockGridPair = typeA == v3_blockGridShape && typeB == v3_blockGridShape && childIndex == v3_blockGridPairChildIndex;
	if ( isBlockGridPair )
	{
		if ( createBlockGridPairs == false )
		{
			return;
		}
		if ( shapeB->id < shapeA->id )
		{
			b3Shape* swap = shapeA;
			shapeA = shapeB;
			shapeB = swap;
		}
	}
	else if ( s_registers[typeA][typeB].supported == false )
	{
		// For example, no mesh vs mesh collision
		return;
	}

	if ( isBlockGridPair == false && s_registers[typeA][typeB].primary == false )
	{
		// flip order
		b3CreateContact( world, shapeB, shapeA, childIndex, createBlockGridPairs );
		return;
	}

	b3Body* bodyA = b3Array_Get( world->bodies, shapeA->bodyId );
	b3Body* bodyB = b3Array_Get( world->bodies, shapeB->bodyId );

	B3_ASSERT( bodyA->setIndex != b3_disabledSet && bodyB->setIndex != b3_disabledSet );
	B3_ASSERT( bodyA->setIndex != b3_staticSet || bodyB->setIndex != b3_staticSet );

	int setIndex;
	if ( bodyA->setIndex == b3_awakeSet || bodyB->setIndex == b3_awakeSet )
	{
		setIndex = b3_awakeSet;
	}
	else
	{
		// sleeping and non-touching contacts live in the disabled set
		// later if this set is found to be touching then the sleeping
		// islands will be linked and the contact moved to the merged island

		// This is possible if a shape moves slightly then falls asleep
		setIndex = b3_disabledSet;
	}

	b3SolverSet* set = b3Array_Get( world->solverSets, setIndex );

	// Create contact key and contact
	int contactId = b3AllocId( &world->contactIdPool );
	if ( contactId == world->contacts.count )
	{
		b3Contact emptyContact = { 0 };
		b3Array_Push( world->contacts, emptyContact );
	}

	int shapeIdA = shapeA->id;
	int shapeIdB = shapeB->id;

	b3Contact* contact = b3Array_Get( world->contacts, contactId );
	int generation = contact->generation;
	*contact = (b3Contact){ 0 };
	contact->contactId = contactId;
	contact->generation = generation + 1;
	contact->setIndex = setIndex;
	contact->colorIndex = B3_NULL_INDEX;
	contact->localIndex = set->contactIndices.count;
	contact->islandId = B3_NULL_INDEX;
	contact->islandIndex = B3_NULL_INDEX;
	contact->shapeIdA = shapeIdA;
	contact->shapeIdB = shapeIdB;
	contact->childIndex = childIndex;
	contact->kind = b3_convexContactKind;
	// BlockGrid payloads are immutable, so a new pair starts with its first source epochs
	if ( isBlockGridPair && v3BlockGridPairInitializeContact( contact, 1, 1 ) == false )
	{
		contact->contactId = B3_NULL_INDEX;
		contact->setIndex = B3_NULL_INDEX;
		contact->colorIndex = B3_NULL_INDEX;
		contact->localIndex = B3_NULL_INDEX;
		b3FreeId( &world->contactIdPool, contactId );
		return;
	}

	// Both bodies must enable recycling
	if ( ( bodyA->flags & b3_bodyEnableContactRecycling ) != 0 && ( bodyB->flags & b3_bodyEnableContactRecycling ) != 0 )
	{
		contact->flags |= b3_contactRecycleFlag;
	}

	if ( isBlockGridPair == false && ( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape ) )
	{
		contact->kind = b3_meshContactKind;
	}
	else if ( isBlockGridPair == false && shapeA->type == b3_compoundShape )
	{
		b3ChildShape child = b3GetCompoundChild( shapeA->compound, childIndex );
		if ( child.type == b3_meshShape )
		{
			contact->kind = b3_meshContactKind;
		}
	}

	// todo impose these restrictions to make life easier
	B3_ASSERT( isBlockGridPair || shapeB->type == b3_sphereShape || shapeB->type == b3_capsuleShape ||
			   shapeB->type == b3_hullShape );
	// B3_ASSERT( bodyB->type != b3_staticBody );

	// Is either body static?
	// Note: it is possible to have a dynamic mesh collide with a static convex shape. Maybe I should disallow this.
	if ( bodyA->type == b3_staticBody || bodyB->type == b3_staticBody )
	{
		contact->flags |= b3_contactStaticFlag;
	}

	B3_ASSERT( shapeA->sensorIndex == B3_NULL_INDEX && shapeB->sensorIndex == B3_NULL_INDEX );

	if ( ( shapeA->flags & b3_enableContactEvents ) || ( shapeB->flags & b3_enableContactEvents ) )
	{
		contact->flags |= b3_contactEnableContactEvents;
	}

	if ( ( shapeA->flags & b3_enableSpeculative ) && ( shapeB->flags & b3_enableSpeculative ) )
	{
		contact->flags |= b3_enableSpeculativePoints;
	}

	// Connect to body A
	{
		contact->edges[0].bodyId = shapeA->bodyId;
		contact->edges[0].prevKey = B3_NULL_INDEX;
		contact->edges[0].nextKey = bodyA->headContactKey;

		int keyA = ( contactId << 1 ) | 0;
		int headContactKey = bodyA->headContactKey;
		if ( headContactKey != B3_NULL_INDEX )
		{
			b3Contact* headContact = b3Array_Get( world->contacts, headContactKey >> 1 );
			headContact->edges[headContactKey & 1].prevKey = keyA;
		}
		bodyA->headContactKey = keyA;
		bodyA->contactCount += 1;
	}

	// Connect to body B
	{
		contact->edges[1].bodyId = shapeB->bodyId;
		contact->edges[1].prevKey = B3_NULL_INDEX;
		contact->edges[1].nextKey = bodyB->headContactKey;

		int keyB = ( contactId << 1 ) | 1;
		int headContactKey = bodyB->headContactKey;
		if ( bodyB->headContactKey != B3_NULL_INDEX )
		{
			b3Contact* headContact = b3Array_Get( world->contacts, headContactKey >> 1 );
			headContact->edges[headContactKey & 1].prevKey = keyB;
		}
		bodyB->headContactKey = keyB;
		bodyB->contactCount += 1;
	}

	// Add to pair set for fast lookup
	uint64_t pairKey = b3ShapePairKey( shapeIdA, shapeIdB, childIndex );
	b3AddKey( &world->broadPhase.pairSet, pairKey );

	// Contacts are created as non-touching. Later if they are found to be touching
	// they will link islands and be moved into the constraint graph.
	b3Array_Push( set->contactIndices, contactId );

	float radiusA = 0.0f;
	if ( typeA == b3_sphereShape )
	{
		radiusA = shapeA->sphere.radius;
	}
	else if ( typeA == b3_capsuleShape )
	{
		radiusA = shapeA->capsule.radius;
	}

	float radiusB = 0.0f;
	if ( typeB == b3_sphereShape )
	{
		radiusB = shapeB->sphere.radius;
	}
	else if ( typeB == b3_capsuleShape )
	{
		radiusB = shapeB->capsule.radius;
	}

	float maxRadius = b3MaxFloat( radiusA, radiusB );

	// Assuming the rolling resistance doesn't change
	contact->rollingResistance =
		b3MaxFloat( b3GetShapeMaterials( shapeA )[0].rollingResistance, b3GetShapeMaterials( shapeB )[0].rollingResistance ) *
		maxRadius;

	if ( ( shapeA->flags & b3_enablePreSolveEvents ) || ( shapeB->flags & b3_enablePreSolveEvents ) )
	{
		contact->flags |= b3_simEnablePreSolveEvents;
	}
}

// A contact is destroyed when:
// - broad-phase proxies stop overlapping
// - a body is destroyed
// - a body is disabled
// - a body changes type from dynamic to kinematic or static
// - a shape is destroyed
// - contact filtering is modified
void b3DestroyContact( b3World* world, b3Contact* contact, bool wakeBodies )
{
	if ( b3DestroyContactStorage( world, contact ) == false )
	{
		B3_ASSERT( false );
		return;
	}

	// Remove pair from set
	uint64_t pairKey = b3ShapePairKey( contact->shapeIdA, contact->shapeIdB, contact->childIndex );
	b3RemoveKey( &world->broadPhase.pairSet, pairKey );

	b3ContactEdge* edgeA = contact->edges + 0;
	b3ContactEdge* edgeB = contact->edges + 1;

	int bodyIdA = edgeA->bodyId;
	int bodyIdB = edgeB->bodyId;
	b3Body* bodyA = b3Array_Get( world->bodies, bodyIdA );
	b3Body* bodyB = b3Array_Get( world->bodies, bodyIdB );

	uint32_t flags = contact->flags;
	bool touching = ( flags & b3_contactTouchingFlag ) != 0;

	// End touch event
	if ( touching && ( flags & b3_contactEnableContactEvents ) != 0 )
	{
		uint16_t worldId = world->worldId;
		const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
		const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
		b3ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->generation };
		b3ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->generation };

		b3ContactId contactId = {
			.index1 = contact->contactId + 1,
			.world0 = world->worldId,
			.padding = 0,
			.generation = contact->generation,
		};

		b3ContactEndTouchEvent event = {
			.shapeIdA = shapeIdA,
			.shapeIdB = shapeIdB,
			.contactId = contactId,
		};

		b3Array_Push( world->contactEndEvents[world->endEventArrayIndex], event );
	}

	// Remove from body A
	if ( edgeA->prevKey != B3_NULL_INDEX )
	{
		b3Contact* prevContact = b3Array_Get( world->contacts, edgeA->prevKey >> 1 );
		b3ContactEdge* prevEdge = prevContact->edges + ( edgeA->prevKey & 1 );
		prevEdge->nextKey = edgeA->nextKey;
	}

	if ( edgeA->nextKey != B3_NULL_INDEX )
	{
		b3Contact* nextContact = b3Array_Get( world->contacts, edgeA->nextKey >> 1 );
		b3ContactEdge* nextEdge = nextContact->edges + ( edgeA->nextKey & 1 );
		nextEdge->prevKey = edgeA->prevKey;
	}

	int contactId = contact->contactId;

	int edgeKeyA = ( contactId << 1 ) | 0;
	if ( bodyA->headContactKey == edgeKeyA )
	{
		bodyA->headContactKey = edgeA->nextKey;
	}

	bodyA->contactCount -= 1;

	// Remove from body B
	if ( edgeB->prevKey != B3_NULL_INDEX )
	{
		b3Contact* prevContact = b3Array_Get( world->contacts, edgeB->prevKey >> 1 );
		b3ContactEdge* prevEdge = prevContact->edges + ( edgeB->prevKey & 1 );
		prevEdge->nextKey = edgeB->nextKey;
	}

	if ( edgeB->nextKey != B3_NULL_INDEX )
	{
		b3Contact* nextContact = b3Array_Get( world->contacts, edgeB->nextKey >> 1 );
		b3ContactEdge* nextEdge = nextContact->edges + ( edgeB->nextKey & 1 );
		nextEdge->prevKey = edgeB->prevKey;
	}

	int edgeKeyB = ( contactId << 1 ) | 1;
	if ( bodyB->headContactKey == edgeKeyB )
	{
		bodyB->headContactKey = edgeB->nextKey;
	}

	bodyB->contactCount -= 1;

	// Remove contact from the array that owns it
	if ( contact->islandId != B3_NULL_INDEX )
	{
		b3UnlinkContact( world, contact );
	}

	if ( contact->colorIndex != B3_NULL_INDEX )
	{
		// contact is an active constraint
		B3_ASSERT( contact->setIndex == b3_awakeSet );
		b3RemoveContactFromGraph( world, bodyIdA, bodyIdB, contact->colorIndex, contact->localIndex,
								  (b3ContactKind)contact->kind );
	}
	else
	{
		// contact is non-touching or is sleeping or is a sensor
		B3_ASSERT( contact->setIndex != b3_awakeSet || ( contact->flags & b3_contactTouchingFlag ) == 0 );
		b3SolverSet* set = b3Array_Get( world->solverSets, contact->setIndex );

		int localIndex = contact->localIndex;
		int movedIndex = b3Array_RemoveSwap( set->contactIndices, localIndex );
		if ( movedIndex != B3_NULL_INDEX )
		{
			int movedContactIndex = set->contactIndices.data[localIndex];
			b3Contact* movedContact = b3Array_Get( world->contacts, movedContactIndex );
			movedContact->localIndex = localIndex;
		}
	}

	// Free contact and id (preserve generation)
	contact->contactId = B3_NULL_INDEX;
	contact->setIndex = B3_NULL_INDEX;
	contact->colorIndex = B3_NULL_INDEX;
	contact->localIndex = B3_NULL_INDEX;
	contact->kind = b3_convexContactKind;
	contact->reserved = 0;
	contact->convexContact = (b3ConvexContact){ 0 };
	b3FreeId( &world->contactIdPool, contactId );

	if ( wakeBodies && touching )
	{
		b3WakeBody( world, bodyA );
		b3WakeBody( world, bodyB );
	}
}

static bool b3BlockGridPairReplacementEventsCanFit( const b3World* world, int contactCapacity )
{
	if ( contactCapacity < 0 || contactCapacity > INT_MAX / (int)sizeof( b3ContactBeginTouchEvent ) )
	{
		return false;
	}
	for ( int slot = 0; slot < 2; ++slot )
	{
		int count = world->contactEndEvents[slot].count;
		if ( count < 0 || count > INT_MAX - contactCapacity ||
			 count + contactCapacity > INT_MAX / (int)sizeof( b3ContactEndTouchEvent ) )
		{
			return false;
		}
	}
	return true;
}

void b3ResetBlockGridPairContactsForReplacement( b3World* world, b3Shape* shape )
{
	B3_ASSERT( world != NULL && shape != NULL && shape->type == v3_blockGridShape );
	if ( world == NULL || shape == NULL || shape->type != v3_blockGridShape )
	{
		return;
	}

	int contactCapacity = world->contacts.count;
	bool valid = b3BlockGridPairReplacementEventsCanFit( world, contactCapacity );
	for ( int contactKey = world->bodies.data[shape->bodyId].headContactKey; valid && contactKey != B3_NULL_INDEX; )
	{
		const b3Contact* contact = b3Array_Get( world->contacts, contactKey >> 1 );
		int edgeIndex = contactKey & 1;
		contactKey = contact->edges[edgeIndex].nextKey;
		if ( ( contact->shapeIdA == shape->id || contact->shapeIdB == shape->id ) &&
			 contact->kind == v3_blockGridPairContactKind )
		{
			valid = v3BlockGridPairContactIsValid( contact ) &&
					( contact->blockGridPair.state == NULL ||
					  v3BlockGridPairStateBelongsToWorld( contact->blockGridPair.state, world ) );
		}
	}
	B3_ASSERT( valid );
	if ( valid == false )
	{
		return;
	}

	b3Array_Reserve( world->contactBeginEvents, contactCapacity );
	for ( int slot = 0; slot < 2; ++slot )
	{
		int target = world->contactEndEvents[slot].count + contactCapacity;
		b3Array_Reserve( world->contactEndEvents[slot], target );
	}

	b3Body* shapeBody = b3Array_Get( world->bodies, shape->bodyId );
	int contactKey = shapeBody->headContactKey;
	while ( contactKey != B3_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;
		b3Contact* contact = b3Array_Get( world->contacts, contactId );
		contactKey = contact->edges[edgeIndex].nextKey;
		if ( ( contact->shapeIdA != shape->id && contact->shapeIdB != shape->id ) ||
			 contact->kind != v3_blockGridPairContactKind )
		{
			continue;
		}

		b3Body* bodyA = b3Array_Get( world->bodies, contact->edges[0].bodyId );
		b3Body* bodyB = b3Array_Get( world->bodies, contact->edges[1].bodyId );
		bool touching = ( contact->flags & b3_contactTouchingFlag ) != 0;
		if ( touching && ( contact->flags & b3_contactEnableContactEvents ) != 0 )
		{
			const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
			const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
			b3ContactEndTouchEvent event = {
				.shapeIdA = { shapeA->id + 1, world->worldId, shapeA->generation },
				.shapeIdB = { shapeB->id + 1, world->worldId, shapeB->generation },
				.contactId = { contact->contactId + 1, world->worldId, 0, contact->generation },
			};
			b3Array_Push( world->contactEndEvents[world->endEventArrayIndex], event );
		}

		b3WakeBody( world, bodyA );
		b3WakeBody( world, bodyB );
		if ( touching )
		{
			B3_ASSERT( contact->setIndex == b3_awakeSet && contact->colorIndex != B3_NULL_INDEX &&
					   contact->islandId != B3_NULL_INDEX );
			int colorIndex = contact->colorIndex;
			int localIndex = contact->localIndex;
			int bodyIdA = contact->edges[0].bodyId;
			int bodyIdB = contact->edges[1].bodyId;
			b3UnlinkContact( world, contact );

			b3SolverSet* awakeSet = b3Array_Get( world->solverSets, b3_awakeSet );
			contact->colorIndex = B3_NULL_INDEX;
			contact->localIndex = awakeSet->contactIndices.count;
			b3Array_Push( awakeSet->contactIndices, contact->contactId );
			b3RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex, (b3ContactKind)contact->kind );
		}

		contact->bodySimIndexA = B3_NULL_INDEX;
		contact->bodySimIndexB = B3_NULL_INDEX;
		uint32_t persistentFlags =
			contact->flags & ( b3_contactEnableContactEvents | b3_contactStaticFlag | b3_contactRecycleFlag |
							   b3_simEnablePreSolveEvents | b3_enableSpeculativePoints );
		contact->flags = persistentFlags;
		contact->cachedRotationA = b3Quat_identity;
		contact->cachedRotationB = b3Quat_identity;
		contact->cachedRelativePose = b3Transform_identity;
		contact->friction = 0.0f;
		contact->restitution = 0.0f;
		contact->rollingResistance = 0.0f;
		contact->tangentVelocity = b3Vec3_zero;
		contact->generation += 1;
		bool retained = v3BlockGridPairRetainCapacityForReplacement( contact );
		B3_ASSERT( retained );
		B3_UNUSED( retained );
	}
}

static bool b3ComputeConvexManifold( b3World* world, int workerIndex, b3Contact* contact, const b3Shape* shapeA,
									 b3WorldTransform xfA, const b3Shape* shapeB, b3WorldTransform xfB, b3Arena arena )
{
	b3ShapeType typeA = shapeA->type;
	b3ShapeType typeB = shapeB->type;

	b3ContactCache* cache = &contact->convexContact.cache;

	int pointCapacity = 32;
	b3LocalManifoldPoint* pointBuffer = (b3LocalManifoldPoint*)b3Bump( &arena, pointCapacity * sizeof( b3LocalManifoldPoint ) );

	b3LocalManifold geomManifold = { 0 };
	geomManifold.points = pointBuffer;

	b3Transform transformBtoA = b3InvMulWorldTransforms( xfA, xfB );

	if ( typeA == b3_sphereShape )
	{
		B3_ASSERT( typeB == b3_sphereShape );
		b3CollideSpheres( &geomManifold, pointCapacity, &shapeA->sphere, &shapeB->sphere, transformBtoA );
	}
	else if ( typeA == b3_capsuleShape )
	{
		if ( typeB == b3_sphereShape )
		{
			b3CollideCapsuleAndSphere( &geomManifold, pointCapacity, &shapeA->capsule, &shapeB->sphere, transformBtoA );
		}
		else
		{
			B3_ASSERT( typeB == b3_capsuleShape );
			b3CollideCapsules( &geomManifold, pointCapacity, &shapeA->capsule, &shapeB->capsule, transformBtoA );
		}
	}
	else
	{
		B3_ASSERT( typeA == b3_hullShape );

		if ( typeB == b3_sphereShape )
		{
			b3CollideHullAndSphere( &geomManifold, pointCapacity, shapeA->hull, &shapeB->sphere, transformBtoA,
									&cache->simplexCache );
		}
		else if ( typeB == b3_capsuleShape )
		{
			b3CollideHullAndCapsule( &geomManifold, pointCapacity, shapeA->hull, &shapeB->capsule, transformBtoA,
									 &cache->simplexCache );
		}
		else
		{
			B3_ASSERT( typeB == b3_hullShape );
			b3CollideHulls( &geomManifold, pointCapacity, shapeA->hull, shapeB->hull, transformBtoA, &cache->satCache );
			world->taskContexts.data[workerIndex].satCallCount += 1;
			world->taskContexts.data[workerIndex].satCacheHitCount += cache->satCache.hit;
		}
	}

	if ( geomManifold.pointCount == 0 )
	{
		if ( contact->manifoldCount > 0 )
		{
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
			contact->manifolds = NULL;
			contact->manifoldCount = 0;
		}

		return false;
	}

	b3ManifoldPoint oldPoints[B3_MAX_MANIFOLD_POINTS];
	int oldCount = 0;

	if ( contact->manifoldCount == 0 )
	{
		contact->manifolds = b3AllocateManifolds( world, 1 );
		contact->manifoldCount = 1;
	}
	else
	{
		oldCount = contact->manifolds[0].pointCount;
		memcpy( oldPoints, contact->manifolds[0].points, oldCount * sizeof( b3ManifoldPoint ) );
	}

	b3Manifold* manifold = contact->manifolds;
	manifold->pointCount = geomManifold.pointCount;

	b3Matrix3 matrixA = b3MakeMatrixFromQuat( xfA.q );
	manifold->normal = b3MulMV( matrixA, geomManifold.normal );

	// Store point data in contact
	for ( int i = 0; i < geomManifold.pointCount; ++i )
	{
		const b3LocalManifoldPoint* source = geomManifold.points + i;
		b3ManifoldPoint* target = manifold->points + i;

		// Contact points are computed in frame A
		target->anchorA = b3MulMV( matrixA, source->point );
		target->anchorB = b3Add( target->anchorA, b3SubPos( xfA.p, xfB.p ) );
		target->separation = source->separation;
		target->featureId = b3MakeFeatureId( source->pair );
		target->triangleIndex = B3_NULL_INDEX;
		target->normalVelocity = 0.0f;
	}

	// Copy impulses from old points
	for ( int i = 0; i < geomManifold.pointCount; ++i )
	{
		b3ManifoldPoint* pt2 = manifold->points + i;
		pt2->totalNormalImpulse = 0.0f;
		pt2->persisted = false;

		for ( int j = 0; j < oldCount; ++j )
		{
			b3ManifoldPoint* pt1 = oldPoints + j;

			if ( pt2->featureId == pt1->featureId )
			{
				pt2->normalImpulse = pt1->normalImpulse;
				pt2->persisted = true;

				// claimed
				pt1->featureId = UINT32_MAX;

				break;
			}
		}

		if ( pt2->persisted == false )
		{
			pt2->normalImpulse = 0.0f;
		}
	}

	return true;
}

bool b3ResolveContactMaterial( const b3World* world, const b3SurfaceMaterial* materialA, b3Quat rotationA,
							   const b3SurfaceMaterial* materialB, b3Quat rotationB, float effectiveRadius,
							   b3ContactMaterial* contactMaterial )
{
	if ( world == NULL || materialA == NULL || materialB == NULL || contactMaterial == NULL ||
		 b3IsValidFloat( effectiveRadius ) == false || effectiveRadius < 0.0f )
	{
		return false;
	}

	float friction =
		world->frictionCallback( materialA->friction, materialA->userMaterialId, materialB->friction, materialB->userMaterialId );
	float restitution = world->restitutionCallback( materialA->restitution, materialA->userMaterialId, materialB->restitution,
													materialB->userMaterialId );
	float rollingResistance = b3MaxFloat( materialA->rollingResistance, materialB->rollingResistance ) * effectiveRadius;
	b3Vec3 tangentVelocityA = b3RotateVector( rotationA, materialA->tangentVelocity );
	b3Vec3 tangentVelocityB = b3RotateVector( rotationB, materialB->tangentVelocity );
	b3Vec3 tangentVelocity = b3Sub( tangentVelocityA, tangentVelocityB );
	if ( b3IsValidFloat( friction ) == false || friction < 0.0f || b3IsValidFloat( restitution ) == false || restitution < 0.0f ||
		 b3IsValidFloat( rollingResistance ) == false || b3IsValidVec3( tangentVelocity ) == false )
	{
		return false;
	}

	*contactMaterial = (b3ContactMaterial){
		.friction = friction,
		.restitution = restitution,
		.rollingResistance = rollingResistance,
		.tangentVelocity = tangentVelocity,
	};
	return true;
}

static float b3GetConvexRollingRadius( const b3Shape* shape )
{
	switch ( shape->type )
	{
		case b3_sphereShape:
			return shape->sphere.radius;
		case b3_capsuleShape:
			return shape->capsule.radius;
		case b3_hullShape:
			return 0.25f * shape->hull->innerRadius;
		default:
			return 0.0f;
	}
}

static void b3UpdateHitEventFlag( b3Contact* contact, const b3Shape* shapeA, const b3Shape* shapeB, bool touching )
{
	if ( touching && ( ( shapeA->flags & b3_enableHitEvents ) || ( shapeB->flags & b3_enableHitEvents ) ) )
	{
		contact->flags |= b3_simEnableHitEvent;
	}
	else
	{
		contact->flags &= ~b3_simEnableHitEvent;
	}
}

static bool v3BlockGridPairPassesPreSolve( b3World* world, const b3Contact* contact, const b3Shape* shapeA, b3Vec3 localCenterA,
										   b3WorldTransform transformA, const b3Shape* shapeB )
{
	if ( world->preSolveFcn == NULL || ( contact->flags & b3_simEnablePreSolveEvents ) == 0 )
	{
		return true;
	}

	B3_ASSERT( contact->manifoldCount > 0 && contact->manifolds[0].pointCount > 0 );
	b3ShapeId shapeIdA = { shapeA->id + 1, world->worldId, shapeA->generation };
	b3ShapeId shapeIdB = { shapeB->id + 1, world->worldId, shapeB->generation };
	b3Pos centerA = b3OffsetPos( transformA.p, b3RotateVector( transformA.q, localCenterA ) );
	b3Pos point = b3OffsetPos( centerA, contact->manifolds[0].points[0].anchorA );

	// Sorted manifolds make the first point a stable parent-level callback
	return world->preSolveFcn( shapeIdA, shapeIdB, point, contact->manifolds[0].normal, world->preSolveContext );
}

static bool b3UpdateConvexContact( b3World* world, int workerIndex, b3Contact* contact, b3Shape* shapeA, b3WorldTransform xfA,
								   b3Shape* shapeB, b3WorldTransform xfB, bool flip, b3Arena arena )
{
	// Compute new manifold
	bool touching = b3ComputeConvexManifold( world, workerIndex, contact, shapeA, xfA, shapeB, xfB, arena );

	if ( touching == false )
	{
		B3_ASSERT( contact->manifolds == NULL && contact->manifoldCount == 0 );
		return false;
	}

	B3_ASSERT( contact->manifoldCount == 1 );

	if ( flip )
	{
		// Not flipping the feature ids because they just need to match and flipping is consistent.
		b3Manifold* manifold = contact->manifolds + 0;
		manifold->normal = b3Neg( manifold->normal );
		int pointCount = manifold->pointCount;
		for ( int i = 0; i < pointCount; ++i )
		{
			b3ManifoldPoint* mp = manifold->points + i;
			B3_SWAP( mp->anchorA, mp->anchorB );
		}
	}

	const b3SurfaceMaterial* materialA = b3GetShapeMaterials( shapeA );
	const b3SurfaceMaterial* materialB = b3GetShapeMaterials( shapeB );

	float effectiveRadius = 0.0f;
	if ( materialA->rollingResistance > 0.0f || materialB->rollingResistance > 0.0f )
	{
		effectiveRadius = b3MaxFloat( b3GetConvexRollingRadius( shapeA ), b3GetConvexRollingRadius( shapeB ) );
	}

	b3ContactMaterial contactMaterial;
	if ( b3ResolveContactMaterial( world, materialA, xfA.q, materialB, xfB.q, effectiveRadius, &contactMaterial ) == false )
	{
		b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
		contact->manifolds = NULL;
		contact->manifoldCount = 0;
		return false;
	}
	contact->friction = contactMaterial.friction;
	contact->restitution = contactMaterial.restitution;
	contact->rollingResistance = contactMaterial.rollingResistance;
	contact->tangentVelocity = contactMaterial.tangentVelocity;

	if ( world->preSolveFcn && ( contact->flags & b3_simEnablePreSolveEvents ) != 0 )
	{
		b3ShapeId shapeIdA = { shapeA->id + 1, world->worldId, shapeA->generation };
		b3ShapeId shapeIdB = { shapeB->id + 1, world->worldId, shapeB->generation };

		// this call assumes thread safety
		b3Pos point = b3OffsetPos( xfA.p, contact->manifolds[0].points[0].anchorA );
		b3Vec3 normal = contact->manifolds[0].normal;
		touching = world->preSolveFcn( shapeIdA, shapeIdB, point, normal, world->preSolveContext );
		if ( touching == false )
		{
			// disable contact
			b3FreeManifolds( world, contact->manifolds, contact->manifoldCount );
			contact->manifolds = NULL;
			contact->manifoldCount = 0;
			return false;
		}
	}

	return true;
}

// Update the contact manifold and touching status
// The caller must not assume that the shape AABBs overlap or are valid
b3ContactUpdateResult b3UpdateContact( b3World* world, int workerIndex, b3Contact* contact, b3Shape* shapeA, b3Vec3 localCenterA,
									   b3WorldTransform xfA, b3Shape* shapeB, b3Vec3 localCenterB, b3WorldTransform xfB,
									   bool isFast, b3Arena arena )
{
	if ( contact->kind == v3_blockGridPairContactKind )
	{
		// Aggregate manifolds already store center-of-mass anchors, so this phase only publishes their touch state
		b3ContactUpdateResult result = { .touching = ( contact->flags & b3_simTouchingFlag ) != 0 };
		if ( shapeA->type != v3_blockGridShape || shapeB->type != v3_blockGridShape ||
			 v3BlockGridPairContactIsValid( contact ) == false || contact->blockGridPair.sourceEpochA == 0 ||
			 contact->blockGridPair.sourceEpochB == 0 )
		{
			return result;
		}

		v3BlockGridPairOutcome outcome = (v3BlockGridPairOutcome)contact->blockGridPair.lastOutcome;
		bool completedSeparated = outcome == v3_blockGridPairOutcomeSeparated && contact->manifoldCount == 0;
		bool completedTouching = outcome == v3_blockGridPairOutcomeTouching && contact->manifoldCount > 0;
		if ( completedSeparated == false && completedTouching == false )
		{
			// The pair update has not reached this contact this step, so its touch state stands
			return result;
		}

		result.touching = completedTouching;
		if ( result.touching && v3BlockGridPairPassesPreSolve( world, contact, shapeA, localCenterA, xfA, shapeB ) == false )
		{
			// The pair still overlaps, but the callback removes it from the solver for this step
			if ( v3BlockGridPairClear( world, contact ) == false )
			{
				return result;
			}
			result.touching = false;
		}
		if ( result.touching )
		{
			contact->flags |= b3_simTouchingFlag;
		}
		else
		{
			contact->flags &= ~b3_simTouchingFlag;
		}
		b3UpdateHitEventFlag( contact, shapeA, shapeB, result.touching );
		result.completed = true;
		return result;
	}

	bool touching;

	B3_ASSERT( shapeB->type != b3_compoundShape && shapeB->type != v3_blockGridShape );

	if ( shapeA->type == v3_blockGridShape )
	{
		// Resolve the contact hitbox into a temporary box hull and run the convex manifold function.
		int hitboxIndex = contact->childIndex;
		b3BoxHull box = v3MakeBlockGridHitboxHull( shapeA->blockGrid, hitboxIndex );

		b3Shape childShapeA;
		memcpy( &childShapeA, shapeA, sizeof( b3Shape ) );
		childShapeA.type = b3_hullShape;
		childShapeA.hull = &box.base;

		// Hitbox material, so a grid may mix surfaces the way a compound does.
		uint32_t materialIndex = v3BlockGrid_GetHitboxMaterial( shapeA->blockGrid, hitboxIndex );
		const b3SurfaceMaterial* parentMaterials = b3GetShapeMaterials( shapeA );
		if ( materialIndex < (uint32_t)shapeA->materialCount )
		{
			childShapeA.material = parentMaterials[materialIndex];
		}
		childShapeA.materials = NULL;
		childShapeA.materialCount = 1;

		bool flip = false;
		touching = b3UpdateConvexContact( world, workerIndex, contact, &childShapeA, xfA, shapeB, xfB, flip, arena );
	}
	else if ( shapeA->type == b3_compoundShape )
	{
		int childIndex = contact->childIndex;
		b3ChildShape child = b3GetCompoundChild( shapeA->compound, childIndex );

		// Temporary child shape to match existing function signatures
		b3Shape childShapeA;
		memcpy( &childShapeA, shapeA, sizeof( b3Shape ) );

		childShapeA.type = child.type;

		// Handle child material for non-meshes.
		if ( child.type != b3_meshShape )
		{
			B3_ASSERT( 0 <= child.materialIndices[0] && child.materialIndices[0] < shapeA->materialCount );
			const b3SurfaceMaterial* parentMaterials = b3GetShapeMaterials( shapeA );
			childShapeA.material = parentMaterials[child.materialIndices[0]];
			childShapeA.materials = NULL;
			childShapeA.materialCount = 1;
		}

		if ( child.type == b3_capsuleShape )
		{
			childShapeA.capsule = child.capsule;
			if ( shapeB->type == b3_hullShape )
			{
				// Flip
				bool flip = true;
				touching = b3UpdateConvexContact( world, workerIndex, contact, shapeB, xfB, &childShapeA, xfA, flip, arena );
			}
			else
			{
				bool flip = false;
				touching = b3UpdateConvexContact( world, workerIndex, contact, &childShapeA, xfA, shapeB, xfB, flip, arena );
			}
		}
		else if ( child.type == b3_hullShape )
		{
			childShapeA.hull = child.hull;
			b3WorldTransform xfChild = b3MulWorldTransforms( xfA, child.transform );
			bool flip = false;
			touching = b3UpdateConvexContact( world, workerIndex, contact, &childShapeA, xfChild, shapeB, xfB, flip, arena );
		}
		else if ( child.type == b3_meshShape )
		{
			childShapeA.mesh = child.mesh;
			b3WorldTransform xfChild = b3MulWorldTransforms( xfA, child.transform );

			touching = b3ComputeMeshManifolds( world, workerIndex, contact, &childShapeA, child.materialIndices, xfChild, shapeB,
											   xfB, isFast, arena );

			B3_ASSERT( ( touching == true && contact->manifoldCount > 0 ) ||
					   ( touching == false && contact->manifoldCount == 0 ) );
		}
		else
		{
			B3_ASSERT( child.type == b3_sphereShape );

			childShapeA.sphere = child.sphere;
			if ( shapeB->type == b3_capsuleShape || shapeB->type == b3_hullShape )
			{
				// Flip
				bool flip = true;
				touching = b3UpdateConvexContact( world, workerIndex, contact, shapeB, xfB, &childShapeA, xfA, flip, arena );
			}
			else
			{
				bool flip = false;
				touching = b3UpdateConvexContact( world, workerIndex, contact, &childShapeA, xfA, shapeB, xfB, flip, arena );
			}
		}

		// The anchor is relative to the child origin but oriented in world space.
		// Offset the anchor to be relative to the compound origin.
		int manifoldCount = contact->manifoldCount;
		b3Vec3 offset = b3RotateVector( xfA.q, child.transform.p );
		for ( int i = 0; i < manifoldCount; ++i )
		{
			b3Manifold* manifold = contact->manifolds + i;
			int pointCount = manifold->pointCount;
			for ( int j = 0; j < pointCount; ++j )
			{
				b3ManifoldPoint* mp = manifold->points + j;
				mp->anchorA = b3Add( mp->anchorA, offset );
			}
		}
	}
	else if ( shapeA->type == b3_meshShape || shapeA->type == b3_heightShape )
	{
		// Does this contact touch a mesh or height-field?

		// Compute mesh manifolds
		touching = b3ComputeMeshManifolds( world, workerIndex, contact, shapeA, NULL, xfA, shapeB, xfB, isFast, arena );

		B3_ASSERT( ( touching == true && contact->manifoldCount > 0 ) || ( touching == false && contact->manifoldCount == 0 ) );
	}
	else
	{
		// Convex-vs-convex
		bool flip = false;
		touching = b3UpdateConvexContact( world, workerIndex, contact, shapeA, xfA, shapeB, xfB, flip, arena );
	}

	if ( touching )
	{
		b3Vec3 centerA = b3RotateVector( xfA.q, localCenterA );
		b3Vec3 centerB = b3RotateVector( xfB.q, localCenterB );

		// Adjust anchors to be relative to center of mass
		for ( int i = 0; i < contact->manifoldCount; ++i )
		{
			b3Manifold* manifold = contact->manifolds + i;
			for ( int j = 0; j < manifold->pointCount; ++j )
			{
				b3ManifoldPoint* mp = manifold->points + j;
				mp->anchorA = b3Sub( mp->anchorA, centerA );
				mp->anchorB = b3Sub( mp->anchorB, centerB );
			}
		}

		contact->flags |= b3_simTouchingFlag;
	}
	else
	{
		contact->flags &= ~b3_simTouchingFlag;
	}
	b3UpdateHitEventFlag( contact, shapeA, shapeB, touching );

	return (b3ContactUpdateResult){ .touching = touching, .completed = true };
}

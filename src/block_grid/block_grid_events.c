// SPDX-License-Identifier: MIT

#include "block_grid_events.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "physics_world.h"
#include "shape.h"
#include "block_grid.h"
#include "block_grid_contact.h"
#include "block_grid_internal.h"

#include "box3d/box3d.h"

#include <math.h>
#include <string.h>

static uint32_t v3SaturatingAdd( uint32_t value, uint32_t addend )
{
	return UINT32_MAX - value < addend ? UINT32_MAX : value + addend;
}

static bool v3SameShapeId( b3ShapeId a, b3ShapeId b )
{
	return a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation;
}

static bool v3SameSideIdentity( const v3BlockContactSide* a, const v3BlockContactSide* b )
{
	return v3SameShapeId( a->shapeId, b->shapeId ) && a->isBlockGrid == b->isBlockGrid && a->cellX == b->cellX &&
		   a->cellY == b->cellY && a->cellZ == b->cellZ && a->subHitboxIndex == b->subHitboxIndex;
}

static bool v3SameRecordIdentity( const v3BlockContactRecord* a, const v3BlockContactRecord* b )
{
	return a->contactId == b->contactId && a->contactGeneration == b->contactGeneration &&
		   v3SameSideIdentity( &a->event.sideA, &b->event.sideA ) && v3SameSideIdentity( &a->event.sideB, &b->event.sideB );
}

static int v3FindRecord( const b3Array( v3BlockContactRecord ) * records, const v3BlockContactRecord* sought )
{
	for ( int i = 0; i < records->count; ++i )
	{
		if ( v3SameRecordIdentity( records->data + i, sought ) )
		{
			return i;
		}
	}
	return B3_NULL_INDEX;
}

static void v3PushEvent( b3Array( v3BlockContactEvent ) * events, v3BlockContactEvent event, uint32_t* dropped )
{
	if ( events->count < events->capacity )
	{
		events->data[events->count++] = event;
	}
	else
	{
		*dropped = v3SaturatingAdd( *dropped, 1 );
	}
}

static bool v3AllocateEventArray( b3Array( v3BlockContactEvent ) * array )
{
	array->data = b3TryAlloc( V3_BLOCK_CONTACT_EVENT_CAPACITY * sizeof( v3BlockContactEvent ) );
	if ( array->data == NULL )
	{
		return false;
	}
	array->count = 0;
	array->capacity = V3_BLOCK_CONTACT_EVENT_CAPACITY;
	return true;
}

static bool v3AllocateRecordArray( b3Array( v3BlockContactRecord ) * array )
{
	array->data = b3TryAlloc( V3_BLOCK_CONTACT_EVENT_CAPACITY * sizeof( v3BlockContactRecord ) );
	if ( array->data == NULL )
	{
		return false;
	}
	array->count = 0;
	array->capacity = V3_BLOCK_CONTACT_EVENT_CAPACITY;
	return true;
}

bool v3BlockContactEventsReserve( b3World* world )
{
	if ( world->blockContactBeginEvents.capacity != 0 )
	{
		return true;
	}

	bool ok = v3AllocateEventArray( &world->blockContactBeginEvents ) && v3AllocateEventArray( &world->blockContactHitEvents ) &&
			  v3AllocateEventArray( world->blockContactEndEvents + 0 ) &&
			  v3AllocateEventArray( world->blockContactEndEvents + 1 ) &&
			  v3AllocateRecordArray( world->blockContactStates + 0 ) && v3AllocateRecordArray( world->blockContactStates + 1 );
	if ( ok == false )
	{
		v3BlockContactEventsDestroy( world );
		return false;
	}
	return true;
}

void v3BlockContactEventsDestroy( b3World* world )
{
	b3Array_Destroy( world->blockContactBeginEvents );
	b3Array_Destroy( world->blockContactHitEvents );
	b3Array_Destroy( world->blockContactEndEvents[0] );
	b3Array_Destroy( world->blockContactEndEvents[1] );
	b3Array_Destroy( world->blockContactStates[0] );
	b3Array_Destroy( world->blockContactStates[1] );
}

static bool v3ResolveGridSide( v3BlockContactSide* side, const b3Shape* shape, b3WorldTransform transform, b3Pos point,
							   int hitboxIndex )
{
	v3BlockGridIdentity identity;
	if ( v3BlockGrid_ResolveIdentity( shape->blockGrid, transform, point, hitboxIndex, &identity ) == false )
	{
		return false;
	}

	side->isBlockGrid = true;
	side->cellX = identity.cellX;
	side->cellY = identity.cellY;
	side->cellZ = identity.cellZ;
	side->subHitboxIndex = identity.cellBoxIndex;
	side->materialIndex = identity.materialIndex;
	side->userMaterialId = identity.userMaterialId;
	side->userData = identity.userData;
	return true;
}

static bool v3MakeRecord( b3World* world, const b3Contact* contact, int manifoldIndex, int pointIndex,
						  v3BlockContactRecord* record )
{
	const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
	const b3Body* bodyA = b3Array_Get( world->bodies, shapeA->bodyId );
	const b3Body* bodyB = b3Array_Get( world->bodies, shapeB->bodyId );
	const b3BodySim* simA = b3GetBodySim( world, bodyA );
	const b3BodySim* simB = b3GetBodySim( world, bodyB );
	const b3Manifold* manifold = contact->manifolds + manifoldIndex;
	const b3ManifoldPoint* mp = manifold->points + pointIndex;
	b3Pos midCenter = b3LerpPosition( simA->center, simB->center, 0.5f );
	b3Pos point = b3OffsetPos( midCenter, b3Lerp( mp->anchorA, mp->anchorB, 0.5f ) );

	memset( record, 0, sizeof( *record ) );
	record->contactId = contact->contactId;
	record->contactGeneration = contact->generation;
	record->event.point = point;
	record->event.normal = manifold->normal;
	record->event.normalImpulse = mp->totalNormalImpulse;
	record->event.approachSpeed = b3MaxFloat( 0.0f, -mp->normalVelocity );
	record->event.sideA.shapeId = (b3ShapeId){ shapeA->id + 1, world->worldId, shapeA->generation };
	record->event.sideB.shapeId = (b3ShapeId){ shapeB->id + 1, world->worldId, shapeB->generation };
	record->event.sideA.bodyId = b3MakeBodyId( world, shapeA->bodyId );
	record->event.sideB.bodyId = b3MakeBodyId( world, shapeB->bodyId );

	int hitboxA = B3_NULL_INDEX;
	int hitboxB = B3_NULL_INDEX;
	if ( contact->kind == v3_blockGridPairContactKind )
	{
		if ( v3BlockGridPairPointHitboxes( contact, manifoldIndex, pointIndex, &hitboxA, &hitboxB ) == false )
		{
			return false;
		}
	}
	else if ( shapeA->type == v3_blockGridShape )
	{
		hitboxA = contact->childIndex;
	}

	if ( shapeA->type == v3_blockGridShape &&
		 v3ResolveGridSide( &record->event.sideA, shapeA, simA->transform, point, hitboxA ) == false )
	{
		return false;
	}
	if ( shapeB->type == v3_blockGridShape &&
		 v3ResolveGridSide( &record->event.sideB, shapeB, simB->transform, point, hitboxB ) == false )
	{
		return false;
	}
	return record->event.sideA.isBlockGrid || record->event.sideB.isBlockGrid;
}

static bool v3RecordIsInPrevious( const b3Array( v3BlockContactRecord ) * previous, const v3BlockContactRecord* record )
{
	return v3FindRecord( previous, record ) != B3_NULL_INDEX;
}

static void v3KeepCurrentRecord( b3World* world, b3Array( v3BlockContactRecord ) * current,
								 const b3Array( v3BlockContactRecord ) * previous, v3BlockContactRecord record )
{
	if ( v3FindRecord( current, &record ) != B3_NULL_INDEX )
	{
		return;
	}
	if ( current->count < current->capacity )
	{
		current->data[current->count++] = record;
		return;
	}

	if ( v3RecordIsInPrevious( previous, &record ) )
	{
		for ( int i = 0; i < current->count; ++i )
		{
			if ( v3RecordIsInPrevious( previous, current->data + i ) == false )
			{
				current->data[i] = record;
				world->blockContactDroppedBeginCount = v3SaturatingAdd( world->blockContactDroppedBeginCount, 1 );
				return;
			}
		}
	}
	world->blockContactDroppedBeginCount = v3SaturatingAdd( world->blockContactDroppedBeginCount, 1 );
}

static void v3CollectCurrentRecords( b3World* world, b3Array( v3BlockContactRecord ) * current,
									 const b3Array( v3BlockContactRecord ) * previous )
{
	for ( int i = 0; i < world->contacts.count; ++i )
	{
		const b3Contact* contact = world->contacts.data + i;
		if ( contact->contactId == B3_NULL_INDEX || ( contact->flags & b3_contactTouchingFlag ) == 0 ||
			 ( contact->flags & b3_contactEnableContactEvents ) == 0 )
		{
			continue;
		}
		const b3Shape* shapeA = world->shapes.data + contact->shapeIdA;
		const b3Shape* shapeB = world->shapes.data + contact->shapeIdB;
		if ( shapeA->type != v3_blockGridShape && shapeB->type != v3_blockGridShape )
		{
			continue;
		}
		for ( int m = 0; m < contact->manifoldCount; ++m )
		{
			for ( int p = 0; p < contact->manifolds[m].pointCount; ++p )
			{
				v3BlockContactRecord record;
				if ( v3MakeRecord( world, contact, m, p, &record ) )
				{
					v3KeepCurrentRecord( world, current, previous, record );
				}
			}
		}
	}
}

void v3BlockContactEventsBeginStep( b3World* world )
{
	if ( world->blockContactBeginEvents.capacity == 0 )
	{
		return;
	}
	b3Array_Clear( world->blockContactBeginEvents );
	b3Array_Clear( world->blockContactHitEvents );
	world->blockContactDroppedBeginCount = 0;
	world->blockContactDroppedHitCount = 0;
	world->blockContactTransitionIncomplete = world->blockContactStateIncomplete;
}

void v3BlockContactEventsPushHit( b3World* world, const b3Contact* contact, int manifoldIndex, int pointIndex,
								  const b3ContactHitEvent* nativeEvent )
{
	if ( world->blockContactHitEvents.capacity == 0 )
	{
		return;
	}
	v3BlockContactRecord record;
	if ( v3MakeRecord( world, contact, manifoldIndex, pointIndex, &record ) )
	{
		v3BlockContactEvent event = record.event;
		event.point = nativeEvent->point;
		event.normal = nativeEvent->normal;
		event.approachSpeed = nativeEvent->approachSpeed;
		v3PushEvent( &world->blockContactHitEvents, event, &world->blockContactDroppedHitCount );
	}
}

void v3BlockContactEventsFinishStep( b3World* world )
{
	if ( world->blockContactBeginEvents.capacity == 0 )
	{
		return;
	}
	int previousIndex = world->blockContactStateIndex;
	int currentIndex = 1 - previousIndex;
	b3Array( v3BlockContactRecord )* previous = world->blockContactStates + previousIndex;
	b3Array( v3BlockContactRecord )* current = world->blockContactStates + currentIndex;
	b3Array_Clear( *current );

	bool previousIncomplete = world->blockContactStateIncomplete;
	if ( previousIncomplete || previous->count > 0 || world->contactBeginEvents.count > 0 || world->contactHitEvents.count > 0 )
	{
		v3CollectCurrentRecords( world, current, previous );
	}

	world->blockContactStateIncomplete = world->blockContactDroppedBeginCount != 0;
	// An absent previous record does not prove a new touch after bookkeeping overflow.
	// The recovery step stays truncated, even when this scan fits and restores tracking.
	for ( int i = 0; i < current->count; ++i )
	{
		if ( !previousIncomplete && v3FindRecord( previous, current->data + i ) == B3_NULL_INDEX )
		{
			v3PushEvent( &world->blockContactBeginEvents, current->data[i].event, &world->blockContactDroppedBeginCount );
		}
	}

	int endIndex = world->blockContactEndEventIndex;
	for ( int i = 0; i < previous->count; ++i )
	{
		if ( v3FindRecord( current, previous->data + i ) == B3_NULL_INDEX )
		{
			v3PushEvent( world->blockContactEndEvents + endIndex, previous->data[i].event,
						 &world->blockContactDroppedEndCount[endIndex] );
		}
	}
	world->blockContactStateIndex = currentIndex;
	world->blockContactEndEventIndex = 1 - endIndex;
	b3Array_Clear( world->blockContactEndEvents[world->blockContactEndEventIndex] );
	world->blockContactDroppedEndCount[world->blockContactEndEventIndex] = 0;
}

void v3BlockContactEventsFlushShape( b3World* world, const b3Shape* shape )
{
	if ( world->blockContactBeginEvents.capacity == 0 )
	{
		return;
	}
	b3ShapeId shapeId = { shape->id + 1, world->worldId, shape->generation };
	b3Array( v3BlockContactRecord )* states = world->blockContactStates + world->blockContactStateIndex;
	int endIndex = world->blockContactEndEventIndex;
	for ( int i = states->count - 1; i >= 0; --i )
	{
		v3BlockContactRecord* record = states->data + i;
		if ( v3SameShapeId( record->event.sideA.shapeId, shapeId ) || v3SameShapeId( record->event.sideB.shapeId, shapeId ) )
		{
			v3PushEvent( world->blockContactEndEvents + endIndex, record->event, &world->blockContactDroppedEndCount[endIndex] );
			states->data[i] = states->data[--states->count];
		}
	}
}

v3BlockContactEvents v3World_GetBlockContactEvents( b3WorldId worldId )
{
	if ( b3World_IsValid( worldId ) == false )
	{
		return (v3BlockContactEvents){ 0 };
	}
	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL || world->blockContactBeginEvents.capacity == 0 )
	{
		return (v3BlockContactEvents){ 0 };
	}
	int endIndex = 1 - world->blockContactEndEventIndex;
	uint32_t droppedEnd = world->blockContactDroppedEndCount[endIndex];
	return (v3BlockContactEvents){
		.beginEvents = world->blockContactBeginEvents.data,
		.hitEvents = world->blockContactHitEvents.data,
		.endEvents = world->blockContactEndEvents[endIndex].data,
		.beginCount = world->blockContactBeginEvents.count,
		.hitCount = world->blockContactHitEvents.count,
		.endCount = world->blockContactEndEvents[endIndex].count,
		.droppedBeginCount = world->blockContactDroppedBeginCount,
		.droppedHitCount = world->blockContactDroppedHitCount,
		.droppedEndCount = droppedEnd,
		.capacity = world->blockContactBeginEvents.capacity,
		.truncated = world->blockContactTransitionIncomplete || world->blockContactDroppedBeginCount != 0 ||
					 world->blockContactDroppedHitCount != 0 || droppedEnd != 0,
	};
}

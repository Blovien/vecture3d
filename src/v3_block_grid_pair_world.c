// SPDX-License-Identifier: MIT

#include "v3_block_grid_pair_world.h"

#include "aabb.h"
#include "body.h"
#include "contact.h"
#include "physics_world.h"
#include "shape.h"
#include "solver.h"
#include "solver_set.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

// Derive the current step again after pair creation: waking a sleeping body starts
// from the same zero velocity used here, so broad and narrow admission agree.
static float v3BlockGridPairMotionDistance( b3World* world, const b3Body* body, const b3BodySim* sim, float timeStep,
											int subStepCount )
{
	if ( body->type == b3_staticBody || timeStep <= 0.0f )
	{
		return 0.0f;
	}
	if ( !b3IsValidFloat( timeStep ) )
	{
		return -1.0f;
	}
	b3BodyState state = b3_identityBodyState;
	state.flags = body->flags;
	if ( body->setIndex == b3_awakeSet )
	{
		state = world->solverSets.data[b3_awakeSet].bodyStates.data[body->localIndex];
	}
	float distance = b3ComputeFreeMotionDistance( world, sim, &state, timeStep, subStepCount );
	double x = sim->maxExtent.x, y = sim->maxExtent.y, z = sim->maxExtent.z;
	double radius = sqrt( x * x + y * y + z * z );
	double extent = radius + distance + B3_SPECULATIVE_DISTANCE + B3_MAX_AABB_MARGIN;
	// Keep the float tree's surface-area arithmetic finite. Invalid predictions
	// use the existing all-or-separated pair outcome rather than partial contacts.
	return distance >= 0.0f && 24.0 * extent * extent < FLT_MAX ? distance : -1.0f;
}

void v3BlockGridPairPrepareStep( b3World* world, float timeStep, int subStepCount )
{
	if ( world->blockGridShapeCount < 2 )
	{
		return;
	}
	// No admission state survives the step. Replacing these bounds also shrinks a
	// previous larger timestep/limit and handles targets, teleports and recooked mass.
	for ( int shapeIndex = 0; shapeIndex < world->shapes.count; ++shapeIndex )
	{
		b3Shape* shape = world->shapes.data + shapeIndex;
		if ( shape->id != shapeIndex || shape->type != v3_blockGridShape || shape->proxyKey == B3_NULL_INDEX ||
			 shape->sensorIndex != B3_NULL_INDEX )
		{
			continue;
		}
		b3Body* body = world->bodies.data + shape->bodyId;
		if ( body->type == b3_staticBody )
		{
			continue;
		}
		b3BodySim* sim = b3GetBodySim( world, body );
		float distance = v3BlockGridPairMotionDistance( world, body, sim, timeStep, subStepCount );
		if ( distance < 0.0f )
		{
			continue;
		}
		float extra = nextafterf( distance + B3_SPECULATIVE_DISTANCE + shape->aabbMargin, INFINITY );
		b3AABB bounds = b3ComputeFatShapeAABB( shape, sim->transform, extra );
		if ( !b3IsValidAABB( bounds ) || !b3IsValidFloat( b3Perimeter( bounds ) ) )
		{
			continue;
		}
		if ( b3AABB_Contains( shape->fatAABB, bounds ) && b3AABB_Contains( bounds, shape->fatAABB ) )
		{
			continue;
		}
		shape->fatAABB = bounds;
		b3BroadPhase_MoveProxy( &world->broadPhase, shape->proxyKey, bounds );
	}
}

static bool v3BlockGridPairContactNeedsUpdate( const b3World* world, const b3Contact* contact )
{
	if ( contact->setIndex != b3_awakeSet || contact->kind != v3_blockGridPairContactKind )
	{
		return false;
	}

	const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
	return b3AABB_Overlaps( shapeA->fatAABB, shapeB->fatAABB );
}

static bool v3BlockGridPairBodyCanUpdate( const b3World* world, int bodyId )
{
	if ( bodyId < 0 || bodyId >= world->bodies.count )
	{
		return false;
	}

	const b3Body* body = b3Array_Get( world->bodies, bodyId );
	if ( body->id != bodyId || body->setIndex < 0 || body->setIndex >= world->solverSets.count )
	{
		return false;
	}

	const b3SolverSet* set = b3Array_Get( world->solverSets, body->setIndex );
	return body->localIndex >= 0 && body->localIndex < set->bodySims.count;
}

static bool v3BlockGridPairContactCanUpdate( const b3World* world, const b3Contact* contact, int contactId )
{
	if ( v3BlockGridPairContactIsValid( contact ) == false || contact->contactId != contactId || contact->shapeIdA < 0 ||
		 contact->shapeIdA >= world->shapes.count || contact->shapeIdB < 0 || contact->shapeIdB >= world->shapes.count )
	{
		return false;
	}

	const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
	return shapeA->id == contact->shapeIdA && shapeB->id == contact->shapeIdB && shapeA->type == v3_blockGridShape &&
		   shapeB->type == v3_blockGridShape && shapeA->blockGrid != NULL && shapeB->blockGrid != NULL &&
		   v3BlockGridPairBodyCanUpdate( world, shapeA->bodyId ) && v3BlockGridPairBodyCanUpdate( world, shapeB->bodyId );
}

static v3BlockGridPairUpdateInput v3BlockGridPairMakeWorldInput( b3World* world, b3Contact* contact, float timeStep,
																 int subStepCount )
{
	b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
	b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
	b3Body* bodyA = b3Array_Get( world->bodies, shapeA->bodyId );
	b3Body* bodyB = b3Array_Get( world->bodies, shapeB->bodyId );
	b3BodySim* bodySimA = b3GetBodySim( world, bodyA );
	b3BodySim* bodySimB = b3GetBodySim( world, bodyB );

	float motionA = v3BlockGridPairMotionDistance( world, bodyA, bodySimA, timeStep, subStepCount );
	float motionB = v3BlockGridPairMotionDistance( world, bodyB, bodySimB, timeStep, subStepCount );
	float distance = motionA < 0.0f || motionB < 0.0f
						 ? -1.0f
						 : nextafterf( (float)( (double)B3_SPECULATIVE_DISTANCE + motionA + motionB ), INFINITY );
	return (v3BlockGridPairUpdateInput){
		.contact = contact,
		.shapeA = shapeA,
		.shapeB = shapeB,
		.sourceEpochA = contact->blockGridPair.sourceEpochA,
		.sourceEpochB = contact->blockGridPair.sourceEpochB,
		.localCenterA = bodySimA->localCenter,
		.localCenterB = bodySimB->localCenter,
		.transformA = bodySimA->transform,
		.transformB = bodySimB->transform,
		.admissionDistance = distance,
	};
}

// Every counter is a plain per-step increment that clamps instead of wrapping
static uint64_t v3BlockGridPairCountUp( uint64_t counter, uint64_t amount )
{
	return amount > UINT64_MAX - counter ? UINT64_MAX : counter + amount;
}

static bool v3BlockGridPairScratchBytesFitInt( size_t count, size_t elementSize, int* byteCount )
{
	if ( elementSize == 0 || count > (size_t)INT_MAX / elementSize )
	{
		return false;
	}

	*byteCount = (int)( count * elementSize );
	return true;
}

void v3BlockGridPairUpdateAwakeContacts( b3World* world, b3Arena arena, float timeStep, int subStepCount )
{
	if ( world == NULL )
	{
		return;
	}

	v3BlockGridPairCounters counters = { 0 };

	// Replacements happen between steps, so they are parked on the world and moved
	// into this step's counters here. Every exit below publishes counters, so the
	// count is never dropped once it has been taken off the world.
	counters.replacementPublishedCount = world->blockGridReplacementPendingCount;
	world->blockGridReplacementPendingCount = 0;

	int queryWordCountA = 0;
	int queryWordCountB = 0;
	for ( int contactId = 0; contactId < world->contacts.count; ++contactId )
	{
		b3Contact* contact = b3Array_Get( world->contacts, contactId );
		if ( v3BlockGridPairContactNeedsUpdate( world, contact ) == false ||
			 v3BlockGridPairContactCanUpdate( world, contact, contactId ) == false )
		{
			continue;
		}
		const b3Shape* shapeA = b3Array_Get( world->shapes, contact->shapeIdA );
		const b3Shape* shapeB = b3Array_Get( world->shapes, contact->shapeIdB );
		counters.contactCount = v3BlockGridPairCountUp( counters.contactCount, 1 );
		queryWordCountA = b3MaxInt( queryWordCountA, v3BlockGrid_GetQueryScratchWordCount( shapeA->blockGrid ) );
		queryWordCountB = b3MaxInt( queryWordCountB, v3BlockGrid_GetQueryScratchWordCount( shapeB->blockGrid ) );
	}
	if ( counters.contactCount == 0 )
	{
		world->blockGridPairCounters = counters;
		return;
	}

	size_t updateScratchByteCount = v3BlockGridPairUpdateScratchByteCount();
	int queryBytesA;
	int queryBytesB;
	int updateScratchBytes;
	if ( arena.memory == NULL || arena.shared == NULL || arena.index < 0 || queryWordCountA <= 0 || queryWordCountB <= 0 ||
		 v3BlockGridPairScratchBytesFitInt( (size_t)queryWordCountA, sizeof( uint64_t ), &queryBytesA ) == false ||
		 v3BlockGridPairScratchBytesFitInt( (size_t)queryWordCountB, sizeof( uint64_t ), &queryBytesB ) == false ||
		 v3BlockGridPairScratchBytesFitInt( updateScratchByteCount, 1, &updateScratchBytes ) == false )
	{
		world->blockGridPairCounters = counters;
		return;
	}

	uint64_t* queryA = b3Bump( &arena, queryBytesA );
	uint64_t* queryB = b3Bump( &arena, queryBytesB );
	void* updateMemory = b3Bump( &arena, updateScratchBytes );
	if ( queryA == NULL || queryB == NULL || updateMemory == NULL )
	{
		world->blockGridPairCounters = counters;
		return;
	}
	counters.scratchPeakBytes = (uint64_t)queryBytesA + (uint64_t)queryBytesB + (uint64_t)updateScratchBytes;

	memset( queryA, 0, (size_t)queryBytesA );
	memset( queryB, 0, (size_t)queryBytesB );
	v3BlockGridPairUpdateScratch scratch = {
		.traversal =
			{
				.queryA = { queryA, queryWordCountA },
				.queryB = { queryB, queryWordCountB },
			},
		.memory = updateMemory,
		.byteCapacity = updateScratchByteCount,
	};

	for ( int contactId = 0; contactId < world->contacts.count; ++contactId )
	{
		b3Contact* contact = b3Array_Get( world->contacts, contactId );
		if ( v3BlockGridPairContactNeedsUpdate( world, contact ) == false ||
			 v3BlockGridPairContactCanUpdate( world, contact, contactId ) == false )
		{
			continue;
		}

		v3BlockGridPairUpdateInput input = v3BlockGridPairMakeWorldInput( world, contact, timeStep, subStepCount );
		v3BlockGridPairUpdateResult update = v3BlockGridPairUpdateContact( world, &input, &scratch );
		counters.candidateHitboxPairCount =
			v3BlockGridPairCountUp( counters.candidateHitboxPairCount, update.candidateHitboxPairCount );
		counters.touchingPairCount = v3BlockGridPairCountUp( counters.touchingPairCount, update.touchingPairCount );
		counters.contactReductionCount = v3BlockGridPairCountUp( counters.contactReductionCount, update.reductionCount );
	}

	world->blockGridPairCounters = counters;
}

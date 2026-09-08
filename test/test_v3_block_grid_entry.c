// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "contact.h"
#include "physics_world.h"
#include "test_macros.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

// These plates cover shell material exactly, including its empty interior.
typedef struct EntryBox
{
	b3Vec3 center, extent;
} EntryBox;
static const EntryBox entryShell[6] = {
	{ { -17.5f, 0, 0 }, { .5f, 6, 10 } }, { { 17.5f, 0, 0 }, { .5f, 6, 10 } }, { { 0, -5.5f, 0 }, { 17, .5f, 10 } },
	{ { 0, 5.5f, 0 }, { 17, .5f, 10 } },  { { 0, 0, -9.5f }, { 17, 5, .5f } }, { { 0, 0, 9.5f }, { 17, 5, .5f } },
};
static const EntryBox entryCube = { { 0, 0, 0 }, { 1, 1, 1 } };
static const EntryBox entryWall = { { .5f, 0, 0 }, { .5f, 16, 32 } };

// Independent OBB SAT over input material, never engine candidates or manifold separation.
static double EntryGap( EntryBox a, b3WorldTransform ta, EntryBox b, b3WorldTransform tb )
{
	b3Vec3 aa[3] = { b3RotateVector( ta.q, b3Vec3_axisX ), b3RotateVector( ta.q, b3Vec3_axisY ),
					 b3RotateVector( ta.q, b3Vec3_axisZ ) };
	b3Vec3 bb[3] = { b3RotateVector( tb.q, b3Vec3_axisX ), b3RotateVector( tb.q, b3Vec3_axisY ),
					 b3RotateVector( tb.q, b3Vec3_axisZ ) };
	b3Vec3 d = b3Add( b3SubPos( tb.p, ta.p ), b3Sub( b3RotateVector( tb.q, b.center ), b3RotateVector( ta.q, a.center ) ) );
	float ae[3] = { a.extent.x, a.extent.y, a.extent.z }, be[3] = { b.extent.x, b.extent.y, b.extent.z };
	double best = -DBL_MAX;
	for ( int k = 0; k < 15; ++k )
	{
		b3Vec3 n = k < 3 ? aa[k] : k < 6 ? bb[k - 3] : b3Cross( aa[( k - 6 ) / 3], bb[( k - 6 ) % 3] );
		float length = b3Length( n );
		if ( length < 1e-6f )
			continue;
		n = b3MulSV( 1.0f / length, n );
		double s = fabs( b3Dot( d, n ) );
		for ( int j = 0; j < 3; ++j )
			s -= ae[j] * fabs( b3Dot( aa[j], n ) ) + be[j] * fabs( b3Dot( bb[j], n ) );
		if ( s > best )
			best = s;
	}
	return best;
}

static v3BlockGridData* CookEntryGridAtOffset( int nx, int ny, int nz, bool shell, int offset )
{
	v3BlockGridBlock* blocks = calloc( (size_t)nx * ny * nz, sizeof( *blocks ) );
	if ( blocks == NULL )
		return NULL;
	v3BlockGridBox box = { .bounds = { { 0, 0, 0 }, { 1, 1, 1 } } };
	int count = 0;
	for ( int x = 0; x < nx; ++x )
		for ( int y = 0; y < ny; ++y )
			for ( int z = 0; z < nz; ++z )
			{
				if ( shell && x > 0 && x < nx - 1 && y > 0 && y < ny - 1 && z > 0 && z < nz - 1 )
					continue;
				blocks[count] = (v3BlockGridBlock){ .x = x - nx / 2 + offset,
													.y = y - ny / 2,
													.z = z - nz / 2,
													.userData = (uint64_t)count + 1,
													.boxes = &box,
													.boxCount = 1 };
				++count;
			}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0;
	material.restitution = 0;
	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = count };
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	free( blocks );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

static v3BlockGridData* CookEntryGrid( int nx, int ny, int nz, bool shell )
{
	return CookEntryGridAtOffset( nx, ny, nz, shell, 0 );
}

static b3BodyId CreateEntryBody( b3WorldId world, v3BlockGridData* grid, b3BodyType type, b3Pos position, b3Vec3 velocity,
								 b3Vec3 spin )
{
	b3BodyDef def = b3DefaultBodyDef();
	def.type = type;
	def.position = position;
	def.linearVelocity = velocity;
	def.angularVelocity = spin;
	b3BodyId body = b3CreateBody( world, &def );
	b3ShapeDef shape = b3DefaultShapeDef();
	shape.enableContactEvents = true;
	shape.enableHitEvents = true;
	if ( B3_IS_NULL( v3CreateBlockGridShape( body, &shape, grid ) ) )
	{
		b3DestroyBody( body );
		return b3_nullBodyId;
	}
	return body;
}

static uint64_t EntryHash( uint64_t hash, const void* value, size_t count )
{
	const unsigned char* bytes = value;
	for ( size_t i = 0; i < count; ++i )
	{
		hash ^= bytes[i];
		hash *= 1099511628211ULL;
	}
	return hash;
}
#define ENTRY_HASH( H, V ) ( ( H ) = EntryHash( ( H ), &( V ), sizeof( V ) ) )

static uint64_t EntryBodyHash( uint64_t hash, b3BodyId body )
{
	b3Pos p = b3Body_GetPosition( body );
	b3Quat q = b3Body_GetRotation( body );
	b3Vec3 v = b3Body_GetLinearVelocity( body ), w = b3Body_GetAngularVelocity( body );
	ENTRY_HASH( hash, p.x );
	ENTRY_HASH( hash, p.y );
	ENTRY_HASH( hash, p.z );
	ENTRY_HASH( hash, q.v.x );
	ENTRY_HASH( hash, q.v.y );
	ENTRY_HASH( hash, q.v.z );
	ENTRY_HASH( hash, q.s );
	ENTRY_HASH( hash, v.x );
	ENTRY_HASH( hash, v.y );
	ENTRY_HASH( hash, v.z );
	ENTRY_HASH( hash, w.x );
	ENTRY_HASH( hash, w.y );
	ENTRY_HASH( hash, w.z );
	bool awake = b3Body_IsAwake( body );
	ENTRY_HASH( hash, awake );
	return hash;
}

static uint64_t EntryEventHash( uint64_t hash, const v3BlockContactEvent* event, b3BodyId bodyA )
{
	const v3BlockContactSide* sides[2] = { &event->sideA, &event->sideB };
	for ( int i = 0; i < 2; ++i )
	{
		const v3BlockContactSide* s = sides[i];
		bool isA = B3_ID_EQUALS( s->bodyId, bodyA );
		ENTRY_HASH( hash, isA );
		ENTRY_HASH( hash, s->isBlockGrid );
		ENTRY_HASH( hash, s->cellX );
		ENTRY_HASH( hash, s->cellY );
		ENTRY_HASH( hash, s->cellZ );
		ENTRY_HASH( hash, s->subHitboxIndex );
		ENTRY_HASH( hash, s->materialIndex );
		ENTRY_HASH( hash, s->userMaterialId );
		ENTRY_HASH( hash, s->userData );
	}
	ENTRY_HASH( hash, event->point.x );
	ENTRY_HASH( hash, event->point.y );
	ENTRY_HASH( hash, event->point.z );
	ENTRY_HASH( hash, event->normal.x );
	ENTRY_HASH( hash, event->normal.y );
	ENTRY_HASH( hash, event->normal.z );
	ENTRY_HASH( hash, event->normalImpulse );
	ENTRY_HASH( hash, event->approachSpeed );
	return hash;
}

typedef struct EntryResult
{
	double minimumGap, movement, impulse;
	uint64_t stateHash, eventHash, workHash, candidates, maxCandidates;
	int touches, begins, hits, ends;
	bool crossed, eventsTruncated;
} EntryResult;

static int RunEntryScene( bool small, bool terrain, int approach, bool close, float speedCap, EntryResult* result )
{
	v3BlockGridData* hull = CookEntryGrid( small ? 2 : 36, small ? 2 : 12, small ? 2 : 20, !small );
	v3BlockGridData* wall = CookEntryGrid( 1, 32, 64, false );
	if ( hull == NULL || wall == NULL )
	{
		v3DestroyBlockGridData( hull );
		v3DestroyBlockGridData( wall );
		return 1;
	}
	b3WorldDef def = b3DefaultWorldDef();
	def.workerCount = 1;
	def.maximumLinearSpeed = speedCap;
	def.maximumAngularSpeed = .25f;
	def.gravity = b3Vec3_zero;
	def.enableSleep = false;
	b3WorldId world = b3CreateWorld( &def );
	double half = small ? 1 : 18, initialGap = close ? .01 : .25;
	b3Pos start = { -half - ( terrain ? initialGap : .5 * initialGap ), 0, 0 };
	b3BodyId a =
		CreateEntryBody( world, hull, b3_dynamicBody, start, (b3Vec3){ approach == 1 ? 8 : 10, 0, approach == 1 ? 6 : 0 },
						 (b3Vec3){ 0, approach == 2 ? .25f : 0, 0 } );
	b3BodyId b = CreateEntryBody( world, terrain ? wall : hull, terrain ? b3_staticBody : b3_dynamicBody,
								  (b3Pos){ terrain ? 0 : half + .5 * initialGap, 0, 0 }, (b3Vec3){ terrain ? 0 : -10, 0, 0 },
								  b3Vec3_zero );
	int failed = B3_IS_NULL( a ) || B3_IS_NULL( b );
	*result = (EntryResult){ .minimumGap = DBL_MAX,
							 .stateHash = 1469598103934665603ULL,
							 .eventHash = 1469598103934665603ULL,
							 .workHash = 1469598103934665603ULL };
	for ( int step = 0; step < 120 && !failed; ++step )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		b3WorldTransform ta = b3Body_GetTransform( a ), tb = b3Body_GetTransform( b );
		for ( int i = 0; i < ( small ? 1 : 6 ); ++i )
			for ( int j = 0; j < ( terrain ? 1 : 6 ); ++j )
			{
				double gap = EntryGap( small ? entryCube : entryShell[i], ta, terrain ? entryWall : entryShell[j], tb );
				if ( gap < result->minimumGap )
					result->minimumGap = gap;
				if ( !isfinite( gap ) )
					failed = 1;
			}
		double movement = b3Length( b3SubPos( ta.p, start ) );
		if ( movement > result->movement )
			result->movement = movement;
		// These finite approaches retain overlapping footprints. A leap entirely through cannot pass SAT alone.
		result->crossed |= ta.p.x > tb.p.x;
		b3ContactData contacts[8];
		int count = b3Body_GetContactData( a, contacts, 8 ), points = 0;
		if ( count == 8 )
			failed = 1;
		for ( int i = 0; i < count; ++i )
			for ( int j = 0; j < contacts[i].manifoldCount; ++j )
				for ( int k = 0; k < contacts[i].manifolds[j].pointCount; ++k )
				{
					const b3ManifoldPoint* p = contacts[i].manifolds[j].points + k;
					++points;
					result->impulse += p->totalNormalImpulse;
					ENTRY_HASH( result->stateHash, p->separation );
					ENTRY_HASH( result->stateHash, p->normalImpulse );
				}
		result->touches += points > 0;
		ENTRY_HASH( result->stateHash, points );
		result->stateHash = EntryBodyHash( EntryBodyHash( result->stateHash, a ), b );
		v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
		result->begins += events.beginCount;
		result->hits += events.hitCount;
		result->ends += events.endCount;
		result->eventsTruncated |= events.truncated;
		ENTRY_HASH( result->eventHash, events.beginCount );
		ENTRY_HASH( result->eventHash, events.hitCount );
		ENTRY_HASH( result->eventHash, events.endCount );
		for ( int i = 0; i < events.beginCount; ++i )
			result->eventHash = EntryEventHash( result->eventHash, events.beginEvents + i, a );
		for ( int i = 0; i < events.hitCount; ++i )
			result->eventHash = EntryEventHash( result->eventHash, events.hitEvents + i, a );
		for ( int i = 0; i < events.endCount; ++i )
			result->eventHash = EntryEventHash( result->eventHash, events.endEvents + i, a );
		v3BlockGridPairCounters c = v3World_GetBlockGridPairCounters( world );
		result->candidates += c.candidateHitboxPairCount;
		if ( c.candidateHitboxPairCount > result->maxCandidates )
			result->maxCandidates = c.candidateHitboxPairCount;
		ENTRY_HASH( result->workHash, c.candidateHitboxPairCount );
		ENTRY_HASH( result->workHash, c.touchingPairCount );
		ENTRY_HASH( result->workHash, c.contactCount );
		ENTRY_HASH( result->workHash, c.projectileSweepCount );
		ENTRY_HASH( result->workHash, c.capExhaustionCount );
		ENTRY_HASH( result->workHash, c.replacementPublishedCount );
		ENTRY_HASH( result->workHash, c.scratchPeakBytes );
		ENTRY_HASH( result->workHash, c.contactReductionCount );
	}
	b3DestroyWorld( world );
	v3DestroyBlockGridData( hull );
	v3DestroyBlockGridData( wall );
	// 1e-5m covers local float arithmetic at these coordinates; slop remains Box3D's .005m.
	return failed || result->minimumGap < -B3_LINEAR_SLOP - 1e-5 || result->movement <= 1e-4 || result->impulse <= 0 ||
		   result->touches == 0 || result->begins == 0 || result->crossed || result->eventsTruncated;
}

static int EntryMatrix( float speedCap )
{
	int failures = 0;
	for ( int row = 0; row < 14; ++row )
	{
		bool small = row < 2, terrain = small || ( row - 2 ) % 6 >= 3, close = small ? row == 1 : ( row - 2 ) / 6 == 1;
		int approach = small ? 0 : ( row - 2 ) % 3;
		EntryResult first;
		int failed = RunEntryScene( small, terrain, approach, close, speedCap, &first );
		for ( int repeat = 0; repeat < 2; ++repeat )
		{
			EntryResult next;
			failed |= RunEntryScene( small, terrain, approach, close, speedCap, &next );
			failed |= first.stateHash != next.stateHash || first.eventHash != next.eventHash || first.workHash != next.workHash;
		}
		printf( "entry cap=%.0f row=%d small=%d terrain=%d approach=%d close=%d pass=%d min=%.9f move=%.6f impulse=%.6f "
				"crossed=%d contacts=%d events=%d/%d/%d candidates=%llu max=%llu state=%016llx events=%016llx work=%016llx\n",
				speedCap, row, small, terrain, approach, close, !failed, first.minimumGap, first.movement, first.impulse,
				first.crossed, first.touches, first.begins, first.hits, first.ends, (unsigned long long)first.candidates,
				(unsigned long long)first.maxCandidates, (unsigned long long)first.stateHash, (unsigned long long)first.eventHash,
				(unsigned long long)first.workHash );
		failures += failed != 0;
	}
	return failures ? 1 : 0;
}

// Removing the carrier leaves a unit grid two metres above terrain. The force-driven
// full-step entry is sufficient to reproduce the transit failure without a carrier fixture.
static int FallingGridEntry( void )
{
	v3BlockGridData* cargo = CookEntryGrid( 1, 1, 1, false );
	v3BlockGridData* floor = CookEntryGrid( 32, 1, 32, false );
	if ( cargo == NULL || floor == NULL )
	{
		v3DestroyBlockGridData( cargo );
		v3DestroyBlockGridData( floor );
		return 1;
	}
	b3WorldDef def = b3DefaultWorldDef();
	def.workerCount = 1;
	def.gravity = (b3Vec3){ 0, -10, 0 };
	def.maximumLinearSpeed = 120;
	def.maximumAngularSpeed = .25f;
	def.enableSleep = false;
	b3WorldId world = b3CreateWorld( &def );
	b3BodyId a = CreateEntryBody( world, cargo, b3_dynamicBody, (b3Pos){ 0, 2, 0 }, b3Vec3_zero, b3Vec3_zero );
	b3BodyId b = CreateEntryBody( world, floor, b3_staticBody, (b3Pos){ 0, -1, 0 }, b3Vec3_zero, b3Vec3_zero );
	bool failed = B3_IS_NULL( a ) || B3_IS_NULL( b );
	double minimum = DBL_MAX, impulse = 0, descent = 0, minimumVelocity = 0;
	int minimumStep = -1;
	for ( int step = 0; step < 120 && !failed; ++step )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		b3WorldTransform transform = b3Body_GetTransform( a );
		double bottom = DBL_MAX;
		for ( int x = 0; x < 2; ++x )
			for ( int y = 0; y < 2; ++y )
				for ( int z = 0; z < 2; ++z )
				{
					b3Pos p = b3TransformWorldPoint( transform, (b3Vec3){ (float)x, (float)y, (float)z } );
					if ( p.y < bottom )
						bottom = p.y;
				}
		if ( bottom < minimum )
		{
			minimum = bottom;
			minimumStep = step;
			minimumVelocity = b3Body_GetLinearVelocity( a ).y;
		}
		if ( 2 - transform.p.y > descent )
			descent = 2 - transform.p.y;
		// Independent semi-implicit gravity trajectory before impact, including all four substeps.
		if ( step < 30 )
		{
			double n = 4 * ( step + 1 ), h = 1.0 / 240.0;
			double expected = 2 - 10 * h * h * n * ( n + 1 ) / 2;
			failed |= fabs( transform.p.y - expected ) > 1e-5;
		}
		b3ContactData contacts[8];
		int count = b3Body_GetContactData( a, contacts, 8 );
		for ( int i = 0; i < count; ++i )
			for ( int j = 0; j < contacts[i].manifoldCount; ++j )
				for ( int k = 0; k < contacts[i].manifolds[j].pointCount; ++k )
					impulse += contacts[i].manifolds[j].points[k].totalNormalImpulse;
		failed |= !isfinite( bottom ) || transform.p.y < -1;
	}
	printf( "fall min=%.9f step=%d vy=%.9f descent=%.9f impulse=%.6f\n", minimum, minimumStep, minimumVelocity, descent,
			impulse );
	b3DestroyWorld( world );
	v3DestroyBlockGridData( cargo );
	v3DestroyBlockGridData( floor );
	return failed || minimum < -B3_LINEAR_SLOP - 1e-5 || descent < 1.9 || impulse <= 0;
}

// A new step must use mutations made after the previous recorded step. Each input
// starts with zero velocity, so force-only admission cannot hide behind prior speed.
static int EntryUsesCurrentInputs( void )
{
	int failures = 0;
	for ( int input = 0; input < 3; ++input )
	{
		v3BlockGridData* cube = CookEntryGrid( 2, 2, 2, false );
		v3BlockGridData* wall = CookEntryGrid( 1, 32, 64, false );
		if ( cube == NULL || wall == NULL )
		{
			v3DestroyBlockGridData( cube );
			v3DestroyBlockGridData( wall );
			return 1;
		}
		b3WorldDef def = b3DefaultWorldDef();
		def.workerCount = 1;
		def.gravity = b3Vec3_zero;
		def.maximumLinearSpeed = 1;
		def.maximumAngularSpeed = .25f;
		b3WorldId world = b3CreateWorld( &def );
		b3BodyId a = CreateEntryBody( world, cube, b3_dynamicBody, (b3Pos){ -4, 0, 0 }, b3Vec3_zero, b3Vec3_zero );
		b3BodyId b = CreateEntryBody( world, wall, b3_staticBody, b3Pos_zero, b3Vec3_zero, b3Vec3_zero );
		bool failed = B3_IS_NULL( a ) || B3_IS_NULL( b );
		b3Recording* recording = b3CreateRecording( 0 );
		b3World_StartRecording( world, recording );
		b3World_Step( world, .001f, 1 );
		b3Body_SetAwake( a, false );
		b3Body_SetTransform( a, (b3Pos){ -1.25, 0, 0 }, b3Quat_identity );
		b3Body_SetAwake( a, true );
		b3World_SetMaximumLinearSpeed( world, 10 );
		float mass = b3Body_GetMass( a );
		if ( input == 0 )
		{
			b3Body_SetLinearDamping( a, 2 );
			b3Body_ApplyForceToCenter( a, (b3Vec3){ 200 * mass, 0, 0 }, true );
		}
		else if ( input == 1 )
		{
			b3Body_ApplyLinearImpulseToCenter( a, (b3Vec3){ 10 * mass, 0, 0 }, true );
		}
		else
		{
			b3Body_SetTargetTransform( a, (b3WorldTransform){ { -.75, 0, 0 }, b3Quat_identity }, .05f, true );
		}
		double minimum = DBL_MAX, impulse = 0, movement = 0;
		for ( int step = 0; step < 4; ++step )
		{
			b3World_Step( world, .05f, 4 );
			double advance = b3Body_GetPosition( a ).x + 1.25;
			if ( advance > movement )
				movement = advance;
			double gap = EntryGap( entryCube, b3Body_GetTransform( a ), entryWall, b3Body_GetTransform( b ) );
			if ( gap < minimum )
				minimum = gap;
			b3ContactData contacts[4];
			int count = b3Body_GetContactData( a, contacts, 4 );
			for ( int i = 0; i < count; ++i )
				for ( int j = 0; j < contacts[i].manifoldCount; ++j )
					for ( int k = 0; k < contacts[i].manifolds[j].pointCount; ++k )
						impulse += contacts[i].manifolds[j].points[k].totalNormalImpulse;
			failed |= !isfinite( gap ) || b3Body_GetPosition( a ).x > 0;
			failed |= b3Length( b3Body_GetLinearVelocity( a ) ) > 10 + 1e-5f;
		}
		failed |= minimum < -B3_LINEAR_SLOP - 1e-5 || movement <= 1e-4 || impulse <= 0;
		printf( "entry input=%d min=%.9f movement=%.9f impulse=%.6f pass=%d\n", input, minimum, movement, impulse, !failed );
		b3World_StopRecording( world );
		failed |= recording == NULL || b3Recording_GetSize( recording ) == 0;
		if ( recording != NULL )
		{
			failed |= !b3ValidateReplay( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
			failed |= !b3ValidateReplay( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 4 );
		}
		failures += failed;
		b3DestroyRecording( recording );
		b3DestroyWorld( world );
		v3DestroyBlockGridData( cube );
		v3DestroyBlockGridData( wall );
	}
	return failures != 0;
}

// Malformed point identity is injected into a real contact because valid public
// commands cannot create it. The public recording writer must refuse that state.
static bool EntryRejectsDuplicatePointRecording( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	for ( int i = 0; i < world->contacts.count; ++i )
	{
		b3Contact* contact = world->contacts.data + i;
		if ( contact->kind != v3_blockGridPairContactKind || contact->manifoldCount == 0 ||
			 contact->manifolds[0].pointCount != 4 )
			continue;
		b3Manifold* manifold = contact->manifolds;
		v3BlockGridPairPatch* patch = v3BlockGridPairPatches( contact->blockGridPair.state );
		uint32_t feature = manifold->points[3].featureId, key = patch->hitboxPairKeys[3];
		manifold->points[3].featureId = manifold->points[0].featureId;
		patch->hitboxPairKeys[3] = patch->hitboxPairKeys[0];
		b3Recording* recording = b3CreateRecording( 0 );
		b3World_StartRecording( worldId, recording );
		bool rejected = recording != NULL && b3Recording_GetSize( recording ) == 0;
		b3World_StopRecording( worldId );
		manifold->points[3].featureId = feature;
		patch->hitboxPairKeys[3] = key;
		b3DestroyRecording( recording );
		return rejected;
	}
	return false;
}

// The quarter-face patch has corners (1,1,2), (1,1,3), (1,2,2), (1,2,3).
// Against a static body, (1,2,3) lies on B's COM normal line and has no rotational
// response. With dynamic A made 1000 times lighter, A's response dominates and
// (1,1,2) comes first. The middle corners have the same order in both cases.
// The shifted cook changes A's local origin by 100m without changing its world
// geometry or COM. Neither this re-expression nor creating B first may reorder
// the physical corners. These exact coordinates also exercise score ties.
static int EntryPointOrderUsesBothCenters( void )
{
	int failures = 0;
	for ( int dynamic = 0; dynamic < 2; ++dynamic )
		for ( int swap = 0; swap < 2; ++swap )
			for ( int shifted = 0; shifted < 2; ++shifted )
			{
				int offset = shifted ? 100 : 0;
				v3BlockGridData* gridA = CookEntryGridAtOffset( 2, 4, 6, false, offset );
				v3BlockGridData* gridB = CookEntryGrid( 2, 2, 2, false );
				if ( gridA == NULL || gridB == NULL )
				{
					v3DestroyBlockGridData( gridA );
					v3DestroyBlockGridData( gridB );
					return 1;
				}
				b3WorldDef def = b3DefaultWorldDef();
				def.gravity = b3Vec3_zero;
				def.workerCount = 1;
				b3WorldId world = b3CreateWorld( &def );
				b3BodyId a = b3_nullBodyId, b = b3_nullBodyId;
				for ( int order = 0; order < 2; ++order )
				{
					if ( order == swap )
						a = CreateEntryBody( world, gridA, dynamic ? b3_dynamicBody : b3_staticBody, (b3Pos){ -offset, 0, 0 },
											 b3Vec3_zero, b3Vec3_zero );
					else
						b = CreateEntryBody( world, gridB, b3_dynamicBody, (b3Pos){ 2, 2, 3 }, b3Vec3_zero, b3Vec3_zero );
				}
				if ( dynamic )
				{
					b3MassData mass = b3Body_GetMassData( a );
					mass.mass *= .001f;
					mass.inertia.cx = b3MulSV( .001f, mass.inertia.cx );
					mass.inertia.cy = b3MulSV( .001f, mass.inertia.cy );
					mass.inertia.cz = b3MulSV( .001f, mass.inertia.cz );
					b3Body_SetMassData( a, mass );
				}
				b3World_Step( world, 0, 4 );
				b3ContactData contacts[4];
				int count = b3Body_GetContactData( b, contacts, 4 );
				bool failed = count != 1 || contacts[0].manifoldCount != 1 || contacts[0].manifolds[0].pointCount != 4;
				if ( !failed )
				{
					b3BodyId firstBody = b3Shape_GetBody( contacts[0].shapeIdA );
					failed |= !B3_ID_EQUALS( firstBody, swap ? b : a );
					b3Pos center = b3Body_GetWorldCenter( firstBody );
					const b3Manifold* manifold = contacts[0].manifolds;
					const b3Vec3 expected[4] = {
						{ 1, dynamic ? 1 : 2, dynamic ? 2 : 3 },
						{ 1, 1, 3 },
						{ 1, 2, 2 },
						{ 1, dynamic ? 2 : 1, dynamic ? 3 : 2 },
					};
					for ( int i = 0; i < 4; ++i )
					{
						b3Pos point = b3OffsetPos( center, manifold->points[i].anchorA );
						failed |= point.x != expected[i].x || point.y != expected[i].y || point.z != expected[i].z;
					}
				}
				b3Recording* recording = b3CreateRecording( 0 );
				b3World_StartRecording( world, recording );
				b3World_Step( world, 0, 4 );
				b3World_StopRecording( world );
				failed |= recording == NULL || b3Recording_GetSize( recording ) == 0;
				if ( recording != NULL )
				{
					failed |= !b3ValidateReplay( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
					failed |= !b3ValidateReplay( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 4 );
				}
				if ( !dynamic && !swap && !shifted )
					failed |= !EntryRejectsDuplicatePointRecording( world );
				printf( "entry order dynamic=%d swap=%d shifted=%d pass=%d\n", dynamic, swap, shifted, !failed );
				failures += failed;
				b3DestroyRecording( recording );
				b3DestroyWorld( world );
				v3DestroyBlockGridData( gridA );
				v3DestroyBlockGridData( gridB );
			}
	return failures != 0;
}

static v3BlockGridData* CookEntryRollingGrid( bool terrain )
{
	v3BlockGridBlock blocks[12];
	int count = 0;
	v3BlockGridBox box = { .bounds = { { 0, 0, 0 }, { 1, terrain ? 1 : .5f, 1 } } };
	for ( int x = -2; x < 1; ++x )
		for ( int y = terrain ? -2 : 0; y < ( terrain ? 0 : 1 ); ++y )
			for ( int z = -1; z < 1; ++z )
			{
				if ( terrain && x == 0 && y == -2 )
					continue;
				blocks[count] = (v3BlockGridBlock){ .x = x, .y = y, .z = z, .boxes = &box, .boxCount = 1 };
				++count;
			}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0;
	material.rollingResistance = .2f;
	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = count };
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

// The floor's coplanar 2x2x2 and 1x1x2 portions have rolling radii .25 and .125.
// Its original material representative is the first, larger portion. Moving the
// plate's COM reverses the solve order of the same four corners without changing
// that representative. With spin remaining, rolling impulse must saturate at
// .2 * .25 times normal impulse in both worlds.
static int EntryPointOrderPreservesRollingResistance( void )
{
	v3BlockGridData* floor = CookEntryRollingGrid( true );
	v3BlockGridData* plate = CookEntryRollingGrid( false );
	if ( floor == NULL || plate == NULL )
	{
		v3DestroyBlockGridData( floor );
		v3DestroyBlockGridData( plate );
		return 1;
	}
	int failures = 0;
	for ( int variant = 0; variant < 2; ++variant )
	{
		b3WorldDef def = b3DefaultWorldDef();
		def.gravity = (b3Vec3){ 0, -10, 0 };
		def.enableSleep = false;
		b3WorldId world = b3CreateWorld( &def );
		b3BodyId a = CreateEntryBody( world, floor, b3_staticBody, b3Pos_zero, b3Vec3_zero, b3Vec3_zero );
		b3BodyId b = CreateEntryBody( world, plate, b3_dynamicBody, b3Pos_zero, b3Vec3_zero, b3Vec3_zero );
		bool failed = B3_IS_NULL( a ) || B3_IS_NULL( b );
		b3MassData mass = { .mass = 10, .center = { variant ? 1 : -2, .25f, 0 },
							.inertia = { { 10, 0, 0 }, { 0, 10, 0 }, { 0, 0, 10 } } };
		b3Body_SetMassData( b, mass );
		b3Body_SetAngularVelocity( b, (b3Vec3){ 0, 10, 0 } );
		b3World_Step( world, 0, 1 );
		b3ContactData contacts[4];
		int count = b3Body_GetContactData( b, contacts, 4 );
		int support = 0;
		for ( int i = 0; i < count; ++i )
			for ( int j = 0; j < contacts[i].manifoldCount; ++j )
			{
				const b3Manifold* manifold = contacts[i].manifolds + j;
				if ( fabsf( manifold->normal.y ) < .99f )
					continue;
				++support;
				failed |= manifold->pointCount != 4;
				b3Pos center = b3Body_GetWorldCenter( b3Shape_GetBody( contacts[i].shapeIdA ) );
				for ( int k = 0; k < manifold->pointCount; ++k )
				{
					b3Pos point = b3OffsetPos( center, manifold->points[k].anchorA );
					float expectedX = ( k < 2 ) == ( variant == 0 ) ? -2 : 1;
					failed |= point.x != expectedX || point.y != 0 || point.z != ( k % 2 == 0 ? -1 : 1 );
				}
			}
		failed |= support != 1;
		b3World_Step( world, 1.0f / 600, 1 );
		count = b3Body_GetContactData( b, contacts, 4 );
		support = 0;
		for ( int i = 0; i < count; ++i )
			for ( int j = 0; j < contacts[i].manifoldCount; ++j )
			{
				const b3Manifold* manifold = contacts[i].manifolds + j;
				if ( fabsf( manifold->normal.y ) < .99f )
					continue;
				++support;
				double normalImpulse = 0;
				for ( int k = 0; k < manifold->pointCount; ++k )
					normalImpulse += manifold->points[k].normalImpulse;
				double rollingImpulse = b3Length( manifold->rollingImpulse );
				failed |= normalImpulse <= 1e-5 || fabs( rollingImpulse - .05 * normalImpulse ) > 1e-6 * normalImpulse;
				printf( "entry rolling variant=%d normal=%.9f rolling=%.9f pass=%d\n", variant, normalImpulse,
						rollingImpulse, !failed );
			}
		failed |= support != 1 || b3Body_GetAngularVelocity( b ).y <= 1;
		failures += failed;
		b3DestroyWorld( world );
	}
	v3DestroyBlockGridData( floor );
	v3DestroyBlockGridData( plate );
	return failures != 0;
}

int V3BlockGridEntryTest( void )
{
	int matrix = EntryMatrix( 10.0f );
	int cap120 = EntryMatrix( 120.0f );
	int falling = FallingGridEntry();
	int inputs = EntryUsesCurrentInputs();
	int order = EntryPointOrderUsesBothCenters();
	int rolling = EntryPointOrderPreservesRollingResistance();
	return matrix || cap120 || falling || inputs || order || rolling;
}

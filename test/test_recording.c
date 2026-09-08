// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "island.h"
#include "physics_world.h"
#include "recording.h"
#include "test_macros.h"
#include "block_grid/block_grid_contact.h"
#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#if defined( _WIN32 )
#include <malloc.h>
#endif
#include <stdio.h>
#include <string.h>

static const char* s_recPath = "recording_allops_test.b3rec";

// Sphere round-trip: record/step/stop, then replay and validate.
static int SphereRoundTrip( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );

	// Set a non-default gravity so the setter op appears in the stream.
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	// Static ground
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );

	b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Dynamic body with a sphere shape
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere;
	sphere.center = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	sphere.radius = 0.5f;
	b3ShapeDef sphereDef = b3DefaultShapeDef();
	sphereDef.density = 1.0f;
	b3CreateSphereShape( bodyId, &sphereDef, &sphere );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	b3DestroyRecording( rec );
	return 0;
}

static int SafetyFactorRoundTrip( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3Sphere sphere = { b3Vec3_zero, 0.5f };

	// This body is captured by the seed snapshot.
	b3BodyDef snapshotDef = b3DefaultBodyDef();
	snapshotDef.type = b3_dynamicBody;
	snapshotDef.safetyFactor = 0.125f;
	b3BodyId snapshotBodyId = b3CreateBody( worldId, &snapshotDef );
	b3CreateSphereShape( snapshotBodyId, &shapeDef, &sphere );

	b3World_StartRecording( worldId, rec );

	// This body exercises the widened body-definition payload.
	b3BodyDef streamDef = b3DefaultBodyDef();
	streamDef.type = b3_dynamicBody;
	streamDef.position = (b3Pos){ 3.0f, 0.0f, 0.0f };
	streamDef.safetyFactor = 0.1f;
	b3BodyId streamBodyId = b3CreateBody( worldId, &streamDef );
	b3CreateSphereShape( streamBodyId, &shapeDef, &sphere );
	b3Body_SetSafetyFactor( snapshotBodyId, 0.25f );
	b3World_Step( worldId, 1.0f / 60.0f, 1 );

	// A second frame separates this setter opcode from body creation.
	b3Body_SetSafetyFactor( streamBodyId, 0.4f );
	b3World_Step( worldId, 1.0f / 60.0f, 1 );

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const void* data = b3Recording_GetData( rec );
	int size = b3Recording_GetSize( rec );
	ENSURE( size >= (int)sizeof( b3RecHeader ) + 2 * (int)sizeof( uint32_t ) );
	const b3RecHeader* header = data;
	ENSURE( header->versionMajor == 16 );
	ENSURE( header->snapshotSize >= 2 * sizeof( uint32_t ) );
	uint32_t snapshotVersion = 0;
	memcpy( &snapshotVersion, (const uint8_t*)data + sizeof( b3RecHeader ) + sizeof( uint32_t ), sizeof( snapshotVersion ) );
	ENSURE( snapshotVersion == 11 );
	ENSURE( b3ValidateReplay( data, size, 1 ) );

	b3RecPlayer* player = b3CreatePlayer( data, size, 1 );
	ENSURE( player != NULL );
	ENSURE( b3RecPlayer_GetBodyCount( player ) == 1 );
	b3BodyId replaySnapshotBody = b3RecPlayer_GetBodyId( player, 0 );
	ENSURE( b3Body_GetSafetyFactor( replaySnapshotBody ) == 0.125f );

	ENSURE( b3RecPlayer_StepFrame( player ) );
	ENSURE( b3RecPlayer_GetBodyCount( player ) == 2 );
	replaySnapshotBody = b3RecPlayer_GetBodyId( player, 0 );
	b3BodyId replayStreamBody = b3RecPlayer_GetBodyId( player, 1 );
	ENSURE( b3Body_GetSafetyFactor( replaySnapshotBody ) == 0.25f );
	ENSURE( b3Body_GetSafetyFactor( replayStreamBody ) == 0.1f );

	ENSURE( b3RecPlayer_StepFrame( player ) );
	replayStreamBody = b3RecPlayer_GetBodyId( player, 1 );
	ENSURE( b3Body_GetSafetyFactor( replayStreamBody ) == 0.4f );
	ENSURE( b3RecPlayer_HasDiverged( player ) == false );

	b3DestroyPlayer( player );

	// Copy a real recording and change one version at a time. Old shape tags
	// must be rejected even when the rest of the recording is valid.
	uint8_t* oldVersion = b3Alloc( (size_t)size );
	memcpy( oldVersion, data, (size_t)size );
	b3RecHeader oldHeader;
	memcpy( &oldHeader, oldVersion, sizeof( oldHeader ) );
	oldHeader.versionMajor = 14;
	memcpy( oldVersion, &oldHeader, sizeof( oldHeader ) );
	ENSURE( b3CreatePlayer( oldVersion, size, 1 ) == NULL );
	ENSURE( b3ValidateReplay( oldVersion, size, 1 ) == false );

	// The snapshot flags follow its magic, version and layout hash. Clear the double
	// precision flag while preserving the layout hash to exercise the format check.
	memcpy( oldVersion, data, (size_t)size );
	size_t flagsOffset = sizeof( b3RecHeader ) + 3 * sizeof( uint32_t );
	ENSURE( header->snapshotSize >= 4 * sizeof( uint32_t ) );
	uint32_t snapshotFlags;
	memcpy( &snapshotFlags, oldVersion + flagsOffset, sizeof( snapshotFlags ) );
	ENSURE( ( snapshotFlags & 0x2u ) != 0 );
	snapshotFlags &= ~0x2u;
	memcpy( oldVersion + flagsOffset, &snapshotFlags, sizeof( snapshotFlags ) );
	ENSURE( b3CreatePlayer( oldVersion, size, 1 ) == NULL );
	ENSURE( b3ValidateReplay( oldVersion, size, 1 ) == false );

	memcpy( oldVersion, data, (size_t)size );
	uint32_t oldSnapshotVersion = 9;
	memcpy( oldVersion + sizeof( b3RecHeader ) + sizeof( uint32_t ), &oldSnapshotVersion, sizeof( oldSnapshotVersion ) );
	ENSURE( b3CreatePlayer( oldVersion, size, 1 ) == NULL );
	ENSURE( b3ValidateReplay( oldVersion, size, 1 ) == false );

	memcpy( oldVersion, data, (size_t)size );
	ENSURE( b3ValidateReplay( oldVersion, size, 1 ) );
	b3Free( oldVersion, (size_t)size );
	b3DestroyRecording( rec );
	return 0;
}

// Hull dedup: three bodies sharing the same hull should produce one registry entry.
static int HullDedup( void )
{
	// Build a small convex hull
	b3Vec3 pts[8] = {
		{ -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f },
		{ -1.0f, -1.0f, 1.0f },	 { 1.0f, -1.0f, 1.0f },	 { 1.0f, 1.0f, 1.0f },	{ -1.0f, 1.0f, 1.0f },
	};
	b3HullData* hull = b3CreateHull( pts, 8, 8 );
	ENSURE( hull != NULL );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;

	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)( i * 3 ), 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &shapeDef, hull );
	}

	float timeStep = 1.0f / 60.0f;
	for ( int i = 0; i < 5; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );
	b3DestroyHull( hull );

	// Validate the replay
	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	// Confirm the registry was deduped to 1 hull entry.
	// Parse registryOffset from the header and count entries.
	const uint8_t* bytes = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );
	ENSURE( sz >= 48 );

	uint64_t regOff = 0;
	memcpy( &regOff, bytes + 32, 8 ); // registryOffset at offset 32 in b3RecHeader
	ENSURE( regOff != 0 && (int)regOff + 4 <= sz );

	// entryCount is a little-endian u32 at the start of the registry block
	const uint8_t* rp = bytes + (int)regOff;
	uint32_t entryCount = (uint32_t)rp[0] | ( (uint32_t)rp[1] << 8 ) | ( (uint32_t)rp[2] << 16 ) | ( (uint32_t)rp[3] << 24 );
	ENSURE( entryCount == 1 );

	b3DestroyRecording( rec );
	return 0;
}

// Mid-stream snapshot with only dynamic bodies floating in air (no contacts).
static int MidStreamNoContacts( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3Sphere sphere;
	sphere.center = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	sphere.radius = 0.5f;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;

	// A few dynamic bodies well apart from each other so no contacts form
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)( i * 10 ), 50.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// Start recording mid-stream, with a snapshot of the current world
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	b3World_StartRecording( worldId, rec );

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	b3DestroyRecording( rec );
	return 0;
}

// Mid-stream snapshot with contacts: bodies touching ground with warm-start manifolds.
static int MidStreamContacts( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	// Static ground using a box hull
	{
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );

		b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
		b3ShapeDef groundShape = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &groundShape, &groundBox.base );
	}

	b3ShapeDef dynamicShape = b3DefaultShapeDef();
	dynamicShape.density = 1.0f;

	// A few dynamic boxes dropped onto the ground
	for ( int i = 0; i < 3; ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)( i * 2 ) - 2.0f, 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &dynamicShape, &box.base );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	// Let the scene settle: bodies hit ground, build manifolds, islands, graph colors
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// Start recording with snapshot of the settled world (contacts, islands, warm starts)
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	b3World_StartRecording( worldId, rec );

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	b3DestroyRecording( rec );
	return 0;
}

// Record a scene with hull boxes settling on a ground plane, create a player, step to
// the end recording per-frame world hashes, then seek backward to several frames and
// verify each reproduces the recorded hash exactly.
static int ScrubBackward( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3World_StartRecording( worldId, rec );

	// Static ground
	{
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );
		b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3ShapeDef groundShape = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &groundShape, &groundBox.base );
	}

	// A small stack of dynamic hull boxes
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < 4; ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 2.0f + (float)i * 1.5f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	int totalFrames = 80;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );

	// Create the player
	b3RecPlayer* player = b3CreatePlayer( data, sz, 1 );
	ENSURE( player != NULL );
	ENSURE( b3RecPlayer_GetFrameCount( player ) == totalFrames );

	// Forward pass: record per-frame hashes
	uint64_t* hashes = (uint64_t*)b3Alloc( (size_t)( totalFrames + 1 ) * sizeof( uint64_t ) );
	hashes[0] = 0; // frame 0 before any step

	while ( !b3RecPlayer_IsAtEnd( player ) )
	{
		b3RecPlayer_StepFrame( player );
		int f = b3RecPlayer_GetFrame( player );
		if ( f <= totalFrames )
		{
			b3WorldId wid = b3RecPlayer_GetWorldId( player );
			b3World* w = b3GetWorldFromId( wid );
			hashes[f] = b3HashWorldState( w );
		}
	}
	ENSURE( b3RecPlayer_GetFrame( player ) == totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	// Backward seek to several interesting frames and verify hash matches
	int seekTargets[] = { totalFrames, totalFrames / 2, 5, totalFrames - 1, 0, 1 };
	int seekCount = (int)( sizeof( seekTargets ) / sizeof( seekTargets[0] ) );
	for ( int k = 0; k < seekCount; ++k )
	{
		int target = seekTargets[k];
		b3RecPlayer_SeekFrame( player, target );
		ENSURE( b3RecPlayer_GetFrame( player ) == target );
		ENSURE( !b3RecPlayer_HasDiverged( player ) );

		if ( target > 0 )
		{
			b3WorldId wid = b3RecPlayer_GetWorldId( player );
			b3World* w = b3GetWorldFromId( wid );
			uint64_t got = b3HashWorldState( w );
			ENSURE( got == hashes[target] );
		}
	}

	b3Free( hashes, (size_t)( totalFrames + 1 ) * sizeof( uint64_t ) );
	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	return 0;
}

// Record a scene that includes a mesh shape, create a player, seek backward, verify
// it works and no divergence is reported. Also checks the keyframe-by-geometry-id
// invariant: keyframeRec registry count should not grow beyond initial slot count.
static int SeekWithHull( void )
{
	b3Vec3 pts[8] = {
		{ -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f },
		{ -1.0f, -1.0f, 1.0f },	 { 1.0f, -1.0f, 1.0f },	 { 1.0f, 1.0f, 1.0f },	{ -1.0f, 1.0f, 1.0f },
	};
	b3HullData* hull = b3CreateHull( pts, 8, 8 );
	ENSURE( hull != NULL );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );
	b3World_StartRecording( worldId, rec );

	// Static ground
	{
		b3BodyDef groundDef = b3DefaultBodyDef();
		groundDef.type = b3_staticBody;
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );
		b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3ShapeDef gs = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &gs, &groundBox.base );
	}

	// Dynamic bodies using the custom hull
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 1.0f;
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_dynamicBody;
		bd.position = (b3Pos){ (float)( i * 4 ) - 4.0f, 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bd );
		b3CreateHullShape( bodyId, &sd, hull );
	}

	float timeStep = 1.0f / 60.0f;
	int totalFrames = 40;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );
	b3DestroyHull( hull );

	const uint8_t* data = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );

	b3RecPlayer* player = b3CreatePlayer( data, sz, 1 );
	ENSURE( player != NULL );

	// Step to end
	while ( !b3RecPlayer_IsAtEnd( player ) )
	{
		b3RecPlayer_StepFrame( player );
	}
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	int midFrame = totalFrames / 2;
	b3RecPlayer_SeekFrame( player, midFrame );
	ENSURE( b3RecPlayer_GetFrame( player ) == midFrame );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	b3RecPlayer_SeekFrame( player, 0 );
	ENSURE( b3RecPlayer_GetFrame( player ) == 0 );

	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	return 0;
}

// Host debug-shape callbacks just count create/destroy so the test can prove the player
// wires them into every world it builds. The returned token is opaque to the engine.
typedef struct
{
	int created;
	int destroyed;
} DebugShapeCounters;

static void* RecTestCreateDebugShape( const b3DebugShape* debugShape, void* userContext )
{
	(void)debugShape;
	DebugShapeCounters* counters = (DebugShapeCounters*)userContext;
	counters->created += 1;
	return userContext; // any non-NULL token; engine stores and hands it back to destroy
}

static void RecTestDestroyDebugShape( void* userShape, void* userContext )
{
	(void)userShape;
	DebugShapeCounters* counters = (DebugShapeCounters*)userContext;
	counters->destroyed += 1;
}

static void RecTestDrawShape( void* userShape, b3WorldTransform transform, b3HexColor color, void* context )
{
	(void)userShape;
	(void)transform;
	(void)color;
	(void)context;
}

// b3World_Draw lazily fires createDebugShape for shapes entering the draw set, the same way the
// sample renderer does. Drive a draw so the player's wired callbacks actually run.
static void RecTestDrawWorld( b3WorldId worldId )
{
	b3DebugDraw draw = b3DefaultDebugDraw();
	draw.DrawShapeFcn = RecTestDrawShape;
	draw.drawShapes = true;
	float big = 1.0e6f;
	draw.drawingBounds = (b3AABB){ { -big, -big, -big }, { big, big, big } };
	b3World_Draw( worldId, &draw, B3_DEFAULT_MASK_BITS );
}

// The 3D sample renderer builds per-shape GPU meshes through createDebugShape, so the replay
// world must carry the host callbacks. Verify b3RecPlayer_SetDebugShapeCallbacks rewinds, fires
// the callbacks for every replayed shape, keeps them across a backward-seek world rebuild, and
// balances create/destroy at teardown.
static int DebugShapeCallbacks( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	// Four dynamic boxes sharing one hull: ground + 4 boxes = 5 shapes total.
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 1.0f + 1.1f * (float)i, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}

	int totalFrames = 30;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	// Round-trip through a file, mirroring the replay sample's Generate/Load path exactly.
	const char* path = "replay_test.b3rec";
	ENSURE( b3SaveRecordingToFile( rec, path ) );
	b3Recording* loaded = b3LoadRecordingFromFile( path );
	ENSURE( loaded != NULL );

	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( loaded ), b3Recording_GetSize( loaded ), 1 );
	ENSURE( player != NULL );

	// Wiring the callbacks rebuilds the world and rewinds to frame 0.
	DebugShapeCounters counters = { 0, 0 };
	b3RecPlayer_SetDebugShapeCallbacks( player, RecTestCreateDebugShape, RecTestDestroyDebugShape, &counters );
	ENSURE( b3RecPlayer_GetFrame( player ) == 0 );

	// Replay to the end, then draw: createDebugShape fires once per shape (ground + 4 boxes = 5).
	while ( !b3RecPlayer_IsAtEnd( player ) )
	{
		b3RecPlayer_StepFrame( player );
	}
	RecTestDrawWorld( b3RecPlayer_GetWorldId( player ) );
	ENSURE( counters.created >= 5 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	// A backward seek restores the empty seed in place, releasing the live debug shapes, then forward
	// stepping recreates them through the callbacks, so more creates fire.
	int createdBefore = counters.created;
	b3RecPlayer_SeekFrame( player, 0 );
	b3RecPlayer_SeekFrame( player, totalFrames );
	RecTestDrawWorld( b3RecPlayer_GetWorldId( player ) );
	ENSURE( counters.created > createdBefore );

	// Teardown destroys the final world; every live shape is released, so the counts balance.
	b3DestroyPlayer( player );
	ENSURE( counters.created == counters.destroyed );

	b3DestroyRecording( loaded );
	b3DestroyRecording( rec );
	remove( path );
	return 0;
}

// Exercise the viewer-facing player accessors: recording info, creation-ordinal body tracking
// (seeded from a snapshot), divergence frame, and keyframe policy. Recording starts after the
// bodies exist, so the snapshot seeds the outliner list and ordinals are stable from frame 0.
static int PlayerAccessors( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	// Static ground (creation ordinal 0)
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	// Four dynamic boxes (ordinals 1..4)
	const int dynamicCount = 4;
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < dynamicCount; ++i )
	{
		b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 2.0f + (float)i * 1.5f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	// Settle, then record with a snapshot of the populated world.
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	b3World_StartRecording( worldId, rec );

	int totalFrames = 80;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}
	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );

	// Info reflects the recorded tuning and a non-degenerate bounds.
	b3RecPlayerInfo info = b3RecPlayer_GetInfo( player );
	ENSURE( info.frameCount == totalFrames );
	ENSURE( info.subStepCount == subStepCount );
	ENSURE( info.timeStep > 0.0f );
	b3Vec3 extent = b3Sub( info.bounds.upperBound, info.bounds.lowerBound );
	ENSURE( extent.x > 0.0f && extent.y > 0.0f && extent.z > 0.0f );

	// Body ordinals: ground + 4 dynamic, seeded from the snapshot and present at frame 0.
	ENSURE( b3RecPlayer_GetBodyCount( player ) == 1 + dynamicCount );
	b3BodyId ground = b3RecPlayer_GetBodyId( player, 0 );
	ENSURE( b3Body_IsValid( ground ) );
	ENSURE( b3Body_GetType( ground ) == b3_staticBody );
	for ( int i = 1; i <= dynamicCount; ++i )
	{
		b3BodyId id = b3RecPlayer_GetBodyId( player, i );
		ENSURE( b3Body_IsValid( id ) );
		ENSURE( b3Body_GetType( id ) == b3_dynamicBody );
	}
	ENSURE( B3_IS_NULL( b3RecPlayer_GetBodyId( player, 1 + dynamicCount ) ) );

	// No divergence on a clean serial replay.
	b3RecPlayer_SeekFrame( player, totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( b3RecPlayer_GetDivergeFrame( player ) == -1 );

	// Ordinals survive a backward seek that restores from a keyframe.
	b3BodyId before = b3RecPlayer_GetBodyId( player, 2 );
	b3RecPlayer_SeekFrame( player, totalFrames / 2 );
	b3RecPlayer_SeekFrame( player, totalFrames );
	b3BodyId after = b3RecPlayer_GetBodyId( player, 2 );
	ENSURE( B3_ID_EQUALS( before, after ) );

	// Keyframe policy: defaults present, setter takes effect and clears the ring.
	ENSURE( b3RecPlayer_GetKeyframeMinInterval( player ) == 16 );
	b3RecPlayer_SetKeyframePolicy( player, (size_t)256 * 1024 * 1024, 8 );
	ENSURE( b3RecPlayer_GetKeyframeMinInterval( player ) == 8 );
	ENSURE( b3RecPlayer_GetKeyframeInterval( player ) == 8 );
	ENSURE( b3RecPlayer_GetKeyframeBudget( player ) == (size_t)256 * 1024 * 1024 );
	ENSURE( b3RecPlayer_GetKeyframeBytes( player ) == 0 );

	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	return 0;
}

// A keyframe restore is a deterministic replay state, so shapes that persist must keep their renderer
// handle rather than being torn down and rebuilt every seek. Record a snapshot-seeded session (shapes
// exist at frame 0), replay with handle callbacks, scrub backward across keyframes repeatedly, and
// verify no new handles are built per restore and the create/destroy counts still balance at teardown.
static int KeyframeHandleReuse( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	const int dynamicCount = 5;
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef boxShape = b3DefaultShapeDef();
	boxShape.density = 1.0f;
	for ( int i = 0; i < dynamicCount; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 1.0f + 1.1f * (float)i, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3CreateHullShape( bodyId, &boxShape, &box.base );
	}
	int shapeCount = 1 + dynamicCount;

	// Settle, then record with a snapshot of the populated world so shapes exist at frame 0.
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	b3World_StartRecording( worldId, rec );

	int totalFrames = 80;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );

	DebugShapeCounters counters = { 0, 0 };
	b3RecPlayer_SetDebugShapeCallbacks( player, RecTestCreateDebugShape, RecTestDestroyDebugShape, &counters );

	// Replay to the end and draw: one handle per shape.
	b3RecPlayer_SeekFrame( player, totalFrames );
	RecTestDrawWorld( b3RecPlayer_GetWorldId( player ) );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( counters.created == shapeCount );

	// Scrub backward and forward across keyframes. Each restore keeps the persistent shapes' handles,
	// so drawing builds no new ones.
	int createdAfterFirstDraw = counters.created;
	int seekTargets[] = { 40, 70, 20, 60, 8, 75 };
	for ( int k = 0; k < (int)( sizeof( seekTargets ) / sizeof( seekTargets[0] ) ); ++k )
	{
		b3RecPlayer_SeekFrame( player, seekTargets[k] );
		RecTestDrawWorld( b3RecPlayer_GetWorldId( player ) );
	}
	ENSURE( counters.created == createdAfterFirstDraw );

	// Teardown releases exactly the live handles, so the leak-free invariant holds.
	b3DestroyPlayer( player );
	ENSURE( counters.created == counters.destroyed );

	b3DestroyRecording( rec );
	return 0;
}

static bool QueryReplayOverlapFcn( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	(void)context;
	return true;
}

static float QueryReplayCastFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
								 int triangleIndex, int childIndex, void* context )
{
	(void)shapeId;
	(void)point;
	(void)normal;
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;
	(void)context;
	// Return the fraction to keep the closest hit, exercising the recorded user-return path.
	return fraction;
}

static bool QueryReplayPlaneFcn( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	(void)shapeId;
	(void)planes;
	(void)planeCount;
	(void)context;
	return true;
}

static bool QueryReplayMoverFilterFcn( b3ShapeId shapeId, void* context )
{
	(void)shapeId;
	(void)context;
	return true;
}

// Issue all seven world queries each frame, then replay. Every query is re-issued against the replay
// world and compared to what was recorded, so a clean (non-diverged) replay proves the queries
// reproduce. Also opens a player and confirms the per-frame query store surfaces all seven.
static int QueryReplay( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	// A few dynamic spheres for the queries to find.
	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)i - 1.5f, 3.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3ShapeDef sphereDef = b3DefaultShapeDef();
		sphereDef.density = 1.0f;
		b3CreateSphereShape( bodyId, &sphereDef, &sphere );
	}

	b3World_StartRecording( worldId, rec );

	b3QueryFilter filter = b3DefaultQueryFilter();

	const int totalFrames = 30;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3Pos origin = { 0.0f, 6.0f, 0.0f };
		b3Vec3 translation = { 0.0f, -8.0f, 0.0f };
		b3AABB aabb = { { -5.0f, -1.0f, -5.0f }, { 5.0f, 6.0f, 5.0f } };

		b3Vec3 proxyPts = { 0.0f, 0.0f, 0.0f };
		b3ShapeProxy proxy = { &proxyPts, 1, 0.5f };
		b3Capsule mover = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 0.3f };

		b3World_OverlapAABB( worldId, aabb, filter, QueryReplayOverlapFcn, NULL );
		b3World_OverlapShape( worldId, origin, &proxy, filter, QueryReplayOverlapFcn, NULL );
		b3World_CastRay( worldId, origin, translation, filter, QueryReplayCastFcn, NULL );
		b3World_CastRayClosest( worldId, origin, translation, filter );
		b3World_CastShape( worldId, origin, &proxy, translation, filter, QueryReplayCastFcn, NULL );
		b3World_CastMover( worldId, origin, &mover, translation, filter, QueryReplayMoverFilterFcn, NULL );
		b3World_CollideMover( worldId, origin, &mover, filter, QueryReplayPlaneFcn, NULL );

		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	// Headless validation: re-issues every recorded query and compares the results.
	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	// Player path: seek to a mid frame and confirm the per-frame store holds all seven queries.
	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );

	b3RecPlayer_SeekFrame( player, 15 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( b3RecPlayer_GetFrameQueryCount( player ) == 7 );

	b3RecQueryInfo first = b3RecPlayer_GetFrameQuery( player, 0 );
	ENSURE( first.type == b3_recQueryOverlapAABB );

	// The ray cast should find at least the ground, so its recorded hit list is non-empty.
	bool sawCastRay = false;
	for ( int qi = 0; qi < b3RecPlayer_GetFrameQueryCount( player ); ++qi )
	{
		b3RecQueryInfo info = b3RecPlayer_GetFrameQuery( player, qi );
		if ( info.type == b3_recQueryCastRay )
		{
			sawCastRay = true;
			ENSURE( info.hitCount > 0 );
		}
	}
	ENSURE( sawCastRay );

	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	return 0;
}

// Tagged queries: the caller (id, label) is hashed into a key that rides the QueryTag op, with the id
// and label interned in the trailing tag table. The same label under two entity ids is two distinct
// keys. All survive a file round-trip and resolve back through b3RecQueryInfo; untagged queries report
// key 0 / id 0 / name NULL.
static int TaggedQuery( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
	b3ShapeDef groundShape = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShape, &groundBox.base );

	b3World_StartRecording( worldId, rec );

	// Same label, different entity ids: distinct logical queries with distinct keys.
	b3QueryFilter bullet53 = b3DefaultQueryFilter();
	bullet53.id = 53;
	bullet53.name = "bullet";

	b3QueryFilter bullet54 = b3DefaultQueryFilter();
	bullet54.id = 54;
	bullet54.name = "bullet";

	b3QueryFilter untagged = b3DefaultQueryFilter();

	uint64_t key53 = b3HashQueryTag( 53, "bullet" );
	uint64_t key54 = b3HashQueryTag( 54, "bullet" );
	ENSURE( key53 != 0 && key54 != 0 && key53 != key54 );

	const int totalFrames = 10;
	for ( int i = 0; i < totalFrames; ++i )
	{
		b3Pos origin = { 0.0f, 6.0f, 0.0f };
		b3Vec3 translation = { 0.0f, -8.0f, 0.0f };
		b3AABB aabb = { { -5.0f, -1.0f, -5.0f }, { 5.0f, 6.0f, 5.0f } };

		// Two tagged rays sharing the label "bullet" plus one untagged overlap, every frame.
		b3World_CastRay( worldId, origin, translation, bullet53, QueryReplayCastFcn, NULL );
		b3World_CastRay( worldId, origin, translation, bullet54, QueryReplayCastFcn, NULL );
		b3World_OverlapAABB( worldId, aabb, untagged, QueryReplayOverlapFcn, NULL );

		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );

	// Round-trip through a file so the interned tag table is exercised on the persisted bytes.
	const char* path = "tagged_query_test.b3rec";
	ENSURE( b3SaveRecordingToFile( rec, path ) );
	b3Recording* loaded = b3LoadRecordingFromFile( path );
	ENSURE( loaded != NULL );

	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( loaded ), b3Recording_GetSize( loaded ), 1 );
	ENSURE( player != NULL );

	b3RecPlayer_SeekFrame( player, 5 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );
	ENSURE( b3RecPlayer_GetFrameQueryCount( player ) == 3 );

	bool saw53 = false, saw54 = false, sawUntagged = false;
	for ( int qi = 0; qi < b3RecPlayer_GetFrameQueryCount( player ); ++qi )
	{
		b3RecQueryInfo info = b3RecPlayer_GetFrameQuery( player, qi );
		if ( info.key == key53 )
		{
			saw53 = true;
			ENSURE( info.id == 53 && info.name != NULL && strcmp( info.name, "bullet" ) == 0 );
		}
		else if ( info.key == key54 )
		{
			saw54 = true;
			ENSURE( info.id == 54 && info.name != NULL && strcmp( info.name, "bullet" ) == 0 );
		}
		else
		{
			sawUntagged = true;
			ENSURE( info.key == 0 && info.id == 0 && info.name == NULL );
		}
	}
	ENSURE( saw53 && saw54 && sawUntagged );

	b3DestroyPlayer( player );
	b3DestroyRecording( loaded );
	b3DestroyRecording( rec );
	return 0;
}

// Empty world: recording starts with no bodies and none are ever created. The empty world is still
// seed-serialized like any other, so replay validates and Restart restores in place with a stable
// world id rather than tearing down and rebuilding the world.
static int EmptyWorldRoundTrip( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int size = b3Recording_GetSize( rec );

	// The seed snapshot is written even with no bodies.
	b3RecHeader hdr;
	memcpy( &hdr, data, sizeof( hdr ) );
	ENSURE( hdr.snapshotSize > 0 );

	ENSURE( b3ValidateReplay( data, size, 1 ) );

	// Restart restores in place, so the replay world id survives a rewind.
	b3RecPlayer* player = b3CreatePlayer( data, size, 1 );
	ENSURE( player != NULL );

	uint32_t worldKey = b3StoreWorldId( b3RecPlayer_GetWorldId( player ) );
	while ( !b3RecPlayer_IsAtEnd( player ) )
	{
		b3RecPlayer_StepFrame( player );
	}
	b3RecPlayer_Restart( player );
	ENSURE( b3StoreWorldId( b3RecPlayer_GetWorldId( player ) ) == worldKey );
	ENSURE( b3RecPlayer_GetFrame( player ) == 0 );
	ENSURE( !b3RecPlayer_HasDiverged( player ) );

	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	return 0;
}

// Exercise every recorded op in a single session, then validate replay at two worker
// counts, round-trip through a file, and drive the incremental player. Mirrors the
// comprehensive RecordingTest in Box2D's test suite (box2d/test/test_recording.c).
static int AllOps( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3World_StartRecording( worldId, rec );

	// Static ground with a box-hull shape
	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	ENSURE( b3Body_IsValid( groundId ) );
	b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 1.0f, 50.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3ShapeId groundShapeId = b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );
	ENSURE( b3Shape_IsValid( groundShapeId ) );

	// Dynamic body with a sphere shape.
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	bodyDef.name = "testBodyWithVeryLongNameThatIsAVeryLongNameLength";
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsValid( bodyId ) );

	b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
	sphereShapeDef.density = 1.0f;
	// Over-length shape name so replay exercises the clamp in the shape def reader, like the body above.
	sphereShapeDef.name = "sphereNameThatExceedsTheLimit";
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( bodyId, &sphereShapeDef, &sphere );
	ENSURE( b3Shape_IsValid( sphereShapeId ) );

	// Capsule shape on a second dynamic body
	b3BodyDef capsuleBodyDef = b3DefaultBodyDef();
	capsuleBodyDef.type = b3_dynamicBody;
	capsuleBodyDef.position = (b3Pos){ 3.0f, 5.0f, 0.0f };
	b3BodyId capsuleBodyId = b3CreateBody( worldId, &capsuleBodyDef );
	ENSURE( b3Body_IsValid( capsuleBodyId ) );

	b3ShapeDef capsuleShapeDef = b3DefaultShapeDef();
	capsuleShapeDef.density = 1.0f;
	b3Capsule capsule = { { 0.0f, -0.4f, 0.0f }, { 0.0f, 0.4f, 0.0f }, 0.25f };
	b3ShapeId capsuleShapeId = b3CreateCapsuleShape( capsuleBodyId, &capsuleShapeDef, &capsule );
	ENSURE( b3Shape_IsValid( capsuleShapeId ) );

	// Custom hull shape on a third dynamic body
	b3Vec3 hullPts[8] = {
		{ -0.5f, -0.5f, -0.5f }, { 0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, -0.5f }, { -0.5f, 0.5f, -0.5f },
		{ -0.5f, -0.5f, 0.5f },	 { 0.5f, -0.5f, 0.5f },	 { 0.5f, 0.5f, 0.5f },	{ -0.5f, 0.5f, 0.5f },
	};
	b3HullData* customHull = b3CreateHull( hullPts, 8, 8 );
	ENSURE( customHull != NULL );

	b3BodyDef hullBodyDef = b3DefaultBodyDef();
	hullBodyDef.type = b3_dynamicBody;
	hullBodyDef.position = (b3Pos){ -3.0f, 5.0f, 0.0f };
	b3BodyId hullBodyId = b3CreateBody( worldId, &hullBodyDef );
	ENSURE( b3Body_IsValid( hullBodyId ) );

	b3ShapeDef hullShapeDef = b3DefaultShapeDef();
	hullShapeDef.density = 1.0f;
	b3ShapeId hullShapeId = b3CreateHullShape( hullBodyId, &hullShapeDef, customHull );
	ENSURE( b3Shape_IsValid( hullShapeId ) );

	// Box hull shape on a fourth dynamic body (b3MakeBoxHull path)
	b3BodyDef boxBodyDef = b3DefaultBodyDef();
	boxBodyDef.type = b3_dynamicBody;
	boxBodyDef.position = (b3Pos){ 6.0f, 5.0f, 0.0f };
	b3BodyId boxBodyId = b3CreateBody( worldId, &boxBodyDef );
	ENSURE( b3Body_IsValid( boxBodyId ) );

	b3ShapeDef boxShapeDef = b3DefaultShapeDef();
	boxShapeDef.density = 2.0f;
	b3BoxHull boxHull = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeId boxShapeId = b3CreateHullShape( boxBodyId, &boxShapeDef, &boxHull.base );
	ENSURE( b3Shape_IsValid( boxShapeId ) );

	// Transformed hull shape on a fifth dynamic body (b3CreateTransformedHullShape path)
	b3BodyDef xformBodyDef = b3DefaultBodyDef();
	xformBodyDef.type = b3_dynamicBody;
	xformBodyDef.position = (b3Pos){ 12.0f, 5.0f, 0.0f };
	b3BodyId xformBodyId = b3CreateBody( worldId, &xformBodyDef );
	ENSURE( b3Body_IsValid( xformBodyId ) );
	b3ShapeDef xformShapeDef = b3DefaultShapeDef();
	xformShapeDef.density = 1.0f;
	b3Transform xformXf = { (b3Vec3){ 0.1f, 0.2f, -0.1f }, b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, 0.4f ) };
	b3ShapeId xformShapeId =
		b3CreateTransformedHullShape( xformBodyId, &xformShapeDef, customHull, xformXf, (b3Vec3){ 1.25f, 0.75f, 1.5f } );
	ENSURE( b3Shape_IsValid( xformShapeId ) );

	// Mesh, height field, and compound static shapes (3D-only)
	b3BodyDef meshBodyDef = b3DefaultBodyDef();
	meshBodyDef.type = b3_staticBody;
	meshBodyDef.position = (b3Pos){ 20.0f, 0.0f, 0.0f };
	b3BodyId meshBodyId = b3CreateBody( worldId, &meshBodyDef );
	b3MeshData* meshData = b3CreateGridMesh( 3, 3, 2.0f, 0, false );
	ENSURE( meshData != NULL );
	b3MeshData* swapMeshData = b3CreateGridMesh( 4, 4, 1.5f, 0, false );
	ENSURE( swapMeshData != NULL );
	b3ShapeDef meshShapeDef = b3DefaultShapeDef();
	b3ShapeId meshShapeId = b3CreateMeshShape( meshBodyId, &meshShapeDef, meshData, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
	ENSURE( b3Shape_IsValid( meshShapeId ) );

	b3BodyDef hfBodyDef = b3DefaultBodyDef();
	hfBodyDef.type = b3_staticBody;
	hfBodyDef.position = (b3Pos){ -20.0f, 0.0f, 0.0f };
	b3BodyId hfBodyId = b3CreateBody( worldId, &hfBodyDef );
	b3HeightFieldData* hf = b3CreateGrid( 4, 4, (b3Vec3){ 2.0f, 1.0f, 2.0f }, false );
	ENSURE( hf != NULL );
	b3ShapeDef hfShapeDef = b3DefaultShapeDef();
	b3CreateHeightFieldShape( hfBodyId, &hfShapeDef, hf );

	b3BodyDef compoundBodyDef = b3DefaultBodyDef();
	compoundBodyDef.type = b3_staticBody;
	compoundBodyDef.position = (b3Pos){ 30.0f, 0.0f, 0.0f };
	b3BodyId compoundBodyId = b3CreateBody( worldId, &compoundBodyDef );
	b3CompoundSphereDef compSphere;
	compSphere.sphere = (b3Sphere){ { 0.0f, 0.0f, 0.0f }, 1.0f };
	compSphere.material = b3DefaultSurfaceMaterial();
	b3CompoundDef compoundDef;
	memset( &compoundDef, 0, sizeof( compoundDef ) );
	compoundDef.spheres = &compSphere;
	compoundDef.sphereCount = 1;
	b3CompoundData* compound = b3CreateCompound( &compoundDef );
	ENSURE( compound != NULL );
	b3ShapeDef compoundShapeDef = b3DefaultShapeDef();
	b3CreateBakedCompoundShape( compoundBodyId, &compoundShapeDef, compound );

	// Throwaway shape to exercise DestroyShape
	b3Sphere tmpSphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
	b3ShapeId tmpShapeId = b3CreateSphereShape( capsuleBodyId, &capsuleShapeDef, &tmpSphere );
	b3DestroyShape( tmpShapeId, true );

	// Shape mutators: SetFriction, SetRestitution, SetDensity, SetSurfaceMaterial, SetMeshMaterial,
	// SetFilter, EnableSensorEvents, EnableContactEvents, EnableHitEvents, EnablePreSolveEvents,
	// ApplyWind, SetSphere, SetCapsule, SetHull, SetMesh, SetName
	b3Shape_SetFriction( boxShapeId, 0.3f );
	b3Shape_SetRestitution( capsuleShapeId, 0.5f );
	b3Shape_SetDensity( boxShapeId, 3.0f, true );
	b3SurfaceMaterial surfMat = b3DefaultSurfaceMaterial();
	surfMat.friction = 0.7f;
	surfMat.restitution = 0.1f;
	b3Shape_SetSurfaceMaterial( capsuleShapeId, surfMat );
	b3Filter shapeFilter = b3DefaultFilter();
	shapeFilter.categoryBits = 0x2;
	b3Shape_SetFilter( boxShapeId, shapeFilter, false );
	b3Shape_EnableSensorEvents( capsuleShapeId, true );
	b3Shape_EnableContactEvents( capsuleShapeId, true );
	b3Shape_EnableHitEvents( boxShapeId, true );
	b3Shape_EnablePreSolveEvents( boxShapeId, true );
	b3Shape_ApplyWind( capsuleShapeId, (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.1f, 0.0f, 10.0f, true );
	b3Sphere newSphere = { { 0.0f, 0.0f, 0.0f }, 0.45f };
	b3Shape_SetSphere( sphereShapeId, &newSphere );
	b3Capsule newCapsule = { { 0.0f, -0.3f, 0.0f }, { 0.0f, 0.3f, 0.0f }, 0.3f };
	b3Shape_SetCapsule( capsuleShapeId, &newCapsule );
	b3Shape_SetName( boxShapeId, "box" );

	// Geometry swaps intern into the registry at the record site. The repeated SetHull takes the
	// shared hull short circuit, which changes nothing and so must leave the stream alone.
	b3BoxHull swapHull = b3MakeBoxHull( 0.3f, 0.7f, 0.4f );
	b3Shape_SetHull( boxShapeId, &swapHull.base );
	b3Shape_SetHull( boxShapeId, &swapHull.base );
	b3Shape_SetMesh( meshShapeId, swapMeshData, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
	b3Shape_SetMeshMaterial( meshShapeId, surfMat, 0 );

	// Body mutators: SetTransform, SetLinearVelocity/AngularVelocity (Vec3), SetName,
	// damping, gravity scale, sleep threshold, SetAwake, EnableSleep, SetBullet, SetMotionLocks,
	// SetMassData, ApplyMassFromShapes, SetType, SetTargetTransform, Disable/Enable, EnableContactRecycling,
	// EnableHitEvents, all force/impulse/torque variants
	b3Body_SetTransform( bodyId, (b3Pos){ 1.0f, 6.0f, 0.0f }, b3Quat_identity );
	b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 0.5f, 0.0f, 0.0f } );
	b3Body_SetAngularVelocity( bodyId, (b3Vec3){ 0.0f, 0.25f, 0.0f } );
	b3Body_SetName( bodyId, "renamedBody" );
	b3Body_SetLinearDamping( bodyId, 0.1f );
	b3Body_SetAngularDamping( bodyId, 0.05f );
	b3Body_SetGravityScale( bodyId, 0.9f );
	b3Body_SetSleepThreshold( bodyId, 0.02f );
	b3Body_SetSafetyFactor( bodyId, 0.25f );
	b3Body_EnableSleep( bodyId, false );
	b3Body_SetBullet( bodyId, true );
	b3Body_EnableContactRecycling( bodyId, false );
	b3Body_EnableHitEvents( bodyId, true );
	b3Body_SetMotionLocks( bodyId, (b3MotionLocks){ false, false, false, false, false, true } );
	b3MassData massData;
	massData.mass = 2.0f;
	massData.center = (b3Vec3){ 0.0f, 0.0f, 0.0f };
	massData.inertia = b3Mat3_identity;
	b3Body_SetMassData( bodyId, massData );
	b3Body_ApplyMassFromShapes( bodyId );
	b3Body_SetType( capsuleBodyId, b3_kinematicBody );
	b3Body_SetType( capsuleBodyId, b3_dynamicBody );
	b3Body_SetAwake( bodyId, true );

	// Kinematic body to exercise SetTargetTransform
	b3BodyDef kinematicDef = b3DefaultBodyDef();
	kinematicDef.type = b3_kinematicBody;
	kinematicDef.position = (b3Pos){ -6.0f, 5.0f, 0.0f };
	b3BodyId kinematicId = b3CreateBody( worldId, &kinematicDef );
	b3BoxHull kinBox = b3MakeBoxHull( 0.4f, 0.4f, 0.4f );
	b3ShapeDef kinShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( kinematicId, &kinShapeDef, &kinBox.base );
	b3WorldTransform kinTarget;
	kinTarget.p = (b3Pos){ -5.0f, 5.0f, 0.0f };
	kinTarget.q = b3Quat_identity;
	b3Body_SetTargetTransform( kinematicId, kinTarget, 1.0f / 60.0f, true );

	// Body to exercise Disable/Enable
	b3BodyDef disableDef = b3DefaultBodyDef();
	disableDef.type = b3_dynamicBody;
	disableDef.position = (b3Pos){ 9.0f, 5.0f, 0.0f };
	b3BodyId disableId = b3CreateBody( worldId, &disableDef );
	b3Sphere disableSphere = { { 0.0f, 0.0f, 0.0f }, 0.3f };
	b3CreateSphereShape( disableId, &sphereShapeDef, &disableSphere );
	b3Body_Disable( disableId );
	b3Body_Enable( disableId );

	// Force/impulse/torque (Vec3 args in 3D)
	b3Body_ApplyForce( bodyId, (b3Vec3){ 0.0f, 50.0f, 0.0f }, (b3Pos){ 1.0f, 6.0f, 0.0f }, true );
	b3Body_ApplyForceToCenter( bodyId, (b3Vec3){ 5.0f, 0.0f, 0.0f }, true );
	b3Body_ApplyTorque( bodyId, (b3Vec3){ 0.0f, 1.0f, 0.0f }, true );
	b3Body_ApplyLinearImpulse( bodyId, (b3Vec3){ 0.1f, 0.0f, 0.0f }, (b3Pos){ 1.0f, 6.0f, 0.0f }, true );
	b3Body_ApplyLinearImpulseToCenter( bodyId, (b3Vec3){ 0.0f, 0.1f, 0.0f }, true );
	b3Body_ApplyAngularImpulse( bodyId, (b3Vec3){ 0.0f, 0.05f, 0.0f }, true );

	// Joint bodies: a row of dynamic bodies connected by each joint type
	b3BodyId jb[9];
	for ( int i = 0; i < 9; ++i )
	{
		b3BodyDef jbd = b3DefaultBodyDef();
		jbd.type = b3_dynamicBody;
		jbd.position = (b3Pos){ -8.0f + (float)i * 2.0f, 10.0f, 0.0f };
		jb[i] = b3CreateBody( worldId, &jbd );
		b3Sphere js = { { 0.0f, 0.0f, 0.0f }, 0.25f };
		b3ShapeDef jsd = b3DefaultShapeDef();
		jsd.density = 1.0f;
		b3CreateSphereShape( jb[i], &jsd, &js );
	}

	// Revolute joint with full setter coverage and the generic joint mutators
	b3RevoluteJointDef revDef = b3DefaultRevoluteJointDef();
	revDef.base.bodyIdA = jb[0];
	revDef.base.bodyIdB = jb[1];
	revDef.base.localFrameA.p = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	revDef.base.localFrameB.p = (b3Vec3){ -1.0f, 0.0f, 0.0f };
	b3JointId revId = b3CreateRevoluteJoint( worldId, &revDef );
	ENSURE( b3Joint_IsValid( revId ) );
	b3RevoluteJoint_EnableLimit( revId, true );
	b3RevoluteJoint_SetLimits( revId, -1.0f, 1.0f );
	b3RevoluteJoint_EnableMotor( revId, true );
	b3RevoluteJoint_SetMotorSpeed( revId, 0.5f );
	b3RevoluteJoint_SetMaxMotorTorque( revId, 10.0f );
	b3RevoluteJoint_EnableSpring( revId, true );
	b3RevoluteJoint_SetSpringHertz( revId, 2.0f );
	b3RevoluteJoint_SetSpringDampingRatio( revId, 0.5f );
	b3RevoluteJoint_SetTargetAngle( revId, 0.25f );
	b3Joint_SetLocalFrameA( revId, (b3Transform){ (b3Vec3){ 1.0f, 0.0f, 0.0f }, b3Quat_identity } );
	b3Joint_SetLocalFrameB( revId, (b3Transform){ (b3Vec3){ -1.0f, 0.0f, 0.0f }, b3Quat_identity } );
	b3Joint_SetConstraintTuning( revId, 60.0f, 2.0f );
	b3Joint_SetForceThreshold( revId, 100.0f );
	b3Joint_SetTorqueThreshold( revId, 50.0f );
	b3Joint_SetCollideConnected( revId, false );
	b3Joint_WakeBodies( revId );

	// Distance joint
	b3DistanceJointDef distDef = b3DefaultDistanceJointDef();
	distDef.base.bodyIdA = jb[1];
	distDef.base.bodyIdB = jb[2];
	distDef.length = 2.0f;
	b3JointId distId = b3CreateDistanceJoint( worldId, &distDef );
	b3DistanceJoint_SetLength( distId, 2.2f );
	b3DistanceJoint_EnableSpring( distId, true );
	b3DistanceJoint_SetSpringHertz( distId, 3.0f );
	b3DistanceJoint_SetSpringDampingRatio( distId, 0.4f );
	b3DistanceJoint_SetSpringForceRange( distId, -50.0f, 50.0f );
	b3DistanceJoint_EnableLimit( distId, true );
	b3DistanceJoint_SetLengthRange( distId, 1.0f, 4.0f );
	b3DistanceJoint_EnableMotor( distId, true );
	b3DistanceJoint_SetMotorSpeed( distId, 0.3f );
	b3DistanceJoint_SetMaxMotorForce( distId, 5.0f );

	// Filter joint (plus a throwaway to exercise DestroyJoint)
	b3FilterJointDef filterDef = b3DefaultFilterJointDef();
	filterDef.base.bodyIdA = jb[2];
	filterDef.base.bodyIdB = jb[3];
	b3JointId filterId = b3CreateFilterJoint( worldId, &filterDef );
	ENSURE( b3Joint_IsValid( filterId ) );

	b3DistanceJointDef tmpJointDef = b3DefaultDistanceJointDef();
	tmpJointDef.base.bodyIdA = jb[0];
	tmpJointDef.base.bodyIdB = jb[8];
	tmpJointDef.length = 5.0f;
	b3JointId tmpJointId = b3CreateDistanceJoint( worldId, &tmpJointDef );
	b3DestroyJoint( tmpJointId, true );

	// Motor joint (Vec3 velocities in 3D)
	b3MotorJointDef motorDef = b3DefaultMotorJointDef();
	motorDef.base.bodyIdA = jb[3];
	motorDef.base.bodyIdB = jb[4];
	b3JointId motorId = b3CreateMotorJoint( worldId, &motorDef );
	b3MotorJoint_SetLinearVelocity( motorId, (b3Vec3){ 0.1f, 0.0f, 0.0f } );
	b3MotorJoint_SetAngularVelocity( motorId, (b3Vec3){ 0.0f, 0.2f, 0.0f } );
	b3MotorJoint_SetMaxVelocityForce( motorId, 10.0f );
	b3MotorJoint_SetMaxVelocityTorque( motorId, 10.0f );
	b3MotorJoint_SetLinearHertz( motorId, 2.0f );
	b3MotorJoint_SetLinearDampingRatio( motorId, 0.5f );
	b3MotorJoint_SetAngularHertz( motorId, 2.0f );
	b3MotorJoint_SetAngularDampingRatio( motorId, 0.5f );
	b3MotorJoint_SetMaxSpringForce( motorId, 20.0f );
	b3MotorJoint_SetMaxSpringTorque( motorId, 20.0f );

	// Prismatic joint
	b3PrismaticJointDef prisDef = b3DefaultPrismaticJointDef();
	prisDef.base.bodyIdA = jb[4];
	prisDef.base.bodyIdB = jb[5];
	b3JointId prisId = b3CreatePrismaticJoint( worldId, &prisDef );
	b3PrismaticJoint_EnableSpring( prisId, true );
	b3PrismaticJoint_SetSpringHertz( prisId, 2.0f );
	b3PrismaticJoint_SetSpringDampingRatio( prisId, 0.5f );
	b3PrismaticJoint_SetTargetTranslation( prisId, 0.1f );
	b3PrismaticJoint_EnableLimit( prisId, true );
	b3PrismaticJoint_SetLimits( prisId, -1.0f, 1.0f );
	b3PrismaticJoint_EnableMotor( prisId, true );
	b3PrismaticJoint_SetMotorSpeed( prisId, 0.2f );
	b3PrismaticJoint_SetMaxMotorForce( prisId, 8.0f );

	// Spherical joint (3D-only)
	b3SphericalJointDef sphDef = b3DefaultSphericalJointDef();
	sphDef.base.bodyIdA = jb[5];
	sphDef.base.bodyIdB = jb[6];
	b3JointId sphId = b3CreateSphericalJoint( worldId, &sphDef );
	b3SphericalJoint_EnableConeLimit( sphId, true );
	b3SphericalJoint_SetConeLimit( sphId, 0.5f );
	b3SphericalJoint_EnableTwistLimit( sphId, true );
	b3SphericalJoint_SetTwistLimits( sphId, -0.3f, 0.3f );
	b3SphericalJoint_EnableSpring( sphId, true );
	b3SphericalJoint_SetSpringHertz( sphId, 3.0f );
	b3SphericalJoint_SetSpringDampingRatio( sphId, 0.5f );
	b3SphericalJoint_SetTargetRotation( sphId, b3Quat_identity );
	b3SphericalJoint_EnableMotor( sphId, true );
	b3SphericalJoint_SetMotorVelocity( sphId, (b3Vec3){ 0.0f, 0.1f, 0.0f } );
	b3SphericalJoint_SetMaxMotorTorque( sphId, 5.0f );

	// Weld joint
	b3WeldJointDef weldDef = b3DefaultWeldJointDef();
	weldDef.base.bodyIdA = jb[6];
	weldDef.base.bodyIdB = jb[7];
	b3JointId weldId = b3CreateWeldJoint( worldId, &weldDef );
	b3WeldJoint_SetLinearHertz( weldId, 5.0f );
	b3WeldJoint_SetLinearDampingRatio( weldId, 0.6f );
	b3WeldJoint_SetAngularHertz( weldId, 5.0f );
	b3WeldJoint_SetAngularDampingRatio( weldId, 0.6f );

	// Wheel joint (Box3D uses Suspension/Spin/Steering naming)
	b3WheelJointDef wheelDef = b3DefaultWheelJointDef();
	wheelDef.base.bodyIdA = jb[7];
	wheelDef.base.bodyIdB = jb[8];
	b3JointId wheelId = b3CreateWheelJoint( worldId, &wheelDef );
	b3WheelJoint_EnableSuspension( wheelId, true );
	b3WheelJoint_SetSuspensionHertz( wheelId, 4.0f );
	b3WheelJoint_SetSuspensionDampingRatio( wheelId, 0.7f );
	b3WheelJoint_EnableSuspensionLimit( wheelId, true );
	b3WheelJoint_SetSuspensionLimits( wheelId, -0.5f, 0.5f );
	b3WheelJoint_EnableSpinMotor( wheelId, true );
	b3WheelJoint_SetSpinMotorSpeed( wheelId, 1.0f );
	b3WheelJoint_SetMaxSpinTorque( wheelId, 6.0f );
	b3WheelJoint_EnableSteering( wheelId, true );
	b3WheelJoint_SetSteeringHertz( wheelId, 2.0f );
	b3WheelJoint_SetSteeringDampingRatio( wheelId, 0.5f );
	b3WheelJoint_SetMaxSteeringTorque( wheelId, 3.0f );
	b3WheelJoint_EnableSteeringLimit( wheelId, true );
	b3WheelJoint_SetSteeringLimits( wheelId, -0.5f, 0.5f );
	b3WheelJoint_SetTargetSteeringAngle( wheelId, 0.1f );

	// Parallel joint (3D-only)
	b3ParallelJointDef parallelDef = b3DefaultParallelJointDef();
	parallelDef.base.bodyIdA = groundId;
	parallelDef.base.bodyIdB = bodyId;
	b3JointId parallelId = b3CreateParallelJoint( worldId, &parallelDef );
	b3ParallelJoint_SetSpringHertz( parallelId, 2.0f );
	b3ParallelJoint_SetSpringDampingRatio( parallelId, 0.5f );
	b3ParallelJoint_SetMaxTorque( parallelId, 20.0f );

	// World config mutators
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -9.8f, 0.0f } );
	b3World_EnableSleeping( worldId, true );
	b3World_EnableContinuous( worldId, false );
	b3World_EnableWarmStarting( worldId, true );
	b3World_EnableSpeculative( worldId, true );
	b3World_SetRestitutionThreshold( worldId, 1.5f );
	b3World_SetHitEventThreshold( worldId, 2.0f );
	b3World_SetContactTuning( worldId, 30.0f, 10.0f, 3.0f );
	b3World_SetContactRecycleDistance( worldId, 0.05f );
	b3World_SetMaximumLinearSpeed( worldId, 100.0f );
	b3World_RebuildStaticTree( worldId );

	b3ExplosionDef explosion = b3DefaultExplosionDef();
	explosion.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	explosion.radius = 3.0f;
	explosion.falloff = 1.0f;
	explosion.impulsePerArea = 2.0f;
	b3World_Explode( worldId, &explosion );

	// Pre-step queries
	b3QueryFilter qfilter = b3DefaultQueryFilter();
	b3AABB qaabb = { { -10.0f, -5.0f, -10.0f }, { 10.0f, 15.0f, 10.0f } };
	b3World_OverlapAABB( worldId, qaabb, qfilter, QueryReplayOverlapFcn, NULL );
	b3Pos qorigin = { 0.0f, 15.0f, 0.0f };
	b3Vec3 proxyPts = { 0.0f, 0.0f, 0.0f };
	b3ShapeProxy proxy = { &proxyPts, 1, 0.5f };
	b3World_OverlapShape( worldId, qorigin, &proxy, qfilter, QueryReplayOverlapFcn, NULL );
	b3Vec3 qTranslation = { 0.0f, -20.0f, 0.0f };
	b3World_CastRay( worldId, qorigin, qTranslation, qfilter, QueryReplayCastFcn, NULL );
	b3World_CastRayClosest( worldId, qorigin, qTranslation, qfilter );
	b3World_CastShape( worldId, qorigin, &proxy, qTranslation, qfilter, QueryReplayCastFcn, NULL );
	b3Capsule mover = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 0.3f };
	b3World_CastMover( worldId, qorigin, &mover, qTranslation, qfilter, QueryReplayMoverFilterFcn, NULL );
	b3World_CollideMover( worldId, qorigin, &mover, qfilter, QueryReplayPlaneFcn, NULL );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	for ( int i = 0; i < 12; ++i )
	{
		// Inject mutators mid-simulation
		if ( i == 6 )
		{
			b3Body_ApplyLinearImpulseToCenter( capsuleBodyId, (b3Vec3){ 2.0f, 0.0f, 0.0f }, true );
			b3Body_SetGravityScale( bodyId, 1.0f );
		}

		// Issue queries mid-loop to exercise recording across steps
		if ( i == 3 )
		{
			b3World_OverlapAABB( worldId, qaabb, qfilter, QueryReplayOverlapFcn, NULL );
			b3World_CastRay( worldId, qorigin, qTranslation, qfilter, QueryReplayCastFcn, NULL );
		}

		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	// Free geometry allocated for this subtest
	b3DestroyHull( customHull );
	b3DestroyMesh( meshData );
	b3DestroyMesh( swapMeshData );
	b3DestroyHeightField( hf );
	b3DestroyCompound( compound );

	const uint8_t* recData = b3Recording_GetData( rec );
	int recSize = b3Recording_GetSize( rec );
	ENSURE( recSize > 0 );

	// Replay headless at worker count 1 and 4, a cross-thread determinism check matching Box2D.
	ENSURE( b3ValidateReplay( recData, recSize, 1 ) );
	ENSURE( b3ValidateReplay( recData, recSize, 4 ) );

	// File round-trip
	ENSURE( b3SaveRecordingToFile( rec, s_recPath ) );
	b3Recording* loaded = b3LoadRecordingFromFile( s_recPath );
	ENSURE( loaded != NULL );
	ENSURE( b3ValidateReplay( b3Recording_GetData( loaded ), b3Recording_GetSize( loaded ), 1 ) );
	b3DestroyRecording( loaded );

	// Drive the incremental player. Exercises per-frame stepping, restart, getters, and the
	// draw path beyond what b3ValidateReplay covers.
	{
		b3RecPlayer* player = b3CreatePlayer( recData, recSize, 1 );
		ENSURE( player != NULL );

		b3RecPlayerInfo info = b3RecPlayer_GetInfo( player );
		b3Vec3 recExtents = b3Sub( info.bounds.upperBound, info.bounds.lowerBound );
		ENSURE( recExtents.x > 0.0f && recExtents.y > 0.0f );

		// Build a no-op b3DebugDraw to exercise the draw path headlessly
		b3DebugDraw dd = b3DefaultDebugDraw();
		dd.DrawShapeFcn = RecTestDrawShape;
		dd.drawShapes = true;
		dd.drawingBounds = (b3AABB){ { -1.0e6f, -1.0e6f, -1.0e6f }, { 1.0e6f, 1.0e6f, 1.0e6f } };

		int frames = 0;
		while ( b3RecPlayer_StepFrame( player ) )
		{
			if ( frames % 2 == 0 )
			{
				b3RecPlayer_DrawFrameQueries( player, &dd, -1, -1 );
			}
			frames += 1;
		}
		ENSURE( frames == 12 );
		ENSURE( b3RecPlayer_GetFrame( player ) == 12 );
		ENSURE( b3RecPlayer_IsAtEnd( player ) );
		ENSURE( b3RecPlayer_HasDiverged( player ) == false );

		// The trailing DestroyWorld is an end marker; the world stays valid after end
		ENSURE( b3World_IsValid( b3RecPlayer_GetWorldId( player ) ) );

		// Restart reproduces the same run without reloading the file
		b3RecPlayer_Restart( player );
		ENSURE( b3RecPlayer_GetFrame( player ) == 0 );
		ENSURE( b3RecPlayer_IsAtEnd( player ) == false );

		int frames2 = 0;
		while ( b3RecPlayer_StepFrame( player ) )
		{
			frames2 += 1;
		}
		ENSURE( frames2 == 12 );
		ENSURE( b3RecPlayer_HasDiverged( player ) == false );

		b3DestroyPlayer( player );
	}

	b3DestroyRecording( rec );
	remove( s_recPath );
	return 0;
}

// A transformed hull bakes its transform and non-uniform scale into fresh hull data at create time.
// It must be recorded like any other shape create, else its shape id allocation is invisible to the
// player and every later id drifts. A plain hull created after it would then mismatch on replay.
static int TransformedHullRoundTrip( void )
{
	b3Vec3 pts[8] = {
		{ -1.0f, -1.0f, -1.0f }, { 1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, -1.0f }, { -1.0f, 1.0f, -1.0f },
		{ -1.0f, -1.0f, 1.0f },	 { 1.0f, -1.0f, 1.0f },	 { 1.0f, 1.0f, 1.0f },	{ -1.0f, 1.0f, 1.0f },
	};
	b3HullData* hull = b3CreateHull( pts, 8, 8 );
	ENSURE( hull != NULL );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3World_StartRecording( worldId, rec );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;

	// Baked transform with a rotation and non-uniform scale, the path Unreal uses for instanced hulls.
	b3Transform xf = { (b3Vec3){ 0.25f, 0.0f, -0.5f }, b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.3f ) };
	b3Vec3 scl = { 1.5f, 0.5f, 2.0f };
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ (float)( i * 3 ), 5.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeId sid = b3CreateTransformedHullShape( bodyId, &shapeDef, hull, xf, scl );
		ENSURE( b3Shape_IsValid( sid ) );
	}

	// A plain hull after the transformed ones: if the transformed creates desynced the id pool, this
	// shape's recorded id would not match what replay allocates and b3ValidateReplay would fail.
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.0f, 10.0f, 0.0f };
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeId sid = b3CreateHullShape( bodyId, &shapeDef, hull );
		ENSURE( b3Shape_IsValid( sid ) );
	}

	// Step past the keyframe interval so replay captures a keyframe. That path re-serializes the live
	// world and interns the baked hull, which must already be in the pre-seeded registry.
	for ( int i = 0; i < 20; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );
	b3DestroyHull( hull );

	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 ) );
	ENSURE( b3ValidateReplay( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 4 ) );

	b3DestroyRecording( rec );
	return 0;
}

// Patch the reserved header bytes to nonzero and confirm b3ValidateReplay ignores them.
// Guards a future change that starts validating them or shrinks the header.
static int ReservedHeaderBytes( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_StartRecording( worldId, rec );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	b3BodyId bodyId = b3CreateBody( worldId, &bd );
	b3Sphere s = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 1.0f;
	b3CreateSphereShape( bodyId, &sd, &s );

	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* recData = b3Recording_GetData( rec );
	int recSize = b3Recording_GetSize( rec );
	ENSURE( recSize >= (int)sizeof( b3RecHeader ) );

	uint8_t* patched = (uint8_t*)b3Alloc( (size_t)recSize );
	memcpy( patched, recData, (size_t)recSize );
	// Mutate named reserved fields so a layout change cannot turn stale byte offsets into active-header corruption.
	b3RecHeader patchedHeader;
	memcpy( &patchedHeader, patched, sizeof( patchedHeader ) );
	patchedHeader.reserved = 0xAB;
	patchedHeader.reserved3 = 0xBC9A7856;
	memcpy( patched, &patchedHeader, sizeof( patchedHeader ) );
	ENSURE( b3ValidateReplay( patched, recSize, 1 ) );
	b3Free( patched, (size_t)recSize );

	b3DestroyRecording( rec );
	return 0;
}

// Geometry that shares a content hash but differs in bytes must dedup exactly. Mirrors the keyframe
// flow that crashed: the seed appends every slot 1:1 (even byte-identical duplicates a hash collision
// left in an already-recorded file), then capture must resolve a live blob back to an existing slot
// without growing the registry. Forces the collision by handing the same hash to distinct blobs.
static int GeometryHashCollision( void )
{
	const int n = 16;
	const uint64_t sharedHash = 0xABCD1234ull;

	b3GeometryRegistry reg = { 0 };

	uint8_t* blobA = (uint8_t*)b3Alloc( (size_t)n );
	uint8_t* blobB = (uint8_t*)b3Alloc( (size_t)n );
	memset( blobA, 0xAA, (size_t)n );
	memset( blobB, 0xBB, (size_t)n );

	// Distinct blobs colliding on the hash must become two entries, not a false dedup.
	uint32_t idA = b3InternGeometry( &reg, b3_geometryHull, sharedHash, blobA, n );
	uint32_t idB = b3InternGeometry( &reg, b3_geometryHull, sharedHash, blobB, n );
	ENSURE( idA != idB );
	ENSURE( reg.entries.count == 2 );

	// Re-interning either blob must find it through the hash chain and never grow the registry,
	// including the one shadowed behind the bucket head. The old single-entry lookup missed the
	// shadowed blob and appended a duplicate, which is exactly what tripped the keyframe assert.
	uint8_t* blobA2 = (uint8_t*)b3Alloc( (size_t)n );
	memset( blobA2, 0xAA, (size_t)n );
	ENSURE( b3InternGeometry( &reg, b3_geometryHull, sharedHash, blobA2, n ) == idA );
	ENSURE( reg.entries.count == 2 );

	uint8_t* blobB2 = (uint8_t*)b3Alloc( (size_t)n );
	memset( blobB2, 0xBB, (size_t)n );
	ENSURE( b3InternGeometry( &reg, b3_geometryHull, sharedHash, blobB2, n ) == idB );
	ENSURE( reg.entries.count == 2 );

	b3FreeRegistry( &reg );

	// Seed-then-capture: appending byte-identical duplicate slots keeps id == slot index, and a later
	// exact intern still resolves to one of them without appending a new entry.
	b3GeometryRegistry seeded = { 0 };
	uint8_t* slot0 = (uint8_t*)b3Alloc( (size_t)n );
	uint8_t* slot1 = (uint8_t*)b3Alloc( (size_t)n );
	uint8_t* slot2 = (uint8_t*)b3Alloc( (size_t)n );
	memset( slot0, 0xAA, (size_t)n );
	memset( slot1, 0xBB, (size_t)n );
	memset( slot2, 0xAA, (size_t)n ); // duplicate of slot0
	ENSURE( b3AppendGeometry( &seeded, b3_geometryHull, sharedHash, slot0, n ) == 0 );
	ENSURE( b3AppendGeometry( &seeded, b3_geometryHull, sharedHash, slot1, n ) == 1 );
	ENSURE( b3AppendGeometry( &seeded, b3_geometryHull, sharedHash, slot2, n ) == 2 );

	uint8_t* live = (uint8_t*)b3Alloc( (size_t)n );
	memset( live, 0xAA, (size_t)n );
	uint32_t resolved = b3InternGeometry( &seeded, b3_geometryHull, sharedHash, live, n );
	ENSURE( seeded.entries.count == 3 );	  // no growth
	ENSURE( resolved == 0 || resolved == 2 ); // a valid slot index for that content
	ENSURE( seeded.entries.data[resolved].byteCount == n );
	ENSURE( memcmp( seeded.entries.data[resolved].bytes, slot0, (size_t)n ) == 0 );

	b3FreeRegistry( &seeded );
	return 0;
}

// Staged stepping must reveal a mid-stream body at its creation transform. A body created and given an
// impulse in one recorded step is first placed by CreateBody, then displaced by the following Step. Atomic
// replay fuses the two, so the body is only ever seen already moved. Staged replay parks between them so
// the pre-integration pose is drawable. Verify the parked pose is the creation transform, that atomic
// replay does not show it, and that the extra park does not perturb the end state.
static int StagedStepCreationPose( void )
{
	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetGravity( worldId, (b3Vec3){ 0.0f, -10.0f, 0.0f } );

	b3World_StartRecording( worldId, rec );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	// A few empty steps so the creation lands inside the stream, not at frame 0.
	const int leadFrames = 3;
	for ( int i = 0; i < leadFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// The creation transform under test. No ground, so the body is the only create and stays ballistic.
	// Components are exact in float so the round-trip pose compares tight.
	const b3Pos spawn = { 1.0, 20.0, -2.0 };
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = spawn;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	// Impulse along +x so the first integrated step moves the body a visible distance off the spawn.
	b3Body_ApplyLinearImpulseToCenter( bodyId, (b3Vec3){ 5.0f, 0.0f, 0.0f }, true );

	const int creationFrame = leadFrames + 1;
	b3World_Step( worldId, timeStep, subStepCount ); // advances to creationFrame

	const int totalFrames = creationFrame + 3;
	for ( int i = creationFrame; i < totalFrames; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );

	// Atomic replay fuses the create and its step, so at the creation frame the body is already
	// integrated and displaced along +x. Capture that pose and the final state hash.
	b3RecPlayer* atomic = b3CreatePlayer( data, sz, 1 );
	ENSURE( atomic != NULL );
	b3Pos atomicPose = { 0.0, 0.0, 0.0 };
	while ( !b3RecPlayer_IsAtEnd( atomic ) )
	{
		b3RecPlayer_StepFrame( atomic );
		if ( b3RecPlayer_GetFrame( atomic ) == creationFrame )
		{
			b3BodyId id = b3RecPlayer_GetBodyId( atomic, 0 );
			ENSURE( b3Body_IsValid( id ) );
			atomicPose = b3Body_GetPosition( id );
		}
	}
	ENSURE( b3RecPlayer_GetFrame( atomic ) == totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( atomic ) );
	ENSURE( atomicPose.x - spawn.x > 0.01 ); // the impulse moved it before it was ever seen
	uint64_t atomicHash = b3HashWorldState( b3GetWorldFromId( b3RecPlayer_GetWorldId( atomic ) ) );
	b3DestroyPlayer( atomic );

	// Staged replay: step forward until the first pre-step park. It must sit at the creation frame's
	// pre-integration state with the new body at exactly its spawn transform.
	b3RecPlayer* staged = b3CreatePlayer( data, sz, 1 );
	ENSURE( staged != NULL );
	while ( !b3RecPlayer_IsAtEnd( staged ) )
	{
		b3RecPlayer_SubStepFrame( staged );
		if ( b3RecPlayer_IsAtPreStep( staged ) )
		{
			break;
		}
	}
	ENSURE( b3RecPlayer_IsAtPreStep( staged ) );
	ENSURE( b3RecPlayer_GetFrame( staged ) == creationFrame - 1 ); // parked before the step, frame not advanced

	b3BodyId stagedId = b3RecPlayer_GetBodyId( staged, 0 );
	ENSURE( b3Body_IsValid( stagedId ) );
	b3Pos parkPose = b3Body_GetPosition( stagedId );
	ENSURE_SMALL( parkPose.x - spawn.x, 1.0e-4 );
	ENSURE_SMALL( parkPose.y - spawn.y, 1.0e-4 );
	ENSURE_SMALL( parkPose.z - spawn.z, 1.0e-4 );

	// Finishing the frame integrates the body, so it leaves the spawn pose on the next advance.
	b3RecPlayer_SubStepFrame( staged );
	ENSURE( b3RecPlayer_GetFrame( staged ) == creationFrame );
	ENSURE( !b3RecPlayer_IsAtPreStep( staged ) );

	// Run staged to the end: the same op stream, so it must land on the atomic end state bit for bit.
	while ( !b3RecPlayer_IsAtEnd( staged ) )
	{
		b3RecPlayer_SubStepFrame( staged );
	}
	ENSURE( b3RecPlayer_GetFrame( staged ) == totalFrames );
	ENSURE( !b3RecPlayer_HasDiverged( staged ) );
	uint64_t stagedHash = b3HashWorldState( b3GetWorldFromId( b3RecPlayer_GetWorldId( staged ) ) );
	ENSURE( stagedHash == atomicHash );
	b3DestroyPlayer( staged );

	b3DestroyRecording( rec );
	return 0;
}

// Shape names are debug only and do not feed the determinism hash, so b3ValidateReplay cannot catch a
// broken name round-trip. Replay through the player and read the names back to prove the def field and
// the ShapeSetName op survive serialization.
static int ShapeNameReplay( void )
{
	const char* names[3] = {
		"def",
		"set",
		"abcdefghijklmnopqrstuvwxyz",
	};

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_StartRecording( worldId, rec );

	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };

	// One shape per body so read-back never depends on per-body shape ordering.
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyDef bd = b3DefaultBodyDef();
		bd.type = b3_dynamicBody;
		bd.position = (b3Pos){ 3.0f * (float)i, 5.0f, 0.0f };
		b3BodyId body = b3CreateBody( worldId, &bd );

		b3ShapeDef sd = b3DefaultShapeDef();
		sd.density = 1.0f;
		if ( i != 1 )
		{
			sd.name = names[i];
		}
		b3ShapeId shape = b3CreateSphereShape( body, &sd, &sphere );
		if ( i == 1 )
		{
			b3Shape_SetName( shape, names[i] );
		}
	}

	for ( int i = 0; i < 4; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );

	const uint8_t* data = b3Recording_GetData( rec );
	int sz = b3Recording_GetSize( rec );

	b3RecPlayer* player = b3CreatePlayer( data, sz, 1 );
	ENSURE( player != NULL );
	while ( b3RecPlayer_StepFrame( player ) )
	{
	}
	ENSURE( b3RecPlayer_HasDiverged( player ) == false );

	// Body ordinals follow creation order in the replayed world.
	for ( int i = 0; i < 3; ++i )
	{
		b3BodyId body = b3RecPlayer_GetBodyId( player, i );
		ENSURE( b3Body_IsValid( body ) );

		b3ShapeId shape;
		ENSURE( b3Body_GetShapes( body, &shape, 1 ) == 1 );
		const char* got = b3Shape_GetName( shape );
		ENSURE( got != NULL );

		int srcLen = (int)strlen( names[i] );
		ENSURE( (int)strlen( got ) == srcLen );
		if ( srcLen > 0 )
		{
			ENSURE( strncmp( got, names[i], (size_t)srcLen ) == 0 );
		}
	}

	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	return 0;
}

// A box sliding across a mesh floor, with the option to swap both geometries and retune a
// per-triangle material part way through. Returns the final state hash so the caller can prove the
// mutations move the simulation. Recording is optional so the same scene serves as the control.
static uint64_t RunGeometryMutatorScene( b3Recording* rec, bool mutate, const b3MeshData* meshA, const b3MeshData* meshB,
										 const b3HullData* swapHull, float swapFriction )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	if ( rec != NULL )
	{
		b3World_StartRecording( worldId, rec );
	}

	// The mesh body is created first so its ordinal is stable for the read back after replay.
	b3SurfaceMaterial meshMaterials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
	meshMaterials[1].friction = 0.05f;

	b3BodyDef meshBodyDef = b3DefaultBodyDef();
	meshBodyDef.type = b3_staticBody;
	b3BodyId meshBodyId = b3CreateBody( worldId, &meshBodyDef );

	b3ShapeDef meshShapeDef = b3DefaultShapeDef();
	meshShapeDef.materials = meshMaterials;
	meshShapeDef.materialCount = 2;
	b3ShapeId meshShapeId = b3CreateMeshShape( meshBodyId, &meshShapeDef, meshA, (b3Vec3){ 1.0f, 1.0f, 1.0f } );

	b3BodyDef boxBodyDef = b3DefaultBodyDef();
	boxBodyDef.type = b3_dynamicBody;
	boxBodyDef.position = (b3Pos){ -2.0f, 2.0f, 0.0f };
	boxBodyDef.linearVelocity = (b3Vec3){ 4.0f, 0.0f, 0.0f };
	b3BodyId boxBodyId = b3CreateBody( worldId, &boxBodyDef );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef boxShapeDef = b3DefaultShapeDef();
	boxShapeDef.density = 1.0f;
	b3ShapeId boxShapeId = b3CreateHullShape( boxBodyId, &boxShapeDef, &box.base );

	float timeStep = 1.0f / 60.0f;
	for ( int i = 0; i < 10; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	if ( mutate )
	{
		b3Shape_SetHull( boxShapeId, swapHull );
		b3Shape_SetMesh( meshShapeId, meshB, (b3Vec3){ 1.0f, 1.0f, 1.0f } );

		b3SurfaceMaterial grippy = b3DefaultSurfaceMaterial();
		grippy.friction = swapFriction;
		b3Shape_SetMeshMaterial( meshShapeId, grippy, 1 );
	}

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	uint64_t hash = b3HashWorldState( b3GetWorldFromId( worldId ) );

	if ( rec != NULL )
	{
		b3World_StopRecording( worldId );
	}
	b3DestroyWorld( worldId );

	return hash;
}

// Swapping a shape's hull or mesh, or retuning one of its per-triangle materials, is a world
// mutation like any other and has to ride the stream. The geometry pair interns into the registry
// at the record site so replay rebuilds the same shape instead of running on the geometry it was
// created with. The control run proves the mutations move the simulation, so the state hash gate
// has teeth, and the read back covers each op on its own where dynamics alone would not.
static int GeometryMutatorReplay( void )
{
	// Two flat floors with different triangulations, both carrying two material slots so the
	// material index stays live across the swap.
	b3MeshData* meshA = b3CreateGridMesh( 8, 8, 2.0f, 2, false );
	ENSURE( meshA != NULL );
	b3MeshData* meshB = b3CreateGridMesh( 12, 12, 1.5f, 2, false );
	ENSURE( meshB != NULL );
	ENSURE( meshA->triangleCount != meshB->triangleCount );

	b3BoxHull swapHull = b3MakeBoxHull( 0.25f, 1.5f, 0.25f );
	const float swapFriction = 0.95f;

	uint64_t controlHash = RunGeometryMutatorScene( NULL, false, meshA, meshB, &swapHull.base, swapFriction );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	uint64_t mutatedHash = RunGeometryMutatorScene( rec, true, meshA, meshB, &swapHull.base, swapFriction );

	// Without this the replay gate below could pass on a recording that never carried the ops.
	ENSURE( mutatedHash != controlHash );

	const uint8_t* data = b3Recording_GetData( rec );
	int size = b3Recording_GetSize( rec );
	ENSURE( size > 0 );

	ENSURE( b3ValidateReplay( data, size, 1 ) );
	ENSURE( b3ValidateReplay( data, size, 4 ) );

	b3RecPlayer* player = b3CreatePlayer( data, size, 1 );
	ENSURE( player != NULL );
	while ( b3RecPlayer_StepFrame( player ) )
	{
	}
	ENSURE( b3RecPlayer_HasDiverged( player ) == false );

	// Body ordinals follow creation order in the replayed world.
	b3BodyId replayMeshBody = b3RecPlayer_GetBodyId( player, 0 );
	b3BodyId replayBoxBody = b3RecPlayer_GetBodyId( player, 1 );
	ENSURE( b3Body_IsValid( replayMeshBody ) && b3Body_IsValid( replayBoxBody ) );

	b3ShapeId replayMeshShape;
	ENSURE( b3Body_GetShapes( replayMeshBody, &replayMeshShape, 1 ) == 1 );
	b3ShapeId replayBoxShape;
	ENSURE( b3Body_GetShapes( replayBoxBody, &replayBoxShape, 1 ) == 1 );

	const b3HullData* replayHull = b3Shape_GetHull( replayBoxShape );
	ENSURE( replayHull != NULL );
	ENSURE( replayHull->hash == swapHull.base.hash );

	b3Mesh replayMesh = b3Shape_GetMesh( replayMeshShape );
	ENSURE( replayMesh.data != NULL );
	ENSURE( replayMesh.data->hash == meshB->hash );
	ENSURE( replayMesh.data->triangleCount == meshB->triangleCount );

	b3SurfaceMaterial replayMaterial = b3Shape_GetMeshSurfaceMaterial( replayMeshShape, 1 );
	ENSURE( replayMaterial.friction == swapFriction );

	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	b3DestroyMesh( meshA );
	b3DestroyMesh( meshB );
	return 0;
}

// One unit-cube revision per occupied column of a rectangular slab.
static v3BlockGridData* CookReplaySlab( int width, int depth )
{
	v3BlockGridBox boxes[64];
	v3BlockGridBlock blocks[64];
	int count = 0;
	for ( int z = 0; z < depth; ++z )
	{
		for ( int x = 0; x < width; ++x )
		{
			boxes[count] = (v3BlockGridBox){ .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
			blocks[count] = (v3BlockGridBlock){
				.x = x,
				.z = z,
				.userData = (uint64_t)count + 1,
				.boxes = boxes + count,
				.boxCount = 1,
			};
			count += 1;
		}
	}

	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0.8f;
	v3BlockGridCookDef def = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = count,
	};
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

static v3BlockGridData* CookReplayStripedSlab( int width, int depth )
{
	v3BlockGridBox boxes[64];
	v3BlockGridBlock blocks[64];
	int count = 0;
	for ( int z = 0; z < depth; ++z )
	{
		for ( int x = 0; x < width; ++x )
		{
			boxes[count] = (v3BlockGridBox){
				.bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } },
				.materialIndex = (uint32_t)( ( x + z ) & 1 ),
			};
			blocks[count] = (v3BlockGridBlock){
				.x = x,
				.z = z,
				.userData = (uint64_t)count + 1,
				.boxes = boxes + count,
				.boxCount = 1,
			};
			count += 1;
		}
	}
	b3SurfaceMaterial materials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
	materials[0].friction = materials[1].friction = 0.8f;
	materials[0].userMaterialId = 101;
	materials[1].userMaterialId = 102;
	v3BlockGridCookDef def = {
		.materials = materials,
		.materialCount = 2,
		.blocks = blocks,
		.blockCount = count,
	};
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

static v3BlockGridData* CookReplaySeparatedGrid( int xA, int xB )
{
	v3BlockGridBox boxes[2] = {
		{ .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } },
		{ .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } },
	};
	v3BlockGridBlock blocks[2] = {
		{ .x = xA, .userData = 1, .boxes = boxes, .boxCount = 1 },
		{ .x = xB, .userData = 2, .boxes = boxes + 1, .boxCount = 1 },
	};
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	v3BlockGridCookDef def = {
		.materials = &material,
		.materialCount = 1,
		.blocks = blocks,
		.blockCount = 2,
	};
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

// A BlockGrid ship settling on BlockGrid terrain whose revision is optionally replaced part way
// through. Returns the final state hash so the caller can prove the replacement moves the
// simulation. Recording is optional so the same scene serves as the control.
static uint64_t RunBlockGridReplaceScene( b3Recording* rec, bool replace, v3BlockGridData* wideTerrain,
										  v3BlockGridData* narrowTerrain, v3BlockGridData* ship )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	if ( rec != NULL )
	{
		b3World_StartRecording( worldId, rec );
	}

	b3BodyDef terrainDef = b3DefaultBodyDef();
	terrainDef.type = b3_staticBody;
	b3BodyId terrainBodyId = b3CreateBody( worldId, &terrainDef );
	b3ShapeDef terrainShapeDef = b3DefaultShapeDef();
	b3ShapeId terrainShapeId = v3CreateBlockGridShape( terrainBodyId, &terrainShapeDef, wideTerrain );

	b3BodyDef shipDef = b3DefaultBodyDef();
	shipDef.type = b3_dynamicBody;
	shipDef.position = (b3Pos){ 1.5, 3.0, 1.5 };
	b3BodyId shipBodyId = b3CreateBody( worldId, &shipDef );
	b3ShapeDef shipShapeDef = b3DefaultShapeDef();
	shipShapeDef.density = 1.0f;
	v3CreateBlockGridShape( shipBodyId, &shipShapeDef, ship );

	float timeStep = 1.0f / 60.0f;
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	if ( replace )
	{
		v3ReplaceBlockGridShape( terrainShapeId, narrowTerrain, true );
	}

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, timeStep, 4 );
	}

	uint64_t hash = b3HashWorldState( b3GetWorldFromId( worldId ) );

	if ( rec != NULL )
	{
		b3World_StopRecording( worldId );
	}
	b3DestroyWorld( worldId );

	return hash;
}

// Record BlockGrid replacement with its interned geometry. The control run verifies
// that replacement changes the simulation, and replay must reproduce that change.
static int BlockGridReplaceReplay( void )
{
	v3BlockGridData* wideTerrain = CookReplaySlab( 6, 6 );
	v3BlockGridData* narrowTerrain = CookReplaySlab( 3, 3 );
	v3BlockGridData* ship = CookReplaySlab( 2, 2 );
	ENSURE( wideTerrain != NULL && narrowTerrain != NULL && ship != NULL );

	uint64_t controlHash = RunBlockGridReplaceScene( NULL, false, wideTerrain, narrowTerrain, ship );

	b3Recording* rec = b3CreateRecording( 0 );
	ENSURE( rec != NULL );
	uint64_t replacedHash = RunBlockGridReplaceScene( rec, true, wideTerrain, narrowTerrain, ship );

	// Verify that replacement changes the result so replay cannot pass with the operation missing.
	ENSURE( replacedHash != controlHash );

	const uint8_t* data = b3Recording_GetData( rec );
	int size = b3Recording_GetSize( rec );
	ENSURE( size > 0 );

	b3RecPlayer* player = b3CreatePlayer( data, size, 1 );
	ENSURE( player != NULL );
	while ( b3RecPlayer_StepFrame( player ) )
	{
	}
	ENSURE( b3RecPlayer_HasDiverged( player ) == false );

	uint64_t replayHash = b3HashWorldState( b3GetWorldFromId( b3RecPlayer_GetWorldId( player ) ) );
	printf( "  block grid replace hash recorded 0x%016llx replay 0x%016llx control 0x%016llx\n", (unsigned long long)replacedHash,
			(unsigned long long)replayHash, (unsigned long long)controlHash );
	ENSURE( replayHash == replacedHash );

	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	v3DestroyBlockGridData( wideTerrain );
	v3DestroyBlockGridData( narrowTerrain );
	v3DestroyBlockGridData( ship );
	return 0;
}

// Public observations are copied before their borrowed views expire. World slots
// are checked separately, then normalized while body/shape generations survive.
typedef struct ReplayEvents
{
	v3BlockContactEvent begin[256], hit[256], end[256];
	int beginCount, hitCount, endCount;
	uint32_t droppedBegin, droppedHit, droppedEnd;
	bool truncated;
	v3BlockGridPairCounters counters;
	b3ContactEndTouchEvent nativeEnd[512];
	int nativeEndCount;
} ReplayEvents;

static void NormalizeReplayEvent( v3BlockContactEvent* e )
{
	e->sideA.bodyId.world0 = e->sideA.shapeId.world0 = 0;
	e->sideB.bodyId.world0 = e->sideB.shapeId.world0 = 0;
}

static int CaptureReplayEvents( b3WorldId world, ReplayEvents* frame )
{
	v3BlockContactEvents e = v3World_GetBlockContactEvents( world );
	ENSURE( e.beginCount <= 256 && e.hitCount <= 256 && e.endCount <= 256 );
	frame->beginCount = e.beginCount;
	frame->hitCount = e.hitCount;
	frame->endCount = e.endCount;
	frame->droppedBegin = e.droppedBeginCount;
	frame->droppedHit = e.droppedHitCount;
	frame->droppedEnd = e.droppedEndCount;
	frame->truncated = e.truncated;
	const v3BlockContactEvent* sources[] = { e.beginEvents, e.hitEvents, e.endEvents };
	v3BlockContactEvent* destinations[] = { frame->begin, frame->hit, frame->end };
	int counts[] = { e.beginCount, e.hitCount, e.endCount };
	for ( int kind = 0; kind < 3; ++kind )
	{
		for ( int i = 0; i < counts[kind]; ++i )
		{
			v3BlockContactEvent event = sources[kind][i];
			ENSURE( event.sideA.shapeId.world0 == world.index1 - 1 && event.sideB.shapeId.world0 == world.index1 - 1 );
			ENSURE( event.sideA.bodyId.world0 == world.index1 - 1 && event.sideB.bodyId.world0 == world.index1 - 1 );
			NormalizeReplayEvent( &event );
			destinations[kind][i] = event;
		}
	}
	frame->counters = v3World_GetBlockGridPairCounters( world );
	b3ContactEvents native = b3World_GetContactEvents( world );
	ENSURE( native.endCount <= 512 );
	frame->nativeEndCount = native.endCount;
	for ( int i = 0; i < native.endCount; ++i )
	{
		b3ContactEndTouchEvent event = native.endEvents[i];
		ENSURE( event.shapeIdA.world0 == world.index1 - 1 && event.shapeIdB.world0 == world.index1 - 1 );
		ENSURE( event.contactId.world0 == world.index1 - 1 );
		event.shapeIdA.world0 = event.shapeIdB.world0 = event.contactId.world0 = 0;
		frame->nativeEnd[i] = event;
	}
	return 0;
}

static int CompareReplaySide( const v3BlockContactSide* a, const v3BlockContactSide* b )
{
	ENSURE( B3_ID_EQUALS( a->bodyId, b->bodyId ) && B3_ID_EQUALS( a->shapeId, b->shapeId ) );
	ENSURE( a->isBlockGrid == b->isBlockGrid );
	ENSURE( a->cellX == b->cellX && a->cellY == b->cellY && a->cellZ == b->cellZ );
	ENSURE( a->subHitboxIndex == b->subHitboxIndex && a->materialIndex == b->materialIndex );
	ENSURE( a->userMaterialId == b->userMaterialId && a->userData == b->userData );
	return 0;
}

static int CompareReplayEvents( const ReplayEvents* a, const ReplayEvents* b )
{
	ENSURE( a->beginCount == b->beginCount && a->hitCount == b->hitCount && a->endCount == b->endCount );
	ENSURE( a->droppedBegin == b->droppedBegin && a->droppedHit == b->droppedHit && a->droppedEnd == b->droppedEnd );
	ENSURE( a->truncated == b->truncated );
	const v3BlockContactEvent* left[] = { a->begin, a->hit, a->end };
	const v3BlockContactEvent* right[] = { b->begin, b->hit, b->end };
	int counts[] = { a->beginCount, a->hitCount, a->endCount };
	for ( int kind = 0; kind < 3; ++kind )
	{
		for ( int i = 0; i < counts[kind]; ++i )
		{
			const v3BlockContactEvent* x = left[kind] + i;
			const v3BlockContactEvent* y = right[kind] + i;
			ENSURE( CompareReplaySide( &x->sideA, &y->sideA ) == 0 && CompareReplaySide( &x->sideB, &y->sideB ) == 0 );
			ENSURE( x->point.x == y->point.x && x->point.y == y->point.y && x->point.z == y->point.z );
			ENSURE( x->normal.x == y->normal.x && x->normal.y == y->normal.y && x->normal.z == y->normal.z );
			ENSURE( x->normalImpulse == y->normalImpulse && x->approachSpeed == y->approachSpeed );
		}
	}
#define CHECK_COUNTER( field ) ENSURE( a->counters.field == b->counters.field )
	CHECK_COUNTER( candidateHitboxPairCount );
	CHECK_COUNTER( touchingPairCount );
	CHECK_COUNTER( contactCount );
	CHECK_COUNTER( projectileSweepCount );
	CHECK_COUNTER( capExhaustionCount );
	CHECK_COUNTER( replacementPublishedCount );
	CHECK_COUNTER( scratchPeakBytes );
	CHECK_COUNTER( contactReductionCount );
#undef CHECK_COUNTER
	ENSURE( a->nativeEndCount == b->nativeEndCount );
	for ( int i = 0; i < a->nativeEndCount; ++i )
	{
		ENSURE( B3_ID_EQUALS( a->nativeEnd[i].shapeIdA, b->nativeEnd[i].shapeIdA ) );
		ENSURE( B3_ID_EQUALS( a->nativeEnd[i].shapeIdB, b->nativeEnd[i].shapeIdB ) );
		ENSURE( B3_ID_EQUALS( a->nativeEnd[i].contactId, b->nativeEnd[i].contactId ) );
	}
	return 0;
}

typedef struct ReplacementCapacityView
{
	uint64_t pairBytes;
	uint32_t pairCapacityBytes;
	int manifoldCount;
	int nativeBeginCapacity;
	int nativeEndCapacities[2];
	int awakeContactCapacity;
	int graphConvexCapacities[B3_GRAPH_COLOR_COUNT];
	int graphContactCapacities[B3_GRAPH_COLOR_COUNT];
	int moveCapacity;
	int moveCount;
	int treeRebuildCapacities[b3_bodyTypeCount];
	uint8_t treeScratchMasks[b3_bodyTypeCount];
	int taskCount;
	int arenaCapacities[B3_MAX_WORKERS];
	int arenaOverflowCapacities[B3_MAX_WORKERS];
	int arenaPeaks[B3_MAX_WORKERS];
	int islandSlotCount;
	int islandContactCapacities[64];
	int islandContactCounts[64];
} ReplacementCapacityView;

static ReplacementCapacityView CaptureReplacementCapacities( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	ReplacementCapacityView view = { 0 };
	view.pairBytes = world->blockGridPairBytes;
	view.nativeBeginCapacity = world->contactBeginEvents.capacity;
	view.nativeEndCapacities[0] = world->contactEndEvents[0].capacity;
	view.nativeEndCapacities[1] = world->contactEndEvents[1].capacity;
	view.awakeContactCapacity = world->solverSets.data[b3_awakeSet].contactIndices.capacity;
	view.moveCapacity = world->broadPhase.moveArray.capacity;
	view.moveCount = world->broadPhase.moveArray.count;
	for ( int type = 0; type < b3_bodyTypeCount; ++type )
	{
		const b3DynamicTree* tree = world->broadPhase.trees + type;
		view.treeRebuildCapacities[type] = tree->rebuildCapacity;
		view.treeScratchMasks[type] = ( tree->leafIndices != NULL ? 1u : 0u ) | ( tree->leafBoxes != NULL ? 2u : 0u ) |
									  ( tree->leafCenters != NULL ? 4u : 0u ) | ( tree->binIndices != NULL ? 8u : 0u );
	}
	view.taskCount = world->taskContexts.count;
	for ( int i = 0; i < world->taskContexts.count && i < B3_MAX_WORKERS; ++i )
	{
		const b3Arena* arena = &world->taskContexts.data[i].arena;
		view.arenaCapacities[i] = arena->capacity;
		view.arenaOverflowCapacities[i] = arena->shared->overflows.capacity;
		view.arenaPeaks[i] = arena->shared->peakDemand;
	}
	view.islandSlotCount = world->islands.count <= ARRAY_COUNT( view.islandContactCapacities ) ? world->islands.count : -1;
	for ( int i = 0; i < view.islandSlotCount; ++i )
	{
		view.islandContactCapacities[i] = world->islands.data[i].contacts.capacity;
		view.islandContactCounts[i] = world->islands.data[i].contacts.count;
	}
	for ( int color = 0; color < B3_GRAPH_COLOR_COUNT; ++color )
	{
		view.graphConvexCapacities[color] = world->constraintGraph.colors[color].convexContacts.capacity;
		view.graphContactCapacities[color] = world->constraintGraph.colors[color].contacts.capacity;
	}
	for ( int i = 0; i < world->contacts.count; ++i )
	{
		b3Contact* contact = world->contacts.data + i;
		if ( contact->contactId == i && contact->kind == v3_blockGridPairContactKind )
		{
			view.pairCapacityBytes = v3BlockGridPairCapacityBytes( contact->blockGridPair.state );
			view.manifoldCount = v3BlockGridPairManifoldCount( contact->blockGridPair.state );
			break;
		}
	}
	return view;
}

static int CompareReplacementCapacities( const ReplacementCapacityView* a, const ReplacementCapacityView* b )
{
	ENSURE( a->pairBytes == b->pairBytes && a->pairCapacityBytes == b->pairCapacityBytes );
	ENSURE( a->manifoldCount == b->manifoldCount );
	ENSURE( a->nativeBeginCapacity == b->nativeBeginCapacity );
	ENSURE( a->nativeEndCapacities[0] == b->nativeEndCapacities[0] && a->nativeEndCapacities[1] == b->nativeEndCapacities[1] );
	ENSURE( a->awakeContactCapacity == b->awakeContactCapacity );
	ENSURE( a->moveCapacity == b->moveCapacity && a->moveCount == b->moveCount );
	for ( int type = 0; type < b3_bodyTypeCount; ++type )
	{
		ENSURE( a->treeRebuildCapacities[type] == b->treeRebuildCapacities[type] );
		ENSURE( a->treeScratchMasks[type] == b->treeScratchMasks[type] );
	}
	ENSURE( a->taskCount == b->taskCount );
	for ( int i = 0; i < a->taskCount; ++i )
	{
		ENSURE( a->arenaCapacities[i] == b->arenaCapacities[i] );
		ENSURE( a->arenaOverflowCapacities[i] == b->arenaOverflowCapacities[i] );
		ENSURE( a->arenaPeaks[i] == b->arenaPeaks[i] );
	}
	ENSURE( a->islandSlotCount >= 0 && a->islandSlotCount == b->islandSlotCount );
	for ( int i = 0; i < a->islandSlotCount; ++i )
	{
		ENSURE( a->islandContactCapacities[i] == b->islandContactCapacities[i] );
		ENSURE( a->islandContactCounts[i] == b->islandContactCounts[i] );
	}
	for ( int color = 0; color < B3_GRAPH_COLOR_COUNT; ++color )
	{
		ENSURE( a->graphConvexCapacities[color] == b->graphConvexCapacities[color] );
		ENSURE( a->graphContactCapacities[color] == b->graphContactCapacities[color] );
	}
	return 0;
}

static int GrowReplacementReplayCapacities( b3World* world, const ReplacementCapacityView* seed )
{
	b3Array_Reserve( world->contactBeginEvents, world->contactBeginEvents.capacity + 1 );
	for ( int i = 0; i < 2; ++i )
	{
		b3Array_Reserve( world->contactEndEvents[i], world->contactEndEvents[i].capacity + 1 );
	}
	b3Array_Reserve( world->solverSets.data[b3_awakeSet].contactIndices,
					 world->solverSets.data[b3_awakeSet].contactIndices.capacity + 1 );
	b3Array_Reserve( world->broadPhase.moveArray, world->broadPhase.moveArray.capacity + 1 );

	b3DynamicTree* tree = world->broadPhase.trees + b3_dynamicBody;
	int treeCapacity = tree->rebuildCapacity + 1;
	b3Free( tree->leafIndices, (size_t)tree->rebuildCapacity * sizeof( int ) );
	b3Free( tree->leafBoxes, (size_t)tree->rebuildCapacity * sizeof( b3AABB ) );
	b3Free( tree->leafCenters, (size_t)tree->rebuildCapacity * sizeof( b3Vec3 ) );
	b3Free( tree->binIndices, (size_t)tree->rebuildCapacity * sizeof( int ) );
	tree->leafIndices = b3Alloc( (size_t)treeCapacity * sizeof( int ) );
	tree->leafBoxes = NULL;
	tree->leafCenters = b3Alloc( (size_t)treeCapacity * sizeof( b3Vec3 ) );
	tree->binIndices = NULL;
	tree->rebuildCapacity = treeCapacity;

	for ( int i = 0; i < world->taskContexts.count; ++i )
	{
		b3Arena* arena = &world->taskContexts.data[i].arena;
		int arenaCapacity = arena->capacity + 1;
		int overflowCapacity = arena->shared->overflows.capacity + 1;
		b3DestroyArena( arena );
		*arena = b3CreateArena( arenaCapacity );
		b3Array_Reserve( arena->shared->overflows, overflowCapacity );
	}

	for ( int i = 0; i < world->islands.count; ++i )
	{
		b3Island* island = world->islands.data + i;
		if ( island->islandId == i )
		{
			b3Array_Reserve( island->contacts, island->contacts.capacity + 1 );
		}
	}
	for ( int color = 0; color < B3_GRAPH_COLOR_COUNT; ++color )
	{
		b3GraphColor* graphColor = world->constraintGraph.colors + color;
		b3Array_Reserve( graphColor->convexContacts, graphColor->convexContacts.capacity + 1 );
		b3Array_Reserve( graphColor->contacts, graphColor->contacts.capacity + 1 );
	}

	ReplacementCapacityView grown = CaptureReplacementCapacities( (b3WorldId){ world->worldId + 1, world->generation } );
	ENSURE( grown.nativeBeginCapacity > seed->nativeBeginCapacity );
	ENSURE( grown.nativeEndCapacities[0] > seed->nativeEndCapacities[0] &&
			grown.nativeEndCapacities[1] > seed->nativeEndCapacities[1] );
	ENSURE( grown.awakeContactCapacity > seed->awakeContactCapacity && grown.moveCapacity > seed->moveCapacity );
	ENSURE( grown.treeRebuildCapacities[b3_dynamicBody] > seed->treeRebuildCapacities[b3_dynamicBody] );
	ENSURE( grown.arenaCapacities[0] > seed->arenaCapacities[0] &&
			grown.arenaOverflowCapacities[0] > seed->arenaOverflowCapacities[0] );
	bool grewIsland = false;
	for ( int i = 0; i < grown.islandSlotCount; ++i )
	{
		grewIsland = grewIsland || grown.islandContactCapacities[i] > seed->islandContactCapacities[i];
	}
	ENSURE( grewIsland );
	for ( int color = 0; color < B3_GRAPH_COLOR_COUNT; ++color )
	{
		ENSURE( grown.graphConvexCapacities[color] > seed->graphConvexCapacities[color] );
		ENSURE( grown.graphContactCapacities[color] > seed->graphContactCapacities[color] );
	}
	return 0;
}

static b3ContactId RecordingTouchingContactId( b3ShapeId shapeId, b3ShapeId otherShapeId )
{
	b3ContactData contacts[16];
	int count = b3Shape_GetContactData( shapeId, contacts, ARRAY_COUNT( contacts ) );
	for ( int i = 0; i < count; ++i )
	{
		bool samePair = ( B3_ID_EQUALS( contacts[i].shapeIdA, shapeId ) && B3_ID_EQUALS( contacts[i].shapeIdB, otherShapeId ) ) ||
						( B3_ID_EQUALS( contacts[i].shapeIdA, otherShapeId ) && B3_ID_EQUALS( contacts[i].shapeIdB, shapeId ) );
		if ( samePair && contacts[i].manifoldCount > 0 )
		{
			return contacts[i].contactId;
		}
	}
	return b3_nullContactId;
}

static int replacementReplayAllocationCount;
static int replacementReplayFreeCount;

static void* ReplacementReplayAlloc( int size, int alignment )
{
	replacementReplayAllocationCount += 1;
#if defined( _WIN32 )
	return _aligned_malloc( size, alignment );
#else
	return aligned_alloc( alignment, size );
#endif
}

static void ReplacementReplayFree( void* memory )
{
	replacementReplayFreeCount += 1;
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

static void BeginReplacementReplayMeasurement( void )
{
	replacementReplayAllocationCount = 0;
	replacementReplayFreeCount = 0;
	b3SetAllocator( ReplacementReplayAlloc, ReplacementReplayFree );
}

static void EndReplacementReplayMeasurement( void )
{
	b3SetAllocator( NULL, NULL );
}

enum
{
	replacementMoveCapacityMarker = 50021,
	replacementTreeCapacityMarker = 50023,
	replacementArenaCapacityMarker = 8000039,
	replacementOverflowCapacityMarker = 50047,
	replacementIslandCapacityMarker = 50051,
};

static int PrepareReplacementCapacityMarkers( b3World* world )
{
	b3Array_Reserve( world->broadPhase.moveArray, replacementMoveCapacityMarker );

	b3DynamicTree* tree = world->broadPhase.trees + b3_dynamicBody;
	b3Free( tree->leafIndices, tree->rebuildCapacity * (int)sizeof( int ) );
	b3Free( tree->leafBoxes, tree->rebuildCapacity * (int)sizeof( b3AABB ) );
	b3Free( tree->leafCenters, tree->rebuildCapacity * (int)sizeof( b3Vec3 ) );
	b3Free( tree->binIndices, tree->rebuildCapacity * (int)sizeof( int ) );
	tree->leafIndices = b3Alloc( replacementTreeCapacityMarker * (int)sizeof( int ) );
	tree->leafBoxes = NULL;
	tree->leafCenters = b3Alloc( replacementTreeCapacityMarker * (int)sizeof( b3Vec3 ) );
	tree->binIndices = NULL;
	tree->rebuildCapacity = replacementTreeCapacityMarker;

	b3Arena* arena = &world->taskContexts.data[0].arena;
	b3DestroyArena( arena );
	*arena = b3CreateArena( replacementArenaCapacityMarker );
	b3Array_Reserve( arena->shared->overflows, replacementOverflowCapacityMarker );
	arena->shared->peakDemand = 1234;

	for ( int i = 0; i < world->islands.count; ++i )
	{
		b3Island* island = world->islands.data + i;
		if ( island->islandId == i )
		{
			b3Array_Reserve( island->contacts, replacementIslandCapacityMarker );
			return 0;
		}
	}
	return 1;
}

static int FindSnapshotMarker( const b3Recording* recording, int marker )
{
	const uint8_t* data = b3Recording_GetData( recording );
	b3RecHeader header;
	memcpy( &header, data, sizeof( header ) );
	int begin = (int)sizeof( header );
	int end = begin + (int)header.snapshotSize;
	int found = -1;
	for ( int offset = begin; offset + (int)sizeof( marker ) <= end; ++offset )
	{
		int value;
		memcpy( &value, data + offset, sizeof( value ) );
		if ( value == marker )
		{
			ENSURE( found == -1 );
			found = offset;
		}
	}
	ENSURE( found >= begin );
	return found;
}

static int replacementLargeScratchAllocationCount;

static void* ReplacementScratchBoundaryAlloc( int32_t size, int32_t alignment )
{
	if ( size > 64 * 1024 * 1024 )
	{
		replacementLargeScratchAllocationCount += 1;
		size = alignment;
	}
#if defined( _WIN32 )
	return _aligned_malloc( (size_t)size, (size_t)alignment );
#else
	return aligned_alloc( (size_t)alignment, (size_t)size );
#endif
}

static void ReplacementScratchBoundaryFree( void* memory )
{
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

// A recorded tree scratch capacity may be larger than the inactive scratch element types. Restore
// it twice with bounded test allocations so the cleanup size arithmetic is exercised without
// reserving gigabytes.
static int CheckTreeScratchCleanupBoundary( const b3Recording* recording )
{
	int size = b3Recording_GetSize( recording );
	const uint8_t* original = b3Recording_GetData( recording );
	uint8_t* data = b3Alloc( size );
	memcpy( data, original, size );

	int offset = FindSnapshotMarker( recording, replacementTreeCapacityMarker );
	int boundaryCapacity = INT_MAX / (int)sizeof( b3AABB ) + 1;
	ENSURE( boundaryCapacity <= INT_MAX / (int)sizeof( b3Vec3 ) );
	memcpy( data + offset, &boundaryCapacity, sizeof( boundaryCapacity ) );

	int allocationBaseline = b3GetByteCount();
	replacementLargeScratchAllocationCount = 0;
	b3SetAllocator( ReplacementScratchBoundaryAlloc, ReplacementScratchBoundaryFree );
	b3RecPlayer* player = b3CreatePlayer( data, size, 1 );
	if ( player != NULL )
	{
		b3RecPlayer_Restart( player );
		b3DestroyPlayer( player );
	}
	b3SetAllocator( NULL, NULL );

	int largeAllocationCount = replacementLargeScratchAllocationCount;
	b3Free( data, size );
	ENSURE( player != NULL && largeAllocationCount >= 4 );
	ENSURE( b3GetByteCount() == allocationBaseline - size );
	return 0;
}

static int CheckMalformedReplacementCapacities( const b3Recording* recording )
{
	int size = b3Recording_GetSize( recording );
	const uint8_t* original = b3Recording_GetData( recording );
	uint8_t* data = b3Alloc( size );
	int markers[] = { replacementMoveCapacityMarker, replacementTreeCapacityMarker, replacementArenaCapacityMarker,
					  replacementOverflowCapacityMarker, replacementIslandCapacityMarker };
	int offsets[ARRAY_COUNT( markers )];
	for ( int i = 0; i < ARRAY_COUNT( markers ); ++i )
	{
		offsets[i] = FindSnapshotMarker( recording, markers[i] );
	}
	int allocationBaseline = b3GetByteCount();

	for ( int i = 0; i < ARRAY_COUNT( markers ); ++i )
	{
		memcpy( data, original, size );
		int invalid = -1;
		memcpy( data + offsets[i], &invalid, sizeof( invalid ) );
		ENSURE( b3CreatePlayer( data, size, 1 ) == NULL );
		ENSURE( b3GetByteCount() == allocationBaseline );
	}

	int capacityCountMarkers[] = { replacementMoveCapacityMarker, replacementIslandCapacityMarker };
	for ( int i = 0; i < ARRAY_COUNT( capacityCountMarkers ); ++i )
	{
		memcpy( data, original, size );
		int offset = FindSnapshotMarker( recording, capacityCountMarkers[i] );
		int invalidCount = capacityCountMarkers[i] + 1;
		memcpy( data + offset + (int)sizeof( int ), &invalidCount, sizeof( invalidCount ) );
		ENSURE( b3CreatePlayer( data, size, 1 ) == NULL );
		ENSURE( b3GetByteCount() == allocationBaseline );
	}

	memcpy( data, original, size );
	int invalidPeak = replacementArenaCapacityMarker + 1;
	memcpy( data + offsets[2] + 2 * (int)sizeof( int ), &invalidPeak, sizeof( invalidPeak ) );
	ENSURE( b3CreatePlayer( data, size, 1 ) == NULL );
	ENSURE( b3GetByteCount() == allocationBaseline );

	memcpy( data, original, size );
	b3RecHeader truncated;
	memcpy( &truncated, data, sizeof( truncated ) );
	truncated.snapshotSize = (uint64_t)( offsets[3] - (int)sizeof( truncated ) + 2 );
	memcpy( data, &truncated, sizeof( truncated ) );
	ENSURE( b3CreatePlayer( data, size, 1 ) == NULL );
	ENSURE( b3GetByteCount() == allocationBaseline );

	b3Free( data, size );
	return 0;
}

typedef struct ReplayMotion
{
	b3Pos position;
	b3Quat rotation;
	b3Vec3 linearVelocity, angularVelocity;
} ReplayMotion;

static ReplayMotion CaptureReplayMotion( b3BodyId body )
{
	return (ReplayMotion){ b3Body_GetPosition( body ), b3Body_GetRotation( body ), b3Body_GetLinearVelocity( body ),
						   b3Body_GetAngularVelocity( body ) };
}

static int CompareReplayMotion( ReplayMotion a, ReplayMotion b )
{
	ENSURE( a.position.x == b.position.x && a.position.y == b.position.y && a.position.z == b.position.z );
	ENSURE( a.rotation.v.x == b.rotation.v.x && a.rotation.v.y == b.rotation.v.y && a.rotation.v.z == b.rotation.v.z &&
			a.rotation.s == b.rotation.s );
	ENSURE( a.linearVelocity.x == b.linearVelocity.x && a.linearVelocity.y == b.linearVelocity.y &&
			a.linearVelocity.z == b.linearVelocity.z );
	ENSURE( a.angularVelocity.x == b.angularVelocity.x && a.angularVelocity.y == b.angularVelocity.y &&
			a.angularVelocity.z == b.angularVelocity.z );
	return 0;
}

// Recording begins while the retained pair is empty and an old end is pending. The next live and
// replay steps must use the same warmed capacities, event identities and physical result.
static int BlockGridReplacementCapacityReplay( void )
{
	int bytes = b3GetByteCount();
	v3BlockGridData* stripedTerrain = CookReplayStripedSlab( 6, 6 );
	v3BlockGridData* plainTerrain = CookReplaySlab( 6, 6 );
	v3BlockGridData* shipGrid = CookReplaySlab( 3, 3 );
	ENSURE( stripedTerrain != NULL && plainTerrain != NULL && shipGrid != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	worldDef.enableSleep = false;
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableContactEvents = true;
	b3ShapeId terrainShape = v3CreateBlockGridShape( b3CreateBody( worldId, &bodyDef ), &shapeDef, stripedTerrain );
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 1.5, 2.0, 1.5 };
	shapeDef.density = 1.0f;
	b3BodyId shipBody = b3CreateBody( worldId, &bodyDef );
	b3ShapeId shipShape = v3CreateBlockGridShape( shipBody, &shapeDef, shipGrid );
	for ( int i = 0; i < 180; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3ContactId oldContact = RecordingTouchingContactId( shipShape, terrainShape );
	ReplacementCapacityView warm = CaptureReplacementCapacities( worldId );
	ENSURE( B3_IS_NON_NULL( oldContact ) && b3Contact_IsValid( oldContact ) && warm.manifoldCount > 1 );
	ENSURE( v3ReplaceBlockGridShape( terrainShape, plainTerrain, true ) == v3_blockGridReplaceOk );
	ENSURE( b3Contact_IsValid( oldContact ) == false );
	ENSURE( PrepareReplacementCapacityMarkers( b3GetWorldFromId( worldId ) ) == 0 );
	ReplacementCapacityView seed = CaptureReplacementCapacities( worldId );
	ENSURE( seed.manifoldCount == 0 && seed.pairCapacityBytes == warm.pairCapacityBytes && seed.pairBytes == warm.pairBytes );

	b3Recording* recording = b3CreateRecording( 1024 * 1024 );
	b3World_StartRecording( worldId, recording );
	ENSURE( b3Recording_GetSize( recording ) > 0 );
	BeginReplacementReplayMeasurement();
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	EndReplacementReplayMeasurement();
	printf( "  live replacement replay allocations/frees %d/%d\n", replacementReplayAllocationCount, replacementReplayFreeCount );
	ENSURE( replacementReplayAllocationCount == 0 && replacementReplayFreeCount == 0 );
	b3World_StopRecording( worldId );

	ReplayEvents expectedEvents;
	ENSURE( CaptureReplayEvents( worldId, &expectedEvents ) == 0 );
	b3ContactEvents native = b3World_GetContactEvents( worldId );
	b3ContactId newContact = RecordingTouchingContactId( shipShape, terrainShape );
	ENSURE( native.beginCount == 1 && native.endCount == 1 );
	ENSURE( B3_ID_EQUALS( native.endEvents[0].contactId, oldContact ) );
	ENSURE( B3_ID_EQUALS( native.beginEvents[0].contactId, newContact ) );
	ENSURE( newContact.index1 == oldContact.index1 && newContact.generation != oldContact.generation );
	ENSURE( expectedEvents.beginCount > 0 && expectedEvents.endCount > 0 && expectedEvents.truncated == false );
	for ( int i = 0; i < expectedEvents.endCount; ++i )
	{
		const v3BlockContactSide* side =
			expectedEvents.end[i].sideA.isBlockGrid ? &expectedEvents.end[i].sideA : &expectedEvents.end[i].sideB;
		ENSURE( side->userMaterialId == 101 || side->userMaterialId == 102 );
	}
	for ( int i = 0; i < expectedEvents.beginCount; ++i )
	{
		const v3BlockContactSide* side =
			expectedEvents.begin[i].sideA.isBlockGrid ? &expectedEvents.begin[i].sideA : &expectedEvents.begin[i].sideB;
		ENSURE( side->userMaterialId == 0 );
	}
	ReplacementCapacityView expectedCapacity = CaptureReplacementCapacities( worldId );
	ReplayMotion expectedMotion = CaptureReplayMotion( shipBody );
	ENSURE( expectedCapacity.manifoldCount > 0 && expectedCapacity.manifoldCount < warm.manifoldCount );
	b3World* liveWorld = b3GetWorldFromId( worldId );
	v3BlockGridPairState* capacityProbe =
		v3BlockGridPairTryRestore( liveWorld, expectedCapacity.pairCapacityBytes, expectedCapacity.manifoldCount + 1 );
	ENSURE( capacityProbe != NULL && v3BlockGridPairFreeState( liveWorld, capacityProbe ) );

	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
	ENSURE( player != NULL );
	ReplacementCapacityView replaySeed = CaptureReplacementCapacities( b3RecPlayer_GetWorldId( player ) );
	ENSURE( CompareReplacementCapacities( &seed, &replaySeed ) == 0 );
	for ( int repeat = 0; repeat < 3; ++repeat )
	{
		if ( repeat > 0 )
		{
			if ( repeat == 1 )
			{
				b3World* replayWorld = b3GetWorldFromId( b3RecPlayer_GetWorldId( player ) );
				ENSURE( GrowReplacementReplayCapacities( replayWorld, &seed ) == 0 );
			}
			b3RecPlayer_SeekFrame( player, 0 );
			ReplayEvents pending;
			ENSURE( CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &pending ) == 0 );
			ReplacementCapacityView restoredSeed = CaptureReplacementCapacities( b3RecPlayer_GetWorldId( player ) );
			ENSURE( CompareReplacementCapacities( &seed, &restoredSeed ) == 0 );
		}
		BeginReplacementReplayMeasurement();
		ENSURE( b3RecPlayer_StepFrame( player ) );
		EndReplacementReplayMeasurement();
		printf( "  replay replacement %d allocations/frees %d/%d\n", repeat, replacementReplayAllocationCount,
				replacementReplayFreeCount );
		ENSURE( replacementReplayAllocationCount == 0 && replacementReplayFreeCount == 0 );
		ReplayEvents actualEvents;
		ENSURE( CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actualEvents ) == 0 );
		ENSURE( CompareReplayEvents( &expectedEvents, &actualEvents ) == 0 );
		ReplacementCapacityView actualCapacity = CaptureReplacementCapacities( b3RecPlayer_GetWorldId( player ) );
		ENSURE( CompareReplacementCapacities( &expectedCapacity, &actualCapacity ) == 0 );
		ENSURE( CompareReplayMotion( expectedMotion, CaptureReplayMotion( b3RecPlayer_GetBodyId( player, 1 ) ) ) == 0 );
		ENSURE( b3RecPlayer_HasDiverged( player ) == false );
	}

	b3DestroyPlayer( player );
	ENSURE( CheckTreeScratchCleanupBoundary( recording ) == 0 );
	ENSURE( CheckMalformedReplacementCapacities( recording ) == 0 );
	b3DestroyRecording( recording );
	b3DestroyWorld( worldId );
	v3DestroyBlockGridData( stripedTerrain );
	v3DestroyBlockGridData( plainTerrain );
	v3DestroyBlockGridData( shipGrid );
	ENSURE( b3GetByteCount() == bytes );
	return 0;
}

// The aggregate bounds overlap while the occupied cells do not, so the broad phase owns a real
// potential pair that never touches. Replacement must clear its transient solver indices before a
// recording captures the retained empty pair.
static int BlockGridNonTouchingReplacementSeed( void )
{
	int bytes = b3GetByteCount();
	v3BlockGridData* outer = CookReplaySeparatedGrid( 0, 10 );
	v3BlockGridData* inner = CookReplaySeparatedGrid( 4, 6 );
	ENSURE( outer != NULL && inner != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.enableSleep = false;
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId outerShape = v3CreateBlockGridShape( b3CreateBody( worldId, &bodyDef ), &shapeDef, outer );
	bodyDef.type = b3_dynamicBody;
	b3ShapeId innerShape = v3CreateBlockGridShape( b3CreateBody( worldId, &bodyDef ), &shapeDef, inner );
	ENSURE( B3_IS_NON_NULL( outerShape ) && B3_IS_NON_NULL( innerShape ) );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b3Shape_GetContactCapacity( outerShape ) == 1 && b3Shape_GetContactCapacity( innerShape ) == 1 );
	b3ContactData touching[1];
	ENSURE( b3Shape_GetContactData( outerShape, touching, 1 ) == 0 );
	ENSURE( v3ReplaceBlockGridShape( outerShape, outer, true ) == v3_blockGridReplaceOk );
	ENSURE( b3Shape_GetContactCapacity( outerShape ) == 1 && b3Shape_GetContactData( outerShape, touching, 1 ) == 0 );

	b3Recording* recording = b3CreateRecording( 0 );
	b3World_StartRecording( worldId, recording );
	ENSURE( b3Recording_GetSize( recording ) > 0 );
	b3World_StopRecording( worldId );
	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
	ENSURE( player != NULL );
	b3ShapeId replayOuterShape;
	ENSURE( b3Body_GetShapes( b3RecPlayer_GetBodyId( player, 0 ), &replayOuterShape, 1 ) == 1 );
	ENSURE( b3Shape_GetContactCapacity( replayOuterShape ) == 1 );

	b3DestroyPlayer( player );
	b3DestroyRecording( recording );
	b3DestroyWorld( worldId );
	v3DestroyBlockGridData( outer );
	v3DestroyBlockGridData( inner );
	ENSURE( b3GetByteCount() == bytes );
	return 0;
}

static v3BlockGridData* CookRecordingEventSlab( void )
{
	v3BlockGridBox box = { .bounds = { { 0, 0, 0 }, { 1, 1, 1 } } };
	v3BlockGridBlock blocks[4];
	for ( int i = 0; i < 4; ++i )
	{
		blocks[i] = (v3BlockGridBlock){ .x = i - 2, .userData = 991, .boxes = &box, .boxCount = 1 };
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	material.friction = 0;
	material.userMaterialId = 73;
	v3BlockGridCookDef cook = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = 4 };
	return v3CookBlockGrid( &cook ).data;
}

static uint64_t HashReplayMotion( uint64_t hash, ReplayMotion motion )
{
	hash = b3FnvMixPosition( hash, motion.position );
	float scalars[] = { motion.rotation.v.x,	  motion.rotation.v.y,	   motion.rotation.v.z,		motion.rotation.s,
						motion.linearVelocity.x,  motion.linearVelocity.y, motion.linearVelocity.z, motion.angularVelocity.x,
						motion.angularVelocity.y, motion.angularVelocity.z };
	for ( int i = 0; i < ARRAY_COUNT( scalars ); ++i )
	{
		uint32_t bits;
		memcpy( &bits, scalars + i, sizeof( bits ) );
		hash = ( hash ^ bits ) * B3_SNAP_FNV_PRIME;
	}
	return hash;
}

// Both seed fixtures have one pending BlockGrid end and one ordinary end, with
// no published events or active history. Change only a cached payload or the
// pending replacement count, so the replay itself produces the mismatching output.
static int CheckPendingDigestMismatch( b3Recording* rec, bool replacement )
{
	int size = b3Recording_GetSize( rec );
	const uint8_t* original = b3Recording_GetData( rec );
	uint8_t* data = b3Alloc( size );
	memcpy( data, original, size );
	b3RecHeader header;
	memcpy( &header, data, sizeof( header ) );
	int eventSize = 118 + 3 * (int)sizeof( ( (v3BlockContactEvent*)0 )->point.x );
	int section = sizeof( header ) + (int)header.snapshotSize - ( 118 + eventSize + 32 );
	int offset = replacement ? 2 : 102 + 41;
	uint64_t value;
	memcpy( &value, data + section + offset, sizeof( value ) );
	ENSURE( value == ( replacement ? 1u : 991u ) );
	value += 1;
	memcpy( data + section + offset, &value, sizeof( value ) );
	b3RecPlayer* control = b3CreatePlayer( original, size, 1 );
	b3RecPlayer* changed = b3CreatePlayer( data, size, 1 );
	ENSURE( control != NULL && changed != NULL );
	ENSURE( b3RecPlayer_StepFrame( control ) && b3RecPlayer_StepFrame( changed ) );
	b3WorldId a = b3RecPlayer_GetWorldId( control );
	b3WorldId b = b3RecPlayer_GetWorldId( changed );
	ENSURE( b3RecPlayer_GetBodyCount( control ) == b3RecPlayer_GetBodyCount( changed ) );
	for ( int i = 0; i < b3RecPlayer_GetBodyCount( control ); ++i )
	{
		ENSURE( CompareReplayMotion( CaptureReplayMotion( b3RecPlayer_GetBodyId( control, i ) ),
									 CaptureReplayMotion( b3RecPlayer_GetBodyId( changed, i ) ) ) == 0 );
	}
	ReplayEvents expected, actual;
	ENSURE( CaptureReplayEvents( a, &expected ) == 0 && CaptureReplayEvents( b, &actual ) == 0 );
	if ( replacement )
	{
		ENSURE( expected.counters.replacementPublishedCount == 1 && actual.counters.replacementPublishedCount == 2 );
		actual.counters.replacementPublishedCount = expected.counters.replacementPublishedCount;
	}
	else
	{
		ENSURE( expected.endCount == 1 && actual.endCount == 1 );
		v3BlockContactSide* side = actual.end[0].sideA.isBlockGrid ? &actual.end[0].sideA : &actual.end[0].sideB;
		ENSURE( side->userData == 992 );
		side->userData = 991;
	}
	// After accounting for the one deliberately changed observation, every other
	// public event field and counter must agree exactly.
	ENSURE( CompareReplayEvents( &expected, &actual ) == 0 );
	ENSURE( !b3RecPlayer_HasDiverged( control ) );
	ENSURE( b3RecPlayer_HasDiverged( changed ) && b3RecPlayer_GetDivergeFrame( changed ) == 1 );
	ENSURE( b3RecPlayer_StepFrame( changed ) );
	ENSURE( b3RecPlayer_HasDiverged( changed ) && b3RecPlayer_GetDivergeFrame( changed ) == 1 );
	b3DestroyPlayer( control );
	b3DestroyPlayer( changed );
	b3Free( data, size );
	return 0;
}

static int CheckRecordingEventSeed( bool settled, bool replace, bool destroy )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = (b3Vec3){ 0, -10, 0 };
	wd.enableSleep = false;
	wd.hitEventThreshold = 0.1f;
	wd.workerCount = 1;
	b3WorldId world = b3CreateWorld( &wd );
	v3BlockGridData* grid = CookRecordingEventSlab();
	ENSURE( grid != NULL );
	b3BodyDef bd = b3DefaultBodyDef();
	b3BodyId ground = b3CreateBody( world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.enableContactEvents = sd.enableHitEvents = true;
	sd.baseMaterial.friction = 0;
	b3ShapeId terrain = v3CreateBlockGridShape( ground, &sd, grid );
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ -0.25, 2, 0.5 };
	b3BodyId body = b3CreateBody( world, &bd );
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3ShapeId shape = b3CreateSphereShape( body, &sd, &sphere );
	int warmBegins = 0;
	for ( int i = 0; i < ( settled ? 120 : 0 ); ++i )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		warmBegins += v3World_GetBlockContactEvents( world ).beginCount;
	}
	ENSURE( !settled || ( warmBegins == 1 && b3Body_GetPosition( body ).y > 1.19 && b3Body_GetPosition( body ).y < 1.21 ) );
	if ( replace )
	{
		v3BlockGridData* revision = CookRecordingEventSlab();
		v3ReplaceBlockGridShape( terrain, revision, true );
		v3DestroyBlockGridData( revision );
	}
	if ( destroy )
	{
		b3DestroyShape( terrain, true );
		ENSURE( !b3Shape_IsValid( terrain ) );
		b3Sphere distant = { { 100, 0, 0 }, 0.2f };
		b3ShapeId reused = b3CreateSphereShape( ground, &sd, &distant );
		ENSURE( reused.index1 == terrain.index1 && reused.generation != terrain.generation );
	}
	b3Recording* rec = b3CreateRecording( 0 );
	b3World_StartRecording( world, rec );
	ReplayEvents* trace = b3Alloc( 121 * sizeof( ReplayEvents ) );
	int result = CaptureReplayEvents( world, trace );
	int begins = 0, hits = 0, ends = 0;
	for ( int i = 0; i < 120; ++i )
	{
		if ( i == 5 && !replace && !destroy && settled )
			b3Body_SetLinearVelocity( body, (b3Vec3){ 0, 5, 0 } );
		b3World_Step( world, 1.0f / 60.0f, 4 );
		result |= CaptureReplayEvents( world, trace + i + 1 );
		begins += trace[i + 1].beginCount;
		hits += trace[i + 1].hitCount;
		ends += trace[i + 1].endCount;
	}
	b3World_StopRecording( world );
	if ( replace || destroy )
		ENSURE( CheckPendingDigestMismatch( rec, replace ) == 0 );
	ENSURE( begins >= ( destroy ? 0 : 1 ) && ends >= ( settled ? 1 : 0 ) && ( destroy || replace || hits == 1 ) );
	if ( replace )
		ENSURE( trace[1].counters.replacementPublishedCount == 1 && trace[2].counters.replacementPublishedCount == 0 );
	if ( destroy || replace )
	{
		ENSURE( trace[1].endCount == 1 && trace[1].nativeEndCount == 1 );
		const v3BlockContactEvent* e = trace[1].end;
		const v3BlockContactSide* g = e->sideA.isBlockGrid ? &e->sideA : &e->sideB;
		const v3BlockContactSide* s = e->sideA.isBlockGrid ? &e->sideB : &e->sideA;
		ENSURE( g->shapeId.index1 == terrain.index1 && g->shapeId.generation == terrain.generation );
		ENSURE( s->shapeId.index1 == shape.index1 && g->cellX == -1 && g->userData == 991 && g->userMaterialId == 73 );
	}
	for ( int repeat = 0; repeat < 2; ++repeat )
	{
		b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
		ENSURE( player != NULL );
		b3RecPlayer_SetKeyframePolicy( player, 16 * 1024 * 1024, 10 );
		ReplayEvents actual;
		for ( int i = 0; i <= 120; ++i )
		{
			if ( i > 0 )
				ENSURE( b3RecPlayer_StepFrame( player ) );
			result |= CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actual );
			if ( CompareReplayEvents( trace + i, &actual ) != 0 )
			{
				printf( "seed settled=%d replace=%d destroy=%d frame=%d\n", settled, replace, destroy, i );
				result = 1;
				break;
			}
		}
		if ( result == 0 )
		{
			int seeks[] = { 20, 70, 10, 120, 0, 1 };
			for ( int i = 0; i < ARRAY_COUNT( seeks ); ++i )
			{
				b3RecPlayer_SeekFrame( player, seeks[i] );
				result |= CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actual );
				result |= CompareReplayEvents( trace + seeks[i], &actual );
			}
			b3RecPlayer_Restart( player );
			result |= CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actual );
			result |= CompareReplayEvents( trace, &actual );
			ENSURE( b3RecPlayer_StepFrame( player ) );
			result |= CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actual );
			result |= CompareReplayEvents( trace + 1, &actual );
		}
		result |= b3RecPlayer_HasDiverged( player );
		b3DestroyPlayer( player );
	}
	b3Free( trace, 121 * sizeof( ReplayEvents ) );
	b3DestroyRecording( rec );
	b3DestroyWorld( world );
	v3DestroyBlockGridData( grid );
	return result;
}

static int BlockGridEventSeeds( void )
{
	int result = CheckRecordingEventSeed( false, false, false );
	result |= CheckRecordingEventSeed( true, false, false );
	result |= CheckRecordingEventSeed( true, true, false );
	result |= CheckRecordingEventSeed( true, false, true );
	return result;
}

static int BlockGridOverflowReplay( void )
{
	int bytes = b3GetByteCount();
	for ( int removeTracked = 0; removeTracked < 2; ++removeTracked )
	{
		b3WorldDef wd = b3DefaultWorldDef();
		wd.gravity = (b3Vec3){ 0, -10, 0 };
		wd.enableSleep = false;
		b3WorldId world = b3CreateWorld( &wd );
		v3BlockGridData* grid = CookRecordingEventSlab();
		b3BodyDef bd = b3DefaultBodyDef();
		b3ShapeDef sd = b3DefaultShapeDef();
		sd.enableContactEvents = true;
		b3ShapeId terrain = v3CreateBlockGridShape( b3CreateBody( world, &bd ), &sd, grid );
		v3DestroyBlockGridData( grid );
		ENSURE( v3World_GetBlockContactEvents( world ).capacity == 256 );
		bd.type = b3_dynamicBody;
		bd.position = (b3Pos){ -0.25, 1.19, 0.5 };
		sd.filter.groupIndex = -1;
		b3Sphere sphere = { b3Vec3_zero, 0.2f };
		b3BodyId bodies[257];
		for ( int i = 0; i < 257; ++i )
		{
			bodies[i] = b3CreateBody( world, &bd );
			b3CreateSphereShape( bodies[i], &sd, &sphere );
		}
		b3World_Step( world, 1.0f / 60.0f, 4 );
		v3BlockContactEvents events = v3World_GetBlockContactEvents( world );
		ENSURE( events.beginCount == 256 && events.droppedBeginCount == 1 && events.truncated );
		b3BodyId tracked =
			events.beginEvents[0].sideA.isBlockGrid ? events.beginEvents[0].sideB.bodyId : events.beginEvents[0].sideA.bodyId;
		b3BodyId omitted = b3_nullBodyId;
		for ( int i = 0; i < 257; ++i )
		{
			bool found = false;
			for ( int j = 0; j < events.beginCount; ++j )
			{
				found |= B3_ID_EQUALS( bodies[i], events.beginEvents[j].sideA.bodyId ) ||
						 B3_ID_EQUALS( bodies[i], events.beginEvents[j].sideB.bodyId );
			}
			if ( !found )
				omitted = bodies[i];
		}
		ENSURE( B3_IS_NON_NULL( omitted ) && B3_IS_NON_NULL( terrain ) );
		// Seed with both the published overflow and an already pending cached end.
		b3DestroyBody( removeTracked ? tracked : omitted );
		b3Recording* rec = b3CreateRecording( 0 );
		b3World_StartRecording( world, rec );
		ReplayEvents trace[4];
		ENSURE( CaptureReplayEvents( world, trace ) == 0 );
		for ( int i = 1; i < 4; ++i )
		{
			if ( i == 3 && removeTracked )
				b3DestroyBody( omitted );
			b3World_Step( world, 1.0f / 60.0f, 4 );
			ENSURE( CaptureReplayEvents( world, trace + i ) == 0 );
		}
		ENSURE( trace[1].truncated && !trace[2].truncated && trace[1].beginCount == 0 );
		ENSURE( trace[1].endCount == removeTracked && trace[1].nativeEndCount == 1 );
		ENSURE( trace[3].endCount == removeTracked && trace[3].beginCount == 0 );
		b3World_StopRecording( world );
		for ( int repeat = 0; repeat < 2; ++repeat )
		{
			b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
			ENSURE( player != NULL );
			for ( int restart = 0; restart < 2; ++restart )
			{
				b3RecPlayer_Restart( player );
				for ( int i = 0; i < 4; ++i )
				{
					if ( i > 0 )
						ENSURE( b3RecPlayer_StepFrame( player ) );
					ReplayEvents actual;
					ENSURE( CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actual ) == 0 );
					ENSURE( CompareReplayEvents( trace + i, &actual ) == 0 );
				}
				ENSURE( !b3RecPlayer_HasDiverged( player ) );
			}
			b3DestroyPlayer( player );
		}
		b3DestroyRecording( rec );
		b3DestroyWorld( world );
	}
	ENSURE( b3GetByteCount() == bytes );
	return 0;
}

// Read the captured frame before replay has a chance to rebuild its counters.
static int CheckImmediateRecordingView( b3WorldId world, const ReplayEvents* expected )
{
	b3Recording* rec = b3CreateRecording( 0 );
	b3World_StartRecording( world, rec );
	b3World_StopRecording( world );
	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );
	ReplayEvents actual;
	b3WorldId replay = b3RecPlayer_GetWorldId( player );
	ENSURE( CaptureReplayEvents( replay, &actual ) == 0 );
	ENSURE( CompareReplayEvents( expected, &actual ) == 0 );
	// Give the shell a different completed frame, then replace it with its seed.
	b3World_Step( replay, 1.0f / 60.0f, 4 );
	b3RecPlayer_Restart( player );
	ENSURE( CaptureReplayEvents( replay, &actual ) == 0 );
	ENSURE( CompareReplayEvents( expected, &actual ) == 0 );
	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	return 0;
}

static int CheckCombinedRecording( bool replace, uint64_t* behaviorHash )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.workerCount = 1;
	wd.enableSleep = false;
	wd.gravity = (b3Vec3){ 0, -10, 0 };
	wd.hitEventThreshold = 0.1f;
	wd.maximumLinearSpeed = 40;
	wd.maximumAngularSpeed = 3;
	wd.projectileCandidateCap = 12;
	b3WorldId world = b3CreateWorld( &wd );
	b3Recording* rec = b3CreateRecording( 0 );
	b3World_StartRecording( world, rec );
	b3World_SetMaximumLinearSpeed( world, 30 );
	b3World_SetMaximumAngularSpeed( world, 2 );
	b3World_SetProjectileCandidateCap( world, 8 );
	v3BlockGridData* terrain = CookReplaySlab( 6, 6 );
	v3BlockGridData* hull = CookReplaySlab( 2, 2 );
	v3BlockGridData* revision = CookReplaySlab( 1, 2 );
	ENSURE( terrain && hull && revision );
	b3BodyDef bd = b3DefaultBodyDef();
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.enableContactEvents = sd.enableHitEvents = true;
	sd.density = 1.0f;
	b3BodyId ground = b3CreateBody( world, &bd );
	v3CreateBlockGridShape( ground, &sd, terrain );
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ 1.5, 3, 1.5 };
	b3BodyId ship = b3CreateBody( world, &bd );
	b3ShapeId shipShape = v3CreateBlockGridShape( ship, &sd, hull );
	b3BodyId bullet = b3_nullBodyId;
	b3ShapeId bulletShape = b3_nullShapeId;
	ReplayEvents* trace = b3Alloc( 120 * sizeof( ReplayEvents ) );
	ReplayMotion shipMotion[120], bulletMotion[60];
	uint64_t liveHash = B3_SNAP_FNV_INIT;
	uint64_t replacements = 0, sweeps = 0, candidates = 0, touching = 0, contacts = 0;
	bool projectileHit = false, landed = false;
	for ( int i = 0; i < 120; ++i )
	{
		if ( i == 60 )
		{
			printf( "  combined hull before shot p=(%.6f,%.6f,%.6f)\n", (double)b3Body_GetPosition( ship ).x,
					(double)b3Body_GetPosition( ship ).y, (double)b3Body_GetPosition( ship ).z );
			if ( replace )
				ENSURE( v3ReplaceBlockGridShape( shipShape, revision, true ) == v3_blockGridReplaceOk );
			ENSURE( b3Shape_IsValid( shipShape ) && B3_ID_EQUALS( b3Shape_GetBody( shipShape ), ship ) );
			ENSURE( b3Body_GetMass( ship ) == ( replace ? 2.0f : 4.0f ) );
			bd.position = (b3Pos){ -2, 1.5, 2.25 };
			bd.linearVelocity = (b3Vec3){ 120, 0, 0 };
			bd.gravityScale = 0;
			bd.isBullet = true;
			bullet = b3CreateBody( world, &bd );
			b3Sphere sphere = { b3Vec3_zero, 0.2f };
			bulletShape = b3CreateSphereShape( bullet, &sd, &sphere );
		}
		b3World_Step( world, 1.0f / 60.0f, 4 );
		ENSURE( CaptureReplayEvents( world, trace + i ) == 0 );
		shipMotion[i] = CaptureReplayMotion( ship );
		liveHash = HashReplayMotion( liveHash, shipMotion[i] );
		if ( i >= 60 )
		{
			bulletMotion[i - 60] = CaptureReplayMotion( bullet );
			liveHash = HashReplayMotion( liveHash, bulletMotion[i - 60] );
			ENSURE( b3Length( bulletMotion[i - 60].linearVelocity ) <= 30.0001f );
		}
		v3BlockGridPairCounters c = trace[i].counters;
		replacements += c.replacementPublishedCount;
		sweeps += c.projectileSweepCount;
		candidates += c.candidateHitboxPairCount;
		touching += c.touchingPairCount;
		contacts += c.contactCount;
		for ( int j = 0; j < trace[i].hitCount; ++j )
		{
			v3BlockContactEvent* e = trace[i].hit + j;
			bool shipBullet = ( e->sideA.shapeId.index1 == shipShape.index1 && e->sideB.shapeId.index1 == bulletShape.index1 ) ||
							  ( e->sideB.shapeId.index1 == shipShape.index1 && e->sideA.shapeId.index1 == bulletShape.index1 );
			projectileHit |= shipBullet && e->normalImpulse > 0 && e->approachSpeed > 0.1f;
			landed |= i < 60 && e->sideA.isBlockGrid && e->sideB.isBlockGrid;
		}
	}
	printf( "  combined replace=%d landed=%d projectileHit=%d sweeps=%llu replacements=%llu pairs=%llu touching=%llu\n", replace,
			landed, projectileHit, (unsigned long long)sweeps, (unsigned long long)replacements, (unsigned long long)candidates,
			(unsigned long long)touching );
	ENSURE( landed && projectileHit && sweeps > 0 && replacements == (uint64_t)replace && candidates > 0 && touching > 0 &&
			contacts > 0 );
	ENSURE( bulletMotion[0].linearVelocity.x == 30.0f );
	b3World_StopRecording( world );
	ENSURE( trace[119].counters.contactCount > 0 && trace[119].counters.candidateHitboxPairCount > 0 );
	ENSURE( CheckImmediateRecordingView( world, trace + 119 ) == 0 );
	for ( int repeat = 0; repeat < 2; ++repeat )
	{
		b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
		ENSURE( player != NULL );
		b3RecPlayer_SetKeyframePolicy( player, 16 * 1024 * 1024, 10 );
		b3WorldId replayWorld = b3RecPlayer_GetWorldId( player );
		ENSURE( b3World_GetMaximumLinearSpeed( replayWorld ) == 40 && b3World_GetMaximumAngularSpeed( replayWorld ) == 3 &&
				b3World_GetProjectileCandidateCap( replayWorld ) == 12 );
		uint64_t replayHash = B3_SNAP_FNV_INIT;
		for ( int i = 0; i < 120; ++i )
		{
			ENSURE( b3RecPlayer_StepFrame( player ) );
			ENSURE( b3World_GetMaximumLinearSpeed( replayWorld ) == 30 && b3World_GetMaximumAngularSpeed( replayWorld ) == 2 &&
					b3World_GetProjectileCandidateCap( replayWorld ) == 8 );
			ReplayEvents actual;
			ENSURE( CaptureReplayEvents( replayWorld, &actual ) == 0 );
			if ( CompareReplayEvents( trace + i, &actual ) )
			{
				printf( "combined mismatch frame=%d repeat=%d\n", i + 1, repeat );
				return 1;
			}
			b3BodyId replayShip = b3RecPlayer_GetBodyId( player, 1 );
			ENSURE( replayShip.index1 == ship.index1 && replayShip.generation == ship.generation );
			ReplayMotion motion = CaptureReplayMotion( replayShip );
			ENSURE( CompareReplayMotion( shipMotion[i], motion ) == 0 );
			replayHash = HashReplayMotion( replayHash, motion );
			if ( i >= 60 )
			{
				motion = CaptureReplayMotion( b3RecPlayer_GetBodyId( player, 2 ) );
				ENSURE( CompareReplayMotion( bulletMotion[i - 60], motion ) == 0 );
				replayHash = HashReplayMotion( replayHash, motion );
			}
		}
		ENSURE( replayHash == liveHash && !b3RecPlayer_HasDiverged( player ) );
		ENSURE( b3RecPlayer_GetKeyframeBytes( player ) > 0 );
		int seeks[] = { 21, 61, 91, 41, 119 };
		for ( int j = 0; j < ARRAY_COUNT( seeks ); ++j )
		{
			int frame = seeks[j];
			b3RecPlayer_SeekFrame( player, frame );
			ReplayEvents actual;
			ENSURE( CaptureReplayEvents( replayWorld, &actual ) == 0 );
			ENSURE( CompareReplayEvents( trace + frame - 1, &actual ) == 0 );
			ENSURE( CompareReplayMotion( shipMotion[frame - 1], CaptureReplayMotion( b3RecPlayer_GetBodyId( player, 1 ) ) ) ==
					0 );
			ENSURE( !b3RecPlayer_HasDiverged( player ) );
		}
		b3DestroyPlayer( player );
	}
	*behaviorHash = liveHash;
	b3Free( trace, 120 * sizeof( ReplayEvents ) );
	b3DestroyRecording( rec );
	b3DestroyWorld( world );
	v3DestroyBlockGridData( terrain );
	v3DestroyBlockGridData( hull );
	v3DestroyBlockGridData( revision );
	return 0;
}

static int BlockGridCombinedReplay( void )
{
	int bytes = b3GetByteCount();
	uint64_t control, replaced;
	ENSURE( CheckCombinedRecording( false, &control ) == 0 );
	ENSURE( CheckCombinedRecording( true, &replaced ) == 0 );
	ENSURE( control != replaced );
	printf( "  combined behavior hash control=%016llx replaced=%016llx\n", (unsigned long long)control,
			(unsigned long long)replaced );
	ENSURE( b3GetByteCount() == bytes );
	return 0;
}

// Corrupt only a digest in frame 2. Simulation bytes stay identical, and both
// stepping interfaces must consume all frame digests before returning frame 2.
static int RecordingDigestDivergence( void )
{
	b3WorldDef wd = b3DefaultWorldDef();
	b3WorldId world = b3CreateWorld( &wd );
	b3Recording* rec = b3CreateRecording( 0 );
	b3World_StartRecording( world, rec );
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = b3_dynamicBody;
	b3BodyId body = b3CreateBody( world, &bd );
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3ShapeDef sd = b3DefaultShapeDef();
	b3CreateSphereShape( body, &sd, &sphere );
	ReplayMotion motion[3];
	for ( int i = 0; i < 3; ++i )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		motion[i] = CaptureReplayMotion( body );
	}
	b3World_StopRecording( world );
	b3DestroyWorld( world );
	int size = b3Recording_GetSize( rec );
	uint8_t* data = b3Alloc( size );
	int opcodes[] = { b3_recOpContactEventHash, b3_recOpBlockGridCountersHash };
	ENSURE( b3_recOpBlockGridCountersHash == 0xF4 );
	for ( int kind = 0; kind < 2; ++kind )
	{
		memcpy( data, b3Recording_GetData( rec ), size );
		b3RecHeader header;
		memcpy( &header, data, sizeof( header ) );
		int cursor = sizeof( header ) + (int)header.snapshotSize;
		int frame = 0, digests = 0;
		while ( cursor + 4 <= (int)header.registryOffset )
		{
			int opcode = data[cursor];
			int length = data[cursor + 1] | ( data[cursor + 2] << 8 ) | ( data[cursor + 3] << 16 );
			ENSURE( cursor + 4 + length <= (int)header.registryOffset );
			if ( opcode == b3_recOpStep )
				++frame;
			if ( opcode == opcodes[kind] )
			{
				++digests;
				if ( frame == 2 )
					data[cursor + 4 + length - 1] ^= 1;
			}
			cursor += 4 + length;
		}
		ENSURE( digests == 3 );
		for ( int staged = 0; staged < 2; ++staged )
		{
			b3RecPlayer* player = b3CreatePlayer( data, size, 1 );
			ENSURE( player != NULL );
			for ( int i = 1; i <= 3; ++i )
			{
				if ( staged )
				{
					for ( int call = 0; call < 3 && b3RecPlayer_GetFrame( player ) < i; ++call )
						b3RecPlayer_SubStepFrame( player );
				}
				else
					ENSURE( b3RecPlayer_StepFrame( player ) );
				ENSURE( b3RecPlayer_GetFrame( player ) == i );
				ENSURE( b3RecPlayer_HasDiverged( player ) == ( i >= 2 ) );
				ENSURE( b3RecPlayer_GetDivergeFrame( player ) == ( i >= 2 ? 2 : -1 ) );
				ENSURE( CompareReplayMotion( motion[i - 1], CaptureReplayMotion( b3RecPlayer_GetBodyId( player, 0 ) ) ) == 0 );
			}
			b3DestroyPlayer( player );
		}
	}
	b3Free( data, size );
	b3DestroyRecording( rec );
	return 0;
}

// Format tests target the appended section, leaving the established object and
// geometry encodings intact. This fixture has one active contact at first impact.
static int RecordingEventSectionValidation( void )
{
	int bytes = b3GetByteCount();
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = (b3Vec3){ 0, -10, 0 };
	wd.enableSleep = false;
	wd.hitEventThreshold = 0.1f;
	b3WorldId world = b3CreateWorld( &wd );
	v3BlockGridData* grid = CookRecordingEventSlab();
	b3BodyDef bd = b3DefaultBodyDef();
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.enableContactEvents = sd.enableHitEvents = true;
	v3CreateBlockGridShape( b3CreateBody( world, &bd ), &sd, grid );
	v3DestroyBlockGridData( grid );
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ -0.25, 2, 0.5 };
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3CreateSphereShape( b3CreateBody( world, &bd ), &sd, &sphere );
	ReplayEvents expected;
	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( world, 1.0f / 60.0f, 4 );
		if ( v3World_GetBlockContactEvents( world ).hitCount != 0 )
			break;
	}
	ENSURE( CaptureReplayEvents( world, &expected ) == 0 );
	ENSURE( expected.beginCount == 1 && expected.hitCount == 1 && expected.endCount == 0 && expected.nativeEndCount == 0 );
	b3Recording* rec = b3CreateRecording( 0 );
	b3World_StartRecording( world, rec );
	b3World_Step( world, 1.0f / 60.0f, 4 );
	b3World_StopRecording( world );
	b3DestroyWorld( world );
	int size = b3Recording_GetSize( rec );
	const uint8_t* original = b3Recording_GetData( rec );
	uint8_t* data = b3Alloc( size );
	b3RecHeader header;
	memcpy( &header, original, sizeof( header ) );
	int eventSize = 118 + 3 * (int)sizeof( expected.begin[0].point.x );
	int sectionSize = 118 + 20 + 3 * eventSize;
	int section = (int)sizeof( header ) + (int)header.snapshotSize - sectionSize;
	ENSURE( section > (int)sizeof( header ) );
	int allocationBaseline = b3GetByteCount();
	for ( int repeat = 0; repeat < 3; ++repeat )
	{
		b3RecPlayer* player = b3CreatePlayer( original, size, 1 );
		ENSURE( player != NULL );
		ReplayEvents actual;
		ENSURE( CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actual ) == 0 );
		ENSURE( CompareReplayEvents( &expected, &actual ) == 0 );
		ENSURE( b3RecPlayer_StepFrame( player ) );
		b3RecPlayer_Restart( player );
		ENSURE( CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actual ) == 0 );
		ENSURE( CompareReplayEvents( &expected, &actual ) == 0 );
		b3DestroyPlayer( player );
		ENSURE( b3GetByteCount() == allocationBaseline );
	}
	int cuts[] = { 1, 4, 8, 32, eventSize, sectionSize - 1 };
	for ( int i = 0; i < ARRAY_COUNT( cuts ); ++i )
	{
		memcpy( data, original, size );
		b3RecHeader truncated = header;
		truncated.snapshotSize -= cuts[i];
		memcpy( data, &truncated, sizeof( truncated ) );
		ENSURE( b3CreatePlayer( data, size, 1 ) == NULL );
		ENSURE( b3GetByteCount() == allocationBaseline );
	}
	// Invalid flags, an oversized history, invalid side encoding/ID, a NaN point,
	// a negative contact slot, an oversized event array, and an unbounded native end count.
	int offsets[] = { 0, 1, 74, 78 + 12, 78, 78 + 98, 78 + eventSize, 78 + eventSize + 8 + 4, sectionSize - 8 };
	uint32_t values[] = { 2, 2, 257, 2, 0, 0x7fc00000, UINT32_MAX, 257, INT32_MAX };
	for ( int i = 0; i < ARRAY_COUNT( offsets ); ++i )
	{
		memcpy( data, original, size );
		int width = ( i == 0 || i == 1 || i == 3 ) ? 1 : 4;
		if ( i == 5 )
		{
			uint64_t nan = UINT64_C( 0x7ff8000000000000 );
			memcpy( data + section + offsets[i], &nan, sizeof( nan ) );
		}
		else
			memcpy( data + section + offsets[i], values + i, width );
		ENSURE( b3CreatePlayer( data, size, 1 ) == NULL );
		ENSURE( b3GetByteCount() == allocationBaseline );
	}
	b3Free( data, size );
	b3DestroyRecording( rec );
	ENSURE( b3GetByteCount() == bytes );
	return 0;
}

static int recordingEventAllocations;
static int recordingFailEventAllocation;

static void* RecordingEventAlloc( int size, int alignment )
{
	unsigned int eventBytes = (unsigned int)( 256 * sizeof( v3BlockContactEvent ) );
	unsigned int recordBytes = (unsigned int)( 256 * sizeof( v3BlockContactRecord ) );
	if ( size == eventBytes || size == recordBytes )
	{
		if ( ++recordingEventAllocations == recordingFailEventAllocation )
			return NULL;
	}
#if defined( _WIN32 )
	return _aligned_malloc( size, alignment );
#else
	return aligned_alloc( alignment, size );
#endif
}

static void RecordingEventFree( void* memory )
{
#if defined( _WIN32 )
	_aligned_free( memory );
#else
	free( memory );
#endif
}

static int RecordingPendingAllocationFailure( void )
{
	int bytes = b3GetByteCount();
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = b3Vec3_zero;
	b3WorldId world = b3CreateWorld( &wd );
	v3BlockGridData* grid = CookRecordingEventSlab();
	b3BodyDef bd = b3DefaultBodyDef();
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.enableContactEvents = true;
	b3BodyId ground = b3CreateBody( world, &bd );
	v3CreateBlockGridShape( ground, &sd, grid );
	v3DestroyBlockGridData( grid );
	bd.type = b3_dynamicBody;
	bd.position = (b3Pos){ -0.25, 1.19, 0.5 };
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3CreateSphereShape( b3CreateBody( world, &bd ), &sd, &sphere );
	b3World_Step( world, 1.0f / 60.0f, 4 );
	ENSURE( v3World_GetBlockContactEvents( world ).beginCount == 1 );
	b3DestroyBody( ground );
	bd = b3DefaultBodyDef();
	b3BodyId reused = b3CreateBody( world, &bd );
	ENSURE( reused.index1 == ground.index1 && reused.generation != ground.generation );
	b3Recording* rec = b3CreateRecording( 0 );
	b3World_StartRecording( world, rec );
	b3World_Step( world, 1.0f / 60.0f, 4 );
	ENSURE( v3World_GetBlockContactEvents( world ).endCount == 1 );
	b3World_StopRecording( world );
	b3DestroyWorld( world );
	int allocationBaseline = b3GetByteCount();
	// Every partial reservation must release the preceding successful allocations.
	for ( int failure = 1; failure <= 6; ++failure )
	{
		recordingEventAllocations = 0;
		recordingFailEventAllocation = failure;
		b3SetAllocator( RecordingEventAlloc, RecordingEventFree );
		b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
		b3SetAllocator( NULL, NULL );
		ENSURE( player == NULL && recordingEventAllocations == failure );
		ENSURE( b3GetByteCount() == allocationBaseline );
	}
	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL && b3RecPlayer_StepFrame( player ) );
	v3BlockContactEvents ends = v3World_GetBlockContactEvents( b3RecPlayer_GetWorldId( player ) );
	ENSURE( ends.endCount == 1 );
	const v3BlockContactSide* old = ends.endEvents[0].sideA.isBlockGrid ? &ends.endEvents[0].sideA : &ends.endEvents[0].sideB;
	ENSURE( old->bodyId.index1 == ground.index1 && old->bodyId.generation == ground.generation );
	ENSURE( old->bodyId.world0 == b3RecPlayer_GetWorldId( player ).index1 - 1 );
	b3DestroyPlayer( player );
	ENSURE( b3GetByteCount() == allocationBaseline );
	b3DestroyRecording( rec );
	ENSURE( b3GetByteCount() == bytes );
	return 0;
}

// AllOps covers the mutator encodings. This short sequence supplies independent
// motion evidence for a two-step target and a wrench applied before each fixed step.
static int RecordingAlphaMotion( void )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = b3Vec3_zero;
	wd.enableSleep = false;
	wd.maximumLinearSpeed = 30;
	wd.maximumAngularSpeed = 2;
	b3WorldId world = b3CreateWorld( &wd );
	b3Recording* rec = b3CreateRecording( 0 );
	b3World_StartRecording( world, rec );
	b3BodyDef bd = b3DefaultBodyDef();
	b3BodyId ground = b3CreateBody( world, &bd );
	bd.type = b3_dynamicBody;
	b3BodyId hinge = b3CreateBody( world, &bd );
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = 1;
	b3Sphere sphere = { b3Vec3_zero, 0.2f };
	b3CreateSphereShape( hinge, &sd, &sphere );
	b3RevoluteJointDef jd = b3DefaultRevoluteJointDef();
	jd.base.bodyIdA = ground;
	jd.base.bodyIdB = hinge;
	jd.enableMotor = true;
	jd.motorSpeed = 1;
	jd.maxMotorTorque = 10;
	b3JointId joint = b3CreateRevoluteJoint( world, &jd );
	bd.position = (b3Pos){ 5, 0, 0 };
	b3BodyId freeBody = b3CreateBody( world, &bd );
	b3CreateSphereShape( freeBody, &sd, &sphere );
	b3MassData mass = { .mass = 2, .inertia = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } };
	b3Body_SetMassData( freeBody, mass );
	// Unit inertia keeps the motor accelerating across the first fixed step.
	b3Body_SetMassData( hinge, mass );
	bd.type = b3_kinematicBody;
	bd.position = (b3Pos){ 10, 0, 0 };
	b3BodyId targetBody = b3CreateBody( world, &bd );
	b3CreateSphereShape( targetBody, &sd, &sphere );
	ReplayMotion trace[6][3];
	float angles[6], torques[6];
	float dt = 1.0f / 60.0f;
	for ( int batch = 0; batch < 3; ++batch )
	{
		b3WorldTransform target = { .p = { 10.0 + 0.1 * ( batch + 1 ), 0, 0 }, .q = b3Quat_identity };
		b3Body_SetTargetTransform( targetBody, target, 2 * dt, true );
		if ( batch == 2 )
		{
			b3Body_SetLinearVelocity( freeBody, (b3Vec3){ 1000, 0, 0 } );
			b3Body_SetAngularVelocity( freeBody, (b3Vec3){ 0, 0, 100 } );
		}
		for ( int fixed = 0; fixed < 2; ++fixed )
		{
			b3Body_ApplyForceToCenter( freeBody, (b3Vec3){ 6, 0, 0 }, true );
			b3Body_ApplyTorque( freeBody, (b3Vec3){ 0, 0, 3 }, true );
			b3World_Step( world, dt, 4 );
			int frame = 2 * batch + fixed;
			trace[frame][0] = CaptureReplayMotion( hinge );
			trace[frame][1] = CaptureReplayMotion( freeBody );
			trace[frame][2] = CaptureReplayMotion( targetBody );
			angles[frame] = b3RevoluteJoint_GetAngle( joint );
			torques[frame] = b3RevoluteJoint_GetMotorTorque( joint );
			if ( batch < 2 )
			{
				ENSURE_SMALL( trace[frame][1].linearVelocity.x - 3 * dt * ( frame + 1 ), 1.0e-6f );
				ENSURE_SMALL( trace[frame][1].angularVelocity.z - 3 * dt * ( frame + 1 ), 1.0e-6f );
			}
			else
			{
				ENSURE_SMALL( trace[frame][1].linearVelocity.x - 30, 1.0e-5f );
				ENSURE_SMALL( trace[frame][1].angularVelocity.z - 2, 1.0e-5f );
			}
		}
		ENSURE_SMALL( b3Body_GetPosition( targetBody ).x - target.p.x, 1.0e-6 );
	}
	printf( "  alpha hinge angles %.9g %.9g motor torque %.9g angular velocity %.9g\n", angles[0], angles[5], torques[0],
			trace[0][0].angularVelocity.z );
	ENSURE( angles[0] > 0 && angles[5] > angles[0] && torques[0] > 0 );
	b3World_StopRecording( world );
	b3DestroyWorld( world );
	b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
	ENSURE( player != NULL );
	for ( int frame = 0; frame < 6; ++frame )
	{
		ENSURE( b3RecPlayer_StepFrame( player ) );
		for ( int body = 0; body < 3; ++body )
		{
			ENSURE( CompareReplayMotion( trace[frame][body], CaptureReplayMotion( b3RecPlayer_GetBodyId( player, body + 1 ) ) ) ==
					0 );
		}
		b3JointId replayJoint;
		ENSURE( b3Body_GetJoints( b3RecPlayer_GetBodyId( player, 1 ), &replayJoint, 1 ) == 1 );
		ENSURE( b3RevoluteJoint_GetAngle( replayJoint ) == angles[frame] );
		ENSURE( b3RevoluteJoint_GetMotorTorque( replayJoint ) == torques[frame] );
		ENSURE( !b3RecPlayer_HasDiverged( player ) );
	}
	b3DestroyPlayer( player );
	b3DestroyRecording( rec );
	return 0;
}

static int RecordingProjectileCap( void )
{
	// The existing sweep acceptance fixture's varying far faces keep the 16x16
	// wall fragmented. A four-metre capsule has several equally early candidates.
	v3BlockGridBlock blocks[256];
	v3BlockGridBox boxes[256];
	for ( int y = 0; y < 16; ++y )
	{
		for ( int z = 0; z < 16; ++z )
		{
			int i = 16 * y + z;
			boxes[i] = (v3BlockGridBox){ .bounds = { { 0, 0, 0 }, { 0.5f + 0.02f * ( ( y * 7 + z * 3 ) % 21 ), 1, 1 } } };
			blocks[i] = (v3BlockGridBlock){ .y = y, .z = z, .userData = i + 1, .boxes = boxes + i, .boxCount = 1 };
		}
	}
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	v3BlockGridCookDef cook = { .materials = &material, .materialCount = 1, .blocks = blocks, .blockCount = 256 };
	v3BlockGridData* grid = v3CookBlockGrid( &cook ).data;
	ENSURE( grid != NULL && v3BlockGrid_GetCookStats( grid ).boxCount > 1 );
	double unlimitedX = 0;
	for ( int cap = 0; cap <= 1; ++cap )
	{
		b3WorldDef wd = b3DefaultWorldDef();
		wd.gravity = b3Vec3_zero;
		wd.enableSleep = false;
		wd.workerCount = 1;
		b3WorldId world = b3CreateWorld( &wd );
		b3BodyDef bd = b3DefaultBodyDef();
		b3ShapeDef sd = b3DefaultShapeDef();
		sd.enableContactEvents = sd.enableHitEvents = true;
		v3CreateBlockGridShape( b3CreateBody( world, &bd ), &sd, grid );
		bd.type = b3_dynamicBody;
		bd.position = (b3Pos){ -1, 6.5, 9.5 };
		bd.linearVelocity = (b3Vec3){ 120, 0, 0 };
		bd.isBullet = true;
		b3BodyId bullet = b3CreateBody( world, &bd );
		b3Capsule capsule = { { 0, 0, -2 }, { 0, 0, 2 }, 0.1f };
		b3CreateCapsuleShape( bullet, &sd, &capsule );
		b3Recording* rec = b3CreateRecording( 0 );
		b3World_StartRecording( world, rec );
		b3World_SetProjectileCandidateCap( world, cap );
		b3World_Step( world, 1.0f / 60.0f, 4 );
		ReplayMotion motion = CaptureReplayMotion( bullet );
		ReplayEvents expected;
		ENSURE( CaptureReplayEvents( world, &expected ) == 0 );
		ENSURE( expected.counters.projectileSweepCount > 0 );
		if ( cap == 0 )
		{
			ENSURE( expected.counters.capExhaustionCount == 0 && motion.position.x > -1 && motion.position.x < 0 );
			unlimitedX = motion.position.x;
		}
		else
		{
			ENSURE( expected.counters.capExhaustionCount > 0 && motion.position.x == -1 && motion.linearVelocity.x == 120 );
			ENSURE( motion.position.x < unlimitedX );
		}
		b3World_StopRecording( world );
		ENSURE( CheckImmediateRecordingView( world, &expected ) == 0 );
		for ( int repeat = 0; repeat < 2; ++repeat )
		{
			b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( rec ), b3Recording_GetSize( rec ), 1 );
			ENSURE( player != NULL && b3RecPlayer_StepFrame( player ) );
			ReplayEvents actual;
			ENSURE( CaptureReplayEvents( b3RecPlayer_GetWorldId( player ), &actual ) == 0 );
			ENSURE( CompareReplayEvents( &expected, &actual ) == 0 );
			ENSURE( CompareReplayMotion( motion, CaptureReplayMotion( b3RecPlayer_GetBodyId( player, 1 ) ) ) == 0 );
			ENSURE( !b3RecPlayer_HasDiverged( player ) );
			b3DestroyPlayer( player );
		}
		b3DestroyRecording( rec );
		b3DestroyWorld( world );
	}
	v3DestroyBlockGridData( grid );
	return 0;
}

int RecordingTest( void )
{
	RUN_SUBTEST( BlockGridReplacementCapacityReplay );
	RUN_SUBTEST( BlockGridNonTouchingReplacementSeed );
	RUN_SUBTEST( BlockGridEventSeeds );
	RUN_SUBTEST( BlockGridOverflowReplay );
	RUN_SUBTEST( BlockGridCombinedReplay );
	RUN_SUBTEST( RecordingDigestDivergence );
	RUN_SUBTEST( RecordingEventSectionValidation );
	RUN_SUBTEST( RecordingPendingAllocationFailure );
	RUN_SUBTEST( RecordingAlphaMotion );
	RUN_SUBTEST( RecordingProjectileCap );
	RUN_SUBTEST( GeometryHashCollision );
	RUN_SUBTEST( ShapeNameReplay );
	RUN_SUBTEST( SphereRoundTrip );
	RUN_SUBTEST( SafetyFactorRoundTrip );
	RUN_SUBTEST( EmptyWorldRoundTrip );
	RUN_SUBTEST( HullDedup );
	RUN_SUBTEST( MidStreamNoContacts );
	RUN_SUBTEST( MidStreamContacts );
	RUN_SUBTEST( StagedStepCreationPose );
	RUN_SUBTEST( ScrubBackward );
	RUN_SUBTEST( SeekWithHull );
	RUN_SUBTEST( DebugShapeCallbacks );
	RUN_SUBTEST( PlayerAccessors );
	RUN_SUBTEST( KeyframeHandleReuse );
	RUN_SUBTEST( QueryReplay );
	RUN_SUBTEST( TaggedQuery );
	RUN_SUBTEST( TransformedHullRoundTrip );
	RUN_SUBTEST( GeometryMutatorReplay );
	RUN_SUBTEST( BlockGridReplaceReplay );
	RUN_SUBTEST( AllOps );
	RUN_SUBTEST( ReservedHeaderBytes );
	return 0;
}

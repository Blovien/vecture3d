// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "vecture3d/block_grid.h"

#include "box3d/box3d.h"

#include <stdio.h>
#include <string.h>

#define CHECK( condition )                                                                                                       \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( !( condition ) )                                                                                                    \
		{                                                                                                                        \
			fprintf( stderr, "condition false at %s:%d: %s\n", __FILE__, __LINE__, #condition );                                 \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static v3BlockGridData* CookUnitBlock( void )
{
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	v3BlockGridBox box = { .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
	v3BlockGridBlock block = { .userData = 17, .boxes = &box, .boxCount = 1 };
	v3BlockGridCookDef cookDef = { .materials = &material, .materialCount = 1, .blocks = &block, .blockCount = 1 };
	v3BlockGridCookResult result = v3CookBlockGrid( &cookDef );
	return result.status == v3_blockGridCookOk ? result.data : NULL;
}

static int TestCrossWorldAttachmentsRetainCookedData( void )
{
	int byteCountBefore = b3GetByteCount();
	v3BlockGridData* data = CookUnitBlock();
	CHECK( data != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId staticWorldId = b3CreateWorld( &worldDef );
	b3WorldId dynamicWorldId = b3CreateWorld( &worldDef );
	CHECK( b3World_IsValid( staticWorldId ) );
	CHECK( b3World_IsValid( dynamicWorldId ) );

	b3BodyDef staticBodyDef = b3DefaultBodyDef();
	b3BodyId staticBodyId = b3CreateBody( staticWorldId, &staticBodyDef );
	b3BodyDef dynamicBodyDef = b3DefaultBodyDef();
	dynamicBodyDef.type = b3_dynamicBody;
	dynamicBodyDef.position = (b3Pos){ 4.0f, 2.0f, 0.0f };
	b3BodyId dynamicBodyId = b3CreateBody( dynamicWorldId, &dynamicBodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId staticShapeId = v3CreateBlockGridShape( staticBodyId, &shapeDef, data );
	b3ShapeId dynamicShapeId = v3CreateBlockGridShape( dynamicBodyId, &shapeDef, data );
	CHECK( b3Shape_IsValid( staticShapeId ) );
	CHECK( b3Shape_IsValid( dynamicShapeId ) );
	CHECK( b3Body_GetShapeCount( staticBodyId ) == 1 );
	CHECK( b3Body_GetShapeCount( dynamicBodyId ) == 1 );

	b3BodyDef sphereBodyDef = b3DefaultBodyDef();
	sphereBodyDef.type = b3_dynamicBody;
	sphereBodyDef.position = (b3Pos){ 0.5f, 1.4f, 0.5f };
	b3BodyId sphereBodyId = b3CreateBody( staticWorldId, &sphereBodyDef );
	b3Sphere sphere = { .radius = 0.5f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );
	CHECK( b3Shape_IsValid( sphereShapeId ) );

	v3DestroyBlockGridData( data );
	b3World_Step( staticWorldId, 1.0f / 60.0f, 4 );
	b3World_Step( dynamicWorldId, 1.0f / 60.0f, 4 );
	CHECK( b3Shape_IsValid( staticShapeId ) );
	CHECK( b3Shape_IsValid( dynamicShapeId ) );

	b3DestroyBody( dynamicBodyId );
	CHECK( b3Body_IsValid( dynamicBodyId ) == false );
	CHECK( b3Shape_IsValid( dynamicShapeId ) == false );
	b3World_Step( dynamicWorldId, 1.0f / 60.0f, 4 );
	CHECK( b3World_IsValid( dynamicWorldId ) );
	CHECK( b3Shape_IsValid( staticShapeId ) );

	b3DestroyWorld( dynamicWorldId );
	CHECK( b3World_IsValid( dynamicWorldId ) == false );
	CHECK( b3Shape_IsValid( staticShapeId ) );
	b3World_Step( staticWorldId, 1.0f / 60.0f, 4 );
	CHECK( b3Shape_IsValid( staticShapeId ) );

	b3DestroyShape( staticShapeId, true );
	CHECK( b3Shape_IsValid( staticShapeId ) == false );
	b3DestroyWorld( staticWorldId );
	CHECK( b3GetByteCount() == byteCountBefore );
	return 0;
}

typedef struct LockedAttachmentContext
{
	b3BodyId bodyId;
	b3ShapeDef shapeDef;
	v3BlockGridData* data;
	b3ShapeId result;
	int attemptCount;
} LockedAttachmentContext;

static bool expectedLockedWorldAssert;
static int expectedLockedWorldAssertCount;

static int HandleExpectedLockedWorldAssert( const char* condition, const char* fileName, int lineNumber )
{
	if ( expectedLockedWorldAssert && strcmp( condition, "false" ) == 0 )
	{
		expectedLockedWorldAssertCount += 1;
		return 0;
	}

	fprintf( stderr, "unexpected Box3D assertion at %s:%d: %s\n", fileName, lineNumber, condition );
	return 1;
}

static void* AttemptAttachmentWhileWorldIsLocked( b3TaskCallback* task, void* taskContext, void* userContext,
												  const char* taskName )
{
	(void)taskName;
	LockedAttachmentContext* context = userContext;
	if ( context->attemptCount == 0 )
	{
		context->attemptCount = 1;
		expectedLockedWorldAssert = true;
		context->result = v3CreateBlockGridShape( context->bodyId, &context->shapeDef, context->data );
		expectedLockedWorldAssert = false;
	}

	task( taskContext );
	return NULL;
}

static void FinishSynchronousTask( void* userTask, void* userContext )
{
	(void)userTask;
	(void)userContext;
}

static int TestAttachmentFailuresAreAtomic( void )
{
	int byteCountBefore = b3GetByteCount();
	v3BlockGridData* data = CookUnitBlock();
	CHECK( data != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	CHECK( b3Body_GetShapeCount( bodyId ) == 0 );

	CHECK( v3CreateBlockGridShape( bodyId, &shapeDef, NULL ).index1 == 0 );
	CHECK( b3Body_GetShapeCount( bodyId ) == 0 );
	CHECK( v3CreateBlockGridShape( bodyId, NULL, data ).index1 == 0 );
	CHECK( b3Body_GetShapeCount( bodyId ) == 0 );
	b3ShapeDef sensorDef = shapeDef;
	sensorDef.isSensor = true;
	CHECK( v3CreateBlockGridShape( bodyId, &sensorDef, data ).index1 == 0 );
	CHECK( b3Body_GetShapeCount( bodyId ) == 0 );
	CHECK( v3CreateBlockGridShape( b3_nullBodyId, &shapeDef, data ).index1 == 0 );
	CHECK( b3Body_GetShapeCount( bodyId ) == 0 );

	b3WorldId staleWorldId = b3CreateWorld( &worldDef );
	b3BodyId staleBodyId = b3CreateBody( staleWorldId, &bodyDef );
	b3DestroyWorld( staleWorldId );
	CHECK( v3CreateBlockGridShape( staleBodyId, &shapeDef, data ).index1 == 0 );
	CHECK( b3Body_GetShapeCount( bodyId ) == 0 );

	b3ShapeId shapeId = v3CreateBlockGridShape( bodyId, &shapeDef, data );
	CHECK( b3Shape_IsValid( shapeId ) );
	CHECK( b3Body_GetShapeCount( bodyId ) == 1 );
	v3DestroyBlockGridData( data );
	b3DestroyWorld( worldId );
	CHECK( b3GetByteCount() == byteCountBefore );
	return 0;
}

static int TestLockedWorldAttachmentFailureIsAtomic( void )
{
	int byteCountBefore = b3GetByteCount();
	v3BlockGridData* data = CookUnitBlock();
	CHECK( data != NULL );
	v3BlockGridCookStats statsBefore = v3BlockGrid_GetCookStats( data );

	LockedAttachmentContext context = { .shapeDef = b3DefaultShapeDef(), .data = data };
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	worldDef.enqueueTask = AttemptAttachmentWhileWorldIsLocked;
	worldDef.finishTask = FinishSynchronousTask;
	worldDef.userTaskContext = &context;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	CHECK( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	context.bodyId = b3CreateBody( worldId, &bodyDef );
	CHECK( b3Body_IsValid( context.bodyId ) );
	CHECK( b3Body_GetShapeCount( context.bodyId ) == 0 );

	b3SetAssertFcn( HandleExpectedLockedWorldAssert );
	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	CHECK( context.attemptCount == 1 );
	CHECK( context.result.index1 == 0 );
	CHECK( b3Body_GetShapeCount( context.bodyId ) == 0 );
#if !defined( NDEBUG ) || defined( B3_ENABLE_ASSERT )
	CHECK( expectedLockedWorldAssertCount == 1 );
#endif

	v3BlockGridCookStats statsAfter = v3BlockGrid_GetCookStats( data );
	CHECK( statsAfter.materialCount == statsBefore.materialCount );
	CHECK( statsAfter.blockCount == statsBefore.blockCount );
	CHECK( statsAfter.boxCount == statsBefore.boxCount );
	CHECK( statsAfter.byteCount == statsBefore.byteCount );

	b3ShapeId shapeId = v3CreateBlockGridShape( context.bodyId, &context.shapeDef, data );
	CHECK( b3Shape_IsValid( shapeId ) );
	CHECK( b3Body_GetShapeCount( context.bodyId ) == 1 );
	v3DestroyBlockGridData( data );
	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	CHECK( b3Shape_IsValid( shapeId ) );
	b3DestroyWorld( worldId );
	CHECK( b3GetByteCount() == byteCountBefore );
	return 0;
}

int main( void )
{
	CHECK( TestCrossWorldAttachmentsRetainCookedData() == 0 );
	CHECK( TestAttachmentFailuresAreAtomic() == 0 );
	CHECK( TestLockedWorldAttachmentFailureIsAtomic() == 0 );
	return 0;
}

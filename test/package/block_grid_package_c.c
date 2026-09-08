// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "vecture3d/block_grid.h"

#include <stddef.h>

int main( void )
{
	int byteCountBefore = b3GetByteCount();
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	v3BlockGridBox box = { .bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } } };
	v3BlockGridBlock block = { .boxes = &box, .boxCount = 1 };
	v3BlockGridCookDef def = { .materials = &material, .materialCount = 1, .blocks = &block, .blockCount = 1 };
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	if ( result.status != v3_blockGridCookOk || result.data == NULL )
	{
		return 1;
	}
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId shapeId = v3CreateBlockGridShape( bodyId, &shapeDef, result.data );
	v3DestroyBlockGridData( result.data );
	if ( b3Shape_IsValid( shapeId ) == false )
	{
		b3DestroyWorld( worldId );
		return 2;
	}
	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	b3DestroyShape( shapeId, true );
	b3DestroyWorld( worldId );
	return b3GetByteCount() == byteCountBefore ? 0 : 3;
}

// SPDX-License-Identifier: MIT

#include "box3d/box3d.h"
#include "vecture3d/block_grid.h"

int main()
{
	const int byteCountBefore = b3GetByteCount();
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
	v3BlockGridBox box{};
	box.bounds.upperBound = { 1.0f, 1.0f, 1.0f };
	v3BlockGridBlock block{};
	block.boxes = &box;
	block.boxCount = 1;
	v3BlockGridCookDef def{};
	def.materials = &material;
	def.materialCount = 1;
	def.blocks = &block;
	def.blockCount = 1;
	v3BlockGridCookResult result = v3CookBlockGrid( &def );
	if ( result.status != v3_blockGridCookOk || result.data == nullptr )
	{
		return 1;
	}
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId shapeId = v3CreateBlockGridShape( bodyId, &shapeDef, result.data );
	v3DestroyBlockGridData( result.data );
	if ( !b3Shape_IsValid( shapeId ) )
	{
		b3DestroyWorld( worldId );
		return 2;
	}
	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	b3DestroyWorld( worldId );
	return b3GetByteCount() == byteCountBefore ? 0 : 3;
}

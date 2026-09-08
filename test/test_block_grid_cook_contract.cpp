// SPDX-FileCopyrightText: 2026 Andrea Rossi
// SPDX-License-Identifier: MIT

#include "vecture3d/block_grid.h"

#include <array>
#include <cstdio>
#include <future>

static bool CheckStats( const v3BlockGridCookStats& stats )
{
	return stats.materialCount == 1 && stats.blockCount == 2 && stats.boxCount == 3 && stats.byteCount > 0;
}

int main()
{
	const int byteCountBeforeCook = b3GetByteCount();
	b3SurfaceMaterial material = b3DefaultSurfaceMaterial();

	std::array<v3BlockGridBox, 2> firstBoxes{};
	firstBoxes[0].bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.4f, 1.0f } };
	firstBoxes[1].bounds = { { 0.2f, 0.3f, 0.2f }, { 0.8f, 1.0f, 0.8f } };
	std::array<v3BlockGridBox, 1> secondBoxes{};
	secondBoxes[0].bounds = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f } };

	std::array<v3BlockGridBlock, 2> blocks{};
	blocks[0].x = 5;
	blocks[0].userData = 99;
	blocks[0].boxes = firstBoxes.data();
	blocks[0].boxCount = static_cast<int>( firstBoxes.size() );
	blocks[1].x = -2;
	blocks[1].userData = 99;
	blocks[1].boxes = secondBoxes.data();
	blocks[1].boxCount = static_cast<int>( secondBoxes.size() );

	v3BlockGridCookDef def{};
	def.materials = &material;
	def.materialCount = 1;
	def.blocks = blocks.data();
	def.blockCount = static_cast<int>( blocks.size() );

	std::array<std::future<v3BlockGridCookResult>, 4> cooks;
	for ( auto& cook : cooks )
	{
		cook = std::async( std::launch::async, [&def] { return v3CookBlockGrid( &def ); } );
	}

	std::array<v3BlockGridCookResult, 4> results{};
	for ( size_t i = 0; i < results.size(); ++i )
	{
		results[i] = cooks[i].get();
		if ( results[i].status != v3_blockGridCookOk || results[i].data == nullptr || !CheckStats( results[i].stats ) )
		{
			std::fprintf( stderr, "independent cook %zu failed\n", i );
			return 1;
		}
	}

	const v3BlockGridData* shared = results[0].data;
	std::array<std::future<bool>, 4> readers;
	for ( auto& reader : readers )
	{
		reader = std::async( std::launch::async, [shared] {
			for ( int i = 0; i < 1000; ++i )
			{
				if ( !CheckStats( v3BlockGrid_GetCookStats( shared ) ) )
				{
					return false;
				}
			}
			return true;
		} );
	}
	for ( auto& reader : readers )
	{
		if ( !reader.get() )
		{
			std::fprintf( stderr, "immutable shared read failed\n" );
			return 1;
		}
	}

	for ( v3BlockGridCookResult& result : results )
	{
		v3DestroyBlockGridData( result.data );
	}
	if ( b3GetByteCount() != byteCountBeforeCook )
	{
		std::fprintf( stderr, "cook ownership leaked bytes\n" );
		return 1;
	}
	return 0;
}

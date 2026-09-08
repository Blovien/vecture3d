// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "world_snapshot.h"

#include "arena_allocator.h"
#include "bitset.h"
#include "body.h"
#include "broad_phase.h"
#include "compound.h"
#include "constraint_graph.h"
#include "contact.h"
#include "container.h"
#include "core.h"
#include "id_pool.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "recording.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"
#include "table.h"
#include "block_grid/block_grid.h"
#include "block_grid/block_grid_contact.h"
#include "block_grid/block_grid_internal.h"
#include "block_grid/block_grid_pair.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/types.h"

#include <limits.h>
#include <math.h>
#include <string.h>

// Snapshot image magic 'BNS3' and version
#define B3_SNAP_MAGIC 0x33534E42u
#define B3_SNAP_VERSION 11u // Retained BlockGrid pair and affected contact array capacities

#define B3_SNAP_FLAG_VALIDATION 0x1u
#define B3_SNAP_FLAG_DOUBLE_PRECISION 0x2u

// Layout hash over all POD-copied structs + key constants.
// Changing a struct size updates this, catching ABI drift early.
static uint32_t b3ComputeLayoutHash( void )
{
	uint32_t h = 2166136261u;
#define MIX( x )                                                                                                                 \
	h ^= (uint32_t)( x );                                                                                                        \
	h *= 16777619u;
	MIX( sizeof( b3Body ) )
	MIX( sizeof( b3BodySim ) )
	MIX( sizeof( b3BodyState ) )
	MIX( sizeof( b3Shape ) )
	MIX( sizeof( b3Contact ) )
	MIX( sizeof( b3Manifold ) )
	MIX( sizeof( b3Joint ) )
	MIX( sizeof( b3JointSim ) )
	MIX( sizeof( b3Island ) )
	MIX( sizeof( b3IslandSim ) )
	MIX( sizeof( b3ContactLink ) )
	MIX( sizeof( b3JointLink ) )
	MIX( sizeof( b3Sensor ) )
	MIX( sizeof( b3Visitor ) )
	MIX( sizeof( b3SolverSet ) )
	MIX( sizeof( b3GraphColor ) )
	MIX( sizeof( b3DynamicTree ) )
	MIX( sizeof( b3TreeNode ) )
	MIX( sizeof( b3SetItem ) )
	MIX( sizeof( b3IdPool ) )
	MIX( sizeof( b3SurfaceMaterial ) )
	MIX( sizeof( b3ContactSpec ) )
	MIX( sizeof( b3TriangleCache ) )
	MIX( sizeof( v3BlockGridPairControl ) )
	MIX( sizeof( v3BlockGridPairPatch ) )
	MIX( sizeof( b3ContactMaterial ) )
	MIX( v3_blockGridPairCorrelationPolicyVersion )
	MIX( B3_GRAPH_COLOR_COUNT )
	MIX( b3_bodyTypeCount )
	MIX( sizeof( void* ) )
#undef MIX
	return h;
}

typedef struct b3SnapHeader
{
	uint32_t magic;
	uint32_t version;
	uint32_t layoutHash;
	uint32_t flags;
	uint16_t materialPolicyKind;
	uint16_t materialPolicyVersion;
} b3SnapHeader;

_Static_assert( sizeof( b3SnapHeader ) == 20, "snapshot header must be 20 bytes" );

// Bounds-checked read cursor
typedef struct b3SnapReader
{
	const uint8_t* data;
	int cursor;
	int size;
	bool ok;
} b3SnapReader;

static void b3SnapRCheck( b3SnapReader* r, int need )
{
	if ( need < 0 || (int64_t)r->cursor + (int64_t)need > (int64_t)r->size )
	{
		r->ok = false;
	}
}

static void b3SnapR_Bytes( b3SnapReader* r, void* dst, int n )
{
	b3SnapRCheck( r, n );
	if ( !r->ok )
	{
		return;
	}
	memcpy( dst, r->data + r->cursor, n );
	r->cursor += n;
}

static int b3SnapR_I32( b3SnapReader* r )
{
	int32_t v = 0;
	b3SnapR_Bytes( r, &v, 4 );
	return (int)v;
}

static uint32_t b3SnapR_U32( b3SnapReader* r )
{
	uint32_t v = 0;
	b3SnapR_Bytes( r, &v, 4 );
	return v;
}

static uint8_t b3SnapR_U8( b3SnapReader* r )
{
	uint8_t v = 0;
	b3SnapR_Bytes( r, &v, 1 );
	return v;
}

static uint16_t b3SnapR_U16( b3SnapReader* r )
{
	uint16_t v = 0;
	b3SnapR_Bytes( r, &v, 2 );
	return v;
}

static uint64_t b3SnapR_U64( b3SnapReader* r )
{
	uint64_t v = 0;
	b3SnapR_Bytes( r, &v, 8 );
	return v;
}

static float b3SnapR_F32( b3SnapReader* r )
{
	float v = 0.0f;
	b3SnapR_Bytes( r, &v, 4 );
	return v;
}

static b3Vec3 b3SnapR_Vec3( b3SnapReader* r )
{
	b3Vec3 v;
	v.x = b3SnapR_F32( r );
	v.y = b3SnapR_F32( r );
	v.z = b3SnapR_F32( r );
	return v;
}

static b3Quat b3SnapR_Quat( b3SnapReader* r )
{
	b3Quat q;
	q.v = b3SnapR_Vec3( r );
	q.s = b3SnapR_F32( r );
	return q;
}

static b3Transform b3SnapR_Transform( b3SnapReader* r )
{
	b3Transform transform;
	transform.p = b3SnapR_Vec3( r );
	transform.q = b3SnapR_Quat( r );
	return transform;
}

static void b3SnapW_I32( b3RecBuffer* buf, int v )
{
	int32_t w = (int32_t)v;
	b3RecBufAppend( buf, &w, 4 );
}

static void b3SnapW_U32( b3RecBuffer* buf, uint32_t v )
{
	b3RecBufAppend( buf, &v, 4 );
}

static void b3SnapW_U8( b3RecBuffer* buf, uint8_t v )
{
	b3RecBufAppend( buf, &v, 1 );
}

static void b3SnapW_U16( b3RecBuffer* buf, uint16_t v )
{
	b3RecBufAppend( buf, &v, 2 );
}

static void b3SnapW_U64( b3RecBuffer* buf, uint64_t v )
{
	b3RecBufAppend( buf, &v, 8 );
}

static void b3SnapW_F32( b3RecBuffer* buf, float v )
{
	b3RecBufAppend( buf, &v, 4 );
}

static void b3SnapW_Vec3( b3RecBuffer* buf, b3Vec3 v )
{
	b3SnapW_F32( buf, v.x );
	b3SnapW_F32( buf, v.y );
	b3SnapW_F32( buf, v.z );
}

static void b3SnapW_Quat( b3RecBuffer* buf, b3Quat q )
{
	b3SnapW_Vec3( buf, q.v );
	b3SnapW_F32( buf, q.s );
}

static void b3SnapW_Transform( b3RecBuffer* buf, b3Transform transform )
{
	b3SnapW_Vec3( buf, transform.p );
	b3SnapW_Quat( buf, transform.q );
}

static void b3SnapW_Bytes( b3RecBuffer* buf, const void* src, int n )
{
	b3RecBufAppend( buf, src, n );
}

// Bounds check before allocating from image
static bool b3SnapCheckCount( const b3SnapReader* r, int count, int memSize, int minStreamBytes )
{
	if ( count < 0 || memSize < 0 || minStreamBytes < 0 )
	{
		return false;
	}
	if ( memSize > 0 && count > 0x7FFFFFFF / memSize )
	{
		return false;
	}
	int64_t remaining = (int64_t)r->size - (int64_t)r->cursor;
	return (int64_t)count * (int64_t)minStreamBytes <= remaining;
}

// POD array: count + raw bytes
#define b3SerPodArray( buf, arr )                                                                                                \
	do                                                                                                                           \
	{                                                                                                                            \
		b3SnapW_I32( buf, ( arr ).count );                                                                                       \
		if ( ( arr ).count > 0 )                                                                                                 \
		{                                                                                                                        \
			b3SnapW_Bytes( buf, ( arr ).data, ( arr ).count * (int)sizeof( *( arr ).data ) );                                    \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

#define b3DesPodArray( r, arr )                                                                                                  \
	do                                                                                                                           \
	{                                                                                                                            \
		int cnt = b3SnapR_I32( r );                                                                                              \
		int elemSize = (int)sizeof( *( arr ).data );                                                                             \
		if ( ( r )->ok && b3SnapCheckCount( r, cnt, elemSize, elemSize ) == false )                                              \
		{                                                                                                                        \
			( r )->ok = false;                                                                                                   \
		}                                                                                                                        \
		if ( ( r )->ok && cnt > 0 )                                                                                              \
		{                                                                                                                        \
			b3Array_Resize( arr, cnt );                                                                                          \
			b3SnapR_Bytes( r, ( arr ).data, cnt * elemSize );                                                                    \
		}                                                                                                                        \
		else if ( ( r )->ok )                                                                                                    \
		{                                                                                                                        \
			( arr ).count = 0;                                                                                                   \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Capacity-aware encoding is limited to arrays whose retained storage affects the next BlockGrid
// contact transition. It writes only live elements while restoring the exact allocation size.
#define b3SerCapacityArray( buf, arr )                                                                                           \
	do                                                                                                                           \
	{                                                                                                                            \
		b3SnapW_I32( buf, ( arr ).capacity );                                                                                    \
		b3SnapW_I32( buf, ( arr ).count );                                                                                       \
		if ( ( arr ).count > 0 )                                                                                                 \
		{                                                                                                                        \
			b3SnapW_Bytes( buf, ( arr ).data, ( arr ).count * (int)sizeof( *( arr ).data ) );                                    \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

#define b3DesCapacityArray( r, arr )                                                                                             \
	do                                                                                                                           \
	{                                                                                                                            \
		int cap = b3SnapR_I32( r );                                                                                              \
		int cnt = b3SnapR_I32( r );                                                                                              \
		int elemSize = (int)sizeof( *( arr ).data );                                                                             \
		if ( ( r )->ok && ( cap < 0 || cnt < 0 || cnt > cap || cap > INT_MAX / elemSize ||                                       \
							b3SnapCheckCount( r, cnt, elemSize, elemSize ) == false ) )                                          \
		{                                                                                                                        \
			( r )->ok = false;                                                                                                   \
		}                                                                                                                        \
		b3Array_Destroy( arr );                                                                                                  \
		if ( ( r )->ok && cap > 0 )                                                                                              \
		{                                                                                                                        \
			( arr ).data = b3Alloc( (size_t)cap * (size_t)elemSize );                                                            \
			( arr ).capacity = cap;                                                                                              \
			( arr ).count = cnt;                                                                                                 \
			if ( cnt > 0 )                                                                                                       \
			{                                                                                                                    \
				b3SnapR_Bytes( r, ( arr ).data, cnt * elemSize );                                                                \
			}                                                                                                                    \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Id pool: nextIndex + freeArray
static void b3SerIdPool( b3RecBuffer* buf, const b3IdPool* pool )
{
	b3SnapW_I32( buf, pool->nextIndex );
	b3SerPodArray( buf, pool->freeArray );
}

static void b3DesIdPool( b3SnapReader* r, b3IdPool* pool )
{
	pool->nextIndex = b3SnapR_I32( r );
	b3DesPodArray( r, pool->freeArray );
}

// BitSet: blockCount + raw words
static void b3SerBitSet( b3RecBuffer* buf, const b3BitSet* bs )
{
	b3SnapW_U32( buf, bs->blockCount );
	if ( bs->blockCount > 0 )
	{
		b3SnapW_Bytes( buf, bs->bits, (int)( bs->blockCount * sizeof( uint64_t ) ) );
	}
}

static void b3DesBitSet( b3SnapReader* r, b3BitSet* bs )
{
	uint32_t blockCount = b3SnapR_U32( r );
	if ( r->ok && b3SnapCheckCount( r, (int)blockCount, (int)sizeof( uint64_t ), (int)sizeof( uint64_t ) ) == false )
	{
		r->ok = false;
	}
	b3DestroyBitSet( bs );
	if ( !r->ok )
	{
		return;
	}
	uint32_t blockCapacity = blockCount > 0 ? blockCount : 1;
	bs->bits = (uint64_t*)b3Alloc( blockCapacity * sizeof( uint64_t ) );
	memset( bs->bits, 0, blockCapacity * sizeof( uint64_t ) );
	bs->blockCapacity = blockCapacity;
	bs->blockCount = blockCount;
	if ( blockCount > 0 )
	{
		b3SnapR_Bytes( r, bs->bits, (int)( blockCount * sizeof( uint64_t ) ) );
	}
}

// HashSet: capacity + count + raw items (probe order depends on layout)
static void b3SerHashSet( b3RecBuffer* buf, const b3HashSet* hs )
{
	b3SnapW_U32( buf, hs->capacity );
	b3SnapW_U32( buf, hs->count );
	if ( hs->capacity > 0 )
	{
		b3SnapW_Bytes( buf, hs->items, (int)( hs->capacity * sizeof( b3SetItem ) ) );
	}
}

static void b3DesHashSet( b3SnapReader* r, b3HashSet* hs )
{
	uint32_t cap = b3SnapR_U32( r );
	uint32_t cnt = b3SnapR_U32( r );
	bool valid = b3SnapCheckCount( r, (int)cap, (int)sizeof( b3SetItem ), (int)sizeof( b3SetItem ) ) &&
				 ( cap & ( cap - 1 ) ) == 0 && cnt <= cap;
	if ( r->ok && valid == false && ( cap != 0 || cnt != 0 ) )
	{
		r->ok = false;
	}
	b3DestroySet( hs );
	if ( !r->ok )
	{
		return;
	}
	if ( cap > 0 )
	{
		hs->items = (b3SetItem*)b3Alloc( cap * sizeof( b3SetItem ) );
		hs->capacity = cap;
		hs->count = cnt;
		b3SnapR_Bytes( r, hs->items, (int)( cap * sizeof( b3SetItem ) ) );
	}
	else
	{
		hs->items = NULL;
		hs->capacity = 0;
		hs->count = 0;
	}
}

// DynamicTree: version, scalars, full nodeCapacity nodes, and retained rebuild scratch capacity.
static void b3SerTree( b3RecBuffer* buf, const b3DynamicTree* tree )
{
	b3SnapW_Bytes( buf, &tree->version, sizeof( uint64_t ) );
	b3SnapW_I32( buf, tree->root );
	b3SnapW_I32( buf, tree->nodeCount );
	b3SnapW_I32( buf, tree->nodeCapacity );
	b3SnapW_I32( buf, tree->freeList );
	b3SnapW_I32( buf, tree->proxyCount );
	b3SnapW_I32( buf, tree->rebuildCapacity );
	if ( tree->nodeCapacity > 0 )
	{
		b3SnapW_Bytes( buf, tree->nodes, tree->nodeCapacity * (int)sizeof( b3TreeNode ) );
	}
}

static void b3DesTree( b3SnapReader* r, b3DynamicTree* tree )
{
	uint64_t version = 0;
	b3SnapR_Bytes( r, &version, sizeof( uint64_t ) );
	int root = b3SnapR_I32( r );
	int nodeCount = b3SnapR_I32( r );
	int nodeCapacity = b3SnapR_I32( r );
	int freeList = b3SnapR_I32( r );
	int proxyCount = b3SnapR_I32( r );
	int rebuildCapacity = b3SnapR_I32( r );

	if ( r->ok && ( b3SnapCheckCount( r, nodeCapacity, (int)sizeof( b3TreeNode ), (int)sizeof( b3TreeNode ) ) == false ||
					rebuildCapacity < 0 || rebuildCapacity > INT_MAX / (int)sizeof( b3Vec3 ) ) )
	{
		r->ok = false;
	}

	// Free existing allocation including any rebuild scratch
	b3Free( tree->nodes, (size_t)tree->nodeCapacity * sizeof( b3TreeNode ) );
	b3Free( tree->leafIndices, (size_t)tree->rebuildCapacity * sizeof( int ) );
	b3Free( tree->leafBoxes, (size_t)tree->rebuildCapacity * sizeof( b3AABB ) );
	b3Free( tree->leafCenters, (size_t)tree->rebuildCapacity * sizeof( b3Vec3 ) );
	b3Free( tree->binIndices, (size_t)tree->rebuildCapacity * sizeof( int ) );
	tree->nodes = NULL;
	tree->leafIndices = NULL;
	tree->leafBoxes = NULL;
	tree->leafCenters = NULL;
	tree->binIndices = NULL;
	tree->nodeCapacity = 0;
	tree->rebuildCapacity = 0;

	if ( !r->ok )
	{
		return;
	}

	tree->version = version;
	tree->root = root;
	tree->nodeCount = nodeCount;
	tree->nodeCapacity = nodeCapacity;
	tree->freeList = freeList;
	tree->proxyCount = proxyCount;
	tree->rebuildCapacity = rebuildCapacity;

	if ( nodeCapacity > 0 )
	{
		tree->nodes = (b3TreeNode*)b3Alloc( nodeCapacity * (int)sizeof( b3TreeNode ) );
		b3SnapR_Bytes( r, tree->nodes, nodeCapacity * (int)sizeof( b3TreeNode ) );
	}
	if ( rebuildCapacity > 0 )
	{
		// B3_TREE_HEURISTIC is zero in dynamic_tree.c, so current rebuild storage consists of these
		// two arrays. The other heuristic's leafBoxes and binIndices remain null.
		tree->leafIndices = (int*)b3Alloc( rebuildCapacity * (int)sizeof( int ) );
		tree->leafCenters = (b3Vec3*)b3Alloc( rebuildCapacity * (int)sizeof( b3Vec3 ) );
	}
}

// Solver set: setIndex + 4 arrays (note: contactIndices is int array, not contactSims)
static void b3SerSolverSet( b3RecBuffer* buf, const b3SolverSet* set )
{
	b3SnapW_I32( buf, set->setIndex );
	b3SerPodArray( buf, set->bodySims );
	b3SerPodArray( buf, set->bodyStates );
	b3SerPodArray( buf, set->jointSims );
	b3SerCapacityArray( buf, set->contactIndices );
	b3SerPodArray( buf, set->islandSims );
}

static void b3DesSolverSet( b3SnapReader* r, b3SolverSet* set )
{
	set->setIndex = b3SnapR_I32( r );
	b3DesPodArray( r, set->bodySims );
	b3DesPodArray( r, set->bodyStates );
	b3DesPodArray( r, set->jointSims );
	b3DesCapacityArray( r, set->contactIndices );
	b3DesPodArray( r, set->islandSims );
}

static void b3SerNames( b3RecBuffer* buf, const b3NameCache* cache )
{
	b3SnapW_I32( buf, cache->entries.count );
	int count = cache->entries.count;
	for ( int i = 0; i < count; ++i )
	{
		const b3NameEntry* entry = cache->entries.data + i;
		b3SnapW_U32( buf, entry->hash );
		b3SnapW_I32( buf, entry->length );
		b3RecBufAppend( buf, entry->name, entry->length );
	}
}

static void b3DesNames( b3SnapReader* r, b3NameCache* cache )
{
	int count = b3SnapR_I32( r );

	if ( r->ok && b3SnapCheckCount( r, count, (int)sizeof( b3NameEntry ), 8 ) == false )
	{
		r->ok = false;
	}

	if ( r->ok == false )
	{
		return;
	}

	b3Array_Reserve( cache->entries, count );
	for ( int i = 0; i < count; ++i )
	{
		uint32_t hash = b3SnapR_U32( r );
		int length = b3SnapR_I32( r );
		if ( r->ok == false || length < 0 || length > r->size - r->cursor )
		{
			r->ok = false;
			return;
		}
		char* name = b3Alloc( length + 1 );
		b3SnapR_Bytes( r, name, length );
		name[length] = 0;
		b3LoadName( cache, hash, name, length );
	}
}

// Graph color: bodySet (non-overflow only) + jointSims + convexContacts + contacts
static void b3SerGraphColor( b3RecBuffer* buf, const b3GraphColor* color, bool isOverflow )
{
	if ( !isOverflow )
	{
		b3SerBitSet( buf, &color->bodySet );
	}
	b3SerPodArray( buf, color->jointSims );
	b3SerCapacityArray( buf, color->convexContacts );
	b3SerCapacityArray( buf, color->contacts );
	// wideConstraints / manifoldConstraints / contactConstraints are transient, not serialized
}

static void b3DesGraphColor( b3SnapReader* r, b3GraphColor* color, bool isOverflow )
{
	if ( !isOverflow )
	{
		b3DesBitSet( r, &color->bodySet );
	}
	b3DesPodArray( r, color->jointSims );
	b3DesCapacityArray( r, color->convexContacts );
	b3DesCapacityArray( r, color->contacts );
	// Transient pointers left at NULL/0 from shell
}

typedef struct b3SnapArenaCapacity
{
	int backing;
	int overflowRecords;
	int peakDemand;
} b3SnapArenaCapacity;

static void b3SerTaskArenaCapacities( b3RecBuffer* buf, const b3World* world )
{
	b3SnapW_I32( buf, world->taskContexts.count );
	for ( int i = 0; i < world->taskContexts.count; ++i )
	{
		const b3Arena* arena = &world->taskContexts.data[i].arena;
		b3SnapW_I32( buf, arena->capacity );
		b3SnapW_I32( buf, arena->shared->overflows.capacity );
		b3SnapW_I32( buf, arena->shared->peakDemand );
	}
}

static void b3DesTaskArenaCapacities( b3SnapReader* r, b3World* world )
{
	int sourceCount = b3SnapR_I32( r );
	b3SnapArenaCapacity capacities[B3_MAX_WORKERS] = { 0 };
	if ( r->ok == false || sourceCount < 1 || sourceCount > B3_MAX_WORKERS ||
		 b3SnapCheckCount( r, sourceCount, (int)sizeof( b3SnapArenaCapacity ), 3 * (int)sizeof( int ) ) == false )
	{
		r->ok = false;
		return;
	}

	for ( int i = 0; i < sourceCount; ++i )
	{
		b3SnapArenaCapacity* capacity = capacities + i;
		capacity->backing = b3SnapR_I32( r );
		capacity->overflowRecords = b3SnapR_I32( r );
		capacity->peakDemand = b3SnapR_I32( r );
		if ( capacity->backing < 8 || capacity->overflowRecords < 0 ||
			 capacity->overflowRecords > INT_MAX / (int)sizeof( b3OverflowBlock ) || capacity->peakDemand < 0 ||
			 capacity->peakDemand > capacity->backing )
		{
			r->ok = false;
		}
	}
	if ( r->ok == false )
	{
		return;
	}

	int destinationCount = world->taskContexts.count;
	for ( int i = 0; i < destinationCount; ++i )
	{
		int backing = 128 * 1024;
		int overflowRecords = 0;
		int peakDemand = 0;
		if ( i < sourceCount )
		{
			backing = capacities[i].backing;
			overflowRecords = capacities[i].overflowRecords;
			peakDemand = capacities[i].peakDemand;
		}

		b3Arena* arena = &world->taskContexts.data[i].arena;
		b3DestroyArena( arena );
		*arena = b3CreateArena( backing );
		b3Array_Reserve( arena->shared->overflows, overflowRecords );
		arena->shared->peakDemand = peakDemand;
	}
}

// World simulation scalars (never host/callback/worker state)
static void b3SerWorldConfig( b3RecBuffer* buf, const b3World* world )
{
	b3SnapW_Bytes( buf, &world->gravity, sizeof( b3Vec3 ) );
	b3SnapW_Bytes( buf, &world->hitEventThreshold, sizeof( float ) );
	b3SnapW_Bytes( buf, &world->restitutionThreshold, sizeof( float ) );
	b3SnapW_Bytes( buf, &world->maxLinearSpeed, sizeof( float ) );
	b3SnapW_Bytes( buf, &world->maxAngularSpeed, sizeof( float ) );
	b3SnapW_I32( buf, world->projectileCandidateCap );
	b3SnapW_Bytes( buf, &world->contactSpeed, sizeof( float ) );
	b3SnapW_Bytes( buf, &world->contactHertz, sizeof( float ) );
	b3SnapW_Bytes( buf, &world->contactDampingRatio, sizeof( float ) );
	b3SnapW_Bytes( buf, &world->contactRecycleDistance, sizeof( float ) );
	b3SnapW_Bytes( buf, &world->stepIndex, sizeof( uint64_t ) );
	b3SnapW_I32( buf, world->splitIslandId );
	b3SnapW_Bytes( buf, &world->inv_h, sizeof( float ) );
	b3SnapW_Bytes( buf, &world->inv_dt, sizeof( float ) );
	b3SnapW_I32( buf, world->endEventArrayIndex );
	b3SnapW_Bytes( buf, &world->maxCapacity, sizeof( b3Capacity ) );
	uint8_t flags = 0;
	flags |= world->enableSleep ? 0x01u : 0u;
	flags |= world->enableWarmStarting ? 0x02u : 0u;
	flags |= world->enableContinuous ? 0x04u : 0u;
	flags |= world->enableSpeculative ? 0x08u : 0u;
	b3RecBufAppend( buf, &flags, 1 );
}

static void b3DesWorldConfig( b3SnapReader* r, b3World* world )
{
	b3SnapR_Bytes( r, &world->gravity, sizeof( b3Vec3 ) );
	b3SnapR_Bytes( r, &world->hitEventThreshold, sizeof( float ) );
	b3SnapR_Bytes( r, &world->restitutionThreshold, sizeof( float ) );
	b3SnapR_Bytes( r, &world->maxLinearSpeed, sizeof( float ) );
	b3SnapR_Bytes( r, &world->maxAngularSpeed, sizeof( float ) );
	world->projectileCandidateCap = b3SnapR_I32( r );
	b3SnapR_Bytes( r, &world->contactSpeed, sizeof( float ) );
	b3SnapR_Bytes( r, &world->contactHertz, sizeof( float ) );
	b3SnapR_Bytes( r, &world->contactDampingRatio, sizeof( float ) );
	b3SnapR_Bytes( r, &world->contactRecycleDistance, sizeof( float ) );
	b3SnapR_Bytes( r, &world->stepIndex, sizeof( uint64_t ) );
	world->splitIslandId = b3SnapR_I32( r );
	b3SnapR_Bytes( r, &world->inv_h, sizeof( float ) );
	b3SnapR_Bytes( r, &world->inv_dt, sizeof( float ) );
	world->endEventArrayIndex = b3SnapR_I32( r );
	b3SnapR_Bytes( r, &world->maxCapacity, sizeof( b3Capacity ) );
	uint8_t flags = 0;
	b3SnapR_Bytes( r, &flags, 1 );
	world->enableSleep = ( flags & 0x01u ) != 0;
	world->enableWarmStarting = ( flags & 0x02u ) != 0;
	world->enableContinuous = ( flags & 0x04u ) != 0;
	world->enableSpeculative = ( flags & 0x08u ) != 0;
}

// Shapes carry pointer fields: materials, userData, userShape, and the geometry union.
// Serialize the POD scalars with pointers nulled, then the owned materials array, then geometry.
// A single material lives inline in the struct image.
// Hull/mesh/heightField/compound are interned into the recording registry; sphere/capsule inline.
static void b3SerShapes( b3RecBuffer* buf, b3World* world, b3Recording* rec )
{
	int count = world->shapes.count;
	b3SnapW_I32( buf, count );

	for ( int i = 0; i < count; ++i )
	{
		b3Shape shape = world->shapes.data[i];
		bool isLive = ( shape.id == i );

		// Null out pointer fields before writing the raw struct
		shape.materials = NULL;
		shape.userData = NULL;
		shape.userShape = NULL;
		// Zero the geometry union so free-slot images have deterministic bytes
		if ( !isLive )
		{
			memset( &shape.capsule, 0, sizeof( shape.capsule ) );
		}
		b3SnapW_Bytes( buf, &shape, sizeof( b3Shape ) );

		if ( !isLive )
		{
			// Free slot: no materials or geometry
			b3SnapW_I32( buf, 0 );	// materialCount
			b3SnapW_I32( buf, -1 ); // geometry kind sentinel
			continue;
		}

		// Owned material array. Only multi material meshes and compounds have one. A single material
		// already rode along inline in the struct image, so write a zero length for it.
		const b3Shape* src = world->shapes.data + i;
		if ( src->materials != NULL )
		{
			b3SnapW_I32( buf, src->materialCount );
			b3SnapW_Bytes( buf, src->materials, src->materialCount * (int)sizeof( b3SurfaceMaterial ) );
		}
		else
		{
			b3SnapW_I32( buf, 0 );
		}

		// Geometry
		switch ( src->type )
		{
			case b3_sphereShape:
				b3SnapW_I32( buf, (int)b3_sphereShape );
				b3SnapW_Bytes( buf, &src->sphere, sizeof( b3Sphere ) );
				break;
			case b3_capsuleShape:
				b3SnapW_I32( buf, (int)b3_capsuleShape );
				b3SnapW_Bytes( buf, &src->capsule, sizeof( b3Capsule ) );
				break;
			case b3_hullShape:
			{
				b3SnapW_I32( buf, (int)b3_hullShape );
				uint32_t gid = b3RecInternHull( rec, src->hull );
				b3SnapW_U32( buf, gid );
				break;
			}
			case b3_meshShape:
			{
				b3SnapW_I32( buf, (int)b3_meshShape );
				uint32_t gid = b3RecInternMesh( rec, src->mesh.data );
				b3SnapW_U32( buf, gid );
				b3SnapW_Bytes( buf, &src->mesh.scale, sizeof( b3Vec3 ) );
				break;
			}
			case b3_heightShape:
			{
				b3SnapW_I32( buf, (int)b3_heightShape );
				uint32_t gid = b3RecInternHeightField( rec, src->heightField );
				b3SnapW_U32( buf, gid );
				break;
			}
			case b3_compoundShape:
			{
				b3SnapW_I32( buf, (int)b3_compoundShape );
				uint32_t gid = b3RecInternCompound( rec, src->compound );
				b3SnapW_U32( buf, gid );
				break;
			}
			case v3_blockGridShape:
			{
				b3SnapW_I32( buf, (int)v3_blockGridShape );
				uint32_t gid = b3RecInternBlockGrid( rec, src->blockGrid );
				b3SnapW_U32( buf, gid );

				// Store placement with the shape. The geometry registry zeroes the
				// mutable header fields so identical geometry shares a slot.
				b3SnapW_I32( buf, src->blockGrid->worldOriginX );
				b3SnapW_I32( buf, src->blockGrid->worldOriginY );
				b3SnapW_I32( buf, src->blockGrid->worldOriginZ );
				b3SnapW_I32( buf, src->blockGrid->placement );
				break;
			}
			default:
				// A live shape must have a known geometry type. Fail loudly rather than emit a shape
				// with no geometry that would silently lose its collision on restore.
				B3_ASSERT( false );
				b3SnapW_I32( buf, -1 );
				break;
		}
	}
}

static void b3DesShapes( b3SnapReader* r, b3World* world, b3RecReader* rdr )
{
	int count = b3SnapR_I32( r );
	if ( r->ok && b3SnapCheckCount( r, count, (int)sizeof( b3Shape ), (int)sizeof( b3Shape ) ) == false )
	{
		r->ok = false;
	}
	if ( !r->ok )
	{
		return;
	}

	// Save renderer handles before the array is wiped. A keyframe restore is a deterministic replay
	// state, so a live shape that still occupies the same slot with the same generation is the same
	// shape with the same geometry. Carrying its handle over avoids tearing down and rebuilding every
	// GPU mesh on each seek, which the host (a 3D renderer) would otherwise pay for. Handles that are
	// not reclaimed below belong to shapes that are gone or were replaced, and get released so the host
	// pool does not leak across seeks. Box2D has no such handles, so its restore skips all of this.
	int oldShapeCount = world->shapes.count;
	void** savedUserShape = NULL;
	uint16_t* savedGeneration = NULL;
	if ( oldShapeCount > 0 )
	{
		savedUserShape = (void**)b3Alloc( (size_t)oldShapeCount * sizeof( void* ) );
		savedGeneration = (uint16_t*)b3Alloc( (size_t)oldShapeCount * sizeof( uint16_t ) );
		for ( int i = 0; i < oldShapeCount; ++i )
		{
			b3Shape* old = world->shapes.data + i;
			bool oldLive = ( old->id == i );
			savedUserShape[i] = oldLive ? old->userShape : NULL;
			savedGeneration[i] = old->generation;
		}
	}

	b3Array_Resize( world->shapes, count );
	if ( count > 0 )
	{
		memset( world->shapes.data, 0, (size_t)count * sizeof( b3Shape ) );
	}
	for ( int i = 0; i < count; ++i )
	{
		world->shapes.data[i].id = B3_NULL_INDEX;
	}

	for ( int i = 0; i < count && r->ok; ++i )
	{
		b3Shape* dst = world->shapes.data + i;
		b3SnapR_Bytes( r, dst, sizeof( b3Shape ) );
		// Pointer fields were written as NULL; set them cleanly
		dst->materials = NULL;
		dst->userData = NULL;
		dst->userShape = NULL;
		memset( &dst->capsule, 0, sizeof( dst->capsule ) );

		bool isLive = ( dst->id == i );

		// Carry the renderer handle over when the same shape still occupies this slot. Consumed
		// handles are nulled so the teardown sweep only releases the ones that vanished.
		if ( isLive && i < oldShapeCount && savedUserShape != NULL && savedUserShape[i] != NULL &&
			 savedGeneration[i] == dst->generation )
		{
			dst->userShape = savedUserShape[i];
			savedUserShape[i] = NULL;
		}

		// Serializer writes: matCount, matData, geoKind, geoData
		int matCount = b3SnapR_I32( r );

		if ( !r->ok )
		{
			break;
		}

		if ( !isLive )
		{
			// Free slot: matCount=0, geoKind=-1
			(void)matCount;
			b3SnapR_I32( r ); // consume the geoKind sentinel
			continue;
		}

		// Owned material array (written before geoKind in serializer). A zero length means the single
		// material is already inline in the restored struct image, so leave materialCount as restored.
		if ( matCount > 0 )
		{
			if ( b3SnapCheckCount( r, matCount, (int)sizeof( b3SurfaceMaterial ), (int)sizeof( b3SurfaceMaterial ) ) == false )
			{
				r->ok = false;
				break;
			}
			dst->materialCount = matCount;
			dst->materials = (b3SurfaceMaterial*)b3Alloc( (size_t)matCount * sizeof( b3SurfaceMaterial ) );
			b3SnapR_Bytes( r, dst->materials, matCount * (int)sizeof( b3SurfaceMaterial ) );
		}
		else
		{
			dst->materials = NULL;
		}

		int geoKind = b3SnapR_I32( r );

		// Geometry
		switch ( (b3ShapeType)geoKind )
		{
			case b3_sphereShape:
				b3SnapR_Bytes( r, &dst->sphere, sizeof( b3Sphere ) );
				break;
			case b3_capsuleShape:
				b3SnapR_Bytes( r, &dst->capsule, sizeof( b3Capsule ) );
				break;
			case b3_hullShape:
			{
				uint32_t gid = b3SnapR_U32( r );
				if ( !r->ok )
				{
					break;
				}
				if ( rdr == NULL || gid >= (uint32_t)rdr->slotCount )
				{
					r->ok = false;
					break;
				}
				// Hull is cloned into the world DB; pass raw bytes directly
				b3RegistrySlot* slot = rdr->slots + gid;
				dst->hull = b3AddHullToDatabase( world, (const b3HullData*)slot->bytes );
				break;
			}
			case b3_meshShape:
			{
				uint32_t gid = b3SnapR_U32( r );
				b3Vec3 scale;
				b3SnapR_Bytes( r, &scale, sizeof( b3Vec3 ) );
				if ( !r->ok )
				{
					break;
				}
				if ( rdr == NULL || gid >= (uint32_t)rdr->slotCount )
				{
					r->ok = false;
					break;
				}
				b3RegistrySlot* slot = rdr->slots + gid;
				// Mesh is a self-contained blob used by reference; point straight at the pristine bytes.
				dst->mesh.data = (const b3MeshData*)slot->bytes;
				dst->mesh.scale = scale;
				break;
			}
			case b3_heightShape:
			{
				uint32_t gid = b3SnapR_U32( r );
				if ( !r->ok )
				{
					break;
				}
				if ( rdr == NULL || gid >= (uint32_t)rdr->slotCount )
				{
					r->ok = false;
					break;
				}
				b3RegistrySlot* slot = rdr->slots + gid;
				// Self-contained blob used by reference; point straight at the pristine bytes.
				dst->heightField = (const b3HeightFieldData*)slot->bytes;
				break;
			}
			case b3_compoundShape:
			{
				uint32_t gid = b3SnapR_U32( r );
				if ( !r->ok )
				{
					break;
				}
				if ( rdr == NULL || gid >= (uint32_t)rdr->slotCount )
				{
					r->ok = false;
					break;
				}
				b3RegistrySlot* slot = rdr->slots + gid;
				if ( slot->live == NULL )
				{
					slot->live = b3Alloc( (size_t)slot->byteCount );
					memcpy( slot->live, slot->bytes, (size_t)slot->byteCount );
					b3ConvertBytesToCompound( (uint8_t*)slot->live, slot->byteCount );
				}
				dst->compound = (const b3CompoundData*)slot->live;
				break;
			}
			case v3_blockGridShape:
			{
				uint32_t gid = b3SnapR_U32( r );
				if ( !r->ok || rdr == NULL || gid >= (uint32_t)rdr->slotCount )
				{
					r->ok = false;
					break;
				}

				// Restore placement from the shape record.
				int originX = b3SnapR_I32( r );
				int originY = b3SnapR_I32( r );
				int originZ = b3SnapR_I32( r );
				int placement = b3SnapR_I32( r );
				if ( r->ok == false )
				{
					break;
				}

				b3RegistrySlot* slot = rdr->slots + gid;
				if ( slot->live == NULL )
				{
					// The payload is pointer-free, so restoring is a copy with no
					// fixups. The registry keeps this clone alive for the whole
					// replay and frees it itself, so it is seeded with one
					// reference standing for the registry.
					slot->live = b3Alloc( (size_t)slot->byteCount );
					memcpy( slot->live, slot->bytes, (size_t)slot->byteCount );
					v3BlockGridAdoptRestored( (v3BlockGridData*)slot->live );
					v3BlockGridRestorePlacement( (v3BlockGridData*)slot->live, originX, originY, originZ, placement );
				}

				v3BlockGridData* live = (v3BlockGridData*)slot->live;
				if ( v3BlockGridPlacementMatches( live, originX, originY, originZ, placement ) )
				{
					// Each restored shape takes its own reference on top of the
					// registry's, matching how a live shape is attached.
					dst->blockGrid = live;
					v3BlockGrid_Retain( dst->blockGrid );
				}
				else
				{
					// This is the same content at a second placement, and since
					// interning deliberately collapses those into a single
					// slot, the shape needs a copy of its own rather than
					// overwriting the trailer the first one is using.
					v3BlockGridData* clone = b3Alloc( (size_t)slot->byteCount );
					memcpy( clone, slot->live, (size_t)slot->byteCount );
					v3BlockGridAdoptRestored( clone );
					v3BlockGridRestorePlacement( clone, originX, originY, originZ, placement );
					dst->blockGrid = clone;
				}

				// A restored grid is attached the same way a live one is, so the world's
				// BlockGrid scratch has to grow for it here too. Restore bypasses
				// v3CreateBlockGridShape, so nothing else would size it and the first sweep
				// against a restored grid would find the scratch too small.
				if ( v3WorldReserveBlockGridScratch( world, dst->blockGrid ) == false )
				{
					r->ok = false;
					break;
				}
				world->blockGridShapeCount += 1;
				break;
			}
			default:
				// Unknown geometry kind means a corrupt or unsupported snapshot. Fail the load instead
				// of leaving a shape with no geometry.
				r->ok = false;
				break;
		}
	}

	// Release handles for shapes that are gone or were replaced this restore, so the host pool and any
	// GPU resources they pinned do not leak across seeks.
	if ( savedUserShape != NULL )
	{
		for ( int i = 0; i < oldShapeCount; ++i )
		{
			if ( savedUserShape[i] != NULL && world->destroyDebugShape != NULL )
			{
				world->destroyDebugShape( savedUserShape[i], world->userDebugShapeContext );
			}
		}
		b3Free( savedUserShape, (size_t)oldShapeCount * sizeof( void* ) );
		b3Free( savedGeneration, (size_t)oldShapeCount * sizeof( uint16_t ) );
	}
}

enum b3SnapContactSlotKind
{
	b3_snapContactFree,
	b3_snapContactOrdinary,
	b3_snapContactBlockGridPair,
};

static void b3SerBlockGridPairCommonContact( b3RecBuffer* buf, const b3Contact* contact )
{
	b3SnapW_I32( buf, contact->setIndex );
	b3SnapW_I32( buf, contact->colorIndex );
	b3SnapW_I32( buf, contact->localIndex );
	for ( int i = 0; i < 2; ++i )
	{
		b3SnapW_I32( buf, contact->edges[i].bodyId );
		b3SnapW_I32( buf, contact->edges[i].prevKey );
		b3SnapW_I32( buf, contact->edges[i].nextKey );
	}
	b3SnapW_I32( buf, contact->shapeIdA );
	b3SnapW_I32( buf, contact->shapeIdB );
	b3SnapW_I32( buf, contact->childIndex );
	b3SnapW_I32( buf, contact->islandId );
	b3SnapW_I32( buf, contact->islandIndex );
	b3SnapW_I32( buf, contact->contactId );
	b3SnapW_U32( buf, contact->flags );
	b3SnapW_Quat( buf, contact->cachedRotationA );
	b3SnapW_Quat( buf, contact->cachedRotationB );
	b3SnapW_Transform( buf, contact->cachedRelativePose );
	b3SnapW_F32( buf, contact->friction );
	b3SnapW_F32( buf, contact->restitution );
	b3SnapW_F32( buf, contact->rollingResistance );
	b3SnapW_Vec3( buf, contact->tangentVelocity );
	b3SnapW_U32( buf, contact->generation );
}

static void b3DesBlockGridPairCommonContact( b3SnapReader* r, b3Contact* contact )
{
	contact->setIndex = b3SnapR_I32( r );
	contact->colorIndex = b3SnapR_I32( r );
	contact->localIndex = b3SnapR_I32( r );
	for ( int i = 0; i < 2; ++i )
	{
		contact->edges[i].bodyId = b3SnapR_I32( r );
		contact->edges[i].prevKey = b3SnapR_I32( r );
		contact->edges[i].nextKey = b3SnapR_I32( r );
	}
	contact->shapeIdA = b3SnapR_I32( r );
	contact->shapeIdB = b3SnapR_I32( r );
	contact->childIndex = b3SnapR_I32( r );
	contact->islandId = b3SnapR_I32( r );
	contact->islandIndex = b3SnapR_I32( r );
	contact->contactId = b3SnapR_I32( r );
	contact->bodySimIndexA = B3_NULL_INDEX;
	contact->bodySimIndexB = B3_NULL_INDEX;
	contact->flags = b3SnapR_U32( r );
	contact->kind = v3_blockGridPairContactKind;
	contact->cachedRotationA = b3SnapR_Quat( r );
	contact->cachedRotationB = b3SnapR_Quat( r );
	contact->cachedRelativePose = b3SnapR_Transform( r );
	contact->friction = b3SnapR_F32( r );
	contact->restitution = b3SnapR_F32( r );
	contact->rollingResistance = b3SnapR_F32( r );
	contact->tangentVelocity = b3SnapR_Vec3( r );
	contact->generation = b3SnapR_U32( r );
}

static void b3SerBlockGridPairManifold( b3RecBuffer* buf, const b3Manifold* manifold, const v3BlockGridPairPatch* patch,
										const b3ContactMaterial* material )
{
	b3SnapW_I32( buf, manifold->pointCount );
	b3SnapW_Vec3( buf, manifold->normal );
	b3SnapW_F32( buf, manifold->twistImpulse );
	b3SnapW_Vec3( buf, manifold->frictionImpulse );
	b3SnapW_Vec3( buf, manifold->rollingImpulse );
	b3SnapW_U32( buf, patch->supportRegionKey );
	b3SnapW_U8( buf, patch->directedSatKey );
	b3SnapW_F32( buf, material->friction );
	b3SnapW_F32( buf, material->restitution );
	b3SnapW_F32( buf, material->rollingResistance );
	b3SnapW_Vec3( buf, material->tangentVelocity );

	for ( int i = 0; i < manifold->pointCount; ++i )
	{
		const b3ManifoldPoint* point = manifold->points + i;
		b3SnapW_U32( buf, patch->hitboxPairKeys[i] );
		b3SnapW_Vec3( buf, point->anchorA );
		b3SnapW_Vec3( buf, point->anchorB );
		b3SnapW_F32( buf, point->separation );
		b3SnapW_F32( buf, point->baseSeparation );
		b3SnapW_F32( buf, point->normalImpulse );
		b3SnapW_F32( buf, point->totalNormalImpulse );
		b3SnapW_F32( buf, point->normalVelocity );
		b3SnapW_U32( buf, point->featureId );
		b3SnapW_I32( buf, point->triangleIndex );
		b3SnapW_U8( buf, point->persisted ? 1 : 0 );
	}
}

static void b3SerBlockGridPairContact( b3RecBuffer* buf, const b3Contact* contact )
{
	const v3BlockGridPairControl* control = &contact->blockGridPair;
	b3SnapW_U64( buf, control->sourceEpochA );
	b3SnapW_U64( buf, control->sourceEpochB );
	b3SnapW_U16( buf, control->correlationPolicy );
	b3SnapW_U8( buf, control->lastOutcome );
	b3SnapW_U8( buf, control->state != NULL ? 1 : 0 );

	if ( control->state == NULL )
	{
		return;
	}

	b3SnapW_U32( buf, v3BlockGridPairCapacityBytes( control->state ) );
	int manifoldCount = v3BlockGridPairManifoldCount( control->state );
	b3SnapW_I32( buf, manifoldCount );

	const b3Manifold* manifolds = v3BlockGridPairManifolds( control->state );
	const v3BlockGridPairPatch* patches = v3BlockGridPairPatches( control->state );
	const b3ContactMaterial* materials = v3BlockGridPairContactMaterials( control->state );
	for ( int i = 0; i < manifoldCount; ++i )
	{
		b3SerBlockGridPairManifold( buf, manifolds + i, patches + i, materials + i );
	}
}

static bool b3DesBlockGridPairManifold( b3SnapReader* r, b3Manifold* manifold, v3BlockGridPairPatch* patch,
										b3ContactMaterial* material )
{
	int pointCount = b3SnapR_I32( r );
	if ( r->ok == false || pointCount <= 0 || pointCount > B3_MAX_MANIFOLD_POINTS )
	{
		r->ok = false;
		return false;
	}

	manifold->pointCount = pointCount;
	manifold->normal = b3SnapR_Vec3( r );
	manifold->twistImpulse = b3SnapR_F32( r );
	manifold->frictionImpulse = b3SnapR_Vec3( r );
	manifold->rollingImpulse = b3SnapR_Vec3( r );
	patch->supportRegionKey = b3SnapR_U32( r );
	patch->directedSatKey = b3SnapR_U8( r );
	material->friction = b3SnapR_F32( r );
	material->restitution = b3SnapR_F32( r );
	material->rollingResistance = b3SnapR_F32( r );
	material->tangentVelocity = b3SnapR_Vec3( r );

	for ( int i = 0; i < pointCount && r->ok; ++i )
	{
		b3ManifoldPoint* point = manifold->points + i;
		patch->hitboxPairKeys[i] = b3SnapR_U32( r );
		point->anchorA = b3SnapR_Vec3( r );
		point->anchorB = b3SnapR_Vec3( r );
		point->separation = b3SnapR_F32( r );
		point->baseSeparation = b3SnapR_F32( r );
		point->normalImpulse = b3SnapR_F32( r );
		point->totalNormalImpulse = b3SnapR_F32( r );
		point->normalVelocity = b3SnapR_F32( r );
		point->featureId = b3SnapR_U32( r );
		point->triangleIndex = b3SnapR_I32( r );
		uint8_t persisted = b3SnapR_U8( r );
		if ( persisted > 1 )
		{
			r->ok = false;
			return false;
		}
		point->persisted = persisted != 0;
	}
	return r->ok;
}

static bool b3DesBlockGridPairContact( b3SnapReader* r, b3World* world, b3Contact* contact )
{
	v3BlockGridPairControl control = { 0 };
	control.sourceEpochA = b3SnapR_U64( r );
	control.sourceEpochB = b3SnapR_U64( r );
	control.correlationPolicy = b3SnapR_U16( r );
	control.lastOutcome = b3SnapR_U8( r );
	uint8_t hasState = b3SnapR_U8( r );
	if ( r->ok == false || hasState > 1 )
	{
		r->ok = false;
		return false;
	}

	contact->blockGridPair = control;
	contact->manifolds = NULL;
	contact->manifoldCount = 0;
	if ( hasState == 0 )
	{
		if ( v3BlockGridPairSnapshotIsValid( world, contact ) == false )
		{
			r->ok = false;
			return false;
		}
		return true;
	}

	uint32_t capacityBytes = b3SnapR_U32( r );
	int manifoldCount = b3SnapR_I32( r );
	if ( r->ok == false || manifoldCount < 0 || manifoldCount > v3_blockGridPairStandardMaxRegions ||
		 b3SnapCheckCount( r, manifoldCount, 1, 130 ) == false )
	{
		r->ok = false;
		return false;
	}

	v3BlockGridPairState* state = v3BlockGridPairTryRestore( world, capacityBytes, manifoldCount );
	if ( state == NULL )
	{
		r->ok = false;
		return false;
	}

	b3Manifold* manifolds = v3BlockGridPairManifolds( state );
	v3BlockGridPairPatch* patches = v3BlockGridPairPatches( state );
	b3ContactMaterial* materials = v3BlockGridPairContactMaterials( state );
	for ( int i = 0; i < manifoldCount && r->ok; ++i )
	{
		b3DesBlockGridPairManifold( r, manifolds + i, patches + i, materials + i );
	}

	if ( r->ok )
	{
		v3BlockGridPairExchangeState( contact, state );
		if ( v3BlockGridPairSnapshotIsValid( world, contact ) )
		{
			return true;
		}
		v3BlockGridPairExchangeState( contact, NULL );
	}
	if ( v3BlockGridPairFreeState( world, state ) == false )
	{
		B3_ASSERT( false );
	}
	r->ok = false;
	return false;
}

// Each slot starts with its storage kind, while only ordinary contacts keep the same-build struct image
static void b3SerContacts( b3RecBuffer* buf, b3World* world )
{
	int count = world->contacts.count;
	b3SnapW_I32( buf, count );

	for ( int i = 0; i < count; ++i )
	{
		const b3Contact* c = world->contacts.data + i;
		bool isLive = ( c->contactId == i );
		if ( isLive == false )
		{
			b3SnapW_U8( buf, b3_snapContactFree );
			b3SnapW_U32( buf, c->generation );
			continue;
		}
		if ( c->kind == v3_blockGridPairContactKind )
		{
			b3SnapW_U8( buf, b3_snapContactBlockGridPair );
			b3SerBlockGridPairCommonContact( buf, c );
			b3SerBlockGridPairContact( buf, c );
			continue;
		}

		b3SnapW_U8( buf, b3_snapContactOrdinary );
		// Ordinary contacts retain the existing same-build struct image
		b3Contact copy = *c;
		copy.manifolds = NULL;
		copy.bodySimIndexA = B3_NULL_INDEX;
		copy.bodySimIndexB = B3_NULL_INDEX;
		if ( copy.kind == b3_meshContactKind )
		{
			copy.meshContact.triangleCache.data = NULL;
			copy.meshContact.triangleCache.count = 0;
			copy.meshContact.triangleCache.capacity = 0;
		}
		b3SnapW_Bytes( buf, &copy, sizeof( b3Contact ) );

		// Manifolds
		b3SnapW_I32( buf, c->manifoldCount );
		if ( c->manifoldCount > 0 && c->manifolds != NULL )
		{
			b3SnapW_Bytes( buf, c->manifolds, c->manifoldCount * (int)sizeof( b3Manifold ) );
		}

		// Mesh triangleCache
		if ( c->kind == b3_meshContactKind )
		{
			b3SnapW_I32( buf, c->meshContact.triangleCache.count );
			if ( c->meshContact.triangleCache.count > 0 )
			{
				b3SnapW_Bytes( buf, c->meshContact.triangleCache.data,
							   c->meshContact.triangleCache.count * (int)sizeof( b3TriangleCache ) );
			}
		}
	}
}

static void b3DesContacts( b3SnapReader* r, b3World* world )
{
	int count = b3SnapR_I32( r );
	if ( r->ok && b3SnapCheckCount( r, count, (int)sizeof( b3Contact ), 5 ) == false )
	{
		r->ok = false;
	}
	if ( !r->ok )
	{
		return;
	}

	b3Array_Resize( world->contacts, count );
	if ( count > 0 )
	{
		memset( world->contacts.data, 0, (size_t)count * sizeof( b3Contact ) );
	}
	for ( int i = 0; i < count; ++i )
	{
		world->contacts.data[i].contactId = B3_NULL_INDEX;
		world->contacts.data[i].setIndex = B3_NULL_INDEX;
	}

	for ( int i = 0; i < count && r->ok; ++i )
	{
		b3Contact* dst = world->contacts.data + i;
		uint8_t slotKind = b3SnapR_U8( r );
		if ( slotKind == b3_snapContactFree )
		{
			dst->generation = b3SnapR_U32( r );
			continue;
		}
		if ( slotKind == b3_snapContactBlockGridPair )
		{
			b3DesBlockGridPairCommonContact( r, dst );
			if ( r->ok == false || dst->contactId != i || b3DesBlockGridPairContact( r, world, dst ) == false )
			{
				dst->contactId = B3_NULL_INDEX;
				r->ok = false;
				break;
			}
			continue;
		}
		if ( slotKind != b3_snapContactOrdinary )
		{
			r->ok = false;
			break;
		}

		b3SnapR_Bytes( r, dst, sizeof( b3Contact ) );
		dst->manifolds = NULL;
		dst->manifoldCount = 0;
		dst->bodySimIndexA = B3_NULL_INDEX;
		dst->bodySimIndexB = B3_NULL_INDEX;

		if ( dst->contactId != i || dst->kind >= b3_contactKindCount || dst->kind == v3_blockGridPairContactKind )
		{
			dst->contactId = B3_NULL_INDEX;
			dst->kind = b3_convexContactKind;
			dst->reserved = 0;
			dst->manifoldCount = 0;
			dst->convexContact = (b3ConvexContact){ 0 };
			r->ok = false;
			break;
		}
		else if ( dst->kind == b3_meshContactKind )
		{
			dst->meshContact.triangleCache.data = NULL;
			dst->meshContact.triangleCache.count = 0;
			dst->meshContact.triangleCache.capacity = 0;
		}

		int manifoldCount = b3SnapR_I32( r );

		if ( !r->ok )
		{
			break;
		}

		if ( manifoldCount > 0 )
		{
			if ( manifoldCount > UINT16_MAX ||
				 b3SnapCheckCount( r, manifoldCount, (int)sizeof( b3Manifold ), (int)sizeof( b3Manifold ) ) == false )
			{
				r->ok = false;
				break;
			}
			dst->manifolds = b3AllocateManifolds( world, manifoldCount );
			dst->manifoldCount = (uint16_t)manifoldCount;
			b3SnapR_Bytes( r, dst->manifolds, manifoldCount * (int)sizeof( b3Manifold ) );
		}
		else
		{
			dst->manifolds = NULL;
			dst->manifoldCount = 0;
		}

		// Mesh triangleCache
		if ( dst->kind == b3_meshContactKind )
		{
			int cacheCount = b3SnapR_I32( r );
			if ( !r->ok )
			{
				break;
			}
			if ( cacheCount > 0 )
			{
				if ( b3SnapCheckCount( r, cacheCount, (int)sizeof( b3TriangleCache ), (int)sizeof( b3TriangleCache ) ) == false )
				{
					r->ok = false;
					break;
				}
				b3Array_Resize( dst->meshContact.triangleCache, cacheCount );
				b3SnapR_Bytes( r, dst->meshContact.triangleCache.data, cacheCount * (int)sizeof( b3TriangleCache ) );
			}
		}
	}
}

// Release owned storage before the sparse arrays are overwritten during replay restore
static void b3FreeLiveSimElements( b3World* world )
{
	// Shape heap: materials and hull DB references
	for ( int i = 0; i < world->shapes.count; ++i )
	{
		b3Shape* s = world->shapes.data + i;
		if ( s->id != i )
		{
			continue;
		}
		// A single material lives inline (materials == NULL). Multi material meshes and compounds own
		// the array, so free it exactly as b3DestroyShapeAllocations does.
		if ( s->materials != NULL )
		{
			b3Free( s->materials, (size_t)s->materialCount * sizeof( b3SurfaceMaterial ) );
			s->materials = NULL;
			s->materialCount = 0;
		}
		// Hull is ref-counted in the world DB; release before overwrite so re-adding is ref-neutral.
		if ( s->type == b3_hullShape && s->hull != NULL )
		{
			b3RemoveHullFromDatabase( world, s->hull );
			s->hull = NULL;
		}
		// A BlockGrid is reference counted as well, and since restore takes a
		// fresh reference for every shape, skipping the release here would let
		// the count climb by one per restore until a replay that seeks
		// repeatedly never frees its grids at all.
		if ( s->type == v3_blockGridShape && s->blockGrid != NULL )
		{
			v3BlockGrid_Release( s->blockGrid );
			s->blockGrid = NULL;
			world->blockGridShapeCount -= 1;
			B3_ASSERT( world->blockGridShapeCount >= 0 );
		}
		// name / userData / userShape are host-owned; do not free
	}

	// Contact storage includes manifolds and any owner selected by the contact kind
	for ( int i = 0; i < world->contacts.count; ++i )
	{
		b3Contact* c = world->contacts.data + i;
		if ( c->contactId == i )
		{
			if ( b3DestroyContactStorage( world, c ) == false )
			{
				B3_ASSERT( false );
			}
		}
	}

	// Sensor heap: inner arrays
	for ( int i = 0; i < world->sensors.count; ++i )
	{
		b3Sensor* sensor = world->sensors.data + i;
		b3Array_Destroy( sensor->hits );
		b3Array_Destroy( sensor->overlaps1 );
		b3Array_Destroy( sensor->overlaps2 );
	}

	// Island heap: inner arrays
	for ( int i = 0; i < world->islands.count; ++i )
	{
		b3Island* island = world->islands.data + i;
		b3Array_Destroy( island->bodies );
		b3Array_Destroy( island->contacts );
		b3Array_Destroy( island->joints );
	}
}

static bool b3MovedProxyStateIsValid( const b3BroadPhase* broadPhase )
{
	int bitCount = 0;
	for ( int type = 0; type < b3_bodyTypeCount; ++type )
	{
		bitCount += b3CountSetBits( (b3BitSet*)( broadPhase->movedProxies + type ) );
	}
	if ( bitCount != broadPhase->moveArray.count )
	{
		return false;
	}

	for ( int i = 0; i < broadPhase->moveArray.count; ++i )
	{
		int proxyKey = broadPhase->moveArray.data[i];
		int type = B3_PROXY_TYPE( proxyKey );
		int proxyId = B3_PROXY_ID( proxyKey );
		if ( proxyKey < 0 || type < 0 || type >= b3_bodyTypeCount || proxyId < 0 ||
			 proxyId >= broadPhase->trees[type].nodeCapacity ||
			 ( broadPhase->trees[type].nodes[proxyId].flags & ( b3_allocatedNode | b3_leafNode ) ) !=
				 ( b3_allocatedNode | b3_leafNode ) ||
			 b3GetBit( broadPhase->movedProxies + type, (uint32_t)proxyId ) == false )
		{
			return false;
		}
		for ( int j = 0; j < i; ++j )
		{
			if ( broadPhase->moveArray.data[j] == proxyKey )
			{
				return false;
			}
		}
	}
	return true;
}

static bool b3CanSerializeRetainedCapacities( const b3World* world )
{
	if ( world->taskContexts.count < 1 || world->taskContexts.count > B3_MAX_WORKERS ||
		 b3MovedProxyStateIsValid( &world->broadPhase ) == false )
	{
		return false;
	}

	for ( int type = 0; type < b3_bodyTypeCount; ++type )
	{
		const b3DynamicTree* tree = world->broadPhase.trees + type;
		if ( tree->rebuildCapacity < 0 || tree->rebuildCapacity > INT_MAX / (int)sizeof( b3Vec3 ) )
		{
			return false;
		}
		bool empty =
			tree->leafIndices == NULL && tree->leafCenters == NULL && tree->leafBoxes == NULL && tree->binIndices == NULL;
		bool currentHeuristic =
			tree->leafIndices != NULL && tree->leafCenters != NULL && tree->leafBoxes == NULL && tree->binIndices == NULL;
		if ( ( tree->rebuildCapacity == 0 && empty == false ) || ( tree->rebuildCapacity > 0 && currentHeuristic == false ) )
		{
			return false;
		}
	}

	for ( int i = 0; i < world->taskContexts.count; ++i )
	{
		const b3Arena* arena = &world->taskContexts.data[i].arena;
		if ( arena->memory == NULL || arena->capacity < 8 || arena->index != 0 || arena->shared == NULL ||
			 arena->shared->maxIndex != 0 || arena->shared->overflowBytes != 0 || arena->shared->overflows.count != 0 ||
			 arena->shared->overflows.capacity < 0 ||
			 arena->shared->overflows.capacity > INT_MAX / (int)sizeof( b3OverflowBlock ) || arena->shared->peakDemand < 0 ||
			 arena->shared->peakDemand > arena->capacity )
		{
			return false;
		}
	}
	return true;
}

static bool b3CanSerializeContacts( const b3World* world )
{
	for ( int i = 0; i < world->contacts.count; ++i )
	{
		const b3Contact* contact = world->contacts.data + i;
		if ( contact->contactId != i )
		{
			continue;
		}
		if ( b3ContactStorageIsValid( contact ) == false ||
			 ( contact->kind == v3_blockGridPairContactKind && v3BlockGridPairSnapshotIsValid( world, contact ) == false ) )
		{
			return false;
		}
	}

	return true;
}

// Cached event IDs can outlive their source. Only the world slot changes on replay.
static void b3SerBlockSide( b3RecBuffer* buf, const v3BlockContactSide* side )
{
	b3SnapW_I32( buf, side->bodyId.index1 );
	b3SnapW_U16( buf, side->bodyId.generation );
	b3SnapW_I32( buf, side->shapeId.index1 );
	b3SnapW_U16( buf, side->shapeId.generation );
	b3SnapW_U8( buf, side->isBlockGrid );
	b3SnapW_I32( buf, side->cellX );
	b3SnapW_I32( buf, side->cellY );
	b3SnapW_I32( buf, side->cellZ );
	b3SnapW_I32( buf, side->subHitboxIndex );
	b3SnapW_U32( buf, side->materialIndex );
	b3SnapW_U64( buf, side->userMaterialId );
	b3SnapW_U64( buf, side->userData );
}

static v3BlockContactSide b3DesBlockSide( b3SnapReader* r, uint16_t worldId )
{
	v3BlockContactSide side = { 0 };
	side.bodyId.index1 = b3SnapR_I32( r );
	side.bodyId.generation = b3SnapR_U16( r );
	side.shapeId.index1 = b3SnapR_I32( r );
	side.shapeId.generation = b3SnapR_U16( r );
	uint8_t isGrid = b3SnapR_U8( r );
	if ( isGrid > 1 )
		r->ok = false;
	side.isBlockGrid = isGrid != 0;
	side.cellX = b3SnapR_I32( r );
	side.cellY = b3SnapR_I32( r );
	side.cellZ = b3SnapR_I32( r );
	side.subHitboxIndex = b3SnapR_I32( r );
	side.materialIndex = b3SnapR_U32( r );
	side.userMaterialId = b3SnapR_U64( r );
	side.userData = b3SnapR_U64( r );
	side.bodyId.world0 = side.shapeId.world0 = worldId;
	if ( side.bodyId.index1 <= 0 || side.shapeId.index1 <= 0 || side.subHitboxIndex < 0 )
		r->ok = false;
	return side;
}

static void b3SerBlockEvent( b3RecBuffer* buf, const v3BlockContactEvent* event )
{
	b3SerBlockSide( buf, &event->sideA );
	b3SerBlockSide( buf, &event->sideB );
	b3SnapW_Bytes( buf, &event->point.x, sizeof( event->point.x ) );
	b3SnapW_Bytes( buf, &event->point.y, sizeof( event->point.y ) );
	b3SnapW_Bytes( buf, &event->point.z, sizeof( event->point.z ) );
	b3SnapW_Vec3( buf, event->normal );
	b3SnapW_F32( buf, event->normalImpulse );
	b3SnapW_F32( buf, event->approachSpeed );
}

static v3BlockContactEvent b3DesBlockEvent( b3SnapReader* r, uint16_t worldId )
{
	v3BlockContactEvent event = { 0 };
	event.sideA = b3DesBlockSide( r, worldId );
	event.sideB = b3DesBlockSide( r, worldId );
	b3SnapR_Bytes( r, &event.point.x, sizeof( event.point.x ) );
	b3SnapR_Bytes( r, &event.point.y, sizeof( event.point.y ) );
	b3SnapR_Bytes( r, &event.point.z, sizeof( event.point.z ) );
	event.normal = b3SnapR_Vec3( r );
	event.normalImpulse = b3SnapR_F32( r );
	event.approachSpeed = b3SnapR_F32( r );
	if ( ( !event.sideA.isBlockGrid && !event.sideB.isBlockGrid ) || !isfinite( event.point.x ) || !isfinite( event.point.y ) ||
		 !isfinite( event.point.z ) || !b3IsValidVec3( event.normal ) || !isfinite( event.normalImpulse ) ||
		 !isfinite( event.approachSpeed ) )
	{
		r->ok = false;
	}
	return event;
}

static void b3SerEventState( b3RecBuffer* buf, const b3World* world )
{
	// Normalize active history and pending ends to slot 0, published ends to slot 1.
	b3SnapW_U8( buf, world->blockContactStateIncomplete );
	b3SnapW_U8( buf, world->blockContactTransitionIncomplete );
	b3SnapW_U64( buf, world->blockGridReplacementPendingCount );
	b3SnapW_U64( buf, world->blockGridPairCounters.candidateHitboxPairCount );
	b3SnapW_U64( buf, world->blockGridPairCounters.touchingPairCount );
	b3SnapW_U64( buf, world->blockGridPairCounters.contactCount );
	b3SnapW_U64( buf, world->blockGridPairCounters.projectileSweepCount );
	b3SnapW_U64( buf, world->blockGridPairCounters.capExhaustionCount );
	b3SnapW_U64( buf, world->blockGridPairCounters.replacementPublishedCount );
	b3SnapW_U64( buf, world->blockGridPairCounters.scratchPeakBytes );
	b3SnapW_U64( buf, world->blockGridPairCounters.contactReductionCount );
	const b3Array( v3BlockContactRecord )* active = world->blockContactStates + world->blockContactStateIndex;
	b3SnapW_I32( buf, active->count );
	for ( int i = 0; i < active->count; ++i )
	{
		b3SerBlockEvent( buf, &active->data[i].event );
		b3SnapW_I32( buf, active->data[i].contactId );
		b3SnapW_U32( buf, active->data[i].contactGeneration );
	}
	int pending = world->blockContactEndEventIndex;
	const b3Array( v3BlockContactEvent ) * arrays[] = { &world->blockContactBeginEvents, &world->blockContactHitEvents,
														world->blockContactEndEvents + pending,
														world->blockContactEndEvents + 1 - pending };
	uint32_t dropped[] = { world->blockContactDroppedBeginCount, world->blockContactDroppedHitCount,
						   world->blockContactDroppedEndCount[pending], world->blockContactDroppedEndCount[1 - pending] };
	for ( int kind = 0; kind < 4; ++kind )
	{
		b3SnapW_U32( buf, dropped[kind] );
		b3SnapW_I32( buf, arrays[kind]->count );
		for ( int i = 0; i < arrays[kind]->count; ++i )
			b3SerBlockEvent( buf, arrays[kind]->data + i );
	}
	b3SnapW_I32( buf, world->contactBeginEvents.capacity );
	for ( int slot = 0; slot < 2; ++slot )
	{
		const b3Array( b3ContactEndTouchEvent )* ends = world->contactEndEvents + slot;
		b3SnapW_I32( buf, ends->capacity );
		b3SnapW_I32( buf, ends->count );
		for ( int i = 0; i < ends->count; ++i )
		{
			const b3ContactEndTouchEvent* e = ends->data + i;
			b3SnapW_I32( buf, e->shapeIdA.index1 );
			b3SnapW_U16( buf, e->shapeIdA.generation );
			b3SnapW_I32( buf, e->shapeIdB.index1 );
			b3SnapW_U16( buf, e->shapeIdB.generation );
			b3SnapW_I32( buf, e->contactId.index1 );
			b3SnapW_U32( buf, e->contactId.generation );
		}
	}
}

static int b3DesBlockEventCount( b3SnapReader* r, int encodedSize )
{
	int count = b3SnapR_I32( r );
	if ( count < 0 || count > V3_BLOCK_CONTACT_EVENT_CAPACITY ||
		 !b3SnapCheckCount( r, count, sizeof( v3BlockContactRecord ), encodedSize ) )
	{
		r->ok = false;
		return 0;
	}
	return count;
}

static void b3DesEventState( b3SnapReader* r, b3World* world )
{
	if ( !r->ok )
		return;
	// Validate the bounded BlockGrid section before replacing live logical contents.
	v3BlockContactRecord active[V3_BLOCK_CONTACT_EVENT_CAPACITY];
	v3BlockContactEvent events[4][V3_BLOCK_CONTACT_EVENT_CAPACITY];
	int counts[4];
	uint32_t dropped[4];
	uint8_t incomplete = b3SnapR_U8( r );
	uint8_t transition = b3SnapR_U8( r );
	uint64_t replacements = b3SnapR_U64( r );
	v3BlockGridPairCounters counters;
	counters.candidateHitboxPairCount = b3SnapR_U64( r );
	counters.touchingPairCount = b3SnapR_U64( r );
	counters.contactCount = b3SnapR_U64( r );
	counters.projectileSweepCount = b3SnapR_U64( r );
	counters.capExhaustionCount = b3SnapR_U64( r );
	counters.replacementPublishedCount = b3SnapR_U64( r );
	counters.scratchPeakBytes = b3SnapR_U64( r );
	counters.contactReductionCount = b3SnapR_U64( r );
	if ( incomplete > 1 || transition > 1 || world->endEventArrayIndex < 0 || world->endEventArrayIndex > 1 )
		r->ok = false;
	// Two 49-byte sides, three position coordinates, and five floats.
	int eventSize = 118 + 3 * (int)sizeof( events[0][0].point.x );
	int activeCount = b3DesBlockEventCount( r, eventSize + 8 );
	for ( int i = 0; i < activeCount && r->ok; ++i )
	{
		active[i].event = b3DesBlockEvent( r, world->worldId );
		active[i].contactId = b3SnapR_I32( r );
		active[i].contactGeneration = b3SnapR_U32( r );
		if ( active[i].contactId < 0 )
			r->ok = false;
	}
	bool needsStorage = activeCount != 0 || incomplete != 0 || transition != 0;
	for ( int kind = 0; kind < 4; ++kind )
	{
		dropped[kind] = b3SnapR_U32( r );
		counts[kind] = b3DesBlockEventCount( r, eventSize );
		needsStorage = needsStorage || counts[kind] != 0 || dropped[kind] != 0;
		for ( int i = 0; i < counts[kind] && r->ok; ++i )
			events[kind][i] = b3DesBlockEvent( r, world->worldId );
	}
	if ( !r->ok )
		return;
	if ( needsStorage && !v3BlockContactEventsReserve( world ) )
	{
		r->ok = false;
		return;
	}
	world->blockContactStateIndex = world->blockContactEndEventIndex = 0;
	world->blockContactStateIncomplete = incomplete != 0;
	world->blockContactTransitionIncomplete = transition != 0;
	world->blockGridReplacementPendingCount = replacements;
	world->blockGridPairCounters = counters;
	world->blockContactStates[0].count = activeCount;
	world->blockContactStates[1].count = 0;
	if ( activeCount > 0 )
		memcpy( world->blockContactStates[0].data, active, activeCount * sizeof( active[0] ) );
	b3Array( v3BlockContactEvent ) * arrays[] = { &world->blockContactBeginEvents, &world->blockContactHitEvents,
												  world->blockContactEndEvents, world->blockContactEndEvents + 1 };
	for ( int kind = 0; kind < 4; ++kind )
	{
		arrays[kind]->count = counts[kind];
		if ( counts[kind] > 0 )
			memcpy( arrays[kind]->data, events[kind], counts[kind] * sizeof( events[kind][0] ) );
	}
	world->blockContactDroppedBeginCount = dropped[0];
	world->blockContactDroppedHitCount = dropped[1];
	world->blockContactDroppedEndCount[0] = dropped[2];
	world->blockContactDroppedEndCount[1] = dropped[3];
	int beginCapacity = b3SnapR_I32( r );
	if ( beginCapacity < 0 || beginCapacity > INT_MAX / (int)sizeof( b3ContactBeginTouchEvent ) )
	{
		r->ok = false;
		return;
	}
	b3Array_Destroy( world->contactBeginEvents );
	if ( beginCapacity > 0 )
	{
		world->contactBeginEvents.data = b3Alloc( (size_t)beginCapacity * sizeof( b3ContactBeginTouchEvent ) );
		world->contactBeginEvents.capacity = beginCapacity;
	}
	for ( int slot = 0; slot < 2; ++slot )
	{
		int capacity = b3SnapR_I32( r );
		int count = b3SnapR_I32( r );
		// Ordinary ends are not capped at the BlockGrid bound. Their bytes bound allocation.
		if ( !r->ok || capacity < 0 || count < 0 || count > capacity ||
			 capacity > INT_MAX / (int)sizeof( b3ContactEndTouchEvent ) ||
			 !b3SnapCheckCount( r, count, sizeof( b3ContactEndTouchEvent ), 20 ) )
		{
			r->ok = false;
			return;
		}
		b3Array( b3ContactEndTouchEvent )* ends = world->contactEndEvents + slot;
		b3Array_Destroy( *ends );
		if ( capacity > 0 )
		{
			ends->data = b3Alloc( (size_t)capacity * sizeof( b3ContactEndTouchEvent ) );
			ends->capacity = capacity;
			ends->count = count;
		}
		for ( int i = 0; i < count; ++i )
		{
			b3ContactEndTouchEvent e = { 0 };
			e.shapeIdA.index1 = b3SnapR_I32( r );
			e.shapeIdA.generation = b3SnapR_U16( r );
			e.shapeIdB.index1 = b3SnapR_I32( r );
			e.shapeIdB.generation = b3SnapR_U16( r );
			e.contactId.index1 = b3SnapR_I32( r );
			e.contactId.generation = b3SnapR_U32( r );
			e.shapeIdA.world0 = e.shapeIdB.world0 = e.contactId.world0 = world->worldId;
			if ( e.shapeIdA.index1 <= 0 || e.shapeIdB.index1 <= 0 || e.contactId.index1 <= 0 )
				r->ok = false;
			ends->data[i] = e;
		}
	}
}

int b3SerializeWorld( b3World* world, b3RecBuffer* buf, b3Recording* rec )
{
	if ( b3CanSerializeContacts( world ) == false || b3CanSerializeRetainedCapacities( world ) == false ||
		 b3WorldUsesBuiltinContactMaterialPolicy( world ) == false )
	{
		return -1;
	}

	int startSize = buf->size;

	// Snapshot header
	b3SnapHeader hdr;
	hdr.magic = B3_SNAP_MAGIC;
	hdr.version = B3_SNAP_VERSION;
	hdr.layoutHash = b3ComputeLayoutHash();
	hdr.flags = B3_ENABLE_VALIDATION ? B3_SNAP_FLAG_VALIDATION : 0u;
	hdr.materialPolicyKind = b3_contactMaterialPolicyBuiltin;
	hdr.materialPolicyVersion = b3_contactMaterialPolicyBuiltinVersion;
#if defined( BOX3D_DOUBLE_PRECISION )
	hdr.flags |= B3_SNAP_FLAG_DOUBLE_PRECISION;
#endif
	b3SnapW_Bytes( buf, &hdr, (int)sizeof( hdr ) );

	// World scalars
	b3SerWorldConfig( buf, world );

	// 6 id pools (Box3D has no chainIdPool)
	b3SerIdPool( buf, &world->bodyIdPool );
	b3SerIdPool( buf, &world->shapeIdPool );
	b3SerIdPool( buf, &world->contactIdPool );
	b3SerIdPool( buf, &world->jointIdPool );
	b3SerIdPool( buf, &world->islandIdPool );
	b3SerIdPool( buf, &world->solverSetIdPool );

	// Solver sets
	int setCount = world->solverSets.count;
	b3SnapW_I32( buf, setCount );
	for ( int i = 0; i < setCount; ++i )
	{
		b3SerSolverSet( buf, world->solverSets.data + i );
	}

	// Sparse body array (userData is host wiring, zero it on the copy)
	{
		int bodyCount = world->bodies.count;
		b3SnapW_I32( buf, bodyCount );
		for ( int i = 0; i < bodyCount; ++i )
		{
			b3Body elem = world->bodies.data[i];
			elem.userData = NULL;
			b3SnapW_Bytes( buf, &elem, sizeof( b3Body ) );
		}
	}

	// Shape sparse array with geometry interning
	b3SerShapes( buf, world, rec );

	// Contact sparse array with manifold and mesh triangleCache
	b3SerContacts( buf, world );

	// Joint sparse array (userData scrubbed)
	{
		int jointCount = world->joints.count;
		b3SnapW_I32( buf, jointCount );
		for ( int i = 0; i < jointCount; ++i )
		{
			b3Joint elem = world->joints.data[i];
			elem.userData = NULL;
			b3SnapW_Bytes( buf, &elem, sizeof( b3Joint ) );
		}
	}

	// Sensors: shapeId + 3 inner arrays each
	{
		int sensorCount = world->sensors.count;
		b3SnapW_I32( buf, sensorCount );
		for ( int i = 0; i < sensorCount; ++i )
		{
			b3Sensor* s = world->sensors.data + i;
			b3SnapW_I32( buf, s->shapeId );
			b3SerPodArray( buf, s->hits );
			b3SerPodArray( buf, s->overlaps1 );
			b3SerPodArray( buf, s->overlaps2 );
		}
	}

	// Islands: 4 scalars + 3 inner arrays each
	{
		int islandCount = world->islands.count;
		b3SnapW_I32( buf, islandCount );
		for ( int i = 0; i < islandCount; ++i )
		{
			b3Island* island = world->islands.data + i;
			b3SnapW_I32( buf, island->setIndex );
			b3SnapW_I32( buf, island->localIndex );
			b3SnapW_I32( buf, island->islandId );
			b3SnapW_I32( buf, island->constraintRemoveCount );
			b3SerPodArray( buf, island->bodies );
			b3SerCapacityArray( buf, island->contacts );
			b3SerPodArray( buf, island->joints );
		}
	}

	// Broad phase
	b3BroadPhase* bp = &world->broadPhase;
	for ( int t = 0; t < b3_bodyTypeCount; ++t )
	{
		b3SerTree( buf, &bp->trees[t] );
	}
	for ( int t = 0; t < b3_bodyTypeCount; ++t )
	{
		b3SerBitSet( buf, &bp->movedProxies[t] );
	}
	b3SerCapacityArray( buf, bp->moveArray );
	b3SerHashSet( buf, &bp->pairSet );

	// Constraint graph
	b3ConstraintGraph* graph = &world->constraintGraph;
	for ( int c = 0; c < B3_GRAPH_COLOR_COUNT; ++c )
	{
		b3SerGraphColor( buf, &graph->colors[c], c == B3_OVERFLOW_INDEX );
	}

	b3SerTaskArenaCapacities( buf, world );
	b3SerNames( buf, &world->names );
	b3SerEventState( buf, world );

	return buf->size - startSize;
}

bool b3DeserializeIntoShell( const uint8_t* data, int size, b3World* world, b3RecReader* rdr )
{
	if ( data == NULL || size < (int)sizeof( b3SnapHeader ) )
	{
		return false;
	}

	// Validate header
	b3SnapHeader hdr;
	memcpy( &hdr, data, sizeof( hdr ) );
	if ( hdr.magic != B3_SNAP_MAGIC || hdr.version != B3_SNAP_VERSION )
	{
		printf( "b3DeserializeIntoShell: bad magic/version\n" );
		return false;
	}
	uint32_t supportedFlags = B3_SNAP_FLAG_VALIDATION | B3_SNAP_FLAG_DOUBLE_PRECISION;
	if ( ( hdr.flags & ~supportedFlags ) != 0 )
	{
		printf( "b3DeserializeIntoShell: unsupported flags\n" );
		return false;
	}
	bool imageDouble = ( hdr.flags & B3_SNAP_FLAG_DOUBLE_PRECISION ) != 0;
#if defined( BOX3D_DOUBLE_PRECISION )
	bool buildDouble = true;
#else
	bool buildDouble = false;
#endif
	if ( imageDouble != buildDouble )
	{
		printf( "b3DeserializeIntoShell: precision mismatch\n" );
		return false;
	}
	if ( hdr.layoutHash != b3ComputeLayoutHash() )
	{
		printf( "b3DeserializeIntoShell: layout hash mismatch\n" );
		return false;
	}
	if ( hdr.materialPolicyKind != b3_contactMaterialPolicyBuiltin ||
		 hdr.materialPolicyVersion != b3_contactMaterialPolicyBuiltinVersion ||
		 b3WorldUsesBuiltinContactMaterialPolicy( world ) == false )
	{
		printf( "b3DeserializeIntoShell: material policy mismatch\n" );
		return false;
	}

	b3SnapReader readerStorage;
	b3SnapReader* r = &readerStorage;
	r->data = data;
	r->cursor = (int)sizeof( b3SnapHeader );
	r->size = size;
	r->ok = true;

	// Free existing per-object heap before overwriting
	b3FreeLiveSimElements( world );

	// 1. World scalars
	b3DesWorldConfig( r, world );

	// 2. 6 id pools; destroy the pre-created sets' pool state first
	b3DesIdPool( r, &world->bodyIdPool );
	b3DesIdPool( r, &world->shapeIdPool );
	b3DesIdPool( r, &world->contactIdPool );
	b3DesIdPool( r, &world->jointIdPool );
	b3DesIdPool( r, &world->islandIdPool );
	b3DesIdPool( r, &world->solverSetIdPool );

	// 3. Solver sets: destroy inner arrays of existing sets first
	for ( int i = 0; i < world->solverSets.count; ++i )
	{
		b3SolverSet* set = world->solverSets.data + i;
		b3Array_Destroy( set->bodySims );
		b3Array_Destroy( set->bodyStates );
		b3Array_Destroy( set->jointSims );
		b3Array_Destroy( set->contactIndices );
		b3Array_Destroy( set->islandSims );
	}

	int setCount = b3SnapR_I32( r );
	if ( r->ok && b3SnapCheckCount( r, setCount, (int)sizeof( b3SolverSet ), 6 * (int)sizeof( int ) ) == false )
	{
		r->ok = false;
	}
	if ( r->ok )
	{
		b3Array_Resize( world->solverSets, setCount );
		if ( setCount > 0 )
		{
			memset( world->solverSets.data, 0, (size_t)setCount * sizeof( b3SolverSet ) );
		}
		for ( int i = 0; i < setCount; ++i )
		{
			world->solverSets.data[i].setIndex = B3_NULL_INDEX;
		}
		for ( int i = 0; i < setCount; ++i )
		{
			b3DesSolverSet( r, world->solverSets.data + i );
		}
	}

	if ( !r->ok )
	{
		return false;
	}

	// 4. Body sparse array
	{
		int bodyCount = b3SnapR_I32( r );
		if ( r->ok && b3SnapCheckCount( r, bodyCount, (int)sizeof( b3Body ), (int)sizeof( b3Body ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b3Array_Resize( world->bodies, bodyCount );
			for ( int i = 0; i < bodyCount; ++i )
			{
				b3SnapR_Bytes( r, world->bodies.data + i, sizeof( b3Body ) );
				world->bodies.data[i].userData = NULL;
			}
		}
	}

	if ( !r->ok )
	{
		return false;
	}

	// 5. Shape sparse array
	b3DesShapes( r, world, rdr );

	if ( !r->ok )
	{
		return false;
	}

	// 6. Contact sparse array
	b3DesContacts( r, world );

	if ( !r->ok )
	{
		return false;
	}

	// 7. Joint sparse array
	{
		int jointCount = b3SnapR_I32( r );
		if ( r->ok && b3SnapCheckCount( r, jointCount, (int)sizeof( b3Joint ), (int)sizeof( b3Joint ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b3Array_Resize( world->joints, jointCount );
			for ( int i = 0; i < jointCount; ++i )
			{
				b3SnapR_Bytes( r, world->joints.data + i, sizeof( b3Joint ) );
				world->joints.data[i].userData = NULL;
			}
		}
	}

	// 8. Sensors
	{
		b3Array_Destroy( world->sensors );
		b3Array_Create( world->sensors );

		int sensorCount = b3SnapR_I32( r );
		if ( r->ok && b3SnapCheckCount( r, sensorCount, (int)sizeof( b3Sensor ), 4 * (int)sizeof( int ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b3Array_Resize( world->sensors, sensorCount );
			if ( sensorCount > 0 )
			{
				memset( world->sensors.data, 0, (size_t)sensorCount * sizeof( b3Sensor ) );
			}
		}

		for ( int i = 0; i < sensorCount && r->ok; ++i )
		{
			b3Sensor* s = world->sensors.data + i;
			s->shapeId = b3SnapR_I32( r );
			b3Array_Create( s->hits );
			b3Array_Create( s->overlaps1 );
			b3Array_Create( s->overlaps2 );
			b3DesPodArray( r, s->hits );
			b3DesPodArray( r, s->overlaps1 );
			b3DesPodArray( r, s->overlaps2 );
		}
	}

	// 9. Islands
	{
		b3Array_Destroy( world->islands );
		b3Array_Create( world->islands );

		int islandCount = b3SnapR_I32( r );
		if ( r->ok && b3SnapCheckCount( r, islandCount, (int)sizeof( b3Island ), 7 * (int)sizeof( int ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b3Array_Resize( world->islands, islandCount );
			if ( islandCount > 0 )
			{
				memset( world->islands.data, 0, (size_t)islandCount * sizeof( b3Island ) );
			}
		}

		for ( int i = 0; i < islandCount && r->ok; ++i )
		{
			b3Island* island = world->islands.data + i;
			island->setIndex = b3SnapR_I32( r );
			island->localIndex = b3SnapR_I32( r );
			island->islandId = b3SnapR_I32( r );
			island->constraintRemoveCount = b3SnapR_I32( r );
			b3Array_Create( island->bodies );
			b3Array_Create( island->contacts );
			b3Array_Create( island->joints );
			b3DesPodArray( r, island->bodies );
			b3DesCapacityArray( r, island->contacts );
			b3DesPodArray( r, island->joints );
		}
	}

	// 10. Broad phase
	{
		b3BroadPhase* bp = &world->broadPhase;

		for ( int t = 0; t < b3_bodyTypeCount; ++t )
		{
			b3DesTree( r, &bp->trees[t] );
		}
		for ( int t = 0; t < b3_bodyTypeCount; ++t )
		{
			b3DesBitSet( r, &bp->movedProxies[t] );
		}

		b3DesCapacityArray( r, bp->moveArray );
		if ( r->ok && b3MovedProxyStateIsValid( bp ) == false )
		{
			r->ok = false;
		}

		b3DesHashSet( r, &bp->pairSet );
		// Transient moveResults/movePairs stay at shell's NULL/0
	}

	// 11. Constraint graph
	{
		b3ConstraintGraph* graph = &world->constraintGraph;
		for ( int c = 0; c < B3_GRAPH_COLOR_COUNT; ++c )
		{
			b3DesGraphColor( r, &graph->colors[c], c == B3_OVERFLOW_INDEX );
		}
	}

	b3DesTaskArenaCapacities( r, world );
	b3DesNames( r, &world->names );
	b3DesEventState( r, world );

	return r->ok && r->cursor == r->size;
}

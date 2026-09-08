// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "manifold.h"
#include "test_macros.h"
#include "block_grid/separation_certifier.h"
#include "block_grid/witness_point.h"

#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>

static const float kRoot2 = 1.41421356f;
static const b3Vec3 kAxisX = { 1.0f, 0.0f, 0.0f };
static const b3Vec3 kAxisY = { 0.0f, 1.0f, 0.0f };
static const b3Vec3 kAxisZ = { 0.0f, 0.0f, 1.0f };

// b3ComputeCosSin is a coarse rational approximation, so the fixtures build their rotations from
// libm to keep the analytic edge and face positions exact.
static b3Quat ExactQuat( b3Vec3 axis, float radians )
{
	float half = 0.5f * radians;
	float s = sinf( half );
	return (b3Quat){ { s * axis.x, s * axis.y, s * axis.z }, cosf( half ) };
}

static b3Transform ExactRotation( b3Vec3 axis, float radians )
{
	return (b3Transform){ b3Vec3_zero, ExactQuat( axis, radians ) };
}

// The local x edges are nearly parallel. Their cross axis follows tiltAxis while the twist keeps
// that axis distinct from every face normal.
static b3Quat CachedEdgeRotation( float tilt )
{
	b3Vec3 tiltAxis = { 0.0f, cosf( 2.8423846f ), sinf( 2.8423846f ) };
	return b3MulQuat( ExactQuat( tiltAxis, tilt ), ExactQuat( kAxisX, 3.5171536f ) );
}

static bool CachedEdgeFrame( b3Quat rotation1, b3Quat rotation2, double t0, double t1, v3PairIntervalFrame* frame )
{
	b3Vec3 centerB = { -0.78193786f, -1.16055842f, 1.48409746f };
	b3Sweep sweepA = { .q1 = b3Quat_identity, .q2 = b3Quat_identity };
	b3Sweep sweepB = { .c1 = centerB, .c2 = centerB, .q1 = rotation1, .q2 = rotation2 };
	v3SweepPoly polyA, polyB;
	return v3SweepPolyBuild( &sweepA, &polyA ) && v3SweepPolyBuild( &sweepB, &polyB ) &&
		   v3PairIntervalFrameBuild( &polyA, &polyB, t0, t1, frame );
}

// Directed separation along a unit axis n pointing from A to B. Positive means B clears A along n.
// This is the projection gap min over B minus max over A, which is exactly what a valid separating
// axis measures, so the winning face or edge axis must reproduce it.
static float SepAlong( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB, b3Vec3 n )
{
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	float maxA = -FLT_MAX;
	for ( int i = 0; i < hullA->vertexCount; ++i )
	{
		maxA = b3MaxFloat( maxA, b3Dot( n, pointsA[i] ) );
	}

	const b3Vec3* pointsB = b3GetHullPoints( hullB );
	float minB = FLT_MAX;
	for ( int i = 0; i < hullB->vertexCount; ++i )
	{
		minB = b3MinFloat( minB, b3Dot( n, b3TransformPoint( xfB, pointsB[i] ) ) );
	}

	return minB - maxA;
}

static void TryAxis( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB, b3Vec3 n, float* best,
					 b3Vec3* bestNormal )
{
	float s = SepAlong( hullA, hullB, xfB, n );
	if ( s > *best )
	{
		*best = s;
		*bestNormal = n;
	}
}

// Brute force SAT. The maximum separation over every face normal and every edge cross product is
// the true answer the SIMD query has to match. Each undirected axis is tried both ways so the
// winner comes out oriented from A to B with no sign bookkeeping.
static float OracleSeparation( const b3HullData* hullA, const b3HullData* hullB, b3Transform xfB, b3Vec3* outNormal )
{
	float best = -FLT_MAX;
	b3Vec3 bestNormal = { 0.0f, 0.0f, 0.0f };

	const b3Plane* planesA = b3GetHullPlanes( hullA );
	for ( int i = 0; i < hullA->faceCount; ++i )
	{
		TryAxis( hullA, hullB, xfB, planesA[i].normal, &best, &bestNormal );
		TryAxis( hullA, hullB, xfB, b3Neg( planesA[i].normal ), &best, &bestNormal );
	}

	const b3Plane* planesB = b3GetHullPlanes( hullB );
	for ( int i = 0; i < hullB->faceCount; ++i )
	{
		b3Vec3 n = b3RotateVector( xfB.q, planesB[i].normal );
		TryAxis( hullA, hullB, xfB, n, &best, &bestNormal );
		TryAxis( hullA, hullB, xfB, b3Neg( n ), &best, &bestNormal );
	}

	const b3HullHalfEdge* edgesA = b3GetHullEdges( hullA );
	const b3Vec3* pointsA = b3GetHullPoints( hullA );
	const b3HullHalfEdge* edgesB = b3GetHullEdges( hullB );
	const b3Vec3* pointsB = b3GetHullPoints( hullB );

	// Half edges are stored as adjacent twin pairs, so the even index and its successor bound one edge.
	for ( int i = 0; i < hullA->edgeCount; i += 2 )
	{
		b3Vec3 dirA = b3Sub( pointsA[edgesA[i + 1].origin], pointsA[edgesA[i].origin] );
		for ( int j = 0; j < hullB->edgeCount; j += 2 )
		{
			b3Vec3 dirB = b3RotateVector( xfB.q, b3Sub( pointsB[edgesB[j + 1].origin], pointsB[edgesB[j].origin] ) );
			b3Vec3 cross = b3Cross( dirA, dirB );

			// Near parallel edges give no useful axis and cannot be the true maximum.
			if ( b3Dot( cross, cross ) < 1e-10f )
			{
				continue;
			}

			cross = b3Normalize( cross );
			TryAxis( hullA, hullB, xfB, cross, &best, &bestNormal );
			TryAxis( hullA, hullB, xfB, b3Neg( cross ), &best, &bestNormal );
		}
	}

	*outNormal = bestNormal;
	return best;
}

// Two axis aligned cubes pushed apart along x. A's plus x face is the separating axis, the gap is
// large enough to trip the speculative early out, and the reference vertex on B is its minus x side.
static int FaceAxisASeparatedTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3Transform xfB = { { 1.2f, 0.0f, 0.0f }, b3Quat_identity };
	b3AxisQuery aq = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB, true );
	b3SeparatingAxis q = b3GetBestAxis( &aq );

	ENSURE( q.type == b3_faceAxisA );
	ENSURE_SMALL( q.separation - 0.2f, 1e-5f );
	ENSURE_SMALL( q.normal.x - 1.0f, 1e-6f );
	ENSURE_SMALL( q.normal.y, 1e-6f );
	ENSURE_SMALL( q.normal.z, 1e-6f );

	const b3Plane* planesA = b3GetHullPlanes( &hullA.base );
	ENSURE_SMALL( planesA[q.indexA].normal.x - 1.0f, 1e-6f );

	const b3Vec3* pointsB = b3GetHullPoints( &hullB.base );
	ENSURE_SMALL( pointsB[q.indexB].x + 0.5f, 1e-6f );

	ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 1e-5f );
	return 0;
}

// A yawed 45 degrees about z so all of its faces sit oblique to x. Only B still has a face square to
// the gap, so the reference has to be B's minus x face. This is the case that ties when both hulls
// are aligned, and the tie is what a face A bias would hide.
static int FaceAxisBSeparatedTest( void )
{
	b3BoxHull hullA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisZ, 0.25f * B3_PI ) );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	float aExtent = 0.5f * kRoot2;
	float gap = 0.2f;
	float d = aExtent + 0.5f + gap;
	b3Transform xfB = { { d, 0.0f, 0.0f }, b3Quat_identity };

	b3AxisQuery aq = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB, true );
	b3SeparatingAxis q = b3GetBestAxis( &aq );

	ENSURE( q.type == b3_faceAxisB );
	ENSURE_SMALL( q.separation - gap, 1e-5f );
	ENSURE_SMALL( q.normal.x - 1.0f, 1e-5f );
	ENSURE_SMALL( q.normal.y, 1e-5f );
	ENSURE_SMALL( q.normal.z, 1e-5f );

	const b3Plane* planesB = b3GetHullPlanes( &hullB.base );
	ENSURE_SMALL( planesB[q.indexB].normal.x + 1.0f, 1e-5f );

	const b3Vec3* pointsA = b3GetHullPoints( &hullA.base );
	ENSURE_SMALL( pointsA[q.indexA].x - aExtent, 1e-4f );

	ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 1e-4f );
	return 0;
}

// Well beyond the speculative distance the face phase must report the axis and separation without
// running the edge phase. A sign slip in the early out would read as deep overlap.
static int FaceFarSeparatedTest( void )
{
	b3BoxHull hullA = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3Transform xfB = { { 3.0f, 0.0f, 0.0f }, b3Quat_identity };
	b3AxisQuery aq = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB, true );
	b3SeparatingAxis q = b3GetBestAxis( &aq );

	ENSURE( q.type == b3_faceAxisA );
	ENSURE_SMALL( q.separation - 2.0f, 1e-5f );
	ENSURE_SMALL( q.normal.x - 1.0f, 1e-6f );

	b3Vec3 oracleNormal;
	float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );
	ENSURE_SMALL( q.separation - oracleSep, 1e-5f );
	return 0;
}

// The support bias is derived from each hull's AABB, not from an origin assumed to sit inside it. A
// box built far from its own local origin is the case the old diagonal bias got wrong. A is yawed
// and pushed out to x = 5, so the face B query has to run getSupport over vertices clustered near
// x = 5 and still pick the right one.
static int OffsetFaceAxisBTest( void )
{
	float cx = 5.0f;
	b3Transform placeA = { { cx, 0.0f, 0.0f }, ExactQuat( kAxisZ, 0.25f * B3_PI ) };
	b3BoxHull hullA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, placeA );
	b3BoxHull hullB = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	float aExtent = cx + 0.5f * kRoot2;
	float gap = 0.2f;
	float d = aExtent + 0.5f + gap;
	b3Transform xfB = { { d, 0.0f, 0.0f }, b3Quat_identity };

	b3AxisQuery aq = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB, true );
	b3SeparatingAxis q = b3GetBestAxis( &aq );

	ENSURE( q.type == b3_faceAxisB );
	ENSURE_SMALL( q.separation - gap, 1e-4f );
	ENSURE_SMALL( q.normal.x - 1.0f, 1e-5f );
	ENSURE_SMALL( q.normal.y, 1e-5f );
	ENSURE_SMALL( q.normal.z, 1e-5f );

	const b3Plane* planesB = b3GetHullPlanes( &hullB.base );
	ENSURE_SMALL( planesB[q.indexB].normal.x + 1.0f, 1e-5f );

	const b3Vec3* pointsA = b3GetHullPoints( &hullA.base );
	ENSURE_SMALL( pointsA[q.indexA].x - aExtent, 1e-4f );

	b3Vec3 oracleNormal;
	float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );
	ENSURE_SMALL( q.separation - oracleSep, 1e-4f );
	ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 1e-4f );
	return 0;
}

// Cube A yawed 45 about y presents an edge along y at x = h*root2. Cube B rolled 45 about z presents
// an edge along z at x = -h*root2. Sliding B along x makes those edges the closest features, so the
// axis is x and the separation is d - root2. The sweep straddles contact into overlap while staying
// under the speculative distance, so no face axis can short circuit the edge phase. The overlap rows
// pin the maximum separation, which is where a premature edge early out shows up.
static int EdgePairSweepTest( void )
{
	b3BoxHull hullA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisY, 0.25f * B3_PI ) );
	b3BoxHull hullB = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, ExactRotation( kAxisZ, 0.25f * B3_PI ) );

	float distances[] = { 1.43f, 1.42f, kRoot2, 1.40f, 1.38f, 1.35f, 1.30f };

	for ( int i = 0; i < ARRAY_COUNT( distances ); ++i )
	{
		float d = distances[i];
		float expected = d - kRoot2;

		b3Transform xfB = { { d, 0.0f, 0.0f }, b3Quat_identity };
		b3AxisQuery aq = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB, true );
		b3SeparatingAxis q = b3GetBestAxis( &aq );

		ENSURE( q.type == b3_edgePairAxis );
		ENSURE_SMALL( q.separation - expected, 1e-4f );
		ENSURE_SMALL( q.normal.x - 1.0f, 1e-4f );
		ENSURE_SMALL( q.normal.y, 1e-4f );
		ENSURE_SMALL( q.normal.z, 1e-4f );

		// Half edge indices are the even twin of each pair.
		ENSURE( ( q.indexA & 1 ) == 0 );
		ENSURE( ( q.indexB & 1 ) == 0 );

		b3Vec3 oracleNormal;
		float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );
		ENSURE_SMALL( q.separation - oracleSep, 1e-4f );
		ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 1e-4f );
	}

	return 0;
}

// These boxes overlap on every accepted family but have a positive raw gap on family 6. Its short
// cross axis is below the edge cutoff, so remembering it cannot strengthen the result.
static int CachedDegenerateEdgeFamilyTest( void )
{
	v3PairIntervalFrame frame;
	b3Quat rotation = CachedEdgeRotation( 5.0e-4f );
	ENSURE( CachedEdgeFrame( rotation, rotation, 0.0, 0.0, &frame ) );
	double halfExtentA[3] = { 0.90249028, 0.25391016, 0.11520207 };
	double halfExtentB[3] = { 0.12106524, 0.34135800, 1.60601002 };
	double centerOffset[3] = { 0.0, 0.0, 0.0 };
	v3Interval rawGap = v3ObbSatFamilyGap( &frame, halfExtentA, centerOffset, halfExtentB, centerOffset, 6 );

	v3SeparationWitness uncached = V3_SEPARATION_WITNESS_NONE;
	v3SeparationVerdict uncachedVerdict =
		v3SeparationCertify( &frame, halfExtentA, centerOffset, halfExtentB, centerOffset, 0.0, &uncached );
	v3SeparationWitness cached = { .family = 6 };
	v3SeparationVerdict cachedVerdict =
		v3SeparationCertify( &frame, halfExtentA, centerOffset, halfExtentB, centerOffset, 0.0, &cached );

	ENSURE( rawGap.lo > 0.0 );
	ENSURE( v3ObbSatCertifySeparation( &frame, halfExtentA, centerOffset, halfExtentB, centerOffset, 0.0 ) == -1 );
	ENSURE( uncachedVerdict == v3_notCertified );
	ENSURE( cachedVerdict == uncachedVerdict );
	return 0;
}

// A remembered face remains eligible for the first certifier tier.
static int CachedFaceFamilyTest( void )
{
	b3Vec3 centerB = { 3.0f, 0.0f, 0.0f };
	b3Sweep sweepA = { .q1 = b3Quat_identity, .q2 = b3Quat_identity };
	b3Sweep sweepB = { .c1 = centerB, .c2 = centerB, .q1 = b3Quat_identity, .q2 = b3Quat_identity };
	v3SweepPoly polyA, polyB;
	ENSURE( v3SweepPolyBuild( &sweepA, &polyA ) );
	ENSURE( v3SweepPolyBuild( &sweepB, &polyB ) );
	v3PairIntervalFrame frame;
	ENSURE( v3PairIntervalFrameBuild( &polyA, &polyB, 0.0, 0.0, &frame ) );

	double halfExtent[3] = { 0.5, 0.5, 0.5 };
	double centerOffset[3] = { 0.0, 0.0, 0.0 };
	v3SeparationWitness cached = { .family = 0 };
	ENSURE( v3SeparationCertify( &frame, halfExtent, centerOffset, halfExtent, centerOffset, 0.0, &cached ) ==
			v3_certifiedSeparated );
	ENSURE( cached.tier == v3_separationTierCached );
	ENSURE( cached.familyTests == 1 );
	return 0;
}

// One continuous rotation moves family 6 from above the edge cutoff to below it. The early point
// keeps the remembered fast path, while the narrow final interval must agree with an empty witness.
static int CachedEdgeOrientationChangeTest( void )
{
	double halfExtentA[3] = { 0.90249028, 0.25391016, 0.11520207 };
	double halfExtentB[3] = { 0.12106524, 0.34135800, 1.60601002 };
	double centerOffset[3] = { 0.0, 0.0, 0.0 };
	b3Quat validRotation = CachedEdgeRotation( 2.0e-3f );
	b3Quat degenerateRotation = CachedEdgeRotation( 5.0e-4f );

	v3PairIntervalFrame validFrame;
	ENSURE( CachedEdgeFrame( validRotation, degenerateRotation, 0.0, 0.0, &validFrame ) );
	v3SeparationWitness valid = { .family = 6 };
	ENSURE( v3SeparationCertify( &validFrame, halfExtentA, centerOffset, halfExtentB, centerOffset, 0.0, &valid ) ==
			v3_certifiedSeparated );
	ENSURE( valid.tier == v3_separationTierCached );
	ENSURE( valid.familyTests == 1 );

	v3PairIntervalFrame degenerateFrame;
	ENSURE( CachedEdgeFrame( validRotation, degenerateRotation, 0.999999, 1.0, &degenerateFrame ) );
	v3Interval rawGap = v3ObbSatFamilyGap( &degenerateFrame, halfExtentA, centerOffset, halfExtentB, centerOffset, 6 );
	v3SeparationWitness uncached = V3_SEPARATION_WITNESS_NONE;
	v3SeparationWitness cached = { .family = 6 };
	v3SeparationVerdict uncachedVerdict =
		v3SeparationCertify( &degenerateFrame, halfExtentA, centerOffset, halfExtentB, centerOffset, 0.0, &uncached );
	v3SeparationVerdict cachedVerdict =
		v3SeparationCertify( &degenerateFrame, halfExtentA, centerOffset, halfExtentB, centerOffset, 0.0, &cached );
	ENSURE( rawGap.lo > 0.0 );
	ENSURE( v3ObbSatCertifySeparation( &degenerateFrame, halfExtentA, centerOffset, halfExtentB, centerOffset, 0.0 ) == -1 );
	ENSURE( uncachedVerdict == v3_notCertified );
	ENSURE( cachedVerdict == uncachedVerdict );
	return 0;
}

// The parked witness point primitive at explicit poses. Two unit boxes given by a body origin plus
// a centre offset. Every coordinate is an exact binary fraction so the expected witness point is
// exact. A is centred on the origin (-0.25 + 0.25), B is either overlapping at 0.25 or separated at
// 1.25, which leaves a 0.25 m gap. Alternating projection puts the witness for the separated pair
// at the midpoint of the closest points, 0.5 and 0.75, so 0.625, which is 0.125 m outside each box.
// Certification inflates each box by delta over root three, not by delta, and the refusing delta is
// chosen to tell those two apart: 0.1875 / sqrt(3) is 0.108, which cannot bridge the 0.125 m
// overhang, while a plain 0.1875 inflation would reach it and wrongly certify. 0.5 m certifies
// either way, so it only pins the positive direction.
static int WitnessPointPoseTest( void )
{
	const double identity[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	const double halfExtent[3] = { 0.5, 0.5, 0.5 };
	// Body origin and centre offset are distinct so the offset rotation path is exercised
	const double originA[3] = { -0.25, 0.0, 0.0 };
	const double offsetA[3] = { 0.25, 0.0, 0.0 };
	const double zeroOffset[3] = { 0.0, 0.0, 0.0 };

	const double overlappingOriginB[3] = { 0.25, 0.0, 0.0 };
	v3WitnessPointResult overlapping;
	ENSURE( v3WitnessCertifyContactAtPose( identity, originA, halfExtent, offsetA, identity, overlappingOriginB, halfExtent,
										   zeroOffset, 0.0, &overlapping ) );
	ENSURE( overlapping.certified );
	ENSURE_SMALL( overlapping.point[0], 1e-12 );
	ENSURE_SMALL( overlapping.point[1], 1e-12 );
	ENSURE_SMALL( overlapping.point[2], 1e-12 );

	const double separatedOriginB[3] = { 1.25, 0.0, 0.0 };
	v3WitnessPointResult refused;
	ENSURE( v3WitnessCertifyContactAtPose( identity, originA, halfExtent, offsetA, identity, separatedOriginB, halfExtent,
										   zeroOffset, 0.1875, &refused ) == false );
	ENSURE( refused.certified == false );

	v3WitnessPointResult inflated;
	ENSURE( v3WitnessCertifyContactAtPose( identity, originA, halfExtent, offsetA, identity, separatedOriginB, halfExtent,
										   zeroOffset, 0.5, &inflated ) );
	ENSURE( inflated.certified );
	ENSURE_SMALL( inflated.point[0] - 0.625, 1e-12 );
	return 0;
}

// The same two boxes reached through the sweep path instead of explicit poses, which is how the
// certifier used to call the witness primitive. Both sweeps are stationary with identity rotation,
// so the pose at t = 0 must reproduce WitnessPointPoseTest exactly: the sweep centre is c1 and the
// box centre is c1 + centerOffset - localCenter, which for A is -0.25 + 0.5 - 0.25 = 0. Same
// verdicts, same witness point, and the same 0.1875 refusing delta that separates an inflation of
// delta over root three from an inflation of delta.
static int WitnessPointSweepTest( void )
{
	const double halfExtent[3] = { 0.5, 0.5, 0.5 };
	const double offsetA[3] = { 0.5, 0.0, 0.0 };
	const double zeroOffset[3] = { 0.0, 0.0, 0.0 };

	b3Sweep sweepA = { .localCenter = { 0.25f, 0.0f, 0.0f },
					   .c1 = { -0.25f, 0.0f, 0.0f },
					   .c2 = { -0.25f, 0.0f, 0.0f },
					   .q1 = b3Quat_identity,
					   .q2 = b3Quat_identity };
	b3Sweep overlappingB = {
		.c1 = { 0.25f, 0.0f, 0.0f }, .c2 = { 0.25f, 0.0f, 0.0f }, .q1 = b3Quat_identity, .q2 = b3Quat_identity };
	b3Sweep separatedB = {
		.c1 = { 1.25f, 0.0f, 0.0f }, .c2 = { 1.25f, 0.0f, 0.0f }, .q1 = b3Quat_identity, .q2 = b3Quat_identity };

	v3SweepPoly polyA, polyOverlapping, polySeparated;
	ENSURE( v3SweepPolyBuild( &sweepA, &polyA ) );
	ENSURE( v3SweepPolyBuild( &overlappingB, &polyOverlapping ) );
	ENSURE( v3SweepPolyBuild( &separatedB, &polySeparated ) );

	v3WitnessPointResult overlapping;
	ENSURE( v3WitnessCertifyContact( &polyA, &polyOverlapping, halfExtent, offsetA, halfExtent, zeroOffset, 0.0, 0.0,
									 &overlapping ) );
	ENSURE( overlapping.certified );
	ENSURE_SMALL( overlapping.point[0], 1e-12 );

	v3WitnessPointResult refused;
	ENSURE( v3WitnessCertifyContact( &polyA, &polySeparated, halfExtent, offsetA, halfExtent, zeroOffset, 0.0, 0.1875,
									 &refused ) == false );
	ENSURE( refused.certified == false );

	v3WitnessPointResult inflated;
	ENSURE( v3WitnessCertifyContact( &polyA, &polySeparated, halfExtent, offsetA, halfExtent, zeroOffset, 0.0, 0.5, &inflated ) );
	ENSURE( inflated.certified );
	ENSURE_SMALL( inflated.point[0] - 0.625, 1e-12 );
	return 0;
}

// The parked advance machinery on a separated pair, every expected value derived by hand from the
// fixture rather than pinned from a run.
//
// A is a stationary unit box at the origin. B is a unit box translating from x = 1.5 to x = 2.0 with
// no rotation, so dc = (0.5, 0, 0). Every coordinate is an exact binary fraction.
//
// Corner envelope: with q1 == q2 the quaternion chord is zero, so the rotation term vanishes for any
// rho and the envelope is pure translation, |dc| times the width of the time range. Over [0, 1] that
// is 0.5, over [0, 0.5] it is 0.25, and for stationary A it is 0 whatever rho is.
//
// Gap at a point frame: identity rotations make family 0 A's x face axis, so the true gap is the
// centre distance less the two half extents. At t = 0 that is 1.5 - 0.5 - 0.5 = 0.5. At t = 0.75 the
// centre is 1.5 + 0.75 * 0.5 = 1.875, so the gap is 0.875.
//
// Advance: with no rotation the closing rate lambda is just |dcB - dcA| = 0.5, and the step is
// (gap - sigma) / lambda clamped to the time left in the step. From t = 0 with gap 0.25 and sigma
// 0.0625 that is 0.1875 / 0.5 = 0.375. From t = 0.75 the same arithmetic gives 0.375 but only 0.25
// of the step remains, so the clamp must win. A gap that does not clear sigma admits no advance.
static int SeparationAdvanceEnvelopeTest( void )
{
	const double halfExtent[3] = { 0.5, 0.5, 0.5 };
	const double zeroOffset[3] = { 0.0, 0.0, 0.0 };

	b3Sweep sweepA = { .c1 = { 0.0f, 0.0f, 0.0f }, .c2 = { 0.0f, 0.0f, 0.0f }, .q1 = b3Quat_identity, .q2 = b3Quat_identity };
	b3Sweep sweepB = { .c1 = { 1.5f, 0.0f, 0.0f }, .c2 = { 2.0f, 0.0f, 0.0f }, .q1 = b3Quat_identity, .q2 = b3Quat_identity };
	v3SweepPoly polyA, polyB;
	ENSURE( v3SweepPolyBuild( &sweepA, &polyA ) );
	ENSURE( v3SweepPolyBuild( &sweepB, &polyB ) );

	// Translation only, so rho cannot change the envelope
	ENSURE_SMALL( v3SweepPolyCornerEnvelope( &polyA, 0.0, 0.0, 1.0 ), 1e-12 );
	ENSURE_SMALL( v3SweepPolyCornerEnvelope( &polyA, 3.0, 0.0, 1.0 ), 1e-12 );
	ENSURE_SMALL( v3SweepPolyCornerEnvelope( &polyB, 0.0, 0.0, 1.0 ) - 0.5, 1e-12 );
	ENSURE_SMALL( v3SweepPolyCornerEnvelope( &polyB, 3.0, 0.0, 1.0 ) - 0.5, 1e-12 );
	ENSURE_SMALL( v3SweepPolyCornerEnvelope( &polyB, 3.0, 0.0, 0.5 ) - 0.25, 1e-12 );

	v3PairIntervalFrame frameAtStart;
	ENSURE( v3PairIntervalFrameBuild( &polyA, &polyB, 0.0, 0.0, &frameAtStart ) );
	ENSURE_SMALL( v3SeparationGapAtTime( &frameAtStart, halfExtent, zeroOffset, halfExtent, zeroOffset, 0 ) - 0.5, 1e-12 );

	v3PairIntervalFrame frameAtThreeQuarters;
	ENSURE( v3PairIntervalFrameBuild( &polyA, &polyB, 0.75, 0.75, &frameAtThreeQuarters ) );
	ENSURE_SMALL( v3SeparationGapAtTime( &frameAtThreeQuarters, halfExtent, zeroOffset, halfExtent, zeroOffset, 0 ) - 0.875,
				  1e-12 );

	// (0.25 - 0.0625) / 0.5, and rho is inert because neither body rotates
	ENSURE_SMALL( v3SeparationAdvance( &polyA, &polyB, &frameAtStart, 0.0, 0.0, 0.25, 0.0625 ) - 0.375, 1e-12 );
	ENSURE_SMALL( v3SeparationAdvance( &polyA, &polyB, &frameAtStart, 2.0, 3.0, 0.25, 0.0625 ) - 0.375, 1e-12 );

	// The same step would run past the end of the range, so the time left must clamp it
	ENSURE_SMALL( v3SeparationAdvance( &polyA, &polyB, &frameAtThreeQuarters, 0.0, 0.0, 0.25, 0.0625 ) - 0.25, 1e-12 );

	// A gap that does not clear sigma buys no time
	ENSURE( v3SeparationAdvance( &polyA, &polyB, &frameAtStart, 0.0, 0.0, 0.0625, 0.0625 ) == 0.0 );
	ENSURE( v3SeparationAdvance( &polyA, &polyB, &frameAtStart, 0.0, 0.0, 0.03125, 0.0625 ) == 0.0 );
	return 0;
}

static uint32_t g_seed = 987654321u;

static float NextFloat( float lower, float upper )
{
	g_seed = 1664525u * g_seed + 1013904223u;
	float t = (float)( g_seed >> 8 ) / (float)( 1 << 24 );
	return lower + t * ( upper - lower );
}

static b3Vec3 NextDirection( void )
{
	b3Vec3 v = { NextFloat( -1.0f, 1.0f ), NextFloat( -1.0f, 1.0f ), NextFloat( -1.0f, 1.0f ) };
	return b3Normalize( v );
}

// A wide net over randomly oriented box pairs, from clearly separated to deep overlap. The brute
// force oracle is the source of truth. Whatever axis the query returns it must be a unit vector that
// reproduces its own separation, it must not exceed the true maximum, and it must not fall short of
// it. The short fall bound is what a suboptimal edge pick trips. Thin boxes are mixed in because a
// crossed pair of long edges stays the axis of minimum penetration even deep in overlap, which is
// the hardest case for the edge phase.
static int SeparatingAxisOracleTest( void )
{
	g_seed = 987654321u;

	int separated = 0;
	int penetrating = 0;
	int edgeWins = 0;
	float worstBandShortfall = 0.0f;
	float worstExcess = 0.0f;

	for ( int i = 0; i < 4000; ++i )
	{
		b3Vec3 halfA, halfB;
		float reach;

		if ( i & 1 )
		{
			// Thin crossed beams driven into deep overlap.
			halfA = (b3Vec3){ NextFloat( 1.0f, 1.6f ), NextFloat( 0.08f, 0.15f ), NextFloat( 0.08f, 0.15f ) };
			halfB = (b3Vec3){ NextFloat( 1.0f, 1.6f ), NextFloat( 0.08f, 0.15f ), NextFloat( 0.08f, 0.15f ) };
			reach = NextFloat( 0.0f, 0.4f );
		}
		else
		{
			// Chunky boxes across the whole separated to overlapping range.
			halfA = (b3Vec3){ NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ) };
			halfB = (b3Vec3){ NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ) };
			reach = NextFloat( 0.0f, 1.7f );
		}

		// Rotate A in place so it stays centered on the origin, which keeps the support bias valid.
		b3BoxHull hullA =
			b3MakeTransformedBoxHull( halfA.x, halfA.y, halfA.z, ExactRotation( NextDirection(), NextFloat( 0.0f, B3_PI ) ) );
		b3BoxHull hullB = b3MakeBoxHull( halfB.x, halfB.y, halfB.z );

		b3Transform xfB = { b3MulSV( reach, NextDirection() ), ExactQuat( NextDirection(), NextFloat( 0.0f, B3_PI ) ) };

		b3AxisQuery aq = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB, true );
		b3SeparatingAxis q = b3GetBestAxis( &aq );

		b3Vec3 oracleNormal;
		float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );

		ENSURE_SMALL( b3Length( q.normal ) - 1.0f, 1e-3f );
		ENSURE_SMALL( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation, 2e-3f );

		worstExcess = b3MaxFloat( worstExcess, q.separation - oracleSep );

		// Never overstate the true maximum. This holds on every path since the reported axis is real.
		ENSURE( q.separation <= oracleSep + 2e-3f );

		// The scan stops at the first axis that clears the speculative band, so a widely separated pair
		// reports some separating axis rather than the deepest one. That is all the collide path needs,
		// since past the band it drops the pair either way. Inside the band nothing stops the scan, so
		// the result there is the exhaustive maximum, and that is what a manifold gets built from.
		if ( q.separation <= B3_SPECULATIVE_DISTANCE )
		{
			worstBandShortfall = b3MaxFloat( worstBandShortfall, oracleSep - q.separation );

			// A suboptimal axis pick lands a discrete step below the true maximum. The bound sits well
			// under that step yet clear of the edge normalization noise floor near 6e-4.
			ENSURE( q.separation >= oracleSep - 4e-3f );
		}

		if ( oracleSep > 0.0f )
		{
			separated += 1;
		}
		else
		{
			penetrating += 1;
		}

		if ( q.type == b3_edgePairAxis )
		{
			edgeWins += 1;
		}
	}

	printf( "    oracle: separated=%d penetrating=%d edgeWins=%d worstBandShortfall=%.2e worstExcess=%.2e\n", separated,
			penetrating, edgeWins, worstBandShortfall, worstExcess );

	// The sweep has to cover both regimes and actually drive the edge path.
	ENSURE( separated > 100 );
	ENSURE( penetrating > 100 );
	ENSURE( edgeWins > 20 );
	return 0;
}

// The oracle over hulls whose vertices sit far from their own local origin, which the AABB based
// bias must handle. Half the pairs offset A in its own frame, half offset B, and B is placed so the
// two centers land near each other across the separated to overlapping range. Any bias that assumed
// the origin was inside the hull would pick the wrong support and blow the separation.
static int OffsetHullOracleTest( void )
{
	g_seed = 24681012u;

	int separated = 0;
	int penetrating = 0;
	int edgeWins = 0;
	float worstBandShortfall = 0.0f;
	float worstExcess = 0.0f;
	float worstConsistency = 0.0f;

	for ( int i = 0; i < 3000; ++i )
	{
		b3Vec3 halfA = { NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ) };
		b3Vec3 halfB = { NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ), NextFloat( 0.3f, 0.8f ) };

		b3Vec3 offset = b3MulSV( NextFloat( 2.0f, 6.0f ), NextDirection() );
		float reach = NextFloat( 0.0f, 1.7f );
		b3Vec3 dir = NextDirection();

		b3BoxHull hullA;
		b3BoxHull hullB;
		b3Transform xfB;

		if ( i & 1 )
		{
			// A carries the offset in its own frame. B is centered and dropped near A's center.
			b3Transform placeA = { offset, ExactQuat( NextDirection(), NextFloat( 0.0f, B3_PI ) ) };
			hullA = b3MakeTransformedBoxHull( halfA.x, halfA.y, halfA.z, placeA );
			hullB = b3MakeBoxHull( halfB.x, halfB.y, halfB.z );
			xfB = (b3Transform){ b3MulAdd( offset, reach, dir ), ExactQuat( NextDirection(), NextFloat( 0.0f, B3_PI ) ) };
		}
		else
		{
			// B carries the offset in its own frame. Place it so its world center lands near the origin.
			b3Quat qB = ExactQuat( NextDirection(), NextFloat( 0.0f, B3_PI ) );
			hullA = b3MakeBoxHull( halfA.x, halfA.y, halfA.z );
			hullB = b3MakeOffsetBoxHull( halfB.x, halfB.y, halfB.z, offset );
			xfB = (b3Transform){ b3Sub( b3MulSV( reach, dir ), b3RotateVector( qB, offset ) ), qB };
		}

		b3AxisQuery aq = b3ComputeSeparatingAxis( &hullA.base, &hullB.base, xfB, true );
		b3SeparatingAxis q = b3GetBestAxis( &aq );

		b3Vec3 oracleNormal;
		float oracleSep = OracleSeparation( &hullA.base, &hullB.base, xfB, &oracleNormal );

		float consistency = b3AbsFloat( SepAlong( &hullA.base, &hullB.base, xfB, q.normal ) - q.separation );
		worstConsistency = b3MaxFloat( worstConsistency, consistency );
		worstExcess = b3MaxFloat( worstExcess, q.separation - oracleSep );

		// Consistency is the sharp bias check: a wrong support pick on an offset hull would throw the
		// returned normal off its own separation by a vertex spacing, not a noise floor. It holds on
		// every path, early out or not, so it stays unconditional. The oracle bounds are looser since
		// differencing coordinates out at radius six costs a few more digits.
		ENSURE_SMALL( b3Length( q.normal ) - 1.0f, 1e-3f );
		ENSURE( consistency < 1e-3f );
		ENSURE( q.separation <= oracleSep + 3e-3f );

		// Only the in band result feeds a manifold, and only there does the scan run to completion.
		if ( q.separation <= B3_SPECULATIVE_DISTANCE )
		{
			worstBandShortfall = b3MaxFloat( worstBandShortfall, oracleSep - q.separation );
			ENSURE( q.separation >= oracleSep - 8e-3f );
		}

		if ( oracleSep > 0.0f )
		{
			separated += 1;
		}
		else
		{
			penetrating += 1;
		}

		if ( q.type == b3_edgePairAxis )
		{
			edgeWins += 1;
		}
	}

	printf(
		"    offset: separated=%d penetrating=%d edgeWins=%d worstBandShortfall=%.2e worstExcess=%.2e worstConsistency=%.2e\n",
		separated, penetrating, edgeWins, worstBandShortfall, worstExcess, worstConsistency );

	ENSURE( separated > 100 );
	ENSURE( penetrating > 100 );
	ENSURE( edgeWins > 20 );
	return 0;
}

int SeparatingAxisTest( void )
{
	RUN_SUBTEST( FaceAxisASeparatedTest );
	RUN_SUBTEST( FaceAxisBSeparatedTest );
	RUN_SUBTEST( FaceFarSeparatedTest );
	RUN_SUBTEST( OffsetFaceAxisBTest );
	RUN_SUBTEST( EdgePairSweepTest );
	RUN_SUBTEST( CachedDegenerateEdgeFamilyTest );
	RUN_SUBTEST( CachedFaceFamilyTest );
	RUN_SUBTEST( CachedEdgeOrientationChangeTest );
	RUN_SUBTEST( WitnessPointPoseTest );
	RUN_SUBTEST( WitnessPointSweepTest );
	RUN_SUBTEST( SeparationAdvanceEnvelopeTest );
	RUN_SUBTEST( SeparatingAxisOracleTest );
	RUN_SUBTEST( OffsetHullOracleTest );

	return 0;
}

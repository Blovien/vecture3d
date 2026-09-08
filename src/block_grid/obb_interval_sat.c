// SPDX-License-Identifier: MIT

#include "obb_interval_sat.h"

#include "box3d/base.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <stddef.h>

#define V3_EDGE_DEGENERATE_RATIO 0x1p-20

static v3Interval v3SatAbsProjection( v3Interval value )
{
	// Projection needs mig for the lower bound and mag for the upper bound
	return (v3Interval){ v3IvMig( value ), v3IvMag( value ) };
}

static v3Interval v3SatAbsRadius( v3Interval value )
{
	// A subtracted radius needs mag for its certificate-side upper bound while mig preserves full containment
	return (v3Interval){ v3IvMig( value ), v3IvMag( value ) };
}

static v3Interval v3SatMatrixEntry( const v3IntervalMat3* matrix, int row, int column )
{
	return matrix->e[3 * row + column];
}

static v3Interval v3SatDotColumn( const v3IntervalVec3* vector, const v3IntervalMat3* matrix, int column )
{
	v3Interval sum = v3IvMul( vector->e[0], v3SatMatrixEntry( matrix, 0, column ) );
	sum = v3IvAdd( sum, v3IvMul( vector->e[1], v3SatMatrixEntry( matrix, 1, column ) ) );
	return v3IvAdd( sum, v3IvMul( vector->e[2], v3SatMatrixEntry( matrix, 2, column ) ) );
}

static double v3SatIntervalChord( const v3SweepPoly* poly, double t0, double t1 )
{
	double q1[4] = { poly->srcQ1[0], poly->srcQ1[1], poly->srcQ1[2], poly->srcQ1[3] };
	double q2[4] = { poly->srcQ2[0], poly->srcQ2[1], poly->srcQ2[2], poly->srcQ2[3] };
	double dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
	if ( dot < 0.0 )
	{
		for ( int i = 0; i < 4; ++i )
		{
			q1[i] = -q1[i];
		}
	}

	v3Interval speedSquared = v3IvPoint( 0.0 );
	for ( int i = 0; i < 4; ++i )
	{
		double difference = q2[i] - q1[i];
		speedSquared = v3IvAdd( speedSquared, v3IvSqr( v3IvPoint( difference ) ) );
	}
	double speed = v3IvSqrtUp( speedSquared );
	double minimumLength = v3IvSqrtLo( v3SweepPolyDenominatorRange( poly, t0, t1 ) );
	return v3IvUp( v3IvUp( ( t1 - t0 ) * speed ) / minimumLength );
}

bool v3PairIntervalFrameBuild( const v3SweepPoly* polyA, const v3SweepPoly* polyB, double t0, double t1,
							   v3PairIntervalFrame* frame )
{
	if ( polyA == NULL || polyB == NULL || frame == NULL || isfinite( t0 ) == false || isfinite( t1 ) == false || t0 < 0.0 ||
		 t0 > t1 || t1 > 1.0 )
	{
		return false;
	}

	v3IntervalMat3 matrixA, matrixB;
	v3IntervalVec3 centerA, centerB;
	v3SweepPolyMatrixRange( polyA, t0, t1, &matrixA );
	v3SweepPolyMatrixRange( polyB, t0, t1, &matrixB );
	v3SweepPolyCenterRange( polyA, t0, t1, &centerA );
	v3SweepPolyCenterRange( polyB, t0, t1, &centerB );
	v3Interval denominatorA = v3SweepPolyDenominatorRange( polyA, t0, t1 );
	v3Interval denominatorB = v3SweepPolyDenominatorRange( polyB, t0, t1 );

	v3PairIntervalFrame result = { 0 };
	result.k = v3IvMul( denominatorA, denominatorB );
	for ( int row = 0; row < 3; ++row )
	{
		for ( int column = 0; column < 3; ++column )
		{
			v3Interval product = v3IvMul( matrixA.e[row], matrixB.e[column] );
			product = v3IvAdd( product, v3IvMul( matrixA.e[3 + row], matrixB.e[3 + column] ) );
			result.cTilde.e[3 * row + column] = v3IvAdd( product, v3IvMul( matrixA.e[6 + row], matrixB.e[6 + column] ) );
		}

		v3Interval delta[3] = { v3IvSub( centerB.e[0], centerA.e[0] ), v3IvSub( centerB.e[1], centerA.e[1] ),
								v3IvSub( centerB.e[2], centerA.e[2] ) };
		v3Interval projected = v3IvMul( matrixA.e[row], delta[0] );
		projected = v3IvAdd( projected, v3IvMul( matrixA.e[3 + row], delta[1] ) );
		projected = v3IvAdd( projected, v3IvMul( matrixA.e[6 + row], delta[2] ) );
		result.wTilde.e[row] = v3IvMul( denominatorB, projected );
	}

	result.t0 = t0;
	result.t1 = t1;
	result.chordA = v3SatIntervalChord( polyA, t0, t1 );
	result.chordB = v3SatIntervalChord( polyB, t0, t1 );
	*frame = result;
	return true;
}

static v3IntervalVec3 v3SatCenterDifference( const v3PairIntervalFrame* frame, const double centerOffsetA[3],
											 const double centerOffsetB[3] )
{
	v3IntervalVec3 difference;
	for ( int row = 0; row < 3; ++row )
	{
		v3Interval offsetB = v3IvScale( v3SatMatrixEntry( &frame->cTilde, row, 0 ), centerOffsetB[0] );
		offsetB = v3IvAdd( offsetB, v3IvScale( v3SatMatrixEntry( &frame->cTilde, row, 1 ), centerOffsetB[1] ) );
		offsetB = v3IvAdd( offsetB, v3IvScale( v3SatMatrixEntry( &frame->cTilde, row, 2 ), centerOffsetB[2] ) );
		difference.e[row] = v3IvAdd( frame->wTilde.e[row], offsetB );
		// The cleared-form translation subtracts A's own offset because the frame
		// carries B minus A, so A's leaf center enters with a negative sign
		difference.e[row] = v3IvSub( difference.e[row], v3IvScale( frame->k, centerOffsetA[row] ) );
	}
	return difference;
}

static v3Interval v3SatFaceAGap( const v3PairIntervalFrame* frame, const v3IntervalVec3* difference, const double halfExtentA[3],
								 const double halfExtentB[3], int i )
{
	v3Interval gap = v3IvSub( v3SatAbsProjection( difference->e[i] ), v3IvScale( frame->k, halfExtentA[i] ) );
	for ( int j = 0; j < 3; ++j )
	{
		v3Interval radius = v3IvScale( v3SatAbsRadius( v3SatMatrixEntry( &frame->cTilde, i, j ) ), halfExtentB[j] );
		gap = v3IvSub( gap, radius );
	}
	return gap;
}

static v3Interval v3SatFaceBGap( const v3PairIntervalFrame* frame, const v3IntervalVec3* difference, const double halfExtentA[3],
								 const double halfExtentB[3], int j )
{
	v3Interval kSquared = v3IvMul( frame->k, frame->k );
	v3Interval gap =
		v3IvSub( v3SatAbsProjection( v3SatDotColumn( difference, &frame->cTilde, j ) ), v3IvScale( kSquared, halfExtentB[j] ) );
	v3Interval radius = v3IvPoint( 0.0 );
	for ( int i = 0; i < 3; ++i )
	{
		radius = v3IvAdd( radius, v3IvScale( v3SatAbsRadius( v3SatMatrixEntry( &frame->cTilde, i, j ) ), halfExtentA[i] ) );
	}
	return v3IvSub( gap, v3IvMul( frame->k, radius ) );
}

static v3Interval v3SatEdgeGap( const v3PairIntervalFrame* frame, const v3IntervalVec3* difference, const double halfExtentA[3],
								const double halfExtentB[3], int i, int j )
{
	int ip = ( i + 1 ) % 3;
	int im = ( i + 2 ) % 3;
	int jp = ( j + 1 ) % 3;
	int jm = ( j + 2 ) % 3;
	v3Interval projection = v3IvSub( v3IvMul( difference->e[im], v3SatMatrixEntry( &frame->cTilde, ip, j ) ),
									 v3IvMul( difference->e[ip], v3SatMatrixEntry( &frame->cTilde, im, j ) ) );
	v3Interval radius = v3IvScale( v3SatAbsRadius( v3SatMatrixEntry( &frame->cTilde, im, j ) ), halfExtentA[ip] );
	radius = v3IvAdd( radius, v3IvScale( v3SatAbsRadius( v3SatMatrixEntry( &frame->cTilde, ip, j ) ), halfExtentA[im] ) );
	radius = v3IvAdd( radius, v3IvScale( v3SatAbsRadius( v3SatMatrixEntry( &frame->cTilde, i, jm ) ), halfExtentB[jp] ) );
	radius = v3IvAdd( radius, v3IvScale( v3SatAbsRadius( v3SatMatrixEntry( &frame->cTilde, i, jp ) ), halfExtentB[jm] ) );
	return v3IvSub( v3SatAbsProjection( projection ), v3IvMul( frame->k, radius ) );
}

v3Interval v3ObbSatFamilyGap( const v3PairIntervalFrame* frame, const double halfExtentA[3], const double centerOffsetA[3],
							  const double halfExtentB[3], const double centerOffsetB[3], int family )
{
	B3_ASSERT( frame != NULL && halfExtentA != NULL && centerOffsetA != NULL && halfExtentB != NULL && centerOffsetB != NULL );
	B3_ASSERT( 0 <= family && family < 15 );
	v3IntervalVec3 difference = v3SatCenterDifference( frame, centerOffsetA, centerOffsetB );
	if ( family < 3 )
	{
		return v3SatFaceAGap( frame, &difference, halfExtentA, halfExtentB, family );
	}
	if ( family < 6 )
	{
		return v3SatFaceBGap( frame, &difference, halfExtentA, halfExtentB, family - 3 );
	}
	return v3SatEdgeGap( frame, &difference, halfExtentA, halfExtentB, ( family - 6 ) / 3, ( family - 6 ) % 3 );
}

v3Interval v3ObbSatFamilyScale( const v3PairIntervalFrame* frame, int family )
{
	B3_ASSERT( frame != NULL && 0 <= family && family < 15 );
	if ( family < 3 )
	{
		return frame->k;
	}
	if ( family < 6 )
	{
		return v3IvMul( frame->k, frame->k );
	}

	int i = ( family - 6 ) / 3;
	int j = ( family - 6 ) % 3;
	int ip = ( i + 1 ) % 3;
	int im = ( i + 2 ) % 3;
	v3Interval lengthSquared =
		v3IvAdd( v3IvSqr( v3SatMatrixEntry( &frame->cTilde, ip, j ) ), v3IvSqr( v3SatMatrixEntry( &frame->cTilde, im, j ) ) );
	v3Interval length = { v3IvSqrtLo( lengthSquared ), v3IvSqrtUp( lengthSquared ) };
	return v3IvMul( frame->k, length );
}

double v3ObbSatFamilyMargin( const v3PairIntervalFrame* frame, int family, double margin )
{
	v3Interval scale = v3ObbSatFamilyScale( frame, family );
	// A negative floor is hardest to satisfy where the cleared SAT scale is smallest
	double selectedScale = margin < 0.0 ? scale.lo : scale.hi;
	return v3IvUp( margin * selectedScale );
}

static bool v3SatDegenerateEdge( const v3PairIntervalFrame* frame, int family )
{
	int i = ( family - 6 ) / 3;
	int j = ( family - 6 ) % 3;
	int ip = ( i + 1 ) % 3;
	int im = ( i + 2 ) % 3;
	v3Interval lengthSquared =
		v3IvAdd( v3IvSqr( v3SatMatrixEntry( &frame->cTilde, ip, j ) ), v3IvSqr( v3SatMatrixEntry( &frame->cTilde, im, j ) ) );
	double kMagnitude = v3IvMag( frame->k );
	return lengthSquared.hi < V3_EDGE_DEGENERATE_RATIO * kMagnitude * kMagnitude;
}

bool v3ObbSatFamilyCertifiesSeparation( const v3PairIntervalFrame* frame, const double halfExtentA[3],
										const double centerOffsetA[3], const double halfExtentB[3], const double centerOffsetB[3],
										double sigma, int family )
{
	B3_ASSERT( isfinite( sigma ) && 0 <= family && family < 15 );
	if ( family >= 6 && v3SatDegenerateEdge( frame, family ) )
	{
		return false;
	}
	v3Interval gap = v3ObbSatFamilyGap( frame, halfExtentA, centerOffsetA, halfExtentB, centerOffsetB, family );
	double margin = v3ObbSatFamilyMargin( frame, family, sigma );
	return gap.lo > margin;
}

int v3ObbSatCertifySeparation( const v3PairIntervalFrame* frame, const double halfExtentA[3], const double centerOffsetA[3],
							   const double halfExtentB[3], const double centerOffsetB[3], double sigma )
{
	B3_ASSERT( isfinite( sigma ) );
	// A static family order keeps this scan deterministic. Callers that retain a
	// witness across subintervals try their cached family before falling back here
	for ( int family = 0; family < 15; ++family )
	{
		if ( v3ObbSatFamilyCertifiesSeparation( frame, halfExtentA, centerOffsetA, halfExtentB, centerOffsetB, sigma, family ) )
		{
			return family;
		}
	}
	return -1;
}

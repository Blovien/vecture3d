// SPDX-License-Identifier: MIT

#include "v3_witness_point.h"

#include "box3d/base.h"

#include <math.h>
#include <stddef.h>

static double v3WitnessClamp( double value, double lower, double upper )
{
	return value < lower ? lower : value > upper ? upper : value;
}

static void v3WitnessRotate( const double rotation[9], const double vector[3], double result[3] )
{
	for ( int row = 0; row < 3; ++row )
	{
		result[row] = rotation[3 * row] * vector[0] + rotation[3 * row + 1] * vector[1] + rotation[3 * row + 2] * vector[2];
	}
}

static void v3WitnessBoxCenter( const double rotation[9], const double origin[3], const double centerOffset[3], double center[3] )
{
	double rotatedOffset[3];
	v3WitnessRotate( rotation, centerOffset, rotatedOffset );
	for ( int axis = 0; axis < 3; ++axis )
	{
		center[axis] = origin[axis] + rotatedOffset[axis];
	}
}

static void v3WitnessClampToBox( const double point[3], const double rotation[9], const double center[3],
								 const double halfExtent[3], double clamped[3] )
{
	double local[3];
	for ( int axis = 0; axis < 3; ++axis )
	{
		local[axis] = rotation[axis] * ( point[0] - center[0] ) + rotation[3 + axis] * ( point[1] - center[1] ) +
					  rotation[6 + axis] * ( point[2] - center[2] );
		local[axis] = v3WitnessClamp( local[axis], -halfExtent[axis], halfExtent[axis] );
	}

	double rotated[3];
	v3WitnessRotate( rotation, local, rotated );
	for ( int axis = 0; axis < 3; ++axis )
	{
		clamped[axis] = center[axis] + rotated[axis];
	}
}

static void v3WitnessCandidate( const double rotationA[9], const double centerA[3], const double halfExtentA[3],
								const double rotationB[9], const double centerB[3], const double halfExtentB[3], double pointA[3],
								double pointB[3], double witness[3] )
{
	v3WitnessClampToBox( centerA, rotationB, centerB, halfExtentB, pointB );
	v3WitnessClampToBox( pointB, rotationA, centerA, halfExtentA, pointA );
	v3WitnessClampToBox( pointA, rotationB, centerB, halfExtentB, pointB );
	for ( int axis = 0; axis < 3; ++axis )
	{
		witness[axis] = 0.5 * ( pointA[axis] + pointB[axis] );
	}
}

static bool v3WitnessVerifyBox( const v3IntervalMat3* rotation, const v3Interval center[3], const double halfExtent[3],
								const double witness[3], double inflation )
{
	for ( int axis = 0; axis < 3; ++axis )
	{
		v3Interval local = v3IvMul( rotation->e[axis], v3IvSub( v3IvPoint( witness[0] ), center[0] ) );
		local = v3IvAdd( local, v3IvMul( rotation->e[3 + axis], v3IvSub( v3IvPoint( witness[1] ), center[1] ) ) );
		local = v3IvAdd( local, v3IvMul( rotation->e[6 + axis], v3IvSub( v3IvPoint( witness[2] ), center[2] ) ) );
		double limit = halfExtent[axis] + inflation;

		// Both signed bounds are checked so each box contributes six certain comparisons
		if ( local.hi > limit || -local.lo > limit )
		{
			return false;
		}
	}
	return true;
}

static void v3WitnessIntervalCenter( const v3IntervalMat3* rotation, const v3Interval origin[3], const double centerOffset[3],
									 v3Interval center[3] )
{
	for ( int row = 0; row < 3; ++row )
	{
		v3Interval rotated = v3IvScale( rotation->e[3 * row], centerOffset[0] );
		rotated = v3IvAdd( rotated, v3IvScale( rotation->e[3 * row + 1], centerOffset[1] ) );
		rotated = v3IvAdd( rotated, v3IvScale( rotation->e[3 * row + 2], centerOffset[2] ) );
		center[row] = v3IvAdd( origin[row], rotated );
	}
}

static bool v3WitnessCertifyIntervals( const double rotationA[9], const double centerA[3], const double halfExtentA[3],
									   const v3IntervalMat3* intervalRotationA, const v3Interval intervalCenterA[3],
									   const double rotationB[9], const double centerB[3], const double halfExtentB[3],
									   const v3IntervalMat3* intervalRotationB, const v3Interval intervalCenterB[3], double delta,
									   v3WitnessPointResult* result )
{
	double pointA[3], pointB[3];
	v3WitnessCandidate( rotationA, centerA, halfExtentA, rotationB, centerB, halfExtentB, pointA, pointB, result->point );

	// Rounding downward keeps the axis-aligned inflation inside the requested delta ball
	double inflation = nextafter( delta / sqrt( 3.0 ), 0.0 );
	result->certified = v3WitnessVerifyBox( intervalRotationA, intervalCenterA, halfExtentA, result->point, inflation ) &&
						v3WitnessVerifyBox( intervalRotationB, intervalCenterB, halfExtentB, result->point, inflation );
	return result->certified;
}

bool v3WitnessCertifyContactAtPose( const double rotationA[9], const double originA[3], const double halfExtentA[3],
									const double centerOffsetA[3], const double rotationB[9], const double originB[3],
									const double halfExtentB[3], const double centerOffsetB[3], double delta,
									v3WitnessPointResult* result )
{
	B3_ASSERT( rotationA != NULL && originA != NULL && halfExtentA != NULL && centerOffsetA != NULL );
	B3_ASSERT( rotationB != NULL && originB != NULL && halfExtentB != NULL && centerOffsetB != NULL );
	B3_ASSERT( result != NULL && isfinite( delta ) && delta >= 0.0 );

	double centerA[3], centerB[3];
	v3WitnessBoxCenter( rotationA, originA, centerOffsetA, centerA );
	v3WitnessBoxCenter( rotationB, originB, centerOffsetB, centerB );
	v3IntervalMat3 intervalRotationA, intervalRotationB;
	v3Interval intervalOriginA[3], intervalOriginB[3], intervalCenterA[3], intervalCenterB[3];
	for ( int i = 0; i < 9; ++i )
	{
		intervalRotationA.e[i] = v3IvPoint( rotationA[i] );
		intervalRotationB.e[i] = v3IvPoint( rotationB[i] );
	}
	for ( int axis = 0; axis < 3; ++axis )
	{
		intervalOriginA[axis] = v3IvPoint( originA[axis] );
		intervalOriginB[axis] = v3IvPoint( originB[axis] );
	}
	v3WitnessIntervalCenter( &intervalRotationA, intervalOriginA, centerOffsetA, intervalCenterA );
	v3WitnessIntervalCenter( &intervalRotationB, intervalOriginB, centerOffsetB, intervalCenterB );

	return v3WitnessCertifyIntervals( rotationA, centerA, halfExtentA, &intervalRotationA, intervalCenterA, rotationB, centerB,
									  halfExtentB, &intervalRotationB, intervalCenterB, delta, result );
}

static void v3WitnessSweepPose( const v3SweepPoly* poly, const double centerOffset[3], double t, double rotation[9],
								double center[3], v3IntervalMat3* intervalRotation, v3Interval intervalCenter[3] )
{
	v3Interval denominator = v3SweepPolyDenominatorRange( poly, t, t );
	v3IntervalMat3 numerator;
	v3SweepPolyMatrixRange( poly, t, t, &numerator );
	for ( int i = 0; i < 9; ++i )
	{
		intervalRotation->e[i] = v3IvDivPos( numerator.e[i], denominator );
		rotation[i] = 0.5 * ( intervalRotation->e[i].lo + intervalRotation->e[i].hi );
	}

	v3IntervalVec3 sweepCenter;
	v3SweepPolyCenterRange( poly, t, t, &sweepCenter );
	double relativeOffset[3];
	for ( int axis = 0; axis < 3; ++axis )
	{
		relativeOffset[axis] = centerOffset[axis] - poly->localCenter[axis];
	}
	v3WitnessIntervalCenter( intervalRotation, sweepCenter.e, relativeOffset, intervalCenter );
	double midpointCenter[3] = {
		0.5 * ( sweepCenter.e[0].lo + sweepCenter.e[0].hi ),
		0.5 * ( sweepCenter.e[1].lo + sweepCenter.e[1].hi ),
		0.5 * ( sweepCenter.e[2].lo + sweepCenter.e[2].hi ),
	};
	double rotatedOffset[3];
	v3WitnessRotate( rotation, relativeOffset, rotatedOffset );
	for ( int axis = 0; axis < 3; ++axis )
	{
		center[axis] = midpointCenter[axis] + rotatedOffset[axis];
	}
}

bool v3WitnessCertifyContact( const v3SweepPoly* polyA, const v3SweepPoly* polyB, const double halfExtentA[3],
							  const double centerOffsetA[3], const double halfExtentB[3], const double centerOffsetB[3], double t,
							  double delta, v3WitnessPointResult* result )
{
	B3_ASSERT( polyA != NULL && polyB != NULL && halfExtentA != NULL && centerOffsetA != NULL );
	B3_ASSERT( halfExtentB != NULL && centerOffsetB != NULL && result != NULL );
	B3_ASSERT( 0.0 <= t && t <= 1.0 && isfinite( delta ) && delta >= 0.0 );

	double rotationA[9], rotationB[9], centerA[3], centerB[3];
	v3IntervalMat3 intervalRotationA, intervalRotationB;
	v3Interval intervalCenterA[3], intervalCenterB[3];
	v3WitnessSweepPose( polyA, centerOffsetA, t, rotationA, centerA, &intervalRotationA, intervalCenterA );
	v3WitnessSweepPose( polyB, centerOffsetB, t, rotationB, centerB, &intervalRotationB, intervalCenterB );
	return v3WitnessCertifyIntervals( rotationA, centerA, halfExtentA, &intervalRotationA, intervalCenterA, rotationB, centerB,
									  halfExtentB, &intervalRotationB, intervalCenterB, delta, result );
}

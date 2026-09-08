// SPDX-License-Identifier: MIT

#include "v3_sweep_interval.h"

#include "box3d/base.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <string.h>

// The next double above pi keeps the angle multiplication one-sided
#define V3_PI_UP 0x1.921fb54442d19p+1

static bool v3SweepFiniteQuat( b3Quat q )
{
	return isfinite( q.v.x ) && isfinite( q.v.y ) && isfinite( q.v.z ) && isfinite( q.s );
}

static bool v3SweepFiniteVec3( b3Vec3 v )
{
	return isfinite( v.x ) && isfinite( v.y ) && isfinite( v.z );
}

static double v3SweepDot4( const double a[4], const double b[4] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

static double v3SweepNorm4Up( const double a[4] )
{
	v3Interval sum = v3IvSqr( (v3Interval){ v3IvDown( a[0] ), v3IvUp( a[0] ) } );
	for ( int i = 1; i < 4; ++i )
	{
		sum = v3IvAdd( sum, v3IvSqr( (v3Interval){ v3IvDown( a[i] ), v3IvUp( a[i] ) } ) );
	}
	return v3IvSqrtUp( sum );
}

static double v3SweepNorm3Up( const double a[3] )
{
	v3Interval sum = v3IvSqr( (v3Interval){ v3IvDown( a[0] ), v3IvUp( a[0] ) } );
	for ( int i = 1; i < 3; ++i )
	{
		sum = v3IvAdd( sum, v3IvSqr( (v3Interval){ v3IvDown( a[i] ), v3IvUp( a[i] ) } ) );
	}
	return v3IvSqrtUp( sum );
}

static bool v3SweepNormalize4( const double q[4], double qHat[4] )
{
	double lengthSquared = v3SweepDot4( q, q );
	if ( isfinite( lengthSquared ) == false || lengthSquared <= 0.0 )
	{
		return false;
	}

	double inverseLength = 1.0 / sqrt( lengthSquared );
	if ( isfinite( inverseLength ) == false )
	{
		return false;
	}
	for ( int i = 0; i < 4; ++i )
	{
		qHat[i] = q[i] * inverseLength;
	}
	return true;
}

static void v3SweepMatrix( const double q[4], double matrix[9] )
{
	double x = q[0];
	double y = q[1];
	double z = q[2];
	double s = q[3];
	matrix[0] = s * s + x * x - y * y - z * z;
	matrix[4] = s * s - x * x + y * y - z * z;
	matrix[8] = s * s - x * x - y * y + z * z;
	matrix[1] = 2.0 * ( x * y - s * z );
	matrix[3] = 2.0 * ( x * y + s * z );
	matrix[2] = 2.0 * ( x * z + s * y );
	matrix[6] = 2.0 * ( x * z - s * y );
	matrix[5] = 2.0 * ( y * z - s * x );
	matrix[7] = 2.0 * ( y * z + s * x );
}

static void v3SweepBilinearMatrix( const double p[4], const double r[4], double matrix[9] )
{
	double xp = p[0], yp = p[1], zp = p[2], sp = p[3];
	double xr = r[0], yr = r[1], zr = r[2], sr = r[3];
	matrix[0] = 2.0 * ( sp * sr + xp * xr - yp * yr - zp * zr );
	matrix[4] = 2.0 * ( sp * sr - xp * xr + yp * yr - zp * zr );
	matrix[8] = 2.0 * ( sp * sr - xp * xr - yp * yr + zp * zr );
	matrix[1] = 2.0 * ( xp * yr + xr * yp - sp * zr - sr * zp );
	matrix[3] = 2.0 * ( xp * yr + xr * yp + sp * zr + sr * zp );
	matrix[2] = 2.0 * ( xp * zr + xr * zp + sp * yr + sr * yp );
	matrix[6] = 2.0 * ( xp * zr + xr * zp - sp * yr - sr * yp );
	matrix[5] = 2.0 * ( yp * zr + yr * zp - sp * xr - sr * xp );
	matrix[7] = 2.0 * ( yp * zr + yr * zp + sp * xr + sr * xp );
}

static v3Interval v3SweepCoefficientRange( double c0, double c1, double c2, double t0, double t1 )
{
	v3Interval range = v3IvQuadRange( c0, c1, c2, t0, t1 );
	return (v3Interval){ v3IvDown( range.lo ), v3IvUp( range.hi ) };
}

bool v3SweepPolyBuild( const b3Sweep* sweep, v3SweepPoly* poly )
{
	if ( sweep == NULL || poly == NULL || v3SweepFiniteQuat( sweep->q1 ) == false || v3SweepFiniteQuat( sweep->q2 ) == false ||
		 v3SweepFiniteVec3( sweep->c1 ) == false || v3SweepFiniteVec3( sweep->c2 ) == false ||
		 v3SweepFiniteVec3( sweep->localCenter ) == false )
	{
		return false;
	}

	v3SweepPoly result;
	memset( &result, 0, sizeof( result ) );
	b3Quat q1 = sweep->q1;
	b3Quat q2 = sweep->q2;

	// The branch and negation must stay in float to match b3NLerp at dot knife edges
	float sourceDot = b3DotQuat( q1, q2 );
	if ( isfinite( sourceDot ) == false )
	{
		return false;
	}
	if ( sourceDot < 0.0f )
	{
		q1 = (b3Quat){ { -q1.v.x, -q1.v.y, -q1.v.z }, -q1.s };
	}
	float fixedDot = b3DotQuat( q1, q2 );
	if ( isfinite( fixedDot ) == false || fixedDot < 0.0f )
	{
		return false;
	}

	double p[4] = { q1.v.x, q1.v.y, q1.v.z, q1.s };
	double q[4] = { q2.v.x, q2.v.y, q2.v.z, q2.s };
	double v[4];
	for ( int i = 0; i < 4; ++i )
	{
		v[i] = q[i] - p[i];
	}

	v3SweepMatrix( p, result.m0 );
	v3SweepBilinearMatrix( p, v, result.m1 );
	v3SweepMatrix( v, result.m2 );
	result.n0 = v3SweepDot4( p, p );
	result.n1 = 2.0 * v3SweepDot4( p, v );
	result.n2 = v3SweepDot4( v, v );

	result.c1[0] = sweep->c1.x;
	result.c1[1] = sweep->c1.y;
	result.c1[2] = sweep->c1.z;
	result.dc[0] = (double)sweep->c2.x - sweep->c1.x;
	result.dc[1] = (double)sweep->c2.y - sweep->c1.y;
	result.dc[2] = (double)sweep->c2.z - sweep->c1.z;
	result.localCenter[0] = sweep->localCenter.x;
	result.localCenter[1] = sweep->localCenter.y;
	result.localCenter[2] = sweep->localCenter.z;

	if ( v3SweepNormalize4( p, result.qHat1 ) == false || v3SweepNormalize4( q, result.qHat2 ) == false )
	{
		return false;
	}
	double chord[4];
	for ( int i = 0; i < 4; ++i )
	{
		chord[i] = result.qHat2[i] - result.qHat1[i];
	}
	result.chordFull = v3SweepNorm4Up( chord );
	result.angleFull = v3IvUp( V3_PI_UP * result.chordFull );

	result.srcQ1[0] = sweep->q1.v.x;
	result.srcQ1[1] = sweep->q1.v.y;
	result.srcQ1[2] = sweep->q1.v.z;
	result.srcQ1[3] = sweep->q1.s;
	result.srcQ2[0] = sweep->q2.v.x;
	result.srcQ2[1] = sweep->q2.v.y;
	result.srcQ2[2] = sweep->q2.v.z;
	result.srcQ2[3] = sweep->q2.s;
	result.srcC1[0] = sweep->c1.x;
	result.srcC1[1] = sweep->c1.y;
	result.srcC1[2] = sweep->c1.z;
	result.srcC2[0] = sweep->c2.x;
	result.srcC2[1] = sweep->c2.y;
	result.srcC2[2] = sweep->c2.z;
	result.srcLocalCenter[0] = sweep->localCenter.x;
	result.srcLocalCenter[1] = sweep->localCenter.y;
	result.srcLocalCenter[2] = sweep->localCenter.z;

	v3Interval denominator = v3SweepCoefficientRange( result.n0, result.n1, result.n2, 0.0, 1.0 );
	if ( denominator.lo <= 0.4 )
	{
		return false;
	}

	*poly = result;
	return true;
}

v3Interval v3SweepPolyDenominatorRange( const v3SweepPoly* poly, double t0, double t1 )
{
	B3_ASSERT( poly != NULL );
	B3_ASSERT( 0.0 <= t0 && t0 <= t1 && t1 <= 1.0 );
	v3Interval range = v3SweepCoefficientRange( poly->n0, poly->n1, poly->n2, t0, t1 );
	B3_ASSERT( range.lo > 0.4 );
	return range;
}

void v3SweepPolyMatrixRange( const v3SweepPoly* poly, double t0, double t1, v3IntervalMat3* unnormalized )
{
	B3_ASSERT( poly != NULL && unnormalized != NULL );
	B3_ASSERT( 0.0 <= t0 && t0 <= t1 && t1 <= 1.0 );
	for ( int i = 0; i < 9; ++i )
	{
		unnormalized->e[i] = v3SweepCoefficientRange( poly->m0[i], poly->m1[i], poly->m2[i], t0, t1 );
	}
}

void v3SweepPolyCenterRange( const v3SweepPoly* poly, double t0, double t1, v3IntervalVec3* center )
{
	B3_ASSERT( poly != NULL && center != NULL );
	B3_ASSERT( 0.0 <= t0 && t0 <= t1 && t1 <= 1.0 );
	for ( int i = 0; i < 3; ++i )
	{
		v3Interval range = v3IvLinRange( poly->c1[i], poly->dc[i], t0, t1 );
		center->e[i] = (v3Interval){ v3IvDown( range.lo ), v3IvUp( range.hi ) };
	}
}

double v3SweepPolyCornerEnvelope( const v3SweepPoly* poly, double rho, double t0, double t1 )
{
	B3_ASSERT( poly != NULL );
	B3_ASSERT( isfinite( rho ) && rho >= 0.0 );
	B3_ASSERT( 0.0 <= t0 && t0 <= t1 && t1 <= 1.0 );

	double dt = v3IvUp( t1 - t0 );
	double translation = v3IvUp( dt * v3SweepNorm3Up( poly->dc ) );
	b3Quat q1 = { { poly->srcQ1[0], poly->srcQ1[1], poly->srcQ1[2] }, poly->srcQ1[3] };
	b3Quat q2 = { { poly->srcQ2[0], poly->srcQ2[1], poly->srcQ2[2] }, poly->srcQ2[3] };
	if ( b3DotQuat( q1, q2 ) < 0.0f )
	{
		q1 = (b3Quat){ { -q1.v.x, -q1.v.y, -q1.v.z }, -q1.s };
	}
	double v[4] = {
		(double)q2.v.x - q1.v.x,
		(double)q2.v.y - q1.v.y,
		(double)q2.v.z - q1.v.z,
		(double)q2.s - q1.s,
	};

	// The normalization map is 1/sqrt(N)-Lipschitz along the linear quaternion path
	v3Interval denominator = v3SweepPolyDenominatorRange( poly, t0, t1 );
	double minimumLength = v3IvSqrtLo( denominator );
	double chord = v3IvUp( v3IvUp( dt * v3SweepNorm4Up( v ) ) / minimumLength );
	double angle = v3IvUp( V3_PI_UP * chord );
	double rotation = v3IvUp( rho * angle );
	return v3IvUp( translation + rotation );
}

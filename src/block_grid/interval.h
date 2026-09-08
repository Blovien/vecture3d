// SPDX-License-Identifier: MIT

#pragma once

#include <float.h>
#include <math.h>
#include <stdbool.h>

// Inf-sup interval arithmetic for the rotational CCD certifier
//
// Every operation returns an interval that contains the true real result, which
// is what lets a positive lower bound act as a separation certificate. Outward
// rounding uses Rump-style multiplicative inflation instead of fesetround, since
// GCC ignores FENV_ACCESS and a mode switch would leak global state across
// worker tasks. The certifier stays polynomial plus one square root because libm
// transcendentals are not correctly rounded and differ between platforms, and
// recording replay depends on bit-identical results
//
// The one-operation error model assumes round-to-nearest double with no fused
// contraction, so these units build with -ffp-contract=off and never on x87
#if defined( FLT_EVAL_METHOD ) && FLT_EVAL_METHOD != 0
#error "interval.h requires FLT_EVAL_METHOD == 0; build with SSE2 or better, never x87"
#endif

#define V3_IV_U 0x1p-53
#define V3_IV_ETA 0x1p-1074

typedef struct v3Interval
{
	// Invariant lo <= hi with both endpoints finite
	double lo;
	double hi;
} v3Interval;

typedef struct v3IntervalVec3
{
	v3Interval e[3];
} v3IntervalVec3;

typedef struct v3IntervalMat3
{
	// Row major with e[3 * row + col]
	v3Interval e[9];
} v3IntervalMat3;

// One inflation step covers the rounding of one nearest-mode operation, with the
// eta term absorbing underflow into subnormals. Inflation applies to the result
// of every elementary operation rather than once per composite expression
static inline double v3IvDown( double x )
{
	return x - ( fabs( x ) * ( 2.0 * V3_IV_U ) + V3_IV_ETA );
}

static inline double v3IvUp( double x )
{
	return x + ( fabs( x ) * ( 2.0 * V3_IV_U ) + V3_IV_ETA );
}

// Exact for any double, which makes widening float inputs exact as well
static inline v3Interval v3IvPoint( double x )
{
	return (v3Interval){ x, x };
}

static inline v3Interval v3IvAdd( v3Interval x, v3Interval y )
{
	return (v3Interval){ v3IvDown( x.lo + y.lo ), v3IvUp( x.hi + y.hi ) };
}

static inline v3Interval v3IvSub( v3Interval x, v3Interval y )
{
	return (v3Interval){ v3IvDown( x.lo - y.hi ), v3IvUp( x.hi - y.lo ) };
}

// Negation permutes exact endpoints, so no inflation is needed
static inline v3Interval v3IvNeg( v3Interval x )
{
	return (v3Interval){ -x.hi, -x.lo };
}

static inline double v3IvMinDouble( double a, double b )
{
	return a < b ? a : b;
}

static inline double v3IvMaxDouble( double a, double b )
{
	return a > b ? a : b;
}

// Branchless four-product reference form. A sign-case dispatch is an accepted
// alternative when measurement justifies it
static inline v3Interval v3IvMul( v3Interval x, v3Interval y )
{
	double p1 = x.lo * y.lo;
	double p2 = x.lo * y.hi;
	double p3 = x.hi * y.lo;
	double p4 = x.hi * y.hi;
	double lo = v3IvMinDouble( v3IvMinDouble( p1, p2 ), v3IvMinDouble( p3, p4 ) );
	double hi = v3IvMaxDouble( v3IvMaxDouble( p1, p2 ), v3IvMaxDouble( p3, p4 ) );
	return (v3Interval){ v3IvDown( lo ), v3IvUp( hi ) };
}

// For exact constants such as half extents and box offsets
static inline v3Interval v3IvScale( v3Interval x, double c )
{
	double lo = c >= 0.0 ? c * x.lo : c * x.hi;
	double hi = c >= 0.0 ? c * x.hi : c * x.lo;
	return (v3Interval){ v3IvDown( lo ), v3IvUp( hi ) };
}

// Strictly tighter than v3IvMul( x, x ) when x straddles zero, and the SAT
// bounds depend on that tightness
static inline v3Interval v3IvSqr( v3Interval x )
{
	if ( x.lo >= 0.0 )
	{
		return (v3Interval){ v3IvDown( x.lo * x.lo ), v3IvUp( x.hi * x.hi ) };
	}
	if ( x.hi <= 0.0 )
	{
		return (v3Interval){ v3IvDown( x.hi * x.hi ), v3IvUp( x.lo * x.lo ) };
	}
	double m = v3IvMaxDouble( -x.lo, x.hi );
	return (v3Interval){ 0.0, v3IvUp( m * m ) };
}

// Upper bound on the magnitude of every value in the interval
static inline double v3IvMag( v3Interval x )
{
	return v3IvMaxDouble( fabs( x.lo ), fabs( x.hi ) );
}

// Lower bound on the magnitude, which is zero whenever the interval straddles
// zero. Confusing mig with mag is the classic soundness mistake in interval SAT,
// so every call site should say which bound the certificate direction needs
static inline double v3IvMig( v3Interval x )
{
	if ( x.lo > 0.0 )
	{
		return x.lo;
	}
	if ( x.hi < 0.0 )
	{
		return -x.hi;
	}
	return 0.0;
}

static inline v3Interval v3IvAbs( v3Interval x )
{
	return (v3Interval){ v3IvMig( x ), v3IvMag( x ) };
}

static inline v3Interval v3IvHull( v3Interval x, v3Interval y )
{
	return (v3Interval){ v3IvMinDouble( x.lo, y.lo ), v3IvMaxDouble( x.hi, y.hi ) };
}

// The argument clamps at zero so a slightly negative lower bound from earlier
// inflation cannot produce NaN
static inline double v3IvSqrtLo( v3Interval x )
{
	double a = v3IvMaxDouble( x.lo, 0.0 );
	return v3IvMaxDouble( v3IvDown( sqrt( a ) ), 0.0 );
}

static inline double v3IvSqrtUp( v3Interval x )
{
	double a = v3IvMaxDouble( x.hi, 0.0 );
	return v3IvUp( sqrt( a ) );
}

// Only used in pose evaluation, never in the hot loop. The caller guarantees a
// certainly positive divisor
static inline v3Interval v3IvDivPos( v3Interval x, v3Interval y )
{
	double q1 = x.lo / y.lo;
	double q2 = x.lo / y.hi;
	double q3 = x.hi / y.lo;
	double q4 = x.hi / y.hi;
	double lo = v3IvMinDouble( v3IvMinDouble( q1, q2 ), v3IvMinDouble( q3, q4 ) );
	double hi = v3IvMaxDouble( v3IvMaxDouble( q1, q2 ), v3IvMaxDouble( q3, q4 ) );
	return (v3Interval){ v3IvDown( lo ), v3IvUp( hi ) };
}

// Preserve the summation order for deterministic results.
static inline v3Interval v3IvDot3( const v3Interval x[3], const v3Interval y[3] )
{
	v3Interval s = v3IvMul( x[0], y[0] );
	s = v3IvAdd( s, v3IvMul( x[1], y[1] ) );
	s = v3IvAdd( s, v3IvMul( x[2], y[2] ) );
	return s;
}

static inline v3Interval v3IvLinRange( double c0, double c1, double t0, double t1 )
{
	v3Interval f0 = v3IvAdd( v3IvPoint( c0 ), v3IvScale( v3IvPoint( t0 ), c1 ) );
	v3Interval f1 = v3IvAdd( v3IvPoint( c0 ), v3IvScale( v3IvPoint( t1 ), c1 ) );
	return v3IvHull( f0, f1 );
}

// Evaluates the quadratic at one point through interval Horner steps
static inline v3Interval v3IvQuadAt( double c0, double c1, double c2, v3Interval t )
{
	v3Interval inner = v3IvAdd( v3IvPoint( c1 ), v3IvScale( t, c2 ) );
	return v3IvAdd( v3IvPoint( c0 ), v3IvMul( t, inner ) );
}

// Exact range of a degree-two polynomial over the interval, which removes the
// dependency problem at the base of the expression tree where a generic Horner
// extension would be amplified by every later product. The vertex is enclosed
// rather than computed, so its division is the only one in the level-zero path
static inline v3Interval v3IvQuadRange( double c0, double c1, double c2, double t0, double t1 )
{
	v3Interval range = v3IvHull( v3IvQuadAt( c0, c1, c2, v3IvPoint( t0 ) ), v3IvQuadAt( c0, c1, c2, v3IvPoint( t1 ) ) );
	if ( c2 != 0.0 )
	{
		double tv = -c1 / ( 2.0 * c2 );
		v3Interval vertex = { v3IvDown( tv ), v3IvUp( tv ) };
		if ( vertex.hi >= t0 && vertex.lo <= t1 )
		{
			v3Interval clamped = { v3IvMaxDouble( vertex.lo, t0 ), v3IvMinDouble( vertex.hi, t1 ) };
			range = v3IvHull( range, v3IvQuadAt( c0, c1, c2, clamped ) );
		}
	}
	return range;
}

// Certainly positive, which is the predicate a separation certificate needs
static inline bool v3IvIsPos( v3Interval x )
{
	return x.lo > 0.0;
}

static inline bool v3IvIsNeg( v3Interval x )
{
	return x.hi < 0.0;
}

static inline double v3IvWidth( v3Interval x )
{
	return v3IvUp( x.hi - x.lo );
}

static inline bool v3IvIsFinite( v3Interval x )
{
	return isfinite( x.lo ) && isfinite( x.hi ) && x.lo <= x.hi;
}

// Checks whether floating-point arithmetic satisfies the interval inflation assumptions.
static inline bool v3IntervalSelfTest( void )
{
	if ( v3IvUp( 1.0 ) <= 1.0 || v3IvDown( 1.0 ) >= 1.0 )
	{
		return false;
	}
	if ( v3IvUp( 0.0 ) <= 0.0 || v3IvDown( 0.0 ) >= 0.0 )
	{
		return false;
	}

	// A straddling product must contain the true result with outward slack
	v3Interval a = { -3.0, 2.0 };
	v3Interval b = { -1.0, 5.0 };
	v3Interval p = v3IvMul( a, b );
	if ( p.lo > -15.0 || p.hi < 10.0 )
	{
		return false;
	}

	// The square of a straddling interval must be tighter than the product form
	v3Interval s = v3IvSqr( a );
	if ( s.lo != 0.0 || s.hi < 9.0 || s.hi > v3IvMul( a, a ).hi )
	{
		return false;
	}

	// The quadratic range must include the interior vertex of t^2 - t
	v3Interval q = v3IvQuadRange( 0.0, -1.0, 1.0, 0.0, 1.0 );
	if ( q.lo > -0.25 || q.hi < 0.0 )
	{
		return false;
	}

	// One ulp of directed-rounding headroom on a representative magnitude
	double x = 1.5;
	if ( v3IvUp( x ) < nextafter( x, INFINITY ) || v3IvDown( x ) > nextafter( x, -INFINITY ) )
	{
		return false;
	}
	return true;
}

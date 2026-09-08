// SPDX-License-Identifier: MIT

#include "separation_certifier.h"

#include "box3d/base.h"

#include <math.h>
#include <stddef.h>

static double v3SeparationNorm3Up( const double value[3] )
{
	v3Interval squared = v3IvSqr( v3IvPoint( value[0] ) );
	squared = v3IvAdd( squared, v3IvSqr( v3IvPoint( value[1] ) ) );
	squared = v3IvAdd( squared, v3IvSqr( v3IvPoint( value[2] ) ) );
	return v3IvSqrtUp( squared );
}

static bool v3SeparationEnvelopeCertifies( const v3PairIntervalFrame* frame, const double halfExtentA[3],
										   const double centerOffsetA[3], const double halfExtentB[3],
										   const double centerOffsetB[3], double sigma )
{
	if ( sigma < 0.0 )
	{
		return false;
	}

	// The interval leaf-center displacement already contains translation and both rotation chords
	v3Interval squared = v3IvPoint( 0.0 );
	for ( int row = 0; row < 3; ++row )
	{
		v3Interval offsetB = v3IvScale( frame->cTilde.e[3 * row], centerOffsetB[0] );
		offsetB = v3IvAdd( offsetB, v3IvScale( frame->cTilde.e[3 * row + 1], centerOffsetB[1] ) );
		offsetB = v3IvAdd( offsetB, v3IvScale( frame->cTilde.e[3 * row + 2], centerOffsetB[2] ) );
		v3Interval displacement = v3IvAdd( frame->wTilde.e[row], offsetB );
		displacement = v3IvSub( displacement, v3IvScale( frame->k, centerOffsetA[row] ) );
		double minimum = v3IvMig( displacement );
		squared = v3IvAdd( squared, v3IvSqr( v3IvPoint( minimum ) ) );
	}
	double centerDistance = v3IvSqrtLo( squared );
	double radius = v3IvUp( v3SeparationNorm3Up( halfExtentA ) + v3SeparationNorm3Up( halfExtentB ) );
	double cornerA[3], cornerB[3];
	for ( int axis = 0; axis < 3; ++axis )
	{
		cornerA[axis] = v3IvUp( fabs( centerOffsetA[axis] ) + halfExtentA[axis] );
		cornerB[axis] = v3IvUp( fabs( centerOffsetB[axis] ) + halfExtentB[axis] );
	}
	double chordSlack = v3IvUp( v3IvUp( frame->chordA * v3SeparationNorm3Up( cornerA ) ) +
								v3IvUp( frame->chordB * v3SeparationNorm3Up( cornerB ) ) );
	radius = v3IvUp( radius + chordSlack );
	double required = v3IvUp( v3IvUp( radius + sigma ) * v3IvMag( frame->k ) );
	return centerDistance > required;
}

v3SeparationVerdict v3SeparationCertify( const v3PairIntervalFrame* frame, const double halfExtentA[3],
										 const double centerOffsetA[3], const double halfExtentB[3],
										 const double centerOffsetB[3], double sigma, v3SeparationWitness* inOut )
{
	B3_ASSERT( frame != NULL && halfExtentA != NULL && centerOffsetA != NULL && halfExtentB != NULL && centerOffsetB != NULL );
	B3_ASSERT( inOut != NULL && isfinite( sigma ) );
	int cachedFamily = inOut->family;
	inOut->tier = v3_separationTierNone;
	inOut->familyTests = 0;

	// Tier one exploits temporal coherence before paying for any broad test
	if ( 0 <= cachedFamily && cachedFamily < 15 )
	{
		inOut->familyTests = 1;
		if ( v3ObbSatFamilyCertifiesSeparation( frame, halfExtentA, centerOffsetA, halfExtentB, centerOffsetB, sigma,
												cachedFamily ) )
		{
			inOut->tier = v3_separationTierCached;
			return v3_certifiedSeparated;
		}
	}

	// Tier two uses circumscribed leaf spheres over the interval frame as a cheap swept envelope
	if ( v3SeparationEnvelopeCertifies( frame, halfExtentA, centerOffsetA, halfExtentB, centerOffsetB, sigma ) )
	{
		inOut->family = V3_SEPARATION_FAMILY_ENVELOPE;
		inOut->tier = v3_separationTierEnvelope;
		return v3_certifiedSeparated;
	}

	// Tier three uses interval SAT with its fixed axis order and edge degeneracy policy.
	int family = v3ObbSatCertifySeparation( frame, halfExtentA, centerOffsetA, halfExtentB, centerOffsetB, sigma );
	inOut->familyTests += 15;
	if ( family >= 0 )
	{
		inOut->family = family;
		inOut->tier = v3_separationTierFamilies;
		return v3_certifiedSeparated;
	}
	inOut->family = V3_SEPARATION_FAMILY_NONE;
	return v3_notCertified;
}

double v3SeparationGapAtTime( const v3PairIntervalFrame* pointFrame, const double halfExtentA[3], const double centerOffsetA[3],
							  const double halfExtentB[3], const double centerOffsetB[3], int family )
{
	B3_ASSERT( pointFrame != NULL && pointFrame->t0 == pointFrame->t1 );
	B3_ASSERT( 0 <= family && family < 15 );
	v3Interval scaled = v3ObbSatFamilyGap( pointFrame, halfExtentA, centerOffsetA, halfExtentB, centerOffsetB, family );
	v3Interval scale = v3ObbSatFamilyScale( pointFrame, family );
	double divisor = scaled.lo < 0.0 ? scale.lo : scale.hi;
	if ( isfinite( scaled.lo ) == false || isfinite( divisor ) == false || divisor <= 0.0 )
	{
		return -INFINITY;
	}
	return v3IvDown( scaled.lo / divisor );
}

double v3SeparationAdvance( const v3SweepPoly* polyA, const v3SweepPoly* polyB, const v3PairIntervalFrame* frameAtT0, double rhoA,
							double rhoB, double gapLowerBound, double sigma )
{
	if ( polyA == NULL || polyB == NULL || frameAtT0 == NULL || isfinite( rhoA ) == false || isfinite( rhoB ) == false ||
		 rhoA < 0.0 || rhoB < 0.0 || isfinite( gapLowerBound ) == false || isfinite( sigma ) == false || gapLowerBound <= sigma )
	{
		return 0.0;
	}
	double relativeVelocity[3] = { polyB->dc[0] - polyA->dc[0], polyB->dc[1] - polyA->dc[1], polyB->dc[2] - polyA->dc[2] };
	double lambda = v3SeparationNorm3Up( relativeVelocity );
	lambda = v3IvUp( lambda + v3IvUp( polyA->angleFull * rhoA ) );
	lambda = v3IvUp( lambda + v3IvUp( polyB->angleFull * rhoB ) );
	if ( isfinite( lambda ) == false || lambda <= 0.0 )
	{
		return 0.0;
	}
	double numerator = v3IvDown( gapLowerBound - sigma );
	if ( numerator <= 0.0 )
	{
		return 0.0;
	}
	double dt = v3IvDown( numerator / lambda );
	double remaining = v3IvDown( 1.0 - frameAtT0->t0 );
	return fmax( 0.0, fmin( dt, remaining ) );
}

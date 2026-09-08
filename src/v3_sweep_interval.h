// SPDX-License-Identifier: MIT

#pragma once

#include "v3_interval.h"

#include "box3d/types.h"

#include <stdbool.h>

// Level-zero polynomial data shared by every leaf on one body during a step
//
// The coefficient arrays hold round-to-nearest doubles rather than endpoint
// pairs. Range consumers add one extra outward inflation after v3IvQuadRange
// so coefficient construction rounding is never mistaken for exact input
typedef struct v3SweepPoly
{
	double m0[9];
	double m1[9];
	double m2[9];
	double n0;
	double n1;
	double n2;
	double c1[3];
	double dc[3];
	double localCenter[3];
	double qHat1[4];
	double qHat2[4];
	double chordFull;
	double angleFull;
	float srcQ1[4];
	float srcQ2[4];
	float srcC1[3];
	float srcC2[3];
	float srcLocalCenter[3];
} v3SweepPoly;

bool v3SweepPolyBuild( const b3Sweep* sweep, v3SweepPoly* poly );
v3Interval v3SweepPolyDenominatorRange( const v3SweepPoly* poly, double t0, double t1 );
void v3SweepPolyMatrixRange( const v3SweepPoly* poly, double t0, double t1, v3IntervalMat3* unnormalized );
void v3SweepPolyCenterRange( const v3SweepPoly* poly, double t0, double t1, v3IntervalVec3* center );
double v3SweepPolyCornerEnvelope( const v3SweepPoly* poly, double rho, double t0, double t1 );

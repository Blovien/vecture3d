// SPDX-License-Identifier: MIT

#pragma once

#include "sweep_interval.h"

// Shape-pair data shared by every leaf pair over one time interval
typedef struct v3PairIntervalFrame
{
	v3IntervalMat3 cTilde;
	v3IntervalVec3 wTilde;
	v3Interval k;
	double t0;
	double t1;
	double chordA;
	double chordB;
} v3PairIntervalFrame;

bool v3PairIntervalFrameBuild( const v3SweepPoly* polyA, const v3SweepPoly* polyB, double t0, double t1,
							   v3PairIntervalFrame* frame );
v3Interval v3ObbSatFamilyGap( const v3PairIntervalFrame* frame, const double halfExtentA[3], const double centerOffsetA[3],
							  const double halfExtentB[3], const double centerOffsetB[3], int family );
v3Interval v3ObbSatFamilyScale( const v3PairIntervalFrame* frame, int family );
double v3ObbSatFamilyMargin( const v3PairIntervalFrame* frame, int family, double margin );
bool v3ObbSatFamilyCertifiesSeparation( const v3PairIntervalFrame* frame, const double halfExtentA[3],
										const double centerOffsetA[3], const double halfExtentB[3], const double centerOffsetB[3],
										double sigma, int family );
int v3ObbSatCertifySeparation( const v3PairIntervalFrame* frame, const double halfExtentA[3], const double centerOffsetA[3],
							   const double halfExtentB[3], const double centerOffsetB[3], double sigma );

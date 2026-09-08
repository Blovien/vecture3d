// SPDX-License-Identifier: MIT

#pragma once

#include "v3_obb_interval_sat.h"

#define V3_SEPARATION_FAMILY_NONE -1
#define V3_SEPARATION_FAMILY_ENVELOPE 0xFE

typedef enum v3SeparationVerdict
{
	v3_notCertified,
	v3_certifiedSeparated,
} v3SeparationVerdict;

typedef enum v3SeparationTier
{
	v3_separationTierNone,
	v3_separationTierCached,
	v3_separationTierEnvelope,
	v3_separationTierFamilies,
} v3SeparationTier;

typedef struct v3SeparationWitness
{
	int family;
	v3SeparationTier tier;
	int familyTests;
} v3SeparationWitness;

#define V3_SEPARATION_WITNESS_NONE { V3_SEPARATION_FAMILY_NONE, v3_separationTierNone, 0 }

v3SeparationVerdict v3SeparationCertify( const v3PairIntervalFrame* frame, const double halfExtentA[3],
										 const double centerOffsetA[3], const double halfExtentB[3],
										 const double centerOffsetB[3], double sigma, v3SeparationWitness* inOut );
double v3SeparationGapAtTime( const v3PairIntervalFrame* pointFrame, const double halfExtentA[3], const double centerOffsetA[3],
							  const double halfExtentB[3], const double centerOffsetB[3], int family );
double v3SeparationAdvance( const v3SweepPoly* polyA, const v3SweepPoly* polyB, const v3PairIntervalFrame* frameAtT0, double rhoA,
							double rhoB, double gapLowerBound, double sigma );

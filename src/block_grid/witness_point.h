// SPDX-License-Identifier: MIT

#pragma once

#include "sweep_interval.h"

#include <stdbool.h>

typedef struct v3WitnessPointResult
{
	bool certified;
	double point[3];
} v3WitnessPointResult;

// The alternating projections only choose a candidate and carry no soundness
// A true result comes solely from the interval comparisons against both inflated boxes
bool v3WitnessCertifyContact( const v3SweepPoly* polyA, const v3SweepPoly* polyB, const double halfExtentA[3],
							  const double centerOffsetA[3], const double halfExtentB[3], const double centerOffsetB[3], double t,
							  double delta, v3WitnessPointResult* result );

// Explicit poses use row-major rotations and body origins
// Point intervals make this the shared primitive for float-pose re-certification
bool v3WitnessCertifyContactAtPose( const double rotationA[9], const double originA[3], const double halfExtentA[3],
									const double centerOffsetA[3], const double rotationB[9], const double originB[3],
									const double halfExtentB[3], const double centerOffsetB[3], double delta,
									v3WitnessPointResult* result );

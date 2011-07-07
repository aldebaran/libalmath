/**
 * @author Cyrille Collette - ccollette@aldebaran-robotics.com
 * Aldebaran Robotics (c) 2010 All Rights Reserved
 *
 */

#pragma once

#ifndef _LIB_ALMATH_TEST_ALMATHTESTUTILS_H_
#define _LIB_ALMATH_TEST_ALMATHTESTUTILS_H_

//#include <gtest/gtest.h>
#include <almath/tools/almath.h>
#include <stdexcept>

static const float kEpsilon = 1.0e-4f;

void validateFindRotation(
    const AL::Math::Position3D& pA,
    const AL::Math::Position3D& pB);


#endif  // _LIB_ALMATH_TEST_ALMATHTESTUTILS_H_

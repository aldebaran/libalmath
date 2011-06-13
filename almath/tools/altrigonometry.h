/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/


#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALTRIGONOMETRY_H_
#define _LIB_ALMATH_ALMATH_ALTRIGONOMETRY_H_

#include <math.h>

static const float _4_PI_ = 12.56637056f;
static const float _2_PI_ = 6.28318528f;
static const float PI     = 3.14159265358979323846f;
static const float PI_2   = 1.57079632679489661923f;
static const float PI_4   = 0.785398163397448309616f;
static const float TO_RAD = 0.017453292f;
static const float TO_DEG = 57.295779579f;

namespace AL {
  namespace Math {


    /****************************************************
    *****************************************************
    * *     COS, SIN, ACOS, ASIN LOOKUP TABLES :      * *
    *****************************************************
    ****************************************************/
    static const int inlineCosSinArraySize = 16384; // gives an approximation at 4.10-4 rad
    static const int inlineACosASinArraySize = 16384;

    extern float inlineCosArray[inlineCosSinArraySize];
    extern float inlineSinArray[inlineCosSinArraySize];
    extern float inlineACosArray[inlineACosASinArraySize];
    extern float inlineASinArray[inlineACosASinArraySize];

    static const float freqForInlineCosSin = (float)inlineCosSinArraySize / (_2_PI_ );
    static const float stepForInlineCosSin = _2_PI_ / (float)inlineCosSinArraySize;

    static const float freqForInlineACosASin = (float)inlineACosASinArraySize / (2.0f);
    static const float stepForInlineACosASin = (2.0f) / (float)inlineACosASinArraySize;

    extern bool inlineCosSinInitialized;
    extern bool inlineACosASinInitialized;

    /*
    * initialize the tables
    */
    void initInlineCosSin();
    void initInlineACosASin();

    /*
    * return cos(n) in rad
    */
    float inlineCos(float n);

    /*
    * return sin(n) in rad
    */
    float inlineSin(float n);

    /*
    * return ArcCos(n) if -1 <= n <= 1
    * return value between 0 and PI
    */
    float inlineACos(float n);

    /*
    * return ArcSin(n) if -1 <= n <= 1
    * return value between -PI_2 and +PI_2
    */
    float inlineASin(float n);

  }
}
#endif  // _LIB_ALMATH_ALMATH_ALTRIGONOMETRY_H_

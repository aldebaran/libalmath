/**
* Copyright (c) Aldebaran Robotics 2009 All Rights Reserved
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALAXISMASK_H_
#define _LIB_ALMATH_ALMATH_ALAXISMASK_H_

#include <bitset>

namespace AL {
  namespace Math {
    /// AXIS MASKS
    typedef std::bitset<6> AXIS_MASK;
    static const int AXIS_MASK_X    =  1;
    static const int AXIS_MASK_Y    =  2;
    static const int AXIS_MASK_XY   =  3;
    static const int AXIS_MASK_Z    =  4;
    static const int AXIS_MASK_WX   =  8;
    static const int AXIS_MASK_WY   = 16;
    static const int AXIS_MASK_WZ   = 32;
    static const int AXIS_MASK_WYWZ = 48;
    static const int AXIS_MASK_ALL  = 63;
    static const int AXIS_MASK_VEL  =  7;
    static const int AXIS_MASK_ROT  = 56;
    static const int AXIS_MASK_NONE =  0;
  }
}
#endif  // _LIB_ALMATH_ALMATH_ALAXISMASK_H_

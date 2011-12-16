/*
 * Copyright (c) 2012, Aldebaran Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Aldebaran Robotics nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Aldebaran Robotics BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#pragma once
#ifndef _LIBALMATH_ALMATH_TYPES_ALAXISMASK_H_
#define _LIBALMATH_ALMATH_TYPES_ALAXISMASK_H_

#include <bitset>

namespace AL {
  namespace Math {

    /// <summary>
    /// Definition of an AXIS_MASK as a bit set.
    ///
    /// static const int AXIS_MASK_X    =  1 \n
    /// static const int AXIS_MASK_Y    =  2 \n
    /// static const int AXIS_MASK_XY   =  3 \n
    /// static const int AXIS_MASK_Z    =  4 \n
    /// static const int AXIS_MASK_WX   =  8 \n
    /// static const int AXIS_MASK_WY   = 16 \n
    /// static const int AXIS_MASK_WZ   = 32 \n
    /// static const int AXIS_MASK_WYWZ = 48 \n
    /// static const int AXIS_MASK_ALL  = 63 \n
    /// static const int AXIS_MASK_VEL  =  7 \n
    /// static const int AXIS_MASK_ROT  = 56 \n
    /// static const int AXIS_MASK_NONE =  0 \n
    /// </summary>
    /// \ingroup Types
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
#endif  // _LIBALMATH_ALMATH_TYPES_ALAXISMASK_H_

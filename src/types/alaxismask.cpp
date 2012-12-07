/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/types/alaxismask.h>

namespace AL {
  namespace Math {

    bool isAxisMask(const int pAxisMask)
    {
      if (pAxisMask < 64 && pAxisMask >= 1)
      {
        return true;
      }
      return false;
    } // end isAxisMask

  }
}


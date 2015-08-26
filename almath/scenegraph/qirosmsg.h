/*
 * Copyright (c) 2016 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#pragma once
#ifndef _LIBALMATH_ALMATH_SCENEGRAPH_QIROSMSG_H_
#define _LIBALMATH_ALMATH_SCENEGRAPH_QIROSMSG_H_

#include <qi/clock.hpp>
#include <ros/time.h>

namespace AL {
namespace Math {

// return a ros::Time, clipped to [ros::TIME_MIN, ros::TIME_MAX]
inline ros::Time toValidRosTime(qi::SystemClock::time_point time) {
  // We need to care for several problems:
  //
  // * ros::Time::fromNSec expects an unsigned number of nano seconds,
  //   but qi::SystemClock::time_point is signed
  //
  // * ros::Time range is smaller than qi::SystemClock::time_point range
  //   (many bits are wasted in ros::Time because it splits seconds and
  //   nanoseconds in two 32 bits members)
  //
  // * ros::Time(0, 0) stands for "Not a Time". Thus
  //   ros::TIME_MIN == ros::Time(0, 1)
  //
  //  In the end we have:
  //
  //    using qiTime = qi::SystemClock::time_point;
  //    qiTime::min() < 0 < ros::TIME_MIN < ros::TIME_MAX < qiTime::max()
  using qirep = qi::SystemClock::time_point::duration::rep;
  constexpr qirep rosMin = 1;
  constexpr qirep rosMax =
      999999999 +
      static_cast<qirep>(std::numeric_limits<uint32_t>::max()) * 1000000000;
  assert(rosMin == static_cast<qirep>(ros::TIME_MIN.toNSec()));
  assert(rosMax == static_cast<qirep>(ros::TIME_MAX.toNSec()));
  ros::Time ret;
  ret.fromNSec(
      std::min(rosMax, std::max(rosMin, time.time_since_epoch().count())));
  return ret;
}
}
}
#endif

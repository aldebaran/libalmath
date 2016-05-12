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
//
// Note:
//
// altough this can be changed with an #ifdef, ros::Time typically uses
// std::chrono::system_clock as its underlying time source.
// As a consequence, ros::Time typically matches libqi's
// qi::SystemClock::time_point.
//
// However, since the system clock is subject to clock jumps, we tend
// to favour qi::Clock for timestamping sensor data, and when mapping
// this data to ros messages for offline use, it is more convenient
// to set the ros::Time with the value from qi::Clock instead of
// qi::SystemClock.
//
// Given that both use case are valid, this conversion function does not
// operate on qi::Clock::time_point nor qi::SystemClock::time_point but
// rather on qi::Duration.
inline ros::Time toValidRosTime(qi::Duration time_since_epoch) {
  // We need to care for several problems:
  //
  // * ros::Time::fromNSec expects an unsigned number of nano seconds,
  //   but libqi clocks use signed time_points
  //
  // * ros::Time range is smaller than libqi clocks range
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
  using qirep = qi::Duration::rep;
  const qirep rosMin = 1;
  const qirep rosMax =
      999999999 +
      static_cast<qirep>(std::numeric_limits<uint32_t>::max()) * 1000000000;
  assert(rosMin == static_cast<qirep>(ros::TIME_MIN.toNSec()));
  assert(rosMax == static_cast<qirep>(ros::TIME_MAX.toNSec()));
  ros::Time ret;
  ret.fromNSec(std::min(rosMax, std::max(rosMin, time_since_epoch.count())));
  return ret;
}

inline qi::Duration toQiDuration(ros::Time time) {
  using qirep = qi::Duration::rep;
  // if time <= ros::TIME_MAX, we know it can be casted to a qirep,
  // which is signed, without wrapping over.
  assert(time <= ros::TIME_MAX);
  return qi::Duration(static_cast<qirep>(time.toNSec()));
}
}
}
#endif

/**
 * @author Lucas Souchet - lsouchet@aldebaran.com
 * Aldebaran Robotics (c) 2016 All Rights Reserved - This file is confidential.
 *
 * the user is expected to
 *  #include <qi/geometry/geometry.hpp>
 * and link with qigeometry
 */

#pragma once
#ifndef LIBALMATH_SCENEGRAPH_ALMATHQIGEOMETRY_HPP
#define LIBALMATH_SCENEGRAPH_ALMATHQIGEOMETRY_HPP

#include <almath/tools/altransformhelpers.h>
#include <almath/scenegraph/qigeometry.h>

namespace AL
{
namespace Math
{

  inline Transform transformFromQiTransform(const qi::geometry::Transform& r)
  {
    Quaternion almathQuat(r.rotation.w, r.rotation.x, r.rotation.y,
                          r.rotation.z);
    Transform almathTf = transformFromQuaternion(almathQuat);
    almathTf.r1_c4 = r.translation.x;
    almathTf.r2_c4 = r.translation.y;
    almathTf.r3_c4 = r.translation.z;
    return almathTf;
  }

  inline AL::Math::Pose2D pose2DFromQiTransform(
      const qi::geometry::Transform& r)
  {
    return pose2DFromTransform(transformFromQiTransform(r));
  }

  inline qi::geometry::Transform qiTransformFromPose2D(const Pose2D& pose)
  {
    const Quaternion quat = quaternionFromAngleAndAxisRotation(
          pose.theta, 0.0f, 0.0f, 1.0f);
    return qi::geometry::makeTransform(
          qi::geometry::makeQuaternion(quat.x, quat.y, quat.z, quat.w),
          qi::geometry::makeVector3(pose.x, pose.y, 0.0f));
  }

}
}
#endif

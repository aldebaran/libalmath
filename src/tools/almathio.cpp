/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/tools/almathio.h>

namespace AL {
  namespace Math {


    std::ostream& operator<< (std::ostream& pStream, const Pose2D& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{x: ";
      pStream << p.x;
      pStream << ", y:";
      pStream << p.y;
      pStream << ", theta:";
      pStream << p.theta;
      pStream << "}";
      return pStream;
    }

    std::ostream& operator<< (std::ostream& pStream, const Position2D& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{x: ";
      pStream << p.x;
      pStream << ", y:";
      pStream << p.y;
      pStream << "}";
      return pStream;
    }

    std::ostream& operator<< (std::ostream& pStream, const Position3D& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{x: ";
      pStream << p.x;
      pStream << ", y:";
      pStream << p.y;
      pStream << ", z:";
      pStream << p.z;
      pStream << "}";
      return pStream;
    }

    std::ostream& operator<< (std::ostream& pStream, const Position6D& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{x: ";
      pStream << p.x;
      pStream << ", y: ";
      pStream << p.y;
      pStream << ", z: ";
      pStream << p.z;
      pStream << ", wx: ";
      pStream << p.wx;
      pStream << ", wy: ";
      pStream << p.wy;
      pStream << ", wz: ";
      pStream << p.wz;
      pStream << "}";
      return pStream;
    }

    std::ostream& operator<< (std::ostream& pStream, const PositionAndVelocity& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{q: ";
      pStream << p.q;
      pStream << ", dq: ";
      pStream << p.dq;
      pStream << "}";
      return pStream;
    }


    std::ostream& operator<< (std::ostream& pStream, const Rotation& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << p.r1_c1;
      pStream << " ";
      pStream << p.r1_c2;
      pStream << " ";
      pStream << p.r1_c3;
      pStream << std::endl;
      pStream << p.r2_c1;
      pStream << " ";
      pStream << p.r2_c2;
      pStream << " ";
      pStream << p.r2_c3;
      pStream << std::endl;
      pStream << p.r3_c1;
      pStream << " ";
      pStream << p.r3_c2;
      pStream << " ";
      pStream << p.r3_c3;
      pStream << std::endl;
      return pStream;
    }


    std::ostream& operator<< (std::ostream& pStream, const Rotation3D& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{wx: ";
      pStream << p.wx;
      pStream << " ,wy: ";
      pStream << p.wy;
      pStream << " ,wz: ";
      pStream << p.wz;
      pStream << "}";
      return pStream;
    }

    std::ostream& operator<< (std::ostream& pStream, const Transform& pT)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      //pStream << std::setw(10) << std::setprecision(6);
      pStream << pT.r1_c1;
      pStream << " ";
      pStream << pT.r1_c2;
      pStream << " ";
      pStream << pT.r1_c3;
      pStream << " ";
      pStream << pT.r1_c4;
      pStream << std::endl;
      pStream << pT.r2_c1;
      pStream << " ";
      pStream << pT.r2_c2;
      pStream << " ";
      pStream << pT.r2_c3;
      pStream << " ";
      pStream << pT.r2_c4;
      pStream << std::endl;
      pStream << pT.r3_c1;
      pStream << " ";
      pStream << pT.r3_c2;
      pStream << " ";
      pStream << pT.r3_c3;
      pStream << " ";
      pStream << pT.r3_c4;
      pStream << std::endl;
      pStream << "0.0 0.0 0.0 1.0";
      pStream << std::endl;
      return pStream;
    }


    std::ostream& operator<< (std::ostream& pStream, const TransformAndVelocity6D& pDat)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      //pStream << std::setw(10) << std::setprecision(6);
      pStream << pDat.T.r1_c1;
      pStream << " ";
      pStream << pDat.T.r1_c2;
      pStream << " ";
      pStream << pDat.T.r1_c3;
      pStream << " ";
      pStream << pDat.T.r1_c4;
      pStream << std::endl;
      pStream << pDat.T.r2_c1;
      pStream << " ";
      pStream << pDat.T.r2_c2;
      pStream << " ";
      pStream << pDat.T.r2_c3;
      pStream << " ";
      pStream << pDat.T.r2_c4;
      pStream << std::endl;
      pStream << pDat.T.r3_c1;
      pStream << " ";
      pStream << pDat.T.r3_c2;
      pStream << " ";
      pStream << pDat.T.r3_c3;
      pStream << " ";
      pStream << pDat.T.r3_c4;
      pStream << std::endl;
      pStream << "0.0 0.0 0.0 1.0";
      pStream << std::endl;
      pStream << "{xd: ";
      pStream << pDat.V.xd;
      pStream << " ,yd: ";
      pStream << pDat.V.yd;
      pStream << " ,zd: ";
      pStream << pDat.V.zd;
      pStream << " ,wxd: ";
      pStream << pDat.V.wxd;
      pStream << " ,wyd: ";
      pStream << pDat.V.wyd;
      pStream << " ,wzd: ";
      pStream << pDat.V.wzd;
      pStream << "}";
      return pStream;
    }

    std::ostream& operator<< (std::ostream& pStream, const Quaternion& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{w: ";
      pStream << p.w;
      pStream << ", x:";
      pStream << p.x;
      pStream << ", y:";
      pStream << p.y;
      pStream << ", z:";
      pStream << p.z;
      pStream << "}";
      return pStream;
    }

    std::ostream& operator<< (std::ostream& pStream, const Velocity3D& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{xd: ";
      pStream << p.xd;
      pStream << ", yd:";
      pStream << p.yd;
      pStream << ", zd:";
      pStream << p.zd;
      pStream << "}";
      return pStream;
    }

    std::ostream& operator<< (std::ostream& pStream, const Velocity6D& p)
    {
      pStream.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      pStream << "{xd: ";
      pStream << p.xd;
      pStream << " ,yd: ";
      pStream << p.yd;
      pStream << " ,zd: ";
      pStream << p.zd;
      pStream << " ,wxd: ";
      pStream << p.wxd;
      pStream << " ,wyd: ";
      pStream << p.wyd;
      pStream << " ,wzd: ";
      pStream << p.wzd;
      pStream << "}";
      return pStream;
    }

    std::string toSpaceSeparated(const Velocity6D& p)
    {
      std::stringstream os;
      os << p;
      return os.str();
    }

    std::string toSpaceSeparated(const Transform& pT)
    {
      std::stringstream os;
      os << pT;
      return os.str();
    }

    std::string toSpaceSeparated(const Position3D& p)
    {
      std::stringstream os;
      os << p;
      return os.str();
    }

    std::string toSpaceSeparated(const Rotation3D& p)
    {
      std::stringstream os;
      os << p;
      return os.str();
    }

    std::string toSpaceSeparated(const Position6D& p)
    {
      std::stringstream os;
      os << p;
      return os.str();
    }

    std::string toSpaceSeparated(const Quaternion &pQuat)
    {
      std::stringstream os;
      os << pQuat;
      return os.str();
    }

  }
}

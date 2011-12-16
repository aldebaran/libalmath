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
      pStream << " ";
      pStream << p.r2_c1;
      pStream << " ";
      pStream << p.r2_c2;
      pStream << " ";
      pStream << p.r2_c3;
      pStream << " ";
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
      os.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      os << p.xd << " ";
      os << p.yd << " ";
      os << p.zd << " ";
      os << p.wxd << " ";
      os << p.wyd << " ";
      os << p.wzd << " ";
      return os.str();
    }

     std::string toSpaceSeparated(const Transform& pT)
     {
        std::stringstream os;
        os.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
        os << pT.r1_c1 << " ";
        os << pT.r1_c2 << " ";
        os << pT.r1_c3 << " ";
        os << pT.r1_c4 << " ";
        os << pT.r2_c1 << " ";
        os << pT.r2_c2 << " ";
        os << pT.r2_c3 << " ";
        os << pT.r2_c4 << " ";
        os << pT.r3_c1 << " ";
        os << pT.r3_c2 << " ";
        os << pT.r3_c3 << " ";
        os << pT.r3_c4 << " ";
        os << "0.0 0.0 0.0 1.0 ";
        return os.str();
    }

    std::string toSpaceSeparated(const Position6D& p)
    {
      std::stringstream os;
      os.setf(std::ios::showpoint | std::ios::left | std::ios::showpos);
      os << p.x << " ";
      os << p.y << " ";
      os << p.z << " ";
      os << p.wx << " ";
      os << p.wy << " ";
      os << p.wz << " ";
      return os.str();
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

  }
}

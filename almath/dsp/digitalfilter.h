/*
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#pragma once

#ifndef _LIB_ALMATH_ALMATH_DSP_DIGITALFILTER_H_
#define _LIB_ALMATH_ALMATH_DSP_DIGITALFILTER_H_

#include <vector>
#include <boost/circular_buffer.hpp>

namespace AL
{
namespace Math
{
namespace DSP
{

class DigitalFilter
{
public:
  DigitalFilter(void);
  ~DigitalFilter(void);

  /*! \fn AL::Math::DSP::DigitalFilter::configureFilter(
                                 const std::vector<float> & weightsIn,
                                 const std::vector<float> & weightsOut,
                                 const float dcGain)
    \brief Configure the digital filter
    \param weightsIn A vector of float describing weights applied on input vector
    \param weightsOut A vector of float describing weights applied on output vector
    \param dcGain Static gain of the filter
  */
  void configureFilter(const std::vector<float> & pWeightsIn,
                       const std::vector<float> & pWeightsOut,
                       float pDcGain);

  /*! \fn void AL::Math::DSP::DigitalFilter::resetFilter()
    \brief Reset the processing of the filter
  */
  void resetFilter();

  /*! \fn float AL::Math::DSP::DigitalFilter::processFilter(const float inputData)
    \brief Process a step of the filter
    \param in Signal input
  */
  float processFilter(float pInputData);

private:
  boost::circular_buffer<float> fFilterBufferIn;
  boost::circular_buffer<float> fFilterBufferOut;

  float fFilterDcGain;

  std::vector<float> fFilterWeightsIn;
  std::vector<float> fFilterWeightsOut;

};
}
}
}





#endif  // _LIB_ALMATH_ALMATH_DSP_DIGITALFILTER_H_

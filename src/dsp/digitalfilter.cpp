/*
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/dsp/digitalfilter.h>

namespace AL
{
namespace Math
{
namespace DSP
{

DigitalFilter::DigitalFilter(void)
  :fFilterOrder(0u)
  ,fFilterDcGain(1.0f)
  ,fFilterWeightsIn(1u, 1.0f)
{}

DigitalFilter::~DigitalFilter(void){}

void DigitalFilter::configureFilter(const unsigned int pOrder,
                                    const std::vector<float> &pWeightsIn,
                                    const std::vector<float> &pWeightsOut,
                                    const float pDcGain)
{
  fFilterOrder = pOrder;
  fFilterDcGain = pDcGain;
  fFilterWeightsIn = pWeightsIn;
  fFilterWeightsOut = pWeightsOut;
}

void DigitalFilter::resetFilter()
{
  fFilterBufferIn.clear();
  fFilterBufferOut.clear();
}

float DigitalFilter::processFilter(const float pInputData)
{
  float lOutputData = pInputData;
  fFilterBufferIn.push_back(pInputData/fFilterDcGain);

  if (fFilterBufferIn.size()>fFilterOrder)
  {
    lOutputData = 0.0f;

    for(unsigned int i=0; i < fFilterOrder+1; ++i)
    {
      lOutputData += fFilterBufferIn[i] * fFilterWeightsIn[i];
    }
    for(unsigned int i=0; i < fFilterOrder; ++i)
    {
      lOutputData += fFilterBufferOut[i] * fFilterWeightsOut[i];
    }

    fFilterBufferIn.pop_front();
    fFilterBufferOut.pop_front();
  }

  fFilterBufferOut.push_back(lOutputData);

  return lOutputData;
}
}
}
}


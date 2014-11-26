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
  :fFilterDcGain(1.0f)
  ,fFilterWeightsIn(1u, 1.0f)
{}

DigitalFilter::~DigitalFilter(void){}

void DigitalFilter::configureFilter(const std::vector<float> &pWeightsIn,
                                    const std::vector<float> &pWeightsOut,
                                    const float pDcGain)
{
  fFilterDcGain = pDcGain;
  fFilterWeightsIn = pWeightsIn;
  fFilterWeightsOut = pWeightsOut;
  fFilterBufferIn.set_capacity(pWeightsIn.size());
  fFilterBufferOut.set_capacity(pWeightsOut.size());
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

  if (fFilterBufferIn.full() && fFilterBufferOut.full())
  {
    lOutputData = 0.0f;

    for(unsigned int i=0; i < fFilterWeightsIn.size(); ++i)
    {
      lOutputData += fFilterBufferIn[i] * fFilterWeightsIn[i];
    }
    for(unsigned int i=0; i < fFilterWeightsOut.size(); ++i)
    {
      lOutputData += fFilterBufferOut[i] * fFilterWeightsOut[i];
    }
  }

  fFilterBufferOut.push_back(lOutputData);

  return lOutputData;
}
}
}
}


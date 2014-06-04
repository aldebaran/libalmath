/*
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <almath/dsp/digitalfilter.h>
#include <vector>
#include <gtest/gtest.h>

TEST(DigitalFilter, testRecurence)
{
  std::vector<float> input;
  input.push_back(1.0f);
  input.push_back(-0.55f);
  input.push_back(45.6f);
  input.push_back(0.085f);
  input.push_back(15.2f);

  const unsigned int order = 2;

  std::vector<float> weightX;
  weightX.push_back(1.0f);
  weightX.push_back(-1.0f);
  weightX.push_back(-0.5f);

  std::vector<float> weightY;
  weightY.push_back(0.0f);
  weightY.push_back(0.0f);

  const float dcGain = 1.0f;

  AL::Math::DSP::DigitalFilter filter = AL::Math::DSP::DigitalFilter();

  filter.configureFilter(order, weightX, weightY, dcGain);

  for(unsigned int i=0; i < input.size(); ++i)
  {
    if (i<2)
    {
        const float output = filter.processFilter(input[i]);
        EXPECT_NEAR(output, input[i], 0.00001f);
    }
    else
    {
        const float output = filter.processFilter(input[i]);
        EXPECT_NEAR(output, input[i-2]*weightX[0]+input[i-1]*weightX[1]+input[i]*weightX[2], 0.00001f);
    }
  }

}

TEST(DigitalFilter, testWeightsY)
{
  std::vector<float> input;
  input.push_back(1.0f);
  input.push_back(-0.55f);
  input.push_back(45.6f);
  input.push_back(0.085f);
  input.push_back(15.2f);

  const unsigned int order = 2;

  std::vector<float> weightX;
  weightX.push_back(1.0f);
  weightX.push_back(-1.0f);
  weightX.push_back(-0.5f);

  std::vector<float> weightY;
  weightY.push_back(1.0f);
  weightY.push_back(0.5f);

  const float dcGain = 1.0f;

  AL::Math::DSP::DigitalFilter filter = AL::Math::DSP::DigitalFilter();

  filter.configureFilter(order, weightX, weightY, dcGain);

  std::vector<float> outputVector;

  for(unsigned int i=0; i < input.size(); ++i)
  {
    float output = 0.0f;
    if (i<2)
    {
        output = filter.processFilter(input[i]);
        EXPECT_NEAR(output, input[i], 0.00001f);
    }
    else
    {
        output = filter.processFilter(input[i]);
        EXPECT_NEAR(
              output,
              input[i-2]*weightX[0] +
              input[i-1]*weightX[1] +
              input[i]*weightX[2] +
              outputVector[i - 2]*weightY[0]+
              outputVector[i - 1]*weightY[1]
            , 0.00001f);
    }
    outputVector.push_back(output);
  }

}

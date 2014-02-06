/*
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/dsp/pidcontroller.h>

#include <gtest/gtest.h>

#include <stdexcept>

TEST(PIDControllerTest, t_00)
{
  ASSERT_NO_THROW(
        AL::Math::DSP::PIDController(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f));

  ASSERT_THROW(
        AL::Math::DSP::PIDController(-0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f),
        std::runtime_error);

  ASSERT_THROW(
        AL::Math::DSP::PIDController(0.0f, -0.1f, 0.0f, 0.0f, 0.0f, 0.01f),
        std::runtime_error);

  ASSERT_THROW(
        AL::Math::DSP::PIDController(0.0f, 0.0f, -0.1f, 0.0f, 0.0f, 0.01f),
        std::runtime_error);

  ASSERT_THROW(
        AL::Math::DSP::PIDController(0.0f, 0.0f, 0.0f, -0.1f, 0.0f, 0.01f),
        std::runtime_error);

  ASSERT_THROW(
        AL::Math::DSP::PIDController(0.0f, 0.0f, 0.0f, 0.0f, -0.1f, 0.01f),
        std::runtime_error);

  ASSERT_THROW(
        AL::Math::DSP::PIDController(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
        std::runtime_error);
}

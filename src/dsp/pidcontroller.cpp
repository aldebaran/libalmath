/*
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <almath/dsp/pidcontroller.h>
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace AL
{
namespace Math
{
namespace DSP
{

// default contructor
PIDController::PIDController(void)
  :fKp(0.0f)
  ,fKv(0.0f)
  ,fKi(0.0f)
  ,fPeriod(1.0f)
  ,fThreshold(0.0f)
  ,fStaticOffset(0.0f)
  ,fErr(0.0f)
  ,fPreviousErr(0.0f)
  ,fdErr(0.0f)
  ,fiErr(0.0f)
  ,fFirstIteration(true)
{}


PIDController::PIDController(const float pKp,
                             const float pKv,
                             const float pKi,
                             const float pThreshold,
                             const float pStaticOffset,
                             const float pPeriod)
  :fKp(pKp)
  ,fKv(pKv)
  ,fKi(pKi)
  ,fPeriod(pPeriod)
  ,fThreshold(pThreshold)
  ,fStaticOffset(pStaticOffset)
  ,fErr(0.0f)
  ,fPreviousErr(0.0f)
  ,fdErr(0.0f)
  ,fiErr(0.0f)
  ,fFirstIteration(true)
{
  xCheckData();
}


PIDController::~PIDController(){}


void PIDController::initialize()
{
  xResetParameters();
}

void PIDController::initialize(const float pKp,
                               const float pKv,
                               const float pKi,
                               const float pThreshold,
                               const float pStaticOffset,
                               const float pPeriod)
{
  fKp = pKp;
  fKv = pKv;
  fKi = pKi;
  fThreshold    = pThreshold;
  fStaticOffset = pStaticOffset;
  fPeriod       = pPeriod;

  xCheckData();
  xResetParameters();
}

float PIDController::computeFeedbackAbsolute(const float pAbsoluteErr)
{
  fPreviousErr = fErr;
  fErr  = pAbsoluteErr;

  // Creation of Threshold Action for injection of Sensor
  if (fErr > fThreshold)
  {
    fErr = fErr-fThreshold;
  }
  else
  {
    fErr = 0.0f;
  }

  if (fFirstIteration)
  {
    fdErr = 0.0f;
    fFirstIteration = false;
  }
  else
  {
    fdErr = std::abs(fErr-fPreviousErr)/fPeriod;
  }

  fiErr += fErr*fPeriod;

  return fStaticOffset + fKp*fErr + fKv*fdErr + fKi*fiErr;
}

void PIDController::setGains(float pKp, float pKv, float pKi)
{
  xCheckPositif(pKp);
  xCheckPositif(pKv);
  xCheckPositif(pKi);
  fKp = pKp;
  fKv = pKv;
  fKi = pKi;
}


float PIDController::computeFeedback(const float pCommand,
                                     const float pSensor,
                                     const float pPeriod)
{
  float period = fPeriod;
  if (pPeriod > 0.0)
  {
    period = fPeriod;
  }

  fPreviousErr = fErr;
  fErr  = pCommand-pSensor;

  // Creation of Threshold Action for injection of Sensor
  if (fErr > fThreshold)
  {
    fErr = fErr-fThreshold;
  }
  else if (fErr < -fThreshold)
  {
    fErr = fErr+fThreshold;
  }
  else
  {
    fErr = 0.0f;
  }

  if (fFirstIteration)
  {
    fdErr = 0.0f;
    fFirstIteration = false;
  }
  else
  {
    fdErr = (fErr-fPreviousErr)/period;
  }

  fiErr += fErr*period;

  return fStaticOffset + fKp*fErr + fKv*fdErr + fKi*fiErr;
}


void PIDController::xResetParameters(void)
{
  fErr  = 0.0f;
  fdErr = 0.0f;
  fiErr = 0.0f;
  fFirstIteration = true;
}

void PIDController::xCheckData(void) const
{
  xCheckPositif(fKp);
  xCheckPositif(fKv);
  xCheckPositif(fKi);
  xCheckPositif(fThreshold);
  xCheckPositif(fStaticOffset);
  xCheckStriclyPositif(fPeriod);
}


void PIDController::xCheckPositif(const float pVal) const
{
  if (pVal < 0.0f)
  {
    // todo: print the given value
    // using boost::lexical_cast<std::string>(pVal)
    throw std::runtime_error(
          "ALMath: PIDController "
          "Must be >= 0.0.");
  }
}

void PIDController::xCheckStriclyPositif(const float pVal) const
{
  if (pVal <= 0.0f)
  {
    // todo: print the given value
    // using boost::lexical_cast<std::string>(pVal)
    throw std::runtime_error(
          "ALMath: PIDController "
          "Must be stricly positif.");
  }
}
}
}
}

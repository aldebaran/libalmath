## Copyright (c) 2012, Aldebaran Robotics
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##     * Redistributions of source code must retain the above copyright
##       notice, this list of conditions and the following disclaimer.
##     * Redistributions in binary form must reproduce the above copyright
##       notice, this list of conditions and the following disclaimer in the
##       documentation and/or other materials provided with the distribution.
##     * Neither the name of the Aldebaran Robotics nor the
##       names of its contributors may be used to endorse or promote products
##       derived from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
## ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
## WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
## DISCLAIMED. IN NO EVENT SHALL Aldebaran Robotics BE LIABLE FOR ANY
## DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
## (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
## LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
## ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
## SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
project(ALMATH)

cmake_minimum_required(VERSION 2.6.4)
include("qibuild.cmake")

enable_testing()

# Add BEFORE keyword so that we can rebuild almath against a sdk
# without getting conflicts.
include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}")

set( ALMATH_SRCS
    src/tools/avoidfootcollision.cpp
    src/tools/almath.cpp
    src/tools/almathio.cpp
    src/tools/aldubinscurve.cpp
    src/tools/altransformhelpers.cpp
    src/types/alpose2d.cpp
    src/types/alrotation3d.cpp
    src/types/alrotation.cpp
    src/types/alpositionandvelocity.cpp
    src/types/altransformandvelocity6d.cpp
    src/types/altransform.cpp
    src/types/alvelocity3d.cpp
    src/types/alvelocity6d.cpp
    src/types/alposition2d.cpp
    src/types/alposition3d.cpp
    src/types/alposition6d.cpp
    src/types/alquaternion.cpp
)

set(ALMATH_H
    almath/tools/avoidfootcollision.h
    almath/tools/almath.h
    almath/tools/almathio.h
    almath/tools/aldubinscurve.h
    almath/tools/altransformhelpers.h
    almath/tools/altrigonometry.h
    almath/types/alaxismask.h
    almath/types/alpose2d.h
    almath/types/alposition2d.h
    almath/types/alposition3d.h
    almath/types/alposition6d.h
    almath/types/alpositionandvelocity.h
    almath/types/alrotation3d.h
    almath/types/alrotation.h
    almath/types/altransformandvelocity6d.h
    almath/types/altransform.h
    almath/types/alvelocity3d.h
    almath/types/alvelocity6d.h
    almath/types/alquaternion.h
)

qi_create_lib(almath ${ALMATH_SRCS} ${ALMATH_H})

qi_stage_lib(almath ALMATH)

option(ALMATH_PYTHON_BINDING
    "Build almath python binding using Swig."
    ON)
qi_install_header(${ALMATH_H} KEEP_RELATIVE_PATHS)

# ALMath python bindings. Optional for now.
if (ALMATH_PYTHON_BINDING STREQUAL "ON")
  #use(PYTHON-TOOLS)
  include(qibuild/swig/python)
  qi_swig_wrap_python(almath almath.i SRC
    src/types/alpose2d.cpp
    DEPENDS ALMATH)
  #add_python_test(TestALMathPy "${CMAKE_CURRENT_SOURCE_DIR}/test/almath_test.py")
endif()

add_subdirectory(test)

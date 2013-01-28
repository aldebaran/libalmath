## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.
project(ALMATH)

cmake_minimum_required(VERSION 2.6.4)
find_package(qibuild)

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
    src/types/alaxismask.cpp
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
    src/types/aldisplacement.cpp
)

# list of the public headers which are wrapped
# check it against swig -MM almath.i
set(ALMATH_H_WRAPPED
    almath/types/alaxismask.h
    almath/types/alpose2d.h
    almath/types/alposition2d.h
    almath/types/alposition3d.h
    almath/types/alposition6d.h
    almath/types/alpositionandvelocity.h
    almath/types/alquaternion.h
    almath/types/aldisplacement.h
    almath/types/alrotation.h
    almath/types/alrotation3d.h
    almath/types/altransform.h
    almath/types/alvelocity3d.h
    almath/types/alvelocity6d.h
    almath/types/altransformandvelocity6d.h
    almath/tools/aldubinscurve.h
    almath/tools/altrigonometry.h
    almath/tools/avoidfootcollision.h
    almath/tools/altransformhelpers.h
    almath/tools/almath.h
)
set(ALMATH_H
    ${ALMATH_H_WRAPPED}
    almath/tools/almathio.h
)
qi_create_lib(almath ${ALMATH_SRCS} ${ALMATH_H})

# we publish SWIG_MODULE_almathswig_DEPS together with the lib.
# It is useful when extending almathswig.
set(SWIG_MODULE_almathswig_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/almath.i")
foreach(dep IN LISTS ALMATH_H_WRAPPED)
  list(APPEND SWIG_MODULE_almathswig_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/${dep}")
endforeach()
qi_stage_lib(almath ALMATH
  CUSTOM_CODE
  "set(SWIG_MODULE_almathswig_DEPS \"${SWIG_MODULE_almathswig_DEPS}\" CACHE INTERNAL \"\" FORCE)")
qi_install_header(${ALMATH_H} KEEP_RELATIVE_PATHS)

qi_add_optional_package(PYTHON)
if(WITH_PYTHON)
  include(qibuild/swig/python)
  set(SWIG_MODULE_almathswig_EXTRA_DEPS ${ALMATH_H_WRAPPED})
  qi_swig_wrap_python(almathswig almath.i DEPENDS ALMATH)
  # we install the swig interface file since it is needed to wrap extensions
  # of almath, such as almathinternal.
  qi_install_header(almath.i KEEP_RELATIVE_PATHS)
  qi_install_python(almath.py)
endif()

add_subdirectory(test)

##
## Copyright (C) 2009 Aldebaran Robotics
##
project(ALMATH)

cmake_minimum_required(VERSION 2.6.4)
include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")


# Add BEFORE keyword so that we can rebuild almath against a sdk
# without getting conflicts.
include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}")

set( ALMATH_SRCS
    almath/collisions/allinecollisions.cpp
    almath/collisions/convexhull.cpp
    almath/collisions/isinsidepolygon.cpp
    almath/collisions/avoidfootcollision.cpp
    almath/interpolation/alinterpolationutils.cpp
    almath/interpolation/alinterpolationarticular.cpp
    almath/interpolation/alinterpolationbezier.cpp
    almath/interpolation/alinterpolationcartesian.cpp
    almath/interpolation/alinterpolationlinear.cpp
    almath/interpolation/alinterpolationsmooth.cpp
    almath/interpolation/alinterpolationspline.cpp
    almath/interpolation/alinterpolationcubicspline.cpp
    almath/tools/almath.cpp
    almath/tools/almathio.cpp
    almath/tools/alpolynomialsolver.cpp
    almath/tools/altrigonometry.cpp
    almath/tools/mersennetwister.cpp
    almath/tools/aldubinscurve.cpp
    almath/tools/altransformhelpers.cpp
    almath/types/alpose2d.cpp
    almath/types/alrotation2d.cpp
    almath/types/alrotation3d.cpp
    almath/types/alrotation.cpp
    almath/types/alpositionandvelocity.cpp
    almath/types/altransformandvelocity6d.cpp
    almath/types/altransform.cpp
    almath/types/alvelocity3d.cpp
    almath/types/alvelocity6d.cpp
    almath/types/alposition2d.cpp
    almath/types/alposition3d.cpp
    almath/types/alposition6d.cpp
)

set(ALMATH_H
    almath/collisions/allinecollisions.h
    almath/collisions/convexhull.h
    almath/collisions/isinsidepolygon.h
    almath/collisions/avoidfootcollision.h
    almath/interpolation/alinterpolationarticular.h
    almath/interpolation/alinterpolationbezier.h
    almath/interpolation/alinterpolationcartesian.h
    almath/interpolation/alinterpolationcubicspline.h
    almath/interpolation/alinterpolation.h
    almath/interpolation/alinterpolationlinear.h
    almath/interpolation/alinterpolationsmooth.h
    almath/interpolation/alinterpolationspline.h
    almath/interpolation/alinterpolationtypes.h
    almath/interpolation/alinterpolationutils.h
    almath/tools/aldubinscurve.h
    almath/tools/almath.h
    almath/tools/almathio.h
    almath/tools/alpolynomialsolver.h
    almath/tools/alrandom.h
    almath/tools/altransformhelpers.h
    almath/tools/altrigonometry.h
    almath/tools/mersennetwister.h
    almath/types/alaxismask.h
    almath/types/alpose2d.h
    almath/types/alposition2d.h
    almath/types/alposition3d.h
    almath/types/alposition6d.h
    almath/types/alpositionandvelocity.h
    almath/types/alrotation2d.h
    almath/types/alrotation3d.h
    almath/types/alrotation.h
    almath/types/altransformandvelocity6d.h
    almath/types/altransform.h
    almath/types/alvelocity3d.h
    almath/types/alvelocity6d.h
)





create_lib(almath ${ALMATH_SRCS} ${ALMATH_H})
use_lib(almath ALCORE QI)

# make sure folders hierarchy is preserved.
file(GLOB_RECURSE _headers RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)
foreach(_header  ${_headers})
  get_filename_component(_header_path ${_header} PATH)
  install_header(ALMATH SUBFOLDER ${_header_path} "${_header}")
endforeach()
stage_lib(almath ALMATH)

option(ALMATH_PYTHON_BINDING
    "Build almath python binding using Swig."
    ON)


# ALMath python bindings. Optional for now.
if (ALMATH_PYTHON_BINDING STREQUAL "ON")
    use(PYTHON-TOOLS)

    wrap_python(almath almath.i SRCS
        almath/types/alpose2d.cpp
        DEPENDENCIES ALMATH)
    add_python_test(TestALMathPy "${CMAKE_CURRENT_SOURCE_DIR}/test/almath_test.py")
endif()

add_subdirectory(test)

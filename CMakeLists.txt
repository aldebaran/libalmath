##
## Copyright (C) 2009, 2011 Aldebaran Robotics
##
project(ALMATH)

cmake_minimum_required(VERSION 2.6.4)
include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")


# Add BEFORE keyword so that we can rebuild almath against a sdk
# without getting conflicts.
include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}")

set( ALMATH_SRCS
    src/collisions/avoidfootcollision.cpp
    src/tools/almath.cpp
    src/tools/almathio.cpp
    src/tools/aldubinscurve.cpp
    src/tools/altransformhelpers.cpp
    src/tools/altrigonometry.cpp
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
)

set(ALMATH_H
    almath/collisions/avoidfootcollision.h
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
)

create_lib(almath ${ALMATH_SRCS} ${ALMATH_H})

stage_lib(almath ALMATH)

option(ALMATH_PYTHON_BINDING
    "Build almath python binding using Swig."
    ON)
if(DEFINED QI_SDK_DIR)
  qi_install_header(KEEP_RELATIVE_PATHS ${ALMATH_H})
endif()


# ALMath python bindings. Optional for now.
if (ALMATH_PYTHON_BINDING STREQUAL "ON")
    use(PYTHON-TOOLS)

    wrap_python(almath almath.i SRCS
        src/types/alpose2d.cpp
        DEPENDENCIES ALMATH)
    add_python_test(TestALMathPy "${CMAKE_CURRENT_SOURCE_DIR}/test/almath_test.py")
endif()

add_subdirectory(test)

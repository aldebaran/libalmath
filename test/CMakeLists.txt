## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

qi_add_optional_package(rostime)
qi_add_optional_package(qi)

set(almath_tests_srcs
    collisions/avoidfootcollision_test.cpp

    dsp/digitalfilter_test.cpp
    dsp/pidcontroller_test.cpp

    tools/aldubinscurve_test.cpp
    tools/almath_test.cpp
    tools/altransformhelpers_test.cpp

    types/alaxismask_test.cpp
    types/alpose2d_test.cpp
    types/alposition2d_test.cpp
    types/alposition3d_test.cpp
    types/alposition6d_test.cpp
    types/alpositionandvelocity_test.cpp
    types/alrotation3d_test.cpp
    types/alrotation_test.cpp
    types/altransformandvelocity6d_test.cpp
    types/altransform_test.cpp
    types/alvelocity3d_test.cpp
    types/alvelocity6d_test.cpp
    types/alquaternion_test.cpp
    types/aldisplacement_test.cpp
    types/occupancymapparams_test.cpp
)

qi_create_gtest(almath_tests ${almath_tests_srcs} DEPENDS GTEST ALMATH TIMEOUT 240)

## geometrics
qi_create_gtest(test_almath_shapes3d
  SRC geometrics/test_shapes3d.cpp
  DEPENDS ALMATH)

qi_create_gtest(test_almath_shapes3d_utils
  SRC geometrics/test_shapes3d_utils.cpp
  DEPENDS ALMATH)

## scenegraph

qi_create_gtest(test_almath_almatheigen
  SRC scenegraph/test_almatheigen.cpp
  DEPENDS ALMATH)

qi_create_gtest(test_almath_bodymass
  SRC scenegraph/test_bodymass_solidworks.cpp
  DEPENDS EIGEN3 BOOST)

qi_create_gtest(test_almath_gltf
  SRC scenegraph/test_gltf.cpp
  DEPENDS ALMATH)

if(WITH_ROSTIME AND WITH_QI)
  qi_create_gtest(test_almath_qirostime
    SRC scenegraph/test_qirostime.cpp
    DEPENDS ALMATH QI ROSTIME)
endif()

qi_create_gtest(test_almath_rigidbodysystembuilder
  SRC scenegraph/test_rigidbodysystembuilder.cpp
  DEPENDS ALMATH)

qi_create_gtest(test_almath_qianim
  SRC scenegraph/test_qianim.cpp
  ARGUMENTS "${CMAKE_CURRENT_SOURCE_DIR}/scenegraph"
  DEPENDS ALMATH BOOST BOOST_FILESYSTEM)

qi_create_gtest(test_almath_urdf
  SRC scenegraph/test_urdf.cpp
  DEPENDS ALMATH BOOST_FILESYSTEM BOOST_REGEX)

qi_create_gtest(test_almath_urdfeigen
  SRC scenegraph/test_urdfeigen.cpp
  DEPENDS ALMATH)

qi_create_gtest(test_almath_visualization3d
  SRC scenegraph/test_visualization3d.cpp
  DEPENDS ALMATH BOOST_FILESYSTEM)

if(ALMATH_WITH_QIGEOMETRY)
  qi_create_gtest(test_almathqigeometry
    SRC scenegraph/test_almathqigeometry.cpp
    DEPENDS almath qigeometry)
endif()

if (WITH_PYTHON AND NOT WIN32)
  find_program(_PYTHON_EXECUTABLE python REQUIRED)
  qi_add_test(test_almath_swig ${_PYTHON_EXECUTABLE}
    ARGUMENTS ${CMAKE_CURRENT_SOURCE_DIR}/test_almathswig.py
    ENVIRONMENT PYTHONPATH=${QI_SDK_DIR}/lib:${QI_SDK_DIR}/lib/python2.7/site-packages)
endif()

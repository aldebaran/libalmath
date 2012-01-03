## Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING file.

set(almath_tests_srcs
    collisions/avoidfootcollision_test.cpp

    tools/aldubinscurve_test.cpp
    tools/almath_test.cpp
    tools/altransformhelpers_test.cpp

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
)

qi_create_gtest(almath_tests ${almath_tests_srcs} DEPENDS GTEST ALMATH)


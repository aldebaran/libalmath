## Author: Cyrille Collette <ccollette@aldebaran-robotics.com>
##
## Copyright (C) 2010 Aldebaran Robotics
##

set(almath_tests_srcs
    almathtestutils.h
    almathtestutils.cpp

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

)

create_gtest(almath_tests SRC ${almath_tests_srcs} DEPENDENCIES GTEST ALMATH ALLOG)


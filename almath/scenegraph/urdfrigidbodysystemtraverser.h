/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#ifndef LIB_ALMATH_SCENEGRAPH_URDFRIGIDBODYSYSTEMTRAVERSER_H
#define LIB_ALMATH_SCENEGRAPH_URDFRIGIDBODYSYSTEMTRAVERSER_H

#include <almath/scenegraph/urdf.h>
#include <almath/scenegraph/rigidbodysystembuilder.h>
#include <iosfwd>

namespace AL {

// Walk an URDF kinematic tree calling the provided builder.
//
// notes:
// * will add an implicit free floating root joint
// * will modify pt.
// * will stop exploring a branch when a massless non-fixed link is reached.
void buildRigidBodySystemFromUrdf(
    RigidBodySystemBuilder::Interface<double> &builder, urdf::ptree &pt,
    bool remove_root_joint = false, bool make_continuous_joints_fixed = false);

void buildRigidBodySystemFromUrdf(
    RigidBodySystemBuilder::Interface<double> &builder, std::istream &is,
    bool remove_root_joint = false, bool make_continuous_joints_fixed = false);
}
#endif

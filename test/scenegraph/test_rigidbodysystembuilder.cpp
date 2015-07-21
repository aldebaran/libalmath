/*
 *  Copyright 2015 Aldebaran. All rights reserved.
 *
 */

#include <almath/scenegraph/rigidbodysystembuilder.h>
#include <gtest/gtest.h>
#include "myrigidbodysystembuilder.h"

using namespace AL;
using namespace AL::RigidBodySystemBuilder;

typedef MyBuilder::Pose Pose;
typedef MyBuilder::Vector3 Vector3;
typedef MyBuilder::Matrix3 Matrix3;
typedef MyBuilder::BodyMass BodyMass;

Pose H_parent_joint = Pose::Identity();
JointType joint_type = JointType::FreeFlyer;
BodyMass body_mass = {1, Vector3::Zero(), Matrix3::Identity()};

TEST_F(RigidBodySystemBuilderTest, add) {
  f.add("", "a", H_parent_joint, joint_type, body_mass, "_a");
  f.add("a", "b", H_parent_joint, joint_type, body_mass, "ab");
  f.add("", "c", H_parent_joint, "_c");
  f.add("c", "d", H_parent_joint, "cd");
  f.add("b", "e", H_parent_joint, "be");

  // check non-name arguments
  // ...for link
  EXPECT_TRUE(H_parent_joint.isApprox(b.links[0].pose_parent_new));
  EXPECT_EQ(joint_type, b.links[0].joint_type);
  EXPECT_EQ(body_mass.mass, b.links[0].body_mass.mass);
  EXPECT_EQ(body_mass.center_of_mass, b.links[0].body_mass.center_of_mass);

  // ...for static frame
  EXPECT_TRUE(H_parent_joint.isApprox(b.staticframes[0].pose_parent_new));

  // check names
  EXPECT_EQ(f.config().galilean_frame, b.links[0].parent_body);
  EXPECT_EQ("a", b.links[0].new_body);
  EXPECT_EQ("_a", b.links[0].new_joint);

  EXPECT_EQ("a", b.links[1].parent_body);
  EXPECT_EQ("b", b.links[1].new_body);
  EXPECT_EQ("ab", b.links[1].new_joint);

  EXPECT_EQ(f.config().galilean_frame, b.staticframes[0].parent_frame);
  EXPECT_EQ("c", b.staticframes[0].new_static_frame);
  EXPECT_EQ("_c", b.staticframes[0].new_static_transform);

  EXPECT_EQ("c", b.staticframes[1].parent_frame);
  EXPECT_EQ("d", b.staticframes[1].new_static_frame);
  EXPECT_EQ("cd", b.staticframes[1].new_static_transform);

  EXPECT_EQ("b", b.staticframes[2].parent_frame);
  EXPECT_EQ("e", b.staticframes[2].new_static_frame);
  EXPECT_EQ("be", b.staticframes[2].new_static_transform);
}

TEST_F(RigidBodySystemBuilderTest, add_default_names) {
  f.add("", "a", H_parent_joint, joint_type, body_mass);
  f.add("", "b", H_parent_joint);
  EXPECT_EQ("a_joint", b.links[0].new_joint);
  EXPECT_EQ("b_joint", b.staticframes[0].new_static_transform);
}

TEST_F(RigidBodySystemBuilderTest, no_parent) {
  EXPECT_ANY_THROW(f.add("a", "b", H_parent_joint, joint_type, body_mass));
  EXPECT_ANY_THROW(f.add("a", "b", H_parent_joint));
  f.add("", "a", H_parent_joint);
  EXPECT_ANY_THROW(f.add("a", "b", H_parent_joint, joint_type, body_mass));
}

TEST_F(RigidBodySystemBuilderTest, duplicate_child) {
  EXPECT_ANY_THROW(f.add("", "", H_parent_joint, joint_type, body_mass));
  EXPECT_ANY_THROW(f.add("", "", H_parent_joint));

  f.add("", "a", H_parent_joint, joint_type, body_mass);
  EXPECT_ANY_THROW(f.add("", "a", H_parent_joint, joint_type, body_mass));
  EXPECT_ANY_THROW(f.add("", "a", H_parent_joint));
}

TEST(RigidBodySystemBuilder, InertiaEraser) {
  MyBuilder b;
  InertiaEraser<double> f(b);
  f.add("", "a", H_parent_joint, joint_type,
        BodyMass{0, Vector3::Zero(), Matrix3::Identity()});
  EXPECT_TRUE(b.links[0].body_mass.rotational_inertia.isZero());
}

TEST(RigidBodySystemBuilder, Adapter) {
  // funny pipeline of builders
  MyBuilder b;  // Interface<double>
  Adapter<float, double> p1(b);
  InertiaEraser<float> p2(p1);
  Adapter<double, float> f(p2);
  f.add("", "a", H_parent_joint, joint_type,
        BodyMass{0, Vector3::Zero(), Matrix3::Identity()});
  EXPECT_TRUE(b.links[0].body_mass.rotational_inertia.isZero());
}

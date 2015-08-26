#include <gtest/gtest.h>
#include <almath/scenegraph/qirosmsg.h>

using namespace AL;

TEST(QiRosMsg, toValidRosTime) {
  using qiTime = qi::SystemClock::time_point;
  using AL::Math::toValidRosTime;
  EXPECT_EQ(ros::TIME_MAX, toValidRosTime(qiTime::max()));
  EXPECT_EQ(ros::TIME_MIN, toValidRosTime(qiTime::min()));
  EXPECT_EQ(ros::TIME_MIN, toValidRosTime(qiTime()));
  EXPECT_EQ(ros::Time(1, 1),
            toValidRosTime(qiTime(qi::Seconds(1) + qi::NanoSeconds(1))));
  EXPECT_EQ(ros::TIME_MIN, toValidRosTime(qiTime(qi::Seconds(-1))));
  EXPECT_EQ(ros::Time(1), toValidRosTime(qiTime(qi::Seconds(1))));
}

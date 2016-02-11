#include <gtest/gtest.h>
#include <almath/scenegraph/qirosmsg.h>

using namespace AL;

TEST(QiRosMsg, toValidRosTime) {
  using AL::Math::toValidRosTime;
  using qiTime = qi::Clock::time_point;
  using qiSystemTime = qi::SystemClock::time_point;
  EXPECT_EQ(ros::TIME_MAX, toValidRosTime(qi::Duration::max()));
  EXPECT_EQ(ros::TIME_MAX, toValidRosTime(qiTime::max().time_since_epoch()));
  EXPECT_EQ(ros::TIME_MAX,
            toValidRosTime(qiSystemTime::max().time_since_epoch()));

  EXPECT_EQ(ros::TIME_MIN, toValidRosTime(qi::Duration::min()));
  EXPECT_EQ(ros::TIME_MIN, toValidRosTime(qiTime::min().time_since_epoch()));
  EXPECT_EQ(ros::TIME_MIN,
            toValidRosTime(qiSystemTime::min().time_since_epoch()));

  EXPECT_EQ(ros::TIME_MIN, toValidRosTime(qi::Duration(0)));
  EXPECT_EQ(ros::TIME_MIN, toValidRosTime(qi::Seconds(-1)));

  EXPECT_EQ(ros::Time(1), toValidRosTime(qi::Seconds(1)));
  EXPECT_EQ(ros::Time(1, 1),
            toValidRosTime(qi::Seconds(1) + qi::NanoSeconds(1)));
  EXPECT_EQ(ros::Time(0, 999999999),
            toValidRosTime(qi::Seconds(1) - qi::NanoSeconds(1)));
}

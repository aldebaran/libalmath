#include <gtest/gtest.h>
#include <almath/scenegraph/qirostime.h>

using namespace AL;

TEST(QiRosTime, toValidRosTime) {
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

TEST(QiRosTime, toQiDuration) {
  using AL::Math::toValidRosTime;
  using AL::Math::toQiDuration;
  EXPECT_EQ(qi::Duration(1), toQiDuration(ros::Time(0, 1)));
  EXPECT_EQ(qi::Seconds(1), toQiDuration(ros::Time(1)));
  EXPECT_EQ(qi::Seconds(1) + qi::NanoSeconds(1), toQiDuration(ros::Time(1, 1)));
  EXPECT_EQ(ros::TIME_MAX, toValidRosTime(toQiDuration(ros::TIME_MAX)));
  EXPECT_EQ(ros::TIME_MIN, toValidRosTime(toQiDuration(ros::TIME_MIN)));
}

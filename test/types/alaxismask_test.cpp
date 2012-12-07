#include <stdexcept>
#include <gtest/gtest.h>
#include <almath/types/alaxismask.h>

TEST(ALAxisMaskTest, isAxisMask)
{
  for (int i=1; i<64; ++i)
  {
    EXPECT_TRUE(AL::Math::isAxisMask(i));
  }

  EXPECT_FALSE(AL::Math::isAxisMask(-10));
  EXPECT_FALSE(AL::Math::isAxisMask(-1));
  EXPECT_FALSE(AL::Math::isAxisMask(0));
  EXPECT_FALSE(AL::Math::isAxisMask(64));
  EXPECT_FALSE(AL::Math::isAxisMask(100));
}

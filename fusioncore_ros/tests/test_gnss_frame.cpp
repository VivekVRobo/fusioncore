#include <gtest/gtest.h>

#include "fusioncore_ros/gnss_frame.hpp"

TEST(GnssFrameResolution, ConfiguredFrameWins)
{
  EXPECT_EQ(fusioncore_ros::resolve_gnss_frame("gps_override", "gps"), "gps_override");
}

TEST(GnssFrameResolution, MessageFrameUsedWhenNoOverride)
{
  EXPECT_EQ(fusioncore_ros::resolve_gnss_frame("", "gps"), "gps");
}

TEST(GnssFrameResolution, FallsBackToGnssLink)
{
  EXPECT_EQ(fusioncore_ros::resolve_gnss_frame("", ""), "gnss_link");
}

TEST(GnssFrameResolution, ExplicitBaseFrameIsPreserved)
{
  EXPECT_EQ(fusioncore_ros::resolve_gnss_frame("base_link", "gps"), "base_link");
}

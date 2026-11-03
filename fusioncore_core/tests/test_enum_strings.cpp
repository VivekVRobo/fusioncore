#include <array>
#include <utility>

#include <gtest/gtest.h>

#include "fusioncore/fusioncore.hpp"

namespace {

template <typename Enum, std::size_t N>
void expect_names(const std::array<std::pair<Enum, const char*>, N>& values)
{
  for (const auto& entry : values) {
    SCOPED_TRACE(entry.second);
    const char* actual = fusioncore::to_string(entry.first);
    ASSERT_NE(actual, nullptr);
    EXPECT_STREQ(entry.second, actual);
    EXPECT_NE(actual[0], '\0');
    EXPECT_STRNE("UNKNOWN", actual);
  }
}

}  // namespace

TEST(EnumStringsTest, EveryPublicEnumValueHasAStableName)
{
  using namespace fusioncore;

  expect_names(std::array<std::pair<sensors::GnssFixType, const char*>, 5>{{
    {sensors::GnssFixType::NO_FIX, "NO_FIX"},
    {sensors::GnssFixType::GPS_FIX, "GPS_FIX"},
    {sensors::GnssFixType::DGPS_FIX, "DGPS_FIX"},
    {sensors::GnssFixType::RTK_FLOAT, "RTK_FLOAT"},
    {sensors::GnssFixType::RTK_FIXED, "RTK_FIXED"},
  }});

  expect_names(std::array<std::pair<HeadingSource, const char*>, 6>{{
    {HeadingSource::NONE, "NONE"},
    {HeadingSource::DUAL_ANTENNA, "DUAL_ANTENNA"},
    {HeadingSource::IMU_ORIENTATION, "IMU_ORIENTATION"},
    {HeadingSource::GPS_TRACK, "GPS_TRACK"},
    {HeadingSource::MAGNETOMETER, "MAGNETOMETER"},
    {HeadingSource::GPS_ROTATION, "GPS_ROTATION"},
  }});

  expect_names(std::array<std::pair<GnssRejectionReason, const char*>, 14>{{
    {GnssRejectionReason::NOT_PROCESSED, "NOT_PROCESSED"},
    {GnssRejectionReason::ACCEPTED, "ACCEPTED"},
    {GnssRejectionReason::FIX_TYPE_LOW, "FIX_TYPE_LOW"},
    {GnssRejectionReason::HDOP_HIGH, "HDOP_HIGH"},
    {GnssRejectionReason::VDOP_HIGH, "VDOP_HIGH"},
    {GnssRejectionReason::MIN_SATS, "MIN_SATS"},
    {GnssRejectionReason::CHI2_FAILED, "CHI2_FAILED"},
    {GnssRejectionReason::DELAY_TOO_LARGE, "DELAY_TOO_LARGE"},
    {GnssRejectionReason::IMPLAUSIBLE_JUMP, "IMPLAUSIBLE_JUMP"},
    {GnssRejectionReason::SIGMA_XY_HIGH, "SIGMA_XY_HIGH"},
    {GnssRejectionReason::SIGMA_Z_HIGH, "SIGMA_Z_HIGH"},
    {GnssRejectionReason::CONTINUITY_BREAK, "CONTINUITY_BREAK"},
    {GnssRejectionReason::NOT_FINITE, "NOT_FINITE"},
    {GnssRejectionReason::QUALITY_OTHER, "QUALITY_OTHER"},
  }});

  expect_names(std::array<std::pair<TrackHeadingState, const char*>, 8>{{
    {TrackHeadingState::NOT_ATTEMPTED, "NOT_ATTEMPTED"},
    {TrackHeadingState::FUSED, "FUSED"},
    {TrackHeadingState::STRONGER_SOURCE, "STRONGER_SOURCE"},
    {TrackHeadingState::MOTION_UNSUITABLE, "MOTION_UNSUITABLE"},
    {TrackHeadingState::BASELINE_SHORT, "BASELINE_SHORT"},
    {TrackHeadingState::SIGMA_HIGH, "SIGMA_HIGH"},
    {TrackHeadingState::CHI2_FAILED, "CHI2_FAILED"},
    {TrackHeadingState::WINDOW_HAD_TURN, "WINDOW_HAD_TURN"},
  }});

  expect_names(std::array<std::pair<EncoderRejectionReason, const char*>, 4>{{
    {EncoderRejectionReason::NOT_PROCESSED, "NOT_PROCESSED"},
    {EncoderRejectionReason::ACCEPTED, "ACCEPTED"},
    {EncoderRejectionReason::CHI2_FAILED, "CHI2_FAILED"},
    {EncoderRejectionReason::NOT_FINITE, "NOT_FINITE"},
  }});

  expect_names(std::array<std::pair<ImuRejectionReason, const char*>, 6>{{
    {ImuRejectionReason::NOT_PROCESSED, "NOT_PROCESSED"},
    {ImuRejectionReason::ACCEPTED, "ACCEPTED"},
    {ImuRejectionReason::CHI2_RATE, "CHI2_RATE"},
    {ImuRejectionReason::CHI2_ROLL_PITCH, "CHI2_ROLL_PITCH"},
    {ImuRejectionReason::CHI2_ORIENTATION, "CHI2_ORIENTATION"},
    {ImuRejectionReason::NOT_FINITE, "NOT_FINITE"},
  }});

  expect_names(std::array<std::pair<VslamRejectionReason, const char*>, 4>{{
    {VslamRejectionReason::NOT_PROCESSED, "NOT_PROCESSED"},
    {VslamRejectionReason::ACCEPTED, "ACCEPTED"},
    {VslamRejectionReason::CHI2_FAILED, "CHI2_FAILED"},
    {VslamRejectionReason::NOT_FINITE, "NOT_FINITE"},
  }});

  expect_names(std::array<std::pair<HeadingRejectionReason, const char*>, 4>{{
    {HeadingRejectionReason::NOT_PROCESSED, "NOT_PROCESSED"},
    {HeadingRejectionReason::ACCEPTED, "ACCEPTED"},
    {HeadingRejectionReason::CHI2_FAILED, "CHI2_FAILED"},
    {HeadingRejectionReason::NOT_FINITE, "NOT_FINITE"},
  }});

  expect_names(std::array<std::pair<MagRejectionReason, const char*>, 5>{{
    {MagRejectionReason::NOT_PROCESSED, "NOT_PROCESSED"},
    {MagRejectionReason::ACCEPTED, "ACCEPTED"},
    {MagRejectionReason::CHI2_FAILED, "CHI2_FAILED"},
    {MagRejectionReason::FIELD_MAGNITUDE, "FIELD_MAGNITUDE"},
    {MagRejectionReason::NOT_FINITE, "NOT_FINITE"},
  }});

  expect_names(std::array<std::pair<SensorHealth, const char*>, 3>{{
    {SensorHealth::OK, "OK"},
    {SensorHealth::STALE, "STALE"},
    {SensorHealth::NOT_INIT, "NOT_INIT"},
  }});
}

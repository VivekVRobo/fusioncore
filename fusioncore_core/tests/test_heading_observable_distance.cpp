#include <gtest/gtest.h>
#include "fusioncore/fusioncore.hpp"
#include "fusioncore/motion_model.hpp"

using namespace fusioncore;

// heading_observable_distance decides when heading_validated flips, and it was
// hardcoded at 5.0 with no way to reach it from a config. Setting
// gnss.track_heading_min_dist looked like it should control this, but that gates
// whether track heading is FUSED, which is a different thing. A rover configured
// for 15 m validated at 5.04 m on 2026-09-05 carrying 48.6 degrees of heading
// uncertainty.
//
// Track heading is the bearing between two fixes, so its error is roughly
// GPS sigma over distance travelled. Whether 5 m is enough depends entirely on
// the receiver, which is exactly why it has to be settable.
namespace {

// There are TWO independent routes to heading_validated from GPS track, and
// missing that is what made the first version of this test wrong:
//
//   fusioncore.cpp:490   distance_traveled_ >= heading_observable_distance
//   fusioncore.cpp:1168  fires whenever track heading actually FUSES, which is
//                        gated by gps_track_heading_min_dist instead
//
// Whichever comes first wins. To test one, the other has to be pushed out of
// reach, otherwise you measure the wrong gate and conclude the config is
// ignored when it is simply being beaten to it.
FusionCoreConfig rover_config(double observable_distance,
                              double track_fuse_dist = 1000.0) {
  FusionCoreConfig cfg;
  cfg.heading_observable_distance = observable_distance;
  cfg.gps_track_heading_min_dist  = track_fuse_dist;
  cfg.imu.gyro_noise_x = cfg.imu.gyro_noise_y = cfg.imu.gyro_noise_z = 0.005;
  cfg.imu.accel_noise_x = cfg.imu.accel_noise_y = cfg.imu.accel_noise_z = 0.1;
  cfg.imu_has_magnetometer = false;
  cfg.encoder.vel_noise_x = cfg.encoder.vel_noise_y = 0.05;
  cfg.encoder.vel_noise_wz = 0.02;
  cfg.gnss.base_noise_xy = 1.0;
  cfg.gnss.base_noise_z  = 1.0;
  cfg.outlier_rejection = false;
  cfg.motion_model = create_motion_model("DifferentialDrive");
  return cfg;
}

sensors::GnssFix fix_at(double x) {
  sensors::GnssFix f;
  f.x = x; f.y = 0.0; f.z = 0.0;
  f.hdop = 1.0; f.vdop = 1.0;
  f.satellites = 12;
  f.fix_type = sensors::GnssFixType::DGPS_FIX;
  return f;
}

// Drive straight east at 1 m/s and report the distance at which heading first
// validated, or -1 if it never did.
double distance_at_validation(double observable_distance, double drive_seconds,
                              double track_fuse_dist = 1000.0) {
  FusionCore fc(rover_config(observable_distance, track_fuse_dist));
  State s0;
  fc.init(s0, 0.0);
  const double dt = 0.01, speed = 1.0, g = 9.80665;

  for (int step = 1; step * dt <= drive_seconds + 1e-9; ++step) {
    const double t = step * dt;
    fc.update_imu(t, 0, 0, 0, 0, 0, g);
    if (step % 2 == 0) {
      fc.update_encoder(t, speed, 0.0, 0.0);
      fc.update_ground_constraint(t);
    }
    if (step % 20 == 0) {
      fc.update_gnss(t, fix_at(speed * t));
      if (fc.is_heading_valid()) return fc.get_status().distance_traveled;
    }
  }
  return -1.0;
}

} // namespace

TEST(HeadingObservableDistance, DefaultValidatesAtFiveMetres) {
  const double d = distance_at_validation(5.0, 30.0);
  ASSERT_GT(d, 0.0) << "heading never validated at all";
  EXPECT_NEAR(d, 5.0, 1.0)
      << "the shipped default should still validate near 5 m, measured " << d;
}

TEST(HeadingObservableDistance, ConfigIsActuallyHonoured) {
  const double d = distance_at_validation(15.0, 40.0);
  ASSERT_GT(d, 0.0) << "heading never validated at all";
  EXPECT_GT(d, 12.0)
      << "asked for 15 m of travel before declaring heading observable but it "
         "validated at " << d << " m. That is the bug this test exists for: the "
         "value was hardcoded and the config was silently ignored.";
}

TEST(HeadingObservableDistance, LongerRequirementDelaysValidation) {
  const double d5  = distance_at_validation(5.0,  40.0);
  const double d15 = distance_at_validation(15.0, 40.0);
  ASSERT_GT(d5, 0.0);
  ASSERT_GT(d15, 0.0);
  EXPECT_GT(d15, d5 * 2.0)
      << "raising the threshold should push validation meaningfully later, "
      << "got " << d5 << " m and " << d15 << " m";
}

// The second route, documented so the interaction is not rediscovered the hard
// way. Fusing a track heading also validates, so a small gps_track_heading_min_dist
// will validate heading early no matter how large heading_observable_distance is.
TEST(HeadingObservableDistance, FusingTrackHeadingAlsoValidates) {
  const double d = distance_at_validation(1000.0, 40.0, 5.0);
  ASSERT_GT(d, 0.0)
      << "heading never validated, but fusing a track heading should validate it "
         "even with heading_observable_distance set out of reach";
  EXPECT_LT(d, 12.0)
      << "expected validation via the fusion path at about the 5 m fuse "
         "threshold, measured " << d;
}

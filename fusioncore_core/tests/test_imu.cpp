#include <gtest/gtest.h>
#include "fusioncore/ukf.hpp"
#include "fusioncore/state.hpp"
#include "fusioncore/sensors/imu.hpp"
#include "fusioncore/fusioncore.hpp"

using namespace fusioncore;
using namespace fusioncore::sensors;

// ─── Test 1: IMU measurement function maps state correctly ───────────────────

TEST(IMUTest, MeasurementFunctionMapsState) {
  StateVector x = StateVector::Zero();

  x[WX] = 0.1;  x[WY] = 0.2;  x[WZ] = 0.3;
  x[AX] = 1.0;  x[AY] = 2.0;  x[AZ] = 9.8;
  x[B_GX] = 0.0; x[B_GY] = 0.0; x[B_GZ] = 0.0;
  x[B_AX] = 0.0; x[B_AY] = 0.0; x[B_AZ] = 0.0;

  ImuMeasurement z = imu_measurement_function(x);

  EXPECT_DOUBLE_EQ(z[0], 0.1);
  EXPECT_DOUBLE_EQ(z[1], 0.2);
  EXPECT_DOUBLE_EQ(z[2], 0.3);
  constexpr double g = 9.80665;  // gravity (cp=1, cr=1 at zero roll/pitch)
  EXPECT_DOUBLE_EQ(z[3], 1.0);   // AX, no pitch → no gravity x-component
  EXPECT_DOUBLE_EQ(z[4], 2.0);   // AY, no roll  → no gravity y-component
  EXPECT_DOUBLE_EQ(z[5], 9.8 + g); // AZ + gravity at zero roll/pitch
}

// ─── Test 2: Bias shifts the expected measurement ────────────────────────────

TEST(IMUTest, BiasShiftsMeasurement) {
  StateVector x = StateVector::Zero();

  x[WX]   = 0.1;
  x[B_GX] = 0.05;

  x[AZ]   = 9.8;
  x[B_AZ] = 0.2;

  ImuMeasurement z = imu_measurement_function(x);

  constexpr double g = 9.80665;
  EXPECT_DOUBLE_EQ(z[0], 0.15);
  EXPECT_DOUBLE_EQ(z[5], 9.8 + 0.2 + g); // AZ + B_AZ + gravity at zero roll/pitch
}

// ─── Test 3: Noise matrix is diagonal and positive ───────────────────────────

TEST(IMUTest, NoiseMatrixIsDiagonalAndPositive) {
  ImuParams params;
  ImuNoiseMatrix R = imu_noise_matrix(params);

  for (int i = 0; i < IMU_DIM; ++i) {
    EXPECT_GT(R(i,i), 0.0);
  }

  for (int i = 0; i < IMU_DIM; ++i) {
    for (int j = 0; j < IMU_DIM; ++j) {
      if (i != j) EXPECT_DOUBLE_EQ(R(i,j), 0.0);
    }
  }
}

// ─── Test 4: UKF fuses IMU: total signal (WZ + B_GZ) matches measurement ────
// Observability note: IMU alone measures WZ + B_GZ, it cannot separate them.
// That separation happens when motion changes (encoder + IMU together).
// What we CAN verify: after fusion, the predicted measurement matches reality.

TEST(IMUTest, UKFUpdateFusesIMUMeasurement) {
  UKFParams ukf_params;
  ukf_params.q_gyro_bias  = 1e-4;
  ukf_params.q_accel_bias = 1e-4;

  UKF ukf(ukf_params);

  State initial;
  initial.x       = StateVector::Zero();
  initial.x[B_GZ] = 0.1;   // initial bias estimate
  initial.P       = StateMatrix::Identity() * 0.1;

  ukf.init(initial);

  // IMU reads 0.5 rad/s on Z
  ImuMeasurement z = ImuMeasurement::Zero();
  z[2] = 0.5;

  ImuParams imu_params;
  ImuNoiseMatrix R = imu_noise_matrix(imu_params);

  for (int i = 0; i < 200; ++i) {
    ukf.predict(0.01);
    ukf.update<IMU_DIM>(z, imu_measurement_function, R);
  }

  // What the filter predicts the IMU should read
  double predicted_reading = ukf.state().x[WZ] + ukf.state().x[B_GZ];

  // The predicted measurement must match the actual measurement
  EXPECT_NEAR(predicted_reading, 0.5, 0.01);
}

// ─── Test 5: Custom noise params change R matrix values ──────────────────────

TEST(IMUTest, CustomNoiseParamsApplied) {
  ImuParams params;
  params.gyro_noise_x  = 0.01;
  params.accel_noise_z = 0.5;

  ImuNoiseMatrix R = imu_noise_matrix(params);

  EXPECT_DOUBLE_EQ(R(0,0), 0.01 * 0.01);
  EXPECT_DOUBLE_EQ(R(5,5), 0.5  * 0.5);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ─── IMU and encoders must be told when they disagree about turn direction ───
//
// A rover ran for months with its gyro yaw rate inverted: the BNO085 in UART-RVC
// mode reports yaw increasing clockwise while REP-103 is counterclockwise
// positive. Both sensors were healthy, the encoders were right, and the filter
// watched them contradict each other every cycle without comment. The
// disagreement was noticed twice and blamed on the wheels both times. Because
// imu.gyro_noise defaults far tighter than encoder.yaw_noise, the filter leans on
// the gyro, which was the sensor that was wrong.
//
// Votes are counted rather than requiring an unbroken stretch: a hand-driven
// rover corrects constantly, and on the 2026-09-06 log the longest interval with
// both sensors above even 0.05 rad/s was 0.9 s, so a continuity rule would never
// fire on real driving. Replaying that log: 6 percent of turning samples disagree
// with the correct sign, 80 percent with the gyro inverted.
namespace {
// Drive a weaving course, feeding the encoder yaw rate and the IMU yaw rate with
// a chosen relative sign.
bool yaw_sign_conflict_after(double imu_sign, int seconds = 40) {
  FusionCoreConfig cfg;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  const double dt = 0.02;
  double t = 0.0;
  for (int i = 0; i < seconds * 50; ++i) {
    t += dt;
    // Alternate left and right every 2 s, well above the 0.08 rad/s gate.
    const double wz = ((i / 100) % 2 == 0) ? 0.30 : -0.30;
    fc.update_imu(t, 0.0, 0.0, imu_sign * wz, 0.0, 0.0, 9.80665);
    fc.update_encoder(t, 0.4, 0.0, wz);
  }
  return fc.get_status().yaw_rate_sign_conflict;
}
}  // namespace

TEST(IMUTest, YawRateSignConflictFlaggedWhenGyroIsInverted) {
  EXPECT_TRUE(yaw_sign_conflict_after(-1.0))
      << "an inverted gyro must be reported, not silently trusted";
}

TEST(IMUTest, YawRateSignConflictQuietWhenTheyAgree) {
  EXPECT_FALSE(yaw_sign_conflict_after(+1.0))
      << "sensors that agree must never raise a frame-convention alarm";
}

TEST(IMUTest, YawRateSignIgnoresSlowDriftThatIsNotATurn) {
  // Below the turning gate the sign carries no information: a straight-driving
  // differential rover fabricates small yaw from wheel scale mismatch, and gyro
  // noise crosses zero freely. Neither may vote.
  FusionCoreConfig cfg;
  FusionCore fc(cfg);
  State s0;
  fc.init(s0, 0.0);
  double t = 0.0;
  for (int i = 0; i < 4000; ++i) {
    t += 0.02;
    fc.update_imu(t, 0.0, 0.0, 0.02, 0.0, 0.0, 9.80665);   // opposite signs,
    fc.update_encoder(t, 0.4, 0.0, -0.02);                  // but far too slow
  }
  EXPECT_FALSE(fc.get_status().yaw_rate_sign_conflict);
  EXPECT_EQ(fc.get_status().yaw_rate_turn_samples, 0)
      << "samples below the turning gate must not be counted at all";
}

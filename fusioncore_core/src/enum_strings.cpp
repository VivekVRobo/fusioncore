#include "fusioncore/fusioncore.hpp"

namespace fusioncore {

const char* to_string(sensors::GnssFixType value)
{
  switch (value) {
    case sensors::GnssFixType::NO_FIX:    return "NO_FIX";
    case sensors::GnssFixType::GPS_FIX:   return "GPS_FIX";
    case sensors::GnssFixType::DGPS_FIX:  return "DGPS_FIX";
    case sensors::GnssFixType::RTK_FLOAT: return "RTK_FLOAT";
    case sensors::GnssFixType::RTK_FIXED: return "RTK_FIXED";
  }
  return "UNKNOWN";
}

const char* to_string(HeadingSource value)
{
  switch (value) {
    case HeadingSource::NONE:            return "NONE";
    case HeadingSource::DUAL_ANTENNA:    return "DUAL_ANTENNA";
    case HeadingSource::IMU_ORIENTATION: return "IMU_ORIENTATION";
    case HeadingSource::GPS_TRACK:       return "GPS_TRACK";
    case HeadingSource::MAGNETOMETER:    return "MAGNETOMETER";
    case HeadingSource::GPS_ROTATION:    return "GPS_ROTATION";
  }
  return "UNKNOWN";
}

const char* to_string(GnssRejectionReason value)
{
  switch (value) {
    case GnssRejectionReason::NOT_PROCESSED:    return "NOT_PROCESSED";
    case GnssRejectionReason::ACCEPTED:         return "ACCEPTED";
    case GnssRejectionReason::FIX_TYPE_LOW:     return "FIX_TYPE_LOW";
    case GnssRejectionReason::HDOP_HIGH:        return "HDOP_HIGH";
    case GnssRejectionReason::VDOP_HIGH:        return "VDOP_HIGH";
    case GnssRejectionReason::MIN_SATS:         return "MIN_SATS";
    case GnssRejectionReason::CHI2_FAILED:      return "CHI2_FAILED";
    case GnssRejectionReason::DELAY_TOO_LARGE:  return "DELAY_TOO_LARGE";
    case GnssRejectionReason::IMPLAUSIBLE_JUMP: return "IMPLAUSIBLE_JUMP";
    case GnssRejectionReason::SIGMA_XY_HIGH:    return "SIGMA_XY_HIGH";
    case GnssRejectionReason::SIGMA_Z_HIGH:     return "SIGMA_Z_HIGH";
    case GnssRejectionReason::CONTINUITY_BREAK: return "CONTINUITY_BREAK";
    case GnssRejectionReason::NOT_FINITE:       return "NOT_FINITE";
    case GnssRejectionReason::QUALITY_OTHER:    return "QUALITY_OTHER";
  }
  return "UNKNOWN";
}

const char* to_string(TrackHeadingState value)
{
  switch (value) {
    case TrackHeadingState::NOT_ATTEMPTED:     return "NOT_ATTEMPTED";
    case TrackHeadingState::FUSED:             return "FUSED";
    case TrackHeadingState::STRONGER_SOURCE:   return "STRONGER_SOURCE";
    case TrackHeadingState::MOTION_UNSUITABLE: return "MOTION_UNSUITABLE";
    case TrackHeadingState::BASELINE_SHORT:    return "BASELINE_SHORT";
    case TrackHeadingState::SIGMA_HIGH:        return "SIGMA_HIGH";
    case TrackHeadingState::CHI2_FAILED:       return "CHI2_FAILED";
    case TrackHeadingState::WINDOW_HAD_TURN:   return "WINDOW_HAD_TURN";
  }
  return "UNKNOWN";
}

const char* to_string(EncoderRejectionReason value)
{
  switch (value) {
    case EncoderRejectionReason::NOT_PROCESSED: return "NOT_PROCESSED";
    case EncoderRejectionReason::ACCEPTED:      return "ACCEPTED";
    case EncoderRejectionReason::CHI2_FAILED:   return "CHI2_FAILED";
    case EncoderRejectionReason::NOT_FINITE:    return "NOT_FINITE";
  }
  return "UNKNOWN";
}

const char* to_string(ImuRejectionReason value)
{
  switch (value) {
    case ImuRejectionReason::NOT_PROCESSED:    return "NOT_PROCESSED";
    case ImuRejectionReason::ACCEPTED:         return "ACCEPTED";
    case ImuRejectionReason::CHI2_RATE:        return "CHI2_RATE";
    case ImuRejectionReason::CHI2_ROLL_PITCH:  return "CHI2_ROLL_PITCH";
    case ImuRejectionReason::CHI2_ORIENTATION: return "CHI2_ORIENTATION";
    case ImuRejectionReason::NOT_FINITE:       return "NOT_FINITE";
  }
  return "UNKNOWN";
}

const char* to_string(VslamRejectionReason value)
{
  switch (value) {
    case VslamRejectionReason::NOT_PROCESSED: return "NOT_PROCESSED";
    case VslamRejectionReason::ACCEPTED:      return "ACCEPTED";
    case VslamRejectionReason::CHI2_FAILED:   return "CHI2_FAILED";
    case VslamRejectionReason::NOT_FINITE:    return "NOT_FINITE";
  }
  return "UNKNOWN";
}

const char* to_string(HeadingRejectionReason value)
{
  switch (value) {
    case HeadingRejectionReason::NOT_PROCESSED: return "NOT_PROCESSED";
    case HeadingRejectionReason::ACCEPTED:      return "ACCEPTED";
    case HeadingRejectionReason::CHI2_FAILED:   return "CHI2_FAILED";
    case HeadingRejectionReason::NOT_FINITE:    return "NOT_FINITE";
  }
  return "UNKNOWN";
}

const char* to_string(MagRejectionReason value)
{
  switch (value) {
    case MagRejectionReason::NOT_PROCESSED:   return "NOT_PROCESSED";
    case MagRejectionReason::ACCEPTED:        return "ACCEPTED";
    case MagRejectionReason::CHI2_FAILED:     return "CHI2_FAILED";
    case MagRejectionReason::FIELD_MAGNITUDE: return "FIELD_MAGNITUDE";
    case MagRejectionReason::NOT_FINITE:      return "NOT_FINITE";
  }
  return "UNKNOWN";
}

const char* to_string(SensorHealth value)
{
  switch (value) {
    case SensorHealth::OK:       return "OK";
    case SensorHealth::STALE:    return "STALE";
    case SensorHealth::NOT_INIT: return "NOT_INIT";
  }
  return "UNKNOWN";
}

}  // namespace fusioncore

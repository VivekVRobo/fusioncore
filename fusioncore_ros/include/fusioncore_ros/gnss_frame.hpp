#pragma once

#include <string>

namespace fusioncore_ros
{

inline std::string resolve_gnss_frame(
  const std::string & configured_frame,
  const std::string & message_frame)
{
  if (!configured_frame.empty()) {
    return configured_frame;
  }
  if (!message_frame.empty()) {
    return message_frame;
  }
  return "gnss_link";
}

}  // namespace fusioncore_ros

// include/planning_pkg/common.hpp
#pragma once

#include <rmw/types.h>
#include <rclcpp/rclcpp.hpp>

namespace planning_pkg::qos
{
inline constexpr rmw_qos_profile_t qos_profile_custom1{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};
} 

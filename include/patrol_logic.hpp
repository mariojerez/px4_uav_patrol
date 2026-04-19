/****************************************************************************
 * Copyright (c) 2026 Mario Jerez
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <algorithm>

namespace patrol_logic
{

// Percent progress through a waypoint traversal defined by [first_wp, last_wp].
// Returns 0.0 before the traversal starts, 100.0 once it has finished, and a
// linear fraction in-between. The range [first_wp, last_wp] is inclusive.
inline float progressPercent(int current_index, int first_wp, int last_wp)
{
  if (first_wp < 0 || last_wp < first_wp) {
    return 0.0f;
  }
  if (current_index < first_wp) {
    return 0.0f;
  }
  if (current_index > last_wp) {
    return 100.0f;
  }
  const int span = last_wp - first_wp;
  if (span <= 0) {
    // Single-waypoint traversal: reaching the waypoint completes the mission.
    return 100.0f;
  }
  const float frac =
    static_cast<float>(current_index - first_wp) / static_cast<float>(span);
  return std::clamp(frac, 0.0f, 1.0f) * 100.0f;
}

// Was the mission index ever inside the waypoint traversal band?
inline bool indexInTraversal(int current_index, int first_wp, int last_wp)
{
  return first_wp >= 0 &&
         last_wp >= first_wp &&
         current_index >= first_wp &&
         current_index <= last_wp;
}

} // namespace patrol_logic

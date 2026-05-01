/****************************************************************************
 * Copyright (c) 2026 Mario Jerez
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include "patrol_logic.hpp"

using patrol_logic::indexInTraversal;
using patrol_logic::progressPercent;

TEST(PatrolLogic, ProgressBeforeTraversalIsZero)
{
  EXPECT_FLOAT_EQ(progressPercent(-1, 3, 10), 0.0f);
  EXPECT_FLOAT_EQ(progressPercent(0, 3, 10), 0.0f);
  EXPECT_FLOAT_EQ(progressPercent(2, 3, 10), 0.0f);
}

TEST(PatrolLogic, ProgressInsideTraversalIsLinear)
{
  // Traversal band [3, 11] spans 8 steps.
  EXPECT_FLOAT_EQ(progressPercent(3, 3, 11), 0.0f);
  EXPECT_FLOAT_EQ(progressPercent(7, 3, 11), 50.0f);
  EXPECT_FLOAT_EQ(progressPercent(11, 3, 11), 100.0f);
}

TEST(PatrolLogic, ProgressPastTraversalIsSaturated)
{
  EXPECT_FLOAT_EQ(progressPercent(12, 3, 11), 100.0f);
  EXPECT_FLOAT_EQ(progressPercent(100, 3, 11), 100.0f);
}

TEST(PatrolLogic, ProgressWithSingleWaypointIsStep)
{
  // Only one waypoint in the mission: either not yet reached (0%) or
  // reached/passed (100%). This mirrors a degenerate but legal mission.
  EXPECT_FLOAT_EQ(progressPercent(0, 4, 4), 0.0f);
  EXPECT_FLOAT_EQ(progressPercent(4, 4, 4), 100.0f);
  EXPECT_FLOAT_EQ(progressPercent(5, 4, 4), 100.0f);
}

TEST(PatrolLogic, ProgressWithInvalidRangeReturnsZero)
{
  EXPECT_FLOAT_EQ(progressPercent(5, -1, -1), 0.0f);
  EXPECT_FLOAT_EQ(progressPercent(5, 8, 3), 0.0f);  // inverted band
}

TEST(PatrolLogic, IndexInTraversalBoundsAreInclusive)
{
  EXPECT_TRUE(indexInTraversal(3, 3, 11));
  EXPECT_TRUE(indexInTraversal(11, 3, 11));
  EXPECT_TRUE(indexInTraversal(5, 3, 11));

  EXPECT_FALSE(indexInTraversal(2, 3, 11));
  EXPECT_FALSE(indexInTraversal(12, 3, 11));
  EXPECT_FALSE(indexInTraversal(5, -1, 11));
  EXPECT_FALSE(indexInTraversal(5, 3, 2));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

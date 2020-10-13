/* @file State.cpp
 * @copyright [2020]
 */

#include <gtest/gtest.h>
#include <State.hpp>

using ackermann::State;

/* @brief check default construction of State */
TEST(State_EmptyConstruction, should_pass) {
  State s;
  EXPECT_EQ(s.left, 0.0);
  EXPECT_EQ(s.right, 0.0);
}

/* @brief check default construction of State */
TEST(State_Construction, should_pass) {
  State s(0.1, 1.0);
  EXPECT_EQ(s.left, 0.1);
  EXPECT_EQ(s.right, 1.0);
}

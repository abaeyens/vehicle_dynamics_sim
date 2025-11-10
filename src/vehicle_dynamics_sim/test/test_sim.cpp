#include <gtest/gtest.h>

#include <vehicle_dynamics_sim/utils.h>

namespace vehicle_dynamics_sim {
TEST(mod_pi, in_range) {
  EXPECT_DOUBLE_EQ(mod_pi(0.5), 0.5);
  EXPECT_DOUBLE_EQ(mod_pi(-0.5), -0.5);
}

TEST(mod_pi, out_range) {
  EXPECT_DOUBLE_EQ(mod_pi(0.5 + 2 * M_PI), 0.5);
  EXPECT_DOUBLE_EQ(mod_pi(-0.5 - 2 * M_PI), -0.5);
}
}  // namespace vehicle_dynamics_sim

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

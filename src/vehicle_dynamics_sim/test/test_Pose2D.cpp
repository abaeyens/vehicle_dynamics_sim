#include <gtest/gtest.h>

#include <cmath>

#include <vehicle_dynamics_sim/Pose2D.hpp>
#include <vehicle_dynamics_sim/utils.hpp>

namespace vehicle_dynamics_sim
{
TEST(Pose2D, multiply_heading_zero)
{
  /// Arrange
  const Pose2D a = Pose2D{1.0, 2.0, 0.0};
  const Pose2D b = Pose2D{3.0, 4.0, 0.0};

  /// Act
  const Pose2D c = a * b;

  /// Assert
  EXPECT_DOUBLE_EQ(c.x(), 4.0);
  EXPECT_DOUBLE_EQ(c.y(), 6.0);
  EXPECT_DOUBLE_EQ(c.heading(), 0.0);
}

TEST(Pose2D, multiply_heading_half_pi)
{
  /// Arrange
  const Pose2D a = Pose2D{1.0, 2.0, M_PI / 2};
  const Pose2D b = Pose2D{3.0, 4.0, 0.0};

  /// Act
  const Pose2D c = a * b;

  /// Assert
  EXPECT_DOUBLE_EQ(c.x(), 1.0 - 4.0);
  EXPECT_DOUBLE_EQ(c.y(), 2.0 + 3.0);
  EXPECT_DOUBLE_EQ(c.heading(), M_PI / 2);
}

TEST(Pose2D, isometry3d)
{
  /// Arrange
  const Pose2D a = Pose2D{1.0, 2.0, M_PI / 2};

  /// Act
  const Pose2D b = Pose2D(a.toIsometry3d());

  /// Assert
  EXPECT_NEAR(a.x(), b.x(), 1e-9);
  EXPECT_NEAR(a.y(), b.y(), 1e-9);
  EXPECT_NEAR(mod_pi(b.heading() - a.heading()), 0.0, 1e-9);
}

class Pose2DInverseTest : public ::testing::TestWithParam<double>
{
};

TEST_P(Pose2DInverseTest, inverse)
{
  const double angle = GetParam();
  /// Arrange
  const Pose2D a = Pose2D{1.0, 2.0, angle};

  /// Act
  const Pose2D b = a.inverse();

  /// Assert
  const Pose2D b_expected = Pose2D(a.toIsometry3d().inverse());
  EXPECT_NEAR(b.x(), b_expected.x(), 1e-9);
  EXPECT_NEAR(b.y(), b_expected.y(), 1e-9);
  EXPECT_NEAR(mod_pi(b_expected.heading() - b.heading()), 0.0, 1e-9);
}
INSTANTIATE_TEST_SUITE_P(AngleValues, Pose2DInverseTest, ::testing::Values(0.0, M_PI / 4, M_PI));
}  // namespace vehicle_dynamics_sim

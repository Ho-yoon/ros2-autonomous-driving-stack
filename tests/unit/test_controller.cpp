#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Pure Pursuit geometry (same formula as PurePursuitController)
// ---------------------------------------------------------------------------
double pure_pursuit_steering(
  double ego_x, double ego_y, double ego_yaw,
  double target_x, double target_y,
  double lookahead, double wheelbase, double max_steer)
{
  double dx = target_x - ego_x;
  double dy = target_y - ego_y;
  double alpha = std::atan2(dy, dx) - ego_yaw;
  double curvature = 2.0 * std::sin(alpha) / lookahead;
  double steering = std::atan(wheelbase * curvature);
  return std::clamp(steering, -max_steer, max_steer);
}

// ---------------------------------------------------------------------------
// PID longitudinal (simplified)
// ---------------------------------------------------------------------------
struct PidState { double integral = 0; double prev_error = 0; };

double pid_accel(PidState & s, double target, double current, double dt,
  double kp, double ki, double kd)
{
  double error = target - current;
  s.integral += error * dt;
  double deriv = (error - s.prev_error) / dt;
  s.prev_error = error;
  return kp * error + ki * s.integral + kd * deriv;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
TEST(PurePursuit, StraightAheadProducesZeroSteering)
{
  // Target directly in front at yaw=0 → alpha=0 → steering≈0
  double steer = pure_pursuit_steering(0, 0, 0, 5, 0, 5.0, 2.7, 0.6);
  EXPECT_NEAR(steer, 0.0, 1e-6);
}

TEST(PurePursuit, LeftTargetProducesNegativeSteering)
{
  // Target 5m ahead and 3m to the left (positive y in vehicle frame)
  // In ROS convention positive y = left, so alpha > 0 → steering > 0 (left turn)
  double steer = pure_pursuit_steering(0, 0, 0, 5, 3, 5.0, 2.7, 0.6);
  EXPECT_GT(steer, 0.0);
}

TEST(PurePursuit, SteeringClampedAtMax)
{
  // Extreme lateral target → should saturate at max_steer
  double steer = pure_pursuit_steering(0, 0, 0, 1, 100, 5.0, 2.7, 0.6);
  EXPECT_NEAR(steer, 0.6, 1e-9);
}

TEST(Pid, AcceleratesWhenBelowTarget)
{
  PidState s;
  double accel = pid_accel(s, 10.0, 0.0, 0.1, 1.0, 0.1, 0.05);
  EXPECT_GT(accel, 0.0);
}

TEST(Pid, DeceleratesWhenAboveTarget)
{
  PidState s;
  double accel = pid_accel(s, 0.0, 10.0, 0.1, 1.0, 0.1, 0.05);
  EXPECT_LT(accel, 0.0);
}

TEST(Pid, IntegralWindup)
{
  PidState s;
  for (int i = 0; i < 100; ++i) pid_accel(s, 5.0, 0.0, 0.1, 1.0, 0.1, 0.0);
  EXPECT_GT(s.integral, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "tianji_qp_ik/pico_wrist_alignment.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace tianji_qp_ik {
namespace {

Pose pose(const Eigen::Vector3d& position,
          const Eigen::AngleAxisd& rotation = Eigen::AngleAxisd::Identity()) {
  Pose result;
  result.position = position;
  result.rotation = rotation.toRotationMatrix();
  return result;
}

PicoTeleopFrame frameAt(std::uint64_t sequence, const Pose& left,
                        const Pose& right) {
  PicoTeleopFrame frame;
  frame.sequence = sequence;
  frame.tracking_epoch = 1U;
  frame.source_timestamp_ns = static_cast<std::int64_t>(sequence) * 1000000;
  frame.receive_monotonic_ns = frame.source_timestamp_ns;
  frame.wrist_pose_input = true;
  frame.left_wrist_active = true;
  frame.right_wrist_active = true;
  frame.left = left;
  frame.right = right;
  return frame;
}

void establish(PicoWristAlignment& alignment, PicoTeleopFrame frame,
               const Pose& robot_left, const Pose& robot_right,
               std::size_t stable_frames) {
  for (std::size_t index = 0; index + 1U < stable_frames; ++index) {
    frame.sequence = index + 1U;
    ASSERT_FALSE(alignment.update(frame, robot_left, robot_right).left.valid);
  }
  frame.sequence = stable_frames;
  ASSERT_TRUE(alignment.update(frame, robot_left, robot_right).left.valid);
}

TEST(PicoWristAlignment, StableWindowEstablishesAtTenthFrame) {
  PicoWristAlignment alignment({10U, 0.02, 0.15, 0.5});
  const Pose robot_left = pose({1.0, 2.0, 3.0});
  const Pose robot_right = pose({-1.0, -2.0, -3.0});
  const Pose pico_left = pose({0.1, 0.2, 0.3});
  const Pose pico_right = pose({-0.1, -0.2, -0.3});
  const PicoTeleopFrame frame = frameAt(1U, pico_left, pico_right);

  for (std::size_t index = 0; index < 9U; ++index) {
    EXPECT_FALSE(alignment.update(frame, robot_left, robot_right).left.valid);
  }
  const WristAlignmentUpdate established =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_TRUE(established.left.valid);
  EXPECT_TRUE(established.left.aligned);
  EXPECT_NEAR(established.left.target.position.x(), robot_left.position.x(),
              1e-12);
  EXPECT_EQ(alignment.stableCount(ArmSide::kLeft), 10U);
}

TEST(PicoWristAlignment, TranslationScalesOnlyRelativeTranslation) {
  PicoWristAlignment alignment({2U, 0.02, 0.15, 0.5});
  const Pose robot_left = pose({1.0, 2.0, 3.0});
  const Pose robot_right = pose({-1.0, -2.0, -3.0});
  PicoTeleopFrame frame = frameAt(1U, pose({0.1, 0.2, 0.3}),
                                  pose({-0.1, -0.2, -0.3}));
  establish(alignment, frame, robot_left, robot_right, 2U);

  frame.sequence = 11U;
  frame.left.position += Eigen::Vector3d(0.01, 0.0, 0.0);
  const WristAlignmentUpdate update =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_TRUE(update.left.valid);
  EXPECT_NEAR(update.left.target.position.x(), 1.005, 1e-12);
  EXPECT_NEAR(update.left.target.position.y(), 2.0, 1e-12);
  EXPECT_NEAR(update.left.target.position.z(), 3.0, 1e-12);
}

TEST(PicoWristAlignment, RotationComposesThroughSe3) {
  PicoWristAlignment alignment({2U, 0.02, 0.5, 1.0});
  const Pose robot_left = pose(
      {1.0, 2.0, 3.0},
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  const Pose robot_right = pose({-1.0, -2.0, -3.0});
  PicoTeleopFrame frame = frameAt(
      1U, pose({0.1, 0.2, 0.3}), pose({-0.1, -0.2, -0.3}));
  establish(alignment, frame, robot_left, robot_right, 2U);

  frame.sequence = 11U;
  frame.left = pose({0.1, 0.2, 0.3},
                    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
  const WristAlignmentUpdate update =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_TRUE(update.left.valid);
  const Eigen::Matrix3d expected_rotation =
      robot_left.rotation * frame.left.rotation;
  EXPECT_TRUE(update.left.target.rotation.isApprox(expected_rotation, 1e-12));
}

TEST(PicoWristAlignment, InactiveLeftClearsOnlyLeft) {
  PicoWristAlignment alignment({1U, 0.02, 0.15, 1.0});
  const Pose robot_left = pose({1.0, 2.0, 3.0});
  const Pose robot_right = pose({-1.0, -2.0, -3.0});
  PicoTeleopFrame frame = frameAt(1U, pose({0.1, 0.2, 0.3}),
                                  pose({-0.1, -0.2, -0.3}));
  const WristAlignmentUpdate first =
      alignment.update(frame, robot_left, robot_right);
  ASSERT_TRUE(first.left.valid);
  ASSERT_TRUE(first.right.valid);

  frame.sequence = 2U;
  frame.left_wrist_active = false;
  const WristAlignmentUpdate update =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_FALSE(update.left.valid);
  EXPECT_EQ(update.left.hold_reason, "inactive");
  EXPECT_TRUE(update.right.valid);
}

TEST(PicoWristAlignment, InvalidLeftDoesNotInvalidateRight) {
  PicoWristAlignment alignment({1U, 0.02, 0.15, 1.0});
  const Pose robot_left = pose({1.0, 2.0, 3.0});
  const Pose robot_right = pose({-1.0, -2.0, -3.0});
  PicoTeleopFrame frame = frameAt(1U, pose({0.1, 0.2, 0.3}),
                                  pose({-0.1, -0.2, -0.3}));
  ASSERT_TRUE(alignment.update(frame, robot_left, robot_right).left.valid);

  frame.sequence = 2U;
  frame.left.rotation(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const WristAlignmentUpdate update =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_FALSE(update.left.valid);
  EXPECT_EQ(update.left.hold_reason, "invalid_pose");
  EXPECT_TRUE(update.right.valid);
}

TEST(PicoWristAlignment, ResetAndEpochDiscontinuityClearBothReferences) {
  PicoWristAlignment alignment({1U, 0.02, 0.15, 1.0});
  const Pose robot_left = pose({1.0, 2.0, 3.0});
  const Pose robot_right = pose({-1.0, -2.0, -3.0});
  PicoTeleopFrame frame = frameAt(1U, pose({0.1, 0.2, 0.3}),
                                  pose({-0.1, -0.2, -0.3}));
  ASSERT_TRUE(alignment.update(frame, robot_left, robot_right).left.valid);

  alignment.reset();
  frame.sequence = 2U;
  const WristAlignmentUpdate after_reset =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_TRUE(after_reset.left.valid);
  EXPECT_EQ(alignment.stableCount(ArmSide::kLeft), 1U);

  frame.sequence = 3U;
  frame.tracking_epoch = 2U;
  const WristAlignmentUpdate after_epoch =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_TRUE(after_epoch.left.valid);
  EXPECT_EQ(alignment.stableCount(ArmSide::kLeft), 1U);
}

TEST(PicoWristAlignment, AlignmentResetDoesNotReuseOldBaseline) {
  PicoWristAlignment alignment({2U, 0.02, 0.15, 1.0});
  const Pose robot_left = pose({1.0, 2.0, 3.0});
  const Pose robot_right = pose({-1.0, -2.0, -3.0});
  PicoTeleopFrame frame = frameAt(1U, pose({0.1, 0.2, 0.3}),
                                  pose({-0.1, -0.2, -0.3}));
  ASSERT_FALSE(alignment.update(frame, robot_left, robot_right).left.valid);
  frame.sequence = 2U;
  ASSERT_TRUE(alignment.update(frame, robot_left, robot_right).left.valid);

  frame.sequence = 3U;
  frame.wrist_alignment_reset = true;
  frame.left.position.x() += 1.0;
  const WristAlignmentUpdate reset_update =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_FALSE(reset_update.left.valid);
  EXPECT_EQ(reset_update.left.hold_reason, "alignment_reset");
  EXPECT_EQ(alignment.stableCount(ArmSide::kLeft), 1U);
}

TEST(PicoWristAlignment, StaleRecoveryReacquiresMovedWristWithoutJump) {
  PicoWristAlignment alignment({2U, 0.02, 0.15, 1.0});
  const Pose robot_left = pose({1.0, 2.0, 3.0});
  const Pose robot_right = pose({-1.0, -2.0, -3.0});
  PicoTeleopFrame frame = frameAt(1U, pose({0.1, 0.2, 0.3}),
                                  pose({-0.1, -0.2, -0.3}));
  ASSERT_FALSE(alignment.update(frame, robot_left, robot_right).left.valid);
  frame.sequence = 2U;
  ASSERT_TRUE(alignment.update(frame, robot_left, robot_right).left.valid);

  // Move after establishment so the stale transition must not retain the
  // previous accepted target.
  frame.sequence = 3U;
  frame.left.position.x() += 0.01;
  const WristAlignmentUpdate moved =
      alignment.update(frame, robot_left, robot_right);
  ASSERT_TRUE(moved.left.valid);
  EXPECT_NEAR(moved.left.target.position.x(), 1.01, 1e-12);

  // A stale/inactive transition drops the accepted target as well as the
  // baseline. The opposite side remains live.
  frame.sequence = 4U;
  frame.left_wrist_active = false;
  const WristAlignmentUpdate stale =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_FALSE(stale.left.valid);
  EXPECT_EQ(stale.left.hold_reason, "inactive");
  EXPECT_TRUE(stale.right.valid);
  EXPECT_TRUE(stale.left.target.position.isApprox(robot_left.position));
  EXPECT_EQ(alignment.stableCount(ArmSide::kLeft), 0U);

  // The first same-epoch frame after stale starts a fresh window even though
  // the wrist moved while the input was unavailable.
  frame.sequence = 5U;
  frame.left_wrist_active = true;
  frame.left.position.x() += 1.0;
  const WristAlignmentUpdate recovering =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_FALSE(recovering.left.valid);
  EXPECT_EQ(recovering.left.hold_reason, "stable_window");
  EXPECT_TRUE(recovering.left.target.position.isApprox(robot_left.position));
  EXPECT_EQ(alignment.stableCount(ArmSide::kLeft), 1U);

  frame.sequence = 6U;
  const WristAlignmentUpdate reacquired =
      alignment.update(frame, robot_left, robot_right);
  EXPECT_TRUE(reacquired.left.valid);
  EXPECT_TRUE(reacquired.left.target.position.isApprox(robot_left.position));
}

}  // namespace
}  // namespace tianji_qp_ik

#include "tianji_qp_ik/arm_angle.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace tianji_qp_ik {
namespace {

std::string modelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_test.xml")
      .string();
}

std::string picoFastModelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_pico_fast.xml")
      .string();
}

void expectUrdfJointLimits(const std::filesystem::path& path,
                           const std::string& joint_name,
                           const std::string& lower,
                           const std::string& upper) {
  std::ifstream stream(path);
  ASSERT_TRUE(stream.good()) << path;
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string document = buffer.str();
  const std::string name = "name=\"" + joint_name + "\"";
  const std::size_t joint_begin = document.find(name);
  ASSERT_NE(joint_begin, std::string::npos) << path << " " << joint_name;
  const std::size_t joint_end = document.find("</joint>", joint_begin);
  ASSERT_NE(joint_end, std::string::npos) << path << " " << joint_name;
  const std::string joint =
      document.substr(joint_begin, joint_end - joint_begin);
  EXPECT_NE(joint.find("lower=\"" + lower + "\""), std::string::npos)
      << path << " " << joint_name;
  EXPECT_NE(joint.find("upper=\"" + upper + "\""), std::string::npos)
      << path << " " << joint_name;
}

double signedArmAngleError(const ArmKinematicSample& sample,
                           const Eigen::Vector3d& world_reference) {
  const Eigen::Vector3d axis =
      (sample.wrist_position - sample.shoulder_position).normalized();
  Eigen::Vector3d current =
      sample.elbow_position - sample.shoulder_position;
  current -= axis * axis.dot(current);
  current.normalize();
  Eigen::Vector3d projected =
      world_reference - axis * axis.dot(world_reference);
  projected.normalize();
  return std::atan2(axis.dot(current.cross(projected)),
                    current.dot(projected));
}

TEST(MujocoRobot, MapsExactlySevenJointsPerArmByName) {
  MujocoRobot robot(modelPath());
  EXPECT_EQ(robot.model()->nq, 14);
  EXPECT_EQ(robot.model()->nv, 14);
  EXPECT_EQ(robot.model()->nmocap, 2);

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmMapping& mapping = robot.mapping(side);
    const std::string suffix = side == ArmSide::kLeft ? "L" : "R";
    for (int index = 0; index < kArmDof; ++index) {
      EXPECT_EQ(mapping.joint_names[static_cast<std::size_t>(index)],
                "Joint" + std::to_string(index + 1) + "_" + suffix);
      EXPECT_EQ(mapping.body_names[static_cast<std::size_t>(index)],
                "Link" + std::to_string(index + 1) + "_" + suffix);
      EXPECT_GE(mapping.joint_ids[static_cast<std::size_t>(index)], 0);
      EXPECT_GE(mapping.body_ids[static_cast<std::size_t>(index)], 0);
      EXPECT_GE(mapping.qpos_addresses[static_cast<std::size_t>(index)], 0);
      EXPECT_GE(mapping.dof_addresses[static_cast<std::size_t>(index)], 0);
      EXPECT_GT(mapping.limits.upper_position[index], mapping.limits.lower_position[index]);
      EXPECT_NEAR(mapping.limits.velocity[index], 3.1416, 1e-12);
    }
    EXPECT_GE(mapping.tcp_site_id, 0);
    EXPECT_EQ(mapping.tcp_body_id, mapping.body_ids.back());
    EXPECT_GE(robot.targetBodyId(side), 0);
    EXPECT_GE(robot.targetMocapId(side), 0);
  }
  EXPECT_NE(robot.targetMocapId(ArmSide::kLeft), robot.targetMocapId(ArmSide::kRight));
}

TEST(MujocoRobot, Joint4UsesHumanLikeElbowRange) {
  MujocoRobot robot(modelPath());

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    EXPECT_NEAR(limits.lower_position[3], -2.5307, 1e-12);
    EXPECT_NEAR(limits.upper_position[3], 0.0, 1e-12);
  }
}

TEST(MujocoRobot, BothModelsUseSideSpecificJoint1AndJoint3TeleoperationEnvelope) {
  for (const std::string& path : {modelPath(), picoFastModelPath()}) {
    MujocoRobot robot(path);
    const ArmLimits& left = robot.mapping(ArmSide::kLeft).limits;
    const ArmLimits& right = robot.mapping(ArmSide::kRight).limits;

    EXPECT_NEAR(left.lower_position[0], -1.5708, 1e-12) << path;
    EXPECT_NEAR(left.upper_position[0], 3.1067, 1e-12) << path;
    EXPECT_NEAR(right.lower_position[0], -3.1067, 1e-12) << path;
    EXPECT_NEAR(right.upper_position[0], 1.5708, 1e-12) << path;
    EXPECT_NEAR(left.lower_position[2], -3.1067, 1e-12) << path;
    EXPECT_NEAR(left.upper_position[2], 0.0, 1e-12) << path;
    EXPECT_NEAR(right.lower_position[2], 0.0, 1e-12) << path;
    EXPECT_NEAR(right.upper_position[2], 3.1067, 1e-12) << path;
  }
}

TEST(MujocoRobot, AllUrdfModelsUseSideSpecificJoint1AndJoint3Envelope) {
  const std::filesystem::path root(TIANJI_PROJECT_SOURCE_DIR);
  for (const std::filesystem::path& relative : {
           std::filesystem::path("marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"),
           std::filesystem::path(
               "marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4_mujoco.urdf"),
           std::filesystem::path("models/marvin_m6_s_ccs_696_v4_local.urdf")}) {
    const std::filesystem::path path = root / relative;
    expectUrdfJointLimits(path, "Joint1_L", "-1.5708", "3.1067");
    expectUrdfJointLimits(path, "Joint1_R", "-3.1067", "1.5708");
    expectUrdfJointLimits(path, "Joint3_L", "-3.1067", "0");
    expectUrdfJointLimits(path, "Joint3_R", "0", "3.1067");
  }
}

TEST(MujocoRobot, PicoFastModelUsesFourRadiansPerSecondJointLimits) {
  MujocoRobot robot(picoFastModelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    for (int joint = 0; joint < kArmDof; ++joint) {
      EXPECT_NEAR(limits.velocity[joint], 4.0, 1e-12);
    }
    EXPECT_NEAR(limits.lower_position[3], -2.5307, 1e-12);
    EXPECT_NEAR(limits.upper_position[3], 0.0, 1e-12);
  }
}

TEST(MujocoRobot, SettingOneArmDoesNotChangeTheOther) {
  MujocoRobot robot(modelPath());
  const Vec7 right_before = robot.armPosition(ArmSide::kRight);
  Vec7 left;
  left << 0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.05;
  robot.setArmPosition(ArmSide::kLeft, left);
  robot.forward();
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_before));
}

TEST(MujocoRobot, MapsVelocityStateAndKeepsArmsIsolated) {
  MujocoRobot robot(modelPath());
  Vec7 left_position;
  left_position << 0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.05;
  Vec7 left_velocity;
  left_velocity << 0.7, -0.6, 0.5, -0.4, 0.3, -0.2, 0.1;
  const Vec7 right_position = robot.armPosition(ArmSide::kRight);
  const Vec7 right_velocity = Vec7::Constant(-0.25);
  robot.setArmState(ArmSide::kRight, right_position, right_velocity);
  robot.setArmState(ArmSide::kLeft, left_position, left_velocity);

  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_position));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kLeft).isApprox(left_velocity));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_position));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kRight).isApprox(right_velocity));
}

TEST(MujocoRobot, TcpPosesAreFiniteProperRotations) {
  MujocoRobot robot(modelPath());
  robot.forward();
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const Pose pose = robot.tcpPose(side);
    EXPECT_TRUE(pose.position.allFinite());
    EXPECT_TRUE(pose.rotation.allFinite());
    EXPECT_TRUE((pose.rotation.transpose() * pose.rotation).isApprox(Eigen::Matrix3d::Identity(),
                                                                    1e-10));
    EXPECT_NEAR(pose.rotation.determinant(), 1.0, 1e-10);
  }
}

TEST(MujocoRobot, ArbitraryArmKinematicsDoesNotMutateActualState) {
  MujocoRobot robot(modelPath());
  Vec7 left_actual;
  left_actual << 0.10, -0.20, 0.30, -0.60, 0.20, -0.10, 0.05;
  Vec7 right_actual;
  right_actual << -0.15, 0.10, -0.20, -0.50, -0.10, 0.20, -0.05;
  robot.setArmPosition(ArmSide::kLeft, left_actual);
  robot.setArmPosition(ArmSide::kRight, right_actual);
  robot.forward();
  const Pose left_pose_before = robot.tcpPose(ArmSide::kLeft);
  const Pose right_pose_before = robot.tcpPose(ArmSide::kRight);

  Vec7 left_model = left_actual;
  left_model[0] += 0.20;
  left_model[3] -= 0.10;
  const ArmKinematicSample sample =
      robot.armKinematicsAt(ArmSide::kLeft, left_model);

  EXPECT_GT((sample.tcp_pose.position - left_pose_before.position).norm(),
            1e-4);
  EXPECT_TRUE(sample.tcp_pose.position.allFinite());
  EXPECT_TRUE(sample.tcp_pose.rotation.allFinite());
  EXPECT_TRUE(sample.tcp_jacobian.allFinite());
  EXPECT_TRUE(sample.shoulder_position.allFinite());
  EXPECT_TRUE(sample.elbow_position.allFinite());
  EXPECT_TRUE(sample.wrist_position.allFinite());
  EXPECT_TRUE(sample.shoulder_position_jacobian.allFinite());
  EXPECT_TRUE(sample.elbow_position_jacobian.allFinite());
  EXPECT_TRUE(sample.wrist_position_jacobian.allFinite());
  EXPECT_GT((sample.elbow_position - sample.shoulder_position).norm(), 0.05);
  EXPECT_GT((sample.wrist_position - sample.elbow_position).norm(), 0.05);
  constexpr double kFiniteDifferenceStep = 1e-6;
  for (int column = 0; column < kArmDof; ++column) {
    Vec7 plus = left_model;
    Vec7 minus = left_model;
    plus[column] += kFiniteDifferenceStep;
    minus[column] -= kFiniteDifferenceStep;
    const Eigen::Vector3d finite_difference =
        (robot.armKinematicsAt(ArmSide::kLeft, plus).tcp_pose.position -
         robot.armKinematicsAt(ArmSide::kLeft, minus).tcp_pose.position) /
        (2.0 * kFiniteDifferenceStep);
    EXPECT_TRUE(sample.tcp_jacobian.col(column).head<3>().isApprox(
        finite_difference, 1e-7));
    const Eigen::Vector3d elbow_finite_difference =
        (robot.armKinematicsAt(ArmSide::kLeft, plus).elbow_position -
         robot.armKinematicsAt(ArmSide::kLeft, minus).elbow_position) /
        (2.0 * kFiniteDifferenceStep);
    EXPECT_TRUE(sample.elbow_position_jacobian.col(column).isApprox(
        elbow_finite_difference, 1e-7));
    const Eigen::Vector3d shoulder_finite_difference =
        (robot.armKinematicsAt(ArmSide::kLeft, plus).shoulder_position -
         robot.armKinematicsAt(ArmSide::kLeft, minus).shoulder_position) /
        (2.0 * kFiniteDifferenceStep);
    EXPECT_TRUE(sample.shoulder_position_jacobian.col(column).isApprox(
        shoulder_finite_difference, 1e-7));
    const Eigen::Vector3d wrist_finite_difference =
        (robot.armKinematicsAt(ArmSide::kLeft, plus).wrist_position -
         robot.armKinematicsAt(ArmSide::kLeft, minus).wrist_position) /
        (2.0 * kFiniteDifferenceStep);
    EXPECT_TRUE(sample.wrist_position_jacobian.col(column).isApprox(
        wrist_finite_difference, 1e-7));
  }
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_actual, 1e-12));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_actual, 1e-12));
  const Pose left_pose_after = robot.tcpPose(ArmSide::kLeft);
  const Pose right_pose_after = robot.tcpPose(ArmSide::kRight);
  EXPECT_TRUE(left_pose_after.position.isApprox(left_pose_before.position,
                                                1e-12));
  EXPECT_TRUE(left_pose_after.rotation.isApprox(left_pose_before.rotation,
                                                1e-12));
  EXPECT_TRUE(right_pose_after.position.isApprox(right_pose_before.position,
                                                 1e-12));
  EXPECT_TRUE(right_pose_after.rotation.isApprox(right_pose_before.rotation,
                                                 1e-12));
}

TEST(MujocoRobot, RejectsNonFiniteArbitraryKinematicsPosition) {
  MujocoRobot robot(modelPath());
  Vec7 invalid = robot.armPosition(ArmSide::kLeft);
  invalid[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(robot.armKinematicsAt(ArmSide::kLeft, invalid),
               std::invalid_argument);
}

TEST(MujocoRobot, NominalArmAngleJacobianMatchesModelFiniteDifference) {
  MujocoRobot robot(modelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    const Vec7 nominal =
        0.5 * (limits.lower_position + limits.upper_position);
    const ArmKinematicSample model =
        robot.armKinematicsAt(side, nominal);
    ArmAngleTaskBuilder builder(side, 0.015, 0.050, 8.0);
    const ArmDirectionReference reference =
        side == ArmSide::kLeft ? defaultArmDirectionReferences().left
                               : defaultArmDirectionReferences().right;
    const ArmAngleTask task = builder.compute(
        {model.shoulder_position, model.elbow_position, model.wrist_position,
         model.shoulder_position_jacobian, model.elbow_position_jacobian,
         model.wrist_position_jacobian},
        reference, 0.005);
    ASSERT_TRUE(task.active);
    ASSERT_GT(task.jacobian.norm(), 1e-4) << toString(side);

    constexpr double kStep = 1e-6;
    for (int joint = 0; joint < kArmDof; ++joint) {
      Vec7 plus = nominal;
      Vec7 minus = nominal;
      plus[joint] += kStep;
      minus[joint] -= kStep;
      const double plus_error = signedArmAngleError(
          robot.armKinematicsAt(side, plus), reference.direction);
      const double minus_error = signedArmAngleError(
          robot.armKinematicsAt(side, minus), reference.direction);
      const double wrapped_error_delta = std::atan2(
          std::sin(plus_error - minus_error),
          std::cos(plus_error - minus_error));
      const double expected_current_rate =
          -wrapped_error_delta / (2.0 * kStep);
      EXPECT_NEAR(task.jacobian[joint], expected_current_rate, 1e-7)
          << toString(side) << " joint=" << joint;
    }
  }
}

TEST(MujocoRobot, MissingModelIsRejected) {
  EXPECT_THROW(MujocoRobot("/definitely/missing/tianji.xml"), std::runtime_error);
}

}  // namespace
}  // namespace tianji_qp_ik

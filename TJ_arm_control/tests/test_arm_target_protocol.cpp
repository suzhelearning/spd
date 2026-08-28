#include "tianji_qp_ik/arm_target_protocol.hpp"

#include <gtest/gtest.h>

#include <string>
#include <fstream>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace tianji_qp_ik {
namespace {

ArmTargetFrame validFrame() {
  ArmTargetFrame frame;
  frame.sequence = 17U;
  frame.tracking_epoch = 9U;
  frame.source_timestamp_ns = 1'000'000'000U;
  frame.control_timestamp_ns = 1'000'001'000U;
  frame.valid_mask = kArmTargetLeftValid | kArmTargetRightValid;
  frame.left_hold_reason = ArmTargetHoldReason::kNone;
  frame.right_hold_reason = ArmTargetHoldReason::kNone;
  for (std::size_t index = 0U; index < 7U; ++index) {
    frame.left_q[index] = 0.1 * static_cast<double>(index);
    frame.right_q[index] = -0.2 * static_cast<double>(index);
    frame.left_qdot[index] = 0.3 * static_cast<double>(index);
    frame.right_qdot[index] = -0.4 * static_cast<double>(index);
  }
  return frame;
}

std::uint32_t readLe32(const std::uint8_t* bytes) {
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

void writeLe32(std::uint8_t* bytes, std::uint32_t value) {
  bytes[0] = static_cast<std::uint8_t>(value & 0xffU);
  bytes[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  bytes[2] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  bytes[3] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
}

}  // namespace

TEST(ArmTargetProtocol, EncodesAndDecodes272ByteLittleEndianPacket) {
  const ArmTargetFrame expected = validFrame();
  std::array<std::uint8_t, kArmTargetPacketV2Size> bytes{};
  ASSERT_TRUE(encodeArmTargetPacket(expected, bytes));
  EXPECT_EQ(bytes.size(), 272U);
  EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 4), "SPDA");
  EXPECT_EQ(bytes[4], 2U);
  EXPECT_EQ(bytes[5], 0U);
  EXPECT_EQ(bytes[6], 0x10U);
  EXPECT_EQ(bytes[7], 0x01U);
  EXPECT_EQ(readLe32(bytes.data() + 268U),
            armTargetCrc32(bytes.data(), 268U));

  const ArmTargetDecodeResult decoded =
      decodeArmTargetPacket(bytes.data(), bytes.size());
  ASSERT_EQ(decoded.error, ArmTargetPacketError::kNone);
  ASSERT_TRUE(decoded.frame.has_value());
  EXPECT_EQ(decoded.frame->sequence, expected.sequence);
  EXPECT_EQ(decoded.frame->tracking_epoch, expected.tracking_epoch);
  EXPECT_EQ(decoded.frame->valid_mask, expected.valid_mask);
  EXPECT_EQ(decoded.frame->left_hold_reason, expected.left_hold_reason);
  EXPECT_EQ(decoded.frame->right_hold_reason, expected.right_hold_reason);
  for (std::size_t index = 0U; index < 7U; ++index) {
    EXPECT_DOUBLE_EQ(decoded.frame->left_q[index], expected.left_q[index]);
    EXPECT_DOUBLE_EQ(decoded.frame->right_qdot[index], expected.right_qdot[index]);
  }
}

TEST(ArmTargetProtocol, MatchesSharedPythonFixtureBytes) {
  const std::string fixture_path =
      std::string(TIANJI_PROJECT_SOURCE_DIR) +
      "/../PICO_tracker/src/spd_vr/test/fixtures/arm_target_v2.hex";
  std::ifstream fixture(fixture_path);
  ASSERT_TRUE(fixture.good()) << fixture_path;
  std::string hex;
  fixture >> hex;
  ASSERT_EQ(hex.size(), kArmTargetPacketV2Size * 2U);
  std::array<std::uint8_t, kArmTargetPacketV2Size> expected{};
  for (std::size_t index = 0U; index < expected.size(); ++index) {
    expected[index] = static_cast<std::uint8_t>(
        std::stoul(hex.substr(index * 2U, 2U), nullptr, 16));
  }
  std::array<std::uint8_t, kArmTargetPacketV2Size> actual{};
  ASSERT_TRUE(encodeArmTargetPacket(validFrame(), actual));
  EXPECT_EQ(actual, expected);
  ASSERT_EQ(decodeArmTargetPacket(expected.data(), expected.size()).error,
            ArmTargetPacketError::kNone);
}

TEST(ArmTargetProtocol, RejectsCorruptCrcAndNonFiniteValues) {
  auto bytes = std::array<std::uint8_t, kArmTargetPacketV2Size>{};
  ASSERT_TRUE(encodeArmTargetPacket(validFrame(), bytes));
  bytes[100] ^= 0x01U;
  EXPECT_EQ(decodeArmTargetPacket(bytes.data(), bytes.size()).error,
            ArmTargetPacketError::kCrcMismatch);

  ASSERT_TRUE(encodeArmTargetPacket(validFrame(), bytes));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::memcpy(bytes.data() + 44U, &nan, sizeof(nan));
  writeLe32(bytes.data() + 268U, armTargetCrc32(bytes.data(), 268U));
  EXPECT_EQ(decodeArmTargetPacket(bytes.data(), bytes.size()).error,
            ArmTargetPacketError::kNonFiniteValue);
}

TEST(ArmTargetProtocol, EncodesAndDecodesPartialHoldReason) {
  ArmTargetFrame frame = validFrame();
  frame.valid_mask = kArmTargetRightValid;
  frame.left_hold_reason = ArmTargetHoldReason::kSolverFailure;
  frame.right_hold_reason = ArmTargetHoldReason::kNone;
  std::array<std::uint8_t, kArmTargetPacketV2Size> bytes{};
  ASSERT_TRUE(encodeArmTargetPacket(frame, bytes));
  const auto decoded = decodeArmTargetPacket(bytes.data(), bytes.size());
  ASSERT_EQ(decoded.error, ArmTargetPacketError::kNone);
  ASSERT_TRUE(decoded.frame.has_value());
  EXPECT_EQ(decoded.frame->valid_mask, kArmTargetRightValid);
  EXPECT_EQ(decoded.frame->left_hold_reason, ArmTargetHoldReason::kSolverFailure);
  EXPECT_EQ(decoded.frame->right_hold_reason, ArmTargetHoldReason::kNone);
}

TEST(ArmTargetProtocol, RejectsInvalidMetadataAndMask) {
  ArmTargetFrame frame = validFrame();
  std::array<std::uint8_t, kArmTargetPacketV2Size> bytes{};
  frame.sequence = 0U;
  EXPECT_FALSE(encodeArmTargetPacket(frame, bytes));
  frame = validFrame();
  frame.valid_mask = 0U;
  frame.left_hold_reason = ArmTargetHoldReason::kNone;
  frame.right_hold_reason = ArmTargetHoldReason::kInputStale;
  EXPECT_FALSE(encodeArmTargetPacket(frame, bytes));
}

}  // namespace tianji_qp_ik

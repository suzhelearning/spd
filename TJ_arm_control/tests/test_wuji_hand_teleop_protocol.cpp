#include "tianji_qp_ik/wuji_hand_teleop_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace tianji_qp_ik {
namespace {

WujiHandTeleopFrame sampleFrame() {
  WujiHandTeleopFrame frame;
  frame.sequence = 17U;
  frame.source_timestamp_ns = 123456;
  frame.left_valid = true;
  frame.right_valid = true;
  for (std::size_t index = 0; index < kWujiHandJointDof; ++index) {
    frame.left[index] = -0.5 + 0.01 * static_cast<double>(index);
    frame.right[index] = 0.5 + 0.01 * static_cast<double>(index);
  }
  return frame;
}

TEST(WujiHandTeleopProtocol, RoundTripsBothSidesWithFixedSize) {
  const WujiHandTeleopFrame frame = sampleFrame();
  const std::vector<std::uint8_t> packet = encodeWujiHandTeleopPacket(frame);

  ASSERT_EQ(packet.size(), kWujiHandTeleopPacketSize);
  const WujiHandPacketDecodeResult decoded =
      decodeWujiHandTeleopPacket(packet.data(), packet.size());

  ASSERT_TRUE(decoded.frame.has_value());
  EXPECT_EQ(decoded.error, WujiHandPacketError::kNone);
  EXPECT_EQ(decoded.frame->sequence, frame.sequence);
  EXPECT_EQ(decoded.frame->source_timestamp_ns, frame.source_timestamp_ns);
  EXPECT_TRUE(decoded.frame->left_valid);
  EXPECT_TRUE(decoded.frame->right_valid);
  EXPECT_EQ(decoded.frame->left, frame.left);
  EXPECT_EQ(decoded.frame->right, frame.right);
}

TEST(WujiHandTeleopProtocol, RejectsWrongMagicVersionSizeAndFlags) {
  const std::vector<std::uint8_t> packet =
      encodeWujiHandTeleopPacket(sampleFrame());

  auto expectError = [](std::vector<std::uint8_t> mutated,
                        WujiHandPacketError error) {
    const WujiHandPacketDecodeResult decoded =
        decodeWujiHandTeleopPacket(mutated.data(), mutated.size());
    EXPECT_FALSE(decoded.frame.has_value());
    EXPECT_EQ(decoded.error, error);
  };

  std::vector<std::uint8_t> wrong_magic = packet;
  wrong_magic[0] = static_cast<std::uint8_t>('X');
  expectError(wrong_magic, WujiHandPacketError::kWrongMagic);

  std::vector<std::uint8_t> wrong_version = packet;
  wrong_version[4] = 2U;
  expectError(wrong_version, WujiHandPacketError::kWrongVersion);

  std::vector<std::uint8_t> wrong_size = packet;
  wrong_size[6] = 0U;
  wrong_size[7] = 0U;
  expectError(wrong_size, WujiHandPacketError::kWrongDeclaredSize);

  std::vector<std::uint8_t> wrong_flags = packet;
  wrong_flags[5] = 0x80U;
  expectError(wrong_flags, WujiHandPacketError::kInvalidFlags);
}

TEST(WujiHandTeleopProtocol, RejectsCrcAndNonFiniteValues) {
  std::vector<std::uint8_t> packet =
      encodeWujiHandTeleopPacket(sampleFrame());
  packet.back() ^= 0x01U;
  const WujiHandPacketDecodeResult crc_result =
      decodeWujiHandTeleopPacket(packet.data(), packet.size());
  EXPECT_FALSE(crc_result.frame.has_value());
  EXPECT_EQ(crc_result.error, WujiHandPacketError::kCrcMismatch);

  WujiHandTeleopFrame invalid = sampleFrame();
  invalid.left[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(encodeWujiHandTeleopPacket(invalid), std::invalid_argument);
}

TEST(WujiHandTeleopProtocol, RejectsWrongInputSizeAndNullBytes) {
  const WujiHandPacketDecodeResult null_result =
      decodeWujiHandTeleopPacket(nullptr, kWujiHandTeleopPacketSize);
  EXPECT_FALSE(null_result.frame.has_value());
  EXPECT_EQ(null_result.error, WujiHandPacketError::kWrongSize);

  const std::array<std::uint8_t, 2> short_packet{};
  const WujiHandPacketDecodeResult short_result =
      decodeWujiHandTeleopPacket(short_packet.data(), short_packet.size());
  EXPECT_FALSE(short_result.frame.has_value());
  EXPECT_EQ(short_result.error, WujiHandPacketError::kWrongSize);
}

}  // namespace
}  // namespace tianji_qp_ik

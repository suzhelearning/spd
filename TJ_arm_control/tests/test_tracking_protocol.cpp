#include "tianji_qp_ik/tracking_protocol.hpp"
#include "tianji_qp_ik/arm_target_protocol.hpp"
#include "tianji_qp_ik/zenoh_keys.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>

namespace tianji_qp_ik {
namespace {

static_assert(kTrackingPacketSize == 1540U);

TrackingFrame fixtureFrame() {
  TrackingFrame frame;
  frame.sequence = 101U;
  frame.tracking_epoch = 7U;
  frame.source_timestamp_ns = 1'000'000'000;
  frame.bridge_monotonic_ns = 1'000'000'500;
  frame.flags = kTrackingLeftActive | kTrackingRightActive | kTrackingHeadValid;
  frame.left_scale = 1.25F;
  frame.right_scale = 0.75F;
  frame.head_pose = {1.0F, 2.0F, 3.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  for (std::size_t index = 0U; index < frame.left_hand.size(); ++index) {
    const float joint = static_cast<float>(index);
    frame.left_hand[index] = {
        joint / 8.0F, index == 0U ? 0.0F : -joint / 16.0F,
        joint / 32.0F, 0.0F, 0.0F, 0.0F, 1.0F};
    frame.right_hand[index] = {
        index == 0U ? 0.0F : -joint / 8.0F, joint / 16.0F,
        index == 0U ? 0.0F : -joint / 32.0F, 0.0F, 0.0F, 0.0F, 1.0F};
  }
  return frame;
}

std::array<std::uint8_t, kTrackingPacketSize> encodedFixture() {
  std::array<std::uint8_t, kTrackingPacketSize> bytes{};
  EXPECT_TRUE(encodeTrackingPacket(fixtureFrame(), bytes));
  return bytes;
}

void writeLe32(std::uint8_t* bytes, std::uint32_t value) {
  for (std::size_t index = 0U; index < 4U; ++index) {
    bytes[index] = static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

void writeLe64(std::uint8_t* bytes, std::uint64_t value) {
  for (std::size_t index = 0U; index < 8U; ++index) {
    bytes[index] = static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

void writeLeFloat(std::uint8_t* bytes, float value) {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(value));
  writeLe32(bytes, bits);
}

void refreshCrc(std::array<std::uint8_t, kTrackingPacketSize>& bytes) {
  writeLe32(bytes.data() + 12U,
            armTargetCrc32(bytes.data() + 16U, bytes.size() - 16U));
}

std::array<std::uint8_t, kTrackingPacketSize> readFixture() {
  const std::string fixture_path =
      std::string(TIANJI_PROJECT_SOURCE_DIR) +
      "/../PICO_tracker/src/spd_vr/test/fixtures/tracking_v1.hex";
  std::ifstream fixture(fixture_path);
  EXPECT_TRUE(fixture.good()) << fixture_path;
  std::string hex;
  fixture >> hex;
  EXPECT_EQ(hex.size(), kTrackingPacketSize * 2U);
  std::array<std::uint8_t, kTrackingPacketSize> bytes{};
  for (std::size_t index = 0U; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>(
        std::stoul(hex.substr(index * 2U, 2U), nullptr, 16));
  }
  return bytes;
}

}  // namespace

TEST(TrackingProtocol, MatchesGoldenVectorAndNormalizesQuaternions) {
  const auto encoded = encodedFixture();
  EXPECT_EQ(encoded.size(), 1540U);
  EXPECT_EQ(encoded[0], static_cast<std::uint8_t>('S'));
  EXPECT_EQ(encoded[1], static_cast<std::uint8_t>('V'));
  EXPECT_EQ(encoded[2], static_cast<std::uint8_t>('T'));
  EXPECT_EQ(encoded[3], static_cast<std::uint8_t>('1'));
  EXPECT_EQ(encoded, readFixture());

  const auto decoded = decodeTrackingPacket(encoded.data(), encoded.size());
  ASSERT_EQ(decoded.error, TrackingPacketError::kNone);
  ASSERT_TRUE(decoded.frame.has_value());
  EXPECT_EQ(decoded.frame->sequence, 101U);
  EXPECT_EQ(decoded.frame->tracking_epoch, 7U);
  EXPECT_FLOAT_EQ(decoded.frame->left_hand[1][0], 0.125F);
  EXPECT_FLOAT_EQ(decoded.frame->right_hand[2][2], -0.0625F);
  EXPECT_FLOAT_EQ(decoded.frame->head_pose[6], 1.0F);

  TrackingFrame nearly_unit = fixtureFrame();
  nearly_unit.left_hand[1][6] = 1.0005F;
  std::array<std::uint8_t, kTrackingPacketSize> bytes{};
  ASSERT_TRUE(encodeTrackingPacket(nearly_unit, bytes));
  const auto normalized = decodeTrackingPacket(bytes.data(), bytes.size());
  ASSERT_EQ(normalized.error, TrackingPacketError::kNone);
  ASSERT_TRUE(normalized.frame.has_value());
  EXPECT_NEAR(std::sqrt(
                  std::pow(normalized.frame->left_hand[1][3], 2.0F) +
                  std::pow(normalized.frame->left_hand[1][4], 2.0F) +
                  std::pow(normalized.frame->left_hand[1][5], 2.0F) +
                  std::pow(normalized.frame->left_hand[1][6], 2.0F)),
              1.0F, 1.0e-6F);
}

TEST(TrackingProtocol, ExposesCanonicalZenohKeys) {
  EXPECT_EQ(kTrackingKey, "spd/vr/v1/tracking");
  EXPECT_EQ(kArmTargetsKey, "spd/vr/v1/arm_targets");
  EXPECT_EQ(kControlKey, "spd/vr/v1/control");
  EXPECT_EQ(kBridgeStatusKey, "spd/vr/v1/status/bridge");
  EXPECT_EQ(kIkStatusKey, "spd/vr/v1/status/ik");
  EXPECT_EQ(kViewerStatusKey, "spd/vr/v1/status/viewer");
}

TEST(TrackingProtocol, RejectsStructuralCorruptionFailClosed) {
  auto bytes = encodedFixture();
  bytes[0] = 'X';
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kWrongMagic);
  EXPECT_FALSE(decodeTrackingPacket(bytes.data(), bytes.size()).frame.has_value());

  bytes = encodedFixture();
  bytes[4] = 2U;
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kWrongVersion);

  bytes = encodedFixture();
  bytes[8] = 0U;
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kWrongDeclaredSize);

  bytes = encodedFixture();
  bytes[6] = 0x08U;
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kUnknownFlags);

  bytes = encodedFixture();
  bytes[100] ^= 1U;
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kCrcMismatch);

  bytes = encodedFixture();
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size() - 1U).error,
            TrackingPacketError::kWrongSize);
}

TEST(TrackingProtocol, RejectsInvalidScalarsMetadataAndQuaternions) {
  auto bytes = encodedFixture();
  writeLeFloat(bytes.data() + 84U, std::numeric_limits<float>::quiet_NaN());
  refreshCrc(bytes);
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kNonFiniteValue);

  bytes = encodedFixture();
  writeLeFloat(bytes.data() + 48U, 0.0F);
  refreshCrc(bytes);
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kInvalidScale);

  bytes = encodedFixture();
  writeLeFloat(bytes.data() + 52U, std::numeric_limits<float>::infinity());
  refreshCrc(bytes);
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kInvalidScale);

  bytes = encodedFixture();
  writeLe64(bytes.data() + 24U, 0U);
  refreshCrc(bytes);
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kInvalidMetadata);

  bytes = encodedFixture();
  writeLe64(bytes.data() + 32U, 0U);
  refreshCrc(bytes);
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kInvalidMetadata);

  bytes = encodedFixture();
  writeLeFloat(bytes.data() + 84U + 6U * sizeof(float), 1.01F);
  refreshCrc(bytes);
  EXPECT_EQ(decodeTrackingPacket(bytes.data(), bytes.size()).error,
            TrackingPacketError::kInvalidQuaternion);
}

TEST(TrackingProtocol, RequiresZeroInvalidHeadAndPermitsInactivePlaceholders) {
  TrackingFrame frame = fixtureFrame();
  frame.flags &= static_cast<std::uint16_t>(~kTrackingHeadValid);
  frame.head_pose[0] = 0.25F;
  std::array<std::uint8_t, kTrackingPacketSize> bytes{};
  EXPECT_FALSE(encodeTrackingPacket(frame, bytes));

  frame.head_pose.fill(0.0F);
  frame.flags &= static_cast<std::uint16_t>(~kTrackingRightActive);
  for (auto& pose : frame.right_hand) pose.fill(0.0F);
  EXPECT_TRUE(encodeTrackingPacket(frame, bytes));
  const auto decoded = decodeTrackingPacket(bytes.data(), bytes.size());
  ASSERT_EQ(decoded.error, TrackingPacketError::kNone);
  ASSERT_TRUE(decoded.frame.has_value());
  EXPECT_EQ(decoded.frame->head_pose, TrackingPose{});
}

TEST(TrackingProtocol, RejectsRollbackAndResetsHistoryOnEpochIncrease) {
  TrackingStreamGate gate;
  TrackingFrame frame = fixtureFrame();
  EXPECT_EQ(gate.evaluate(frame).reason, TrackingStreamRejectReason::kNone);
  EXPECT_EQ(gate.evaluate(frame).reason,
            TrackingStreamRejectReason::kSequenceNotIncreasing);

  frame.sequence += 1U;
  frame.source_timestamp_ns -= 1;
  EXPECT_EQ(gate.evaluate(frame).reason,
            TrackingStreamRejectReason::kSourceTimestampNotIncreasing);

  frame.tracking_epoch += 1U;
  frame.sequence = 1U;
  frame.source_timestamp_ns = 1;
  EXPECT_EQ(gate.evaluate(frame).reason, TrackingStreamRejectReason::kNone);

  frame.tracking_epoch -= 1U;
  frame.sequence = 1000U;
  frame.source_timestamp_ns = 2'000'000'000;
  EXPECT_EQ(gate.evaluate(frame).reason,
            TrackingStreamRejectReason::kEpochRollback);
}

}  // namespace tianji_qp_ik

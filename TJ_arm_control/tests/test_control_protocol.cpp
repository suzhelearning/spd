#include "tianji_qp_ik/control_protocol.hpp"
#include "tianji_qp_ik/arm_target_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <fstream>
#include <string>

namespace tianji_qp_ik {
namespace {

static_assert(kControlPacketSize == 40U);

ControlFrame fixtureFrame() {
  ControlFrame frame;
  frame.sequence = 202U;
  frame.monotonic_timestamp_ns = 2'000'000'000;
  frame.command = ControlCommand::kRealign;
  return frame;
}

std::array<std::uint8_t, kControlPacketSize> encodedFixture() {
  std::array<std::uint8_t, kControlPacketSize> bytes{};
  EXPECT_TRUE(encodeControlPacket(fixtureFrame(), bytes));
  return bytes;
}

void writeLe16(std::uint8_t* bytes, std::uint16_t value) {
  bytes[0] = static_cast<std::uint8_t>(value & 0xffU);
  bytes[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
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

void refreshCrc(std::array<std::uint8_t, kControlPacketSize>& bytes) {
  writeLe32(bytes.data() + 12U,
            armTargetCrc32(bytes.data() + 16U, bytes.size() - 16U));
}

std::array<std::uint8_t, kControlPacketSize> readFixture() {
  const std::string fixture_path =
      std::string(TIANJI_PROJECT_SOURCE_DIR) +
      "/../PICO_tracker/src/spd_vr/test/fixtures/control_v1.hex";
  std::ifstream fixture(fixture_path);
  EXPECT_TRUE(fixture.good()) << fixture_path;
  std::string hex;
  fixture >> hex;
  EXPECT_EQ(hex.size(), kControlPacketSize * 2U);
  std::array<std::uint8_t, kControlPacketSize> bytes{};
  for (std::size_t index = 0U; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>(
        std::stoul(hex.substr(index * 2U, 2U), nullptr, 16));
  }
  return bytes;
}

}  // namespace

TEST(ControlProtocol, MatchesGoldenVectorAndRoundTripsEveryCommand) {
  const auto encoded = encodedFixture();
  EXPECT_EQ(encoded, readFixture());
  EXPECT_EQ(std::string(encoded.begin(), encoded.begin() + 4), "SVTC");
  const auto decoded = decodeControlPacket(encoded.data(), encoded.size());
  ASSERT_EQ(decoded.error, ControlPacketError::kNone);
  ASSERT_TRUE(decoded.frame.has_value());
  EXPECT_EQ(decoded.frame->sequence, 202U);
  EXPECT_EQ(decoded.frame->monotonic_timestamp_ns, 2'000'000'000);
  EXPECT_EQ(decoded.frame->command, ControlCommand::kRealign);

  for (std::uint16_t value = 1U; value <= 6U; ++value) {
    ControlFrame frame = fixtureFrame();
    frame.command = static_cast<ControlCommand>(value);
    std::array<std::uint8_t, kControlPacketSize> bytes{};
    ASSERT_TRUE(encodeControlPacket(frame, bytes));
    const auto result = decodeControlPacket(bytes.data(), bytes.size());
    ASSERT_EQ(result.error, ControlPacketError::kNone);
    ASSERT_TRUE(result.frame.has_value());
    EXPECT_EQ(result.frame->command, frame.command);
  }
}

TEST(ControlProtocol, RejectsMalformedPacketsFailClosed) {
  auto bytes = encodedFixture();
  bytes[0] = 'X';
  EXPECT_EQ(decodeControlPacket(bytes.data(), bytes.size()).error,
            ControlPacketError::kWrongMagic);
  EXPECT_FALSE(decodeControlPacket(bytes.data(), bytes.size()).frame.has_value());

  bytes = encodedFixture();
  bytes[4] = 2U;
  EXPECT_EQ(decodeControlPacket(bytes.data(), bytes.size()).error,
            ControlPacketError::kWrongVersion);

  bytes = encodedFixture();
  bytes[8] = 0U;
  EXPECT_EQ(decodeControlPacket(bytes.data(), bytes.size()).error,
            ControlPacketError::kWrongDeclaredSize);

  bytes = encodedFixture();
  bytes[20] ^= 1U;
  EXPECT_EQ(decodeControlPacket(bytes.data(), bytes.size()).error,
            ControlPacketError::kCrcMismatch);

  bytes = encodedFixture();
  writeLe16(bytes.data() + 6U, 7U);
  refreshCrc(bytes);
  EXPECT_EQ(decodeControlPacket(bytes.data(), bytes.size()).error,
            ControlPacketError::kInvalidCommand);

  bytes = encodedFixture();
  writeLe64(bytes.data() + 32U, 1U);
  refreshCrc(bytes);
  EXPECT_EQ(decodeControlPacket(bytes.data(), bytes.size()).error,
            ControlPacketError::kNonZeroReserved);

  bytes = encodedFixture();
  writeLe64(bytes.data() + 24U, 0U);
  refreshCrc(bytes);
  EXPECT_EQ(decodeControlPacket(bytes.data(), bytes.size()).error,
            ControlPacketError::kInvalidMetadata);

  EXPECT_EQ(decodeControlPacket(bytes.data(), bytes.size() - 1U).error,
            ControlPacketError::kWrongSize);
}

TEST(ControlProtocol, DuplicateIsIdempotentAndSmallerSequenceIsRejected) {
  ControlSequenceGate gate;
  ControlFrame frame = fixtureFrame();
  EXPECT_EQ(gate.evaluate(frame).reason, ControlSequenceRejectReason::kNone);
  EXPECT_EQ(gate.evaluate(frame).reason, ControlSequenceRejectReason::kDuplicate);
  frame.sequence -= 1U;
  EXPECT_EQ(gate.evaluate(frame).reason, ControlSequenceRejectReason::kOutOfOrder);
  frame.sequence += 2U;
  EXPECT_EQ(gate.evaluate(frame).reason, ControlSequenceRejectReason::kNone);
}

}  // namespace tianji_qp_ik

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace tianji_qp_ik {

inline constexpr std::size_t kArmTargetPacketV1Size = 272U;
inline constexpr std::uint8_t kArmTargetLeftValid = 1U << 0U;
inline constexpr std::uint8_t kArmTargetRightValid = 1U << 1U;

// The protocol has one reason for the packet; valid_mask identifies which arm
// accepted the model-reference at this control commit.
enum class ArmTargetHoldReason : std::uint8_t {
  kNone = 0U,
  kInputStale = 1U,
  kSolverFailure = 2U,
  kPaused = 3U,
};

enum class ArmTargetPacketError {
  kNone,
  kWrongSize,
  kWrongMagic,
  kWrongVersion,
  kWrongDeclaredSize,
  kInvalidValidMask,
  kInvalidHoldReason,
  kNonZeroReserved,
  kCrcMismatch,
  kInvalidMetadata,
  kNonFiniteValue,
};

struct ArmTargetFrame {
  std::uint64_t sequence{0U};
  std::uint64_t tracking_epoch{0U};
  std::uint64_t source_timestamp_ns{0U};
  std::uint64_t control_timestamp_ns{0U};
  std::uint8_t valid_mask{0U};
  ArmTargetHoldReason hold_reason{ArmTargetHoldReason::kNone};
  std::array<double, 7> left_q{};
  std::array<double, 7> right_q{};
  std::array<double, 7> left_qdot{};
  std::array<double, 7> right_qdot{};
};

struct ArmTargetDecodeResult {
  ArmTargetPacketError error{ArmTargetPacketError::kWrongSize};
  std::optional<ArmTargetFrame> frame;
};

std::uint32_t armTargetCrc32(const std::uint8_t* bytes,
                            std::size_t size) noexcept;

ArmTargetDecodeResult decodeArmTargetPacket(const std::uint8_t* bytes,
                                            std::size_t size) noexcept;

bool encodeArmTargetPacket(
    const ArmTargetFrame& frame,
    std::array<std::uint8_t, kArmTargetPacketV1Size>& bytes) noexcept;

}  // namespace tianji_qp_ik

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace tianji_qp_ik {

inline constexpr std::size_t kTrackingPacketSize = 1540U;
inline constexpr std::uint16_t kTrackingLeftActive = 1U << 0U;
inline constexpr std::uint16_t kTrackingRightActive = 1U << 1U;
inline constexpr std::uint16_t kTrackingHeadValid = 1U << 2U;

using TrackingPose = std::array<float, 7>;
using TrackingHand = std::array<TrackingPose, 26>;

struct TrackingFrame {
  std::uint64_t sequence{0};
  std::uint64_t tracking_epoch{0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t bridge_monotonic_ns{0};
  std::uint16_t flags{0};
  float left_scale{1.0F};
  float right_scale{1.0F};
  TrackingPose head_pose{};
  TrackingHand left_hand{};
  TrackingHand right_hand{};
};

enum class TrackingPacketError {
  kNone,
  kWrongSize,
  kWrongMagic,
  kWrongVersion,
  kUnknownFlags,
  kWrongDeclaredSize,
  kCrcMismatch,
  kInvalidMetadata,
  kInvalidScale,
  kNonFiniteValue,
  kInvalidHeadPose,
  kInvalidQuaternion,
};

struct TrackingDecodeResult {
  TrackingPacketError error{TrackingPacketError::kWrongSize};
  std::optional<TrackingFrame> frame;
};

TrackingDecodeResult decodeTrackingPacket(const std::uint8_t* bytes,
                                          std::size_t size) noexcept;

bool encodeTrackingPacket(
    const TrackingFrame& frame,
    std::array<std::uint8_t, kTrackingPacketSize>& bytes) noexcept;

enum class TrackingStreamRejectReason {
  kNone,
  kEpochRollback,
  kSequenceNotIncreasing,
  kSourceTimestampNotIncreasing,
};

struct TrackingStreamDecision {
  TrackingStreamRejectReason reason{TrackingStreamRejectReason::kNone};

  [[nodiscard]] bool accepted() const noexcept {
    return reason == TrackingStreamRejectReason::kNone;
  }
};

class TrackingStreamGate {
 public:
  TrackingStreamDecision evaluate(const TrackingFrame& frame) noexcept;
  void reset() noexcept;

 private:
  std::optional<std::uint64_t> last_epoch_;
  std::uint64_t last_sequence_{0U};
  std::int64_t last_source_timestamp_ns_{0};
};

}  // namespace tianji_qp_ik

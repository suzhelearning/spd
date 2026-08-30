#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace tianji_qp_ik {

inline constexpr std::size_t kControlPacketSize = 40U;

enum class ControlCommand : std::uint16_t {
  kStart = 1,
  kPause = 2,
  kResume = 3,
  kRealign = 4,
  kReset = 5,
  kShutdown = 6,
};

struct ControlFrame {
  std::uint64_t sequence{0};
  std::int64_t monotonic_timestamp_ns{0};
  ControlCommand command{ControlCommand::kStart};
};

enum class ControlPacketError {
  kNone,
  kWrongSize,
  kWrongMagic,
  kWrongVersion,
  kInvalidCommand,
  kWrongDeclaredSize,
  kCrcMismatch,
  kInvalidMetadata,
  kNonZeroReserved,
};

struct ControlDecodeResult {
  ControlPacketError error{ControlPacketError::kWrongSize};
  std::optional<ControlFrame> frame;
};

ControlDecodeResult decodeControlPacket(const std::uint8_t* bytes,
                                        std::size_t size) noexcept;

bool encodeControlPacket(
    const ControlFrame& frame,
    std::array<std::uint8_t, kControlPacketSize>& bytes) noexcept;

enum class ControlSequenceRejectReason {
  kNone,
  kDuplicate,
  kOutOfOrder,
};

struct ControlSequenceDecision {
  ControlSequenceRejectReason reason{ControlSequenceRejectReason::kNone};

  [[nodiscard]] bool accepted() const noexcept {
    return reason == ControlSequenceRejectReason::kNone;
  }
};

class ControlSequenceGate {
 public:
  ControlSequenceDecision evaluate(const ControlFrame& frame) noexcept;
  void reset() noexcept;

 private:
  std::optional<std::uint64_t> last_sequence_;
};

}  // namespace tianji_qp_ik

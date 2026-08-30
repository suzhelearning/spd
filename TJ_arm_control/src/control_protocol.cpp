#include "tianji_qp_ik/control_protocol.hpp"

#include "tianji_qp_ik/arm_target_protocol.hpp"

#include <cstring>

namespace tianji_qp_ik {
namespace {

std::uint16_t readLe16(const std::uint8_t* bytes) noexcept {
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U);
}

std::uint32_t readLe32(const std::uint8_t* bytes) noexcept {
  std::uint32_t value = 0U;
  for (std::size_t index = 0U; index < 4U; ++index) {
    value |= static_cast<std::uint32_t>(bytes[index]) << (8U * index);
  }
  return value;
}

std::uint64_t readLe64(const std::uint8_t* bytes) noexcept {
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < 8U; ++index) {
    value |= static_cast<std::uint64_t>(bytes[index]) << (8U * index);
  }
  return value;
}

std::int64_t readLeI64(const std::uint8_t* bytes) noexcept {
  const std::uint64_t bits = readLe64(bytes);
  std::int64_t value = 0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void writeLe16(std::uint8_t* bytes, std::uint16_t value) noexcept {
  bytes[0] = static_cast<std::uint8_t>(value & 0xffU);
  bytes[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void writeLe32(std::uint8_t* bytes, std::uint32_t value) noexcept {
  for (std::size_t index = 0U; index < 4U; ++index) {
    bytes[index] = static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

void writeLe64(std::uint8_t* bytes, std::uint64_t value) noexcept {
  for (std::size_t index = 0U; index < 8U; ++index) {
    bytes[index] = static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

void writeLeI64(std::uint8_t* bytes, std::int64_t value) noexcept {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe64(bytes, bits);
}

bool validCommand(ControlCommand command) noexcept {
  const auto value = static_cast<std::uint16_t>(command);
  return value >= static_cast<std::uint16_t>(ControlCommand::kStart) &&
         value <= static_cast<std::uint16_t>(ControlCommand::kShutdown);
}

}  // namespace

ControlDecodeResult decodeControlPacket(const std::uint8_t* bytes,
                                        std::size_t size) noexcept {
  ControlDecodeResult result;
  if (bytes == nullptr || size != kControlPacketSize) {
    result.error = ControlPacketError::kWrongSize;
    return result;
  }
  if (bytes[0] != static_cast<std::uint8_t>('S') ||
      bytes[1] != static_cast<std::uint8_t>('V') ||
      bytes[2] != static_cast<std::uint8_t>('T') ||
      bytes[3] != static_cast<std::uint8_t>('C')) {
    result.error = ControlPacketError::kWrongMagic;
    return result;
  }
  if (readLe16(bytes + 4U) != 1U) {
    result.error = ControlPacketError::kWrongVersion;
    return result;
  }
  const auto command = static_cast<ControlCommand>(readLe16(bytes + 6U));
  if (!validCommand(command)) {
    result.error = ControlPacketError::kInvalidCommand;
    return result;
  }
  if (readLe32(bytes + 8U) != kControlPacketSize) {
    result.error = ControlPacketError::kWrongDeclaredSize;
    return result;
  }
  if (readLe32(bytes + 12U) !=
      armTargetCrc32(bytes + 16U, kControlPacketSize - 16U)) {
    result.error = ControlPacketError::kCrcMismatch;
    return result;
  }
  if (readLe64(bytes + 32U) != 0U) {
    result.error = ControlPacketError::kNonZeroReserved;
    return result;
  }

  ControlFrame frame;
  frame.sequence = readLe64(bytes + 16U);
  frame.monotonic_timestamp_ns = readLeI64(bytes + 24U);
  frame.command = command;
  if (frame.monotonic_timestamp_ns <= 0) {
    result.error = ControlPacketError::kInvalidMetadata;
    return result;
  }
  result.error = ControlPacketError::kNone;
  result.frame = frame;
  return result;
}

bool encodeControlPacket(
    const ControlFrame& frame,
    std::array<std::uint8_t, kControlPacketSize>& bytes) noexcept {
  if (!validCommand(frame.command) || frame.monotonic_timestamp_ns <= 0) {
    return false;
  }
  bytes.fill(0U);
  bytes[0] = static_cast<std::uint8_t>('S');
  bytes[1] = static_cast<std::uint8_t>('V');
  bytes[2] = static_cast<std::uint8_t>('T');
  bytes[3] = static_cast<std::uint8_t>('C');
  writeLe16(bytes.data() + 4U, 1U);
  writeLe16(bytes.data() + 6U,
            static_cast<std::uint16_t>(frame.command));
  writeLe32(bytes.data() + 8U,
            static_cast<std::uint32_t>(kControlPacketSize));
  writeLe64(bytes.data() + 16U, frame.sequence);
  writeLeI64(bytes.data() + 24U, frame.monotonic_timestamp_ns);
  writeLe64(bytes.data() + 32U, 0U);
  writeLe32(bytes.data() + 12U,
            armTargetCrc32(bytes.data() + 16U, kControlPacketSize - 16U));
  return true;
}

ControlSequenceDecision ControlSequenceGate::evaluate(
    const ControlFrame& frame) noexcept {
  if (!last_sequence_.has_value()) {
    last_sequence_ = frame.sequence;
    return {};
  }
  if (frame.sequence == last_sequence_.value()) {
    return {ControlSequenceRejectReason::kDuplicate};
  }
  if (frame.sequence < last_sequence_.value()) {
    return {ControlSequenceRejectReason::kOutOfOrder};
  }
  last_sequence_ = frame.sequence;
  return {};
}

void ControlSequenceGate::reset() noexcept { last_sequence_.reset(); }

}  // namespace tianji_qp_ik

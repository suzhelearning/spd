#include "tianji_qp_ik/arm_target_protocol.hpp"

#include <cmath>
#include <cstring>

namespace tianji_qp_ik {
namespace {

constexpr std::size_t kCrcOffset = kArmTargetPacketV2Size - 4U;
constexpr std::size_t kLeftQOffset = 44U;
constexpr std::size_t kRightQOffset = kLeftQOffset + 7U * sizeof(double);
constexpr std::size_t kLeftQdotOffset = kRightQOffset + 7U * sizeof(double);
constexpr std::size_t kRightQdotOffset = kLeftQdotOffset + 7U * sizeof(double);

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

double readLeDouble(const std::uint8_t* bytes) noexcept {
  const std::uint64_t bits = readLe64(bytes);
  double value = 0.0;
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

void writeLeDouble(std::uint8_t* bytes, double value) noexcept {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe64(bytes, bits);
}

bool validReasonAndMask(std::uint8_t valid_mask,
                        ArmTargetHoldReason left_reason,
                        ArmTargetHoldReason right_reason) noexcept {
  if ((valid_mask & static_cast<std::uint8_t>(~0x03U)) != 0U) return false;
  const auto valid_reason = [](ArmTargetHoldReason reason) {
    return static_cast<std::uint8_t>(reason) <=
           static_cast<std::uint8_t>(ArmTargetHoldReason::kDisconnected);
  };
  if (!valid_reason(left_reason) || !valid_reason(right_reason)) return false;
  const bool left_valid = (valid_mask & kArmTargetLeftValid) != 0U;
  const bool right_valid = (valid_mask & kArmTargetRightValid) != 0U;
  return (left_valid ? left_reason == ArmTargetHoldReason::kNone
                     : left_reason != ArmTargetHoldReason::kNone) &&
         (right_valid ? right_reason == ArmTargetHoldReason::kNone
                      : right_reason != ArmTargetHoldReason::kNone);
}

bool finiteValues(const ArmTargetFrame& frame) noexcept {
  const auto finite = [](const std::array<double, 7>& values) {
    for (const double value : values) {
      if (!std::isfinite(value)) return false;
    }
    return true;
  };
  return finite(frame.left_q) && finite(frame.right_q) &&
         finite(frame.left_qdot) && finite(frame.right_qdot);
}

void decodeArray(const std::uint8_t* bytes, std::array<double, 7>& target) noexcept {
  for (std::size_t index = 0U; index < target.size(); ++index) {
    target[index] = readLeDouble(bytes + index * sizeof(double));
  }
}

void encodeArray(std::uint8_t* bytes, const std::array<double, 7>& source) noexcept {
  for (std::size_t index = 0U; index < source.size(); ++index) {
    writeLeDouble(bytes + index * sizeof(double), source[index]);
  }
}

}  // namespace

std::uint32_t armTargetCrc32(const std::uint8_t* bytes,
                            std::size_t size) noexcept {
  if (bytes == nullptr && size != 0U) return 0U;
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t index = 0U; index < size; ++index) {
    crc ^= bytes[index];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xedb88320U & mask);
    }
  }
  return crc ^ 0xffffffffU;
}

ArmTargetDecodeResult decodeArmTargetPacket(const std::uint8_t* bytes,
                                            std::size_t size) noexcept {
  ArmTargetDecodeResult result;
  if (bytes == nullptr || size != kArmTargetPacketV2Size) {
    result.error = ArmTargetPacketError::kWrongSize;
    return result;
  }
  if (bytes[0] != static_cast<std::uint8_t>('S') ||
      bytes[1] != static_cast<std::uint8_t>('P') ||
      bytes[2] != static_cast<std::uint8_t>('D') ||
      bytes[3] != static_cast<std::uint8_t>('A')) {
    result.error = ArmTargetPacketError::kWrongMagic;
    return result;
  }
  if (readLe16(bytes + 4U) != 2U) {
    result.error = ArmTargetPacketError::kWrongVersion;
    return result;
  }
  if (readLe16(bytes + 6U) != kArmTargetPacketV2Size) {
    result.error = ArmTargetPacketError::kWrongDeclaredSize;
    return result;
  }
  const std::uint8_t valid_mask = bytes[40U];
  const auto left_reason = static_cast<ArmTargetHoldReason>(bytes[41U]);
  const auto right_reason = static_cast<ArmTargetHoldReason>(bytes[42U]);
  if (bytes[41U] > static_cast<std::uint8_t>(ArmTargetHoldReason::kDisconnected) ||
      bytes[42U] > static_cast<std::uint8_t>(ArmTargetHoldReason::kDisconnected)) {
    result.error = ArmTargetPacketError::kInvalidHoldReason;
    return result;
  }
  if (!validReasonAndMask(valid_mask, left_reason, right_reason)) {
    result.error = ArmTargetPacketError::kInvalidValidMask;
    return result;
  }
  if (bytes[43U] != 0U) {
    result.error = ArmTargetPacketError::kNonZeroReserved;
    return result;
  }
  if (readLe32(bytes + kCrcOffset) != armTargetCrc32(bytes, kCrcOffset)) {
    result.error = ArmTargetPacketError::kCrcMismatch;
    return result;
  }
  ArmTargetFrame frame;
  frame.sequence = readLe64(bytes + 8U);
  frame.tracking_epoch = readLe64(bytes + 16U);
  frame.source_timestamp_ns = readLe64(bytes + 24U);
  frame.control_timestamp_ns = readLe64(bytes + 32U);
  frame.valid_mask = valid_mask;
  frame.left_hold_reason = left_reason;
  frame.right_hold_reason = right_reason;
  if (frame.sequence == 0U || frame.tracking_epoch == 0U ||
      frame.source_timestamp_ns == 0U || frame.control_timestamp_ns == 0U) {
    result.error = ArmTargetPacketError::kInvalidMetadata;
    return result;
  }
  decodeArray(bytes + kLeftQOffset, frame.left_q);
  decodeArray(bytes + kRightQOffset, frame.right_q);
  decodeArray(bytes + kLeftQdotOffset, frame.left_qdot);
  decodeArray(bytes + kRightQdotOffset, frame.right_qdot);
  if (!finiteValues(frame)) {
    result.error = ArmTargetPacketError::kNonFiniteValue;
    return result;
  }
  result.error = ArmTargetPacketError::kNone;
  result.frame = frame;
  return result;
}
bool encodeArmTargetPacket(
    const ArmTargetFrame& frame,
    std::array<std::uint8_t, kArmTargetPacketV2Size>& bytes) noexcept {
  if (frame.sequence == 0U || frame.tracking_epoch == 0U ||
      frame.source_timestamp_ns == 0U || frame.control_timestamp_ns == 0U ||
      !validReasonAndMask(frame.valid_mask, frame.left_hold_reason,
                          frame.right_hold_reason) ||
      !finiteValues(frame)) {
    return false;
  }
  bytes.fill(0U);
  bytes[0] = static_cast<std::uint8_t>('S');
  bytes[1] = static_cast<std::uint8_t>('P');
  bytes[2] = static_cast<std::uint8_t>('D');
  bytes[3] = static_cast<std::uint8_t>('A');
  writeLe16(bytes.data() + 4U, 2U);
  writeLe16(bytes.data() + 6U,
            static_cast<std::uint16_t>(kArmTargetPacketV2Size));
  writeLe64(bytes.data() + 8U, frame.sequence);
  writeLe64(bytes.data() + 16U, frame.tracking_epoch);
  writeLe64(bytes.data() + 24U, frame.source_timestamp_ns);
  writeLe64(bytes.data() + 32U, frame.control_timestamp_ns);
  bytes[40U] = frame.valid_mask;
  bytes[41U] = static_cast<std::uint8_t>(frame.left_hold_reason);
  bytes[42U] = static_cast<std::uint8_t>(frame.right_hold_reason);
  bytes[43U] = 0U;
  encodeArray(bytes.data() + kLeftQOffset, frame.left_q);
  encodeArray(bytes.data() + kRightQOffset, frame.right_q);
  encodeArray(bytes.data() + kLeftQdotOffset, frame.left_qdot);
  encodeArray(bytes.data() + kRightQdotOffset, frame.right_qdot);
  writeLe32(bytes.data() + kCrcOffset,
            armTargetCrc32(bytes.data(), kCrcOffset));
  return true;
}


}  // namespace tianji_qp_ik

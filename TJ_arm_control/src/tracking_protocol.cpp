#include "tianji_qp_ik/tracking_protocol.hpp"

#include "tianji_qp_ik/arm_target_protocol.hpp"

#include <cmath>
#include <cstring>
#include <type_traits>

namespace tianji_qp_ik {
namespace {

constexpr std::uint16_t kKnownFlags =
    kTrackingLeftActive | kTrackingRightActive | kTrackingHeadValid;
constexpr std::size_t kHeadOffset = 56U;
constexpr std::size_t kLeftHandOffset = 84U;
constexpr std::size_t kRightHandOffset = 812U;
constexpr float kQuaternionNormTolerance = 1.0e-3F;

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

float readLeFloat(const std::uint8_t* bytes) noexcept {
  const std::uint32_t bits = readLe32(bytes);
  float value = 0.0F;
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

void writeLeFloat(std::uint8_t* bytes, float value) noexcept {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  writeLe32(bytes, bits);
}

void decodePose(const std::uint8_t* bytes, TrackingPose& pose) noexcept {
  for (std::size_t index = 0U; index < pose.size(); ++index) {
    pose[index] = readLeFloat(bytes + index * sizeof(float));
  }
}

void encodePose(std::uint8_t* bytes, const TrackingPose& pose) noexcept {
  for (std::size_t index = 0U; index < pose.size(); ++index) {
    writeLeFloat(bytes + index * sizeof(float), pose[index]);
  }
}

bool poseFinite(const TrackingPose& pose) noexcept {
  for (const float value : pose) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

bool framePosesFinite(const TrackingFrame& frame) noexcept {
  if (!poseFinite(frame.head_pose)) return false;
  for (const TrackingPose& pose : frame.left_hand) {
    if (!poseFinite(pose)) return false;
  }
  for (const TrackingPose& pose : frame.right_hand) {
    if (!poseFinite(pose)) return false;
  }
  return true;
}

bool zeroPose(const TrackingPose& pose) noexcept {
  for (const float value : pose) {
    if (value != 0.0F) return false;
  }
  return true;
}

template <typename Pose>
bool validateRequiredQuaternion(Pose& pose) noexcept {
  double squared_norm = 0.0;
  for (std::size_t index = 3U; index < pose.size(); ++index) {
    const double value = static_cast<double>(pose[index]);
    squared_norm += value * value;
  }
  const double norm = std::sqrt(squared_norm);
  if (!std::isfinite(norm) ||
      std::abs(norm - 1.0) >
          static_cast<double>(kQuaternionNormTolerance)) {
    return false;
  }
  if constexpr (!std::is_const_v<Pose>) {
    for (std::size_t index = 3U; index < pose.size(); ++index) {
      pose[index] =
          static_cast<float>(static_cast<double>(pose[index]) / norm);
    }
  }
  return true;
}

template <typename Frame>
bool validateRequiredQuaternions(Frame& frame) noexcept {
  if ((frame.flags & kTrackingHeadValid) != 0U &&
      !validateRequiredQuaternion(frame.head_pose)) {
    return false;
  }
  if ((frame.flags & kTrackingLeftActive) != 0U) {
    for (auto& pose : frame.left_hand) {
      if (!validateRequiredQuaternion(pose)) return false;
    }
  }
  if ((frame.flags & kTrackingRightActive) != 0U) {
    for (auto& pose : frame.right_hand) {
      if (!validateRequiredQuaternion(pose)) return false;
    }
  }
  return true;
}

template <typename Frame>
TrackingPacketError validateFrame(Frame& frame) noexcept {
  if ((frame.flags & static_cast<std::uint16_t>(~kKnownFlags)) != 0U) {
    return TrackingPacketError::kUnknownFlags;
  }
  if (frame.tracking_epoch == 0U || frame.source_timestamp_ns <= 0 ||
      frame.bridge_monotonic_ns <= 0) {
    return TrackingPacketError::kInvalidMetadata;
  }
  if (!std::isfinite(frame.left_scale) || frame.left_scale <= 0.0F ||
      !std::isfinite(frame.right_scale) || frame.right_scale <= 0.0F) {
    return TrackingPacketError::kInvalidScale;
  }
  if (!framePosesFinite(frame)) return TrackingPacketError::kNonFiniteValue;
  if ((frame.flags & kTrackingHeadValid) == 0U &&
      !zeroPose(frame.head_pose)) {
    return TrackingPacketError::kInvalidHeadPose;
  }
  if (!validateRequiredQuaternions(frame)) {
    return TrackingPacketError::kInvalidQuaternion;
  }
  return TrackingPacketError::kNone;
}

}  // namespace

TrackingDecodeResult decodeTrackingPacket(const std::uint8_t* bytes,
                                          std::size_t size) noexcept {
  TrackingDecodeResult result;
  if (bytes == nullptr || size != kTrackingPacketSize) {
    result.error = TrackingPacketError::kWrongSize;
    return result;
  }
  if (bytes[0] != static_cast<std::uint8_t>('S') ||
      bytes[1] != static_cast<std::uint8_t>('V') ||
      bytes[2] != static_cast<std::uint8_t>('T') ||
      bytes[3] != static_cast<std::uint8_t>('1')) {
    result.error = TrackingPacketError::kWrongMagic;
    return result;
  }
  if (readLe16(bytes + 4U) != 1U) {
    result.error = TrackingPacketError::kWrongVersion;
    return result;
  }
  const std::uint16_t flags = readLe16(bytes + 6U);
  if ((flags & static_cast<std::uint16_t>(~kKnownFlags)) != 0U) {
    result.error = TrackingPacketError::kUnknownFlags;
    return result;
  }
  if (readLe32(bytes + 8U) != kTrackingPacketSize) {
    result.error = TrackingPacketError::kWrongDeclaredSize;
    return result;
  }
  if (readLe32(bytes + 12U) !=
      armTargetCrc32(bytes + 16U, kTrackingPacketSize - 16U)) {
    result.error = TrackingPacketError::kCrcMismatch;
    return result;
  }

  TrackingFrame frame;
  frame.sequence = readLe64(bytes + 16U);
  frame.tracking_epoch = readLe64(bytes + 24U);
  frame.source_timestamp_ns = readLeI64(bytes + 32U);
  frame.bridge_monotonic_ns = readLeI64(bytes + 40U);
  frame.flags = flags;
  frame.left_scale = readLeFloat(bytes + 48U);
  frame.right_scale = readLeFloat(bytes + 52U);
  decodePose(bytes + kHeadOffset, frame.head_pose);
  for (std::size_t index = 0U; index < frame.left_hand.size(); ++index) {
    decodePose(bytes + kLeftHandOffset + index * 7U * sizeof(float),
               frame.left_hand[index]);
    decodePose(bytes + kRightHandOffset + index * 7U * sizeof(float),
               frame.right_hand[index]);
  }

  result.error = validateFrame(frame);
  if (result.error != TrackingPacketError::kNone) return result;

  result.frame = frame;
  return result;
}

bool encodeTrackingPacket(
    const TrackingFrame& frame,
    std::array<std::uint8_t, kTrackingPacketSize>& bytes) noexcept {
  if (validateFrame(frame) != TrackingPacketError::kNone) return false;

  bytes.fill(0U);
  bytes[0] = static_cast<std::uint8_t>('S');
  bytes[1] = static_cast<std::uint8_t>('V');
  bytes[2] = static_cast<std::uint8_t>('T');
  bytes[3] = static_cast<std::uint8_t>('1');
  writeLe16(bytes.data() + 4U, 1U);
  writeLe16(bytes.data() + 6U, frame.flags);
  writeLe32(bytes.data() + 8U,
            static_cast<std::uint32_t>(kTrackingPacketSize));
  writeLe64(bytes.data() + 16U, frame.sequence);
  writeLe64(bytes.data() + 24U, frame.tracking_epoch);
  writeLeI64(bytes.data() + 32U, frame.source_timestamp_ns);
  writeLeI64(bytes.data() + 40U, frame.bridge_monotonic_ns);
  writeLeFloat(bytes.data() + 48U, frame.left_scale);
  writeLeFloat(bytes.data() + 52U, frame.right_scale);
  encodePose(bytes.data() + kHeadOffset, frame.head_pose);
  for (std::size_t index = 0U; index < frame.left_hand.size(); ++index) {
    encodePose(bytes.data() + kLeftHandOffset + index * 7U * sizeof(float),
               frame.left_hand[index]);
    encodePose(bytes.data() + kRightHandOffset + index * 7U * sizeof(float),
               frame.right_hand[index]);
  }
  writeLe32(bytes.data() + 12U,
            armTargetCrc32(bytes.data() + 16U,
                           kTrackingPacketSize - 16U));
  return true;
}

TrackingStreamDecision TrackingStreamGate::evaluate(
    const TrackingFrame& frame) noexcept {
  if (!last_epoch_.has_value()) {
    last_epoch_ = frame.tracking_epoch;
    last_sequence_ = frame.sequence;
    last_source_timestamp_ns_ = frame.source_timestamp_ns;
    return {};
  }
  if (frame.tracking_epoch < last_epoch_.value()) {
    return {TrackingStreamRejectReason::kEpochRollback};
  }
  if (frame.tracking_epoch == last_epoch_.value()) {
    if (frame.sequence <= last_sequence_) {
      return {TrackingStreamRejectReason::kSequenceNotIncreasing};
    }
    if (frame.source_timestamp_ns <= last_source_timestamp_ns_) {
      return {TrackingStreamRejectReason::kSourceTimestampNotIncreasing};
    }
  }
  last_epoch_ = frame.tracking_epoch;
  last_sequence_ = frame.sequence;
  last_source_timestamp_ns_ = frame.source_timestamp_ns;
  return {};
}

void TrackingStreamGate::reset() noexcept {
  last_epoch_.reset();
  last_sequence_ = 0U;
  last_source_timestamp_ns_ = 0;
}

}  // namespace tianji_qp_ik

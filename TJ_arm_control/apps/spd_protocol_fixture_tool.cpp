#include "tianji_qp_ik/arm_target_protocol.hpp"
#include "tianji_qp_ik/control_protocol.hpp"
#include "tianji_qp_ik/tracking_protocol.hpp"

#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string_view>
#include <vector>

namespace tianji_qp_ik {
namespace {

TrackingFrame trackingFixture() {
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

ControlFrame controlFixture() {
  ControlFrame frame;
  frame.sequence = 202U;
  frame.monotonic_timestamp_ns = 2'000'000'000;
  frame.command = ControlCommand::kRealign;
  return frame;
}

ArmTargetFrame armTargetFixture() {
  ArmTargetFrame frame;
  frame.sequence = 17U;
  frame.tracking_epoch = 9U;
  frame.source_timestamp_ns = 1'000'000'000U;
  frame.control_timestamp_ns = 1'000'001'000U;
  frame.valid_mask = kArmTargetLeftValid;
  frame.left_hold_reason = ArmTargetHoldReason::kNone;
  frame.right_hold_reason = ArmTargetHoldReason::kInactive;
  for (std::size_t index = 0U; index < 7U; ++index) {
    const double joint = static_cast<double>(index);
    frame.left_q[index] = 0.1 * joint;
    frame.right_q[index] = -0.2 * joint;
    frame.left_qdot[index] = 0.3 * joint;
    frame.right_qdot[index] = -0.4 * joint;
  }
  return frame;
}

template <std::size_t Size>
int writeRaw(const std::array<std::uint8_t, Size>& bytes) {
  for (const std::uint8_t byte : bytes) {
    std::cout.put(static_cast<char>(byte));
  }
  return std::cout.good() ? 0 : 1;
}

std::vector<std::uint8_t> readRaw() {
  std::vector<std::uint8_t> bytes;
  char byte = 0;
  while (std::cin.get(byte)) {
    bytes.push_back(static_cast<std::uint8_t>(
        static_cast<unsigned char>(byte)));
  }
  return bytes;
}

template <typename Value, std::size_t Size>
void writeArray(const std::array<Value, Size>& values) {
  std::cout << '[';
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index != 0U) std::cout << ',';
    std::cout << values[index];
  }
  std::cout << ']';
}

void writeHand(const TrackingHand& hand) {
  std::cout << '[';
  for (std::size_t index = 0U; index < hand.size(); ++index) {
    if (index != 0U) std::cout << ',';
    writeArray(hand[index]);
  }
  std::cout << ']';
}

int encodeTracking() {
  std::array<std::uint8_t, kTrackingPacketSize> bytes{};
  if (!encodeTrackingPacket(trackingFixture(), bytes)) return 1;
  return writeRaw(bytes);
}

int encodeControl() {
  std::array<std::uint8_t, kControlPacketSize> bytes{};
  if (!encodeControlPacket(controlFixture(), bytes)) return 1;
  return writeRaw(bytes);
}

int encodeArmTarget() {
  std::array<std::uint8_t, kArmTargetPacketSize> bytes{};
  if (!encodeArmTargetPacket(armTargetFixture(), bytes)) return 1;
  return writeRaw(bytes);
}

int decodeTracking() {
  const std::vector<std::uint8_t> bytes = readRaw();
  const auto decoded = decodeTrackingPacket(bytes.data(), bytes.size());
  if (decoded.error != TrackingPacketError::kNone ||
      !decoded.frame.has_value()) {
    return 1;
  }
  const TrackingFrame& frame = decoded.frame.value();
  std::cout << "{\"sequence\":" << frame.sequence
            << ",\"tracking_epoch\":" << frame.tracking_epoch
            << ",\"source_timestamp_ns\":" << frame.source_timestamp_ns
            << ",\"bridge_monotonic_ns\":" << frame.bridge_monotonic_ns
            << ",\"flags\":" << frame.flags
            << ",\"left_scale\":" << frame.left_scale
            << ",\"right_scale\":" << frame.right_scale
            << ",\"head_pose\":";
  writeArray(frame.head_pose);
  std::cout << ",\"left_hand\":";
  writeHand(frame.left_hand);
  std::cout << ",\"right_hand\":";
  writeHand(frame.right_hand);
  std::cout << '}';
  return std::cout.good() ? 0 : 1;
}

int decodeControl() {
  const std::vector<std::uint8_t> bytes = readRaw();
  const auto decoded = decodeControlPacket(bytes.data(), bytes.size());
  if (decoded.error != ControlPacketError::kNone ||
      !decoded.frame.has_value()) {
    return 1;
  }
  const ControlFrame& frame = decoded.frame.value();
  std::cout << "{\"sequence\":" << frame.sequence
            << ",\"monotonic_timestamp_ns\":"
            << frame.monotonic_timestamp_ns << ",\"command\":"
            << static_cast<std::uint16_t>(frame.command) << '}';
  return std::cout.good() ? 0 : 1;
}

int decodeArmTarget() {
  const std::vector<std::uint8_t> bytes = readRaw();
  const auto decoded = decodeArmTargetPacket(bytes.data(), bytes.size());
  if (decoded.error != ArmTargetPacketError::kNone ||
      !decoded.frame.has_value()) {
    return 1;
  }
  const ArmTargetFrame& frame = decoded.frame.value();
  std::cout << "{\"sequence\":" << frame.sequence
            << ",\"tracking_epoch\":" << frame.tracking_epoch
            << ",\"source_timestamp_ns\":" << frame.source_timestamp_ns
            << ",\"control_timestamp_ns\":" << frame.control_timestamp_ns
            << ",\"valid_mask\":" << static_cast<unsigned>(frame.valid_mask)
            << ",\"left_hold_reason\":"
            << static_cast<unsigned>(frame.left_hold_reason)
            << ",\"right_hold_reason\":"
            << static_cast<unsigned>(frame.right_hold_reason)
            << ",\"left_q\":";
  writeArray(frame.left_q);
  std::cout << ",\"right_q\":";
  writeArray(frame.right_q);
  std::cout << ",\"left_qdot\":";
  writeArray(frame.left_qdot);
  std::cout << ",\"right_qdot\":";
  writeArray(frame.right_qdot);
  std::cout << '}';
  return std::cout.good() ? 0 : 1;
}

}  // namespace
}  // namespace tianji_qp_ik

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
  const std::string_view command{argv[1]};
  if (command == "encode-tracking") return tianji_qp_ik::encodeTracking();
  if (command == "encode-control") return tianji_qp_ik::encodeControl();
  if (command == "encode-arm-target") return tianji_qp_ik::encodeArmTarget();
  if (command == "decode-tracking") return tianji_qp_ik::decodeTracking();
  if (command == "decode-control") return tianji_qp_ik::decodeControl();
  if (command == "decode-arm-target") return tianji_qp_ik::decodeArmTarget();
  return 2;
}

#include "tianji_qp_ik/wuji_hand_udp_receiver.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>

namespace tianji_qp_ik {
namespace {

WujiHandTeleopFrame frameWithSequence(std::uint64_t sequence) {
  WujiHandTeleopFrame frame;
  frame.sequence = sequence;
  frame.source_timestamp_ns = static_cast<std::int64_t>(sequence + 100U);
  frame.right_valid = true;
  frame.right[0] = static_cast<double>(sequence);
  return frame;
}

void sendPacket(std::uint16_t port, const WujiHandTeleopFrame& frame) {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(socket_fd, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  ASSERT_EQ(inet_pton(AF_INET, "127.0.0.1", &address.sin_addr), 1);
  const std::vector<std::uint8_t> packet = encodeWujiHandTeleopPacket(frame);
  ASSERT_EQ(sendto(socket_fd, packet.data(), packet.size(), 0,
                   reinterpret_cast<const sockaddr*>(&address),
                   sizeof(address)),
            static_cast<ssize_t>(packet.size()));
  close(socket_fd);
}

TEST(WujiHandUdpReceiver, ReceivesLatestFrameAndRejectsRollback) {
  LatestSpscExchange<WujiHandTeleopFrame> frames;
  WujiHandUdpReceiverOptions options;
  options.port = 0U;
  options.stale_timeout_seconds = 0.5;
  WujiHandUdpReceiver receiver(options, frames);
  receiver.start();
  ASSERT_NE(receiver.boundPort(), 0U);

  sendPacket(receiver.boundPort(), frameWithSequence(2U));
  WujiHandTeleopFrame received;
  bool received_frame = false;
  for (int attempt = 0; attempt < 100 && !received_frame; ++attempt) {
    received_frame = frames.tryReadLatest(received);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(received_frame);
  EXPECT_EQ(received.sequence, 2U);
  EXPECT_DOUBLE_EQ(received.right[0], 2.0);

  sendPacket(receiver.boundPort(), frameWithSequence(1U));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const WujiHandReceiverStats stats = receiver.stats();
  EXPECT_EQ(stats.accepted, 1U);
  EXPECT_EQ(stats.reordered, 1U);
  EXPECT_FALSE(stats.stale);

  receiver.stop();
}

TEST(WujiHandUdpReceiver, ReportsStaleAfterConfiguredTimeout) {
  LatestSpscExchange<WujiHandTeleopFrame> frames;
  WujiHandUdpReceiverOptions options;
  options.port = 0U;
  options.stale_timeout_seconds = 0.01;
  WujiHandUdpReceiver receiver(options, frames);
  receiver.start();
  ASSERT_NE(receiver.boundPort(), 0U);
  sendPacket(receiver.boundPort(), frameWithSequence(1U));

  for (int attempt = 0; attempt < 100; ++attempt) {
    if (receiver.stats().accepted == 1U) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(receiver.stats().stale);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_TRUE(receiver.stats().stale);
  receiver.stop();
}

}  // namespace
}  // namespace tianji_qp_ik

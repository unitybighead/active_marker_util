#include "multicast.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <cstring>
#include <iostream>

UdpReceiver::UdpReceiver(int port, const std::string &multicast_address,
                         std::function<void(std::vector<uint8_t>)> callback)
    : callback_(callback), is_runnning_(false) {
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return;
  }

  // 再利用可能なソケットに設定
  int reuse = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse,
                 sizeof(reuse)) < 0) {
    std::cerr << "Failed to set SO_REUSEADDR" << std::endl;
    return;
  }

  sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;  // 全インターフェースで受信
  servaddr.sin_port = htons(port);

  if (bind(sockfd_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    std::cerr << "Failed to bind socket" << std::endl;
    return;
  }

  // マルチキャストグループへの参加
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr =
      inet_addr(multicast_address.c_str());       // マルチキャストアドレス
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);  // デフォルトインターフェース
  if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq,
                 sizeof(mreq)) < 0) {
    std::cerr << "Failed to join multicast group" << std::endl;
    return;
  }

  start();
}

UdpReceiver::~UdpReceiver() {
  stop();
  close(sockfd_);
}

void UdpReceiver::start() {
  is_runnning_ = true;
  udp_thread_ = std::thread(&UdpReceiver::udp_receive, this);
}

void UdpReceiver::stop() {
  is_runnning_ = false;
  if (udp_thread_.joinable()) {
    udp_thread_.join();
  }
}

void UdpReceiver::udp_receive() {
  uint8_t buffer[2048];
  sockaddr_in cliaddr;
  socklen_t len = sizeof(cliaddr);

  while (is_runnning_) {
    ssize_t n = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                         (struct sockaddr *)&cliaddr, &len);
    if (n > 0) {
      char addr[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &cliaddr.sin_addr, addr, INET_ADDRSTRLEN);
      std::cout << "Received packet from " << addr << ":"
                << ntohs(cliaddr.sin_port) << " (size: " << n << " bytes)"
                << std::endl;

      std::vector<uint8_t> data(buffer, buffer + n);
      callback_(data);
    } else {
      perror("recvfrom failed");
    }
  }
}

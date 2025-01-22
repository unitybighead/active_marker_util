#ifndef MULTICAST_HPP_
#define MULTICAST_HPP_

#include <unistd.h>

#include <functional>
#include <thread>
#include <vector>

class UdpReceiver {
 public:
  UdpReceiver(int port, const std::string &multicast_address,
              std::function<void(std::vector<uint8_t>)> callback);
  ~UdpReceiver();

  void start();
  void stop();

 private:
  void udp_receive();
  int sockfd_;
  std::thread udp_thread_;
  bool is_runnning_;
  std::function<void(std::vector<uint8_t>)> callback_;
};

#endif  // MULTICAST_HPP_

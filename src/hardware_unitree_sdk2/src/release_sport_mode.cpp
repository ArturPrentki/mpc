// Switches off the Go2's built-in sport mode (or any other active motion
// service) so that low-level commands from ros2_control are not fought by
// Unitree's own controller. Run it before launching the controllers on the
// real robot:
//
//   ros2 run hardware_unitree_sdk2 release_sport_mode enx08beac225241
//
// The network port can also come from UNITREE_NETWORK_INTERFACE. The robot
// goes limp when sport mode is released: put it on a stand or lay it down
// first.

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char **argv) {
  std::string iface;
  if (argc > 1) {
    iface = argv[1];
  } else if (const char *env = std::getenv("UNITREE_NETWORK_INTERFACE")) {
    iface = env;
  }
  if (iface.empty() || iface == "-h" || iface == "--help") {
    std::cerr << "usage: release_sport_mode <network_interface>\n"
                 "  network_interface: Ethernet port wired to the Go2 (see "
                 "'ip a'),\n  or set UNITREE_NETWORK_INTERFACE\n";
    return 2;
  }

  constexpr int kDomain = 0;  // real robot
  std::cout << "Connecting to the Go2 on " << iface << " (domain " << kDomain
            << ")..." << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(kDomain, iface);

  unitree::robot::b2::MotionSwitcherClient switcher;
  switcher.SetTimeout(5.0f);
  switcher.Init();

  constexpr int kMaxAttempts = 5;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    std::string form, name;
    const int32_t ret = switcher.CheckMode(form, name);
    if (ret != 0) {
      std::cerr << "Cannot reach the robot's motion switcher (error " << ret
                << "). Check the cable, the port name and that the PC has a "
                   "192.168.123.x address on it."
                << std::endl;
      return 1;
    }
    if (name.empty()) {
      std::cout << "Sport mode is off. The robot is ready for low-level "
                   "control (launch the controllers now)."
                << std::endl;
      return 0;
    }
    std::cout << "Motion service '" << name << "' is active, releasing it ("
              << attempt << "/" << kMaxAttempts
              << "). The robot will go limp." << std::endl;
    if (switcher.ReleaseMode() != 0) {
      std::cerr << "ReleaseMode failed, retrying" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  std::cerr << "Sport mode is still active after " << kMaxAttempts
            << " attempts. Restart the robot and try again." << std::endl;
  return 1;
}

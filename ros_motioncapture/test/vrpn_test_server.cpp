// A real VRPN tracker with a fixed pose, used only by the launch test.
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) {running = 0;}
}

int main()
{
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  auto * connection = vrpn_create_server_connection("127.0.0.1:43883");
  if (!connection || !connection->doing_okay()) return 1;
  {
    vrpn_Tracker_Server tracker("vrpn_test_body", connection);
    const vrpn_float64 position[] = {1.0, 2.0, 3.0};
    // 90 degrees around Z, in VRPN's x, y, z, w order.
    const vrpn_float64 rotation[] = {0.0, 0.0, 0.7071067811865476, 0.7071067811865476};
    std::cout << "VRPN test server ready" << std::endl;
    while (running) {
      timeval stamp;
      vrpn_gettimeofday(&stamp, nullptr);
      tracker.report_pose(0, stamp, position, rotation);
      tracker.mainloop();
      connection->mainloop();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  connection->removeReference();
  return 0;
}

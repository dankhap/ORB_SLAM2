#include <Converter.h>
#include <Queue.h>
#include <iostream>
#include <thread>

namespace ctello {
class Tello {
public:
  bool Bind() {
    std::cout << "bind finished" << std::endl;
    return true;
  }

  void SendCommand(std::string cmd) {
    std::cout << "sent command: " << cmd << std::endl;
  }
  bool ReceiveResponse() {
    std::cout << "received command" << std::endl;
    return true;
  }
};
} // namespace ctello
namespace ORB_SLAM2 {

class System;
}

enum Mode { CREATED = 0, STARTED, EXPLORE, NAVIGATE };

class TelloDispatcher {
public:
  TelloDispatcher(ctello::Tello *tello, ORB_SLAM2::System *slam);
  void Run();
  Mode getState() { return mState; }
  void setState(Mode state);

  Queue<std::string> messageQueue;

private:
  ctello::Tello *mTello;
  ORB_SLAM2::System *mSLAM;
  Mode mState;
};

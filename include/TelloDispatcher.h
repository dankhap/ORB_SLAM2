#ifndef TELLO_DISPATCHER_H
#define TELLO_DISPATCHER_H

#include <Eigen/Dense>
#include <atomic>
#include <ctello.h>
#include <iostream>
#include <thread>

#include <Converter.h>
#include <Queue.h>
#include <System.h>
// namespgace ctello {
/* class Tello {
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
}; */
// }
using namespace ctello;

enum Mode {
  CREATED = 0,
  READY,
  EXPLORING,
  EXPLORE_COMPLETE,
  EXIT_MAPPING,
  EXIT_FOUND,
  FINISH,
  END
};

class TelloDispatcher {
public:
  TelloDispatcher(ctello::Tello *tello, ORB_SLAM2::System *slam,
                  std::atomic<int> *state);
  void addExplorationCommands();
  void startExitDiscovery();
  void Run();
  int getState() { return *mState; }
  void setState(int state);

  Eigen::Vector4f getExitPos();
  void navigateToExit(Eigen::Vector4f exit);
  Queue<std::string> messageQueue;

private:
  ctello::Tello *mTello;
  ORB_SLAM2::System *mSLAM;
  std::atomic<int> *mState;
};

#endif

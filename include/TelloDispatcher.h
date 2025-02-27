#ifndef TELLO_DISPATCHER_H
#define TELLO_DISPATCHER_H

#include <Eigen/Dense>
#include <atomic>
#include <ctello.h>
#include <iostream>
#include <thread>

#include <Converter.h>
#include <ExitMapper.h>
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
class MyRotatedRect;

enum Mode {
  CREATED = 0,
  READY,
  LOCALIZING,
  EXPLORING,
  EXPLORE_COMPLETE,
  EXIT_MAPPING,
  EXIT_FOUND,
  RESTARTING,
  NAVIGATING,
  FINISH,
  END
};

class TelloDispatcher {
public:
  TelloDispatcher(ctello::Tello *tello, ORB_SLAM2::System *slam,
                  std::atomic<int> *state);
  void startExitDiscovery();
  void Run();
  int getState() { return *mState; }
  void setState(int state);
  void sendStateMessage(int s);
  Eigen::Vector4f getExitPos();
  void navigateToExit(Eigen::Vector4f exit);
  void setExit(Eigen::Vector4f exit);
  void setExitMapping(MyRotatedRect *room, Eigen::Vector4f *exitPoint);
  ~TelloDispatcher();
  Queue<std::string> messageQueue;

private:
  ctello::Tello *mTello;
  ORB_SLAM2::System *mSLAM;
  std::atomic<int> *mState;
  Eigen::Vector4f *mExit;
  MyRotatedRect *mRect;
  float mDestToTarget;

  void addNavigationCommands();
  bool isArrived(cv::Mat cpos);
};

#endif

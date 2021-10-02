#include "TelloDispatcher.h"
#include "MyRotatedRect.h"
#include "System.h"

#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/matx.hpp>

using namespace std;
using namespace ctello;
using namespace Eigen;

void TelloDispatcher::sendStateMessage(int s) {
  messageQueue.push("#" + to_string(s));
}
TelloDispatcher::TelloDispatcher(Tello *tello, ORB_SLAM2::System *slam,
                                 std::atomic<int> *state)
    : mTello(tello), mSLAM(slam), mState(state) {
  mExit = nullptr;
  mRect = nullptr;
  mDestToTarget = numeric_limits<float>::max();
}
void TelloDispatcher::Run() {
  cout << "sending streamon command" << endl;

  mTello->SendCommand("streamon");
  while (!(mTello->ReceiveResponse()))
    ;
  cout << "got stream ok" << endl;

  // Take-off first
  mTello->SendCommand("takeoff");
  while (!(mTello->ReceiveResponse()))
    sleep(1);
  cout << "takeoff ok" << endl;
  *mState = (int)Mode::READY;
  cout << "land ok" << endl;

  string command;
  command.resize(100);
  bool finished = false;
  while (!finished) {
    // Listen response
    cout << "Dispatcher: read queue..." << endl;
    // check emergency abort
    if (filesystem::exists("abort.txt")) {

      while (messageQueue.tryPop(command))
        ;
      messageQueue.push("land");
      *mState = Mode::FINISH;
    }

    messageQueue.waitAndPop(command);
    cout << "Dispatcher: got command: " << command << endl;

    if (command[0] == '#') {
      int state = stoi(command.substr(1));
      // int state = (int)Mode::FINISH;
      cout << "Dispatcher: transition to state: " << state << endl;
      // finished explore state, move to navigate state and clean command
      // queue
      setState(state);
      while (messageQueue.tryPop(command))
        ;
    } else {
      // regular command, send and wait completion
      mTello->SendCommand(command);
      while (!(mTello->ReceiveResponse()))
        sleep(1);
    }
    switch (*mState) {
    case Mode::FINISH:
      cout << "Dispatcher: reached FINISH state" << endl;
      finished = true;
      break;
    case Mode::EXIT_MAPPING:
      cout << "Dispatcher: reached EXIT_MAPPING state" << endl;
      messageQueue.push("land");
      sleep(3);
      break;

    case Mode::EXIT_FOUND:
      cout << "Dispatcher: reached EXIT_FOUND state" << endl;
      messageQueue.push("streamon");
      messageQueue.push("takeoff");
      messageQueue.push("up 20");
      messageQueue.push("down 20");
      *mState = Mode::RESTARTING;
      while (mTello->ReceiveResponse()) {
        cout << "Dispatcher: cleaning response queue" << endl;
      }
      break;
    case Mode::RESTARTING:
      if (messageQueue.empty()) {
        messageQueue.push("up 20");
        messageQueue.push("down 20");
      }
      break;
    case Mode::NAVIGATING:
      cout << "Dispatcher: reached NAVIGATING state" << endl;
      if (!messageQueue.empty()) {
        continue;
      }
      if (isArrived()) {
        cout << "Dispatcher: arrived at destination!!!" << endl;
        sendStateMessage(Mode::FINISH);
      } else {

        cout << "Dispatcher: calculating next navigation step" << endl;
        addNavigationCommands();
      }
      break;
    }
  }

  mTello->SendCommand("land");
  while (!(mTello->ReceiveResponse()))
    sleep(1);
  *mState = Mode::END;
}

void TelloDispatcher::addNavigationCommands() {
  // Navigation plane is in x,z plane, z is forward

  Eigen::Matrix4f cpose;
  cv::cv2eigen(mSLAM->mpTracker->mCurrentFrame.mTcw, cpose);
  Vector4f rpose = cpose * (*mExit);
  Vector4f nrpose = rpose.normalized();
  float theta = acos(nrpose.z());
  if (nrpose.x() < 0) {
    theta *= -1;
  }
  theta *= 180 / M_PI;
  string command;
  if (theta < 0)
    command = "cw ";
  else
    command = "ccw ";
  command += to_string((int)theta);
  messageQueue.push(command);
  messageQueue.push("forward 30");
}

bool TelloDispatcher::isArrived() {
  Vector4f cpos(0);
  cv::cv2eigen(mSLAM->mpTracker->mCurrentFrame.GetCameraCenter(), cpos);
  float dst = (*mExit - cpos).norm();
  cout << "Dispatcher: destination to target= " << dst << endl;

  if (dst > mDestToTarget) {
    return true;
  }
  mDestToTarget = dst;
  return !mRect->isInside(cpos);
}

void TelloDispatcher::setExitMapping(MyRotatedRect *room,
                                     Eigen::Vector4f *exitPoint) {
  mRect = room;
  mExit = exitPoint;
}

void TelloDispatcher::addExplorationCommands() {

  vector<string> commands{"takeoff", "up 5"};
  int dirNum = 11;
  int degInc = 360 / dirNum;
  for (int i = 0; i < dirNum + 3; i++) {
    commands.push_back("forward 30");
    commands.push_back("back 30");
    ostringstream oss;
    oss << "cw " << degInc;
    commands.push_back(oss.str());
  }
  for (string i : commands) {
    messageQueue.push(i);
  }
  sendStateMessage(Mode::EXPLORE_COMPLETE);
  *mState = (int)Mode::EXPLORING;
}

Eigen::Vector4f TelloDispatcher::getExitPos() {
  return Eigen::Vector4f::Zero();
}
void TelloDispatcher::navigateToExit(Eigen::Vector4f exit) { return; }
void TelloDispatcher::startExitDiscovery() { return; }
void TelloDispatcher::setState(int state) { *mState = state; }

TelloDispatcher::~TelloDispatcher() {}

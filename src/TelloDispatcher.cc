#include "TelloDispatcher.h"
// #include "ctello.h"
#include <Eigen/src/Core/Matrix.h>
#include <System.h>
#include <iostream>
using namespace std;
using namespace ctello;

TelloDispatcher::TelloDispatcher(Tello *tello, ORB_SLAM2::System *slam,
                                 std::atomic<int> *state)
    : mTello(tello), mSLAM(slam), mState(state) {}
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
  /* for (int i = 0; i < 12; i++) {

    mTello->SendCommand("cw 35");
    while (!(mTello->ReceiveResponse()))
      sleep(1);
    cout << "cw ok" << endl;

    mTello->SendCommand("forward 30");
    while (!(mTello->ReceiveResponse()))
      sleep(1);
    cout << "forward ok" << endl;

    mTello->SendCommand("back 30");
    while (!(mTello->ReceiveResponse()))
      sleep(1);
    cout << "back ok" << endl;
    sleep(3);
  } */

  cout << "land ok" << endl;

  string command;
  command.resize(100);
  bool finished = false;
  while (!finished) {
    // Listen response
    cout << "read queue..." << endl;

    messageQueue.waitAndPop(command);
    cout << "got command: " << command << endl;

    if (command[0] == '#') {
      // int state = stoi(command.substr(1));
      int state = (int)Mode::FINISH;
      cout << "transition to state: " << state << endl;
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
      cout << "command: '" << command << "' OK" << endl;
    }
    if (*mState == (int)Mode::FINISH) {
      break;
    }
  }

  mTello->SendCommand("land");
  while (!(mTello->ReceiveResponse()))
    sleep(1);
  *mState = Mode::END;
}

void TelloDispatcher::addExplorationCommands() {

  vector<string> commands{"takeoff", "up 5"};
  int dirNum = 9;
  int degInc = 360 / dirNum;
  for (int i = 0; i < dirNum + 1; i++) {
    commands.push_back("forward 30");
    commands.push_back("back 30");
    ostringstream oss;
    oss << "cw " << degInc;
    commands.push_back(oss.str());
  }
  commands.push_back("#3");
  for (string i : commands) {
    messageQueue.push(i);
  }
  *mState = (int)Mode::EXPLORING;
}

Eigen::Vector4f TelloDispatcher::getExitPos() {
  return Eigen::Vector4f::Zero();
}
void TelloDispatcher::navigateToExit(Eigen::Vector4f exit) { return; }
void TelloDispatcher::startExitDiscovery() { return; }
void TelloDispatcher::setState(int state) { *mState = state; }

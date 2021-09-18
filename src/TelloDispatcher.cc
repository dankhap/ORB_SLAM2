#include "TelloDispatcher.h"
// #include "ctello.h"
#include <System.h>
#include <iostream>
using namespace std;
using namespace ctello;

TelloDispatcher::TelloDispatcher(Tello *tello, ORB_SLAM2::System *slam)
    : mTello(tello), mSLAM(slam), mState(Mode::CREATED) {}
void TelloDispatcher::Run() {

  mTello->SendCommand("streamon");

  bool busy{true};
  while (true) {
    string command;
    command.resize(100);
    // Listen response
    if (const auto response = mTello->ReceiveResponse()) {
      std::cout << "Tello: " << response << std::endl;
      busy = false;
    }

    // Act
    if (!busy) {
      if (mState == Mode::CREATED) {
        setState(Mode::EXPLORE);
      }
      messageQueue.waitAndPop(command);
      if (command[0] == '#') {
        cout << "transition state" << endl;
        // finished explore state, move to navigate state and clean command
        // queue
        setState(Mode::NAVIGATE);
        while (messageQueue.tryPop(command))
          ;
        continue;
      }
      mTello->SendCommand(command);
      std::cout << "Command: " << command << std::endl;
      busy = true;
    } else {
      sleep(1);
    }
  }
}

void TelloDispatcher::setState(Mode state) { mState = state; }

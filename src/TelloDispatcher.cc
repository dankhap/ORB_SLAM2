#include <TelloDispatcher.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <iostream>

using namespace boost::interprocess;

TelloDispatcher::TelloDispatcher(Tello &tello, ORB_SLAM2::System &slam)
    : mTello(tello), mSLAM(slam), mState(Mode::EXPLORE) {}
void TelloDispatcher::Run() {
  int message_size_limit = 100;
  boost::interprocess::message_queue mQueue(
      open_or_create, "mq", message_size_limit, message_size_limit);
  mTello.SendCommand("streamon");
  while (!(mTello.ReceiveResponse()))
    ;
  unsigned int priority;
  message_queue::size_type recvd_size;
  mTello.SendCommand("streamon");
  bool busy{false};
  while (true) {
    string command;
    command.resize(100);
    // Listen response
    if (const auto response = mTello.ReceiveResponse()) {
      std::cout << "Tello: " << *response << std::endl;
      busy = false;
    }

    // Act
    if (!busy) {
      mQueue.receive(&command[0], 100, recvd_size, priority);
      mTello.SendCommand(command);
      std::cout << "Command: " << command << std::endl;
      busy = true;
    }
  }
}

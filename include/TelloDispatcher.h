#include "ctello.h"
#include <Converter.h>
#include <System.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <thread>

using namespace std;
using namespace ctello;

enum Mode { EXPLORE = 0, NAVIGATE };

class TelloDispatcher {
public:
  TelloDispatcher(Tello &tello, ORB_SLAM2::System &slam);
  void Run();
  Mode getState() { return mState; }

private:
  Tello &mTello;
  ORB_SLAM2::System &mSLAM;
  Mode mState;
};

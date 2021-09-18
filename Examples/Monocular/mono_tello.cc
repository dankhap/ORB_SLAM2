
/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
// #include <iostream>
#include <unistd.h>
// #include "ctello.h"
#include <Converter.h>
#include <System.h>
#include <TelloDispatcher.h>
#include <thread>

// const char *const TELLO_STREAM_URL{"udp://0.0.0.0:11111"}; // real tello info
// const char *const TELLO_STREAM_URL{"udp://192.168.1.13:11111"};
// const char *const TELLO_STREAM_URL{"http://192.168.1.13:8080"};
const char *const TELLO_STREAM_URL{"http://192.168.1.13/playlist.m3u:8080"};
using namespace std;
using namespace ctello;

using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::waitKey;
void addExplorationCommands(TelloDispatcher *tello) {

  vector<string> commands{"takeoff", "up 10"};
  int dirNum = 8;
  int degInc = 360 / dirNum;
  for (int i = 0; i < dirNum; i++) {
    commands.push_back("forward 20");
    commands.push_back("back 20");
    ostringstream oss;
    oss << "cw " << degInc;
    commands.push_back(oss.str());
  }
  commands.push_back("#cstate");
  for (string i : commands) {
    tello->messageQueue.push(i);
  }
}

int main(int argc, char **argv) {
  if (argc < 3) {
    cerr << endl
         << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
            "[r - reuse map]"
         << endl;
    return 1;
  }
  bool reuseMap = argc == 4;
  // bind to tello
  Tello tello;
  if (!tello.Bind()) {
    cerr << "Unable to connect to tello" << endl;
  }
  TelloDispatcher *oDispatcher;
  bool taskFinished{false};
  bool sentExploration{false};
  VideoCapture capture{TELLO_STREAM_URL, cv::CAP_ANY}; // cv::CAP_FFMPEG};
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true,
                         reuseMap);
  // ORB_SLAM2::System *SLAM = nullptr;
  std::thread *mptDispatcher;
  // Initialize the Local Mapping thread and launch
  oDispatcher = new TelloDispatcher(&tello, &SLAM);
  mptDispatcher = new thread(&TelloDispatcher::Run, oDispatcher);

  // Vector for tracking time statistics
  vector<float> vTimesTrack(1000);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  // Main loop
  cv::Mat im;
  // Adding empty elements to lists to avoid segmentation fault when tracking is
  // lost at the beginning of the loading
  SLAM.mpTracker->mlRelativeFramePoses.push_back(cv::Mat::eye(4, 4, CV_32F));
  SLAM.mpTracker->mlpReferences.push_back(NULL);
  SLAM.mpTracker->mlFrameTimes.push_back(0.0);
  SLAM.mpTracker->mlbLost.push_back(true);
  while (!taskFinished) {
    // Read image from file
    //
    capture >> im;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    auto microsecondsUTC =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    double tframe = microsecondsUTC / 1000000.0;
    // Pass the image to the SLAM system
    imshow("frame", im);
    char c = (char)waitKey(25);
    if (c == 27)
      break;
    // SLAM.TrackMonocular(im, tframe);
    sleep(1);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count();

    vTimesTrack.push_back(ttrack);

    if (!sentExploration && oDispatcher->getState() == Mode::EXPLORE &&
        !reuseMap) {
      // calculate direction
      cout << "starting exploring sequence" << endl;
      addExplorationCommands(oDispatcher);
      sentExploration = true;
    } else if (oDispatcher->getState() == Mode::NAVIGATE) {
      cout << "calculating directions" << endl;
      SLAM.SaveMap("Slam_latest_Map.bin");
    }
  }

  // Stop all threads
  // SLAM.Shutdown();
  capture.release();
  cv::destroyAllWindows();
  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  int nImages = vTimesTrack.size();
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  /* SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  SLAM.SaveMap("Slam_latest_Map.bin");
 */
  return 0;
}


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

#include "MyRotatedRect.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <thread>
#include <unistd.h>

#include <Converter.h>
#include <ExitMapper.h>
#include <System.h>
#include <TelloDispatcher.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

const char *const TELLO_STREAM_URL{
    "udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000"}; // real
                                                                  // tello
// "udp://0.0.0.0:11111?overrun_nonfatal=1"}; // real tello
// info
// const char *const TELLO_STREAM_URL{"udp://192.168.1.13:11111"};
// const char *const TELLO_STREAM_URL{"http://192.168.1.13:8080"};
// const char *const TELLO_STREAM_URL{"http://192.168.1.13/playlist.m3u:8080"};
using namespace std;
using namespace ctello;

using cv::CAP_FFMPEG;
using cv::imshow;
using cv::VideoCapture;
using cv::VideoWriter;
using cv::waitKey;

bool handleState(int state, ExitMapper *mapper, TelloDispatcher *oDispatcher,
                 ORB_SLAM2::System &SLAM, bool gotFrame, VideoCapture **cap) {
  bool taskFinished = false;
  std::thread *mptMapper;
  MyRotatedRect *room;
  Eigen::Vector4f *exitPoint;
  cv::Mat tim;
  switch (state) {
  case Mode::READY:
    cout << "Main: ready, sending exploration commands..." << endl;
    oDispatcher->addExplorationCommands();
    break;
  case Mode::EXPLORE_COMPLETE:
    cout << "Main: exploration finished, calculating map" << endl;
    oDispatcher->messageQueue.push("streamoff");
    oDispatcher->setState(Mode::EXIT_MAPPING);
    oDispatcher->sendStateMessage(Mode::EXIT_MAPPING);
    break;

  case Mode::EXIT_MAPPING:
    if (!gotFrame && !mapper->MappingRequested()) {

      cout << "Main: video feed ended, starting room mapping..." << endl;
      SLAM.ActivateLocalizationMode();
      SLAM.SaveMap("Slam_latest_Map_tello.bin");
      mapper->SetCloud(SLAM.GetMap()->GetAllMapPoints());
      mptMapper = new std::thread(&ExitMapper::Run, mapper);
      (*cap)->release();
      delete *cap;
      *cap = nullptr;
    }
    if (mapper->FinishedMapping()) {
      cout << "Main: exit found, navigating..." << endl;
      exitPoint = mapper->getExitPoint();
      room = mapper->getRoomRect();
      oDispatcher->setExitMapping(room, exitPoint);
      oDispatcher->sendStateMessage(Mode::EXIT_FOUND);
    }
    if (!gotFrame) {
      cout << "Main: waiting for mapper..." << endl;
      sleep(1);
    }
    break;
  case Mode::RESTARTING:
    cout << "Main: wating for restart..." << endl;
    if (gotFrame) {
      cout << "Main: restart complete, got video" << endl;
      oDispatcher->sendStateMessage(Mode::NAVIGATING);
      oDispatcher->setState(Mode::NAVIGATING);
    } else {
      if (*cap == nullptr) {
        cout << "Main: restarting capture" << endl;
        *cap = new VideoCapture(TELLO_STREAM_URL, cv::CAP_FFMPEG);
        while (tim.empty()) {
          (**cap) >> tim;
        }
      }
    }
    break;
  case Mode::END:
    cout << "Main: navigation ended..." << endl;
    taskFinished = true;
    break;
  };
  return taskFinished;
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
  ExitMapper *mapper;
  TelloDispatcher *oDispatcher;
  bool taskFinished{false};

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true,
                         reuseMap);
  std::atomic<int> busy = Mode::CREATED;
  std::thread *mptDispatcher;

  // Initialize the Local Mapping thread and launch
  oDispatcher = new TelloDispatcher(&tello, &SLAM, &busy);
  mptDispatcher = new thread(&TelloDispatcher::Run, oDispatcher);

  mapper = new ExitMapper(&busy);

  // Vector for tracking time statistics
  vector<float> vTimesTrack(1000);
  int state;
  do {
    state = oDispatcher->getState();
    cout << "checking busy: " << state << endl;
    sleep(1);
  } while (state == Mode::CREATED);

  cout << "starting capture..." << endl;
  VideoCapture *capture =
      new VideoCapture(TELLO_STREAM_URL, cv::CAP_FFMPEG); // cv::CAP_FFMPEG};
  capture->set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  capture->set(cv::CAP_PROP_FRAME_WIDTH, 640);
  int frame_width = 640;
  int frame_height = 480;
  /* VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P',
     'G'), 30, cv::Size(frame_width, frame_height)); */
  cout << endl << "-------" << endl;
  cout << "Start processing sequence ... requested " << frame_width << " x "
       << frame_height << endl;
  // Main loop
  cv::Mat im;
  // Adding empty elements to lists to avoid segmentation fault when tracking is
  // lost at the beginning of the loading
  SLAM.mpTracker->mlRelativeFramePoses.push_back(cv::Mat::eye(4, 4, CV_32F));
  SLAM.mpTracker->mlpReferences.push_back(NULL);
  SLAM.mpTracker->mlFrameTimes.push_back(0.0);
  SLAM.mpTracker->mlbLost.push_back(true);
  // Wait for first frame
  do {
    (*capture) >> im;
  } while (im.empty());
  bool gotFrame = true;
  while (!taskFinished) {
    // Read image from file
    if (capture != nullptr) {

      (*capture) >> im;
    }
    gotFrame = !im.empty();
    if (gotFrame) {

      cv::Mat sim;
      cv::resize(im, sim, cv::Size(frame_width, frame_height));

      // video.write(im);
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
      auto microsecondsUTC =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::system_clock::now().time_since_epoch())
              .count();
      double tframe = microsecondsUTC / 1000000.0;
      // Pass the image to the SLAM system
      SLAM.TrackMonocular(sim, tframe);
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      vTimesTrack.push_back(ttrack);
    }
    int cState = oDispatcher->getState();
    taskFinished =
        handleState(cState, mapper, oDispatcher, SLAM, gotFrame, &capture);
  }

  // Stop all threads
  cout << "writing video" << endl;
  SLAM.Shutdown();
  // video.release();
  capture->release();
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
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  delete mapper;
  delete oDispatcher;
  delete mptDispatcher;
  delete capture;
  return 0;
}

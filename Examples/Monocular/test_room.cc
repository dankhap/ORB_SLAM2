/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundati>on, either version 3 of the License, or
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

#include <Converter.h>
#include <Eigen/Dense>
#include <System.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <unistd.h>
using namespace std;
using namespace Eigen;
MatrixXd getPoints(vector<ORB_SLAM2::MapPoint *> pcloud) {

  MatrixXd mp = MatrixXd::Zero(pcloud.size(), 3);
  int i{0};
  double toppoint = -1000;
  double bottpoint = 1000;
  for (auto p : pcloud) {
    Eigen::Matrix<double, 3, 1> v =
        ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
    // cout << v.x() << "," << v.y() << "," << v.z() << endl;
    mp.row(i) = v;
    i++;
    if (v.y() > toppoint)
      toppoint = v.y();
    if (v.y() < bottpoint)
      bottpoint = v.y();
  }
  cout << "max:" << toppoint << endl;
  cout << "min:" << bottpoint << endl;
  double height = toppoint - bottpoint;
  double bth = bottpoint + height * 0.25;
  double tth = bottpoint + height * 0.75;
  std::vector<int> keep_rows;
  for (int i = 0; i < mp.rows(); ++i) {
    double y = mp(i, mp.cols() - 1);
    if (y > bth && y < tth) {
      keep_rows.push_back(i);
    }
  }
  VectorXi keep_cols = VectorXi::LinSpaced(mp.cols(), 0, mp.cols());
  // MatrixXd res = mp(keep_rows, keep_cols);
  return mp;
}

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl
         << "Usage: ./mono_tum path_to_vocabulary path_to_settings "
            "path_to_sequence"
         << endl;
    return 1;
  }
  cout << "eigen v:" << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "."
       << EIGEN_MINOR_VERSION << endl;
  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true,
                         true);
  // Vector for tracking time statistics

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  vector<ORB_SLAM2::MapPoint *> points = SLAM.GetMap()->GetAllMapPoints();
  cout << "# size=" << points.size() << endl;

  // Main loop
  cv::Mat im;
  // Adding empty elements to lists to avoid segmentation fault when tracking is
  // lost at the beginning of the loading
  SLAM.mpTracker->mlRelativeFramePoses.push_back(cv::Mat::eye(4, 4, CV_32F));
  SLAM.mpTracker->mlpReferences.push_back(NULL);
  SLAM.mpTracker->mlFrameTimes.push_back(0.0);
  SLAM.mpTracker->mlbLost.push_back(true);
  // Stop all threadsz
  SLAM.Shutdown();

  // Tracking time statistics
  cout << "-------" << endl << endl;

  return 0;
}

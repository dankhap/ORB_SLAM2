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
#include <RotCalipers.h>
#include <System.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/distances.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <unistd.h>
#include <utility>

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;
using namespace ORB_SLAM2;

template <typename FwdIt> class adjacent_iterator {
public:
  adjacent_iterator(FwdIt first, FwdIt last)
      : m_first(first), m_next(first == last ? first : std::next(first)) {}

  bool operator!=(const adjacent_iterator &other) const {
    return m_next != other.m_next; // NOT m_first!
  }

  adjacent_iterator &operator++() {
    ++m_first;
    ++m_next;
    return *this;
  }

  typedef typename std::iterator_traits<FwdIt>::reference Ref;
  typedef std::pair<Ref, Ref> Pair;

  Pair operator*() const {
    return Pair(*m_first, *m_next); // NOT std::make_pair()!
  }

private:
  FwdIt m_first;
  FwdIt m_next;
};

template <typename FwdIt> class adjacent_range {
public:
  adjacent_range(FwdIt first, FwdIt last) : m_first(first), m_last(last) {}

  adjacent_iterator<FwdIt> begin() const {
    return adjacent_iterator<FwdIt>(m_first, m_last);
  }

  adjacent_iterator<FwdIt> end() const {
    return adjacent_iterator<FwdIt>(m_last, m_last);
  }

private:
  FwdIt m_first;
  FwdIt m_last;
};

template <typename C>
auto make_adjacent_range(C &c) -> adjacent_range<decltype(c.begin())> {
  return adjacent_range<decltype(c.begin())>(c.begin(), c.end());
}

class MyRotatedRect {
public:
  MyRotatedRect() {
    v1s = 0;
    v2s = 0;
  }
  MyRotatedRect(const MyRotatedRect &c) {
    corner = c.corner;
    vector1 = c.vector1;
    vector2 = c.vector2;
    v1s = c.v1s;
    v2s = c.v2s;
    for (auto p : c.lines) {
      lines.push_back(p);
    }
  }
  MyRotatedRect(cv::Point2f cor, cv::Point2f vec1, cv::Point2f vec2) {
    corner = Vector2f(cor.x, cor.y);
    vector1 = Vector2f(vec1.x, vec1.y);
    vector2 = Vector2f(vec2.x, vec2.y);
    v1s = vector1.norm();
    vector1.normalize();
    v2s = vector2.norm();
    vector2.normalize();
    updateLines();
  }
  void scaleDown(float eps) {
    // scale down....
    cout << "vec sizes: " << v1s << ", " << v2s << endl;
    corner -= (eps / 2) * (vector1 + vector2);
    v1s -= eps * 2;
    v2s -= eps * 2;
  }
  vector<PointXYZ> lines;

private:
  void updateLines() {
    lines.clear();
    Vector2f v1 = v1s * vector1;
    Vector2f v2 = v2s * vector2;
    lines.push_back(PointXYZ(corner.x(), corner.y(), 0));
    lines.push_back(PointXYZ(corner.x() + v2.x(), corner.y() + v2.y(), 0));
    lines.push_back(PointXYZ(lines[1].x + v1.x(), lines[1].y + v1.y(), 0));
    lines.push_back(PointXYZ(lines[2].x - v2.x(), lines[2].y - v2.y(), 0));
    lines.push_back(lines[0]);
  }

  Vector2f corner;
  Vector2f vector2;
  Vector2f vector1;
  float v1s, v2s;
};

void subsample(PointCloud<PointXYZ>::Ptr cloud,
               PointCloud<PointXYZ>::Ptr sample, double ratio) {
  int size = cloud->size();
  int sample_size = size * ratio;
  vector<bool> choosen(size + 1, false);
  choosen[0] = true;
  ArrayXi indices(sample_size);
  for (int i = 0; i < sample_size; i++) {

    int idx = 0;
    while (choosen[idx] == true) {

      idx = rand() % size;
    }
    indices[i] = idx - 1;
    choosen[idx] = true;
    PointXYZ p = cloud->points[idx];
    p.z = 0;
    sample->push_back(p);
  }
}

double getEpsilon(MyRotatedRect rectInfo,
                  pcl::PointCloud<PointXYZ>::Ptr subcloud) {
  float maxEps{0};
  vector<double> distanceFrom(subcloud->size());
  int i{0};
  for (auto p : *subcloud) {
    float mindist = 9999;
    for (const auto l : make_adjacent_range(rectInfo.lines)) {
      // dist of p from;
      Eigen::Vector4f pt = p.getVector4fMap(),
                      line_pt = l.first.getVector4fMap(),
                      line_dir = l.second.getVector4fMap();
      double dst = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);
      if (dst < mindist) {
        mindist = dst;
      }
    }
    distanceFrom[i] = mindist;
    i++;
    if (mindist > maxEps) {
      maxEps = mindist;
    }
  }
  return sqrt(maxEps);
}

int scoreRect(MyRotatedRect model, pcl::PointCloud<PointXYZ>::Ptr cloud,
              float eps) {
  eps = eps * eps / 4;
  int score{0};
  for (auto p : *cloud) {
    float mindist = 9999;
    p.z = 0;
    for (const auto l : make_adjacent_range(model.lines)) {
      // dist of p from;
      Eigen::Vector4f pt = p.getVector4fMap(),
                      line_pt = l.first.getVector4fMap(),
                      line_dir = l.second.getVector4fMap();

      double dst = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);
      if (dst < mindist) {
        mindist = dst;
      }
    }
    if (mindist < eps) {
      score++;
    }
  }
  return score;
}

void savePoints(MatrixXd points) {
  ofstream myfile("CSC2134.txt");

  if (myfile.is_open()) {
    string str;
    for (int i = 0; i < points.rows(); i++) {

      myfile << points.row(i) << endl;
    }
    myfile.close();
  }
}
void getPoints(vector<ORB_SLAM2::MapPoint *> pcloud,
               PointCloud<PointXYZ>::Ptr cloud) {

  PointCloud<PointXYZ>::Ptr tmpCloud(new PointCloud<PointXYZ>());
  for (auto p : pcloud) {
    Eigen::Matrix<double, 3, 1> v =
        ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
    tmpCloud->push_back(PointXYZ(v.x(), v.z(), v.y()));
  }
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("original.pcd", *tmpCloud, false);

  StatisticalOutlierRemoval<PointXYZ> sor;
  sor.setInputCloud(tmpCloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud);

  writer.write<pcl::PointXYZ>("cleaned_original.pcd", *cloud, false);
}

MyRotatedRect getMinRect(pcl::PointCloud<PointXYZ>::Ptr ch) {

  int n = ch->size();
  Point2f *hpoints = new Point2f[n];
  Point2f *out = new Point2f[3];
  for (int i = 0; i < n; i++) {
    hpoints[i] = Point2f(ch->points[i].x, ch->points[i].y);
  }
  cv::minAreaRect(hpoints, (int)ch->size(), out);
  Point2f vec1 = out[1];
  Point2f vec2 = out[2];
  Point2f cor1 = out[0];
  float dot = vec1.x * vec2.x + vec1.y * vec2.y;
  MyRotatedRect res(cor1, vec1, vec2);
  return res;
}
void saveCloud() {}
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

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  vector<ORB_SLAM2::MapPoint *> points = SLAM.GetMap()->GetAllMapPoints();
  cout << "# size=" << points.size() << endl;
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
  getPoints(points, cloud);
  int iters = 5;
  double sampleRatio = 0.02;
  VectorXf modelScores = VectorXf::Zero(iters);
  vector<MyRotatedRect> models;
  cout << "creating models.." << endl;

  for (int i = 0; i < iters; i++) {

    PointCloud<PointXYZ> rectModel;
    pcl::PCDWriter writer;
    PointCloud<PointXYZ>::Ptr subcloud(new PointCloud<PointXYZ>());
    subsample(cloud, subcloud, sampleRatio);
    writer.write<pcl::PointXYZ>("subsample.pcd", *subcloud, false);

    PointCloud<PointXYZ>::Ptr ch(new PointCloud<PointXYZ>());
    ConvexHull<PointXYZ> ch2d;
    ch2d.setInputCloud(subcloud);
    ch2d.reconstruct(*ch);

    writer.write<pcl::PointXYZ>("cvhull.pcd", *ch, false);
    MyRotatedRect rectInfo = getMinRect(ch);
    double eps = getEpsilon(rectInfo, subcloud);

    rectModel.clear();
    rectModel.insert(rectModel.begin(), rectInfo.lines.begin(),
                     rectInfo.lines.end());
    writer.write<pcl::PointXYZ>("rect_before_scale.pcd", rectModel, false);
    rectInfo.scaleDown(eps);

    rectModel.clear();
    rectModel.insert(rectModel.begin(), rectInfo.lines.begin(),
                     rectInfo.lines.end());
    writer.write<pcl::PointXYZ>("rect_afterscale.pcd", rectModel, false);
    models.push_back(rectInfo);
    modelScores[i] = eps;
  }

  cout << "evaluating models..." << endl;

  float minEps = modelScores.minCoeff();
  float avgEps = modelScores.mean();
  float maEps = (avgEps - minEps) / 2;
  MyRotatedRect minBestRect, avgBestRect, maBestRect;
  float minBest{0}, avgBest{0}, maBest{0};
  for (int i = 0; i < iters; i++) {

    int score = scoreRect(models[i], cloud, minEps);
    if (score > minBest) {
      minBest = score;
      minBestRect = models[i];
    }
    score = scoreRect(models[i], cloud, avgEps);
    if (score > avgBest) {
      avgBest = score;
      avgBestRect = models[i];
    }
    score = scoreRect(models[i], cloud, maEps);
    if (score > maBest) {
      maBest = score;
      maBestRect = models[i];
    }
  }

  PointCloud<PointXYZ> rectModel;
  pcl::PCDWriter writer;
  rectModel.clear();
  rectModel.insert(rectModel.begin(), minBestRect.lines.begin(),
                   minBestRect.lines.end());
  writer.write<pcl::PointXYZ>("min_eps_model.pcd", rectModel, false);
  cout << "saved best min model: " << minEps << " with score: " << minBest
       << endl;
  rectModel.clear();
  rectModel.insert(rectModel.begin(), avgBestRect.lines.begin(),
                   avgBestRect.lines.end());
  writer.write<pcl::PointXYZ>("avg_eps_model.pcd", rectModel, false);
  cout << "saved best avg model: " << avgEps << " with score: " << avgBest
       << endl;
  rectModel.clear();
  rectModel.insert(rectModel.begin(), maBestRect.lines.begin(),
                   maBestRect.lines.end());
  writer.write<pcl::PointXYZ>("ma_eps_model.pcd", rectModel, false);
  cout << "saved best am model: " << maEps << " with score: " << maBest << endl;

  // savePoints(wallPoints);
  // Stop all threadsz
  SLAM.Shutdown();

  // Tracking time statistics
  cout << "-------" << endl << endl;

  return 0;
}

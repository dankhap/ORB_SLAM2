#include "ExitMapper.h"
#include "Converter.h"
#include "MyRotatedRect.h"
#include "RotCalipers.h"

#include "System.h"
#include <iostream>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace ORB_SLAM2;
using namespace std;
using pcl::PointCloud;
using pcl::PointXYZ;
using namespace Eigen;
using cv::Point2f;

int removeOnRectPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudloud,
                       MyRotatedRect minBestRect,
                       pcl::PointCloud<pcl::PointXYZ>::ConstPtr outCloud) {
  return 0;
}

Eigen::Vector4f *ExitMapper::getExitPoint() { return mExit; }
MyRotatedRect *ExitMapper::getRoomRect() { return mRect; }

pcl::visualization::PCLVisualizer::Ptr
shapesVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
          pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr coloredCloud,
          MyRotatedRect &box, pcl::PointXYZ *far) {

  PointCloud<PointXYZ>::Ptr rectPoints(new PointCloud<PointXYZ>());
  rectPoints->insert(rectPoints->begin(), box.lines.begin(), box.lines.end());

  // pcl::PointCloud<pcl::PointXYZ>::ConstPtr far) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      coloredCloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(coloredCloud, rgb, "sample cloud");
  /* pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud,
     0, 255, 0); */
  // viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "sample cloud");
  // viewer->addCoordinateSystem(0.05);
  viewer->initCameraParameters();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  stringstream ss;
  int i = 0;
  double cprec = 1 / 10;
  ss << "line" << i;
  for (auto p : make_adjacent_range(rectPoints->points)) {

    viewer->addLine<pcl::PointXYZ>(p.first, p.second, 1, 0, cprec * i,
                                   ss.str());

    ss.clear();
    i++;
    ss << "line" << i;
  }
  if (far != nullptr) {

    viewer->addSphere(*far, 0.005, 1, 1, 0, "far");
  }
  return (viewer);
}

PointXYZ findExit(std::vector<pcl::PointIndices> clusters,
                  pcl::PointCloud<PointXYZ>::ConstPtr cloud,
                  MyRotatedRect &roomRect) {

  int selected = 0;
  float maxDst = 0;
  Eigen::Vector4f exit = Eigen::Vector4f::Zero();
  int i = 0;

  for (auto c : clusters) {
    PointCloud<PointXYZ>::Ptr cCloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr flatCloud(new PointCloud<PointXYZ>);
    pcl::copyPointCloud(*cloud, c, *cCloud);
    Eigen::Vector4f cent = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(*cCloud, cent);
    double dst = 0;
    /* double area = 0;
    double vol = 0; */
    /* for (auto p : *cCloud) {
      flatCloud->points.push_back(PointXYZ(p.x, p.y, 0));
    } */
    /* ConvexHull<PointXYZ> ch2d;
    ConvexHull<PointXYZ> ch3d;
    ch2d.setInputCloud(flatCloud);
    area = ch2d.getTotalArea();
    vol = ch2d.getTotalVolume(); */
    dst = roomRect.distanceTo(cent, false);
    if (!roomRect.isInside(cent)) {
      if (dst > maxDst) {
        maxDst = dst;
        exit = cent;
        selected = i;
      }
    }
    i++;
  }
  cout << "selected is: " << selected << endl;
  return PointXYZ(exit[0], exit[1], exit[2]);
}

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
  PointXYZ farthest;
  for (auto p : *subcloud) {
    float mindist = rectInfo.distanceTo(p.getArray4fMap());
    distanceFrom[i] = mindist;
    i++;
    if (mindist > maxEps) {
      maxEps = mindist;
      farthest = p;
    }
  }
  // return farthest;
  return sqrt(maxEps);
}

int scoreRect(MyRotatedRect model, pcl::PointCloud<PointXYZ>::Ptr cloud,
              float eps) {
  eps = eps * eps / 4;
  int score{0};
  for (auto p : *cloud) {
    p.z = 0;
    float mindist = model.distanceTo(p.getArray4fMap(), true);
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

  pcl::StatisticalOutlierRemoval<PointXYZ> sor;
  sor.setInputCloud(tmpCloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.70);
  sor.filter(*cloud);

  writer.write<pcl::PointXYZ>("cleaned_original.pcd", *cloud, false);
}

MyRotatedRect getMinRect(pcl::PointCloud<PointXYZ>::Ptr ch) {

  int n = ch->size();
  Point2f *hpoints = new Point2f[n];
  Point2f *out = new Point2f[3];
  for (int i = 0; i < n; i++) {
    hpoints[i] = Point2f(ch->points[n - 1 - i].x, ch->points[n - 1 - i].y);
  }
  cv::minAreaRect(hpoints, (int)ch->size(), out);
  Point2f vec1 = out[1];
  Point2f vec2 = out[2];
  Point2f cor1 = out[0];
  MyRotatedRect res(cor1, vec1, vec2);
  return res;
}

void ExitMapper::SetCloud(vector<ORB_SLAM2::MapPoint *> points) {

  getPoints(points, mCloud);
}

bool ExitMapper::MappingRequested() { return *mMappingRequested; }

void ExitMapper::Run() {
  *mMappingRequested = true;
  int iters = 150;
  double sampleRatio = 0.05;
  VectorXf modelScores = VectorXf::Zero(iters);
  vector<MyRotatedRect> models;
  cout << "creating models.." << endl;

  for (int i = 0; i < iters; i++) {

    PointCloud<PointXYZ>::Ptr rectModel(new PointCloud<PointXYZ>());
    pcl::PCDWriter writer;
    PointCloud<PointXYZ>::Ptr subcloud(new PointCloud<PointXYZ>());
    subsample(mCloud, subcloud, sampleRatio);

    PointCloud<PointXYZ>::Ptr ch(new PointCloud<PointXYZ>());
    pcl::ConvexHull<PointXYZ> ch2d;
    ch2d.setInputCloud(subcloud);
    ch2d.reconstruct(*ch);

    MyRotatedRect rectInfo = getMinRect(ch);
    double eps = getEpsilon(rectInfo, subcloud);
    rectInfo.scaleDown(eps);

    models.push_back(rectInfo);
    modelScores[i] = eps;
  }

  cout << "evaluating models..." << endl;

  float minEps = modelScores.minCoeff();
  float avgEps = modelScores.mean();
  float maEps = (avgEps - minEps) / 2;
  float avgPlusEps = avgEps + (maEps - minEps);
  MyRotatedRect avpBestRect;
  float avpBest{0};
  for (int i = 0; i < iters; i++) {

    int score = scoreRect(models[i], mCloud, avgPlusEps);
    if (score > avpBest) {
      avpBest = score;
      avpBestRect = models[i];
    }
  }

  PointCloud<PointXYZ>::Ptr outCloud(new PointCloud<PointXYZ>());
  removeOnRectPoints(mCloud, avpBestRect, outCloud);

  pcl::search::Search<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(mCloud);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(mCloud);
  // reg.setIndices (indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud = reg.getColoredCloud();

  cout << "saved best am model: " << avgPlusEps << " with score: " << avpBest
       << endl;
  PointXYZ exitPoint = findExit(clusters, mCloud, avpBestRect);
  pcl::visualization::PCLVisualizer::Ptr viewer;
  /* viewer = shapesVis(mCloud, coloredCloud, avpBestRect, &exitPoint);
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  } */
  mExit = new Vector4f(0);
  *mExit = exitPoint.getArray4fMap();
  mRect = new MyRotatedRect();
  *mRect = avpBestRect;
  *mExitFound = true;
}

ExitMapper::~ExitMapper() { delete mExitFound; }
ExitMapper::ExitMapper(std::atomic<int> *state) : mState(state) {
  mCloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
  mExitFound = new bool(false);
  mMappingRequested = new bool(false);
}
bool ExitMapper::FinishedMapping() { return *mExitFound; }

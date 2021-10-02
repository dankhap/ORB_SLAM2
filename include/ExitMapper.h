#ifndef EXITMAPPER_H
#define EXITMAPPER_H

#include "System.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class MyRotatedRect;

class ExitMapper {
public:
  ExitMapper(std::atomic<int> *state);

  void SetCloud(vector<ORB_SLAM2::MapPoint *> points);
  void Run();
  bool FinishedMapping();
  bool MappingRequested();
  Eigen::Vector4f *getExitPoint();
  MyRotatedRect *getRoomRect();
  ~ExitMapper();

private:
  bool *mExitFound;
  bool *mMappingRequested;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
  Eigen::Vector4f *mExit;
  MyRotatedRect *mRect;
  std::atomic<int> *mState;
};

#endif /* EXITMAPPER_H */

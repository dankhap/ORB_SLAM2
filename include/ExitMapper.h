#ifndef EXITMAPPER_H
#define EXITMAPPER_H

#include "System.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class ExitMapper {
public:
  ExitMapper();

  void SetCloud(vector<ORB_SLAM2::MapPoint *> points);
  void Run();
  pcl::PointXYZ FindExit();
  bool ExitReady();

private:
  bool exit_ready;
  bool request_exit;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointXYZ exit;
};

#endif /* EXITMAPPER_H */

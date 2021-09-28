#ifndef EXITMAPPER_H
#define EXITMAPPER_H

class ExitMapper {
public:
  ExitMapper();
  void SetCloud(pcl::PointCloud<pcl::PointXYZ> cloud);
  void Run();
  pcl::PointXYZ FindExit();
  bool ExitReady();

private:
  bool exit_ready;
  bool request_exit;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  PointXYZ exit;
};

#endif /* EXITMAPPER_H */

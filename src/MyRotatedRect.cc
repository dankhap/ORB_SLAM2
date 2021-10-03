#include "MyRotatedRect.h"
#include <algorithm>
#include <pcl/common/distances.h>

using namespace std;

float MyRotatedRect::distanceTo(Eigen::Vector4f pt, bool inside) {
  float mindist = 99999;
  int lnum = 0;
  for (const auto l : make_adjacent_range(lines)) {
    // dist of p from;
    Eigen::Vector4f line_pt = l.first.getVector4fMap();
    Eigen::Vector4f line_dir = l.second.getVector4fMap() - line_pt;
    pt[3] = 0;
    pt[2] = 0;
    line_pt[2] = 0;
    line_pt[3] = 0;
    line_dir[2] = 0;
    line_dir[3] = 0;
    double dst = 0;
    if (inside) {

      dst = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);
    } else {
      Eigen::Vector4f pseg = pt - line_pt;
      float l2 = (lnum % 2 == 0) ? v2s : v1s;
      float t = max(0.0f, min(1.0f, (float)pseg.dot(line_dir) / l2));
      line_dir = t * line_dir;
      if (t == 0) {
        dst = pcl::squaredEuclideanDistance(
            pcl::PointXY(pt[0], pt[1]), pcl::PointXY(line_pt[0], line_pt[1]));
      } else if (t == 1) {
        Eigen::Vector4f pend = line_pt + line_dir;
        dst = pcl::squaredEuclideanDistance(pcl::PointXY(pt[0], pt[1]),
                                            pcl::PointXY(pend[0], pend[1]));
      } else {
        dst = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);
      }
    }
    if (dst < mindist) {
      mindist = dst;
    }
    lnum++;
  }
  return mindist;
}

bool MyRotatedRect::isInside(Eigen::Vector4f pt) {

  for (const auto l : make_adjacent_range(lines)) {
    // dist of p from;
    Eigen::Vector4f line_pt = l.first.getVector4fMap();
    Eigen::Vector4f line_dir = l.second.getVector4fMap() - line_pt;
    Eigen::Vector4f pseg = pt - line_pt;
    float cosign = line_dir[0] * pseg[1] - line_dir[1] * pseg[0];

    if (cosign > 0)
      return false;
  }
  return true;
}

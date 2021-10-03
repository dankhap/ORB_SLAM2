#ifndef MYROTATEDRECT_H
#define MYROTATEDRECT_H

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include <opencv2/core/core.hpp>
// #include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include <pcl/point_types.h>

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
    corner = Eigen::Vector2f(cor.x, cor.y);
    vector1 = Eigen::Vector2f(vec1.x, vec1.y);
    vector2 = Eigen::Vector2f(vec2.x, vec2.y);
    v1s = vector1.norm();
    vector1.normalize();
    v2s = vector2.norm();
    vector2.normalize();
    updateLines();
  }

  void scaleDown(float eps) {
    // scale down....
    this->eps = eps;
    corner += (eps / 2) * (vector1 + vector2);
    v1s -= eps;
    v2s -= eps;
    updateLines();
  }
  bool isInside(Eigen::Vector4f pt);
  float distanceTo(Eigen::Vector4f pt, bool inside = true);
  std::vector<pcl::PointXYZ> lines;

private:
  void updateLines() {
    lines.clear();
    Eigen::Vector2f v1 = v1s * vector1;
    Eigen::Vector2f v2 = v2s * vector2;
    lines.push_back(pcl::PointXYZ(corner.x(), corner.y(), 0));
    lines.push_back(pcl::PointXYZ(corner.x() + v2.x(), corner.y() + v2.y(), 0));
    lines.push_back(pcl::PointXYZ(lines[1].x + v1.x(), lines[1].y + v1.y(), 0));
    lines.push_back(pcl::PointXYZ(lines[2].x - v2.x(), lines[2].y - v2.y(), 0));
    lines.push_back(lines[0]);
  }

  Eigen::Vector2f corner;
  Eigen::Vector2f vector2;
  Eigen::Vector2f vector1;
  float v1s, v2s;
  float eps;
};

#endif /* MYROTATEDRECT_H */

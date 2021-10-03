#ifndef ROTCALIPERS_H
#define ROTCALIPERS_H

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_types.h>

struct MinAreaState {
  int bottom;
  int left;
  float height;
  float width;
  float base_a;
  float base_b;
};

enum { CALIPERS_MAXHEIGHT = 0, CALIPERS_MINAREARECT = 1, CALIPERS_MAXDIST = 2 };

/*F///////////////////////////////////////////////////////////////////////////////////////
 //    Name:    rotatingCalipers
 //    Purpose:
 //      Rotating calipers algorithm with some applications
 //
 //    Context:
 //    Parameters:
 //      points      - convex hull vertices ( any orientation )
 //      n           - number of vertices
 //      mode        - concrete application of algorithm
 //                    can be  CV_CALIPERS_MAXDIST   or
 //                            CV_CALIPERS_MINAREARECT
 //      left, bottom, right, top - indexes of extremal points
 //      out         - output info.
 //                    In case CV_CALIPERS_MAXDIST it points to float value -
 //                    maximal height of polygon.
 //                    In case CV_CALIPERS_MINAREARECT
 //                    ((CvPoint2D32f*)out)[0] - corner
 //                    ((CvPoint2D32f*)out)[1] - vector1
 //                    ((CvPoint2D32f*)out)[2] - corner2
 //
 //                      ^
 //                      |
 //              vector2 |
 //                      |
 //                      |____________\
 //                    corner         /
 //                               vector1
 //
 //    Returns:
 //    Notes:
 //F*/

namespace cv {
/* we will use usual cartesian coordinates */
void rotatingCalipers(const Point2f *points, int n, int mode, float *out);

void minAreaRect(Point2f *hpoints, int n, Point2f *out3);
void boxPoints(RotatedRect box, OutputArray _pts);
} // namespace cv

#endif /* ROTCALIPERS_H */

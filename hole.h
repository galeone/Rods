#ifndef HOLE_H
#define HOLE_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

class Hole {
 public:
  Hole(vector<Point>& contour);
  Point2f getCenter();
  double  getDiameter();
 private:
  vector<Point> _contour;
  Point2f _center;
  double _diameter;
};

#endif  // HOLE_H

#ifndef HOLE_H
#define HOLE_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

class Hole {
 public:
  Hole(vector<Point>& contour);
  Point getCenter();
  float getDiameter();
 private:
  vector<Point> _contour;
};

#endif  // HOLE_H

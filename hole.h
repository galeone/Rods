#ifndef HOLE_H
#define HOLE_H

#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

class Hole {
 public:
  Hole(vector<Point>& contour);
  Point2i getCenter();
  double getDiameter();
  vector<Point> getContour();

 private:
  vector<Point> _contour;
  Point2i _center;
  double _diameter;
};

#endif  // HOLE_H

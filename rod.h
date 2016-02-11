#ifndef ROD_H
#define ROD_H

#include <vector>
#include <set>
#include <memory>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "hole.h"

using namespace std;
using namespace cv;

class Rod {
 public:
  Rod(vector<Point>& contour);
  char getType();
  Point getPosition();
  float getOrientation();
  float getLength();
  float getWidth();
  float getWidthAtTheBarycenter();
  vector<Hole> holes;
  vector<Point> getContour();

 private:
  vector<Point> _contour;
};

#endif  // ROD_H

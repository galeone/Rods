#ifndef ROD_H
#define ROD_H

#include <vector>
#include <set>
#include <memory>
#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "hole.h"
#include <cmath>

using namespace std;
using namespace cv;

class Rod {
 public:
  Rod(vector<Point>& contour, Size size);
  char getType();
  Point getPosition();
  double getOrientation();
  double getLength();
  double getWidth();
  double getWidthAtTheBarycenter();
  vector<Hole> holes;
  vector<Point> getContour();
  RotatedRect getMER();
  vector<Point2d> getEigenVectors();
  vector<double> getEigenValues();
  vector<Point2i> getExtremesAtTheBarycenter();

 private:
  vector<Point> _contour;
  double _angle, _width, _length;
  Point _center;
  Point2d _eigen_vecs[2];
  double _eigen_val[2];
  RotatedRect _mer;
  Size _size;
};

inline ostream& operator<<(ostream& ss, Rod& rod) {
  ss << "[!] Type: " << rod.getType() << "\n";
  ss << "[!] Baricenter: " << rod.getPosition() << "\n";
  ss << "[!] Orientation: " << rod.getOrientation() << "\n";
  ss << "[!] Width: " << rod.getWidth() << "\n";
  ss << "[!] Length: " << rod.getLength() << "\n";
  ss << "[!] Width at Baricenter: " << rod.getWidthAtTheBarycenter() << "\n";
  ss << "[!] N° of holes: " << rod.holes.size() << "\n";
  size_t i = 1;
  for (auto hole : rod.holes) {
    ss << "\tHole N°" << i++ << ": center " << hole.getCenter()
       << ", diameter: " << hole.getDiameter() << "\n";
  }
  return ss;
}

#endif  // ROD_H

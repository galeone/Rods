#include "rod.h"

Rod::Rod(vector<Point>& contour) {
  _contour = contour;
}

char Rod::getType() {
  auto holesCount = holes.size();
  if (holesCount == 1) {
    return 'A';
  }
  if (holesCount == 2) {
    return 'B';
  }
  cout << holesCount << endl;
  return 'E';
}

Point Rod::getPosition() {
  return Point(0, 0);
}

float Rod::getOrientation() {
  return 0;
}

float Rod::getLength() {
  return 0;
}

float Rod::getWidth() {
  return 0;
}

float Rod::getWidthAtTheBarycenter() {
  return 0;
}

vector<Point> Rod::getContour() {
  return _contour;
}

#include "rod.h"

Rod::Rod(vector<Point>& contour) {
  _contour = contour;

  // Construct a buffer used by the pca analysis
  int sz = static_cast<int>(_contour.size());
  Mat data_contour = Mat(sz, 2, CV_64FC1);
  for (int i = 0; i < data_contour.rows; ++i) {
    data_contour.at<double>(i, 0) = _contour[i].x;
    data_contour.at<double>(i, 1) = _contour[i].y;
  }
  // Perform PCA analysis
  PCA pca_analysis(data_contour, Mat(), CV_PCA_DATA_AS_ROW);
  // Store the cente r of the object
  _cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
  // Store the eigenvalues and eigenvectors
  for (int i = 0; i < 2; ++i) {
    _eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                             pca_analysis.eigenvectors.at<double>(i, 1));
    _eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
  }

  //_angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x) * 360/(2*CV_PI);

  _enclosingRectangle = minAreaRect(_contour);

  _width = (_enclosingRectangle.size.height < _enclosingRectangle.size.width)
               ? _enclosingRectangle.size.height
               : _enclosingRectangle.size.width;
  _length = (_enclosingRectangle.size.height > _enclosingRectangle.size.width)
                ? _enclosingRectangle.size.height
                : _enclosingRectangle.size.width;

  _angle = _enclosingRectangle.angle;
}

vector<Point2d> Rod::getEigenVectors() {
  vector<Point2d> ret;
  ret.reserve(2);
  ret.push_back(_eigen_vecs[0]);
  ret.push_back(_eigen_vecs[1]);
  return ret;
}

vector<double> Rod::getEigenValues() {
  vector<double> ret;
  ret.reserve(2);
  ret.push_back(_eigen_val[0]);
  ret.push_back(_eigen_val[1]);
  return ret;
}

char Rod::getType() {
  auto holesCount = holes.size();
  if (holesCount == 1) {
    return 'A';
  }
  if (holesCount == 2) {
    return 'B';
  }
  return 'E';
}

Point Rod::getPosition() {
  return _cntr;
}

double Rod::getOrientation() {
  return _angle;
}

double Rod::getLength() {
  return _length;
}

double Rod::getWidth() {
  return _width;
}

double Rod::getWidthAtTheBarycenter() {
  return 0;
}

vector<Point> Rod::getContour() {
  return _contour;
}

RotatedRect Rod::getMER() {
  return _enclosingRectangle;
}

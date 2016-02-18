#include "rod.h"

Rod::Rod(vector<Point>& contour, Size size) {
  _contour = contour;
  _size = size;

  // Construct a buffer used by the PCA
  int sz = static_cast<int>(_contour.size());
  Mat data_contour = Mat(sz, 2, CV_64FC1);
  for (int i = 0; i < data_contour.rows; ++i) {
    data_contour.at<double>(i, 0) = _contour[i].x;
    data_contour.at<double>(i, 1) = _contour[i].y;
  }

  // Perform PCA analysis
  PCA pca_analysis(data_contour, Mat(), CV_PCA_DATA_AS_ROW);

  // Store the center of the object
  _center = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                  static_cast<int>(pca_analysis.mean.at<double>(0, 1)));

  // Store the eigenvalues and eigenvectors
  for (int i = 0; i < 2; ++i) {
    _eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                             pca_analysis.eigenvectors.at<double>(i, 1));
    _eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
  }

  // Store the orientation wrt the horizontal axis, modulus 180
  //(measured in degree)
  _angle = atan2(_eigen_vecs[0].y, _eigen_vecs[0].x) * 180 / CV_PI;
  _angle = fmod(_angle, 180);
  _angle = fmod(180 - _angle, 180);

  // Store the MER
  _mer = minAreaRect(_contour);

  // Store MER width
  _width =
      _mer.size.height < _mer.size.width ? _mer.size.height : _mer.size.width;

  // Store MER height
  _length =
      _mer.size.height > _mer.size.width ? _mer.size.height : _mer.size.width;
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
  return 'E';  // error
}

Point Rod::getPosition() {
  return _center;
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
  vector<Point2i> e = getExtremesAtTheBarycenter();
  return sqrt(static_cast<double>((e[0].x - e[1].x) * (e[0].x - e[1].x)) +
              static_cast<double>((e[0].y - e[1].y) * (e[0].y - e[1].y)));
}

vector<Point2i> Rod::getExtremesAtTheBarycenter() {
  vector<Point2i> extremes;
  extremes.reserve(2);
  Mat1b rois = Mat1b::zeros(_size);
  vector<vector<Point>> fakeHierarchy;
  fakeHierarchy.reserve(1);
  fakeHierarchy.push_back(_contour);
  drawContours(rois, fakeHierarchy, 0, Scalar(255));

  Point2i eigenPoint(static_cast<int>(_eigen_vecs[1].x * _eigen_val[1]),
                     static_cast<int>(_eigen_vecs[1].y * _eigen_val[1]));

  for (int sign = -1, i = 0; i < 2; i++, sign += 2) {
    // p2 shows the direction along the minor axis
    float scale = 0.01;

    Point2i p2 = _center + sign * scale * eigenPoint;

    while (rois.at<uchar>(p2) == 0 &&
           rois.at<uchar>(p2 + sign * Point(0, 1)) == 0 &&
           rois.at<uchar>(p2 + sign * Point(1, 0)) == 0 &&
           rois.at<uchar>(p2 + sign * Point(1, 1)) == 0 &&
           rois.at<uchar>(p2 + sign * Point(0, -1)) == 0 &&
           rois.at<uchar>(p2 + sign * Point(-1, -1)) == 0) {
      scale += 0.01;
      p2 = _center + sign * scale * eigenPoint;
    }
    extremes.push_back(p2);
  }
  return extremes;
}

vector<Point> Rod::getContour() {
  return _contour;
}

RotatedRect Rod::getMER() {
  return _mer;
}

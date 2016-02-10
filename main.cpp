#include <iostream>
#include <vector>
#include <cstdlib>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

Mat1b read(const char* path) {
  const String name(path);
  return imread(name, IMREAD_GRAYSCALE);
}

void show(const Mat1b& image, string winName) {
  namedWindow(winName);
  imshow(winName, image);
}

void show(const Mat1b& image) {
  static int counter = 1;
  string winName = string("window " + to_string(counter));
  show(image, winName);
  ++counter;
}

double binarize(const Mat1b& input, Mat1b& output) {
  return threshold(input, output, 0, 255, THRESH_OTSU | THRESH_BINARY_INV);
}

void filter(const Mat1b& input, Mat1b& output) {
  medianBlur(input, output, 3);
  medianBlur(output, output, 3);
  medianBlur(output, output, 3);
}

void edge(const Mat1b& input, Mat1b& output, double otsuThreshold) {
  Canny(input, output, 0.5 * otsuThreshold, otsuThreshold);
}

void drawAxis(Mat& img,
              Point p,
              Point q,
              Scalar colour,
              const float scale = 0.2) {
  double angle;
  double hypotenuse;
  angle = atan2((double)p.y - q.y, (double)p.x - q.x);  // angle in radians
  hypotenuse =
      sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
  //    double degrees = angle * 180 / CV_PI; // convert radians to degrees
  //    (0-180 range)
  //    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360
  //    degrees range
  // Here we lengthen the arrow by a factor of scale
  q.x = (int)(p.x - scale * hypotenuse * cos(angle));
  q.y = (int)(p.y - scale * hypotenuse * sin(angle));
  line(img, p, q, colour, 1, CV_AA);
  // create the arrow hooks
  p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
  line(img, p, q, colour, 1, CV_AA);
  p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
  p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
  line(img, p, q, colour, 1, CV_AA);
}

double getOrientation(const vector<Point>& pts, Mat& img) {
  // Construct a buffer used by the pca analysis
  int sz = static_cast<int>(pts.size());
  Mat data_pts = Mat(sz, 2, CV_64FC1);
  for (int i = 0; i < data_pts.rows; ++i) {
    data_pts.at<double>(i, 0) = pts[i].x;
    data_pts.at<double>(i, 1) = pts[i].y;
  }
  // Perform PCA analysis
  PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
  // Store the center of the object
  Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                     static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
  // Store the eigenvalues and eigenvectors
  vector<Point2d> eigen_vecs(2);
  vector<double> eigen_val(2);
  for (int i = 0; i < 2; ++i) {
    eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                            pca_analysis.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
  }
  // Draw the principal components
  circle(img, cntr, 3, Scalar(255, 0, 255), 2);
  Point p1 = cntr +
             0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),
                          static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
  Point p2 = cntr -
             0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]),
                          static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
  drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
  drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);
  double angle =
      atan2(eigen_vecs[0].y, eigen_vecs[0].x);  // orientation in radians
  return angle;
}

#define ROD_AREA_THRESHOLD_LOW 1500
#define ROD_AREA_THRESHOLD_UP 6000

int main() {
  const char images[15][18] = {
      "./rods/TESI00.BMP", "./rods/TESI01.BMP", "./rods/TESI12.BMP",
      "./rods/TESI21.BMP", "./rods/TESI31.BMP", "./rods/Tesi33.bmp",
      "./rods/TESI44.BMP", "./rods/TESI47.BMP", "./rods/TESI48.BMP",
      "./rods/TESI49.BMP", "./rods/TESI50.BMP", "./rods/TESI51.BMP",
      "./rods/TESI90.BMP", "./rods/TESI92.BMP", "./rods/TESI98.BMP"};

  for (auto image : images) {
    cout << "Processing " << image << endl;
    Mat1b gray = read(image);
    Mat1b binary;
    double OtsuThreshold = binarize(gray, binary);
    Mat1b bw, bw_work, bw_copy;
    filter(binary, bw);
    bw.copyTo(bw_work);

    // Find all the external contours in the thresholded image
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;
    findContours(bw_work, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_NONE);

    Mat1b rois = Mat1b::zeros(bw.size());

    // define a tree to store the association
    // < root countour index, vector of child (contour index of the holes) of
    // the rod>
    map<int, vector<int>> tree;

    for (size_t i = 0; i < contours.size(); ++i) {
      // Calculate the area of each contour
      double area = contourArea(contours[i]);
      int fatherPosition = hierarchy[i][3];
      // does not have a father and its area passes the thresold
      bool isRoot = false;
      bool isLeaf = false;
      if (fatherPosition < 0) {
        isRoot = area > ROD_AREA_THRESHOLD_LOW && area < ROD_AREA_THRESHOLD_UP;
      } else {
        // has a father and its father is a rod (passes the threshold)
        auto fatherArea = contourArea(contours[fatherPosition]);
        isLeaf = fatherArea > ROD_AREA_THRESHOLD_LOW &&
                 fatherArea < ROD_AREA_THRESHOLD_UP;
      }

      if (isRoot || isLeaf) {
        cout << "[!] Shape area: " << area << "\n";
        if (isRoot) {
          tree[i] = vector<int>();
        } else {
          tree[fatherPosition].push_back(i);
        }
        // Draw each contour only for visualisation purposes
        // Find the orientation of each shape
        // getOrientation(contours[i], color);
        drawContours(rois, contours, static_cast<int>(i), Scalar(255), 1, 8);
      }
    }

    for (const auto& rod : tree) {
      cout << "[!] Rod type: ";
      if (rod.second.size() == 1) {
        cout << "A";
      } else if (rod.second.size() == 2) {
        cout << "B";
      } else {
        cout << "unknown";
        // When here, touching rods are present in the image (found more than 2
        // holes in a connected component). Therefore, we must erode the blobs
        // until a valid rod is found
        // thus repeat the findcontours ecc.
        // After we can compute the difference between the separated (erosed)
        // rods and the connected one.
        // To findout where the rods where touching and change the contours of
        // the blobs accordingly
      }
      cout << endl;
    }

    /*
    Mat element = getStructuringElement(MORPH_CROSS,Size(3,3));
    for(auto i=0;i<5;++i) {
        morphologyEx(bw, bw, MORPH_OPEN, element);
    }
    */

    // show(bw_work, string(image + string("a")));
    show(rois, string(image));
  }

  waitKey(0);
  return EXIT_SUCCESS;
}

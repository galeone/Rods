#include <iostream>
#include <vector>
#include <cstdlib>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "rod.h"

using namespace std;
using namespace cv;

Mat1b read(const char* path) {
  const String name(path);
  return imread(name, IMREAD_GRAYSCALE);
}

void show(const Mat& image, string winName) {
  namedWindow(winName);
  imshow(winName, image);
}

void show(const Mat& image) {
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

// Experimental area thresholds
#define ROD_AREA_THRESHOLD_LOW 1500
#define ROD_AREA_THRESHOLD_UP 6000

vector<Rod> buildTree(const Mat1b& bw, Mat1b& rois) {
  vector<Rod> ret;
  ret.reserve(4);
  // Find all the contours in the thresholded image
  vector<Vec4i> hierarchy;
  vector<vector<Point>> contours;

  Mat1b bw_work;
  bw.copyTo(bw_work);

  findContours(bw_work, contours, hierarchy, CV_RETR_TREE,
               CV_CHAIN_APPROX_NONE);

  rois = Mat1b::zeros(bw.size());

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
      if (isRoot) {
        tree[i] = vector<int>();
      } else {
        tree[fatherPosition].push_back(i);
      }
      // Draw each contour only for visualisation purposes
      drawContours(rois, contours, static_cast<int>(i), Scalar(255), 1, 8);
    }
  }

  for (const auto& rawRod : tree) {
    Rod rod(contours[rawRod.first], rois.size());
    for (const auto childIdx : rawRod.second) {
      Hole hole(contours[childIdx]);
      rod.holes.push_back(hole);
    }
    ret.push_back(rod);
  }
  return ret;
}

void drawAxis(Mat& img,
              Point p,
              Point q,
              Scalar colour,
              const float scale = 0.2) {
  double angle = atan2(static_cast<double>(p.y - q.y),
                       static_cast<double>(p.x - q.x));  // angle in radians

  double hypotenuse = sqrt(static_cast<double>((p.y - q.y) * (p.y - q.y) +
                                               (p.x - q.x) * (p.x - q.x)));
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

int main() {
  const char images[15][18] = {
      "./rods/TESI00.BMP", "./rods/TESI01.BMP", "./rods/TESI12.BMP",
      "./rods/TESI21.BMP", "./rods/TESI31.BMP", "./rods/Tesi33.bmp",
      "./rods/TESI44.BMP", "./rods/TESI47.BMP", "./rods/TESI48.BMP",
      "./rods/TESI49.BMP", "./rods/TESI50.BMP", "./rods/TESI51.BMP",
      "./rods/TESI90.BMP", "./rods/TESI92.BMP", "./rods/TESI98.BMP"};

  for (auto image : images) {
    if (strlen(image) == 0) {
      continue;
    }
    cout << "Image: " << image << "\n";

    // Read image in grayscale
    Mat1b gray = read(image);

    // Binaryze image
    Mat1b binary;
    binarize(gray, binary);

    // Remove noise
    Mat1b bw;
    filter(binary, bw);

    // Find connected components of not touching rods
    Mat1b rois;
    vector<Rod> tree = buildTree(bw, rois);

    // Remove every detected rods from the work image
    // thus we can focus on the research of connected rods
    // without affecting the founded ones
    Mat1b bw_work;
    bw.copyTo(bw_work);
    for (Rod& rod : tree) {
      vector<vector<Point>> fakeHierarchy;
      fakeHierarchy.push_back(rod.getContour());
      drawContours(bw_work, fakeHierarchy, 0, Scalar(0), CV_FILLED);
    }

    // Separate the rods in the image, using the watershed algorithm

    // First erode the binarized image the increase the degree of separation
    // between objects
    Mat1b bw_erosed;
    Mat dist;
    bw_work.copyTo(bw_erosed);

    // Separe touching objects
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
    morphologyEx(bw_work, bw_erosed, MORPH_ERODE, element, Point(-1, -1), 2);

    // Apply the distance transform to the erosed image to get an image
    // with a distribution of pixel in function their distance
    distanceTransform(bw_erosed, dist, CV_DIST_L2, 3);

    // Normalize the distance image between [0,1] and threshold it with a
    // threshold of 0.5. Than dilate to make the rods bigger
    normalize(dist, dist, 0, 1, NORM_MINMAX);
    show(dist);
    threshold(dist, dist, 0.5, 1, CV_THRESH_BINARY);
    show(dist);

    // Convert the thresholded distance image back to a uchar matrix
    // Invert colour and binarize the image to get object white and non-object
    // black
    Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    // Extract contours of the "objects" (the skeleton of the rods)
    // to get the seeds for the whatershed algoritm
    vector<vector<Point>> contours;
    findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if (contours.size() > 0) {
      // Create the marker image for the watershed algorithm
      Mat markers = Mat::zeros(dist.size(), CV_32SC1);

      // increase the size of every contour, along the major axis direction
      for (size_t i = 0; i < contours.size(); i++) {
        // fake rod
        Rod rod(contours[i], gray.size());
        vector<Point2d> eigen_vecs = rod.getEigenVectors();
        vector<double> eigen_val = rod.getEigenValues();

        // p1 shows the direction along the mayor axis. Is the distance scaled
        // down by a factor of 0.02
        Point center = rod.getPosition();
        Point eigenPoint =
            Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),
                  static_cast<int>(eigen_vecs[0].y * eigen_val[0]));

        Point p1 = center - 0.07 * eigenPoint, p2 = center + 0.04 * eigenPoint;
        line(dist_8u, center, p1, Scalar(255), 8);
        line(dist_8u, center, p2, Scalar(255), 8);
        morphologyEx(dist_8u,dist_8u,MORPH_DILATE,element, Point(-1,-1));
      }

      findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
      // Draw the foreground markers
      for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar::all(static_cast<int>(i) + 1);
        drawContours(markers, contours, static_cast<int>(i), color, CV_FILLED);
      }

      // Draw the background marker
      circle(markers, Point(5, 5), 3, CV_RGB(255, 255, 255), CV_FILLED);

      show(markers * 10000);

      // Perform the watershed algorithm
      // convert bw_work in a color image (required by the algorithm)
      Mat color;
      cvtColor(gray, color, CV_GRAY2BGR);
      watershed(color, markers);

      Mat mark = Mat::zeros(markers.size(), CV_8UC1);
      markers.convertTo(mark, CV_8UC1);
      bitwise_not(mark, mark);

      // Generate random colors
      vector<Vec3b> colors;
      for (size_t i = 0; i < contours.size(); i++) {
        int b = theRNG().uniform(0, 255);
        int g = theRNG().uniform(0, 255);
        int r = theRNG().uniform(0, 255);

        colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
      }

      // Create the result image
      Mat dst = Mat::zeros(markers.size(), CV_8UC3);

      // Fill labeled objects with random colors
      for (int i = 0; i < markers.rows; i++) {
        for (int j = 0; j < markers.cols; j++) {
          int index = markers.at<int>(i, j);
          if (index > 0 && index <= static_cast<int>(contours.size()))
            dst.at<Vec3b>(i, j) = colors[index - 1];
          else
            dst.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
        }
      }

      // Visualize the final image
      show(dst);
    }

    for (Rod& rod : tree) {
      cout << rod << endl;
      // Draw MER
      Point2f rect_points[4];
      rod.getMER().points(rect_points);
      for (size_t i = 0; i < 4; ++i) {
        line(rois, rect_points[i], rect_points[(i + 1) % 4], Scalar(255));
      }

      // Draw a straight line perpendicular to the major axis and passing
      // throught the barycenter
      vector<Point2d> eigen_vecs = rod.getEigenVectors();
      vector<double> eigen_val = rod.getEigenValues();

      // p1 shows the direction along the mayor axis. Is the distance scaled
      // down by a factor of 0.02
      Point center = rod.getPosition();
      Point p1 = center +
                 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]),
                              static_cast<int>(eigen_vecs[0].y * eigen_val[0]));

      auto extremes = rod.getExtremesAtTheBarycenter();
      line(rois, extremes[0], extremes[1], Scalar(255));

      // Draw the center of the holes
      for (Hole& hole : rod.holes) {
        circle(rois, hole.getCenter(), 1, Scalar(255), 2);
      }

      // Draw barycenter
      circle(rois, center, 1, Scalar(255), 2);

      // Draw major axis
      drawAxis(rois, center, p1, Scalar(255), 1);
      // drawAxis(rois, center, p2, Scalar(255), 5);
    }

    // show(rois, string(image));
  }

  waitKey(0);
  return EXIT_SUCCESS;
}

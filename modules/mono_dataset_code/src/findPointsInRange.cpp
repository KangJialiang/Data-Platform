#include "findPointsInRange.h"

#include <time.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "FOVUndistorter.h"

void findPointsInRange(cv::InputArray inputImg, cv::Mat& result,
                       std::string cameraTxtPath) {
  cv::Mat img = inputImg.getMat();

  // grid width for template image.
  int gw = 1000;
  int gh = 1000;
  // width of grid relative to marker (fac times marker size)
  float facw = 5;
  float fach = 5;

  UndistorterFOV* undistorter = new UndistorterFOV(cameraTxtPath.c_str());

  int wI = undistorter->getInputDims()[0];
  int hI = undistorter->getInputDims()[1];
  int w_out = undistorter->getOutputDims()[0];
  int h_out = undistorter->getOutputDims()[1];

  unsigned char* undistorterIn =
      img.isContinuous() ? img.data : img.clone().data;
  float* undistorterOut = new float[w_out * h_out];

  undistorter->undistort<uchar>(undistorterIn, undistorterOut, wI * hI,
                                w_out * h_out);
  cv::Mat(h_out, w_out, CV_32F, undistorterOut).convertTo(img, CV_8U, 1, 0);

  delete[] undistorterOut;

  // affine map from plane cordinates to grid coordinates.
  Eigen::Matrix3f K_p2idx = Eigen::Matrix3f::Identity();
  K_p2idx(0, 0) = gw / facw;
  K_p2idx(1, 1) = gh / fach;
  K_p2idx(0, 2) = gw / 2;
  K_p2idx(1, 2) = gh / 2;
  Eigen::Matrix3f K_p2idx_inverse = K_p2idx.inverse();

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::Ptr<cv::aruco::Dictionary> arucoDict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

  cv::aruco::detectMarkers(img, arucoDict, markerCorners, markerIds);
  if (markerCorners.size() != 1) return;  // do nothing

  std::vector<cv::Point2f> ptsI, ptsP;
  ptsI = markerCorners[0];
  ptsP.push_back(cv::Point2f(-0.5, 0.5));
  ptsP.push_back(cv::Point2f(0.5, 0.5));
  ptsP.push_back(cv::Point2f(0.5, -0.5));
  ptsP.push_back(cv::Point2f(-0.5, -0.5));

  cv::Mat Hcv = cv::findHomography(ptsP, ptsI);
  Eigen::Matrix3f H;
  H(0, 0) = Hcv.at<double>(0, 0);
  H(0, 1) = Hcv.at<double>(0, 1);
  H(0, 2) = Hcv.at<double>(0, 2);
  H(1, 0) = Hcv.at<double>(1, 0);
  H(1, 1) = Hcv.at<double>(1, 1);
  H(1, 2) = Hcv.at<double>(1, 2);
  H(2, 0) = Hcv.at<double>(2, 0);
  H(2, 1) = Hcv.at<double>(2, 1);
  H(2, 2) = Hcv.at<double>(2, 2);

  Eigen::Matrix3f HK = H * K_p2idx_inverse;

  float* plane2imgX = new float[gw * gh];
  float* plane2imgY = new float[gw * gh];

  int idx = 0;
  for (int y = 0; y < gh; y++)
    for (int x = 0; x < gw; x++) {
      Eigen::Vector3f pp = HK * Eigen::Vector3f(x, y, 1);
      plane2imgX[idx] = pp[0] / pp[2];
      plane2imgY[idx] = pp[1] / pp[2];
      idx++;
    }

  undistorter->distortCoordinates(plane2imgX, plane2imgY, gw * gh);

  cv::Mat dbgImg = inputImg.getMat();

  int x = 0;
  int y = 0;
  int step = gw / 100;

  std::vector<cv::Point> contours;

  while (y < gh - 1) {
    int idxS = x + y * gw;

    int u_dS = plane2imgX[idxS] + 0.5;
    int v_dS = plane2imgY[idxS] + 0.5;

    contours.push_back(cv::Point(u_dS, v_dS));
    y + step < gh ? y += step : y = gh - 1;
  }
  while (x < gw - 1) {
    int idxS = x + y * gw;

    int u_dS = plane2imgX[idxS] + 0.5;
    int v_dS = plane2imgY[idxS] + 0.5;

    contours.push_back(cv::Point(u_dS, v_dS));
    x + step < gw ? x += step : x = gw - 1;
  }
  while (y > 0) {
    int idxS = x + y * gw;

    int u_dS = plane2imgX[idxS] + 0.5;
    int v_dS = plane2imgY[idxS] + 0.5;

    contours.push_back(cv::Point(u_dS, v_dS));
    y - step > 0 ? y -= step : y = 0;
  }
  while (x > 0) {
    int idxS = x + y * gw;

    int u_dS = plane2imgX[idxS] + 0.5;
    int v_dS = plane2imgY[idxS] + 0.5;

    contours.push_back(cv::Point(u_dS, v_dS));
    x - step < gw ? x -= step : x = 0;
  }

  // std::clock_t last = std::clock();

  cv::Mat inRange = cv::Mat::zeros(result.size(), CV_8U);
  cv::fillConvexPoly(inRange, contours, cv::Scalar(255 / 20));
  result += inRange;

  // std::cout << (double)(std::clock() - last) / CLOCKS_PER_SEC << "s"
  //           << std::endl;
  // last = std::clock();
  // exit(0);

  delete undistorter;
  delete[] plane2imgX;
  delete[] plane2imgY;
}
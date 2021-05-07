#ifndef RSCAMERA_H
#define RSCAMERA_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class RsCamera {
 public:
  RsCamera();
  RsCamera(int index, std::string pathToCameraTxt, int frameRate = 0);
  ~RsCamera();

  int getMinExposure();
  int getMaxExposure();
  cv::Mat getFrame();
  cv::Mat getFrame(int exposureTime);
  cv::Mat getFrame(int exposureTime, int& timeOfArrival);

 private:
  void setExposureTime(rs2::sensor sensor, int exposureTime);
  rs2::frame getRawFrame();

  int minExposureTime, maxExposureTime;
  int imgHeight, imgWidth;
  int frameRate;
  int rsCameraIndex;
  rs2::pipeline rsPipe;  // a pipeline which abstracts the device
  rs2::sensor rsSensorSelected;
};

#endif
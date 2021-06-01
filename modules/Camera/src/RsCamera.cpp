#include "RsCamera.h"

#include <exception>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

void RsCamera::setExposureTime(rs2::sensor sensor, int exposureTime) {
  // set and (check) exposure time
  int rsActureExposureTime;
  do {
    if (exposureTime < this->minExposureTime ||
        exposureTime > this->maxExposureTime)
      throw std::invalid_argument("Invalid exposure time!");
    sensor.set_option(RS2_OPTION_EXPOSURE, exposureTime);
    rs2::frame rsTmpFrame = getRawFrame();
    rsActureExposureTime =
        rsTmpFrame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
  } while (exposureTime != rsActureExposureTime);
}

rs2::frame RsCamera::getRawFrame() {
  rs2::frameset rsFrameSet = this->rsPipe.wait_for_frames();
  rs2::frame rsFrame;

  if (rsCameraIndex == 0)
    rsFrame = rsFrameSet.get_infrared_frame(1);  // attention
  else if (rsCameraIndex == 1)
    rsFrame = rsFrameSet.get_infrared_frame(2);  // attention
  else if (rsCameraIndex == 2)
    rsFrame = rsFrameSet.get_color_frame();

  return rsFrame;
}
RsCamera::RsCamera() {}
RsCamera::RsCamera(int index, std::string pathToCameraTxt, int frameRate) {
  /* 0 means left infrared, 1 means right infrared*/

  this->frameRate = frameRate;

  std::ifstream cameraTxtFile(pathToCameraTxt);
  if (!cameraTxtFile.good())
    throw std::invalid_argument("Failed to read camera.txt!");

  std::string hightWidthLine;  // that's the second line
  std::getline(cameraTxtFile, hightWidthLine);
  std::getline(cameraTxtFile, hightWidthLine);
  if (std::sscanf(hightWidthLine.c_str(), "%d %d", &this->imgWidth,
                  &this->imgHeight) != 2)
    throw std::invalid_argument("Failed to read image width and height!");

  // create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config rsConf;

  // Add desired streams to configuration
  //  rsConf.enable_stream(RS2_STREAM_COLOR, this->imgWidth, this->imgHeight,
  //                       RS2_FORMAT_BGR8, this->frameRate);
  rsConf.enable_stream(RS2_STREAM_INFRARED, 1, this->imgWidth, this->imgHeight,
                       RS2_FORMAT_Y8, this->frameRate);
  rsConf.enable_stream(RS2_STREAM_INFRARED, 2, this->imgWidth, this->imgHeight,
                       RS2_FORMAT_Y8, this->frameRate);

  // Instruct pipeline to start streaming with the requested configuration
  rs2::pipeline_profile rsPipeProf = this->rsPipe.start(rsConf);

  // disable emitter
  rs2::device rsDevices = rsPipeProf.get_device();
  auto depth_sensor = rsDevices.first<rs2::depth_sensor>();
  if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

  // select sensor
  std::vector<rs2::sensor> rsSensors = rsDevices.query_sensors();
  rs2::sensor rsInfraredSensor = rsSensors[0];
  this->rsSensorSelected = rsInfraredSensor;
  //  rs2::sensor rsColorSensor = rsSensors[1];

  //  if (index == 0) {
  //    this->rsSensorSelected = rsInfraredSensor;
  //    this->rsCameraIndex = index;
  //  } else if (index == 1) {
  //    this->rsSensorSelected = rsInf;
  //    this->rsCameraIndex = index;
  //  } else
  //    throw std::invalid_argument("Invalid camera index!");

  if (this->rsSensorSelected.supports(RS2_OPTION_EXPOSURE)) {
    auto range = this->rsSensorSelected.get_option_range(RS2_OPTION_EXPOSURE);
    this->minExposureTime = range.min;
    this->maxExposureTime = range.max;
  } else
    throw std::runtime_error("Do not support variable exposure time");
}

RsCamera::~RsCamera() {}

cv::Mat RsCamera::getFrame(int exposureTime) {
  this->setExposureTime(rsSensorSelected, exposureTime);

  return this->getFrame();
}

cv::Mat RsCamera::getFrame(int exposureTime, int& timeOfArrival) {
  this->setExposureTime(rsSensorSelected, exposureTime);

  rs2::frame rsFrame = this->getRawFrame();
  cv::Mat frameMat(cv::Size(this->imgWidth, this->imgHeight), CV_8U,
                   (void*)rsFrame.get_data(), cv::Mat::AUTO_STEP);
  timeOfArrival =
      rsFrame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);

  return frameMat.clone();
}

cv::Mat RsCamera::getFrame() {
  rs2::frame rsFrame = this->getRawFrame();
  cv::Mat frameMat(cv::Size(this->imgWidth, this->imgHeight), CV_8U,
                   (void*)rsFrame.get_data(), cv::Mat::AUTO_STEP);

  return frameMat.clone();
}

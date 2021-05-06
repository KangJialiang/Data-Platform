#include "Camera.h"

#include <exception>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

void Camera::setExposureTime(rs2::sensor sensor, int exposureTime) {
  if (exposureTime < minExposureTime || exposureTime > maxExposureTime)
    throw std::invalid_argument("Invalid exposure time!");
  sensor.set_option(RS2_OPTION_EXPOSURE, exposureTime);
}

rs2::frame Camera::getRawFrame() {
  rs2::frameset rsFrameSet = rsPipe.wait_for_frames();
  rs2::frame rsFrame = rsFrameSet.get_color_frame();

  if (rsCameraIndex == 0)
    rsFrame = rsFrameSet.get_infrared_frame(1);  // attention
  else if (rsCameraIndex == 1)
    rsFrame = rsFrameSet.get_infrared_frame(2);  // attention
  else if (rsCameraIndex == 2)
    rsFrame = rsFrameSet.get_color_frame();

  return rsFrame;
}

Camera::Camera(int index, std::string pathToCameraTxt, int framerate) {
  /* 0 means left infrared, 1 means right infrared*/

  std::ifstream cameraTxtFile(pathToCameraTxt);
  if (!cameraTxtFile.good())
    throw std::invalid_argument("Failed to read camera.txt!");

  std::string hightWidthLine;  // that's the second line
  std::getline(cameraTxtFile, hightWidthLine);
  std::getline(cameraTxtFile, hightWidthLine);
  if (std::sscanf(hightWidthLine.c_str(), "%d %d", &imgWidth, &imgHeight) != 2)
    throw std::invalid_argument("Failed to read image width and height!");

  // create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config rsConf;

  // Add desired streams to configuration
  rsConf.enable_stream(RS2_STREAM_COLOR, imgWidth, imgHeight, RS2_FORMAT_BGR8,
                       framerate);
  rsConf.enable_stream(RS2_STREAM_INFRARED, 1, imgWidth, imgHeight,
                       RS2_FORMAT_Y8, framerate);
  rsConf.enable_stream(RS2_STREAM_INFRARED, 2, imgWidth, imgHeight,
                       RS2_FORMAT_Y8, framerate);

  // Instruct pipeline to start streaming with the requested configuration
  rs2::pipeline_profile rsPipeProf = rsPipe.start(rsConf);

  // disable emitter
  rs2::device rsDevices = rsPipeProf.get_device();
  auto depth_sensor = rsDevices.first<rs2::depth_sensor>();
  if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

  // select sensor
  std::vector<rs2::sensor> rsSensors = rsDevices.query_sensors();
  rs2::sensor rsInfraredSensor = rsSensors[0];
  rs2::sensor rsColorSensor = rsSensors[1];

  if (index == 0) {
    rsSensorSelected = rsInfraredSensor;
    rsCameraIndex = index;
  } else if (index == 1) {
    rsSensorSelected = rsInfraredSensor;
    rsCameraIndex = index;
  } else
    throw std::invalid_argument("Invalid camera index!");

  if (rsSensorSelected.supports(RS2_OPTION_EXPOSURE)) {
    auto range = rsSensorSelected.get_option_range(RS2_OPTION_EXPOSURE);
    minExposureTime = range.min, maxExposureTime = range.max;
  } else
    throw std::runtime_error("Do not support variable exposure time");
}

Camera::~Camera() {}

cv::Mat Camera::getFrame(int exposureTime) {
  int rsActureExposureTime;
  // set and (check) exposure time
  do {
    setExposureTime(rsSensorSelected, exposureTime);
    rs2::frame rsTmpFrame = getRawFrame();
    rsActureExposureTime =
        rsTmpFrame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
  } while (exposureTime != rsActureExposureTime);

  return getFrame();
}

cv::Mat Camera::getFrame() {
  rs2::frame rsFrame = getRawFrame();
  cv::Mat frameMat(cv::Size(imgWidth, imgHeight), CV_8UC3,
                   (void*)rsFrame.get_data(), cv::Mat::AUTO_STEP);

  return frameMat.clone();
}
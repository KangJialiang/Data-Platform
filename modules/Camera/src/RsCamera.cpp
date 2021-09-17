#include "RsCamera.h"

#include <algorithm>
#include <exception>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

RsCamera::RsCamera() {
  rs2::context ctx;  // Obtain a list of devices currently present on the system

  rs2::device_list devices_list = ctx.query_devices();
  size_t device_count = devices_list.size();
  if (!device_count)
    throw std::runtime_error("No device detected. Is it plugged in?");
  else if (device_count > 1)
    throw std::runtime_error("More than one devices detected.");
  rs2::device device;
  for (size_t i = 0; i < device_count; i++) {
    try {
      device = devices_list[0];
    } catch (const std::exception &e) {
      throw std::runtime_error("Could not create device - " +
                               std::string(e.what()));
    } catch (...) {
      throw std::runtime_error("Failed to created device.");
    }
  }
  deviceName = device.get_info(RS2_CAMERA_INFO_NAME);
  serialNumber = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  firmwareVersion = device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);

  for (rs2::sensor sensor : device.query_sensors())
    for (rs2::stream_profile profile : sensor.get_stream_profiles())
      if (profile.stream_type() == RS2_STREAM_INFRARED &&
          profile.format() == RS2_FORMAT_Y8)
      // read 8-bit per-pixel grayscale image only
      {
        auto video = profile.as<rs2::video_stream_profile>();

        streamNames.insert(profile.stream_name());
        streamNameProfMap.insert({profile.stream_name(), profile});

        if (sensor.supports(RS2_OPTION_EXPOSURE)) {
          int minExpo, maxExpo;
          minExpo = sensor.get_option_range(RS2_OPTION_EXPOSURE).min;
          maxExpo = sensor.get_option_range(RS2_OPTION_EXPOSURE).max;
          std::pair<int, int> minMaxExpo{minExpo, maxExpo};
          sensorMinMaxExposureMap.insert({sensor, minMaxExpo});
        } else
          throw std::runtime_error("Do not support variable exposure time.");

        try {
          rs2_intrinsics rsIntrinsics =
              video.get_intrinsics();  // may throw an exception
          if (rsIntrinsics.model == RS2_DISTORTION_BROWN_CONRADY ||
              rsIntrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY ||
              rsIntrinsics.model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY) {
            intrinsicT intrinsics{rsIntrinsics.fx, rsIntrinsics.fy,
                                  rsIntrinsics.ppx, rsIntrinsics.ppy};
            std::copy(std::begin(rsIntrinsics.coeffs),
                      std::end(rsIntrinsics.coeffs), intrinsics.coeffs);
            profIntrinsicMap.insert({profile, intrinsics});
          }
        } catch (...) {
          throw std::runtime_error("Failed to get intrinsics.");
        }
      }
}

std::vector<rs2::stream_profile> RsCamera::getRawProfiles(
    const std::string &streamName) {
  std::vector<rs2::stream_profile> result;
  for (auto beg = streamNameProfMap.lower_bound(streamName),
            end = streamNameProfMap.upper_bound(streamName);
       beg != end; beg++)
    result.push_back(beg->second);
  return result;
}

void RsCamera::selectRawProfile(rs2::stream_profile profile) {
  rs2::config conf;
  profSelected = profile;

  // Add desired streams to configuration
  conf.enable_stream(profile.stream_type(), profile.stream_index(),
                     profile.as<rs2::video_stream_profile>().width(),
                     profile.as<rs2::video_stream_profile>().height(),
                     profile.format(), profile.fps());

  // Instruct pipeline to start streaming with the requested configuration
  rs2::pipeline_profile pipeProf = this->pipe.start(conf);

  // disable emitter
  rs2::device devices = pipeProf.get_device();
  auto depth_sensor = devices.first<rs2::depth_sensor>();
  if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

  // sensor will change, can only be determined now
  std::vector<rs2::sensor> sensors = devices.query_sensors();
  for (auto sensor : sensors)
    for (auto profile : sensor.get_stream_profiles())
      if (profSelected == profile) {
        sensorSelected = sensor;
        break;
      }
}

std::string RsCamera::parseProf(const rs2::stream_profile &profile) {
  int width = profile.as<rs2::video_stream_profile>().width();
  int height = profile.as<rs2::video_stream_profile>().height();
  int FPS = profile.fps();
  return std::to_string(width) + "x" + std::to_string(height) + " " +
         std::to_string(FPS) + "Hz";
}

rs2::frame RsCamera::getRawFrame() {
  rs2::frameset rsFrameSet = this->pipe.wait_for_frames();
  rs2::frame rsFrame;

  if (profSelected.stream_type() == RS2_STREAM_INFRARED)
    rsFrame = rsFrameSet.get_infrared_frame(profSelected.stream_index());
  else if ((profSelected.stream_type() == RS2_STREAM_COLOR))
    rsFrame = rsFrameSet.get_color_frame();

  return rsFrame;
}

void RsCamera::selectProfile(const std::string &streamName,
                             const std::string &resFPS) {
  for (const auto &streamNameProf : streamNameProfMap) {
    auto profile = streamNameProf.second;
    if (streamName == profile.stream_name() && resFPS == parseProf(profile)) {
      selectRawProfile(profile);
      break;
    }
  }
}

void RsCamera::setExposureTime(rs2::sensor sensor, int exposureTime) {
  // set and (check) exposure time
  int rsActureExposureTime;
  do {
    if (exposureTime < getMinExposure() || exposureTime > getMaxExposure())
      throw std::invalid_argument("Invalid exposure time!");
    sensor.set_option(RS2_OPTION_EXPOSURE, exposureTime);
    rs2::frame rsTmpFrame = getRawFrame();
    rsActureExposureTime =
        rsTmpFrame.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
  } while (exposureTime != rsActureExposureTime);
}

int RsCamera::getMinExposure() {
  return sensorMinMaxExposureMap.find(sensorSelected)->second.first;
}

int RsCamera::getMaxExposure() {
  return sensorMinMaxExposureMap.find(sensorSelected)->second.second;
}

cv::Mat RsCamera::getFrame(int exposureTime) {
  this->setExposureTime(sensorSelected, exposureTime);

  return this->getFrame();
}

cv::Mat RsCamera::getFrame(int exposureTime, int &timeOfArrival) {
  this->setExposureTime(sensorSelected, exposureTime);

  rs2::frame rsFrame = this->getRawFrame();
  cv::Mat frameMat(
      cv::Size(profSelected.as<rs2::video_stream_profile>().width(),
               profSelected.as<rs2::video_stream_profile>().height()),
      CV_8U, (void *)rsFrame.get_data(), cv::Mat::AUTO_STEP);
  timeOfArrival =
      rsFrame.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);

  return frameMat.clone();
}

cv::Mat RsCamera::getFrame() {
  rs2::frame rsFrame = this->getRawFrame();
  cv::Mat frameMat(
      cv::Size(profSelected.as<rs2::video_stream_profile>().width(),
               profSelected.as<rs2::video_stream_profile>().height()),
      CV_8U, (void *)rsFrame.get_data(), cv::Mat::AUTO_STEP);

  return frameMat.clone();
}

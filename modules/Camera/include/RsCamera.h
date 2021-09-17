#ifndef RSCAMERA_H
#define RSCAMERA_H

#include <array>
#include <librealsense2/rs.hpp>
#include <map>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <utility>

class RsCamera {
 public:
  typedef std::array<float, 9>
      intrinsicT;  // fx, fy, cx, cy, k1, k2, p1, p2, k3

 public:
  RsCamera();
  // RsCamera(int index);
  RsCamera(int index, std::string pathToCameraTxt, int frameRate = 0){};
  ~RsCamera() = default;

  int getMinExposure();
  int getMaxExposure();

  cv::Mat getFrame();
  cv::Mat getFrame(int exposureTime);
  cv::Mat getFrame(int exposureTime, int& timeOfArrival);

  std::set<std::string> getSupportedStreamNames() { return streamNames; }
  std::set<std::string> getProfiles(const std::string& streamName) {
    std::set<std::string> result;
    for (const auto& rawProfile : getRawProfiles(streamName))
      result.insert(parseProf(rawProfile));
    return result;
  }
  void selectProfile(const std::string& streamName, const std::string& resFPS);
  void getResolution(int& width, int& height) {
    width = profSelected.as<rs2::video_stream_profile>().width();
    height = profSelected.as<rs2::video_stream_profile>().height();
  }
  /**
   * fx, fy, cx, cy, k1, k2, p1, p2, k3
   */
  intrinsicT getIntrinsic() {
    return profIntrinsicMap.find(profSelected)->second;
  }

 private:
  std::string deviceName, serialNumber, firmwareVersion;
  rs2::pipeline pipe;
  rs2::stream_profile profSelected;
  rs2::sensor sensorSelected;
  std::set<std::string> streamNames;
  std::map<rs2::sensor, std::pair<int, int>> sensorMinMaxExposureMap;
  std::multimap<std::string, rs2::stream_profile> streamNameProfMap;
  std::map<rs2::stream_profile, intrinsicT> profIntrinsicMap;

  void setExposureTime(rs2::sensor sensor, int exposureTime);
  rs2::frame getRawFrame();
  std::vector<rs2::stream_profile> getRawProfiles(
      const std::string& streamName);
  void selectRawProfile(rs2::stream_profile profile);
  std::string parseProf(const rs2::stream_profile& profile);
};

#endif
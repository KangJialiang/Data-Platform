#ifndef RSCAMERA_H
#define RSCAMERA_H

#include <array>
#include <librealsense2/rs.hpp>
#include <map>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <utility>

#include "Camera.h"

class RsCamera : public Camera {
  //  public:
  //   typedef struct {
  //     float fx;
  //     float fy;
  //     float cx;
  //     float cy;
  //     float coeffs[5];  // Order for Brown-Conrady: [k1, k2, p1, p2, k3].
  //   } intrinsicT;

 public:
  RsCamera();
  ~RsCamera() = default;

  int getMinExposure();
  int getMaxExposure();

  // do not set exposureTime to enable auto exposure
  cv::Mat getFrame() override;
  cv::Mat getFrame(int exposureTime) override;
  cv::Mat getFrame(int exposureTime, long long& timeOfArrival) override;

  std::set<std::string> getSupportedStreamNames() { return streamNames; }
  std::set<std::string> getProfiles(const std::string& streamName) {
    std::set<std::string> result;
    for (const auto& rawProfile : getRawProfiles(streamName))
      result.insert(parseProf(rawProfile));
    return result;
  }
  void selectProfile(const std::string& streamName, const std::string& resFPS);

  void getResolution(int& width, int& height) override;

  intrinsicT getIntrinsic() override;

 private:
  std::string deviceName, serialNumber, firmwareVersion;
  rs2::pipeline pipe;
  rs2::stream_profile profSelected;
  rs2::sensor sensorSelected;
  std::set<std::string> streamNames;
  std::map<rs2::sensor, std::pair<int, int>> sensorMinMaxExposureMap;
  std::multimap<std::string, rs2::stream_profile> streamNameProfMap;
  std::map<rs2::stream_profile, intrinsicT> profIntrinsicMap;

  rs2::frame getRawFrame(int exposureTime);
  std::vector<rs2::stream_profile> getRawProfiles(
      const std::string& streamName);
  void selectRawProfile(rs2::stream_profile profile);
  std::string parseProf(const rs2::stream_profile& profile);
};

#endif
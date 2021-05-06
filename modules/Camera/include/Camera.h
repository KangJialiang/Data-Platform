#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class Camera {
 public:
  Camera(int index, std::string pathToCameraTxt, int framerate = 0);
  ~Camera();

  int getMinExposure();
  int getMaxExposure();
  cv::Mat getFrame();
  cv::Mat getFrame(int exposureTime);

 private:
  void setExposureTime(rs2::sensor sensor, int exposureTime);

  int minExposureTime, maxExposureTime;
  int imgHeight, imgWidth;

  rs2::pipeline rsPipe;  // a pipeline which abstracts the device
  rs2::sensor rsSensorSelected;
  int rsCameraIndex;
};
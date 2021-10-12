#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <QImage>
#include <QMainWindow>
#include <memory>
#include <opencv2/opencv.hpp>

#include "RsCamera.h"
#include "USBCamera.h"
#include "calibrator.h"
#include "data_reader.h"
#include "imageviewer.h"
#include "json.hpp"
#include "pointcloudviewer.h"
#include "tfwindow.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

constexpr double PI = 3.141592653589793238462;

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 public slots:
  void updateWithTransformation(Eigen::Matrix4d tf);
  void tfwindowClose();

 protected:
  void closeEvent(QCloseEvent *);

 private slots:
  void on_nextButtonP4_clicked();

 private slots:
  void on_mainStartLidarCalibButton_clicked();

 private:
  Ui::MainWindow *ui;

  QPixmap cvMat2QPixmap(cv::Mat &inMat);

  std::unique_ptr<Camera> cameraP;

  bool stopResponseData;
  bool stopVignetteData;
  bool stopResponseCalib;
  bool stopVignetteCalib;

  ros::AsyncSpinner rosSpinner;
  ros::NodeHandle nh;
  ros::Subscriber subPointCloud;
  void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg);
  // QTimer *rosTimer;
  pcl::PointCloud<pcl::PointXYZI> pointCloud;

  struct SensorData {
    bool img_good;
    bool pc_good;
    uint32_t id;
    uint32_t pid;
    std::shared_ptr<cv::Mat> img;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc;
    std::shared_ptr<cv::Mat> img_marked;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_marked;
    std::shared_ptr<cv::Mat> img_proj;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_proj;

    SensorData(uint32_t index)
        : img_good(false),
          pc_good(false),
          id(index),
          img(nullptr),
          pc(nullptr),
          img_marked(nullptr),
          pc_marked(nullptr),
          img_proj(nullptr),
          pc_proj(nullptr) {}
    bool good() { return (img_good && pc_good); }
  };

  QString config_path_;
  nlohmann::json js_;

  bool is_calibrated_;
  uint32_t sid_;
  QString data_root_;

  std::shared_ptr<cv::Mat> img_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_;

  std::unique_ptr<DataReader> data_reader_;
  std::unique_ptr<PointcloudViewer> pc_viewer_;
  std::unique_ptr<ImageViewer> img_viewer_;
  std::unique_ptr<TFwindow> tfwindow_;
  std::unique_ptr<lqh::Calibrator> calibrator_;
  std::vector<SensorData> sensor_data_;

  void readConfig();
  void updateLabels();
  bool processData(bool is_check = true);
  void setEnabledAll(bool status);
  void showCalibrateResult();
  void showTFWindow();
  void tfProcess();
  void closeImgAndPcViewers();

 private slots:
  void on_cameraComboBox_currentIndexChanged(const QString &);

  void on_streamNameBoxPmain_currentIndexChanged(const QString &);

  void on_savePathChooseButton_clicked();

  void on_mainStartButton_clicked();

  void on_nextButtonP1_clicked();

  void on_nextButtonP2_clicked();

  void on_nextButtonP3_clicked();

  void on_startButtonP2_clicked();

  void on_startButtonP4_clicked();

  // void on_pathToCameraLineP1_editingFinished();

  // void on_pathToCameraLineP3_editingFinished();

  void on_startButtonP1_clicked();

  void on_maxExposureSliderP1_valueChanged(int value);

  void on_minExposureSliderP1_valueChanged(int value);

  void on_exposureSliderP3_valueChanged(int value);

  void on_startButtonP3_clicked();

  void on_Open_Config_Button_clicked();

  void on_Open_Dataset_Button_clicked();
  void on_Set_K_Button_clicked();
  void on_Set_D_Button_clicked();
  void on_Save_Result_Button_clicked();
  void on_Save_Config_Button_clicked();

  void on_next_pose_clicked();
  void on_quick_next_pose_clicked();
  void on_delete_pose_clicked();
  void on_calibrate_clicked();

  void on_pick_points_start_clicked();
  void on_pick_points_end_clicked();
  void on_pick_points_quit_clicked();
  void processSlider();

  void on_tabWidget_currentChanged(int index);

  // void on_openConfigButton_clicked();

  void on_refreshButton_clicked();

  void on_saveButton_clicked();

  void on_finishButton_clicked();

  void settingFinished();
};
#endif  // MAINWINDOW_H

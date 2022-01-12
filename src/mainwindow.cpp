#include "mainwindow.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/PointCloud2.h>

#include <QAction>
#include <QCloseEvent>
#include <QCompleter>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QStringList>
#include <QThread>
#include <QTimer>
#include <algorithm>
#include <fstream>
#include <memory>
#include <string>

#include "findPointsInRange.h"
#include "responseCalib.h"
#include "ui_mainwindow.h"
#include "vignetteCalib.h"

QPixmap MainWindow::cvMat2QPixmap(cv::Mat& inMat) {
  cv::Mat rgb;
  QImage img;
  if (inMat.channels() == 3) {
    cvtColor(inMat, rgb, cv::COLOR_BGR2RGB);
    img = QImage((const uchar*)(rgb.data), rgb.cols, rgb.rows,
                 rgb.cols * rgb.channels(), QImage::Format_RGB888);
  } else {
    img = QImage((const uchar*)(inMat.data), inMat.cols, inMat.rows,
                 inMat.cols * inMat.channels(), QImage::Format_Indexed8);
  }
  return QPixmap::fromImage(img);
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),

      rosSpinner(1),  // use 1 thread

      sid_(INT_MAX),
      is_calibrated_(false),
      data_reader_(nullptr),
      img_(nullptr),
      pc_(nullptr) {
  ui->setupUi(this);
  this->resize(QSize(1000, 800));

  rosSpinner.start();
  // subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(
  //    "lslidar_point_cloud", 2, &MainWindow::pointCloudHandler, this);
  // rosTimer = new QTimer(this);
  // rosTimer->setInterval(20);  // 单位毫秒
  // connect(rosTimer, &QTimer::timeout, this, &ros::spinOnce);
  // rosTimer->start();

  ui->tabWidget->setCurrentWidget(ui->mainTab);  // always show mainTab first

  // ui->leftorRight->setVisible(false);
  // ui->leftorRightComboBox->setVisible(false);
  ui->rsParamsBoxPmain->setVisible(false);
  ui->usbParamsBoxPmain->setVisible(false);
}

MainWindow::~MainWindow() {
  stopResponseData = true;
  stopVignetteData = true;
  stopResponseCalib = true;
  stopVignetteCalib = true;

  rosSpinner.stop();
  system("killall -9 roscore && killall -9 rosmaster");  // stop roscore

  delete ui;
}

void MainWindow::on_savePathChooseButton_clicked() {
  QString directory = QFileDialog::getExistingDirectory(this, tr("Save Path"),
                                                        QDir::homePath());
  if (!directory.isEmpty()) {
    ui->savePathLine->setText(directory);
  }
}

// void MainWindow::on_cameraPathChooseButton_clicked() {
//   QString pathToCamera = QFileDialog::getOpenFileName(
//       this, tr("Path to Camera Parameters"), QDir::homePath(),
//       tr("Plain text(*.txt);;All files(*.*)"));
//   if (!pathToCamera.isEmpty()) {
//     ui->pathToCameraLine->setText(pathToCamera);
//   }
// }

void MainWindow::on_cameraComboBox_currentIndexChanged(
    const QString& camSelected) {
  if (camSelected == tr("realsense")) {
    try {
      RsCamera* tempRsCameraP(new RsCamera());
      cameraP.reset(tempRsCameraP);

      ui->rsParamsBoxPmain->setVisible(true);
      ui->streamNameBoxPmain->clear();
      auto streamNames = tempRsCameraP->getSupportedStreamNames();
      for (auto streamName : streamNames)
        ui->streamNameBoxPmain->addItem(QString::fromStdString(streamName));
      ui->usbParamsBoxPmain->setVisible(false);
      ui->mainStartButton->setEnabled(true);

    } catch (const std::exception& e) {
      QMessageBox::warning(this, tr("Error"), tr(e.what()));
    }
  } else if (camSelected == tr("usb")) {
    USBCamera* tempUSBCameraP(new USBCamera());
    cameraP.reset(tempUSBCameraP);

    ui->usbParamsBoxPmain->setVisible(true);
    ui->rsParamsBoxPmain->setVisible(false);
    ui->mainStartButton->setEnabled(false);
    cameraP.reset(tempUSBCameraP);

  } else {
    ui->mainStartButton->setEnabled(true);
    ui->rsParamsBoxPmain->setVisible(false);
    ui->usbParamsBoxPmain->setVisible(false);
  }
}

void MainWindow::on_streamNameBoxPmain_currentIndexChanged(
    const QString& streamNameSelected) {
  ui->resFPSBoxPmain->clear();
  auto profiles = dynamic_cast<RsCamera*>(cameraP.get())
                      ->getProfiles(streamNameSelected.toStdString());
  for (const auto& profile : profiles)
    ui->resFPSBoxPmain->addItem(QString::fromStdString(profile));
}

void MainWindow::settingFinished() {
  int width, height;
  Camera* tempCameraP = cameraP.get();
  RsCamera* tempRsCameraP = dynamic_cast<RsCamera*>(tempCameraP);
  USBCamera* tempUSBCameraP = dynamic_cast<USBCamera*>(tempCameraP);

  if (tempRsCameraP) {
    tempRsCameraP->selectProfile(
        ui->streamNameBoxPmain->currentText().toStdString(),
        ui->resFPSBoxPmain->currentText().toStdString());

    int minExp = tempRsCameraP->getMinExposure();
    int maxExp = tempRsCameraP->getMaxExposure();
    ui->maxExposureSliderP1->setRange(minExp, maxExp);
    ui->maxExposureBoxP1->setRange(minExp, maxExp);
    ui->minExposureSliderP1->setRange(minExp, maxExp);
    ui->minExposureBoxP1->setRange(minExp, maxExp);
    ui->exposureSliderP3->setRange(minExp, maxExp);
    ui->exposureBoxP3->setRange(minExp, maxExp);

  } else if (tempUSBCameraP) {
    float fx = std::stof(ui->fxLinePmain->text().toStdString());
    float fy = std::stof(ui->fyLinePmain->text().toStdString());
    float cx = std::stof(ui->cxLinePmain->text().toStdString());
    float cy = std::stof(ui->cyLinePmain->text().toStdString());
    float k1 = std::stof(ui->k1LinePmain->text().toStdString());
    float k2 = std::stof(ui->k2LinePmain->text().toStdString());
    float k3 = std::stof(ui->k3LinePmain->text().toStdString());
    float p1 = std::stof(ui->p1LinePmain->text().toStdString());
    float p2 = std::stof(ui->p2LinePmain->text().toStdString());

    tempUSBCameraP->getResolution(width, height);
    if (fx > 0 && fy > 0 && cx > 0 && cy > 0 && cx < width && cy < height) {
      tempUSBCameraP->setIntrinsic(fx, fy, cx, cy, k1, k2, p1, p2, k3);
    } else {
      throw std::invalid_argument("Invalid fx fy cx or cy!");
    }
  }

  std::string savePath = ui->savePathLine->text().toStdString();
  if (savePath.back() != '/') savePath += '/';
  std::string cameraPath = savePath + "camera.txt";

  ui->pathToCameraLineP1->setEnabled(false);
  ui->pathToCameraLineP1->setText(QString::fromStdString(cameraPath));
  ui->pathToCameraLineP3->setEnabled(false);
  ui->pathToCameraLineP3->setText(QString::fromStdString(cameraPath));

  if (-1 == system(("mkdir -p " + savePath + "jointCalibration/").c_str()))
    throw std::invalid_argument("Cannot create dir " + savePath +
                                "jointCalibration/");

  ui->saveToLine->setEnabled(false);
  ui->saveToLine->setText(ui->savePathLine->text() + "/jointCalibration/");

  ui->configPathP6->setText(ui->savePathLine->text() +
                            "/jointCalibration/config.json");
  ui->datasetPathP6->setEnabled(false);
  ui->datasetPathP6->setText(ui->savePathLine->text() + "/jointCalibration");

  ui->pathToConfigLineP5->setEnabled(false);
  ui->pathToConfigLineP5->setText(
      QString::fromStdString(savePath + "jointCalibration/config.json"));

  config_path_ = ui->pathToConfigLineP5->text();

  if (-1 == system(("mkdir -p " + savePath + "gamma/").c_str()))
    throw std::invalid_argument("Cannot create dir " + savePath + "gamma/");

  ui->gammaPathLineP1->setEnabled(false);
  ui->gammaPathLineP1->setText(ui->savePathLine->text() + "/gamma/");
  // P2 and P4 are automatically changed
  ui->gammaPathLineP2->setEnabled(false);
  ui->gammaPathLineP4->setEnabled(false);

  if (-1 == system(("mkdir -p " + savePath + "vignette/").c_str()))
    throw std::invalid_argument("Cannot create dir " + savePath + "vignette/");

  ui->vignettePathLineP3->setEnabled(false);
  ui->vignettePathLineP3->setText(ui->savePathLine->text() + "/vignette/");
  // P4 is automatically changed
  ui->vignettePathLineP4->setEnabled(false);

  std::ofstream camParamsFile(cameraPath);
  // int width, height;
  cameraP->getResolution(width, height);
  auto intrinsic = cameraP->getIntrinsic();
  float fx, fy, cx, cy;
  fx = intrinsic.fx / width;
  fy = intrinsic.fy / height;
  cx = (intrinsic.cx + 0.5) / width;
  cy = (intrinsic.cy + 0.5) / height;
  camParamsFile << fx << " " << fy << " " << cx << " " << cy << " "
                << "0" << std::endl;
  camParamsFile << width << " " << height << std::endl;
  camParamsFile << "crop" << std::endl;
  camParamsFile << width / 2 << " " << height / 2 << std::endl;
  camParamsFile.close();
}
void MainWindow::on_mainStartButton_clicked() {
  try {
    settingFinished();
    ui->tabWidget->setCurrentWidget(ui->gammaCalibData);

  } catch (std::exception& e) {
    QMessageBox::warning(this, tr("Error"), tr(e.what()));
  }
}
void MainWindow::on_mainStartLidarCalibButton_clicked() {
  try {
    settingFinished();
    generateConfig();
    ui->tabWidget->setCurrentWidget(ui->LiDARCalibData);

  } catch (std::exception& e) {
    if (std::string(e.what()) == "stof") {
      QMessageBox::warning(this, tr("Error:"),
                           tr("fx fy cx cy k1 k2 k3 p1 p2 is not a number!"));
    } else {
      QMessageBox::warning(this, tr("Error"), tr(e.what()));
    }
  }
}
void MainWindow::on_maxExposureSliderP1_valueChanged(int value) {
  if (cameraP) {
    cv::Mat tmpMat = cameraP->getFrame(value);
    QPixmap img = cvMat2QPixmap(tmpMat);
    img = img.scaled(ui->picOutLabelP1->size(), Qt::KeepAspectRatio);
    ui->picOutLabelP1->setPixmap(img);
  }
}

void MainWindow::on_minExposureSliderP1_valueChanged(int value) {
  if (cameraP) {
    cv::Mat tmpMat = cameraP->getFrame(value);
    QPixmap img = cvMat2QPixmap(tmpMat);
    img = img.scaled(ui->picOutLabelP1->size(), Qt::KeepAspectRatio);
    ui->picOutLabelP1->setPixmap(img);
  }
}

void MainWindow::on_startButtonP1_clicked() {
  if (ui->startButtonP1->text() == tr("开始")) {
    stopResponseData = false;

    ui->startButtonP1->setText("取消");
    auto allWidgets = ui->gammaCalibData->findChildren<QWidget*>();
    std::vector<QWidget*> widgetsStatusChanged;
    std::copy_if(allWidgets.constBegin(), allWidgets.constEnd(),
                 std::back_inserter(widgetsStatusChanged), [&](QWidget* w) {
                   return w->isEnabled() && w->focusPolicy() != Qt::NoFocus &&
                          w != ui->startButtonP1;
                 });
    for (auto w : widgetsStatusChanged) w->setDisabled(true);

    std::string dataPath = ui->gammaPathLineP1->text().toStdString();
    std::string pathToCameraTxt = ui->pathToCameraLineP1->text().toStdString();
    int minExposureTime = ui->minExposureSliderP1->value();
    int maxExposureTime = ui->maxExposureSliderP1->value();
    int exposureNum = 150;
    int imgNum = 8;

    if (dataPath.back() != '/') dataPath += '/';

    if (-1 == system(("mkdir -p " + dataPath + "images/").c_str()))
      throw std::invalid_argument("Cannot create dir " + dataPath + "images/");

    if (-1 ==
        system(("cp -f " + pathToCameraTxt + " " + dataPath + "camera.txt")
                   .c_str()))
      throw std::invalid_argument("Cannot copy " + dataPath);

    std::vector<int> exposureTimes;
    double exposureTimeGap =
        pow(maxExposureTime / minExposureTime, 1.0 / exposureNum);

    for (double tempExposureTime = minExposureTime;
         tempExposureTime < maxExposureTime;
         tempExposureTime *= exposureTimeGap)
      for (size_t j = 0; j < imgNum; j++)
        exposureTimes.push_back(tempExposureTime);

    std::fstream timesFile;
    timesFile.open(dataPath + "times.txt", std::ios::out | std::ios::trunc);

    for (int i = 0; i < exposureTimes.size() && !stopResponseData; i++) {
      long long timeOfArrival = 0;
      auto exposureTime = exposureTimes[i];
      char imgId[100];
      snprintf(imgId, 100, "%05d", i);
      char imgName[100];
      snprintf(imgName, 100, "images/%05d.png", i);
      cv::Mat currentFrame = cameraP->getFrame(exposureTime, timeOfArrival);
      cv::imwrite((dataPath + imgName).c_str(), currentFrame);
      timesFile << imgId << " " << timeOfArrival << " " << exposureTime / 1000.0
                << "\n";

      // show frame on qt
      QPixmap img = cvMat2QPixmap(currentFrame);
      img = img.scaled(ui->picOutLabelP1->size(), Qt::KeepAspectRatio);
      ui->picOutLabelP1->setPixmap(img);
      QCoreApplication::processEvents();  // visualizing code ever after,
                                          // needn't care
    }
    timesFile.close();

    ui->startButtonP1->setText("开始");
    for (auto w : widgetsStatusChanged) w->setEnabled(true);

  } else if (ui->startButtonP1->text() == tr("取消"))
    stopResponseData = true;
}

void MainWindow::on_exposureSliderP3_valueChanged(int value) {
  if (cameraP) {
    cv::Mat tmpMat = cameraP->getFrame(value);
    QPixmap img = cvMat2QPixmap(tmpMat);
    img = img.scaled(ui->picOutLabelP3->size(), Qt::KeepAspectRatio);
    ui->picOutLabelP3->setPixmap(img);
  }
}

void MainWindow::on_startButtonP3_clicked() {
  if (ui->startButtonP3->text() == tr("开始")) {
    stopVignetteData = false;

    ui->startButtonP3->setText("取消");
    auto allWidgets = ui->vignetteCalibData->findChildren<QWidget*>();
    std::vector<QWidget*> widgetsStatusChanged;
    std::copy_if(allWidgets.constBegin(), allWidgets.constEnd(),
                 std::back_inserter(widgetsStatusChanged), [&](QWidget* w) {
                   return w->isEnabled() && w->focusPolicy() != Qt::NoFocus &&
                          w != ui->startButtonP3;
                 });
    for (auto w : widgetsStatusChanged) w->setDisabled(true);

    // RsCamera* cameraP = cameraP;
    std::string dataPath = ui->vignettePathLineP3->text().toStdString();
    std::string pathToCameraTxt = ui->pathToCameraLineP3->text().toStdString();
    int exposureTime = ui->exposureSliderP3->value();
    int imgNum = 800;
    // int fps = ui->fpsSliderP3->value();

    if (dataPath.back() != '/') dataPath += '/';
    // if (-1 == system(("mkdir -p " + dataPath).c_str()))
    //   throw std::invalid_argument("Cannot create dir " + dataPath);
    if (-1 == system(("mkdir -p " + dataPath + "images/").c_str()))
      throw std::invalid_argument("Cannot create dir " + dataPath + "images/");
    if (-1 ==
        system(("cp -f " + pathToCameraTxt + " " + dataPath + "camera.txt")
                   .c_str()))
      throw std::invalid_argument("Cannot copy " + dataPath);

    std::fstream timesFile;
    timesFile.open(dataPath + "times.txt", std::ios::out | std::ios::trunc);

    cv::Mat tmpMat = cameraP->getFrame();
    cv::Mat pointsInRange = cv::Mat::zeros(tmpMat.size(), CV_8UC1);

    for (int i = 0; i < imgNum && !stopVignetteData; i++) {
      long long timeOfArrival;
      char imgId[100];
      snprintf(imgId, 100, "%05d", i);
      char imgName[100];
      snprintf(imgName, 100, "images/%05d.png", i);
      cv::Mat currentFrame = cameraP->getFrame(exposureTime, timeOfArrival);
      cv::imwrite((dataPath + imgName).c_str(), currentFrame);
      timesFile << imgId << " " << timeOfArrival << " " << exposureTime / 1000.0
                << "\n";

      // show current frame on qt
      QPixmap img = cvMat2QPixmap(currentFrame);
      img = img.scaled(ui->picOutLabelP3->size(), Qt::KeepAspectRatio);
      ui->picOutLabelP3->setPixmap(img);

      findPointsInRange(currentFrame, pointsInRange, pathToCameraTxt);

      // show covered range on qt
      QPixmap img2 = cvMat2QPixmap(pointsInRange);
      img2 =
          img2.scaled(ui->pointsInRangeOutLabelP3->size(), Qt::KeepAspectRatio);
      ui->pointsInRangeOutLabelP3->setPixmap(img2);
      QCoreApplication::processEvents();  // visualizing code ever after,
                                          // needn't care
    }
    timesFile.close();

    ui->startButtonP3->setText("开始");
    for (auto w : widgetsStatusChanged) w->setEnabled(true);

  } else if (ui->startButtonP3->text() == tr("取消"))
    stopVignetteData = true;
}

void MainWindow::on_nextButtonP1_clicked() {
  ui->tabWidget->setCurrentWidget(ui->gammaCalib);
}

void MainWindow::on_nextButtonP2_clicked() {
  ui->tabWidget->setCurrentWidget(ui->vignetteCalibData);
}

void MainWindow::on_nextButtonP3_clicked() {
  ui->tabWidget->setCurrentWidget(ui->vignetteCalib);
}

void MainWindow::on_nextButtonP4_clicked() {
  try {
    generateConfig();
    ui->tabWidget->setCurrentWidget(ui->LiDARCalibData);
  } catch (std::exception& e) {
    QMessageBox::warning(this, tr("Error"), tr(e.what()));
  }
}

void MainWindow::on_startButtonP2_clicked() {
  if (ui->startButtonP2->text() == tr("开始")) {
    stopResponseCalib = false;

    ui->startButtonP2->setText("取消");
    auto allWidgets = ui->gammaCalib->findChildren<QWidget*>();
    std::vector<QWidget*> widgetsStatusChanged;
    std::copy_if(allWidgets.constBegin(), allWidgets.constEnd(),
                 std::back_inserter(widgetsStatusChanged), [&](QWidget* w) {
                   return w->isEnabled() && w->focusPolicy() != Qt::NoFocus &&
                          w != ui->startButtonP2;
                 });
    for (auto w : widgetsStatusChanged) w->setDisabled(true);

    responseCalib(ui->gammaPathLineP2->text().toStdString(), ui->shellOutTextP2,
                  ui->picOutLabelP2, stopResponseCalib);

    ui->startButtonP2->setText("开始");
    for (auto w : widgetsStatusChanged) w->setEnabled(true);

  } else if (ui->startButtonP2->text() == tr("取消"))
    stopResponseCalib = true;
}

void MainWindow::on_startButtonP4_clicked() {
  if (ui->startButtonP4->text() == tr("开始")) {
    stopVignetteCalib = false;

    ui->startButtonP4->setText("取消");
    auto allWidgets = ui->vignetteCalib->findChildren<QWidget*>();
    std::vector<QWidget*> widgetsStatusChanged;
    std::copy_if(allWidgets.constBegin(), allWidgets.constEnd(),
                 std::back_inserter(widgetsStatusChanged), [&](QWidget* w) {
                   return w->isEnabled() && w->focusPolicy() != Qt::NoFocus &&
                          w != ui->startButtonP4;
                 });
    for (auto w : widgetsStatusChanged) w->setDisabled(true);

    vignetteCalib(ui->vignettePathLineP4->text().toStdString(),
                  ui->shellOutTextP4, ui->gammaPathLineP4->text().toStdString(),
                  ui->picOutLabelP4, stopVignetteCalib);

    ui->startButtonP4->setText("开始");
    for (auto w : widgetsStatusChanged) w->setEnabled(true);

  } else if (ui->startButtonP4->text() == tr("取消"))
    stopVignetteCalib = true;
}

void MainWindow::pointCloudHandler(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::fromROSMsg(*msg, pointCloud);
}

void MainWindow::on_startButtonP5_clicked() {
  try {
    // readConfig();
    calibrator_.reset(new lqh::Calibrator(js_));

    img_viewer_.reset(new ImageViewer);
    img_viewer_->show();
    pc_viewer_.reset(new PointcloudViewer);
    pc_viewer_->show();

    if (ui->topicLineP5->text().isEmpty()) {
      throw std::invalid_argument("Rostopic of lidar is empty!");
    } else {
      rostopic = ui->topicLineP5->text().toStdString();
      subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(
          rostopic, 2, &MainWindow::pointCloudHandler, this);
    }

    const Eigen::Matrix4d& tf = calibrator_->GetTransformation();
    auto setWidget = [](double data, QSlider* sld, QLabel* lb) {
      int32_t val = static_cast<int32_t>(std::round(data * 500 + 500));
      if (val < 0) {
        val = 0;
      }
      if (val > 1000) {
        val = 1000;
      }
      sld->setValue(val);
      lb->setText(QString("%1").arg((val - 500.0) / 500.0, 6, 'f', 3, ' '));
    };

    setWidget(tf(0, 3), ui->tx_slide, ui->tx_text);
    setWidget(tf(1, 3), ui->ty_slide, ui->ty_text);
    setWidget(tf(2, 3), ui->tz_slide, ui->tz_text);

    // xyz - euler
    Eigen::Matrix3d rotation = tf.topLeftCorner(3, 3);
    // [0:pi]x[-pi:pi]x[-pi:pi]
    Eigen::Vector3d angle = rotation.eulerAngles(0, 1, 2) / PI * 180;
    Eigen::Vector3i r = angle.cast<int>();

    uint16_t v = r(0) < 0 ? 0 : (r(0) > 180 ? 180 : r(0));
    ui->rx_slide->setValue(v);
    ui->rx_text->setText(QString::number(v));

    v = r(1) < -180 ? 0 : (r(1) > 180 ? 360 : r(1) + 180);
    ui->ry_slide->setValue(v);
    ui->ry_text->setText(QString::number(r(1)));

    v = r(2) < -180 ? 0 : (r(2) > 180 ? 360 : r(2) + 180);
    ui->rz_slide->setValue(v);
    ui->rz_text->setText(QString::number(r(2)));

    connect(ui->rx_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
    connect(ui->ry_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
    connect(ui->rz_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
    connect(ui->tx_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
    connect(ui->ty_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
    connect(ui->tz_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);

    RsCamera* tempRsCamera = dynamic_cast<RsCamera*>(cameraP.get());
    if (tempRsCamera) {
      ui->AEControlLineP5->setEnabled(true);
    }
  } catch (std::exception& e) {
    QMessageBox::warning(this, tr("Error"), tr(e.what()));
  }
}

void MainWindow::on_playButtonP5_clicked() {
  ui->playButtonP5->setDisabled(true);
  ui->pauseButtonP5->setEnabled(true);

  while (ui->pauseButtonP5->isEnabled()) {
    cv::Mat imgOrg, imgBGR;
    imgOrg = cameraP->getFrame();
    if (imgOrg.type() == CV_8UC1)
      cv::cvtColor(imgOrg, imgBGR, cv::COLOR_GRAY2BGR);
    else
      imgBGR = imgOrg;

    img_ = std::make_shared<cv::Mat>(imgBGR);
    pc_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    *pc_ = pointCloud;

    tfProcess();

    QCoreApplication::processEvents();
  }
}

void MainWindow::on_pauseButtonP5_clicked() {
  ui->pauseButtonP5->setDisabled(true);
  ui->playButtonP5->setEnabled(true);
}

// Save image and pointclouds after 2 seconds.
void MainWindow::on_saveButton_clicked() {
  ui->pauseButtonP5->setDisabled(true);
  ui->playButtonP5->setDisabled(true);
  ui->saveButton->setDisabled(true);
  ui->unsaveButtonP5->setEnabled(true);

  QTimer::singleShot(2 * 1000, this, &MainWindow::doRealSaveImgPcl);
}

void MainWindow::on_unsaveButtonP5_clicked() {
  ui->unsaveButtonP5->setDisabled(true);
  ui->unsaveButtonP5->setText("Unsaved");
}

void MainWindow::doRealSaveImgPcl() {
  if (ui->unsaveButtonP5->isEnabled()) {
    try {
      if (img_ && pc_) {
        std::string dataPath = ui->saveToLine->text().toStdString();
        static int index = 0;
        if (dataPath.back() != '/') dataPath += '/';

        if (index == 0) {
          if (-1 == system(("mkdir -p " + dataPath + "image_orig/").c_str()))
            throw std::invalid_argument("Cannot create dir " + dataPath +
                                        "image_orig/");
          if (-1 == system(("mkdir -p " + dataPath + "pointcloud/").c_str()))
            throw std::invalid_argument("Cannot create dir " + dataPath +
                                        "pointcloud/");
        }
        cv::imwrite(dataPath + "image_orig/" + std::to_string(index) + ".jpg",
                    *img_);
        pcl::io::savePCDFile(
            dataPath + "pointcloud/" + std::to_string(index) + ".pcd", *pc_);
        ++index;
      }
    } catch (std::exception& e) {
      QMessageBox::warning(this, tr("Error"), tr(e.what()));
    }
  }

  ui->saveButton->setEnabled(true);
  ui->unsaveButtonP5->setDisabled(true);
  ui->unsaveButtonP5->setText("Unsave");
  on_playButtonP5_clicked();
}

void MainWindow::on_finishButton_clicked() {
  Eigen::Matrix4d tf;
  tf.setIdentity();
  tf(0, 3) = (ui->tx_slide->value() - 500) / 500.0;
  ui->tx_text->setText(QString::number(tf(0, 3)));
  tf(1, 3) = (ui->ty_slide->value() - 500) / 500.0;
  ui->ty_text->setText(QString::number(tf(1, 3)));
  tf(2, 3) = (ui->tz_slide->value() - 500) / 500.0;
  ui->tz_text->setText(QString::number(tf(2, 3)));

  ui->rx_text->setText(QString::number(ui->rx_slide->value()));
  double rx = ui->rx_slide->value() / 180.0 * PI;
  ui->ry_text->setText(QString::number(ui->ry_slide->value() - 180));
  double ry = (ui->ry_slide->value() - 180.0) / 180.0 * PI;
  ui->rz_text->setText(QString::number(ui->rz_slide->value() - 180));
  double rz = (ui->rz_slide->value() - 180.0) / 180.0 * PI;

  Eigen::Quaterniond ag = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
  tf.topLeftCorner(3, 3) = ag.matrix();
  tf.row(3) << 0, 0, 0, 1;

  js_["tf"] = {};
  for (uint8_t i = 0; i < 4; i++) {
    js_["tf"].push_back({tf(i, 0), tf(i, 1), tf(i, 2), tf(i, 3)});
  }
  generateNewConfig();
  closeImgAndPcViewers();
  ui->tabWidget->setCurrentWidget(ui->LiDARCalib);
  startCalib();
}

void MainWindow::on_tabWidget_currentChanged(int index) {
  closeImgAndPcViewers();
}

void MainWindow::tfProcess() {
  Eigen::Matrix4d tf;
  tf.setIdentity();
  tf(0, 3) = (ui->tx_slide->value() - 500) / 500.0;
  ui->tx_text->setText(QString::number(tf(0, 3)));
  tf(1, 3) = (ui->ty_slide->value() - 500) / 500.0;
  ui->ty_text->setText(QString::number(tf(1, 3)));
  tf(2, 3) = (ui->tz_slide->value() - 500) / 500.0;
  ui->tz_text->setText(QString::number(tf(2, 3)));

  ui->rx_text->setText(QString::number(ui->rx_slide->value()));
  double rx = ui->rx_slide->value() / 180.0 * PI;
  ui->ry_text->setText(QString::number(ui->ry_slide->value() - 180));
  double ry = (ui->ry_slide->value() - 180.0) / 180.0 * PI;
  ui->rz_text->setText(QString::number(ui->rz_slide->value() - 180));
  double rz = (ui->rz_slide->value() - 180.0) / 180.0 * PI;

  Eigen::Quaterniond ag = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
  tf.topLeftCorner(3, 3) = ag.matrix();
  tf.row(3) << 0, 0, 0, 1;
  updateWithTransformation(tf);
}

/*
void MainWindow::on_Open_Config_Button_clicked() {

  readConfig();

  auto& flt = js_["pc"]["filter"];
  ui->angle_start_slide->setValue(static_cast<int>(flt["angle_start"]));
  ui->angle_size_slide->setValue(static_cast<int>(flt["angle_size"]));
  ui->distance_slide->setValue(
      static_cast<int>(10 * flt["distance"].get<double>()));
  ui->floor_gap_slide->setValue(
      static_cast<int>(10 * flt["floor_gap"].get<double>()));

  img_viewer_.reset(new ImageViewer);
  img_viewer_->show();
  pc_viewer_.reset(new PointcloudViewer);
  pc_viewer_->show();

  connect(ui->angle_start_slide, &QSlider::valueChanged, this,
          &MainWindow::processSlider);
  connect(ui->angle_size_slide, &QSlider::valueChanged, this,
          &MainWindow::processSlider);
  connect(ui->distance_slide, &QSlider::valueChanged, this,
          &MainWindow::processSlider);
  connect(ui->floor_gap_slide, &QSlider::valueChanged, this,
          &MainWindow::processSlider);

  config_path_ = QFileDialog::getOpenFileName(
      this, tr("Open File"), QDir::homePath(), tr("Config JSON Files(*.json)"));
  if (config_path_.isEmpty()) {
    QMessageBox::warning(this, tr("Error"), tr("Config file is not changed!"));
    return;
  } else {
    ui->configPathP6->setText(config_path_);
  }
}
*/

void MainWindow::closeEvent(QCloseEvent* event) {
  closeImgAndPcViewers();

  event->accept();
}
/*
void MainWindow::on_Set_K_Button_clicked() {
  bool ok = false;
  QString last;
  auto& K = js_["cam"]["K"];
  for (uint8_t i = 0; i < 9; i++) {
    last.append(QString("%1 ").arg(K[i / 3][i % 3].get<double>()));
  }
  QString ks = QInputDialog::getText(this, tr("Camera matrix, K"), tr("K"),
                                     QLineEdit::Normal, last, &ok);
  if (!ok) {
    return;
  }
  if (!ks.isEmpty()) {
    ks.replace(QChar('\n'), QChar(' '));
    QStringList ns = ks.simplified().split(" ");
    if (ns.size() == 9) {
      Eigen::Matrix3d CK;
      for (uint8_t i = 0; i < 9; i++) {
        CK(i / 3, i % 3) = ns[i].toDouble();
        K[i / 3][i % 3] = CK(i / 3, i % 3);
      }
      if (data_reader_ != nullptr) {
        data_reader_->setCameraK(CK);
      }
      calibrator_->SetCameraK(CK);
      return;
    }
  }
  QMessageBox::warning(this, tr("Error"), tr("Invalid parameters"));
}

void MainWindow::on_Set_D_Button_clicked() {
  bool ok = false;
  auto& D = js_["cam"]["D"];
  QString last = QString("%1 %2 %3 %4 %5")
                     .arg(D[0].get<double>())
                     .arg(D[1].get<double>())
                     .arg(D[2].get<double>())
                     .arg(D[3].get<double>())
                     .arg(D[4].get<double>());

  QString ks = QInputDialog::getText(this, tr("Distortion Parameters"), tr("D"),
                                     QLineEdit::Normal, last, &ok);
  if (!ok) {
    return;
  }
  if (!ks.isEmpty()) {
    Eigen::Matrix<double, 5, 1> CD;
    ks.replace(QChar('\n'), QChar(' '));
    QStringList ns = ks.split(" ");
    if (ns.size() == 5) {
      for (uint8_t i = 0; i < 5; i++) {
        CD(i) = ns[i].toDouble();
        D[i] = CD(i);
      }
      if (data_reader_ != nullptr) {
        data_reader_->setCameraD(CD);
      }
      return;
    }
  }
  QMessageBox::warning(this, tr("Error"), tr("Invalid parameters"));
}
*/
void MainWindow::startCalib() {
  // readConfig();

  calibrator_.reset(new lqh::Calibrator(js_));

  auto& flt = js_["pc"]["filter"];
  ui->angle_start_slide->setValue(static_cast<int>(flt["angle_start"]));
  ui->angle_size_slide->setValue(static_cast<int>(flt["angle_size"]));
  ui->distance_slide->setValue(
      static_cast<int>(10 * flt["distance"].get<double>()));
  ui->floor_gap_slide->setValue(
      static_cast<int>(10 * flt["floor_gap"].get<double>()));

  img_viewer_.reset(new ImageViewer);
  img_viewer_->show();
  pc_viewer_.reset(new PointcloudViewer);
  pc_viewer_->show();

  connect(ui->angle_start_slide, &QSlider::valueChanged, this,
          &MainWindow::processSlider);
  connect(ui->angle_size_slide, &QSlider::valueChanged, this,
          &MainWindow::processSlider);
  connect(ui->distance_slide, &QSlider::valueChanged, this,
          &MainWindow::processSlider);
  connect(ui->floor_gap_slide, &QSlider::valueChanged, this,
          &MainWindow::processSlider);

  QString dir = ui->datasetPathP6->text();
  data_reader_.reset(new DataReader(dir));

  // QString dir = QFileDialog::getExistingDirectory(
  //     this, tr("Open Directory"), QDir::homePath(),
  //     QFileDialog::ShowDirsOnly);
  // if (!dir.isEmpty()) {
  //   data_reader_.reset(new DataReader(dir));  // to reserve
  // if (!data_reader_->isValid()) {
  //   data_reader_ = nullptr;
  //   QMessageBox::warning(this, tr("Warn"), tr("The directory is invalid"));
  //   return;
  // }

  Eigen::Matrix3d K;
  Eigen::Matrix<double, 5, 1> D;
  auto& JK = js_["cam"]["K"];
  auto& JD = js_["cam"]["D"];
  for (uint8_t i = 0; i < 9; i++) {
    K(i / 3, i % 3) = JK[i / 3][i % 3];
    if (i < 5) {
      D(i) = JD[i];
    }
  }
  data_reader_->setCameraK(K);
  data_reader_->setCameraD(D);
  data_root_ = dir;
  ui->total_data_num->setText(QString::number(data_reader_->getDatasetSize()));

  showTFWindow();
  // }
}

void MainWindow::on_next_pose_clicked() {
  if (is_calibrated_) {
    // after calibration
    if (sid_ < sensor_data_.size() - 1) {
      sid_++;
    } else {
      if (data_reader_->moveNext()) {
        sensor_data_.emplace_back(data_reader_->getCurrentId());
        auto& last = sensor_data_.back();
        last.img = data_reader_->getImage();
        last.pc = data_reader_->getPointcloud();
        if (!last.img) {
          sensor_data_.pop_back();
          QMessageBox::warning(this, tr("Error"), tr("Fail to read image"));
          return;
        }
        if (!last.pc) {
          sensor_data_.pop_back();
          QMessageBox::warning(this, tr("Error"),
                               tr("Fail to read pointcloud"));
          return;
        }
        last.pid = INT_MAX;  // invalid
        sid_ = sensor_data_.size() - 1;
      } else {
        // last one and fail to read more, so do noting
        return;
      }
    }
    showCalibrateResult();
  } else {
    // before calibration
    processData();
  }
}

void MainWindow::on_quick_next_pose_clicked() {
  if (is_calibrated_) {
    // previous
    if (sid_ > 0) {
      sid_--;
    }
    showCalibrateResult();
  } else {
    // quick next
    setEnabledAll(false);
    setCursor(Qt::WaitCursor);

    bool res = processData();

    while (res) {
      res = processData();
      QCoreApplication::processEvents(QEventLoop::AllEvents);
      //            QThread::msleep(5);
    }

    setCursor(Qt::ArrowCursor);
    setEnabledAll(true);
  }
}

void MainWindow::on_delete_pose_clicked() {
  if (sensor_data_.size() > 0) {
    sensor_data_.back().img_good = false;
    sensor_data_.back().pc_good = false;
    calibrator_->Remove();
  }
  processData(false);
}

void MainWindow::on_calibrate_clicked() {
  if (sensor_data_.size() < 1) {
    QMessageBox::warning(this, tr("Error"),
                         tr("No enough data to do calibration, at lease 3"));
    return;
  }

  setEnabledAll(false);
  setCursor(Qt::WaitCursor);
  //    calibrator_->SavePolygonData(std::string("/home/nick/tmp"));

  bool res = calibrator_->Compute();

  setCursor(Qt::ArrowCursor);
  setEnabledAll(true);

  if (!res) {
    QMessageBox::warning(this, tr("Error"), tr("Fail to calibrate"));
    return;
  }

  is_calibrated_ = true;
  ui->quick_next_pose->setText(tr("Previous"));
  showCalibrateResult();
  pc_viewer_->showCoordinateSystem(
      Eigen::Affine3f(calibrator_->GetTransformation().inverse().cast<float>()),
      1, 0.5);

  auto T = calibrator_->GetTransformation();
  auto& tf = js_["tf"];
  tf.clear();
  for (uint8_t i = 0; i < 4; i++) {
    tf.push_back(std::vector<double>{T(i, 0), T(i, 1), T(i, 2), T(i, 3)});
  }
}

void MainWindow::on_pick_points_start_clicked() {
  img_viewer_->startPickPoints();
}

void MainWindow::on_pick_points_quit_clicked() {
  img_viewer_->quitPickPoints();
}

void MainWindow::on_pick_points_end_clicked() {
  std::vector<cv::Point2d> pts;
  img_viewer_->getPickPoitns(pts);

  if (pts.size() != js_["size"]) {
    QMessageBox::warning(this, tr("Error"),
                         tr("Must choose ") +
                             QString::number(js_["size"].get<int>()) +
                             tr(" points"));
    return;
  }
  auto& img_i = js_["img"]["init"];
  img_i.clear();
  for (int i = 0; i < js_["size"].get<int>(); i++) {
    img_i.push_back(std::vector<double>{pts[i].x, pts[i].y});
  }

  auto& last = sensor_data_.back();
  if (calibrator_->RefineImage(*last.img, *last.img_marked, pts)) {
    last.img_good = true;
  }

  img_viewer_->showImage(last.img_marked);

  updateLabels();
}
/*
void MainWindow::readConfig() {
  config_path_ = ui->pathToConfigLineP5->text();
  if (config_path_.isEmpty()) {
    QMessageBox::warning(this, tr("Error"), tr("Config file is empty"));
    return;
  }
  std::ifstream f(config_path_.toStdString());
  if (!f.good()) {
    f.close();
    QMessageBox::warning(this, tr("Error"), tr("Failed to read config file"));
    return;
  }
  try {
    f >> js_;
  } catch (nlohmann::json::parse_error& e) {
    std::cerr << e.what();
    f.close();
    QMessageBox::warning(this, tr("Error"), tr(e.what()));
    return;
  }
  calibrator_.reset(new lqh::Calibrator(js_));
}
*/
void MainWindow::updateLabels() {
  if (sensor_data_.size() > 0) {
    ui->current_data_id->setText(QString::number(sid_));

    uint32_t num = sensor_data_.size();
    if (!sensor_data_.back().good()) {
      num -= 1;
    }
    ui->processed_data_num->setText(QString::number(num));
  } else {
    ui->current_data_id->setText("Null");
    ui->processed_data_num->setText("0");
  }
}

bool MainWindow::processData(bool is_check) {
  if (data_reader_ == nullptr) {
    QMessageBox::warning(this, tr("Error"), tr("Open dataset first"));
    return false;
  }

  if (is_check && sensor_data_.size() > 0 && !sensor_data_.back().good()) {
    QMessageBox::warning(this, tr("Error"),
                         tr("Current data is not good, adjust or delete"));
    return false;
  }

  if (sensor_data_.size() > 0 && !data_reader_->moveNext()) {
    return false;
  }

  sensor_data_.emplace_back(data_reader_->getCurrentId());
  sid_ = sensor_data_.size() - 1;
  auto& last = sensor_data_.back();
  last.img = data_reader_->getImage();
  if (!last.img) {
    sensor_data_.pop_back();
    QMessageBox::warning(this, tr("Error"), tr("Fail to read image"));
    return false;
  }

  last.pc = data_reader_->getPointcloud();
  if (!last.pc) {
    sensor_data_.pop_back();
    QMessageBox::warning(this, tr("Error"), tr("Fail to read pointcloud"));
    return false;
  }

  last.img_marked = std::make_shared<cv::Mat>();
  last.img->copyTo(*last.img_marked);
  last.pc_marked.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  last.pid =
      calibrator_->Add(*last.img, *last.pc, *last.img_marked, *last.pc_marked);
  if (calibrator_->ImageGood(last.pid)) {
    last.img_good = true;
  }
  if (calibrator_->PointcloudGood(last.pid)) {
    last.pc_good = true;
  }

  img_viewer_->showImage(last.img_marked);
  pc_viewer_->showPointcloud(last.pc_marked);

  sid_ = sensor_data_.size() - 1;
  updateLabels();

  return true;
}

void MainWindow::setEnabledAll(bool status) {
  // QList<QPushButton*> btns =
  // ui->centralWidget->findChildren<QPushButton*>(); for (auto& it : btns) {
  //   it->setEnabled(status);
  // }

  QList<QSlider*> sliders = ui->pointcloud_group->findChildren<QSlider*>();
  for (auto& it : sliders) {
    it->setEnabled(status);
  }
}

void MainWindow::showCalibrateResult() {
  auto& sd = sensor_data_[sid_];

  if (sd.img_proj == nullptr || sd.pc_proj == nullptr) {
    sd.img_proj.reset(new cv::Mat);
    sd.img->copyTo(*sd.img_proj);
    sd.pc_proj.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*sd.pc, *sd.pc_proj);
    for (auto& p : sd.pc_proj->points) {
      p.rgba = 0xffffffff;
    }
    calibrator_->Project(*sd.pc_proj, *sd.img_proj);
  }

  img_viewer_->showImage(sd.img_proj);
  pc_viewer_->showPointcloud(sd.pc_proj);

  updateLabels();
}

void MainWindow::on_Save_Result_Button_clicked() {
  if (!is_calibrated_) {
    QMessageBox::warning(this, tr("Error"), tr("Not calibrate yet"));
    return;
  }

  QString fp = data_root_ + QDir::separator() + "calibration_" +
               QDateTime::currentDateTime().toString("yyyy-M-d-h-m-s") +
               ".json";
  std::ofstream f(fp.toStdString());
  if (!f.good()) {
    QMessageBox::warning(this, tr("Error"), tr("Fail to open file ") + fp);
    return;
  }

  const Eigen::Matrix4d& tf = calibrator_->GetTransformation();
  nlohmann::json js;
  for (uint8_t i = 0; i < 4; i++) {
    js["T"].push_back({tf(i, 0), tf(i, 1), tf(i, 2), tf(i, 3)});
  }
  js["K"] = js_["cam"]["K"];
  js["D"] = js_["cam"]["D"];

  f << js.dump(4);
  f.close();

  QMessageBox::information(this, tr("Success"),
                           tr("Save calibration result to ") + fp);
}

void MainWindow::processSlider() {
  Eigen::Vector4d param;
  param(0) = ui->angle_start_slide->value();
  param(1) = ui->angle_size_slide->value();
  param(2) = ui->distance_slide->value() / 10.0;
  param(3) = ui->floor_gap_slide->value() / 10.0;

  ui->angle_start_text->setText(QString::number(param(0)));
  ui->angle_size_text->setText(QString::number(param(1)));
  ui->distance_text->setText(QString::number(param(2)));
  ui->floor_gap_text->setText(QString::number(param(3)));

  if (sensor_data_.size() == 0 || calibrator_ == nullptr) {
    return;
  }
  auto& filter = js_["pc"]["filter"];
  filter["angle_start"] = param(0);
  filter["angle_size"] = param(1);
  filter["distance"] = param(2);
  filter["floor_gap"] = param(3);
  auto& sd = sensor_data_.back();
  calibrator_->RefinePointcloud(*sd.pc, *sd.pc_marked, param);
  if (calibrator_->PointcloudGood(sd.pid)) {
    sd.pc_good = true;
  }

  pc_viewer_->showPointcloud(sd.pc_marked);
}

void MainWindow::on_Save_Config_Button_clicked() {
  generateNewConfig();
  /*
  std::ofstream f(config_path_.toStdString());
  if (!f.good()) {
    QMessageBox::warning(this, tr("Error"), tr("Fail to open config file"));
    return;
  }
  f << js_.dump(4);
  f.close();
  */
}

void MainWindow::updateWithTransformation(Eigen::Matrix4d tf) {
  std::shared_ptr<cv::Mat> img_mark = std::make_shared<cv::Mat>();
  img_->copyTo(*img_mark);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*pc_, *pcc);
  for (auto& p : pcc->points) {
    p.rgba = 0xffffffff;
  }

  //    calibrator_->SetTranformation(tf_);
  calibrator_->SetTranformation(tf);
  calibrator_->Project(*pcc, *img_mark);

  img_viewer_->showImage(img_mark);
  pc_viewer_->showPointcloud(pcc);
}

void MainWindow::tfwindowClose() {
  this->move(tfwindow_->pos());
  this->show();
  tfwindow_.reset(nullptr);

  on_next_pose_clicked();
}

void MainWindow::showTFWindow() {
  img_ = data_reader_->getImage();  // always get BGR image
  pc_ = data_reader_->getPointcloud();
  if (img_ == nullptr || pc_ == nullptr) {
    QMessageBox::warning(this, tr("Error"),
                         tr("Fail to read image or pointcloud"));
    data_reader_.reset(nullptr);
    return;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*pc_, *pcc);
  for (auto& p : pcc->points) {
    p.rgba = 0xffffffff;
  }
  std::shared_ptr<cv::Mat> img_mark = std::make_shared<cv::Mat>();
  img_->copyTo(*img_mark);

  tfwindow_.reset(new TFwindow(calibrator_->GetTransformation()));
  connect(tfwindow_.get(), &TFwindow::newTransformation, this,
          &MainWindow::updateWithTransformation);
  connect(tfwindow_.get(), &TFwindow::tfwindowClose, this,
          &MainWindow::tfwindowClose);

  calibrator_->Project(*pcc, *img_mark);
  img_viewer_->showImage(img_mark);
  pc_viewer_->showPointcloud(pcc);

  tfwindow_->move(this->pos());
  tfwindow_->show();
  this->hide();
}

void MainWindow::closeImgAndPcViewers() {
  if (ui->pauseButtonP5->isEnabled()) {
    on_pauseButtonP5_clicked();
  }
  if (img_viewer_) {
    img_viewer_->close();
    img_viewer_.release();
  }
  if (pc_viewer_) {
    pc_viewer_->close();
    img_viewer_.release();
  }
}

void MainWindow::generateNewConfig() {
  std::ofstream f(config_path_.toStdString());
  if (!f.good()) {
    QMessageBox::warning(this, tr("Error"), tr("Fail to open config file"));
    return;
  }
  f << js_.dump(4);
  f.close();
}

void MainWindow::generateConfig() {
  QString fp = ui->saveToLine->text() + QDir::separator() + "config.json";
  std::ofstream f(fp.toStdString());
  if (!f.good()) {
    QMessageBox::warning(this, tr("Error"), tr("Fail to open config ") + fp);
    return;
  }
  auto intrinsic = cameraP->getIntrinsic();
  nlohmann::json js = {
      {"cam",
       {{"D",
         {intrinsic.coeffs[0], intrinsic.coeffs[1], intrinsic.coeffs[2],
          intrinsic.coeffs[3], intrinsic.coeffs[4]}},
        {"K",
         {{intrinsic.fx, 0.0, intrinsic.cx},
          {0.0, intrinsic.fy, intrinsic.cy},
          {0.0, 0.0, 1.0}}}}},
      {"img",
       {{"edlines",
         {{"anchorThreshold", 10},
          {"gradientThreshold", 80},
          {"ksize", 3},
          {"lineFitErrThreshold", 2},
          {"minLineLen", 30},
          {"scanIntervals", 1},
          {"sigma", 1}}},
        {"feature", {{"offset", 3}, {"size", 8}, {"threshold", 0.8}}},
        {"filter",
         {{"line_angle_threshold", 8},
          {"point_center_factor", 0.4},
          {"point_line_threshold", 80}}},
        {"init",
         {{1200.8, 273.6}, {1509.6, 420.0}, {1589.6, 775.2}, {888.0, 645.6}}},
        {"merge",
         {{"angle_threshold", 2},
          {"distance_threshold", 35},
          {"endpoint_distance_threshold", 80}}}}},
      {"pc",
       {{"angle_resolution", 1.5},
        {"edge", {{"distance_threshold", 0.05}, {"point_num_min", 5}}},
        {"filter",
         {{"angle_size", 73.0},
          {"angle_start", 327.0},
          {"distance", 5.3},
          {"floor_gap", 1.2}}},
        {"plane", {{"distance_threshold", 0.06}, {"point_num_min", 20}}},
        {"ring", {{"endpoint_num", 2}, {"point_num_min", 7}}}}},
      {"size", 4},
      {"tf",
       {{1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}}},
      {"track_error_threshold", 30}};
  f << js.dump(4);
  js_ = js;
  f.close();

  // ui->tabWidget->setCurrentWidget(ui->LiDARCalibData);
  /*
  readConfig();

  img_viewer_.reset(new ImageViewer);
  img_viewer_->show();
  pc_viewer_.reset(new PointcloudViewer);
  pc_viewer_->show();

  connect(ui->rx_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->ry_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->rz_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->tx_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->ty_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->tz_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  */
}

void MainWindow::on_mainStartCalibButton_clicked() {
  try {
    settingFinished();
    ui->tabWidget->setCurrentWidget(ui->LiDARCalib);

  } catch (std::exception& e) {
    if (std::string(e.what()) == "stof") {
      QMessageBox::warning(this, tr("Error:"),
                           tr("fx fy cx cy k1 k2 k3 p1 p2 is not a number!"));
    } else {
      QMessageBox::warning(this, tr("Error"), tr(e.what()));
    }
  }

  if (config_path_.isEmpty()) {
    QMessageBox::warning(this, tr("Error"), tr("Config file is empty"));
    return;
  }
  std::ifstream f(config_path_.toStdString());
  if (!f.good()) {
    f.close();
    QMessageBox::warning(this, tr("Error"), tr("Failed to read config file"));
    return;
  }
  try {
    f >> js_;
  } catch (nlohmann::json::parse_error& e) {
    std::cerr << e.what();
    f.close();
    QMessageBox::warning(this, tr("Error"), tr(e.what()));
    return;
  }

  startCalib();
}

#include "mainwindow.h"

// #include <pcl/io/pcd_io.h>  //for debug only
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
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
#include <fstream>
#include <string>

#include "RsCamera.h"
#include "findPointsInRange.h"
#include "responseCalib.h"
#include "ui_mainwindow.h"
#include "vignetteCalib.h"

RsCamera camera;
RsCamera* cameraP = &camera;

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

      sid_(INT_MAX),
      is_calibrated_(false),
      data_reader_(nullptr),
      img_(nullptr),
      pc_(nullptr) {
  ui->setupUi(this);
  this->resize(QSize(1000, 800));

  subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "velodyne_points", 2, &MainWindow::pointCloudHandler, this);
  rosTimer = new QTimer(this);
  rosTimer->setInterval(20);  // 单位毫秒
  connect(rosTimer, &QTimer::timeout, this, &MainWindow::timerLoop);
  rosTimer->start();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_pathToCameraLineP1_editingFinished() {
  int cameraIndex;
  std::string pathToCameraTxt;

  if (ui->LOrRBoxP1->currentText() == QString("L"))
    cameraIndex = 0;
  else if (ui->LOrRBoxP1->currentText() == QString("R"))
    cameraIndex = 1;
  pathToCameraTxt = ui->pathToCameraLineP1->text().toStdString();

  camera = RsCamera(cameraIndex, pathToCameraTxt);
}

void MainWindow::on_maxExposureSliderP1_valueChanged(int value) {
  if (cameraP) {
    cv::Mat tmpMat = cameraP->getFrame(value);
    QPixmap img = cvMat2QPixmap(tmpMat);
    ui->picOutViewP1->resize(img.width(), img.height());
    QGraphicsScene* camScene = new QGraphicsScene;
    camScene->setSceneRect(0, 0, img.width(), img.height());
    camScene->addPixmap(img);
    ui->picOutViewP1->setScene(camScene);
    ui->picOutViewP1->adjustSize();
    ui->picOutViewP1->show();
    // cv::Mat tmpMat = cameraP->getFrame(value);
    // QPixmap tmpPixMap = cvMat2QPixmap(tmpMat);
    // ui->picOutLabelP1->setPixmap(tmpPixMap);
  }
}

void MainWindow::on_minExposureSliderP1_valueChanged(int value) {
  if (cameraP) {
    cv::Mat tmpMat = cameraP->getFrame(value);
    QPixmap img = cvMat2QPixmap(tmpMat);
    ui->picOutViewP1->resize(img.width(), img.height());
    QGraphicsScene* camScene = new QGraphicsScene;
    camScene->setSceneRect(0, 0, img.width(), img.height());
    camScene->addPixmap(img);
    ui->picOutViewP1->setScene(camScene);
    ui->picOutViewP1->adjustSize();
    ui->picOutViewP1->show();
    // cv::Mat tmpMat = cameraP->getFrame(value);
    // QPixmap tmpPixMap = cvMat2QPixmap(tmpMat);
    // ui->picOutLabelP1->setPixmap(tmpPixMap);
  }
}

void MainWindow::on_startButtonP1_clicked() {
  std::string dataPath = ui->gammaPathLineP1->text().toStdString();
  std::string pathToCameraTxt = ui->pathToCameraLineP1->text().toStdString();
  int minExposureTime = ui->minExposureSliderP1->value();
  int maxExposureTime = ui->maxExposureSliderP1->value();
  int exposureNum = 150;
  int imgNum = 8;

  if (dataPath.back() != '/') dataPath += '/';
  if (-1 == system(("mkdir -p " + dataPath).c_str()))
    throw std::invalid_argument("Cannot create dir " + dataPath);
  if (-1 == system(("mkdir -p " + dataPath + "images/").c_str()))
    throw std::invalid_argument("Cannot create dir " + dataPath + "images/");
  if (-1 ==
      system(
          ("cp -f " + pathToCameraTxt + " " + dataPath + "camera.txt").c_str()))
    throw std::invalid_argument("Cannot copy " + dataPath);

  std::vector<int> exposureTimes;
  float exposureTimeGap =
      pow(maxExposureTime / minExposureTime, 1.0 / exposureNum);

  for (size_t i = 0, tempExposureTime = minExposureTime; i < exposureNum; i++) {
    for (size_t j = 0; j < imgNum; j++)
      exposureTimes.push_back(tempExposureTime);
    tempExposureTime *= exposureTimeGap;
  }

  std::fstream timesFile;
  timesFile.open(dataPath + "times.txt", std::ios::out | std::ios::trunc);

  for (int i = 0; i < exposureTimes.size(); i++) {
    int timeOfArrival = 0;
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
    ui->picOutViewP1->resize(img.width(), img.height());
    QGraphicsScene* currentScene = new QGraphicsScene;
    currentScene->setSceneRect(0, 0, img.width(), img.height());
    currentScene->addPixmap(img);
    ui->picOutViewP1->setScene(currentScene);
    ui->picOutViewP1->adjustSize();
    ui->picOutViewP1->show();
    QCoreApplication::processEvents();  // visualizing code ever after, needn't
                                        // care
    // QPixmap currentPixMap = cvMat2QPixmap(currentFrame);
    // ui->picOutLabelP1->setPixmap(currentPixMap);
  }

  timesFile.close();
}

void MainWindow::on_pathToCameraLineP3_editingFinished() {
  int cameraIndex;
  std::string pathToCameraTxt;

  if (ui->LOrRBoxP3->currentText() == QString("L"))
    cameraIndex = 0;
  else if (ui->LOrRBoxP3->currentText() == QString("R"))
    cameraIndex = 1;
  pathToCameraTxt = ui->pathToCameraLineP3->text().toStdString();

  camera = RsCamera(cameraIndex, pathToCameraTxt);
}

void MainWindow::on_exposureSliderP3_valueChanged(int value) {
  if (cameraP) {
    cv::Mat tmpMat = cameraP->getFrame(value);
    QPixmap img = cvMat2QPixmap(tmpMat);
    ui->picOutViewP3->resize(img.width(), img.height());
    QGraphicsScene* currentScene = new QGraphicsScene;
    currentScene->setSceneRect(0, 0, img.width(), img.height());
    currentScene->addPixmap(img);
    ui->picOutViewP3->setScene(currentScene);
    ui->picOutViewP3->adjustSize();
    ui->picOutViewP3->show();
    // cv::Mat tmpMat = cameraP->getFrame(value);
    // QPixmap tmpPixMap = cvMat2QPixmap(tmpMat);
    // ui->picOutLabelP3->setPixmap(tmpPixMap);
  }
}

void MainWindow::on_startButtonP3_clicked() {
  // RsCamera* cameraP = cameraP;
  std::string dataPath = ui->vignettePathLineP3->text().toStdString();
  std::string pathToCameraTxt = ui->pathToCameraLineP3->text().toStdString();
  int exposureTime = ui->exposureSliderP3->value();
  int imgNum = 800;
  int fps = ui->fpsSliderP3->value();

  if (dataPath.back() != '/') dataPath += '/';
  if (-1 == system(("mkdir -p " + dataPath).c_str()))
    throw std::invalid_argument("Cannot create dir " + dataPath);
  if (-1 == system(("mkdir -p " + dataPath + "images/").c_str()))
    throw std::invalid_argument("Cannot create dir " + dataPath + "images/");
  if (-1 ==
      system(
          ("cp -f " + pathToCameraTxt + " " + dataPath + "camera.txt").c_str()))
    throw std::invalid_argument("Cannot copy " + dataPath);

  std::fstream timesFile;
  timesFile.open(dataPath + "times.txt", std::ios::out | std::ios::trunc);

  cv::Mat tmpMat = cameraP->getFrame();
  cv::Mat pointsInRange = cv::Mat::zeros(tmpMat.size(), CV_8UC1);

  for (int i = 0; i < imgNum; i++, cv::waitKey(1 / fps)) {
    int timeOfArrival;
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
    ui->picOutViewP3->resize(img.width(), img.height());
    QGraphicsScene* camScene = new QGraphicsScene;
    camScene->setSceneRect(0, 0, img.width(), img.height());
    camScene->addPixmap(img);
    ui->picOutViewP3->setScene(camScene);
    ui->picOutViewP3->adjustSize();
    ui->picOutViewP3->show();
    // QPixmap currentPixMap = cvMat2QPixmap(currentFrame);
    // ui->picOutLabelP3->setPixmap(currentPixMap);

    findPointsInRange(currentFrame, pointsInRange, pathToCameraTxt);

    // show covered range on qt
    QPixmap img2 = cvMat2QPixmap(pointsInRange);
    QGraphicsScene* pointsInRangeScene = new QGraphicsScene;
    pointsInRangeScene->setSceneRect(0, 0, img2.width(), img2.height());
    pointsInRangeScene->addPixmap(img2);
    ui->pointsInRangeOutViewP3->setScene(pointsInRangeScene);
    ui->pointsInRangeOutViewP3->adjustSize();
    ui->pointsInRangeOutViewP3->show();
    QCoreApplication::processEvents();  // visualizing code ever after, needn't
                                        // care
    // QPixmap pointsInRangePixMap = cvMat2QPixmap(pointsInRange);
    // ui->pointsInRangeOutLabelP3->setPixmap(pointsInRangePixMap);
  }

  timesFile.close();
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

void MainWindow::on_startButtonP2_clicked() {
  responseCalib(ui->gammaPathLineP2->text().toStdString(), ui->shellOutTextP2,
                ui->picOutLabelP2);
}

void MainWindow::on_startButtonP4_clicked() {
  vignetteCalib(ui->vignettePathLineP4->text().toStdString(),
                ui->shellOutTextP4, ui->gammaPathLineP4->text().toStdString(),
                ui->picOutLabelP4);
}

void MainWindow::pointCloudHandler(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::fromROSMsg(*msg, pointCloud);
}

void MainWindow::timerLoop() { ros::spinOnce(); }

void MainWindow::on_openConfigButton_clicked() {
  Eigen::Matrix4d tf;
  tf.setIdentity();

  readConfig();

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

  img_ = std::make_shared<cv::Mat>(
      cv::Mat(240, 320, CV_8UC3, cv::Scalar(175, 175, 255)));  // for test only
  pc_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  *pc_ = pointCloud;
  std::shared_ptr<cv::Mat> img_mark = std::make_shared<cv::Mat>();
  img_->copyTo(*img_mark);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcc(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*pc_, *pcc);
  for (auto& p : pcc->points) {
    p.rgba = 0xffffffff;
  }

  // cv::imshow("test", *img_);
  // cv::waitKey(0);

  //    calibrator_->SetTranformation(tf_);
  calibrator_->SetTranformation(tf);
  calibrator_->Project(*pcc, *img_mark);

  img_viewer_.reset(new ImageViewer);
  img_viewer_->show();
  img_viewer_->showImage(img_mark);
  pc_viewer_.reset(new PointcloudViewer);
  pc_viewer_->show();
  pc_viewer_->showPointcloud(pcc);

  connect(ui->rx_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->ry_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->rz_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->tx_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->ty_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
  connect(ui->tz_slide, &QSlider::valueChanged, this, &MainWindow::tfProcess);
}

void MainWindow::on_finishButton_clicked() { closeImgAndPcViewers(); }

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
};

void MainWindow::closeEvent(QCloseEvent* event) {
  closeImgAndPcViewers();

  event->accept();
}

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

void MainWindow::on_Open_Dataset_Button_clicked() {
  QString dir = QFileDialog::getExistingDirectory(
      this, tr("Open Directory"), QDir::homePath(), QFileDialog::ShowDirsOnly);
  if (!dir.isEmpty()) {
    data_reader_.reset(new DataReader(dir));
    if (!data_reader_->isValid()) {
      data_reader_ = nullptr;
      QMessageBox::warning(this, tr("Warn"), tr("The directory is invalid"));
      return;
    }
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
    ui->total_data_num->setText(
        QString::number(data_reader_->getDatasetSize()));

    showTFWindow();
  }
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

void MainWindow::readConfig() {
  config_path_ = QFileDialog::getOpenFileName(this, tr("Open File"), ".",
                                              tr("Config JSON Files(*.json)"));
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
  // QList<QPushButton*> btns = ui->centralWidget->findChildren<QPushButton*>();
  // for (auto& it : btns) {
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
  std::ofstream f(config_path_.toStdString());
  if (!f.good()) {
    QMessageBox::warning(this, tr("Error"), tr("Fail to open config file"));
    return;
  }
  f << js_.dump(4);
  f.close();
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
  img_ = data_reader_->getImage();
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
  if (img_viewer_) {
    img_viewer_->close();
    img_viewer_.release();
  }
  if (pc_viewer_) {
    pc_viewer_->close();
    img_viewer_.release();
  }
}
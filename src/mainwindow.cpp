#include "mainwindow.h"

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

// QPixmap cvMat2QPixmap(const cv::Mat& inMat) {
//   cv::Mat mat;
//   cv::cvtColor(inMat, mat, cv::COLOR_BGR2RGB);
//   QImage img((uchar*)mat.data, mat.cols, mat.rows, mat.step1(),
//              QImage::Format_RGB32);
//   QPixmap pixMap = QPixmap::fromImage(img);
//   return pixMap;
// }
QPixmap MainWindow::cvMat2QPixmap(cv::Mat& inMat) {
  cv::Mat rgb;
  QImage img;
  if (inMat.channels() == 3) {
    cvtColor(inMat, rgb, cv::COLOR_BGR2RGB);
    img = QImage((const uchar*)(rgb.data), rgb.cols, rgb.rows,
                 rgb.cols * rgb.channels(), QImage::Format_RGB888);
}

  else {
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
    ui->graphicsViewP1->resize(img.width(), img.height());
    QGraphicsScene* cam_scene = new QGraphicsScene;
    cam_scene->setSceneRect(0, 0, img.width(), img.height());
    cam_scene->addPixmap(img);
    ui->graphicsViewP1->setScene(cam_scene);
    ui->graphicsViewP1->adjustSize();
    ui->graphicsViewP1->show();
    // cv::Mat tmpMat = cameraP->getFrame(value);
    // QPixmap tmpPixMap = cvMat2QPixmap(tmpMat);
    // ui->picOutLabelP1->setPixmap(tmpPixMap);
  }
}

void MainWindow::on_minExposureSliderP1_valueChanged(int value) {
  if (cameraP) {
    cv::Mat tmpMat = cameraP->getFrame(value);
    QPixmap img = cvMat2QPixmap(tmpMat);
    ui->graphicsViewP1->resize(img.width(), img.height());
    QGraphicsScene* cam_scene = new QGraphicsScene;
    cam_scene->setSceneRect(0, 0, img.width(), img.height());
    cam_scene->addPixmap(img);
    ui->graphicsViewP1->setScene(cam_scene);
    ui->graphicsViewP1->adjustSize();
    ui->graphicsViewP1->show();
    // cv::Mat tmpMat = cameraP->getFrame(value);
    // QPixmap tmpPixMap = cvMat2QPixmap(tmpMat);
    // ui->picOutLabelP1->setPixmap(tmpPixMap);
  }
}

void MainWindow::on_startButtonP1_clicked() {
  // RsCamera* cameraP = cameraP;
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
    ui->graphicsViewP1->resize(img.width(), img.height());
    QGraphicsScene* cam_scene = new QGraphicsScene;
    cam_scene->setSceneRect(0, 0, img.width(), img.height());
    cam_scene->addPixmap(img);
    ui->graphicsViewP1->setScene(cam_scene);
    ui->graphicsViewP1->adjustSize();
    ui->graphicsViewP1->show();
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
    ui->graphicsViewP3->resize(img.width(), img.height());
    QGraphicsScene* cam_scene = new QGraphicsScene;
    cam_scene->setSceneRect(0, 0, img.width(), img.height());
    cam_scene->addPixmap(img);
    ui->graphicsViewP3->setScene(cam_scene);
    ui->graphicsViewP3->adjustSize();
    ui->graphicsViewP3->show();
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
    ui->graphicsViewP3->resize(img.width(), img.height());
    QGraphicsScene* cam_scene = new QGraphicsScene;
    cam_scene->setSceneRect(0, 0, img.width(), img.height());
    cam_scene->addPixmap(img);
    ui->graphicsViewP3->setScene(cam_scene);
    ui->graphicsViewP3->adjustSize();
    ui->graphicsViewP3->show();
    // QPixmap currentPixMap = cvMat2QPixmap(currentFrame);
    // ui->picOutLabelP3->setPixmap(currentPixMap);

    findPointsInRange(currentFrame, pointsInRange, pathToCameraTxt);

    // show covered range on qt
    QPixmap img2 = cvMat2QPixmap(pointsInRange);
    QGraphicsScene* cam_scene2 = new QGraphicsScene;
    cam_scene2->setSceneRect(0, 0, img2.width(), img2.height());
    cam_scene2->addPixmap(img2);
    ui->graphicsViewP3_1->setScene(cam_scene2);
    ui->graphicsViewP3_1->adjustSize();
    ui->graphicsViewP3_1->show();
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

void MainWindow::on_Open_Config_Button_clicked() {
  config_path_ = QFileDialog::getOpenFileName(this, tr("Open File"), ".",
                                              tr("Config JSON Files(*.json)"));

  if (config_path_.isEmpty()) {
    QTimer::singleShot(0, this, &QApplication::quit);
    return;
  }

  std::ifstream f(config_path_.toStdString());
  if (!f.good()) {
    f.close();
    QTimer::singleShot(0, this, &QApplication::quit);
    return;
  }

  try {
    f >> js_;
  } catch (nlohmann::json::parse_error& e) {
    std::cerr << e.what();
    f.close();
    QTimer::singleShot(0, this, &QApplication::quit);
    return;
  }
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
};

void MainWindow::closeEvent(QCloseEvent* event) {
  if (img_viewer_) {
    img_viewer_->close();
    img_viewer_.release();
  }
  if (pc_viewer_) {
    pc_viewer_->close();
    img_viewer_.release();
  }

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

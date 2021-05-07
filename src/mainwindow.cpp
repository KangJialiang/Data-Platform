#include "mainwindow.h"

#include <QCompleter>
#include <QDebug>
#include <QLabel>
#include <QPushButton>
#include <QStringList>

#include "RsCamera.h"
#include "findPointsInRange.h"
#include "responseCalib.h"
#include "ui_mainwindow.h"
#include "vignetteCalib.h"

RsCamera camera;
RsCamera* cameraP = &camera;

QPixmap cvMat2QPixmap(const cv::Mat& inMat) {
  cv::Mat mat;
  cv::cvtColor(inMat, mat, cv::COLOR_BGR2RGB);
  QImage img((uchar*)mat.data, mat.cols, mat.rows, mat.step1(),
             QImage::Format_RGB32);
  QPixmap pixMap = QPixmap::fromImage(img);
  return pixMap;
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  this->resize(QSize(1500, 1500));
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
    QPixmap tmpPixMap = cvMat2QPixmap(tmpMat);
    ui->picOutLabelP1->setPixmap(tmpPixMap);
  }
}

void MainWindow::on_minExposureSliderP1_valueChanged(int value) {
  if (cameraP) {
    cv::Mat tmpMat = cameraP->getFrame(value);
    QPixmap tmpPixMap = cvMat2QPixmap(tmpMat);
    ui->picOutLabelP1->setPixmap(tmpPixMap);
  }
}

void MainWindow::on_startButtonP1_clicked() {
  RsCamera* cameraP = cameraP;
  std::string dataPath = ui->gammaPathLineP1->text().toStdString();
  std::string pathToCameraTxt = ui->pathToCameraLineP1->text().toStdString();
  int minExposureTime = ui->minExposureSliderP1->value();
  int maxExposureTime = ui->maxExposureSliderP1->value();
  int exposureNum = 150;
  int imgNum = 8;

  if (dataPath.back() != '/') dataPath += '/';
  if (-1 == system(("mkdir -p " + dataPath).c_str()))
    throw std::invalid_argument("Cannot create dir " + dataPath);
  system(("mkdir -p " + dataPath + "images/").c_str());
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
    int timeOfArrival;
    auto exposureTime = exposureTimes[i];
    char imgId[100];
    snprintf(imgId, 100, "%05d", i);
    char imgName[100];
    snprintf(imgName, 100, "images/%05d", i);
    cv::Mat currentFrame = cameraP->getFrame(exposureTime, timeOfArrival);
    cv::imwrite((dataPath + imgName).c_str(), currentFrame);
    timesFile << imgId << " " << timeOfArrival << " " << exposureTime / 1000.0
              << "\n";

    // show frame on qt
    QPixmap currentPixMap = cvMat2QPixmap(currentFrame);
    ui->picOutLabelP1->setPixmap(currentPixMap);
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
    QPixmap tmpPixMap = cvMat2QPixmap(tmpMat);
    ui->picOutLabelP3->setPixmap(tmpPixMap);
  }
}

void MainWindow::on_startButtonP3_clicked() {
  RsCamera* cameraP = cameraP;
  std::string dataPath = ui->vignettePathLineP3->text().toStdString();
  std::string pathToCameraTxt = ui->pathToCameraLineP3->text().toStdString();
  int exposureTime = ui->exposureSliderP3->value();
  int imgNum = 800;
  int fps = ui->fpsSliderP3->value();

  if (dataPath.back() != '/') dataPath += '/';
  if (-1 == system(("mkdir -p " + dataPath).c_str()))
    throw std::invalid_argument("Cannot create dir " + dataPath);
  system(("mkdir -p " + dataPath + "images/").c_str());
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
    snprintf(imgName, 100, "images/%05d", i);
    cv::Mat currentFrame = cameraP->getFrame(exposureTime, timeOfArrival);
    cv::imwrite((dataPath + imgName).c_str(), currentFrame);
    timesFile << imgId << " " << timeOfArrival << " " << exposureTime / 1000.0
              << "\n";

    // show current frame on qt
    QPixmap currentPixMap = cvMat2QPixmap(currentFrame);
    ui->picOutLabelP3->setPixmap(currentPixMap);

    findPointsInRange(currentFrame, pointsInRange, pathToCameraTxt);

    // show covered range on qt
    QPixmap pointsInRangePixMap = cvMat2QPixmap(pointsInRange);
    ui->pointsInRangeOutLabelP3->setPixmap(pointsInRangePixMap);
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

#include "mainwindow.h"

#include <QCompleter>
#include <QDebug>
#include <QLabel>
#include <QPushButton>
#include <QStringList>

#include "findPointsInRange.h"
#include "responseCalib.h"
#include "ui_mainwindow.h"
#include "vignetteCalib.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
}

MainWindow::~MainWindow() { delete ui; }

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
  responseCalib(ui->inOutPathLineP2->text().toStdString(), ui->shellOutTextP2,
                ui->picOutLabelP2);
}

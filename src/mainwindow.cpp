#include "mainwindow.h"

#include <QCompleter>
#include <QDebug>
#include <QLabel>
#include <QPushButton>
#include <QStringList>

#include "photometricCalib.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  // page
  //设置图片
  ui->label_24->hide();
  ui->label_24->setPixmap(QPixmap("://image/test1.jpg"));
  ui->label_24->setScaledContents(true);

  //设置输出路径
  ui->label_25->hide();
  ui->label_25->setText("在此输出路径结果");

  //切换下一页

  // page2
  //输出中间过程
  ui->label_9->setText("在此输出中间过程文字");

  // page3

  // page4
  //输出结果路径
  ui->label_26->hide();
  ui->label_26->setText("在此输出路径结果");

  // page5
  //输出结果路径
  ui->label_27->setText("在此输出路径结果");

  //输出图片
  ui->label_28->setPixmap(QPixmap("://image/test1.jpg"));
  ui->label_28->setScaledContents(true);

  // page6
  //输出中间过程文字
  ui->label_29->hide();
  ui->label_29->setText("在此输出中间过程文字");
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_pushButton_2_clicked() {
  ui->stackedWidget->setCurrentIndex(1);
}

void MainWindow::on_pushButton_5_clicked() {
  ui->stackedWidget->setCurrentIndex(2);
}

void MainWindow::on_pushButton_8_clicked() {
  ui->stackedWidget->setCurrentIndex(3);
}

void MainWindow::on_pushButton_9_clicked() {
  ui->stackedWidget->setCurrentIndex(2);
}

void MainWindow::on_pushButton_12_clicked() {
  ui->stackedWidget->setCurrentIndex(4);
}

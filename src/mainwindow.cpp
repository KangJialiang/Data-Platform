#include "include/mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>
#include "QApplication"
#include <QTextBrowser>
#include <QString>
#include <QGraphicsScene>
#include <QList>
#include <QGraphicsView>

#include <iostream>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/serial.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    conti = true;
    //originalRenderArea = new RenderArea(this);
    //originalRenderArea->setGeometry(250,230,850,300);
    //originalRenderArea->show();
    qDebug()<<"mainwindow is running on thread:"<<QThread::currentThreadId();
    camera.moveToThread(&cam_th);
    QObject::connect(this,SIGNAL(cam_start()),&camera,SLOT(get_img()));
    QObject::connect(this,SIGNAL(cam_look()),&camera,SLOT(look_img()));
    QObject::connect(&camera,SIGNAL(send_img(QImage)),this,SLOT(visual_image(QImage)));
    QObject::connect(&camera,SIGNAL(file_error(char*)),this,SLOT(camera_error(char*)));
    QObject::connect(ui->pushButton_2,SIGNAL(clicked()),&camera,SLOT(exit_collect()));
    QObject::connect(&camera,SIGNAL(imu_data(float*)),this,SLOT(visual_imu(float*)));
    //if (ui->checkBox->isChecked())
    cam_th.start();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete scene;
}

int64_t CurrentTimeMillis()
{
    int64_t timems = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return timems;
}

int get_baud(int baud)
{
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        return -1;
    }
}

void MainWindow::on_pushButton_clicked()
{

    std::ofstream ftxt;
    std::ofstream outF;
    if(!ui->textEdit_10->document()->isEmpty())
    {
      camera.exposure = ui->textEdit_10->toPlainText().toInt();
    }

    if (ui->textEdit_11->document()->isEmpty()&&(ui->checkBox->isChecked()|| ui->checkBox_2->isChecked()))
    {
        QMessageBox::critical(this,"path empty","You should enter logging path before starting") ;
        this->close();
        return;
    }
    else if(ui->checkBox->isChecked()|| ui->checkBox_2->isChecked()){
        QString dir = ui->textEdit_11->toPlainText();
        strcpy(dir_name,dir.toLatin1().data());
        if (access(dir_name, 0)) {
            if (mkdir(dir_name, 0777)) {
            QMessageBox::critical(this,"path failed","create dataset directory failed") ;
            this->close();
            return;
            }
          }
        strcpy(camera.dir_name,dir_name);
    }
//    strcpy(dir_name,"/media/starbike/Samsung_T5/tongji_odom/20201210");
//    strcpy(camera.dir_name,dir_name);

    if (ui->checkBox->isChecked())
        emit cam_start();
    else {
        emit cam_look();
    }
    int serial_port;
    if (ui->checkBox_2->isChecked()){
        char gps_dir[100];
        char bin_dir[100];
        sprintf(gps_dir, "%s/gps_data.txt", dir_name);
        ftxt.open(gps_dir, std::ios::out);
        sprintf(bin_dir, "%s/messages.bin", dir_name);
        outF.open(bin_dir,std::ios::out | std::ios::binary);
        if (!(ftxt.is_open()&&outF.is_open()))
        //if (!(ftxt.is_open()))
        {
            QMessageBox::critical(this,"open failed","failed to open gps files") ;
            this->close();
        }
    //}
    QString portq = ui->comboBox->currentText();
    QString databitq = ui->comboBox_3->currentText();
    QString stopbitq = ui->comboBox_4->currentText();//read comboBox choices for initialize

    serial_port = open("/dev/ttyUSB0", O_RDWR); // USB path need to check，这部分换成虚拟串口
    if (serial_port < 0)
    {
        QString err = "Error "+ QString::number(errno,10) +"from open: "+strerror(errno)+"/n";
        QMessageBox::critical(this,"USB open failed",err) ;
        this->close();
        return;
    }
    qDebug()<<"USB opened successfully!";
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0)
    {
        QString err = "Error "+ QString::number(errno,10) +"from tcgetattr: "+strerror(errno)+"/n";
        this->close();
        return;
    }
    qDebug()<<"getattr sucessfully";


    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 158;

    speed_t baudin;
    QString baudqin = ui->comboBox_2->currentText();
    baudin = get_baud(baudqin.toInt());
    cfsetispeed(&tty, baudin);
    cfsetospeed(&tty, baudin);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        QString err = "Error "+ QString::number(errno,10) +"from tcsetattr: "+strerror(errno)+"/n";
        QMessageBox::critical(this,"USB open failed",err) ;
        this->close();
        return;
    }
    qDebug()<<"Port initialized successfully";
    }
    while (conti)
    {
        if (ui->checkBox_2->isChecked()){
        unsigned char buffer_char[158];
        memset(buffer_char, '\0', 158*sizeof(unsigned char));  // empty this char
        int n = read(serial_port, &buffer_char, 158*sizeof(unsigned char));   // each time read in one-bytes
        outF.write(reinterpret_cast<char*>(buffer_char), sizeof(buffer_char));
        int64_t timestamp1=CurrentTimeMillis();
        qDebug()<<"received " << n << "bytes";

        unsigned char* c = buffer_char;
        for (int i = 0; i < n; i++, c++)
        {

                if ((*c == 0xaa) && (*(c+1) == 0x44) && (*(c+2) == 0x12))
                {
                    c += 16;
                    long *second = reinterpret_cast<long*>(c);
                    c += 16;
                    int *i_status = reinterpret_cast<int*>(c);
                    c += 4;
                    double *d_lat = reinterpret_cast<double*>(c);
                    c += 8;
                    double *d_long = reinterpret_cast<double*>(c);
                    c +=8;
                    double *d_alt = reinterpret_cast<double*>(c);
                    c +=36;
                    double *d_roll = reinterpret_cast<double*>(c);
                    c +=8;
                    double *d_pitch = reinterpret_cast<double*>(c);
                    c += 8;
                    double *d_heading = reinterpret_cast<double*>(c);
                    //long timestamp = (*week)*604800000+*second+315964800000;

                    qDebug()<<QString::number(*d_lat, 'g', 18)<<QString::number(*d_long, 'g', 18);
                    if((*d_lat>30)&&(*d_long>120)&&(*d_long<122)&&(*d_lat<32))
                    {
                        xList.append(*d_lat);
                        yList.append(*d_long);
                        ui->textEdit_9->setText(QString::number(timestamp1, 'g', 18));
                        ui->textEdit_8->setText(QString::number(*i_status));
                        ui->textEdit_6->setText(QString::number(*d_lat, 'g', 18));
                        ui->textEdit_7->setText(QString::number(*d_long, 'g', 18));
                        ui->textEdit_5->setText(QString::number(*d_heading, 'g'));
                        if(ui->checkBox_2->isChecked()){

                            ftxt << std::setprecision(18) << timestamp1 << "\t";
                            ftxt << std::setprecision(18) << *second << "\t";
                            ftxt << std::setprecision(18) << *d_lat << "\t";
                            ftxt << std::setprecision(18) << *d_long << "\t";
                            ftxt << std::setprecision(18) << *d_alt << "\t";
                            ftxt << std::setprecision(18) << *d_roll << "\t";
                            ftxt << std::setprecision(18) << *d_pitch << "\t";
                            ftxt << std::setprecision(18) << *d_heading << "\t";
                            ftxt << *i_status << std::endl;
                        }
                    }

                }
            }
            QCoreApplication::processEvents();//visualizing code ever after, needn't care
        }
        QCoreApplication::processEvents();//visualizing code ever after, needn't care
    }
    if (ui->checkBox_2->isChecked()){
    ::close(serial_port);
    ftxt.close();
    outF.close();
    }
    this->close();
    return;
}

void MainWindow::on_pushButton_2_clicked()
{
    QMessageBox::StandardButton reply;
            reply= QMessageBox::warning(this, "Quit", "Quit?", QMessageBox::Yes| QMessageBox::No);
            if (reply == QMessageBox::No){}
            else{
                cam_th.quit();
                conti=false;
            }
}

void MainWindow::camera_error(char *dirname)
{
    char err[200] = "Create failed:";
    strcat(err,dirname);
    QMessageBox::critical(this,"path failed",err) ;
    cam_th.quit();
    conti=false;
}

void MainWindow::visual_image(QImage img){
    qDebug()<<"mainwindow got camera image to visualize";
    ui->graphicsView->resize(img.width(), img.height());
    QGraphicsScene *cam_scene = new QGraphicsScene;
    cam_scene->setSceneRect(0,0,img.width(),img.height());
    cam_scene->addPixmap(QPixmap::fromImage(img));
    ui->graphicsView->setScene(cam_scene);
    ui->graphicsView->adjustSize();
    ui->graphicsView->show();
    qDebug()<<"showed image in graphicsView";
}

void MainWindow::visual_imu(float* imu)
{
    qDebug()<<"visualizing imu";
    ui->textEdit_12->setText(QString::number(*imu,'g'));
    ui->textEdit_13->setText(QString::number(*(imu+1),'g'));
    ui->textEdit_14->setText(QString::number(*(imu+2),'g'));
    ui->textEdit_17->setText(QString::number(*(imu+3),'g'));
    ui->textEdit_15->setText(QString::number(*(imu+4),'g'));
    ui->textEdit_16->setText(QString::number(*(imu+5),'g'));
}






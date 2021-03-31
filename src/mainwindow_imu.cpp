#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "visual_traj.h"
#include <QMessageBox>
#include <QDebug>
#include "QApplication"
#include <QTextBrowser>
#include <QString>
#include <QGraphicsScene>
#include <QList>

#include <iostream>
#include <chrono>
#include <cstring>
#include <string>
#include <fstream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <vector>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    conti = true;

}

MainWindow::~MainWindow()
{
    delete ui;
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

int64_t CurrentTimeMillis()
{
    int64_t timems = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return timems;
}

void MainWindow::on_pushButton_clicked()
{
    std::ofstream ftxt;
    QGraphicsScene *scene = new QGraphicsScene;
    ftxt.open("../imu_data.txt", std::ios::out | std::ios::app);
    if (!ftxt.is_open())
    {
        QMessageBox::critical(this,"open failed","failed to open data.txt") ;
        this->close();
    }

    QString portq = ui->comboBox->currentText();
    QString databitq = ui->comboBox_3->currentText();
    QString stopbitq = ui->comboBox_4->currentText();//read comboBox choices for initialize

    int serial_port = open("/dev/ttyUSB0", O_RDWR); // USB path need to check，这部分换成虚拟串口
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

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

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
    QList<double> xList;
    QList<double> yList;


    while (conti)
    {
        unsigned char* buffer_char;
        std::vector<unsigned char> line;
        int i=0;
        read(serial_port, buffer_char, sizeof(unsigned char));   // each time read in one-bytes
        while(!(*buffer_char==0x2a))
        {
            if(!(*buffer_char==0x20))
            {
                line.push_back(*buffer_char);
                i++;
                ftxt<<*buffer_char;
            }
            read(serial_port, buffer_char, sizeof(unsigned char));
        }
        ftxt<<std::endl;
        unsigned char charary[line.size()];
        //vector全部转到数组
        memcpy(charary, &line[0], line.size() * sizeof(line[0]));
        char *str1=(char*)charary;
        QString buff_str=QString(QLatin1String(str1));
        qDebug()<<buff_str;
        //std::vector<unsigned char> tmp= line;
        //line.swap(tmp);// empty this char
        memset(charary,'\0',line.size()*sizeof(unsigned char));
        line.clear();
        QStringList itemlist=buff_str.split(',');

        QString main_count=itemlist.at(0);
        qDebug()<<main_count;
        QString v_speed=itemlist.at(14);
        qDebug()<<v_speed;
        QString gps_quality=itemlist.at(15);
        qDebug()<<gps_quality;
        QString gps_soln=itemlist.at(21);
        qDebug()<<gps_soln;
        QString Latitude_GNSS_Int_str = itemlist.at(8);
        double Latitude_GNSS_Int = Latitude_GNSS_Int_str.toDouble();
        QString Longitude_GNSS_Int_str = itemlist.at(9);
        double Longitude_GNSS_Int = Longitude_GNSS_Int_str.toDouble();
        QString Latitude_GNSS_Dec_str = itemlist.at(10);
        double Latitude_GNSS_Dec = Latitude_GNSS_Dec_str.toDouble();
        QString Longitude_GNSS_Dec_str = itemlist.at(11);
        double Longitude_GNSS_Dec = Longitude_GNSS_Dec_str.toDouble();
        double xtmp = Longitude_GNSS_Int + Longitude_GNSS_Dec/1e7;
        double ytmp = Latitude_GNSS_Int + Latitude_GNSS_Dec/1e7;
        qDebug()<<xtmp;
        qDebug()<<ytmp;
        xList.append(xtmp);
        yList.append(ytmp);

        QCoreApplication::processEvents();
        ui->textEdit->setText(main_count);
        ui->textEdit_3->setText(gps_quality);
        ui->textEdit_2->setText(v_speed);
        ui->textEdit_4->setText(gps_soln);
        if (xList.count()>2){
            scene->addLine(xList.at(xList.count()-2),yList.at(yList.count()-2),xtmp,ytmp);
            ui->graphicsView->setScene(scene);
            ui->graphicsView->centerOn(xList.at(0),yList.at(0));
            ui->graphicsView->show();
        }
        usleep(40 * 1000);
        //conti=false;
    }
    ::close(serial_port);
    ftxt.close();
    this->close();
    return;
}

void MainWindow::on_pushButton_2_clicked()
{
    QMessageBox::StandardButton reply;
            reply= QMessageBox::warning(this, "Quit", "Quit?", QMessageBox::Yes| QMessageBox::No);
            if (reply == QMessageBox::No){}
            else{
                conti=false;
            }}




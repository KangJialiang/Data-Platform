/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QLabel *label_4;
    QComboBox *comboBox_3;
    QComboBox *comboBox_4;
    QLabel *label_3;
    QComboBox *comboBox;
    QLabel *label_2;
    QComboBox *comboBox_2;
    QLabel *label;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout;
    QCheckBox *checkBox_2;
    QCheckBox *checkBox;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_13;
    QTextEdit *textEdit_9;
    QLabel *label_18;
    QTextEdit *textEdit_8;
    QWidget *layoutWidget3;
    QHBoxLayout *horizontalLayout;
    QLabel *label_6;
    QLabel *label_7;
    QTextEdit *textEdit_7;
    QLabel *label_8;
    QTextEdit *textEdit_6;
    QLabel *label_9;
    QTextEdit *textEdit_5;
    QGraphicsView *graphicsView;
    QWidget *layoutWidget4;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_5;
    QTextEdit *textEdit;
    QLabel *label_15;
    QTextEdit *textEdit_3;
    QLabel *label_14;
    QTextEdit *textEdit_2;
    QLabel *label_20;
    QTextEdit *textEdit_4;
    QTextEdit *textEdit_11;
    QLabel *label_10;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1171, 533);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(50, 20, 156, 196));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 6, 0, 1, 1);

        comboBox_3 = new QComboBox(layoutWidget);
        comboBox_3->addItem(QString());
        comboBox_3->addItem(QString());
        comboBox_3->addItem(QString());
        comboBox_3->setObjectName(QString::fromUtf8("comboBox_3"));

        gridLayout->addWidget(comboBox_3, 3, 1, 1, 1);

        comboBox_4 = new QComboBox(layoutWidget);
        comboBox_4->addItem(QString());
        comboBox_4->addItem(QString());
        comboBox_4->setObjectName(QString::fromUtf8("comboBox_4"));

        gridLayout->addWidget(comboBox_4, 6, 1, 1, 1);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 3, 0, 1, 1);

        comboBox = new QComboBox(layoutWidget);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        gridLayout->addWidget(comboBox, 1, 1, 1, 1);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        comboBox_2 = new QComboBox(layoutWidget);
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));
        comboBox_2->setEditable(false);

        gridLayout->addWidget(comboBox_2, 2, 1, 1, 1);

        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(60, 280, 113, 178));
        verticalLayout = new QVBoxLayout(layoutWidget1);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        checkBox_2 = new QCheckBox(layoutWidget1);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));

        verticalLayout->addWidget(checkBox_2);

        checkBox = new QCheckBox(layoutWidget1);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));

        verticalLayout->addWidget(checkBox);

        pushButton = new QPushButton(layoutWidget1);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout->addWidget(pushButton);

        pushButton_2 = new QPushButton(layoutWidget1);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setCheckable(false);

        verticalLayout->addWidget(pushButton_2);

        layoutWidget2 = new QWidget(centralWidget);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(240, 100, 871, 47));
        horizontalLayout_4 = new QHBoxLayout(layoutWidget2);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_13 = new QLabel(layoutWidget2);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_13->sizePolicy().hasHeightForWidth());
        label_13->setSizePolicy(sizePolicy);

        horizontalLayout_4->addWidget(label_13);

        textEdit_9 = new QTextEdit(layoutWidget2);
        textEdit_9->setObjectName(QString::fromUtf8("textEdit_9"));
        textEdit_9->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(textEdit_9->sizePolicy().hasHeightForWidth());
        textEdit_9->setSizePolicy(sizePolicy1);
        textEdit_9->setMinimumSize(QSize(100, 45));
        textEdit_9->setMaximumSize(QSize(250, 45));

        horizontalLayout_4->addWidget(textEdit_9);

        label_18 = new QLabel(layoutWidget2);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        sizePolicy.setHeightForWidth(label_18->sizePolicy().hasHeightForWidth());
        label_18->setSizePolicy(sizePolicy);

        horizontalLayout_4->addWidget(label_18);

        textEdit_8 = new QTextEdit(layoutWidget2);
        textEdit_8->setObjectName(QString::fromUtf8("textEdit_8"));
        textEdit_8->setEnabled(true);
        sizePolicy1.setHeightForWidth(textEdit_8->sizePolicy().hasHeightForWidth());
        textEdit_8->setSizePolicy(sizePolicy1);
        textEdit_8->setMinimumSize(QSize(100, 45));
        textEdit_8->setMaximumSize(QSize(100, 45));

        horizontalLayout_4->addWidget(textEdit_8);

        layoutWidget3 = new QWidget(centralWidget);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(210, 50, 961, 51));
        horizontalLayout = new QHBoxLayout(layoutWidget3);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label_6 = new QLabel(layoutWidget3);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(label_6);

        label_7 = new QLabel(layoutWidget3);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);
        label_7->setMaximumSize(QSize(16777215, 8777173));

        horizontalLayout->addWidget(label_7);

        textEdit_7 = new QTextEdit(layoutWidget3);
        textEdit_7->setObjectName(QString::fromUtf8("textEdit_7"));
        textEdit_7->setEnabled(true);
        sizePolicy1.setHeightForWidth(textEdit_7->sizePolicy().hasHeightForWidth());
        textEdit_7->setSizePolicy(sizePolicy1);
        textEdit_7->setMinimumSize(QSize(100, 45));
        textEdit_7->setMaximumSize(QSize(225, 45));

        horizontalLayout->addWidget(textEdit_7);

        label_8 = new QLabel(layoutWidget3);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(label_8);

        textEdit_6 = new QTextEdit(layoutWidget3);
        textEdit_6->setObjectName(QString::fromUtf8("textEdit_6"));
        textEdit_6->setEnabled(true);
        sizePolicy1.setHeightForWidth(textEdit_6->sizePolicy().hasHeightForWidth());
        textEdit_6->setSizePolicy(sizePolicy1);
        textEdit_6->setMinimumSize(QSize(100, 45));
        textEdit_6->setMaximumSize(QSize(225, 45));

        horizontalLayout->addWidget(textEdit_6);

        label_9 = new QLabel(layoutWidget3);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        sizePolicy.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(label_9);

        textEdit_5 = new QTextEdit(layoutWidget3);
        textEdit_5->setObjectName(QString::fromUtf8("textEdit_5"));
        textEdit_5->setEnabled(true);
        sizePolicy1.setHeightForWidth(textEdit_5->sizePolicy().hasHeightForWidth());
        textEdit_5->setSizePolicy(sizePolicy1);
        textEdit_5->setMinimumSize(QSize(100, 45));
        textEdit_5->setMaximumSize(QSize(225, 45));

        horizontalLayout->addWidget(textEdit_5);

        graphicsView = new QGraphicsView(centralWidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(340, 220, 320, 240));
        layoutWidget4 = new QWidget(centralWidget);
        layoutWidget4->setObjectName(QString::fromUtf8("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(210, 0, 939, 47));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget4);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_5 = new QLabel(layoutWidget4);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(label_5);

        textEdit = new QTextEdit(layoutWidget4);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        textEdit->setEnabled(true);
        sizePolicy1.setHeightForWidth(textEdit->sizePolicy().hasHeightForWidth());
        textEdit->setSizePolicy(sizePolicy1);
        textEdit->setMinimumSize(QSize(100, 45));
        textEdit->setMaximumSize(QSize(100, 45));

        horizontalLayout_2->addWidget(textEdit);

        label_15 = new QLabel(layoutWidget4);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        sizePolicy.setHeightForWidth(label_15->sizePolicy().hasHeightForWidth());
        label_15->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(label_15);

        textEdit_3 = new QTextEdit(layoutWidget4);
        textEdit_3->setObjectName(QString::fromUtf8("textEdit_3"));
        textEdit_3->setEnabled(true);
        sizePolicy1.setHeightForWidth(textEdit_3->sizePolicy().hasHeightForWidth());
        textEdit_3->setSizePolicy(sizePolicy1);
        textEdit_3->setMinimumSize(QSize(100, 45));
        textEdit_3->setMaximumSize(QSize(100, 45));

        horizontalLayout_2->addWidget(textEdit_3);

        label_14 = new QLabel(layoutWidget4);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        sizePolicy.setHeightForWidth(label_14->sizePolicy().hasHeightForWidth());
        label_14->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(label_14);

        textEdit_2 = new QTextEdit(layoutWidget4);
        textEdit_2->setObjectName(QString::fromUtf8("textEdit_2"));
        textEdit_2->setEnabled(true);
        sizePolicy1.setHeightForWidth(textEdit_2->sizePolicy().hasHeightForWidth());
        textEdit_2->setSizePolicy(sizePolicy1);
        textEdit_2->setMinimumSize(QSize(100, 45));
        textEdit_2->setMaximumSize(QSize(100, 45));

        horizontalLayout_2->addWidget(textEdit_2);

        label_20 = new QLabel(layoutWidget4);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        sizePolicy.setHeightForWidth(label_20->sizePolicy().hasHeightForWidth());
        label_20->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(label_20);

        textEdit_4 = new QTextEdit(layoutWidget4);
        textEdit_4->setObjectName(QString::fromUtf8("textEdit_4"));
        textEdit_4->setEnabled(true);
        sizePolicy1.setHeightForWidth(textEdit_4->sizePolicy().hasHeightForWidth());
        textEdit_4->setSizePolicy(sizePolicy1);
        textEdit_4->setMinimumSize(QSize(100, 45));
        textEdit_4->setMaximumSize(QSize(100, 45));

        horizontalLayout_2->addWidget(textEdit_4);

        textEdit_11 = new QTextEdit(centralWidget);
        textEdit_11->setObjectName(QString::fromUtf8("textEdit_11"));
        textEdit_11->setGeometry(QRect(340, 160, 771, 31));
        sizePolicy.setHeightForWidth(textEdit_11->sizePolicy().hasHeightForWidth());
        textEdit_11->setSizePolicy(sizePolicy);
        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(230, 167, 101, 20));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1171, 28));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
#ifndef QT_NO_SHORTCUT
        label->setBuddy(comboBox);
#endif // QT_NO_SHORTCUT

        retranslateUi(MainWindow);

        comboBox_2->setCurrentIndex(6);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Stop Bit", nullptr));
        comboBox_3->setItemText(0, QApplication::translate("MainWindow", "6", nullptr));
        comboBox_3->setItemText(1, QApplication::translate("MainWindow", "7", nullptr));
        comboBox_3->setItemText(2, QApplication::translate("MainWindow", "8", nullptr));

        comboBox_4->setItemText(0, QApplication::translate("MainWindow", "1", nullptr));
        comboBox_4->setItemText(1, QApplication::translate("MainWindow", "2", nullptr));

        label_3->setText(QApplication::translate("MainWindow", "Data Bit", nullptr));
        comboBox->setItemText(0, QApplication::translate("MainWindow", "COM1", nullptr));
        comboBox->setItemText(1, QApplication::translate("MainWindow", "COM2", nullptr));
        comboBox->setItemText(2, QApplication::translate("MainWindow", "COM3", nullptr));
        comboBox->setItemText(3, QApplication::translate("MainWindow", "COM4", nullptr));
        comboBox->setItemText(4, QApplication::translate("MainWindow", "COM5", nullptr));
        comboBox->setItemText(5, QApplication::translate("MainWindow", "COM6", nullptr));
        comboBox->setItemText(6, QApplication::translate("MainWindow", "COM7", nullptr));
        comboBox->setItemText(7, QApplication::translate("MainWindow", "COM8", nullptr));

        label_2->setText(QApplication::translate("MainWindow", "Baudrate", nullptr));
        comboBox_2->setItemText(0, QApplication::translate("MainWindow", "9600", nullptr));
        comboBox_2->setItemText(1, QApplication::translate("MainWindow", "19200", nullptr));
        comboBox_2->setItemText(2, QApplication::translate("MainWindow", "38400", nullptr));
        comboBox_2->setItemText(3, QApplication::translate("MainWindow", "57600", nullptr));
        comboBox_2->setItemText(4, QApplication::translate("MainWindow", "115200", nullptr));
        comboBox_2->setItemText(5, QApplication::translate("MainWindow", "230400", nullptr));
        comboBox_2->setItemText(6, QApplication::translate("MainWindow", "460800", nullptr));

        label->setText(QApplication::translate("MainWindow", "COM", nullptr));
        checkBox_2->setText(QApplication::translate("MainWindow", "GNSS", nullptr));
        checkBox->setText(QApplication::translate("MainWindow", "Camera", nullptr));
        pushButton->setText(QApplication::translate("MainWindow", "Start", nullptr));
        pushButton_2->setText(QApplication::translate("MainWindow", "Close All", nullptr));
        label_13->setText(QApplication::translate("MainWindow", "timestamp", nullptr));
        label_18->setText(QApplication::translate("MainWindow", "GPS status", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "GPS Data", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "Lon", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "Lat", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "Heading", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "main10msCount", nullptr));
        label_15->setText(QApplication::translate("MainWindow", "QualityGNSS", nullptr));
        label_14->setText(QApplication::translate("MainWindow", "speed", nullptr));
        label_20->setText(QApplication::translate("MainWindow", "SolnSVsGNSS", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "logging path:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

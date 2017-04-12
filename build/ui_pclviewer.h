/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QGridLayout *gridLayout;
    QLabel *depthlb_2;
    QPushButton *minusButton;
    QPushButton *plusButton;
    QPushButton *pushButton_load;
    QLabel *depthlb;
    QSlider *horizontalSlider;
    QLCDNumber *lcdNumber;
    QLabel *imagelb;
    QPushButton *pushButton_save;
    QSpacerItem *verticalSpacer;
    QGridLayout *gridLayout_2;
    QLabel *label;
    QLabel *pointNumber;
    QVTKWidget *qvtkWidget;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QString::fromUtf8("PCLViewer"));
        PCLViewer->setWindowModality(Qt::NonModal);
        PCLViewer->resize(917, 784);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        depthlb_2 = new QLabel(centralwidget);
        depthlb_2->setObjectName(QString::fromUtf8("depthlb_2"));
        depthlb_2->setMinimumSize(QSize(199, 156));

        gridLayout->addWidget(depthlb_2, 2, 0, 1, 2);

        minusButton = new QPushButton(centralwidget);
        minusButton->setObjectName(QString::fromUtf8("minusButton"));

        gridLayout->addWidget(minusButton, 7, 0, 1, 1);

        plusButton = new QPushButton(centralwidget);
        plusButton->setObjectName(QString::fromUtf8("plusButton"));

        gridLayout->addWidget(plusButton, 7, 1, 1, 1);

        pushButton_load = new QPushButton(centralwidget);
        pushButton_load->setObjectName(QString::fromUtf8("pushButton_load"));
        pushButton_load->setMinimumSize(QSize(50, 40));

        gridLayout->addWidget(pushButton_load, 8, 1, 1, 1);

        depthlb = new QLabel(centralwidget);
        depthlb->setObjectName(QString::fromUtf8("depthlb"));
        depthlb->setMinimumSize(QSize(199, 156));

        gridLayout->addWidget(depthlb, 1, 0, 1, 2);

        horizontalSlider = new QSlider(centralwidget);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider, 5, 0, 1, 2);

        lcdNumber = new QLCDNumber(centralwidget);
        lcdNumber->setObjectName(QString::fromUtf8("lcdNumber"));
        lcdNumber->setDigitCount(10);
        lcdNumber->setSegmentStyle(QLCDNumber::Flat);

        gridLayout->addWidget(lcdNumber, 4, 0, 1, 2);

        imagelb = new QLabel(centralwidget);
        imagelb->setObjectName(QString::fromUtf8("imagelb"));
        imagelb->setMinimumSize(QSize(199, 157));

        gridLayout->addWidget(imagelb, 0, 0, 1, 2);

        pushButton_save = new QPushButton(centralwidget);
        pushButton_save->setObjectName(QString::fromUtf8("pushButton_save"));
        pushButton_save->setMinimumSize(QSize(50, 40));

        gridLayout->addWidget(pushButton_save, 8, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 3, 0, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 1, 0, 1, 1);

        pointNumber = new QLabel(centralwidget);
        pointNumber->setObjectName(QString::fromUtf8("pointNumber"));

        gridLayout_2->addWidget(pointNumber, 1, 1, 1, 1);

        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(50);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy);
        qvtkWidget->setMinimumSize(QSize(690, 480));

        gridLayout_2->addWidget(qvtkWidget, 0, 0, 1, 2);


        horizontalLayout->addLayout(gridLayout_2);

        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0, QApplication::UnicodeUTF8));
        depthlb_2->setText(QString());
        minusButton->setText(QApplication::translate("PCLViewer", "-", 0, QApplication::UnicodeUTF8));
        plusButton->setText(QApplication::translate("PCLViewer", "+", 0, QApplication::UnicodeUTF8));
        pushButton_load->setText(QApplication::translate("PCLViewer", "Load file", 0, QApplication::UnicodeUTF8));
        depthlb->setText(QString());
        imagelb->setText(QString());
        pushButton_save->setText(QApplication::translate("PCLViewer", "Save file", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("PCLViewer", "Number of points:", 0, QApplication::UnicodeUTF8));
        pointNumber->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H

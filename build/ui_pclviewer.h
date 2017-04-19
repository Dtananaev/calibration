/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QWidget>
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
    QLabel *gimagelb;
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
            PCLViewer->setObjectName(QStringLiteral("PCLViewer"));
        PCLViewer->setWindowModality(Qt::NonModal);
        PCLViewer->resize(917, 784);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        depthlb_2 = new QLabel(centralwidget);
        depthlb_2->setObjectName(QStringLiteral("depthlb_2"));
        depthlb_2->setMinimumSize(QSize(199, 156));

        gridLayout->addWidget(depthlb_2, 2, 0, 1, 2);

        minusButton = new QPushButton(centralwidget);
        minusButton->setObjectName(QStringLiteral("minusButton"));

        gridLayout->addWidget(minusButton, 7, 0, 1, 1);

        plusButton = new QPushButton(centralwidget);
        plusButton->setObjectName(QStringLiteral("plusButton"));

        gridLayout->addWidget(plusButton, 7, 1, 1, 1);

        pushButton_load = new QPushButton(centralwidget);
        pushButton_load->setObjectName(QStringLiteral("pushButton_load"));
        pushButton_load->setMinimumSize(QSize(50, 40));

        gridLayout->addWidget(pushButton_load, 8, 1, 1, 1);

        gimagelb = new QLabel(centralwidget);
        gimagelb->setObjectName(QStringLiteral("gimagelb"));
        gimagelb->setMinimumSize(QSize(199, 156));

        gridLayout->addWidget(gimagelb, 1, 0, 1, 2);

        horizontalSlider = new QSlider(centralwidget);
        horizontalSlider->setObjectName(QStringLiteral("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(horizontalSlider, 5, 0, 1, 2);

        lcdNumber = new QLCDNumber(centralwidget);
        lcdNumber->setObjectName(QStringLiteral("lcdNumber"));
        lcdNumber->setDigitCount(10);
        lcdNumber->setSegmentStyle(QLCDNumber::Flat);

        gridLayout->addWidget(lcdNumber, 4, 0, 1, 2);

        imagelb = new QLabel(centralwidget);
        imagelb->setObjectName(QStringLiteral("imagelb"));
        imagelb->setMinimumSize(QSize(199, 157));

        gridLayout->addWidget(imagelb, 0, 0, 1, 2);

        pushButton_save = new QPushButton(centralwidget);
        pushButton_save->setObjectName(QStringLiteral("pushButton_save"));
        pushButton_save->setMinimumSize(QSize(50, 40));

        gridLayout->addWidget(pushButton_save, 8, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 3, 0, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        label = new QLabel(centralwidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout_2->addWidget(label, 1, 0, 1, 1);

        pointNumber = new QLabel(centralwidget);
        pointNumber->setObjectName(QStringLiteral("pointNumber"));

        gridLayout_2->addWidget(pointNumber, 1, 1, 1, 1);

        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
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
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0));
        depthlb_2->setText(QString());
        minusButton->setText(QApplication::translate("PCLViewer", "-", 0));
        plusButton->setText(QApplication::translate("PCLViewer", "+", 0));
        pushButton_load->setText(QApplication::translate("PCLViewer", "Load file", 0));
        gimagelb->setText(QString());
        imagelb->setText(QString());
        pushButton_save->setText(QApplication::translate("PCLViewer", "Save file", 0));
        label->setText(QApplication::translate("PCLViewer", "Number of points:", 0));
        pointNumber->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H

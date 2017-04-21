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
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QVTKWidget *qvtkWidget;
    QFrame *line;
    QLabel *imagelb;
    QHBoxLayout *horizontalLayout_3;
    QHBoxLayout *horizontalLayout_2;
    QSlider *horizontalSlider;
    QLCDNumber *lcdNumber;
    QPushButton *minusButton;
    QPushButton *plusButton;
    QGridLayout *gridLayout_2;
    QLabel *label;
    QLabel *status;
    QCheckBox *checkBox;
    QLabel *label_2;
    QLabel *numCorners;
    QPushButton *calibrateButton;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QStringLiteral("PCLViewer"));
        PCLViewer->setWindowModality(Qt::NonModal);
        PCLViewer->resize(940, 577);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(50);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy);
        qvtkWidget->setMinimumSize(QSize(371, 371));

        horizontalLayout->addWidget(qvtkWidget);

        line = new QFrame(centralwidget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout->addWidget(line);

        imagelb = new QLabel(centralwidget);
        imagelb->setObjectName(QStringLiteral("imagelb"));
        imagelb->setMinimumSize(QSize(411, 321));

        horizontalLayout->addWidget(imagelb);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalSlider = new QSlider(centralwidget);
        horizontalSlider->setObjectName(QStringLiteral("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(horizontalSlider);


        horizontalLayout_3->addLayout(horizontalLayout_2);

        lcdNumber = new QLCDNumber(centralwidget);
        lcdNumber->setObjectName(QStringLiteral("lcdNumber"));
        lcdNumber->setDigitCount(10);
        lcdNumber->setSegmentStyle(QLCDNumber::Flat);

        horizontalLayout_3->addWidget(lcdNumber);

        minusButton = new QPushButton(centralwidget);
        minusButton->setObjectName(QStringLiteral("minusButton"));

        horizontalLayout_3->addWidget(minusButton);

        plusButton = new QPushButton(centralwidget);
        plusButton->setObjectName(QStringLiteral("plusButton"));

        horizontalLayout_3->addWidget(plusButton);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        label = new QLabel(centralwidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout_2->addWidget(label, 0, 0, 1, 1);

        status = new QLabel(centralwidget);
        status->setObjectName(QStringLiteral("status"));

        gridLayout_2->addWidget(status, 0, 1, 1, 2);

        checkBox = new QCheckBox(centralwidget);
        checkBox->setObjectName(QStringLiteral("checkBox"));

        gridLayout_2->addWidget(checkBox, 1, 0, 1, 3);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout_2->addWidget(label_2, 2, 0, 1, 2);

        numCorners = new QLabel(centralwidget);
        numCorners->setObjectName(QStringLiteral("numCorners"));

        gridLayout_2->addWidget(numCorners, 2, 2, 1, 1);


        horizontalLayout_3->addLayout(gridLayout_2);

        calibrateButton = new QPushButton(centralwidget);
        calibrateButton->setObjectName(QStringLiteral("calibrateButton"));
        calibrateButton->setMinimumSize(QSize(191, 71));

        horizontalLayout_3->addWidget(calibrateButton);


        verticalLayout->addLayout(horizontalLayout_3);

        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", 0));
        imagelb->setText(QString());
        minusButton->setText(QApplication::translate("PCLViewer", "-", 0));
        plusButton->setText(QApplication::translate("PCLViewer", "+", 0));
        label->setText(QApplication::translate("PCLViewer", "Pictures for calibration:", 0));
        status->setText(QString());
        checkBox->setText(QApplication::translate("PCLViewer", "use for calibration", 0));
        label_2->setText(QApplication::translate("PCLViewer", "Number of detected corners:", 0));
        numCorners->setText(QString());
        calibrateButton->setText(QApplication::translate("PCLViewer", "Calibrate", 0));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H

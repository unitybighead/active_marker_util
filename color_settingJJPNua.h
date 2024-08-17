/********************************************************************************
** Form generated from reading UI file 'color_settingJJPNua.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef COLOR_SETTINGJJPNUA_H
#define COLOR_SETTINGJJPNUA_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow {
 public:
  QWidget *centralwidget;
  QWidget *verticalLayoutWidget;
  QVBoxLayout *verticalLayout_2;
  QGridLayout *gridLayout;
  QLabel *robot_number_label;
  QSpacerItem *horizontalSpacer;
  QSlider *b_slider;
  QSpinBox *g_spinBox;
  QSlider *r_slider;
  QSpinBox *r_spinBox;
  QLabel *b_label;
  QSpinBox *robot_number_spinBox;
  QVBoxLayout *verticalLayout_3;
  QRadioButton *pink_radio;
  QRadioButton *green_radio;
  QRadioButton *blue_radio;
  QRadioButton *yellow_radio;
  QLabel *g_label;
  QLabel *r_label;
  QSlider *g_slider;
  QSpinBox *b_spinBox;
  QLabel *color_label;
  QPushButton *set_button;

  void setupUi(QMainWindow *MainWindow) {
    if (MainWindow->objectName().isEmpty())
      MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
    MainWindow->resize(850, 425);
    centralwidget = new QWidget(MainWindow);
    centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
    verticalLayoutWidget = new QWidget(centralwidget);
    verticalLayoutWidget->setObjectName(
        QString::fromUtf8("verticalLayoutWidget"));
    verticalLayoutWidget->setGeometry(QRect(10, 10, 831, 395));
    verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget);
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    verticalLayout_2->setContentsMargins(0, 0, 0, 0);
    gridLayout = new QGridLayout();
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    robot_number_label = new QLabel(verticalLayoutWidget);
    robot_number_label->setObjectName(QString::fromUtf8("robot_number_label"));
    robot_number_label->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(robot_number_label, 0, 0, 1, 1);

    horizontalSpacer =
        new QSpacerItem(10, 40, QSizePolicy::Expanding, QSizePolicy::Minimum);

    gridLayout->addItem(horizontalSpacer, 2, 0, 1, 1);

    b_slider = new QSlider(verticalLayoutWidget);
    b_slider->setObjectName(QString::fromUtf8("b_slider"));
    b_slider->setMaximum(255);
    b_slider->setOrientation(Qt::Horizontal);
    b_slider->setInvertedAppearance(false);
    b_slider->setInvertedControls(false);

    gridLayout->addWidget(b_slider, 5, 2, 1, 1);

    g_spinBox = new QSpinBox(verticalLayoutWidget);
    g_spinBox->setObjectName(QString::fromUtf8("g_spinBox"));
    g_spinBox->setMaximum(255);
    g_spinBox->setValue(0);

    gridLayout->addWidget(g_spinBox, 4, 1, 1, 1);

    r_slider = new QSlider(verticalLayoutWidget);
    r_slider->setObjectName(QString::fromUtf8("r_slider"));
    r_slider->setCursor(QCursor(Qt::ArrowCursor));
    r_slider->setMaximum(255);
    r_slider->setOrientation(Qt::Horizontal);

    gridLayout->addWidget(r_slider, 3, 2, 1, 1);

    r_spinBox = new QSpinBox(verticalLayoutWidget);
    r_spinBox->setObjectName(QString::fromUtf8("r_spinBox"));
    r_spinBox->setMaximum(255);
    r_spinBox->setValue(0);

    gridLayout->addWidget(r_spinBox, 3, 1, 1, 1);

    b_label = new QLabel(verticalLayoutWidget);
    b_label->setObjectName(QString::fromUtf8("b_label"));
    b_label->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(b_label, 5, 0, 1, 1);

    robot_number_spinBox = new QSpinBox(verticalLayoutWidget);
    robot_number_spinBox->setObjectName(
        QString::fromUtf8("robot_number_spinBox"));
    robot_number_spinBox->setMaximum(15);
    robot_number_spinBox->setMinimum(0);

    gridLayout->addWidget(robot_number_spinBox, 0, 2, 1, 1);

    verticalLayout_3 = new QVBoxLayout();
    verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
    pink_radio = new QRadioButton(verticalLayoutWidget);
    pink_radio->setObjectName(QString::fromUtf8("pink_radio"));

    verticalLayout_3->addWidget(pink_radio);

    green_radio = new QRadioButton(verticalLayoutWidget);
    green_radio->setObjectName(QString::fromUtf8("green_radio"));

    verticalLayout_3->addWidget(green_radio);

    blue_radio = new QRadioButton(verticalLayoutWidget);
    blue_radio->setObjectName(QString::fromUtf8("blue_radio"));

    verticalLayout_3->addWidget(blue_radio);

    yellow_radio = new QRadioButton(verticalLayoutWidget);
    yellow_radio->setObjectName(QString::fromUtf8("yellow_radio"));

    verticalLayout_3->addWidget(yellow_radio);

    gridLayout->addLayout(verticalLayout_3, 1, 2, 1, 1);

    g_label = new QLabel(verticalLayoutWidget);
    g_label->setObjectName(QString::fromUtf8("g_label"));
    g_label->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(g_label, 4, 0, 1, 1);

    r_label = new QLabel(verticalLayoutWidget);
    r_label->setObjectName(QString::fromUtf8("r_label"));
    r_label->setMouseTracking(false);
    r_label->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(r_label, 3, 0, 1, 1);

    g_slider = new QSlider(verticalLayoutWidget);
    g_slider->setObjectName(QString::fromUtf8("g_slider"));
    g_slider->setMaximum(255);
    g_slider->setOrientation(Qt::Horizontal);

    gridLayout->addWidget(g_slider, 4, 2, 1, 1);

    b_spinBox = new QSpinBox(verticalLayoutWidget);
    b_spinBox->setObjectName(QString::fromUtf8("b_spinBox"));
    b_spinBox->setMaximum(255);
    b_spinBox->setValue(0);

    gridLayout->addWidget(b_spinBox, 5, 1, 1, 1);

    color_label = new QLabel(verticalLayoutWidget);
    color_label->setObjectName(QString::fromUtf8("colro_label"));
    color_label->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(color_label, 1, 0, 1, 1);

    verticalLayout_2->addLayout(gridLayout);

    set_button = new QPushButton(verticalLayoutWidget);
    set_button->setObjectName(QString::fromUtf8("set_button"));
    set_button->setAutoDefault(false);
    set_button->setFlat(false);

    verticalLayout_2->addWidget(set_button);

    MainWindow->setCentralWidget(centralwidget);
    QWidget::setTabOrder(r_spinBox, g_spinBox);
    QWidget::setTabOrder(g_spinBox, b_spinBox);
    QWidget::setTabOrder(b_spinBox, r_slider);
    QWidget::setTabOrder(r_slider, g_slider);
    QWidget::setTabOrder(g_slider, b_slider);

    retranslateUi(MainWindow);

    set_button->setDefault(false);

    QMetaObject::connectSlotsByName(MainWindow);
  }  // setupUi

  void retranslateUi(QMainWindow *MainWindow) {
    MainWindow->setWindowTitle(QCoreApplication::translate(
        "MainWindow", "color setting manager", nullptr));
    robot_number_label->setText(
        QCoreApplication::translate("MainWindow", "Robot Number", nullptr));
    b_label->setText(QCoreApplication::translate("MainWindow", "B", nullptr));
    pink_radio->setText(
        QCoreApplication::translate("MainWindow", "Pink", nullptr));
    green_radio->setText(
        QCoreApplication::translate("MainWindow", "Green", nullptr));
    blue_radio->setText(
        QCoreApplication::translate("MainWindow", "Blue", nullptr));
    yellow_radio->setText(
        QCoreApplication::translate("MainWindow", "Yellow", nullptr));
    g_label->setText(QCoreApplication::translate("MainWindow", "G", nullptr));
    r_label->setText(QCoreApplication::translate("MainWindow", "R", nullptr));
    color_label->setText(
        QCoreApplication::translate("MainWindow", "Color", nullptr));
    set_button->setText(
        QCoreApplication::translate("MainWindow", "set", nullptr));
  }  // retranslateUi
};

namespace Ui {
class MainWindow : public Ui_MainWindow {};
}  // namespace Ui

QT_END_NAMESPACE

#endif  // COLOR_SETTINGJJPNUA_H

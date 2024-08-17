#include "color_setting_gui.hpp"

ColorSettingWindow::ColorSettingWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  setupConnections();
}

int ColorSettingWindow::getRobotID() const {
  return ui->robot_number_spinBox->value();
}

void ColorSettingWindow::setupConnections() {
  // スライダーの値が変更されたときにスピンボックスの値も変更
  connect(ui->r_slider, &QSlider::valueChanged, [this](int value) {
    if (ui->r_spinBox->value() != value) {
      ui->r_spinBox->setValue(value);
    }
  });
  connect(ui->g_slider, &QSlider::valueChanged, [this](int value) {
    if (ui->g_spinBox->value() != value) {
      ui->g_spinBox->setValue(value);
    }
  });
  connect(ui->b_slider, &QSlider::valueChanged, [this](int value) {
    if (ui->b_spinBox->value() != value) {
      ui->b_spinBox->setValue(value);
    }
  });

  // スピンボックスの値が変更されたときにスライダーの値も変更
  connect(ui->r_spinBox, QOverload<int>::of(&QSpinBox::valueChanged),
          [this](int value) {
            if (ui->r_slider->value() != value) {
              ui->r_slider->setValue(value);
            }
          });

  connect(ui->g_spinBox, QOverload<int>::of(&QSpinBox::valueChanged),
          [this](int value) {
            if (ui->g_slider->value() != value) {
              ui->g_slider->setValue(value);
            }
          });

  connect(ui->b_spinBox, QOverload<int>::of(&QSpinBox::valueChanged),
          [this](int value) {
            if (ui->b_slider->value() != value) {
              ui->b_slider->setValue(value);
            }
          });

  // ラジオボックスの選択が変わったら、記録されている値に変更する
  connect(ui->pink_radio, &QRadioButton::toggled, [this]() {
    if (ui->pink_radio->isChecked()) {
      ui->r_slider->setValue(pinkRGB_.r);
      ui->g_slider->setValue(pinkRGB_.g);
      ui->b_slider->setValue(pinkRGB_.b);
    }
  });
  connect(ui->green_radio, &QRadioButton::toggled, [this]() {
    if (ui->green_radio->isChecked()) {
      ui->r_slider->setValue(greenRGB_.r);
      ui->g_slider->setValue(greenRGB_.g);
      ui->b_slider->setValue(greenRGB_.b);
    }
  });
  connect(ui->blue_radio, &QRadioButton::toggled, [this]() {
    if (ui->blue_radio->isChecked()) {
      ui->r_slider->setValue(blueRGB_.r);
      ui->g_slider->setValue(blueRGB_.g);
      ui->b_slider->setValue(blueRGB_.b);
    }
  });
  connect(ui->yellow_radio, &QRadioButton::toggled, [this]() {
    if (ui->yellow_radio->isChecked()) {
      ui->r_slider->setValue(yellowRGB_.r);
      ui->g_slider->setValue(yellowRGB_.g);
      ui->b_slider->setValue(yellowRGB_.b);
    }
  });

  // ボタンがクリックされたときに各種値を取得し、コマンドラインに送信
  connect(ui->set_button, &QPushButton::clicked, [&]() {
    robotID_ = getRobotID();
    RGB comm = {0, 0, 0};
    std::string color = "dammy", command;

    if (ui->pink_radio->isChecked()) {
      pinkRGB_.r = ui->r_slider->value();
      pinkRGB_.g = ui->g_slider->value();
      pinkRGB_.b = ui->b_slider->value();
      color = "pink";
      comm = {pinkRGB_.r, pinkRGB_.g, pinkRGB_.b};
    } else if (ui->green_radio->isChecked()) {
      greenRGB_.r = ui->r_slider->value();
      greenRGB_.g = ui->g_slider->value();
      greenRGB_.b = ui->b_slider->value();
      color = "green";
      comm = {greenRGB_.r, greenRGB_.g, greenRGB_.b};
    } else if (ui->blue_radio->isChecked()) {
      blueRGB_.r = ui->r_slider->value();
      blueRGB_.g = ui->g_slider->value();
      blueRGB_.b = ui->b_slider->value();
      color = "blue";
      comm = {blueRGB_.r, blueRGB_.g, blueRGB_.b};
    } else if (ui->yellow_radio->isChecked()) {
      yellowRGB_.r = ui->r_slider->value();
      yellowRGB_.g = ui->g_slider->value();
      yellowRGB_.b = ui->b_slider->value();
      color = "yellow";
      comm = {yellowRGB_.r, yellowRGB_.g, yellowRGB_.b};
    }

    command = "ros2 topic pub -1 /am" + std::to_string(robotID_) + "/" + color +
              " std_msgs/msg/ColorRGBA \"{r: " + std::to_string(comm.r) +
              ",g: " + std::to_string(comm.g) +
              ",b: " + std::to_string(comm.b) + "}\"";
    system(command.c_str());
  });
}
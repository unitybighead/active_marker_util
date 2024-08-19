#ifndef COLOR_SETTING_GUI_HPP
#define COLOR_SETTING_GUI_HPP

#include <QMainWindow>
#include <cstdlib>
#include <iostream>
#include <string>

#include "color_settingJJPNua.h"  // 自動生成されたヘッダファイル

typedef struct {
  int r;
  int g;
  int b;
} RGB;

class ColorSettingWindow : public QMainWindow {
  Q_OBJECT

 public:
  ColorSettingWindow(QWidget *parent = nullptr);

  int getRobotID() const;

  Ui::MainWindow *ui;

 private:
  void setupConnections();
  int robotID_;
  RGB pinkRGB_ = {255, 0, 50};
  RGB greenRGB_ = {0, 255, 0};
  RGB blueRGB_ = {0, 0, 255};
  RGB yellowRGB_ = {255, 255, 0};
};

#endif  // COLOR_SETTING_GUI_HPP

#include <QApplication>
#include <QMessageBox>

#include "color_setting_gui.hpp"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  ColorSettingWindow mainWindow;
  mainWindow.show();
  return app.exec();
}

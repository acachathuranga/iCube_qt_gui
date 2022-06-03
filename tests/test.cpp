#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <string>

// QT Classes
#include <QApplication>
#include <QTimer>
#include <QtMultimedia/QMediaPlayer>
#include "icube_gui/ui/main_window.h"
#include "ui_splash_screen.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "icube_gui");
  QApplication app(argc, argv);

  std::string path = ros::package::getPath("icube_gui");

  MainWindow window;
  window.setWindowTitle("iCube Main Window");
  window.setWindowFlag(Qt::WindowStaysOnTopHint);
  window.setStyleSheet("background-color:white;");

  window.show();
  app.exec();

  return 0;
}

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

  // Splash Screen
  QWidget *splash_screen = new QWidget;
  Ui::SplashScreen ui_splash_screen;
  ui_splash_screen.setupUi(splash_screen);

  splash_screen->setWindowTitle("iCube");
  ui_splash_screen.sutd_label->setPixmap(QPixmap(":/images/images/sutd_logo.png"));
  ui_splash_screen.singhealth_label->setPixmap(QPixmap(":/images/images/singhealth_logo.png"));

  QMediaPlayer player;
  // ...
  player.setMedia(QUrl::fromLocalFile(QString().fromStdString(path) + "/resources/audio/startup_tone.mp3"));
  player.setVolume(50);
  //player.play(); //TODO Uncomment audio
  splash_screen->show();



  MainWindow window;
  window.setWindowTitle("iCube Main Window");
  window.setWindowFlag(Qt::WindowStaysOnTopHint);
  window.setStyleSheet("background-color:white;");

  // Splash Screen wait
  QTimer::singleShot(4000, splash_screen,SLOT(close())); // Timer
  QTimer::singleShot(4000,&window,SLOT(show()));

  app.exec();

  return 0;
}

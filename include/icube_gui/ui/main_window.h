#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QAction>
#include <QToolBar>
#include <QMenuBar>
#include <QTimer>
#include <QThread>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QStackedWidget>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "icube_gui/Tools/console.h"
#include "icube_gui/collision_indicator.h"
#include "icube_gui/ui/settings_screen.h"

namespace Ui {
class MainWindow;
class RosThread;
}

class RosThread : public QThread
{
    Q_OBJECT
    void run() override;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void button_clicked();
    void settings_screen_timeout();

private:
    Ui::MainWindow *ui;
    ros::NodeHandle nh_;
    RosThread ros_thread_;
    RobotCommunication *robot_com_;

    QDockWidget *bottom_dock_widget_;
    Console *console_;
    QTextEdit console_text_edit_;

    QStackedWidget *central_widget_;
    EmojiFace *collision_indicator;
    SettingsScreen *settings_screen_;
};

#endif // MAIN_WINDOW_H

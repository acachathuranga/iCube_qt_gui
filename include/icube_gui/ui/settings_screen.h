#ifndef SETTINGS_SCREEN_H
#define SETTINGS_SCREEN_H

#include <QWidget>
#include <QTimer>
#include <QMessageBox>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <robot_docker/GripperActionGoal.h>
#include <robot_docker/GripperActionGoal.h>
#include "icube_gui/Tools/robotCommunication.h"
#include "icube_gui/Tools/console.h"
#include "icube_gui/global_defs.h"

namespace Ui {
class SettingsScreen;
}

class SettingsScreen : public QWidget
{
    Q_OBJECT

public:
    int screen_timeout_milliseconds = 5000;

    SettingsScreen(ros::NodeHandle *nh, RobotCommunication *com, Console *console, QWidget *parent = nullptr);
    ~SettingsScreen();

signals:
    /**
     * @brief This signal will be emitted when a screen timeout occurs
     */
    void screen_timeout(void);

protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private slots:

    void on_pin_pushButton_clicked();

    void on_return_pushButton_clicked();

    void on_release_pushButton_clicked();

    void on_clamp_pushButton_clicked();

    void on_extended_release_pushButton_clicked();

    void on_extended_clamp_pushButton_clicked();

    void on_cancel_mission_pushButton_clicked();

    void on_self_test_pushButton_clicked();

    void on_shutdown_pushButton_clicked();

    void on_exit_ui_pushButton_clicked();

    void on_safety_disable_pushButton_clicked();

    void on_robot_initialize_pushButton_clicked();

private:
    Ui::SettingsScreen *ui;
    ros::NodeHandle *nh_;
    RobotCommunication *robot_com_;
    Console *console_;

    ros::Publisher robot_supervisory_command_publisher_;
    ros::Publisher robot_gripper_action_publisher_;

    bool pinned_ = false;
    QTimer *screen_timer_;

    std::string robot_command_topic_;
    std::string robot_command_field_;
    std::string robot_supervisor_command_topic_;
    std::string robot_gripper_action_;

    const std::string robot_command_topic_param_ = K_ROBOT_COMMAND_TOPIC_PARAM;
    const std::string robot_command_field_param_ = K_ROBOT_COMMAND_FIELD_PARAM;
    const std::string robot_supervisor_command_topic_param_ = K_ROBOT_SUPERVISOR_TOPIC_PARAM;
    const std::string robot_gripper_action_topic_param_ = K_ROBOT_GRIPPER_ACTION_PARAM;

    const std::string GRIPPER_CLAMP_COMMAND = K_GRIPPER_CLAMP_COMMAND;
    const std::string GRIPPER_RELEASE_COMMAND = K_GRIPPER_RELEASE_COMMAND;
    const std::string GRIPPER_EXTENDED_CLAMP_COMMAND = K_GRIPPER_EXTENDED_CLAMP_COMMAND;
    const std::string GRIPPER_EXTENDED_RELEASE_COMMAND = K_GRIPPER_EXTENDED_RELEASE_COMMAND;
    const std::string ROBOT_SELF_TEST_COMMAND = K_ROBOT_SELF_TEST_COMMAND;
    const std::string ROBOT_INITIALIZE_COMMAND = K_ROBOT_INITIALIZE_COMMAND;
    const std::string MISSION_CANCEL_COMMAND = K_MISSION_CANCEL_COMMAND;
    const std::string DISABLE_MOTOR_FILTER_COMMAND = K_DISABLE_MOTOR_FILTER_COMMAND;
    const std::string ROBOT_PARK_COMMAND = K_ROBOT_PARK_COMMAND;



    void on_screen_timeout(void);
    void on_robot_command(std::string msg);
};

#endif // SETTINGS_SCREEN_H

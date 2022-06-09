#include "icube_gui/ui/settings_screen.h"
#include "ui_settings_screen.h"

SettingsScreen::SettingsScreen(ros::NodeHandle *nh, RobotCommunication *com, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingsScreen),
    nh_(nh),
    robot_com_(com)
{
    ui->setupUi(this);

    /// Screen timeout handler
    screen_timer_ = new QTimer(this);
    connect(screen_timer_, &QTimer::timeout, this, &SettingsScreen::on_screen_timeout);

    ros::param::param<std::string>(robot_command_topic_param_, robot_command_topic_, "robot_depart");
    ros::param::param<std::string>(robot_command_field_param_, robot_command_field_, "command");
}

SettingsScreen::~SettingsScreen()
{
    delete ui;
}

void SettingsScreen::showEvent(QShowEvent *event)
{
    screen_timer_->start(screen_timeout_milliseconds);
    QWidget::showEvent(event);
}

void SettingsScreen::hideEvent(QHideEvent *event)
{
    QWidget::hideEvent(event);
}

void SettingsScreen::mousePressEvent(QMouseEvent *event)
{
    std::cout << "resetted" << std::endl;
    /// Reset screen_timeout timer upon mouse event
    screen_timer_->stop();
    screen_timer_->start();
    QWidget::mousePressEvent(event);
}

void SettingsScreen::on_screen_timeout(void)
{
    if (!pinned_)
    {
        screen_timer_->stop();
        emit screen_timeout();
    }
}

void SettingsScreen::on_pin_pushButton_clicked()
{
    if (pinned_)
    {
        /// Unpin Page
        ui->pin_pushButton->setText("Pin Page");
        ui->pin_pushButton->setStyleSheet("");
        screen_timer_->start(screen_timeout_milliseconds);
        pinned_ = false;
    }
    else
    {
        /// Pin Page
        ui->pin_pushButton->setText("Release Page");
        ui->pin_pushButton->setStyleSheet("background: rgb(85, 87, 83)");
        screen_timer_->stop();
        pinned_ = true;
    }
}

void SettingsScreen::on_return_pushButton_clicked()
{
    /// Return to previous screen
    on_screen_timeout();
}

void SettingsScreen::on_release_pushButton_clicked()
{
    robot_com_->publish(robot_command_topic_, robot_command_field_, GRIPPER_RELEASE_COMMAND);
}

void SettingsScreen::on_clamp_pushButton_clicked()
{
    robot_com_->publish(robot_command_topic_, robot_command_field_, GRIPPER_CLAMP_COMMAND);
}

void SettingsScreen::on_extended_release_pushButton_clicked()
{
    robot_com_->publish(robot_command_topic_, robot_command_field_, GRIPPER_EXTENDED_RELEASE_COMMAND);
}

void SettingsScreen::on_extended_clamp_pushButton_clicked()
{
    robot_com_->publish(robot_command_topic_, robot_command_field_, GRIPPER_EXTENDED_CLAMP_COMMAND);
}

void SettingsScreen::on_cancel_mission_pushButton_clicked()
{
    robot_com_->publish(robot_command_topic_, robot_command_field_, MISSION_CANCEL_COMMAND);
}

void SettingsScreen::on_self_test_pushButton_clicked()
{
    robot_com_->publish(robot_command_topic_, robot_command_field_, ROBOT_SELF_TEST_COMMAND);
}

void SettingsScreen::on_shutdown_pushButton_clicked()
{
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Cofirm Robot Shutdown Action");
    msgBox.setText("Proceed with shutting down the robot? Please note that you will have to re-initialize robot upon restart!");
    msgBox.setStandardButtons(QMessageBox::Yes);
    msgBox.addButton(QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    if(msgBox.exec() == QMessageBox::Yes){
      system("echo NUC717 | sudo -S shutdown now");
    }
}

void SettingsScreen::on_exit_ui_pushButton_clicked()
{
    QApplication::exit();
}

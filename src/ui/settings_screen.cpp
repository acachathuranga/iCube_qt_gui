#include "icube_gui/ui/settings_screen.h"
#include "ui_settings_screen.h"

SettingsScreen::SettingsScreen(ros::NodeHandle *nh, RobotCommunication *com, Console *console, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SettingsScreen),
    nh_(nh),
    robot_com_(com),
    console_(console)
{
    ui->setupUi(this);

    /// Screen timeout handler
    screen_timer_ = new QTimer(this);
    connect(screen_timer_, &QTimer::timeout, this, &SettingsScreen::on_screen_timeout);

    ros::param::param<std::string>(robot_command_topic_param_, robot_command_topic_, "robot_depart");
    ros::param::param<std::string>(robot_command_field_param_, robot_command_field_, "command");
    ros::param::param<std::string>(robot_supervisor_command_topic_param_, robot_supervisor_command_topic_, "/supervisory_command");
    ros::param::param<std::string>(robot_gripper_action_topic_param_, robot_gripper_action_, "/gripper_control/goal");

    robot_supervisory_command_publisher_ = nh_->advertise<std_msgs::String>(robot_supervisor_command_topic_, 100);
    robot_gripper_action_publisher_ = nh->advertise<robot_docker::GripperActionGoal>(robot_gripper_action_, 100);

    robot_com_->subscribe(robot_command_topic_, boost::bind(&SettingsScreen::on_robot_command, this, _1));
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
    robot_docker::GripperActionGoal goal;
    goal.goal.command = goal.goal.RELEASE;
    robot_gripper_action_publisher_.publish(goal);
    //robot_com_->publish(robot_command_topic_, robot_command_field_, GRIPPER_RELEASE_COMMAND);
}

void SettingsScreen::on_clamp_pushButton_clicked()
{
    robot_docker::GripperActionGoal goal;
    goal.goal.command = goal.goal.CLAMP;
    robot_gripper_action_publisher_.publish(goal);
    //robot_com_->publish(robot_command_topic_, robot_command_field_, GRIPPER_CLAMP_COMMAND);
}

void SettingsScreen::on_extended_release_pushButton_clicked()
{
    robot_docker::GripperActionGoal goal;
    goal.goal.command = goal.goal.EXTENDED_RELEASE;
    robot_gripper_action_publisher_.publish(goal);
    //robot_com_->publish(robot_command_topic_, robot_command_field_, GRIPPER_EXTENDED_RELEASE_COMMAND);
}

void SettingsScreen::on_extended_clamp_pushButton_clicked()
{
    robot_docker::GripperActionGoal goal;
    goal.goal.command = goal.goal.EXTENDED_CLAMP;
    robot_gripper_action_publisher_.publish(goal);
    //robot_com_->publish(robot_command_topic_, robot_command_field_, GRIPPER_EXTENDED_CLAMP_COMMAND);
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
      system("echo password | sudo -S shutdown now");
    }
}

void SettingsScreen::on_exit_ui_pushButton_clicked()
{
    QApplication::exit();
}

void SettingsScreen::on_safety_disable_pushButton_clicked()
{
    std_msgs::String msg;
    msg.data = DISABLE_MOTOR_FILTER_COMMAND;
    robot_supervisory_command_publisher_.publish(msg);
}

void SettingsScreen::on_robot_initialize_pushButton_clicked()
{
    std_msgs::String msg;
    msg.data = ROBOT_INITIALIZE_COMMAND;
    robot_supervisory_command_publisher_.publish(msg);
}


void SettingsScreen::on_robot_command(std::string msg)
{
    Json::Value json_msg;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse( msg, json_msg );     //parse process
    if ( !parsingSuccessful )
    {
        console_->print("Failed to parse msg ['" + msg + "'] : " + reader.getFormattedErrorMessages());
    }

    if (json_msg.get(robot_command_field_, "").asString() == ROBOT_PARK_COMMAND)
    {
        on_extended_clamp_pushButton_clicked();
        
    }
}

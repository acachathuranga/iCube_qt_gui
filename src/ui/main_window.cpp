#include "icube_gui/ui/main_window.h"
#include "ui_main_window.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    /// Start ROS Spinner thread
    ros_thread_.start();

    /// Mqtt Adaptor
    //robot_com_ = new RobotCommunication()

    /// System Console Widget
    bottom_dock_widget_ = new QDockWidget(tr("Sys Console"), this);
    bottom_dock_widget_->setWidget(&console_text_edit_);
    this->addDockWidget(Qt::BottomDockWidgetArea, bottom_dock_widget_);

    console_ = new Console(&console_text_edit_);

    /// Central Widget
    central_widget_ = new QStackedWidget;
    this->setCentralWidget(central_widget_);

    collision_indicator = new CollisionIndicator(&nh_, NULL, console_);
    settings_screen_ = new SettingsScreen;

    central_widget_->addWidget(collision_indicator);
    central_widget_->addWidget(settings_screen_);


    connect(collision_indicator, &CollisionIndicator::pressed, this, &MainWindow::button_clicked);

    chatter_pub = nh_.advertise<std_msgs::String>("chatter", 1000);
    console_->print("Starting Program");

    //button->setVisible(true);
    //ui->stackedWidget->setCurrentIndex(2);
//    std::cout << ui->stackedWidget->currentIndex() << std::endl;
//    std::cout << emoji_widget->isVisible() << std::endl;
//    //ui->stackedWidget->setCurrentIndex(1);
//    std::cout << emoji_widget->isVisible() << std::endl;
//    //ui->stackedWidget->setCurrentIndex(0);
//    std::cout << emoji_widget->isVisible() << std::endl;
}

MainWindow::~MainWindow()
{
    ros::shutdown();
    ros_thread_.wait();
    delete ui;
}

void MainWindow::button_clicked()
{
    std_msgs::String msg;
    msg.data = "hello_world: " + std::to_string(count);

    ROS_INFO_STREAM(msg.data.c_str());
    console_->print(msg.data);
    std::cout << std::endl;
    chatter_pub.publish(msg);

    //ui->pushButton->move(count*10, 790);

    central_widget_->setCurrentIndex(1);

    ++count;

}

void RosThread::run()
{
    ros::Rate rate(10);

    while(ros::ok())
    {
       ros::spinOnce();
       rate.sleep();
    }
}



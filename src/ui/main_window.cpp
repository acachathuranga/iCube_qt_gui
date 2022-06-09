#include "icube_gui/ui/main_window.h"
#include "ui_main_window.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    /// Open MainWindow Full Screen
    this->setWindowState(Qt::WindowFullScreen);

    /// Start ROS Spinner thread
    ros_thread_.start();

    /// Menubar
    QAction *minimize_window = new QAction("Minimize");
    QAction *fullscreen_window = new QAction("Full Screen");
    connect(minimize_window, &QAction::triggered, this, &MainWindow::showMinimized);
    connect(fullscreen_window, &QAction::triggered, this, &MainWindow::showFullScreen);
    QMenu *actions_menu = this->menuBar()->addMenu(tr("&Actions"));
    actions_menu->addAction(minimize_window);
    actions_menu->addAction(fullscreen_window);

    /// System Console Widget
    bottom_dock_widget_ = new QDockWidget(tr("Sys Console"), this);
    bottom_dock_widget_->setWidget(&console_text_edit_);
    this->addDockWidget(Qt::BottomDockWidgetArea, bottom_dock_widget_);

    console_ = new Console(&console_text_edit_);

    /// Mqtt Adaptor
    robot_com_ = new RobotCommunication(console_);

    /// Central Widget
    central_widget_ = new QStackedWidget;
    this->setCentralWidget(central_widget_);

    collision_indicator = new CollisionIndicator(&nh_, robot_com_, console_);
    settings_screen_ = new SettingsScreen(&nh_, robot_com_);

    central_widget_->addWidget(collision_indicator);
    central_widget_->addWidget(settings_screen_);


    connect(collision_indicator, &CollisionIndicator::pressed, this, &MainWindow::button_clicked);
    connect(settings_screen_, &SettingsScreen::screen_timeout, this, &MainWindow::settings_screen_timeout);

    console_->print("Starting Program");
}

MainWindow::~MainWindow()
{
    ros::shutdown();
    ros_thread_.wait();
    delete ui;
}

void MainWindow::button_clicked()
{
    central_widget_->setCurrentIndex(1);
}

void MainWindow::settings_screen_timeout()
{
    central_widget_->setCurrentIndex(0);
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



#include "icube_gui/collision_indicator.h"

CollisionIndicator::CollisionIndicator(ros::NodeHandle *nh, RobotCommunication *com, Console *console):
    nh_(nh),
    robot_com_(com),
    console_(console)
{
    std::string robot_status_topic;
    ros::param::param<std::string>(robot_status_topic_param_, robot_status_topic, "status");
    ros::param::param<std::string>(robot_status_field_param_, robot_status_field_, "robot_status");

    /// Register Robot status callback
    robot_com_->subscribe(robot_status_topic, boost::bind(&CollisionIndicator::robot_status_cb, this, _1));

    std::string obstacle_avoidance_status_topic;
    ros::param::param<std::string>(colision_avoidance_status_topic_param_, obstacle_avoidance_status_topic,
                                   "/collision_avoidance_guard/obstacle_detection");
    collision_avoidance_state_sub_ = nh_->subscribe(obstacle_avoidance_status_topic,
                                                                   1000, &CollisionIndicator::collision_avoidance_status_cb, this);
}

void CollisionIndicator::set_collision_indication(EmojiFace::Mood mood)
{
    if (mood != currentEmoji)
    {
        setMood(mood);
        currentEmoji = mood;
    }
}

void CollisionIndicator::collision_avoidance_status_cb(std_msgs::String status)
{
    if (robot_disabled_)
    {
        /// If robot is disabled, do not display collision indication
        return;
    }
    if (status.data == OBSTACLE_CLEAR)
    {
        set_collision_indication(EmojiFace::Mood::HAPPY);
    }
    else if (status.data == OBSTACLE_NEAR_COLLISION)
    {
        set_collision_indication(EmojiFace::Mood::NERVOUS);
    }
    else if (status.data == OBSTACLE_IMMINENT_COLLISION)
    {
        set_collision_indication(EmojiFace::Mood::SAD);
    }
    else if (status.data == SAFETY_OFF)
    {
        set_collision_indication(EmojiFace::Mood::ANGRY);
    }
    else
    {
        set_collision_indication(EmojiFace::Mood::DEAD);
    }
}

void CollisionIndicator::robot_status_cb(std::string msg)
{
    Json::Value json_msg;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse( msg, json_msg );     //parse process
    if ( !parsingSuccessful )
    {
        console_->print("Failed to parse msg ['" + msg + "'] : " + reader.getFormattedErrorMessages());
    }

    if (json_msg.get(robot_status_field_, "").asString() == ROBOT_DISABLED_STATUS)
    {
        set_collision_indication(EmojiFace::Mood::SLEEPY);
        robot_disabled_ = true;
    }
    else
    {
        robot_disabled_ = false;
    }
}



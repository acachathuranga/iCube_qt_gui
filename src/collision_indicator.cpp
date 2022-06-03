#include "icube_gui/collision_indicator.h"

CollisionIndicator::CollisionIndicator(ros::NodeHandle *nh, RobotCommunication *com, Console *console):
    nh_(nh),
    robot_com_(com),
    console_(console)
{

    std::string obstacle_avoidance_status_topic;
    ros::param::param<std::string>("colision_avoidance_status_topic", obstacle_avoidance_status_topic,
                                   "/collision_avoidance_guard/obstacle_detection");
    std::cout << obstacle_avoidance_status_topic << std::endl;
    collision_avoidance_state_sub_ = nh_->subscribe(obstacle_avoidance_status_topic,
                                                                   1000, &CollisionIndicator::collision_avoidance_status_cb, this);
}

void CollisionIndicator::setCollisionIndication(EmojiFace::Mood mood)
{
    if (mood != currentEmoji)
    {
        setMood(mood);
        currentEmoji = mood;
    }
}

void CollisionIndicator::collision_avoidance_status_cb(std_msgs::String status)
{
    if (status.data == OBSTACLE_CLEAR)
    {
        setCollisionIndication(EmojiFace::Mood::HAPPY);
    }
    else if (status.data == OBSTACLE_NEAR_COLLISION)
    {
        setCollisionIndication(EmojiFace::Mood::NERVOUS);
    }
    else if (status.data == OBSTACLE_IMMINENT_COLLISION)
    {
        setCollisionIndication(EmojiFace::Mood::SAD);
    }
    else if (status.data == SAFETY_OFF)
    {
        setCollisionIndication(EmojiFace::Mood::ANGRY);
    }
    else
    {
        setCollisionIndication(EmojiFace::Mood::DEAD);
    }
}



#ifndef COLLISION_INDICATOR_H
#define COLLISION_INDICATOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "icube_gui/ui/emojiface.h"
#include "icube_gui/Tools/console.h"
#include "icube_gui/Tools/robotCommunication.h"

#include "icube_gui/global_defs.h"

class CollisionIndicator : public EmojiFace
{
public:
    CollisionIndicator(ros::NodeHandle *nh, RobotCommunication *com, Console *console);

private:
    ros::NodeHandle *nh_;
    RobotCommunication *robot_com_;
    Console *console_;
    EmojiFace emoji_face_;

    bool robot_disabled_ = false;

    ros::Subscriber collision_avoidance_state_sub_;
    EmojiFace::Mood currentEmoji;

    std::string robot_status_field_;

    const std::string colision_avoidance_status_topic_param_ = K_COLLISION_AVOIDANCE_TOPIC_PARAM;
    const std::string OBSTACLE_CLEAR = K_COLLISION_AVOIDANCE_OBSTACLE_CLEAR;
    const std::string OBSTACLE_NEAR_COLLISION = K_COLLISION_AVOIDANCE_NEAR_COLLISION;
    const std::string OBSTACLE_IMMINENT_COLLISION = K_COLLISION_AVOIDANCE_IMMINENT_COLLISION;
    const std::string SAFETY_OFF = K_COLLISION_AVOIDANCE_SAFETY_OFF;

    const std::string robot_status_topic_param_ = K_ROBOT_STATUS_TOPIC_PARAM;
    const std::string robot_status_field_param_ = K_ROBOT_STATUS_FIELD_PARAM;
    const std::string ROBOT_DISABLED_STATUS = K_ROBOT_DISABLED;


    void set_collision_indication(EmojiFace::Mood mood);
    void collision_avoidance_status_cb(std_msgs::String status);
    void robot_status_cb(std::string);
};

#endif // COLLISION_INDICATOR_H

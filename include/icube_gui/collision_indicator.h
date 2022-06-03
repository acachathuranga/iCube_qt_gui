#ifndef COLLISION_INDICATOR_H
#define COLLISION_INDICATOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "icube_gui/ui/emojiface.h"
#include "icube_gui/Tools/console.h"
#include "icube_gui/Tools/robotCommunication.h"

class CollisionIndicator : public EmojiFace
{
public:
    CollisionIndicator(ros::NodeHandle *nh, RobotCommunication *com, Console *console);

private:
    ros::NodeHandle *nh_;
    RobotCommunication *robot_com_;
    Console *console_;
    EmojiFace emoji_face_;

    ros::Subscriber collision_avoidance_state_sub_;
    EmojiFace::Mood currentEmoji;

    const QString colision_avoidance_status_topic_name_field = "colision_avoidance_status_topic";
    const std::string OBSTACLE_CLEAR = "clear";
    const std::string OBSTACLE_NEAR_COLLISION = "near_collision";
    const std::string OBSTACLE_IMMINENT_COLLISION = "imminent_collision";
    const std::string SAFETY_OFF = "safety_off";


    void setCollisionIndication(EmojiFace::Mood mood);
    void collision_avoidance_status_cb(std_msgs::String status);
};

#endif // COLLISION_INDICATOR_H

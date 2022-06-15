#ifndef GLOBAL_DEFS_H
#define GLOBAL_DEFS_H

#define K_BATTERY_STATUS_TOPIC_PARAM "battery_status_topic"
#define K_COLLISION_AVOIDANCE_TOPIC_PARAM "colision_avoidance_status_topic"
#define K_ROBOT_STATUS_TOPIC_PARAM "robot_status_topic"
#define K_ROBOT_STATUS_FIELD_PARAM "robot_status_field"
#define K_ROBOT_COMMAND_TOPIC_PARAM "robot_command_topic"
#define K_ROBOT_COMMAND_FIELD_PARAM "robot_command_field"
#define K_ROBOT_SUPERVISOR_TOPIC_PARAM "robot_supervisor_command_topic"
#define K_ROBOT_GRIPPER_ACTION_PARAM "robot_gripper_action"

#define K_COLLISION_AVOIDANCE_OBSTACLE_CLEAR "clear"
#define K_COLLISION_AVOIDANCE_NEAR_COLLISION "near_collision"
#define K_COLLISION_AVOIDANCE_IMMINENT_COLLISION "imminent_collision"
#define K_COLLISION_AVOIDANCE_SAFETY_OFF "safety_off"

#define K_GRIPPER_RELEASE_COMMAND "gripper_release"
#define K_GRIPPER_CLAMP_COMMAND "gripper_clamp"
#define K_GRIPPER_EXTENDED_RELEASE_COMMAND "gripper_extended_release"
#define K_GRIPPER_EXTENDED_CLAMP_COMMAND "gripper_extended_clamp"

#define K_MISSION_CANCEL_COMMAND "cancel_mission"

#define K_ROBOT_SELF_TEST_COMMAND "self_test"
#define K_ROBOT_INITIALIZE_COMMAND "initialize"

#define K_DISABLE_MOTOR_FILTER_COMMAND "disable_motor_filter"

#define K_ROBOT_DISABLED "disabled"


#endif // GLOBAL_DEFS_H

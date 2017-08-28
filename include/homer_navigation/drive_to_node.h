#ifndef DRIVE_TO_NODE_H_U58RL9V7
#define DRIVE_TO_NODE_H_U58RL9V7

// ROS
#include <ros/rate.h>
#include <ros/ros.h>

#include <homer_mapnav_msgs/TargetUnreachable.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

#include <homer_mapnav_msgs/DriveToAction.h>
#include <homer_mapnav_msgs/DetectObstacleAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <homer_robbie_architecture/Architecture/StateMachine/StateMachine.h>

// Messages published

// Other
namespace states
{
enum State
{
  IDLE,
  DRIVING_TO_GOAL_LOCATION,
  FINISHED
};
}
typedef states::State State;

typedef actionlib::SimpleActionServer<homer_mapnav_msgs::DriveToAction> Server;
typedef actionlib::SimpleActionClient<homer_mapnav_msgs::DetectObstacleAction> DetectObstacleActionClient;

class DriveTo
{
protected:
  Server m_as_;

  std::string m_action_name_;
  std::unique_ptr<DetectObstacleActionClient> m_obstacle_client_;

public:
  DriveTo(ros::NodeHandle n, std::string name);
  ~DriveTo();
  void init();

  void targetReachedCallback(const std_msgs::String::ConstPtr& p);
  void targetUnreachableCallback(const homer_mapnav_msgs::TargetUnreachable::ConstPtr& msg);

private:
  int check_obstacle_type(const geometry_msgs::PointStamped& point);
  // Member Variables
  ros::NodeHandle m_nh_;
  homer_mapnav_msgs::DriveToGoal::ConstPtr m_goal;

  void driveToCallback();
  void switchMapLayers(bool state);

  void publishFeedback(float progress, std::string feedback);

  void speakBlocked(std::string text);

  bool m_speaking;
  bool m_check_obstacle;

  StateMachine<State> m_statemachine;
  ros::Subscriber m_target_unreachable_sub_;
  ros::Subscriber m_target_reached_sub_;

  ros::Publisher m_map_layer_pub_;
  ros::Publisher m_navigate_to_poi_pub_;

  // Node Handle
  ros::NodeHandle nh;
};

#endif /* end of include guard: DRIVE_TO_NODE_H_U58RL9V7 */

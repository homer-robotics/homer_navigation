#ifndef DRIVE_TO_NODE_H_U58RL9V7
#define DRIVE_TO_NODE_H_U58RL9V7

// ROS
#include <ros/rate.h>
#include <ros/ros.h>

#include <homer_mapnav_msgs/TargetUnreachable.h>
#include <homer_mapnav_msgs/StartNavigation.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <homer_mapnav_msgs/DriveToAction.h>
#include <homer_mapnav_msgs/DetectObstacleAction.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <homer_robbie_architecture/Architecture/StateMachine/StateMachine.h>
#include <tf/transform_listener.h>

// Messages published

// Other
namespace states
{
	enum State
	{
		IDLE,
		DRIVING_TO_POI,
		DRIVING_TO_POINT
	};
}
typedef states::State State;

typedef actionlib::SimpleActionServer<homer_mapnav_msgs::DriveToAction> Server;
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseServer;
typedef actionlib::SimpleActionClient<homer_mapnav_msgs::DetectObstacleAction> DetectObstacleActionClient;

class DriveTo
{
	protected:
		Server m_as_;
		MoveBaseServer m_mbas_;

		std::unique_ptr<DetectObstacleActionClient> m_obstacle_client_;

	public:
		DriveTo(ros::NodeHandle n);
		~DriveTo();
		void init();

		void targetReachedCallback(const std_msgs::String::ConstPtr& p);
		void targetUnreachableCallback(const homer_mapnav_msgs::TargetUnreachable::ConstPtr& msg);

	private:
		int check_obstacle_type(const geometry_msgs::PointStamped& point);
		// Member Variables
		ros::NodeHandle m_nh_;
		homer_mapnav_msgs::DriveToGoal::ConstPtr m_goal;
		move_base_msgs::MoveBaseGoal::ConstPtr m_mbgoal;

		void driveToCallback();
		void moveBaseCallback();
		void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void switchMapLayers(bool state);

		void publishFeedback(float progress, std::string feedback);

		void speakBlocked(std::string text);

		void setTorsoPosition(float position);

		ros::Publisher m_set_torso_pub_;

		bool m_speaking;
		bool m_check_obstacle;

		StateMachine<State> m_statemachine;
		ros::Subscriber m_target_unreachable_sub_;
		ros::Subscriber m_target_reached_sub_;
		ros::Subscriber m_pose_sub_;

		ros::Publisher m_map_layer_pub_;
		ros::Publisher m_navigate_to_poi_pub_;
		ros::Publisher m_start_navigation_pub_;

		tf::TransformListener m_transform_listener;

		// Node Handle
		ros::NodeHandle nh;
};

#endif /* end of include guard: DRIVE_TO_NODE_H_U58RL9V7 */

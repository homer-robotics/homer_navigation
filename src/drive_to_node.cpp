#include <homer_navigation/drive_to_node.h>

#include <homer_mapnav_msgs/MapLayers.h>
#include <homer_mapnav_msgs/NavigateToPOI.h>
#include <actionlib/client/simple_action_client.h>
#include <homer_tts/SpeakAction.h>

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

typedef actionlib::SimpleActionClient<homer_tts::SpeakAction> SpeakActionClient;

namespace
{
	typedef homer_mapnav_msgs::DetectObstacleResult detect;
	typedef homer_mapnav_msgs::DriveToResult drive;
	std::map<int, int> detectToDriveTypes{
		{ detect::SMALL_OBSTACLE, drive::SMALL_OBSTACLE },
			{ detect::BIG_OBSTACLE, drive::BIG_OBSTACLE },
			{ detect::NO_OBSTACLE, drive::NO_OBSTACLE }
	};
}


void DriveTo::setTorsoPosition(float position)
{
	ROS_INFO_STREAM("Setting torso position to "<< position);

	float time_to_lift = 1.0;
	trajectory_msgs::JointTrajectory msg;
	std::vector<std::string> joint_names;
	joint_names.push_back("torso_lift_joint");
	msg.joint_names = joint_names;
	trajectory_msgs::JointTrajectoryPoint p;
	p.positions.push_back(position);
	p.time_from_start.sec = time_to_lift;
	msg.points.push_back(p);
	m_set_torso_pub_.publish(msg);
	//rospy.sleep(time_to_lift)

}


void DriveTo::publishFeedback(float progress, std::string feedback)
{
	homer_mapnav_msgs::DriveToFeedback drive_to_feedback;
	drive_to_feedback.progress = progress / homer_mapnav_msgs::DriveToFeedback::STEPS;
	drive_to_feedback.feedback = feedback;
	m_as_.publishFeedback(drive_to_feedback);
}

void DriveTo::targetUnreachableCallback(
		const homer_mapnav_msgs::TargetUnreachable::ConstPtr& msg)
{
	if (m_statemachine.state() == states::DRIVING_TO_GOAL_LOCATION)
	{
		homer_mapnav_msgs::DriveToResult result;
		result.result = homer_mapnav_msgs::DriveToResult::FAILED_TARGET_UNREACHABLE;
		if (!m_goal->suppress_speaking)
		{
			speakBlocked("I am not able to reach the target");
		}

		if (m_check_obstacle && (msg->reason == msg->LASER_OBSTACLE ||
					msg->reason == msg->NO_PATH_FOUND))
		{
			result.result = check_obstacle_type(msg->point);
		}

		m_as_.setAborted(result);
		m_statemachine.setState(states::IDLE);
	}
}

int DriveTo::check_obstacle_type(const geometry_msgs::PointStamped& point)
{
	ROS_INFO_STREAM("Checking for obstacles");
	homer_mapnav_msgs::DetectObstacleGoal obstacle_goal;
	obstacle_goal.location = point;
	m_obstacle_client_->waitForServer();
	m_obstacle_client_->sendGoal(obstacle_goal);
	m_obstacle_client_->waitForResult(ros::Duration(2.0));

	auto obstacle_result = m_obstacle_client_->getResult();
	ROS_INFO_STREAM("Obstacle type: " << obstacle_result->result);

	return detectToDriveTypes[obstacle_result->result];
}

void DriveTo::targetReachedCallback(const std_msgs::String::ConstPtr& p)
{
	(void)p;  // ignore unused input message
	if (m_statemachine.state() == states::DRIVING_TO_GOAL_LOCATION)
	{
		publishFeedback(homer_mapnav_msgs::DriveToFeedback::DRIVING_TO_GOAL_LOCATION,
				"Reached goal location");
		ROS_INFO_STREAM("Reached the goal location");

		m_statemachine.setState(states::FINISHED);

		if (!m_goal->suppress_speaking)
		{
			speakBlocked("I completed the drive to action");
		}

		homer_mapnav_msgs::DriveToResult result;
		result.result = homer_mapnav_msgs::DriveToResult::SUCCEEDED;
		m_as_.setSucceeded(result);
	}
}

void DriveTo::switchMapLayers(bool state)
{
	homer_mapnav_msgs::MapLayers msg;
	msg.state = state;
	for (int layer = homer_mapnav_msgs::MapLayers::MASKING_LAYER;
			layer <= homer_mapnav_msgs::MapLayers::RAPID_MAP; ++layer)
	{
		msg.layer = layer;
		m_map_layer_pub_.publish(msg);
		ros::Duration(0.1).sleep();
	}
}

void DriveTo::driveToCallback()
{
	m_goal = m_as_.acceptNewGoal();
	m_check_obstacle = m_goal->check_obstacle;
	if (m_goal->plan_only_on_slam_map)
	{
		ROS_INFO_STREAM("Disabling map layers "
				<< homer_mapnav_msgs::MapLayers::MASKING_LAYER << " to "
				<< homer_mapnav_msgs::MapLayers::RAPID_MAP
				<< " for path planning");
		switchMapLayers(false);
	}
	/*m_lisa.driveToPOI(m_goal->goal_location, m_goal->distance_to_target,
	  m_goal->suppress_speaking, m_goal->skip_final_turn,
	  m_goal->stop_before_obstacle);*/
	if(!m_goal->suppress_speaking)
	{
		SpeakActionClient client("speak");
		client.waitForServer();
		homer_tts::SpeakGoal goal;
		goal.text = "I am driving to " + m_goal->goal_location;
		client.sendGoal(goal);
	}
	homer_mapnav_msgs::NavigateToPOI msg;
	msg.poi_name = m_goal->goal_location;
	msg.skip_final_turn = m_goal->skip_final_turn;
	msg.stop_before_obstacle = m_goal->stop_before_obstacle;
	msg.distance_to_target = m_goal->distance_to_target;
	m_navigate_to_poi_pub_.publish(msg);


	std::string arm = "kinova";
    ros::param::get("arm", arm);
	if (arm == "tiago")
	{
		ROS_INFO_STREAM("[DRIVE_TO] Arm is TIAGO, setting torso position");
		float torso_height = 0.15;
		ros::param::get("/torso_height/"+m_goal->goal_location, torso_height);
		setTorsoPosition(torso_height);
	}

	if (m_goal->plan_only_on_slam_map)
	{
		ros::Duration(2).sleep();
		ROS_INFO_STREAM("Reenabling map layers");
		switchMapLayers(true);
	}
	m_statemachine.setState(states::DRIVING_TO_GOAL_LOCATION);
	publishFeedback(homer_mapnav_msgs::DriveToFeedback::DRIVING_TO_GOAL_LOCATION,
			"Action goal received");
}

DriveTo::DriveTo(ros::NodeHandle n, std::string name)
	: m_as_(n, "drive_to", false), m_action_name_(name)
{
	this->init();
	m_nh_ = n;

	m_as_.registerGoalCallback(boost::bind(&DriveTo::driveToCallback, this));
	m_as_.start();

	m_obstacle_client_.reset(new DetectObstacleActionClient("detect_obstacle"));

	//	publishFeedback(homer_mapnav_msgs::DriveToFeedback::IDLE, "DriveTo Action
	//Loaded");

	m_target_unreachable_sub_ =
		m_nh_.subscribe<homer_mapnav_msgs::TargetUnreachable>(
				"/homer_navigation/target_unreachable", 1,
				&DriveTo::targetUnreachableCallback, this);

	m_target_reached_sub_ =
		m_nh_.subscribe<std_msgs::String>("/homer_navigation/target_reached", 1,
				&DriveTo::targetReachedCallback, this);

	m_map_layer_pub_ = m_nh_.advertise<homer_mapnav_msgs::MapLayers>(
			"/map_manager/toggle_map_visibility", 20);
	m_navigate_to_poi_pub_ = m_nh_.advertise<homer_mapnav_msgs::NavigateToPOI>("/homer_navigation/navigate_to_POI", 1);

	m_set_torso_pub_ = m_nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command",1);

	ROS_INFO_STREAM("DriveTo Action Node initialized");
}

void DriveTo::speakBlocked(std::string text)
{
	SpeakActionClient client("speak");
	bool wait_successful = client.waitForServer(ros::Duration(30.0));
	if( !wait_successful )
		ROS_ERROR_STREAM("drive_to: speakBlocked: waitForServer TIMEOUT");
	homer_tts::SpeakGoal goal;
	goal.text = text;
	client.sendGoal(goal);
	bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));
	if( !finished_before_timeout )
		ROS_ERROR_STREAM("drive_to: speakBlocked: waitForResult TIMEOUT");
}

DriveTo::~DriveTo()
{
}

void DriveTo::init()
{
	ADD_MACHINE_STATE(m_statemachine, states::IDLE);
	ADD_MACHINE_STATE(m_statemachine, states::FINISHED);
	ADD_MACHINE_STATE(m_statemachine, states::DRIVING_TO_GOAL_LOCATION);

	m_statemachine.setName("DriveTo Action State");
	m_statemachine.setState(states::IDLE);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_to_node");
	ros::NodeHandle n;
	DriveTo drive_to_node(n, ros::this_node::getName());
	ros::spin();
	return 0;
}

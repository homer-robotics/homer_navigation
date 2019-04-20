#ifndef FastNavigationModule_H
#define FastNavigationModule_H

#include <cmath>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <homer_mapnav_msgs/GetPointsOfInterest.h>
#include <homer_mapnav_msgs/NavigateToPOI.h>
#include <homer_mapnav_msgs/StartNavigation.h>
#include <homer_mapnav_msgs/TargetUnreachable.h>
#include <homer_ptu_msgs/CenterWorldPoint.h>
#include <homer_ptu_msgs/SetPanTilt.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <homer_nav_libs/Explorer/Explorer.h>
#include <homer_nav_libs/tools.h>

class Explorer;
/**
 * @class  HomerNavigationNode
 * @author Malte Knauf, Stephan Wirth, David Gossow (RX), Florian Polster
 * @brief  Performs autonomous navigation
 */
class HomerNavigationNode
{
public:
  /**
   * @brief   States of the state machines
   */
  enum ProcessState
  {
    IDLE,
    FOLLOWING_PATH,
    AVOIDING_COLLISION,
    FINAL_TURN
  };

  ProcessState m_state;

  /**
   * The constructor
   */
  HomerNavigationNode();

  /**
   * The destructor
   */
  virtual ~HomerNavigationNode();

  /** @brief Is called in constant intervals. */
  void idleProcess();

protected:
  /** @brief Handles incoming messages. */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void ignoreLaserCallback(const std_msgs::String::ConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void laserDataCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void startNavigationCallback(
      const homer_mapnav_msgs::StartNavigation::ConstPtr& msg);
  void
  moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void stopNavigationCallback(const std_msgs::Empty::ConstPtr& msg);

  void
  navigateToPOICallback(const homer_mapnav_msgs::NavigateToPOI::ConstPtr& msg);
  void unknownThresholdCallback(const std_msgs::Int8::ConstPtr& msg);
  void maxDepthMoveDistanceCallback(const std_msgs::Float32::ConstPtr& msg);

  /** @brief initializes and refreshs parameters */
  void loadParameters();

  /** @brief Is called when all modules are loaded and thread has started. */
  virtual void init();

  void initExplorer();

  void initNewTarget();

private:
  /** @brief Start navigation to m_Target on  last_map_data_ */
  void startNavigation();

  float getMinLaserDistance();
  void followPath();
  void avoidingCollision();
  void finalTurn();
  bool checkWaypoints();
  bool updateSpeeds();
  bool checkForObstacles();

  geometry_msgs::Point
  calculateMeanPoint(const std::vector<geometry_msgs::Point>& points);
  /** @brief Check if obstacles are blocking the way in last_map_data_ */
  bool obstacleOnPath();

  /** @brief calculate path from current robot position to target
   * approximation
   */
  void calculatePath(bool setMap = true);

  void setExplorerMap();

  /** @brief Send message containing current navigation path */
  void sendPathData();

  /** @brief Sends target reached and stops the robot. */
  void sendTargetReachedMsg();

  /**
   * @brief Sends a target unreachable with given reason and stops the robot.
   * @param reason reason for unreachable target (see
   * homer_mapnav_msgs::TargetUnreachable for possible reasons)
   */
  void sendTargetUnreachableMsg(int8_t reason);

  /** @brief reloads all params from the parameterserver */
  void refreshParamsCallback(const std_msgs::Empty::ConstPtr& msg);

  /** @brief Navigate robot to next waypoint */
  void performNextMove();

  /** @brief Finishes navigation or starts turning to target direction if the
   * target position has been reached */
  bool isTargetPositionReached();
  bool m_new_target;

  /** @return Angle from robot_pose_ to point in degrees */
  int angleToPointDeg(geometry_msgs::Point point);

  /** @brief Calculates current maximal backwards distance on map Data */
  bool backwardObstacle();

  /** @brief stops the Robot */
  void stopRobot();

  bool isInIgnoreList(std::string frame_id);

  void filterScanPoints(std::vector<geometry_msgs::Point>& points );

  /**
   * @brief Sets each cell of the map to -1 outside the bounding box
   *        containing the robot pose and the current target
   */
  void maskMap(nav_msgs::OccupancyGrid& cmap);

  /**
   * @brief Current path was finished (either successful or not),
   *        sets state machine to path planning to check if the robot is
   * already
   * at the goal
   */
  void currentPathFinished();

  // convenience math functions
  /**
   * Computes minimum turn angle from angle 1 to angle 2
   * @param angle1 from angle
   * @param angle2 to angle
   * @return minimal angle needed to turn from angle 1 to angle 2 [-Pi..Pi]
   */
  static float minTurnAngle(float angle1, float angle2);

  static double checkOrientation(double angle);


  /**
   * converts value from degree to radiant
   * @param deg Value in degree
   * @return value in radiants
   */
  static float deg2Rad(float deg)
  {
    return deg / 180.0 * M_PI;
  }

  /**
   * converts value from radiants to degrees
   * @param rad Value in radiants
   * @return value in degrees
   */
  static float rad2Deg(float rad)
  {
    return rad / M_PI * 180.0;
  }

  bool drawPolygon(std::vector<geometry_msgs::Point> vertices);
  void drawLine(std::vector<int>& data, int startX, int startY, int endX,
                int endY, int value);
  bool fillPolygon(std::vector<int>& data, int x, int y, int value);

  /** @brief calcs the maximal move distance from Laser and DepthData */
  void calcMaxMoveDist();

  /// @brief Worker instances
  Explorer* m_explorer;

  /// @brief Navigation options & data

  /** list of waypoints subsampled from m_PixelPath */
  std::vector<geometry_msgs::PoseStamped> m_waypoints;

  /** target point */
  geometry_msgs::Point m_target_point;

  /** target name if called via Navigate_to_POI */
  std::string m_target_name;

  /** orientation the robot should have at the target point */
  double m_target_orientation;

  /** allowed distance to target */
  float m_desired_distance;

  /** check if the final turn should be skipped */
  bool m_skip_final_turn;

  /**
   *  check if navigation should perform fast planning. In this mode a path is
   * only planned within
   *  a bounding box containing the robot pose and the target point
   */
  bool m_fast_path_planning;

  /** current pose of the robot */
  geometry_msgs::Pose m_robot_pose;

  /** last pose of the robot */
  geometry_msgs::Pose m_robot_last_pose;

  std::map<std::string, sensor_msgs::LaserScan::ConstPtr> m_scan_map;
  std::map<std::string, float> m_max_move_distances;

  std::string m_ignore_scan;

  /** time stamp of the last incoming laser scan */
  ros::Time m_last_laser_time;
  /** time stamp of the last incoming pose */
  ros::Time m_last_pose_time;

  /** Distance factor of a frontier cell considered save for exploration */
  float m_FrontierSafenessFactor;

  double m_SafePathWeight;

  /// @brief Configuration parameters

  /** Allowed distances of obstacles to robot. Robot must move within these
   * bounds */
  std::pair<float, float> m_AllowedObstacleDistance;

  /** Safe distances of obstacles to robot. If possible, robot should move
   * within these bounds */
  std::pair<float, float> m_SafeObstacleDistance;

  /** threshold to sample down waypoints */
  float m_waypoint_sampling_threshold;

  float m_max_move_distance;

  /** if distance to nearest obstacle is below collision distance trigger
   * collision avoidance */
  float m_collision_distance;

  /** if distance to nearest obstacle is below collision distance don't drive
   * backwards */
  float m_backward_collision_distance;
  /** do not drive back in collision avoidance when this near target */
  float m_collision_distance_near_target;

  /** if true, obstacles in path will be detected and path will be replanned
   */
  bool m_check_path;

  bool m_obstacle_on_path;
  geometry_msgs::Point m_obstacle_position;

  /** waypoints will only be checked for obstacles if they are closer than
   * check_path_max_distance to robot */
  float m_check_path_max_distance;
  ros::Time m_last_check_path_time;

  ros::Time m_unreachable_delay;

  /** do not replan if lisa avoids an obstacle, instead send target
   * unreachable*/
  bool m_stop_before_obstacle;

  bool m_avoided_collision;

  float m_min_turn_angle;
  float m_max_turn_speed;
  float m_min_turn_speed;
  float m_max_move_speed;
  float m_max_drive_angle;
  float m_waypoint_radius_factor;
  float m_speed_ramp;

  float m_distance_to_target;
  float m_angular_avoidance;

  float m_map_speed_factor;
  float m_waypoint_speed_factor;
  float m_obstacle_speed_factor;
  float m_target_distance_speed_factor;

  float m_min_y;
  float m_min_x;

  float m_callback_error_duration;

  bool m_seen_obstacle_before;

  bool m_path_reaches_target;
  bool m_initial_path_reaches_target;
  int m_unknown_threshold;

  geometry_msgs::Twist m_cmd_vel;

  /** last map data */
  nav_msgs::OccupancyGrid::ConstPtr m_map;

  // ros specific members
  tf::TransformListener m_transform_listener;

  // subscribers
  ros::Subscriber m_map_sub;
  ros::Subscriber m_pose_sub;
  ros::Subscriber m_laser_data_sub;
  ros::Subscriber m_down_laser_data_sub;
  ros::Subscriber m_laser_back_data_sub;
  ros::Subscriber m_start_navigation_sub;
  ros::Subscriber m_stop_navigation_sub;
  ros::Subscriber m_navigate_to_poi_sub;
  ros::Subscriber m_unknown_threshold_sub;
  ros::Subscriber m_refresh_param_sub;
  ros::Subscriber m_max_move_depth_sub;
  ros::Subscriber m_move_base_simple_goal_sub;
  ros::Subscriber m_ignore_laser_sub;

  // publishers
  ros::Publisher m_cmd_vel_pub;
  ros::Publisher m_target_reached_string_pub;
  ros::Publisher m_target_unreachable_pub;
  ros::Publisher m_path_pub;
  ros::Publisher m_ptu_center_world_point_pub;
  ros::Publisher m_set_pan_tilt_pub;

  // service clients
  ros::ServiceClient m_get_POIs_client;
};
#endif

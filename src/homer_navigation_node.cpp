#include "homer_navigation/homer_navigation_node.h"

HomerNavigationNode::HomerNavigationNode()
{
  ros::NodeHandle nh;

  // subscribers
  m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
      "/map", 3, &HomerNavigationNode::mapCallback, this);
  m_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/pose", 3, &HomerNavigationNode::poseCallback, this);
  m_laser_data_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 3, &HomerNavigationNode::laserDataCallback, this);
  m_down_laser_data_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/front_scan", 3, &HomerNavigationNode::laserDataCallback, this);
  m_start_navigation_sub = nh.subscribe<homer_mapnav_msgs::StartNavigation>(
      "/homer_navigation/start_navigation", 3,
      &HomerNavigationNode::startNavigationCallback, this);
  m_move_base_simple_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 3,
      &HomerNavigationNode::moveBaseSimpleGoalCallback,
      this);  // for RVIZ usage
  m_stop_navigation_sub = nh.subscribe<std_msgs::Empty>(
      "/homer_navigation/stop_navigation", 3,
      &HomerNavigationNode::stopNavigationCallback, this);
  m_navigate_to_poi_sub = nh.subscribe<homer_mapnav_msgs::NavigateToPOI>(
      "/homer_navigation/navigate_to_POI", 3,
      &HomerNavigationNode::navigateToPOICallback, this);
  m_unknown_threshold_sub = nh.subscribe<std_msgs::Int8>(
      "/homer_navigation/unknown_threshold", 3,
      &HomerNavigationNode::unknownThresholdCallback, this);
  m_refresh_param_sub = nh.subscribe<std_msgs::Empty>(
      "/homer_navigation/refresh_params", 3,
      &HomerNavigationNode::refreshParamsCallback, this);
  m_max_move_depth_sub = nh.subscribe<std_msgs::Float32>(
      "/homer_navigation/max_depth_move_distance", 3,
      &HomerNavigationNode::maxDepthMoveDistanceCallback, this);
  m_ignore_laser_sub = nh.subscribe<std_msgs::String>(
      "/homer_navigation/ignore_laser", 3,
      &HomerNavigationNode::ignoreLaserCallback, this);

  std::string cmd_vel_topic;
  nh.param<std::string>("/homer_cmd_vel_topic", cmd_vel_topic, "/robot_platform/cmd_vel");
  m_cmd_vel_pub =
      nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 3);
  m_target_reached_string_pub =
      nh.advertise<std_msgs::String>("/homer_navigation/target_reached", 3);
  // m_target_reached_empty_pub 	=
  // nh.advertise<std_msgs::Empty>("/homer_navigation/target_reached", 3);
  m_target_unreachable_pub = nh.advertise<homer_mapnav_msgs::TargetUnreachable>(
      "/homer_navigation/target_unreachable", 3);
  m_path_pub = nh.advertise<nav_msgs::Path>("/homer_navigation/path", 3);
  m_ptu_center_world_point_pub = nh.advertise<homer_ptu_msgs::CenterWorldPoint>(
      "/ptu/center_world_point", 3);
  m_set_pan_tilt_pub =
      nh.advertise<homer_ptu_msgs::SetPanTilt>("/ptu/set_pan_tilt", 3);

  m_get_POIs_client = nh.serviceClient<homer_mapnav_msgs::GetPointsOfInterest>(
      "/map_manager/get_pois");

  init();
}

void HomerNavigationNode::loadParameters()
{
  // Explorer constructor
  ros::param::param("/homer_navigation/allowed_obstacle_distance/min",
                    m_AllowedObstacleDistance.first, (float)0.3);
  ros::param::param("/homer_navigation/allowed_obstacle_distance/max",
                    m_AllowedObstacleDistance.second, (float)5.0);
  ros::param::param("/homer_navigation/safe_obstacle_distance/min",
                    m_SafeObstacleDistance.first, (float)0.7);
  ros::param::param("/homer_navigation/safe_obstacle_distance/max",
                    m_SafeObstacleDistance.second, (float)1.5);
  ros::param::param("/homer_navigation/frontier_safeness_factor",
                    m_FrontierSafenessFactor, (float)1.4);
  ros::param::param("/homer_navigation/safe_path_weight", m_SafePathWeight,
                    (double)1.2);
  ros::param::param("/homer_navigation/waypoint_sampling_threshold",
                    m_waypoint_sampling_threshold, (float)1.5);

  // check path
  ros::param::param("/homer_navigation/check_path", m_check_path, (bool)true);
  ros::param::param("/homer_navigation/check_path_max_distance",
                    m_check_path_max_distance, (float)2.0);

  // collision
  ros::param::param("/homer_navigation/collision_distance",
                    m_collision_distance, (float)0.3);
  ros::param::param("/homer_navigation/collision_distance_near_target",
                    m_collision_distance_near_target, (float)0.2);
  ros::param::param("/homer_navigation/backward_collision_distance",
                    m_backward_collision_distance, (float)0.5);

  // cmd_vel config values
  ros::param::param("/homer_navigation/min_turn_angle", m_min_turn_angle,
                    (float)0.15);
  ros::param::param("/homer_navigation/max_turn_speed", m_max_turn_speed,
                    (float)0.6);
  ros::param::param("/homer_navigation/min_turn_speed", m_min_turn_speed,
                    (float)0.3);
  ros::param::param("/homer_navigation/max_move_speed", m_max_move_speed,
                    (float)0.4);
  ros::param::param("/homer_navigation/max_drive_angle", m_max_drive_angle,
                    (float)0.6);
  ros::param::param("/homer_navigation/speed_ramp", m_speed_ramp, (float) 0.1);

  // caution factors
  ros::param::param("/homer_navigation/map_speed_factor", m_map_speed_factor,
                    (float)1.0);
  ros::param::param("/homer_navigation/waypoint_speed_factor",
                    m_waypoint_speed_factor, (float)1.0);
  ros::param::param("/homer_navigation/obstacle_speed_factor",
                    m_obstacle_speed_factor, (float)1.0);
  ros::param::param("/homer_navigation/target_distance_speed_factor",
                    m_target_distance_speed_factor, (float)0.4);

  // robot dimensions
  ros::param::param("/homer_navigation/min_x", m_min_x, (float)0.3);
  ros::param::param("/homer_navigation/min_y", m_min_y, (float)0.27);

  // error durations
  ros::param::param("/homer_navigation/callback_error_duration",
                    m_callback_error_duration, (float)0.3);

  ros::param::param("/homer_navigation/use_ptu", m_use_ptu, (bool)false);

  ros::param::param("/homer_navigation/unknown_threshold", m_unknown_threshold,
                    (int)50);
  ros::param::param("/homer_navigation/waypoint_radius_factor",
                    m_waypoint_radius_factor, (float)0.25);
  if (m_explorer)
  {
    delete m_explorer;
    m_explorer = 0;
  }
  initExplorer();
}

void HomerNavigationNode::init()
{
  m_explorer = 0;
  m_robot_pose = geometry_msgs::Pose();
  m_robot_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  m_robot_last_pose = m_robot_pose;
  m_avoided_collision = false;
  m_angular_avoidance = 0;
  m_last_check_path_time = ros::Time::now();
  m_unreachable_delay = ros::Time::now();
  m_ignore_scan = "";
  m_fast_path_planning = false;

  m_cmd_vel = geometry_msgs::Twist();

  m_obstacle_on_path = false;
  m_obstacle_position = geometry_msgs::Point();

  nav_msgs::OccupancyGrid tmp_map = nav_msgs::OccupancyGrid();
  tmp_map.header.frame_id = "/map";
  m_map = boost::make_shared<nav_msgs::OccupancyGrid>(tmp_map);

  loadParameters();

  m_state = IDLE;
}

HomerNavigationNode::~HomerNavigationNode()
{
  if (m_explorer)
  {
    delete m_explorer;
  }
}

void HomerNavigationNode::stopRobot()
{
  m_cmd_vel = geometry_msgs::Twist();
  m_cmd_vel_pub.publish(m_cmd_vel);
  m_cmd_vel_pub.publish(m_cmd_vel);
  m_cmd_vel_pub.publish(m_cmd_vel);
}

void HomerNavigationNode::idleProcess()
{
  if (m_state != IDLE)
  {
    if ((ros::Time::now() - m_last_laser_time) >
        ros::Duration(m_callback_error_duration))
    {
      ROS_ERROR_STREAM("Laser data timeout!\n");
      stopRobot();
    }
    if ((ros::Time::now() - m_last_pose_time) >
        ros::Duration(m_callback_error_duration))
    {
      ROS_ERROR_STREAM("Pose timeout!\n");
      stopRobot();
    }
  }
}

bool HomerNavigationNode::isInIgnoreList(std::string frame_id)
{
  std::regex reg("(^|\\s)" + frame_id + "(\\s|$)");
  return regex_match(m_ignore_scan, reg);
}

void HomerNavigationNode::setExplorerMap()
{
  // adding lasers to map
  nav_msgs::OccupancyGrid temp_map = *m_map;
  for (const auto& scan : m_scan_map)
  {
    if (!isInIgnoreList(scan.second->header.frame_id) &&
        (ros::Time::now() - scan.second->header.stamp) < ros::Duration(1))
    {
      std::vector<geometry_msgs::Point> scan_points;
      scan_points = map_tools::laser_msg_to_points(
          scan.second, m_transform_listener, "/map");
      for (const auto& point : scan_points)
      {
        Eigen::Vector2i map_point = map_tools::toMapCoords(point, m_map);
        int index = map_point.y() * m_map->info.width + map_point.x();
        if (index > 0 && index < m_map->info.width * m_map->info.height)
        {
          temp_map.data[index] = (int8_t)100;
        }
      }
    }
  }
  if (m_fast_path_planning)
  {
    //maskMap(temp_map);
  }
  m_explorer->setOccupancyMap(
      boost::make_shared<nav_msgs::OccupancyGrid>(temp_map));
}

void HomerNavigationNode::calculatePath(bool setMap)
{
  if (!m_explorer)
  {
    return;
  }
  if (isTargetPositionReached())
  {
    return;
  }

  if (setMap)
  {
    setExplorerMap();
  }
  m_explorer->setStart(map_tools::toMapCoords(m_robot_pose.position, m_map));

  bool success;
  std::vector<Eigen::Vector2i> tmp_pixel_path = m_explorer->getPath(success);
  if (!success)
  {
    ROS_WARN_STREAM("no path to target possible - drive to obstacle");
    m_obstacle_on_path = true;
  }
  else
  {
    m_obstacle_on_path = false;
    std::vector<Eigen::Vector2i> waypoint_pixels =
        m_explorer->sampleWaypointsFromPath(tmp_pixel_path,
                                            m_waypoint_sampling_threshold);
    m_waypoints.clear();
    ROS_INFO_STREAM("homer_navigation::calculatePath - Path Size: "
                    << waypoint_pixels.size());
    if (waypoint_pixels.size() > 0)
    {
      for (std::vector<Eigen::Vector2i>::iterator it = waypoint_pixels.begin();
           it != waypoint_pixels.end(); ++it)
      {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "/map";
        poseStamped.pose.position = map_tools::fromMapCoords(*it, m_map);
        poseStamped.pose.orientation.x = 0.0;
        poseStamped.pose.orientation.y = 0.0;
        poseStamped.pose.orientation.z = 0.0;
        poseStamped.pose.orientation.w = 1.0;
        m_waypoints.push_back(poseStamped);
      }
      if (m_path_reaches_target)
      {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "/map";
        poseStamped.pose.position = m_target_point;
        poseStamped.pose.orientation.x = 0;
        poseStamped.pose.orientation.y = 0;
        poseStamped.pose.orientation.z = 0;
        poseStamped.pose.orientation.w = 1;
        m_waypoints.push_back(poseStamped);
      }
      sendPathData();
    }
    else
    {
      sendTargetUnreachableMsg(
          homer_mapnav_msgs::TargetUnreachable::NO_PATH_FOUND);
    }

    m_last_laser_time = ros::Time::now();
    m_last_pose_time = ros::Time::now();
  }
}

void HomerNavigationNode::startNavigation()
{
  if (!m_explorer)
  {
    return;
  }
  if (isTargetPositionReached())
  {
    return;
  }
  // CHECK IF THERE IS A DIRECT PATH TO THE TARGET
  if (m_fast_path_planning)
  {
    m_waypoints.clear();

    geometry_msgs::PoseStamped poseStamped = geometry_msgs::PoseStamped();
    poseStamped.header.frame_id = "/map";
    poseStamped.pose.position = m_target_point;
    poseStamped.pose.orientation.w = 1;
    m_waypoints.push_back(poseStamped);

    if (!obstacleOnPath())
    {
      m_state = FOLLOWING_PATH;
      return;
    }
  }

  ROS_INFO_STREAM("Distance to target still too large ("
                  << m_distance_to_target
                  << "m; requested: " << m_desired_distance << "m)");

  setExplorerMap();
  // check if there still exists a path to the original target
  if (m_avoided_collision && m_initial_path_reaches_target &&
      m_stop_before_obstacle)
  {
    m_explorer->setStart(map_tools::toMapCoords(m_robot_pose.position, m_map));

    bool success;
    std::vector<Eigen::Vector2i> tmp_pixel_path = m_explorer->getPath(success);
    if (!success)
    {
      ROS_INFO_STREAM(
          "Initial path would have reached target, new path does not. "
          << "Sending target unreachable.");
      sendTargetUnreachableMsg(
          homer_mapnav_msgs::TargetUnreachable::LASER_OBSTACLE);
      return;
    }
  }

  m_explorer->setStart(map_tools::toMapCoords(m_robot_pose.position, m_map));
  Eigen::Vector2i new_target_approx = m_explorer->getNearestAccessibleTarget(
      map_tools::toMapCoords(m_target_point, m_map));

  geometry_msgs::Point new_target_approx_world =
      map_tools::fromMapCoords(new_target_approx, m_map);
  ROS_INFO_STREAM(
      "start Navigation: Approx target: " << new_target_approx_world);

  m_path_reaches_target =
      (map_tools::distance(new_target_approx_world, m_target_point) <
       m_desired_distance);
  m_initial_path_reaches_target = m_path_reaches_target;

  if (map_tools::distance(m_robot_pose.position, new_target_approx_world) <
      m_map->info.resolution * 1.5)
  {
    ROS_WARN_STREAM("close to aprox target - final turn");
    m_state = FINAL_TURN;
  }

  bool new_approx_is_better =
      (map_tools::distance(m_robot_pose.position, m_target_point) -
       map_tools::distance(new_target_approx_world, m_target_point)) >
      2 * m_desired_distance;
  if (!new_approx_is_better && !m_path_reaches_target)
  {
    ROS_WARN_STREAM("No better way to target found, turning and then reporting "
                    "as "
                    "unreachable."
                    << std::endl
                    << "Distance to target: " << m_distance_to_target
                    << "m; requested: " << m_desired_distance << "m");
    m_state = FINAL_TURN;
  }
  else
  {
    m_explorer->setTarget(new_target_approx);
    m_state = FOLLOWING_PATH;

    calculatePath(false);
  }
}

void HomerNavigationNode::sendPathData()
{
  nav_msgs::Path msg;
  msg.header.frame_id = "/map";
  msg.header.stamp = ros::Time::now();
  if (m_waypoints.size() > 0)
  {
    msg.poses = m_waypoints;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = m_robot_pose;
    pose_stamped.header.frame_id = "/map";
    pose_stamped.header.stamp = ros::Time::now();
    msg.poses.insert(msg.poses.begin(), pose_stamped);
  }
  m_path_pub.publish(msg);
}

void HomerNavigationNode::sendTargetReachedMsg()
{
  stopRobot();
  m_state = IDLE;
  std_msgs::String reached_string_msg;
  reached_string_msg.data = m_target_name;
  m_target_reached_string_pub.publish(reached_string_msg);
  m_waypoints.clear();
  ROS_INFO_STREAM("=== Reached Target " << m_target_name << " ===");
}

void HomerNavigationNode::sendTargetUnreachableMsg(int8_t reason)
{
  stopRobot();
  if (ros::Time::now() - m_unreachable_delay > ros::Duration(5))
  {
    m_unreachable_delay = ros::Time::now();
  }
  else if (ros::Time::now() - m_unreachable_delay > ros::Duration(3))
  {
    m_state = IDLE;
    homer_mapnav_msgs::TargetUnreachable unreachable_msg;
    unreachable_msg.reason = reason;
    unreachable_msg.point = geometry_msgs::PointStamped();
    unreachable_msg.point.header.frame_id = "/map";
    unreachable_msg.point.point = m_obstacle_position;
    m_target_unreachable_pub.publish(unreachable_msg);
    m_waypoints.clear();
    ROS_INFO_STREAM("=== TargetUnreachableMsg ===");
  }
}

bool HomerNavigationNode::isTargetPositionReached()
{
  if (m_distance_to_target < m_desired_distance && !m_new_target)
  {
    ROS_INFO_STREAM("Target position reached. Distance to target: "
                    << m_distance_to_target
                    << "m. Desired distance:" << m_desired_distance << "m");
    stopRobot();
    m_waypoints.clear();
    m_state = FINAL_TURN;
    ROS_INFO_STREAM("Turning to look-at point");
    return true;
  }
  else
  {
    return false;
  }
}

void HomerNavigationNode::filterScanPoints(
    std::vector<geometry_msgs::Point>& points)
{
  std::vector<int> remove_list;
  for (int i = 0; i < points.size(); i++)
  {
    bool found = false;
    for (int j = i; j < points.size(); j++)
    {
      if (map_tools::distance(points[i], points[j]) < 0.03)
      {
        found = true;
      }
    }

    if (!found)
    {
      remove_list.push_back(i);
    }
  }

  for (int i = remove_list.size() - 1; i >= 0; i--)
  {
    points.erase(points.begin() + remove_list[i]);
  }
}

geometry_msgs::Point HomerNavigationNode::calculateMeanPoint(
    const std::vector<geometry_msgs::Point>& points)
{
  geometry_msgs::Point mean_point = geometry_msgs::Point();
  for (auto const& point : points)
  {
    mean_point.x += point.x;
    mean_point.y += point.y;
  }
  if (points.size() > 0)
  {
    mean_point.x /= points.size();
    mean_point.y /= points.size();
  }
  return mean_point;
}

bool HomerNavigationNode::obstacleOnPath()
{
  m_last_check_path_time = ros::Time::now();
  if (m_waypoints.size() == 0)
  {
    ROS_DEBUG_STREAM("no path found for finding an obstacle");
    return false;
  }

  std::vector<geometry_msgs::Point> obstacle_vec;

  for (auto const& scan : m_scan_map)
  {
    if (!isInIgnoreList(scan.second->header.frame_id) &&
        (ros::Time::now() - scan.second->header.stamp) < ros::Duration(1))
    {
      std::vector<geometry_msgs::Point> scan_points;
      scan_points = map_tools::laser_msg_to_points(
          scan.second, m_transform_listener, "/map");

      filterScanPoints(scan_points);
      geometry_msgs::Point lp;
      geometry_msgs::Point p;

      for (unsigned i = 0; i < m_waypoints.size(); i++)
      {
        if (i == 0)
        {
          lp = m_robot_pose.position;
          p = m_waypoints.at(i).pose.position;
        }
        else
        {
          lp = m_waypoints.at(i - 1).pose.position;
          p = m_waypoints.at(i).pose.position;
        }
        if (map_tools::distance(m_robot_pose.position, p) >
            m_check_path_max_distance * 2)
        {
          if (obstacle_vec.size() > 0)
          {
            m_obstacle_position = calculateMeanPoint(obstacle_vec);
            ROS_DEBUG_STREAM("found obstacle at: " << m_obstacle_position);
            return true;
          }
          else
          {
            continue;
          }
        }
        for (int k = 0; k < 4; k++)
        {
          geometry_msgs::Point t;
          t.x = lp.x + (p.x - lp.x) * k / 4.0;
          t.y = lp.y + (p.y - lp.y) * k / 4.0;
          for (const auto& sp : scan_points)
          {
            if (map_tools::distance(sp, t) < m_AllowedObstacleDistance.first)
            {
              if (map_tools::distance(m_robot_pose.position, sp) <
                  m_check_path_max_distance)
              {
                obstacle_vec.push_back(sp);
              }
            }
          }
        }
      }
    }
  }
  if (obstacle_vec.size() > 0)
  {
    m_obstacle_position = calculateMeanPoint(obstacle_vec);
    ROS_DEBUG_STREAM("found obstacle at: " << m_obstacle_position);
    return true;
  }
  else
  {
    return false;
  }
}

float HomerNavigationNode::getMinLaserDistance()
{
  float min = 10;
  for (auto const& scan : m_scan_map)
  {
    if (!isInIgnoreList(scan.second->header.frame_id) &&
        (ros::Time::now() - scan.second->header.stamp) < ros::Duration(1))
    {
      for (auto const& range : scan.second->ranges)
      {
        if (range < min && range > scan.second->range_min &&
            range < scan.second->range_max)
        {
          min = range;
        }
      }
    }
  }
  return min;
}

bool HomerNavigationNode::checkWaypoints()
{
  if (m_waypoints.size() == 0)
  {
    ROS_WARN_STREAM("No waypoints but trying to perform next move! "
                    "Skipping.");
    startNavigation();
    return false;
  }
  // if we have accidentaly skipped waypoints, recalculate path
  float minDistance = FLT_MAX;
  float distance;
  unsigned nearestWaypoint = 0;
  for (unsigned i = 0; i < m_waypoints.size(); i++)
  {
    distance = map_tools::distance(m_robot_pose.position,
                                   m_waypoints[i].pose.position);
    if (distance < minDistance)
    {
      nearestWaypoint = i;
      minDistance = distance;
    }
  }
  if (nearestWaypoint != 0)
  {
    // if the target is nearer than the waypoint don't recalculate
    if (m_waypoints.size() != 2)
    {
      ROS_WARN_STREAM("Waypoints skipped. Recalculating path!");
      calculatePath();
      if (m_state != FOLLOWING_PATH)
      {
        return false;
      }
    }
    else
    {
      m_waypoints.erase(m_waypoints.begin());
    }
  }

  Eigen::Vector2i waypointPixel =
      map_tools::toMapCoords(m_waypoints[0].pose.position, m_map);
  float obstacleDistanceMap = getMinLaserDistance();
  float waypointRadius = m_waypoint_radius_factor * obstacleDistanceMap;
  float distanceToWaypoint =
      map_tools::distance(m_robot_pose.position, m_waypoints[0].pose.position);

  waypointRadius = std::min((float)0.3, waypointRadius);
  if ((waypointRadius < m_map->info.resolution) || (m_waypoints.size() == 1))
  {
    waypointRadius = m_map->info.resolution;
  }

  if (m_waypoints.size() != 0)
  {
    Eigen::Vector2f wp_to_robot;
    wp_to_robot[0] = m_robot_pose.position.x - m_waypoints[0].pose.position.x;
    wp_to_robot[1] = m_robot_pose.position.y - m_waypoints[0].pose.position.y;

    Eigen::Vector2f wp_0_to_wp_1;
    wp_0_to_wp_1[0] =
        m_waypoints[1].pose.position.x - m_waypoints[0].pose.position.x;
    wp_0_to_wp_1[1] =
        m_waypoints[1].pose.position.y - m_waypoints[0].pose.position.y;

    float dot = wp_to_robot[0] * wp_0_to_wp_1[0] +
                wp_to_robot[1] *
                    wp_0_to_wp_1[1];  // dot product of point_to_start * line

    // check if current waypoint has been reached
    bool waypoint_reached = (dot >= 0 && distanceToWaypoint < 2 * waypointRadius || distanceToWaypoint < waypointRadius);
    if (waypoint_reached && m_waypoints.size() > 1)
    {
      m_waypoints.erase(m_waypoints.begin());
    }
    if (m_waypoints.size() == 1 && distanceToWaypoint < m_map->info.resolution)
    {
      m_waypoints.erase(m_waypoints.begin());
    }
  }

  if (m_waypoints.size() == 0)
  {
    ROS_INFO_STREAM("Last waypoint reached");
    currentPathFinished();
    return false;
  }

  sendPathData();

  if (m_use_ptu)
  {
    ROS_INFO_STREAM("Moving PTU to center next Waypoint");
    homer_ptu_msgs::CenterWorldPoint ptu_msg;
    ptu_msg.point.x = m_waypoints[0].pose.position.x;
    ptu_msg.point.y = m_waypoints[0].pose.position.y;
    ptu_msg.point.z = 0;
    ptu_msg.permanent = true;
    m_ptu_center_world_point_pub.publish(ptu_msg);
  }

  return true;
}

bool HomerNavigationNode::updateSpeeds()
{
  float distanceToWaypoint =
      map_tools::distance(m_robot_pose.position, m_waypoints[0].pose.position);
  float angleToWaypoint = angleToPointDeg(m_waypoints[0].pose.position);
  float angle = deg2Rad(angleToWaypoint);

  // LINEAR SPEED CALCULATION
  if (m_avoided_collision)
  {
    if (std::fabs(angleToWaypoint) < 10)
    {
      m_avoided_collision = false;
    }
  }
  // Angle is bigger than maximal driving angle
  else if (std::fabs(angle) > m_max_drive_angle)
  {
    m_cmd_vel.linear.x = 0.0;
  }
  // There is an obstacle on the path and we are one meter away
  else if (m_obstacle_on_path &&
           map_tools::distance(m_robot_pose.position, m_obstacle_position) <
               1.0)
  {
    m_cmd_vel.linear.x = 0.0;
    angle = deg2Rad(angleToPointDeg(m_obstacle_position));

    if (std::fabs(angle) < m_min_turn_angle)
    {
      ROS_INFO_STREAM("angle = " << angle);
      m_avoided_collision = true;
      m_initial_path_reaches_target = true;
      m_stop_before_obstacle = true;
      startNavigation();
      return false;
    }
  }
  // Normal speed calculation
  else
  {
    std::map<std::string, float> m_min_speed_map;

    m_min_speed_map["move_dist"] =
        m_max_move_speed * m_max_move_distance * m_obstacle_speed_factor;
    m_min_speed_map["laser"] =
        getMinLaserDistance() * m_map_speed_factor * m_max_move_speed;
    m_min_speed_map["waypoint"] = 1;
    m_min_speed_map["target"] =
        std::max((float)0.2, m_distance_to_target * m_distance_to_target *
                                 m_target_distance_speed_factor);
    m_min_speed_map["max_speed"] = m_max_move_speed;

    float new_speed = 200;
    std::string mins = "";

    for (auto& speed : m_min_speed_map)
    {
      if (speed.second < new_speed)
      {
        new_speed = speed.second;
        mins = speed.first;
      }
    }

    ROS_DEBUG_STREAM("max speed " << new_speed << " because of " << mins);

    // RAMP FOR SMOOTHER ACCELERATION
    if (new_speed < m_cmd_vel.linear.x)
    {
      m_cmd_vel.linear.x = new_speed;
    }
    else
    {
      m_cmd_vel.linear.x += std::min((float)(new_speed - m_cmd_vel.linear.x), m_speed_ramp);
    }
  }

  m_cmd_vel.linear.x *=
      std::fabs((m_max_turn_speed - std::fabs(angle)) / m_max_turn_speed);

  // angular speed calculation
  angle *= std::fabs(angle) * 3.0 + std::fabs(m_cmd_vel.linear.x / 2.0);
  if (angle < 0)
  {
    m_cmd_vel.angular.z = std::max(angle, -m_max_turn_speed);
  }
  else
  {
    m_cmd_vel.angular.z = std::min(angle, m_max_turn_speed);
  }

  ROS_DEBUG_STREAM("Driving & turning"
                   << std::endl
                   << "linear: " << m_cmd_vel.linear.x
                   << " angular: " << m_cmd_vel.angular.z << std::endl
                   << "distanceToWaypoint: " << distanceToWaypoint << std::endl
                   << " angleToWaypoint: " << angleToWaypoint << std::endl);
  return true;
}

bool HomerNavigationNode::checkForObstacles()
{
  if ((ros::Time::now() - m_last_check_path_time) > ros::Duration(0.2))
  {
    if (obstacleOnPath())
    {
        if(m_stop_before_obstacle)
            sendTargetUnreachableMsg(30);

      if (m_seen_obstacle_before)
      {
        float distanceToObstacle =
            map_tools::distance(m_robot_pose.position, m_obstacle_position);
        if (!m_obstacle_on_path && distanceToObstacle < 1.0)
        {
          // stopRobot();
          calculatePath();
          return false;
        }
        else
        {
          calculatePath();
        }
      }
      m_seen_obstacle_before = true;
    }
    else
    {
      m_seen_obstacle_before = false;
      m_obstacle_on_path = false;
    }
  }
  return true;
}

void HomerNavigationNode::followPath()
{
  if (isTargetPositionReached())
  {
    return;
  }

  // check if an obstacle is on the current path
  if (m_check_path)
  {
    if (!checkForObstacles())
    {
      return;
    }
  }

  // check if the first waypoint is still the right target
  if (!checkWaypoints())
  {
    return;
  }

  // update m_cmd_vel speeds
  if (!updateSpeeds())
  {
    return;
  }

  m_cmd_vel_pub.publish(m_cmd_vel);
}

void HomerNavigationNode::avoidingCollision()
{
  if (isTargetPositionReached())
  {
    return;
  }
  else if ((m_max_move_distance <= m_collision_distance &&
            m_waypoints.size() > 1) ||
           m_max_move_distance <= m_collision_distance_near_target)
  {
    ROS_WARN_STREAM("Maximum driving distance too short ("
                    << m_max_move_distance << "m)! Moving back.");
    geometry_msgs::Twist cmd_vel_msg;
    if (!HomerNavigationNode::backwardObstacle())
    {
      cmd_vel_msg.linear.x = -0.2;
    }
    else
    {
      if (m_angular_avoidance == 0)
      {
        float angleToWaypoint = angleToPointDeg(m_waypoints[0].pose.position);
        if (angleToWaypoint < 0)
        {
          m_angular_avoidance = -0.4;
        }
        else
        {
          m_angular_avoidance = 0.4;
        }
      }
      cmd_vel_msg.angular.z = m_angular_avoidance;
    }
    m_cmd_vel_pub.publish(cmd_vel_msg);
  }
  else
  {
    m_angular_avoidance = 0;
    m_avoided_collision = true;
    ROS_WARN_STREAM("Collision avoided. Updating path.");
    currentPathFinished();
  }
}

void HomerNavigationNode::finalTurn()
{
  if (m_use_ptu)
  {
    // reset PTU
    homer_ptu_msgs::SetPanTilt msg;
    msg.absolute = true;
    msg.panAngle = 0;
    msg.tiltAngle = 0;
    m_set_pan_tilt_pub.publish(msg);
  }

  if (m_skip_final_turn)
  {
    ROS_INFO_STREAM("Final turn skipped. Target reached.");
    if (m_path_reaches_target)
    {
      sendTargetReachedMsg();
    }
    else
    {
      sendTargetUnreachableMsg(
          homer_mapnav_msgs::TargetUnreachable::NO_PATH_FOUND);
    }
    return;
  }

  float turnAngle =
      minTurnAngle(tf::getYaw(m_robot_pose.orientation), m_target_orientation);
  ROS_DEBUG_STREAM("homer_navigation::PerformNextMove:: Final Turn. Robot "
                   "orientation: "
                   << rad2Deg(tf::getYaw(m_robot_pose.orientation))
                   << ". Target orientation: " << rad2Deg(m_target_orientation)
                   << "homer_navigation::PerformNextMove:: turnAngle: "
                   << rad2Deg(turnAngle));

  if (std::fabs(turnAngle) < m_min_turn_angle)
  {
    ROS_INFO_STREAM(":::::::NEAREST WALKABLE TARGET REACHED BECAUSE lower "
                    << m_min_turn_angle);
    ROS_INFO_STREAM("target angle = " << m_target_orientation);
    ROS_INFO_STREAM("is angle = " << tf::getYaw(m_robot_pose.orientation));
    ROS_INFO_STREAM("m_distance_to_target = " << m_distance_to_target);
    ROS_INFO_STREAM("m_desired_distance = " << m_desired_distance);
    if (m_path_reaches_target)
    {
      sendTargetReachedMsg();
    }
    else
    {
      sendTargetUnreachableMsg(
          homer_mapnav_msgs::TargetUnreachable::NO_PATH_FOUND);
    }
    return;
  }
  else
  {
    if (turnAngle < 0)
    {
      turnAngle =
          std::max(std::min(turnAngle, -m_min_turn_speed), -m_max_turn_speed);
    }
    else
    {
      turnAngle =
          std::min(std::max(turnAngle, m_min_turn_speed), m_max_turn_speed);
    }
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = turnAngle;
    m_cmd_vel_pub.publish(cmd_vel_msg);
  }
}

void HomerNavigationNode::performNextMove()
{
  if (m_state == FOLLOWING_PATH)
  {
    followPath();
  }
  else if (m_state == AVOIDING_COLLISION)
  {
    avoidingCollision();
  }
  else if (m_state == FINAL_TURN)
  {
    finalTurn();
  }
}

void HomerNavigationNode::currentPathFinished()
{
  ROS_INFO_STREAM("Current path was finished, initiating recalculation.");
  m_waypoints.clear();
  stopRobot();
  startNavigation();
}

int HomerNavigationNode::angleToPointDeg(geometry_msgs::Point target)
{
  double cx = m_robot_pose.position.x;
  double cy = m_robot_pose.position.y;
  int targetAngle = rad2Deg(atan2(target.y - cy, target.x - cx));
  int currentAngle = rad2Deg(tf::getYaw(m_robot_pose.orientation));

  int angleDiff = targetAngle - currentAngle;
  angleDiff = (angleDiff + 180) % 360 - 180;
  if (angleDiff < -180)
  {
    angleDiff += 360;
  }
  return angleDiff;
}

bool HomerNavigationNode::drawPolygon(
    std::vector<geometry_msgs::Point> vertices)
{
  if (vertices.size() == 0)
  {
    ROS_INFO_STREAM("No vertices given!");
    return false;
  }
  // make temp. map
  std::vector<int> data(m_map->info.width * m_map->info.height);
  for (int i = 0; i < data.size(); i++)
  {
    data[i] = 0;
  }

  // draw the lines surrounding the polygon
  for (unsigned int i = 0; i < vertices.size(); i++)
  {
    int i2 = (i + 1) % vertices.size();
    drawLine(data, vertices[i].x, vertices[i].y, vertices[i2].x, vertices[i2].y,
             30);
  }
  // calculate a point in the middle of the polygon
  float midX = 0;
  float midY = 0;
  for (unsigned int i = 0; i < vertices.size(); i++)
  {
    midX += vertices[i].x;
    midY += vertices[i].y;
  }
  midX /= vertices.size();
  midY /= vertices.size();
  // fill polygon
  return fillPolygon(data, (int)midX, (int)midY, 30);
}

void HomerNavigationNode::drawLine(std::vector<int>& data, int startX,
                                   int startY, int endX, int endY, int value)
{
  // bresenham algorithm
  int x, y, t, dist, xerr, yerr, dx, dy, incx, incy;
  // compute distances
  dx = endX - startX;
  dy = endY - startY;

  // compute increment
  if (dx < 0)
  {
    incx = -1;
    dx = -dx;
  }
  else
  {
    incx = dx ? 1 : 0;
  }

  if (dy < 0)
  {
    incy = -1;
    dy = -dy;
  }
  else
  {
    incy = dy ? 1 : 0;
  }

  // which distance is greater?
  dist = (dx > dy) ? dx : dy;
  // initializing
  x = startX;
  y = startY;
  xerr = dx;
  yerr = dy;

  // compute cells
  for (t = 0; t < dist; t++)
  {
    int index = x + m_map->info.width * y;
    if (index < 0 || index > data.size())
    {
      continue;
    }
    data[index] = value;

    xerr += dx;
    yerr += dy;
    if (xerr > dist)
    {
      xerr -= dist;
      x += incx;
    }
    if (yerr > dist)
    {
      yerr -= dist;
      y += incy;
    }
  }
}

bool HomerNavigationNode::fillPolygon(std::vector<int>& data, int x, int y,
                                      int value)
{
  int index = x + m_map->info.width * y;
  bool tmp = false;

  if ((int)m_map->data.at(index) > 90)
  {
    tmp = true;
  }
  if (data[index] != value)
  {
    data[index] = value;
    if (fillPolygon(data, x + 1, y, value))
    {
      tmp = true;
    }
    if (fillPolygon(data, x - 1, y, value))
    {
      tmp = true;
    }
    if (fillPolygon(data, x, y + 1, value))
    {
      tmp = true;
    }
    if (fillPolygon(data, x, y - 1, value))
    {
      tmp = true;
    }
  }
  return tmp;
}

bool HomerNavigationNode::backwardObstacle()
{
  std::vector<geometry_msgs::Point> vertices;
  geometry_msgs::Point base_link_point;
  geometry_msgs::Point map_point;
  Eigen::Vector2i map_coord;

  std::vector<float> x;
  std::vector<float> y;

  x.push_back(-m_min_x - m_backward_collision_distance);
  y.push_back(m_min_y);

  x.push_back(-m_min_x - m_backward_collision_distance);
  y.push_back(-m_min_y);

  x.push_back(-0.1);
  y.push_back(-m_min_y);

  x.push_back(-0.1);
  y.push_back(m_min_y);

  for (int i = 0; i < x.size(); i++)
  {
    base_link_point.x = x[i];
    base_link_point.y = y[i];
    map_coord = map_tools::toMapCoords(
        map_tools::transformPoint(base_link_point, m_transform_listener,
                                  "/base_link", "/map"),
        m_map);
    map_point.x = map_coord.x();
    map_point.y = map_coord.y();
    vertices.push_back(map_point);
  }

  return drawPolygon(vertices);
}

void HomerNavigationNode::maskMap(nav_msgs::OccupancyGrid& cmap)
{
  // generate bounding box
  ROS_INFO_STREAM("Calculating Bounding box for fast planning");
  Eigen::Vector2i pose_pixel =
      map_tools::toMapCoords(m_robot_pose.position, m_map);
  Eigen::Vector2i target_pixel = map_tools::toMapCoords(m_target_point, m_map);
  Eigen::Vector2i safe_pixel_distance(
      m_AllowedObstacleDistance.first / m_map->info.resolution * 4,
      m_AllowedObstacleDistance.first / m_map->info.resolution * 4);
  Eigen::AlignedBox2i planning_box;
  planning_box.extend(pose_pixel);
  planning_box.extend(target_pixel);
  ROS_INFO_STREAM("Bounding Box: (" << planning_box.min() << " "
                                    << planning_box.max());
  Eigen::AlignedBox2i safe_planning_box(
      planning_box.min() - safe_pixel_distance,
      planning_box.max() + safe_pixel_distance);
  ROS_INFO_STREAM("safe Bounding Box: (" << safe_planning_box.min() << " "
                                         << safe_planning_box.max());
  ROS_INFO_STREAM(
      "min in m: " << map_tools::fromMapCoords(safe_planning_box.min(), m_map));
  ROS_INFO_STREAM(
      "max in m: " << map_tools::fromMapCoords(safe_planning_box.max(), m_map));
  for (size_t x = 0; x < m_map->info.width; x++)
  {
    for (size_t y = 0; y < m_map->info.height; y++)
    {
      if (!safe_planning_box.contains(Eigen::Vector2i(x, y)))
      {
        cmap.data.at(y * m_map->info.width + x) = -1;
      }
    }
  }
}

// convenience math functions
float HomerNavigationNode::minTurnAngle(float angle1, float angle2)
{
  angle1 *= 180.0 / M_PI;
  angle2 *= 180.0 / M_PI;

  int diff = angle2 - angle1;
  diff = (diff + 180) % 360 - 180;
  if (diff < -180)
  {
    diff += 360;
  }

  float ret = static_cast<double>(diff) * M_PI / 180.0;
  return ret;
}

double HomerNavigationNode::checkOrientation(double angle)
{
    //if(0 <= angle && angle <= 2 * M_PI )
    //{
        return angle;
    //}
    //else
    //{
        //return 0;
    //}
}

void HomerNavigationNode::refreshParamsCallback(
    const std_msgs::Empty::ConstPtr& msg)
{
  (void)msg;
  ROS_INFO_STREAM("Refreshing Parameters");
  loadParameters();
}

void HomerNavigationNode::initExplorer()
{
  if (!m_explorer)
  {
    if (m_map->info.resolution != 0)
    {
      float resolution = m_map->info.resolution;
      m_explorer = new Explorer(m_AllowedObstacleDistance.first / resolution,
                                m_AllowedObstacleDistance.second / resolution,
                                m_SafeObstacleDistance.first / resolution,
                                m_SafeObstacleDistance.second / resolution,
                                m_SafePathWeight, m_FrontierSafenessFactor,
                                m_unknown_threshold);
      setExplorerMap();
    }
  }
}

void HomerNavigationNode::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  m_map = msg;
  initExplorer();
}

void HomerNavigationNode::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  m_robot_last_pose = m_robot_pose;
  m_robot_pose = msg->pose;
  m_last_pose_time = ros::Time::now();
  m_new_target = false;
  m_distance_to_target =
      map_tools::distance(m_robot_pose.position, m_target_point);
  performNextMove();
}

void HomerNavigationNode::calcMaxMoveDist()
{
  m_max_move_distance = 99;
  std::string laser = "";
  for (auto d : m_max_move_distances)
  {
    if (d.first == "depth" || (ros::Time::now() - m_scan_map[d.first]->header.stamp) <
        ros::Duration(1))
    {
      m_max_move_distance = std::min(m_max_move_distance, d.second);
      laser = d.first;
    }
  }
  if (m_max_move_distance < m_cmd_vel.linear.x || m_max_move_distance < 0.05)
  {
    if (m_state == FOLLOWING_PATH)
    {
      stopRobot();
      m_state = AVOIDING_COLLISION;
      ROS_WARN_STREAM("Collision detected while following path!");
    }
  }
}

void HomerNavigationNode::ignoreLaserCallback(
    const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("changed ignore laser to: " << msg->data);
  m_ignore_scan = msg->data;
}

void HomerNavigationNode::maxDepthMoveDistanceCallback(
    const std_msgs::Float32::ConstPtr& msg)
{
  m_max_move_distances["depth"] = msg->data;
  calcMaxMoveDist();
}

void HomerNavigationNode::laserDataCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg)
{
  m_scan_map[msg->header.frame_id] = msg;
  m_last_laser_time = ros::Time::now();
  if (m_state != IDLE)
  {
    if (!isInIgnoreList(msg->header.frame_id))
    {
      m_max_move_distances[msg->header.frame_id] =
          map_tools::get_max_move_distance(
              map_tools::laser_msg_to_points(msg, m_transform_listener, "/base_"
                                                                        "link"),
              m_min_x, m_min_y);
    }
    else
    {
      m_max_move_distances[msg->header.frame_id] = 99;
    }
    calcMaxMoveDist();
  }
}

void HomerNavigationNode::initNewTarget()
{
  m_initial_path_reaches_target = false;
  m_avoided_collision = false;
  m_new_target = true;
}
void HomerNavigationNode::startNavigationCallback(
    const homer_mapnav_msgs::StartNavigation::ConstPtr& msg)
{
  m_target_point = msg->goal.position;
  m_target_orientation = checkOrientation(tf::getYaw(msg->goal.orientation));
  m_desired_distance =
      msg->distance_to_target < 0.1 ? 0.1 : msg->distance_to_target;
  m_skip_final_turn = msg->skip_final_turn;
  m_fast_path_planning = msg->fast_planning;
  m_target_name = "";

  ROS_INFO_STREAM("Navigating to target "
                  << m_target_point.x << ", " << m_target_point.y
                  << "\nTarget orientation: " << m_target_orientation
                  << "Desired distance to target: " << m_desired_distance);
  initNewTarget();
  startNavigation();
}

void HomerNavigationNode::moveBaseSimpleGoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ros::Time start = ros::Time::now();
  if (msg->header.frame_id != "map")
  {
    tf::StampedTransform transform;
    if (m_transform_listener.waitForTransform(
            "map", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0)))
    {
      try
      {
        m_transform_listener.lookupTransform("map", msg->header.frame_id,
                                             msg->header.stamp, transform);
        tf::Vector3 targetPos(msg->pose.position.x, msg->pose.position.y,
                              msg->pose.position.z);
        targetPos = transform * targetPos;
        m_target_point.x = targetPos.getX();
        m_target_point.y = targetPos.getY();
        m_target_orientation = checkOrientation(tf::getYaw(
            transform *
            tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                           msg->pose.orientation.z, msg->pose.orientation.w)));
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR_STREAM(ex.what());
        return;
      }
    }
    else
    {
      try
      {
        m_transform_listener.lookupTransform("map", msg->header.frame_id,
                                             ros::Time(0), transform);
        tf::Vector3 targetPos(msg->pose.position.x, msg->pose.position.y,
                              msg->pose.position.z);
        targetPos = transform * targetPos;
        m_target_point.x = targetPos.getX();
        m_target_point.y = targetPos.getY();
        m_target_orientation = checkOrientation(tf::getYaw(
            transform *
            tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                           msg->pose.orientation.z, msg->pose.orientation.w)));
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR_STREAM(ex.what());
        return;
      }
    }
  }
  else
  {
    m_target_point = msg->pose.position;
    m_target_orientation = checkOrientation(tf::getYaw(msg->pose.orientation));
  }
  m_desired_distance = 0.1;
  m_skip_final_turn = false;
  m_fast_path_planning = false;

  ROS_INFO_STREAM("Navigating to target via Move Base Simple x: "
                  << m_target_point.x << ", y: " << m_target_point.y
                  << "\nTarget orientation: " << m_target_orientation
                  << " Desired distance to target: " << m_desired_distance
                  << "\nframe_id: " << msg->header.frame_id);
  initNewTarget();
  startNavigation();
  ROS_INFO_STREAM("TIME: " << (ros::Time::now() - start).toSec());
}

void HomerNavigationNode::navigateToPOICallback(
    const homer_mapnav_msgs::NavigateToPOI::ConstPtr& msg)
{
  homer_mapnav_msgs::GetPointsOfInterest srv;
  m_get_POIs_client.call(srv);
  std::vector<homer_mapnav_msgs::PointOfInterest>::iterator it;
  for (it = srv.response.poi_list.pois.begin();
       it != srv.response.poi_list.pois.end(); ++it)
  {
    if (it->name == msg->poi_name)
    {
      m_target_point = it->pose.position;
      m_target_orientation = checkOrientation(tf::getYaw(it->pose.orientation));
      m_desired_distance =
          msg->distance_to_target < 0.1 ? 0.1 : msg->distance_to_target;
      m_skip_final_turn = msg->skip_final_turn;
      m_fast_path_planning = false;
      m_target_name = msg->poi_name;
      m_stop_before_obstacle = msg->stop_before_obstacle;

      ROS_INFO_STREAM("Navigating to target "
                      << m_target_point.x << ", " << m_target_point.y
                      << "\nTarget orientation: " << m_target_orientation
                      << "Desired distance to target: " << m_desired_distance);
      initNewTarget();
      startNavigation();
      return;
    }
  }

  ROS_ERROR_STREAM("No point of interest with name '"
                   << msg->poi_name << "' found in current poi list");
  sendTargetUnreachableMsg(homer_mapnav_msgs::TargetUnreachable::UNKNOWN);
}

void HomerNavigationNode::stopNavigationCallback(
    const std_msgs::Empty::ConstPtr& msg)
{
  (void)msg;
  ROS_INFO_STREAM("Stopping navigation.");
  m_state = IDLE;
  stopRobot();

  m_waypoints.clear();
}

void HomerNavigationNode::unknownThresholdCallback(
    const std_msgs::Int8::ConstPtr& msg)
{
  if (m_explorer)
  {
    m_explorer->setUnknownThreshold(static_cast<int>(msg->data));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "homer_navigation");

  HomerNavigationNode node;

  ros::Rate rate(50);

  while (ros::ok())
  {
    ros::spinOnce();
    node.idleProcess();
    rate.sleep();
  }

  return 0;
}

#include "homer_navigation/homer_navigation_node.h"

HomerNavigationNode::HomerNavigationNode() {
  ros::NodeHandle nh;

  // subscribers
  m_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
      "/map", 1, &HomerNavigationNode::mapCallback, this);
  m_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/pose", 1, &HomerNavigationNode::poseCallback, this);
  m_laser_data_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1, &HomerNavigationNode::laserDataCallback, this);
  m_down_laser_data_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/front_scan", 1, &HomerNavigationNode::downlaserDataCallback, this);
  m_start_navigation_sub = nh.subscribe<homer_mapnav_msgs::StartNavigation>(
      "/homer_navigation/start_navigation", 1,
      &HomerNavigationNode::startNavigationCallback, this);
  m_move_base_simple_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1,
      &HomerNavigationNode::moveBaseSimpleGoalCallback,
      this);  // for RVIZ usage
  m_stop_navigation_sub = nh.subscribe<std_msgs::Empty>(
      "/homer_navigation/stop_navigation", 1,
      &HomerNavigationNode::stopNavigationCallback, this);
  m_navigate_to_poi_sub = nh.subscribe<homer_mapnav_msgs::NavigateToPOI>(
      "/homer_navigation/navigate_to_POI", 1,
      &HomerNavigationNode::navigateToPOICallback, this);
  m_unknown_threshold_sub = nh.subscribe<std_msgs::Int8>(
      "/homer_navigation/unknown_threshold", 1,
      &HomerNavigationNode::unknownThresholdCallback, this);
  m_refresh_param_sub = nh.subscribe<std_msgs::Empty>(
      "/homer_navigation/refresh_params", 1,
      &HomerNavigationNode::refreshParamsCallback, this);
  m_max_move_depth_sub = nh.subscribe<std_msgs::Float32>(
      "/homer_navigation/max_depth_move_distance", 1,
      &HomerNavigationNode::maxDepthMoveDistanceCallback, this);

  m_cmd_vel_pub =
      nh.advertise<geometry_msgs::Twist>("/robot_platform/cmd_vel", 1);
  m_target_reached_string_pub =
      nh.advertise<std_msgs::String>("/homer_navigation/target_reached", 1);
  // m_target_reached_empty_pub 	=
  // nh.advertise<std_msgs::Empty>("/homer_navigation/target_reached", 1);
  m_target_unreachable_pub = nh.advertise<homer_mapnav_msgs::TargetUnreachable>(
      "/homer_navigation/target_unreachable", 1);
  m_path_pub = nh.advertise<nav_msgs::Path>("/homer_navigation/path", 1);
  m_ptu_center_world_point_pub = nh.advertise<homer_ptu_msgs::CenterWorldPoint>(
      "/ptu/center_world_point", 1);
  m_set_pan_tilt_pub =
      nh.advertise<homer_ptu_msgs::SetPanTilt>("/ptu/set_pan_tilt", 1);
  m_debug_pub = nh.advertise<std_msgs::String>("/homer_navigation/debug", 1);

  m_get_POIs_client = nh.serviceClient<homer_mapnav_msgs::GetPointsOfInterest>(
      "/map_manager/get_pois");

  m_MainMachine.setName("HomerNavigation Main");
  ADD_MACHINE_STATE(m_MainMachine, IDLE);
  ADD_MACHINE_STATE(m_MainMachine, AWAITING_PATHPLANNING_MAP);
  ADD_MACHINE_STATE(m_MainMachine, FOLLOWING_PATH);
  ADD_MACHINE_STATE(m_MainMachine, AVOIDING_COLLISION);
  ADD_MACHINE_STATE(m_MainMachine, FINAL_TURN);

  init();
}

void HomerNavigationNode::loadParameters() {
  // Explorer constructor
  ros::param::param("/homer_mapping/resolution", m_resolution, (double)0.05);
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
  m_AllowedObstacleDistance.first /= m_resolution;
  m_AllowedObstacleDistance.second /= m_resolution;
  m_SafeObstacleDistance.first /= m_resolution;
  m_SafeObstacleDistance.second /= m_resolution;

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
  ros::param::param("/homer_navigation/no_replanning_on_collision",
                    m_no_replanning_on_collision, (bool)false);

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
}

void HomerNavigationNode::init() {
  m_max_move_sick = 40.0;
  m_max_move_down = 40.0;
  m_max_move_depth = 40.0;
  m_robot_pose.position.x = 0.0;
  m_robot_pose.position.y = 0.0;
  m_robot_pose.position.z = 0.0;
  m_robot_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  m_robot_last_pose = m_robot_pose;
  m_avoided_collision = false;
  m_act_speed = 0;
  m_angular_avoidance = 0;
  m_last_calculations_failed = 0;
  m_last_check_path_res = false;
  m_new_target = true;

  loadParameters();

  m_explorer = new Explorer(
      m_AllowedObstacleDistance.first, m_AllowedObstacleDistance.second,
      m_SafeObstacleDistance.first, m_SafeObstacleDistance.second,
      m_SafePathWeight, m_FrontierSafenessFactor, m_unknown_threshold);
  m_last_map_data = new std::vector<int8_t>(0);

  m_MainMachine.setState(IDLE);
}

HomerNavigationNode::~HomerNavigationNode() {
  if (m_explorer) {
    delete m_explorer;
  }
  if (m_last_map_data) {
    delete m_last_map_data;
  }
}

void HomerNavigationNode::stopRobot() {
  m_act_speed = 0;
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0;
  cmd_vel_msg.linear.y = 0;
  cmd_vel_msg.linear.z = 0;
  cmd_vel_msg.angular.x = 0;
  cmd_vel_msg.angular.y = 0;
  cmd_vel_msg.angular.z = 0;
  m_cmd_vel_pub.publish(cmd_vel_msg);
  ros::Duration(0.1).sleep();
  m_cmd_vel_pub.publish(cmd_vel_msg);
  ros::Duration(0.1).sleep();
  m_cmd_vel_pub.publish(cmd_vel_msg);
}

void HomerNavigationNode::idleProcess() {
  if (m_MainMachine.state() == FOLLOWING_PATH) {
    if ((ros::Time::now() - m_last_laser_time) >
        ros::Duration(m_callback_error_duration)) {
      ROS_ERROR_STREAM("Laser data timeout!\n");
      stopRobot();
    }
    if ((ros::Time::now() - m_last_pose_time) >
        ros::Duration(m_callback_error_duration)) {
      ROS_ERROR_STREAM("Pose timeout!\n");
      stopRobot();
    }
  }
}

void HomerNavigationNode::calculatePath() {
  if (m_distance_to_target < m_desired_distance && !m_new_target) {
    m_path_reaches_target = true;
    return;
  }
  m_explorer->setOccupancyMap(m_width, m_width, m_origin,
                              &(*m_last_map_data)[0]);
  m_explorer->setStart(
      map_tools::toMapCoords(m_robot_pose.position, m_origin, m_resolution));

  bool success;
  m_pixel_path = m_explorer->getPath(success);
  if (!success) {
    ROS_WARN_STREAM("No path found for navigation");
    m_last_calculations_failed++;
    ROS_INFO_STREAM("last_calculation_failed: " << m_last_calculations_failed);
    if (m_last_calculations_failed > 8) {
      sendTargetUnreachableMsg(
          homer_mapnav_msgs::TargetUnreachable::NO_PATH_FOUND);
      if (m_initial_path_reaches_target) {
        // TODO(mroosen) temp obstacle in path
      }
    }
  } else {
    m_last_calculations_failed = 0;
    std::vector<Eigen::Vector2i> waypoint_pixels =
        m_explorer->sampleWaypointsFromPath(m_pixel_path,
                                            m_waypoint_sampling_threshold);
    m_waypoints.clear();
    ROS_INFO_STREAM("homer_navigation::calculatePath - Path Size: "
                    << waypoint_pixels.size());
    if (waypoint_pixels.size() > 0) {
      for (std::vector<Eigen::Vector2i>::iterator it = waypoint_pixels.begin();
           it != waypoint_pixels.end(); ++it) {
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "/map";
        poseStamped.pose.position =
            map_tools::fromMapCoords(*it, m_origin, m_resolution);
        poseStamped.pose.orientation.x = 0.0;
        poseStamped.pose.orientation.y = 0.0;
        poseStamped.pose.orientation.z = 0.0;
        poseStamped.pose.orientation.w = 1.0;
        m_waypoints.push_back(poseStamped);
      }
      sendPathData();
    } else {
      sendTargetUnreachableMsg(
          homer_mapnav_msgs::TargetUnreachable::NO_PATH_FOUND);
    }

    m_last_laser_time = ros::Time::now();
    m_last_pose_time = ros::Time::now();
  }
}

void HomerNavigationNode::startNavigation() {
  if (m_distance_to_target < m_desired_distance && !m_new_target) {
    ROS_INFO_STREAM(
        "Will not (re-)plan path: Target position already reached.");
    m_path_reaches_target = true;
    targetPositionReached();
    return;
  }
  if (m_initial_path_reaches_target && m_no_replanning_on_collision) {
    ROS_INFO_STREAM(
        "Collision avoided and option no_replanning_on_collision is set.");
    ROS_INFO_STREAM("Sending target unreachable.");
    sendTargetUnreachableMsg(
        homer_mapnav_msgs::TargetUnreachable::LASER_OBSTACLE);
  }
  ROS_INFO_STREAM("Distance to target still too large ("
                  << m_distance_to_target
                  << "m; requested: " << m_desired_distance << "m)");

  if (m_fast_path_planning) {
    maskMap();
  }

  m_explorer->setOccupancyMap(m_width, m_width, m_origin,
                              &(*m_last_map_data)[0]);
  m_explorer->setStart(
      map_tools::toMapCoords(m_robot_pose.position, m_origin, m_resolution));
  Eigen::Vector2i new_target_approx = m_explorer->getNearestAccessibleTarget(
      map_tools::toMapCoords(m_target_point, m_origin, m_resolution));

  geometry_msgs::Point new_target_approx_world =
      map_tools::fromMapCoords(new_target_approx, m_origin, m_resolution);
  ROS_INFO_STREAM(
      "start Navigation: Approx target: " << new_target_approx_world);

  m_path_reaches_target =
      (map_tools::distance(new_target_approx_world, m_target_point) <
       m_desired_distance);
  m_initial_path_reaches_target = m_path_reaches_target;

  bool new_approx_is_better =
      (map_tools::distance(m_robot_pose.position, m_target_point) -
       map_tools::distance(new_target_approx_world, m_target_point)) >
      2 * m_desired_distance;
  if (!new_approx_is_better && !m_path_reaches_target) {
    ROS_WARN_STREAM(
        "No better way to target found, turning and then reporting as "
        "unreachable."
        << std::endl
        << "Distance to target: " << m_distance_to_target
        << "m; requested: " << m_desired_distance << "m");
    m_MainMachine.setState(FINAL_TURN);
  } else {
    m_explorer->setTarget(new_target_approx);
    m_MainMachine.setState(FOLLOWING_PATH);
    calculatePath();
  }
}

void HomerNavigationNode::sendPathData() {
  nav_msgs::Path msg;
  msg.header.frame_id = "/map";
  msg.header.stamp = ros::Time::now();
  if (m_waypoints.size() > 0) {
    msg.poses = m_waypoints;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = m_robot_pose;
    pose_stamped.header.frame_id = "/map";
    pose_stamped.header.stamp = ros::Time::now();
    msg.poses.insert(msg.poses.begin(), pose_stamped);
  }
  m_path_pub.publish(msg);
}

void HomerNavigationNode::sendTargetReachedMsg() {
  stopRobot();
  m_MainMachine.setState(IDLE);
  std_msgs::String reached_string_msg;
  reached_string_msg.data = m_target_name;
  m_target_reached_string_pub.publish(reached_string_msg);
  m_waypoints.clear();
  nav_msgs::Path empty_path_msg;
  empty_path_msg.poses = m_waypoints;
  m_path_pub.publish(empty_path_msg);
  ROS_INFO_STREAM("=== Reached Target " << m_target_name << " ===");
}

void HomerNavigationNode::sendTargetUnreachableMsg(int8_t reason) {
  stopRobot();
  m_MainMachine.setState(IDLE);
  homer_mapnav_msgs::TargetUnreachable unreachable_msg;
  unreachable_msg.reason = reason;
  m_target_unreachable_pub.publish(unreachable_msg);
  m_waypoints.clear();
  nav_msgs::Path empty_path_msg;
  empty_path_msg.poses = m_waypoints;
  m_path_pub.publish(empty_path_msg);
  ROS_INFO_STREAM("=== TargetUnreachableMsg ===");
}

void HomerNavigationNode::targetPositionReached() {
  ROS_INFO_STREAM("Target position reached. Distance to target: "
                  << m_distance_to_target
                  << "m. Desired distance:" << m_desired_distance << "m");
  stopRobot();
  m_waypoints.clear();
  sendPathData();
  m_MainMachine.setState(FINAL_TURN);
  ROS_INFO_STREAM("Turning to look-at point");
}

bool HomerNavigationNode::checkPath() {
  if (m_pixel_path.size() != 0) {
    for (unsigned i = 0; i < m_pixel_path.size() - 1; i++) {
      geometry_msgs::Point p =
          map_tools::fromMapCoords(m_pixel_path.at(i), m_origin, m_resolution);
      if (map_tools::distance(m_robot_pose.position, p) >
          m_check_path_max_distance) {
        return true;
      }
      for (int a = 0; a < 5; a++) {
        if (map_tools::findValue(
                m_last_map_data, m_width, m_height,
                m_pixel_path[i].x() +
                    (m_pixel_path[i + 1].x() - m_pixel_path[i].x()) * a / 4,
                m_pixel_path[i].y() +
                    (m_pixel_path[i + 1].y() - m_pixel_path[i].y()) * a / 4,
                90, 2)) {
          ROS_WARN_STREAM("Obstacle detected in current path recalculating");
          return false;
        }
      }
    }
  }
}

void HomerNavigationNode::handleCollision() {
  if (m_MainMachine.state() == FOLLOWING_PATH) {
    stopRobot();
    m_MainMachine.setState(AVOIDING_COLLISION);
    ROS_WARN_STREAM("Collision detected while following path!");
  }
}

void HomerNavigationNode::performNextMove() {
  if (m_MainMachine.state() == FOLLOWING_PATH) {
    if (m_distance_to_target < m_desired_distance && !m_new_target) {
      ROS_INFO_STREAM("Desired distance to target was reached.");
      targetPositionReached();
      return;
    }
    // if we have accidentaly skipped waypoints, recalculate path
    float minDistance = FLT_MAX;
    float distance;
    unsigned nearestWaypoint = 0;
    for (unsigned i = 0; i < m_waypoints.size(); i++) {
      distance = map_tools::distance(m_robot_pose.position,
                                     m_waypoints[i].pose.position);
      if (distance < minDistance) {
        nearestWaypoint = i;
        minDistance = distance;
      }
    }
    if (nearestWaypoint != 0) {
      // if the target is nearer than the waypoint don't recalculate
      if (m_waypoints.size() != 2) {
        ROS_WARN_STREAM("Waypoints skipped. Recalculating path!");
        calculatePath();
        if (m_MainMachine.state() != FOLLOWING_PATH) {
          return;
        }
      } else {
        m_waypoints.erase(m_waypoints.begin());
      }
    }

    Eigen::Vector2i waypointPixel = map_tools::toMapCoords(
        m_waypoints[0].pose.position, m_origin, m_resolution);
    float obstacleDistanceMap = m_explorer->getObstacleTransform()->getValue(
                                    waypointPixel.x(), waypointPixel.y()) *
                                m_resolution;
    float waypointRadius = m_waypoint_radius_factor * obstacleDistanceMap;

    if ((waypointRadius < m_resolution) || (m_waypoints.size() == 1)) {
      waypointRadius = m_resolution;
    }

    if (m_waypoints.size() != 0) {
      // calculate distance between last_pose->current_pose and waypoint
      Eigen::Vector2f line;  // direction of the line
      line[0] = m_robot_pose.position.x - m_robot_last_pose.position.x;
      line[1] = m_robot_pose.position.y - m_robot_last_pose.position.y;
      Eigen::Vector2f
          point_to_start;  // vector from the point to the start of the line
      point_to_start[0] =
          m_robot_last_pose.position.x - m_waypoints[0].pose.position.x;
      point_to_start[1] =
          m_robot_last_pose.position.y - m_waypoints[0].pose.position.y;
      float dot =
          point_to_start[0] * line[0] +
          point_to_start[1] * line[1];  // dot product of point_to_start * line
      Eigen::Vector2f distance;  // shortest distance vector from point to line
      distance[0] = point_to_start[0] - dot * line[0];
      distance[1] = point_to_start[1] - dot * line[1];
      float distance_to_waypoint =
          sqrt((double)(distance[0] * distance[0] + distance[1] * distance[1]));

      // check if current waypoint has been reached
      if ((distance_to_waypoint < waypointRadius && m_waypoints.size() > 1) ||
          (m_distance_to_target < waypointRadius)) {
        m_waypoints.erase(m_waypoints.begin());
      }
    }

    sendPathData();
    // last wayoint reached
    if (m_waypoints.size() == 0) {
      ROS_INFO_STREAM("Last waypoint reached");
      currentPathFinished();
      return;
    }

    if (m_use_ptu) {
      ROS_INFO_STREAM("Moving PTU to center next Waypoint");
      homer_ptu_msgs::CenterWorldPoint ptu_msg;
      ptu_msg.point.x = m_waypoints[0].pose.position.x;
      ptu_msg.point.y = m_waypoints[0].pose.position.y;
      ptu_msg.point.z = 0;
      ptu_msg.permanent = true;
      m_ptu_center_world_point_pub.publish(ptu_msg);
    }

    float distanceToWaypoint = map_tools::distance(
        m_robot_pose.position, m_waypoints[0].pose.position);
    float angleToWaypoint = angleToPointDeg(m_waypoints[0].pose.position);
    if (angleToWaypoint < -180) {
      angleToWaypoint += 360;
    }
    float angle = deg2Rad(angleToWaypoint);

    // linear speed calculation
    if (m_avoided_collision) {
      if (std::abs(angleToWaypoint) < 10) {
        m_avoided_collision = false;
      }
    } else if (abs(angle) > m_max_drive_angle) {
      m_act_speed = 0.0;
    } else {
      float obstacleMapDistance = 1;
      for (int wpi = -1; wpi < std::min((int)m_waypoints.size(), (int)2);
           wpi++) {
        Eigen::Vector2i robotPixel;
        if (wpi == -1) {
          robotPixel = map_tools::toMapCoords(m_robot_pose.position, m_origin,
                                              m_resolution);
        } else {
          robotPixel = map_tools::toMapCoords(m_waypoints[wpi].pose.position,
                                              m_origin, m_resolution);
        }
        obstacleMapDistance =
            std::min((float)obstacleMapDistance,
                     (float)(m_explorer->getObstacleTransform()->getValue(
                                 robotPixel.x(), robotPixel.y()) *
                             m_resolution));
      }

      float max_move_distance_speed =
          m_max_move_speed * m_max_move_distance * m_obstacle_speed_factor;
      float max_map_distance_speed =
          m_max_move_speed * obstacleMapDistance * m_map_speed_factor;
      m_act_speed = std::min(
          std::max((float)0.1,
                   m_distance_to_target * m_target_distance_speed_factor),
          std::min(std::min(m_max_move_speed, max_move_distance_speed),
                   std::min(max_map_distance_speed,
                            distanceToWaypoint * m_waypoint_speed_factor)));
      std_msgs::String tmp;
      std::stringstream str;
      str << "m_obstacle_speed " << max_move_distance_speed
          << " max_map_distance_speed " << max_map_distance_speed;
      tmp.data = str.str();
      m_debug_pub.publish(tmp);
    }

    // angular speed calculation
    if (angle < 0) {
      angle = std::max(angle * (float)0.8, -m_max_turn_speed);
      m_act_speed = m_act_speed + angle / 2.0;
      if (m_act_speed < 0) {
        m_act_speed = 0;
      }
    } else {
      angle = std::min(angle * (float)0.8, m_max_turn_speed);
      m_act_speed = m_act_speed - angle / 2.0;
      if (m_act_speed < 0) {
        m_act_speed = 0;
      }
    }
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = m_act_speed;
    cmd_vel_msg.angular.z = angle;
    m_cmd_vel_pub.publish(cmd_vel_msg);

    ROS_DEBUG_STREAM("Driving & turning"
                     << std::endl
                     << "linear: " << m_act_speed << " angular: " << angle
                     << std::endl
                     << "distanceToWaypoint:" << distanceToWaypoint
                     << "angleToWaypoint: " << angleToWaypoint << std::endl);
  } else if (m_MainMachine.state() == AVOIDING_COLLISION) {
    if (m_distance_to_target < m_desired_distance && !m_new_target) {
      ROS_INFO_STREAM("Collision detected near target. Switch to final turn.");
      targetPositionReached();
    } else if (m_max_move_distance <= m_collision_distance &&
                   m_waypoints.size() > 1 ||
               m_max_move_distance <= m_collision_distance_near_target) {
      ROS_WARN_STREAM("Maximum driving distance too short ("
                      << m_max_move_distance << "m)! Moving back.");
      geometry_msgs::Twist cmd_vel_msg;
      if (!HomerNavigationNode::backwardObstacle()) {
        cmd_vel_msg.linear.x = -0.2;
      } else {
        if (m_angular_avoidance == 0) {
          float angleToWaypoint = angleToPointDeg(m_waypoints[0].pose.position);
          if (angleToWaypoint < -180) {
            angleToWaypoint += 360;
          }
          if (angleToWaypoint < 0) {
            m_angular_avoidance = -0.4;
          } else {
            m_angular_avoidance = 0.4;
          }
        }
        cmd_vel_msg.angular.z = m_angular_avoidance;
      }
      m_cmd_vel_pub.publish(cmd_vel_msg);
    } else {
      m_angular_avoidance = 0;
      m_avoided_collision = true;
      ROS_WARN_STREAM("Collision avoided. Updating path.");
      currentPathFinished();
    }
  } else if (m_MainMachine.state() == FINAL_TURN) {
    if (m_use_ptu) {
      // reset PTU
      homer_ptu_msgs::SetPanTilt msg;
      msg.absolute = true;
      msg.panAngle = 0;
      msg.tiltAngle = 0;
      m_set_pan_tilt_pub.publish(msg);
    }

    if (m_skip_final_turn) {
      ROS_INFO_STREAM("Final turn skipped. Target reached.");
      if (m_path_reaches_target) {
        sendTargetReachedMsg();
      } else {
        sendTargetUnreachableMsg(
            homer_mapnav_msgs::TargetUnreachable::NO_PATH_FOUND);
      }
      return;
    }

    float turnAngle = minTurnAngle(tf::getYaw(m_robot_pose.orientation),
                                   m_target_orientation);
    ROS_DEBUG_STREAM(
        "homer_navigation::PerformNextMove:: Final Turn. Robot orientation: "
        << rad2Deg(tf::getYaw(m_robot_pose.orientation))
        << ". Target orientation: " << rad2Deg(m_target_orientation)
        << "homer_navigation::PerformNextMove:: turnAngle: "
        << rad2Deg(turnAngle));

    if (std::fabs(turnAngle) < m_min_turn_angle) {
      ROS_INFO_STREAM(":::::::NEAREST WALKABLE TARGET REACHED BECAUSE lower "
                      << m_min_turn_angle);
      ROS_INFO_STREAM("target angle = " << m_target_orientation);
      ROS_INFO_STREAM("is angle = " << tf::getYaw(m_robot_pose.orientation));
      ROS_INFO_STREAM("m_distance_to_target = " << m_distance_to_target);
      ROS_INFO_STREAM("m_desired_distance = " << m_desired_distance);
      if (m_path_reaches_target) {
        sendTargetReachedMsg();
      } else {
        sendTargetUnreachableMsg(
            homer_mapnav_msgs::TargetUnreachable::NO_PATH_FOUND);
      }
      return;
    } else {
      if (turnAngle < 0) {
        turnAngle =
            std::max(std::min(turnAngle, -m_min_turn_speed), -m_max_turn_speed);
      } else {
        turnAngle =
            std::min(std::max(turnAngle, m_min_turn_speed), m_max_turn_speed);
      }
      geometry_msgs::Twist cmd_vel_msg;
      cmd_vel_msg.angular.z = turnAngle;
      m_cmd_vel_pub.publish(cmd_vel_msg);
    }
  }
}

void HomerNavigationNode::currentPathFinished() {
  ROS_INFO_STREAM("Current path was finished, initiating recalculation.");
  m_waypoints.clear();
  stopRobot();
  m_MainMachine.setState(AWAITING_PATHPLANNING_MAP);
}

int HomerNavigationNode::angleToPointDeg(geometry_msgs::Point target) {
  double cx = m_robot_pose.position.x;
  double cy = m_robot_pose.position.y;
  int targetAngle = rad2Deg(atan2(target.y - cy, target.x - cx));
  int currentAngle = rad2Deg(tf::getYaw(m_robot_pose.orientation));

  int angleDiff = targetAngle - currentAngle;
  angleDiff = (angleDiff + 180) % 360 - 180;
  return angleDiff;
}

bool HomerNavigationNode::drawPolygon(
    std::vector<geometry_msgs::Point> vertices) {
  if (vertices.size() == 0) {
    ROS_INFO_STREAM("No vertices given!");
    return false;
  }
  // make temp. map
  std::vector<int> data(m_width * m_height);
  for (int i = 0; i < data.size(); i++) {
    data[i] = 0;
  }

  // draw the lines surrounding the polygon
  for (unsigned int i = 0; i < vertices.size(); i++) {
    int i2 = (i + 1) % vertices.size();
    drawLine(data, vertices[i].x, vertices[i].y, vertices[i2].x, vertices[i2].y,
             30);
  }
  // calculate a point in the middle of the polygon
  float midX = 0;
  float midY = 0;
  for (unsigned int i = 0; i < vertices.size(); i++) {
    midX += vertices[i].x;
    midY += vertices[i].y;
  }
  midX /= vertices.size();
  midY /= vertices.size();
  // fill polygon
  return fillPolygon(data, (int)midX, (int)midY, 30);
}

void HomerNavigationNode::drawLine(std::vector<int>& data, int startX,
                                   int startY, int endX, int endY, int value) {
  // bresenham algorithm
  int x, y, t, dist, xerr, yerr, dx, dy, incx, incy;
  // compute distances
  dx = endX - startX;
  dy = endY - startY;

  // compute increment
  if (dx < 0) {
    incx = -1;
    dx = -dx;
  } else {
    incx = dx ? 1 : 0;
  }

  if (dy < 0) {
    incy = -1;
    dy = -dy;
  } else {
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
  for (t = 0; t < dist; t++) {
    int index = x + m_width * y;
    if (index < 0 || index > data.size()) {
      continue;
    }
    data[index] = value;

    xerr += dx;
    yerr += dy;
    if (xerr > dist) {
      xerr -= dist;
      x += incx;
    }
    if (yerr > dist) {
      yerr -= dist;
      y += incy;
    }
  }
}

bool HomerNavigationNode::fillPolygon(std::vector<int>& data, int x, int y,
                                      int value) {
  int index = x + m_width * y;
  bool tmp = false;

  if ((int)m_last_map_data->at(index) > 90) {
    tmp = true;
  }
  if (data[index] != value) {
    data[index] = value;
    if (fillPolygon(data, x + 1, y, value)) {
      tmp = true;
    }
    if (fillPolygon(data, x - 1, y, value)) {
      tmp = true;
    }
    if (fillPolygon(data, x, y + 1, value)) {
      tmp = true;
    }
    if (fillPolygon(data, x, y - 1, value)) {
      tmp = true;
    }
  }
  return tmp;
}

bool HomerNavigationNode::backwardObstacle() {
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

  for (int i = 0; i < x.size(); i++) {
    base_link_point.x = x[i];
    base_link_point.y = y[i];
    map_coord = map_tools::toMapCoords(
        map_tools::transformPoint(base_link_point, m_transform_listener,
                                  "/base_link", "/map"),
        m_origin, m_resolution);
    map_point.x = map_coord.x();
    map_point.y = map_coord.y();
    vertices.push_back(map_point);
  }

  return drawPolygon(vertices);
}

void HomerNavigationNode::maskMap() {
  // generate bounding box
  ROS_INFO_STREAM("Calculating Bounding box for fast planning");
  Eigen::Vector2i pose_pixel =
      map_tools::toMapCoords(m_robot_pose.position, m_origin, m_resolution);
  Eigen::Vector2i target_pixel =
      map_tools::toMapCoords(m_target_point, m_origin, m_resolution);
  Eigen::Vector2i safe_pixel_distance(m_AllowedObstacleDistance.first * 4,
                                      m_AllowedObstacleDistance.first * 4);
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
  ROS_INFO_STREAM("min in m: " << map_tools::fromMapCoords(
                      safe_planning_box.min(), m_origin, m_resolution));
  ROS_INFO_STREAM("max in m: " << map_tools::fromMapCoords(
                      safe_planning_box.max(), m_origin, m_resolution));
  for (size_t x = 0; x < m_width; x++) {
    for (size_t y = 0; y < m_width; y++) {
      if (!safe_planning_box.contains(Eigen::Vector2i(x, y))) {
        m_last_map_data->at(y * m_width + x) = -1;
      }
    }
  }
}

// convenience math functions
float HomerNavigationNode::minTurnAngle(float angle1, float angle2) {
  angle1 *= 180.0 / M_PI;
  angle2 *= 180.0 / M_PI;

  int diff = angle2 - angle1;
  diff = (diff + 180) % 360 - 180;
  if (diff < -180) {
    diff += 360;
  }

  float ret = static_cast<double>(diff) * M_PI / 180.0;
  return ret;
}

void HomerNavigationNode::refreshParamsCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO_STREAM("Refreshing Parameters");
  loadParameters();
}

void HomerNavigationNode::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  if (msg->info.height != msg->info.width) {
    ROS_ERROR_STREAM("Incoming Map not quadratic. No map update!");
    return;
  }
  if (m_last_map_data) {
    delete m_last_map_data;
  }

  m_last_map_data = new std::vector<int8_t>(msg->data);
  m_origin = msg->info.origin;
  m_width = msg->info.width;
  m_height = msg->info.height;
  m_resolution = msg->info.resolution;

  switch (m_MainMachine.state()) {
    case AWAITING_PATHPLANNING_MAP:
      startNavigation();
      break;
    case FOLLOWING_PATH: {
      if (m_check_path) {
        if (!checkPath()) {
          if (!m_last_check_path_res) {
            calculatePath();
          }
          m_last_check_path_res = false;
        } else {
          m_last_check_path_res = true;
        }
      }
      break;
    }
  }
}

void HomerNavigationNode::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  m_robot_last_pose = m_robot_pose;
  m_robot_pose = msg->pose;
  m_last_pose_time = ros::Time::now();
  m_distance_to_target =
      map_tools::distance(m_robot_pose.position, m_target_point);
  m_new_target = false;
  performNextMove();
}

void HomerNavigationNode::calcMaxMoveDist() {
  m_max_move_distance =
      std::min(m_max_move_sick, std::min(m_max_move_down, m_max_move_depth));
  if (m_max_move_distance <= m_collision_distance &&
          std::fabs(m_act_speed) > 0.1 && m_waypoints.size() > 1 ||
      m_max_move_distance <= m_collision_distance_near_target &&
          std::fabs(m_act_speed) > 0.1 && m_waypoints.size() == 1 ||
      m_max_move_distance <= 0.1) {
    handleCollision();
  }
}
void HomerNavigationNode::maxDepthMoveDistanceCallback(
    const std_msgs::Float32::ConstPtr& msg) {
  m_max_move_depth = msg->data;
  calcMaxMoveDist();
}

void HomerNavigationNode::laserDataCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  m_last_laser_time = ros::Time::now();
  m_max_move_sick = map_tools::get_max_move_distance(
      map_tools::laser_ranges_to_points(msg->ranges, msg->angle_min,
                                        msg->angle_increment, msg->range_min,
                                        msg->range_max, m_transform_listener,
                                        msg->header.frame_id, "/base_link"),
      m_min_x, m_min_y);
  calcMaxMoveDist();
}

void HomerNavigationNode::downlaserDataCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  m_max_move_down = map_tools::get_max_move_distance(
      map_tools::laser_ranges_to_points(msg->ranges, msg->angle_min,
                                        msg->angle_increment, msg->range_min,
                                        msg->range_max, m_transform_listener,
                                        msg->header.frame_id, "/base_link"),
      m_min_x, m_min_y);
  calcMaxMoveDist();
}

void HomerNavigationNode::startNavigationCallback(
    const homer_mapnav_msgs::StartNavigation::ConstPtr& msg) {
  m_avoided_collision = false;
  m_target_point = msg->goal.position;
  m_target_orientation = tf::getYaw(msg->goal.orientation);
  m_desired_distance =
      msg->distance_to_target < 0.1 ? 0.1 : msg->distance_to_target;
  m_skip_final_turn = msg->skip_final_turn;
  m_fast_path_planning = msg->fast_planning;
  m_new_target = true;
  m_target_name = "";
  m_initial_path_reaches_target = false;

  ROS_INFO_STREAM("Navigating to target "
                  << m_target_point.x << ", " << m_target_point.y
                  << "\nTarget orientation: " << m_target_orientation
                  << "Desired distance to target: " << m_desired_distance);

  m_MainMachine.setState(AWAITING_PATHPLANNING_MAP);
}

void HomerNavigationNode::moveBaseSimpleGoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (msg->header.frame_id != "map") {
    tf::StampedTransform transform;
    if (m_transform_listener.waitForTransform("map", msg->header.frame_id,
                                              msg->header.stamp,
                                              ros::Duration(1.0))) {
      try {
        m_transform_listener.lookupTransform("map", msg->header.frame_id,
                                             msg->header.stamp, transform);
        tf::Vector3 targetPos(msg->pose.position.x, msg->pose.position.y,
                              msg->pose.position.z);
        targetPos = transform * targetPos;
        m_target_point.x = targetPos.getX();
        m_target_point.y = targetPos.getY();
        m_target_orientation = tf::getYaw(
            transform *
            tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                           msg->pose.orientation.z, msg->pose.orientation.w));
      } catch (tf::TransformException ex) {
        ROS_ERROR_STREAM(ex.what());
        return;
      }
    } else {
      try {
        m_transform_listener.lookupTransform("map", msg->header.frame_id,
                                             ros::Time(0), transform);
        tf::Vector3 targetPos(msg->pose.position.x, msg->pose.position.y,
                              msg->pose.position.z);
        targetPos = transform * targetPos;
        m_target_point.x = targetPos.getX();
        m_target_point.y = targetPos.getY();
        m_target_orientation = tf::getYaw(
            transform *
            tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                           msg->pose.orientation.z, msg->pose.orientation.w));
      } catch (tf::TransformException ex) {
        ROS_ERROR_STREAM(ex.what());
        return;
      }
    }
  } else {
    m_target_point = msg->pose.position;
    m_target_orientation = tf::getYaw(msg->pose.orientation);
  }
  m_avoided_collision = false;
  m_desired_distance = 0.1;
  m_skip_final_turn = false;
  m_fast_path_planning = false;
  m_new_target = true;

  ROS_INFO_STREAM("Navigating to target via Move Base Simple x: "
                  << m_target_point.x << ", y: " << m_target_point.y
                  << "\nTarget orientation: " << m_target_orientation
                  << " Desired distance to target: " << m_desired_distance
                  << "\nframe_id: " << msg->header.frame_id);

  m_MainMachine.setState(AWAITING_PATHPLANNING_MAP);
}

void HomerNavigationNode::navigateToPOICallback(
    const homer_mapnav_msgs::NavigateToPOI::ConstPtr& msg) {
  homer_mapnav_msgs::GetPointsOfInterest srv;
  m_get_POIs_client.call(srv);
  std::vector<homer_mapnav_msgs::PointOfInterest>::iterator it;
  for (it = srv.response.poi_list.pois.begin();
       it != srv.response.poi_list.pois.end(); ++it) {
    if (it->name == msg->poi_name) {
      m_avoided_collision = false;
      m_target_point = it->pose.position;
      m_target_orientation = tf::getYaw(it->pose.orientation);
      m_desired_distance =
          msg->distance_to_target < 0.1 ? 0.1 : msg->distance_to_target;
      m_skip_final_turn = msg->skip_final_turn;
      m_fast_path_planning = false;
      m_new_target = true;
      m_target_name = msg->poi_name;

      ROS_INFO_STREAM("Navigating to target "
                      << m_target_point.x << ", " << m_target_point.y
                      << "\nTarget orientation: " << m_target_orientation
                      << "Desired distance to target: " << m_desired_distance);

      m_MainMachine.setState(AWAITING_PATHPLANNING_MAP);
      return;
    }
  }

  ROS_ERROR_STREAM("No point of interest with name '"
                   << msg->poi_name << "' found in current poi list");
  sendTargetUnreachableMsg(homer_mapnav_msgs::TargetUnreachable::UNKNOWN);
}

void HomerNavigationNode::stopNavigationCallback(
    const std_msgs::Empty::ConstPtr& msg) {
  ROS_INFO_STREAM("Stopping navigation.");
  m_MainMachine.setState(IDLE);
  m_avoided_collision = false;
  stopRobot();

  m_waypoints.clear();
  nav_msgs::Path empty_path_msg;
  empty_path_msg.poses = m_waypoints;
  m_path_pub.publish(empty_path_msg);
}

void HomerNavigationNode::unknownThresholdCallback(
    const std_msgs::Int8::ConstPtr& msg) {
  m_explorer->setUnknownThreshold(static_cast<int>(msg->data));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "homer_navigation");

  HomerNavigationNode node;

  ros::Rate rate(50);

  while (ros::ok()) {
    ros::spinOnce();
    node.idleProcess();
    rate.sleep();
  }

  return 0;
}

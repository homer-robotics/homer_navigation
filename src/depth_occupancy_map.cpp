#include <homer_navigation/depth_occupancy_map.h>
#include <map>
#include <math.h>

depth_occupancy_map::depth_occupancy_map(ros::NodeHandle* nh):
    MAX_DISTANCE(4.0)
{
    m_nh = nh;
    m_DoMappingSubscriber = nh->subscribe("/depth_mapping/do_mapping", 1, &depth_occupancy_map::doMappingCallback, this);
    m_MapSubscriber = nh->subscribe("/map", 1, &depth_occupancy_map::map_callback, this);
    m_ScanSubscriber = nh->subscribe("/scan", 1, &depth_occupancy_map::scan_callback, this);
    m_StartNavigationSubscriber = nh->subscribe("/homer_navigation/start_navigation", 3, &depth_occupancy_map::startNavigationCallback, this);
    m_MoveBaseSimpleGoalSubscriber = nh->subscribe("/move_base_simple/goal", 3, &depth_occupancy_map::moveBaseSimpleGoalCallback, this);
    m_StopNavigationSubscriber = nh->subscribe("/homer_navigation/stop_navigation", 3, &depth_occupancy_map::stopNavigationCallback, this);
    m_NavigateToPoiSubscriber = nh->subscribe("/homer_navigation/navigate_to_POI", 3, &depth_occupancy_map::navigateToPoiCallback, this);
    m_TargetReachedSubscriber = nh->subscribe("/homer_navigation/target_reached", 3, &depth_occupancy_map::targetReachedCallback, this);
    m_TargetUnreachableSubscriber = nh->subscribe("/homer_navigation/target_unreachable", 3, &depth_occupancy_map::targetUnreachableCallback, this);
    m_maxDepthMoveDistancePublisher = nh->advertise<std_msgs::Float32>("/homer_navigation/max_depth_move_distance", 1);
    m_got_map_size = false;

    m_depthMapPublisher = nh->advertise<nav_msgs::OccupancyGrid>("/projected_map", 1);

    m_timer = nh->createTimer(ros::Duration(0.5), &depth_occupancy_map::timerCallback, this);

    m_map.header.stamp = ros::Time::now();
    m_map.header.frame_id = "map";
    m_map.info.origin.position.x = 0.0;
    m_map.info.origin.position.y = 0.0;
    m_map.info.origin.position.z = 0.0;
    m_map.info.origin.orientation.w = 1.0;
    m_map.info.origin.orientation.x = 0.0;
    m_map.info.origin.orientation.y = 0.0;
    m_map.info.origin.orientation.z = 0.0;

    m_do_mapping = false;
    m_navigating = false;
    m_depthSubscribed = false;
    m_nh->getParam("/rgbd_node/points_topic", m_depth_topic);
    ROS_INFO_STREAM("[depth_occupancy_map] points topic: " << m_depth_topic);
    m_nh->param("/depth_occupancy_map/distance_offset", m_distance_offset, 0.15);
    ROS_INFO_STREAM("[depth_occupancy_map] distance offset: " << m_distance_offset);

    m_timer.start();
}

void depth_occupancy_map::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

    ROS_INFO_STREAM("[depth_occupancy_map] got /map");
    const float resolution = msg->info.resolution;
    const int width = MAX_DISTANCE * 2 / resolution;
    m_map.info.width = width;
    m_map.info.height = width;
    m_map.info.resolution = resolution;

    m_ByteSize = width * width;
    m_map.data.resize(m_ByteSize, -1);

    std::fill(m_map.data.begin(), m_map.data.end(), homer_mapnav_msgs::ModifyMap::NOT_MASKED);
    m_got_map_size = true;
    m_MapSubscriber.shutdown();

}

void depth_occupancy_map::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO_STREAM("[depth_occupancy_map] got /scan");
    m_laser_frame = msg->header.frame_id;
    m_ScanSubscriber.shutdown();
}

void depth_occupancy_map::timerCallback(const ros::TimerEvent&)
{
    if (m_do_mapping && !m_depthSubscribed && m_navigating)
    {
        ros::param::set("/homer_navigation/use_ptu", true);
        ROS_INFO_STREAM("[depth_occupancy_map] waiting for points");
        m_DepthSubscriber = m_nh->subscribe(m_depth_topic, 1, &depth_occupancy_map::depth_callback, this);
        m_depthSubscribed = true;
    }
}

void depth_occupancy_map::doMappingCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_STREAM("[depth_occupancy_map] Depth_Map do mapping is set to " << msg->data);
    m_do_mapping = msg->data;
}

void depth_occupancy_map::depth_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if( !m_got_map_size )
        return;
    if( !m_do_mapping || !m_navigating )
    {
        std::fill(m_map.data.begin(), m_map.data.end(), homer_mapnav_msgs::ModifyMap::NOT_MASKED);
        publishMap( msg->header.stamp );
        m_DepthSubscriber.shutdown();
        m_depthSubscribed = false;
        ros::param::set("/homer_navigation/use_ptu", false);
        return;
    }
    updateDepthData(msg);
}

void depth_occupancy_map::updateDepthData(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO_STREAM("[depth_occupancy_map] updateDepthData");
    geometry_msgs::Point robot;
    geometry_msgs::Point laser;
    robot.x = 0;
    robot.y = 0;
    laser.x = 0;
    laser.y = 0;

    // if(  m_tfListener.waitForTransform("/base_link","/map", srv.response.pointCloud.header.stamp,
    // ros::Duration(0.1)))
    if (m_tfListener.waitForTransform("/base_link", "/map", msg->header.stamp, ros::Duration(0.25)))
    {
        // robot = map_tools::transformPoint(robot, m_tfListener, srv.response.pointCloud.header.stamp, "/base_link",
        // "/map");
        robot = map_tools::transformPoint(robot, m_tfListener, msg->header.stamp, "/base_link", "/map");
        laser = map_tools::transformPoint(laser, m_tfListener, msg->header.stamp, m_laser_frame, "/map");
        m_map.info.origin.position.x = robot.x - MAX_DISTANCE;
        m_map.info.origin.position.y = robot.y - MAX_DISTANCE;

        // sensor_msgs::PointCloud2 msg2 = srv.response.pointCloud;
        tf::StampedTransform transform;
        pcl::PointCloud<pcl::PointXYZ> cloud_in;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud_in);
        // m_tfListener.waitForTransform("/map", msg.header.frame_id, srv.response.pointCloud.header.stamp,
        // ros::Duration(0.4));
        // m_tfListener.lookupTransform("/map", msg.header.frame_id, srv.response.pointCloud.header.stamp, transform);
        if (m_tfListener.waitForTransform("/map", msg->header.frame_id, msg->header.stamp, ros::Duration(0.1)))
        {
            m_tfListener.lookupTransform("/map", msg->header.frame_id, msg->header.stamp, transform);

            pcl_ros::transformPointCloud(cloud_in, cloud, transform);

            const float max_range_z = 1.6;
            const float min_range_z = 0.3;
            const float max_distance = MAX_DISTANCE * MAX_DISTANCE;
            const float min_distance = 0.2;
            geometry_msgs::Point point;
            std::map<std::pair<int,int>,std::vector<float>> points_2d;

            std::fill(m_map.data.begin(), m_map.data.end(), homer_mapnav_msgs::ModifyMap::NOT_MASKED);
            for (int i = 0; i < cloud.size(); ++i)
            {
                if( std::isnan(cloud.points[i].z) )
                    continue;
                if (cloud.points[i].z < max_range_z && cloud.points[i].z > min_range_z)
                {
                    point.x = cloud.points[i].x;
                    point.y = cloud.points[i].y;
                    const float distance = (point.x - robot.x) * (point.x - robot.x) + (point.y - robot.y) * (point.y - robot.y);
                    if (distance < max_distance && distance > min_distance)
                    {
                        auto sensor_map_point = map_tools::toMapCoords(point, m_map.info.origin, m_map.info.resolution);
                        points_2d[std::make_pair<int,int>((int)(sensor_map_point.x()), (int)(sensor_map_point.y()))].push_back(cloud.points[i].z);
                    }
                }
            }

            float min_obstacle_distance = 99;

            //for( std::map<std::pair<int,int>,std::vector<float>>::iterator pointIt = points_2d.begin(); pointIt != points_2d.end(); pointIt++ )
            for(const auto& kvpair : points_2d)
            {
                // int x = pointIt->first.first;
                int x = kvpair.first.first;
                double worldX = m_map.info.origin.position.x + (x - 0.5) * m_map.info.resolution;
                // int y = pointIt->first.second;
                int y = kvpair.first.second;
                double worldY = m_map.info.origin.position.y + (y - 0.5) * m_map.info.resolution;
                int j = x + m_map.info.width * y;
                // const std::vector<float>& heights = pointIt->second;
                const std::vector<float>& heights = kvpair.second;
                if(heights.size() > 4 && j > 0 && j < m_ByteSize)
                {
                    m_map.data[j] = homer_mapnav_msgs::ModifyMap::DEPTH;
                    const float distance = ((worldX - laser.x) * (worldX - laser.x)) +  ((worldY - laser.y) * (worldY - laser.y));

                    if( distance < min_obstacle_distance )
                        min_obstacle_distance = distance;
                }

                std::vector<float> neighbour_heights;
                const auto push_points_lambda = [&neighbour_heights, &points_2d](int x, int y)
                {
                    try
                    {
                        const auto& v = points_2d.at(std::make_pair(x, y));
                        neighbour_heights.push_back(std::accumulate(v.begin(), v.end(), 0.0) / (float) v.size());
                    }
                    catch( std::out_of_range ) {}
                };

                for(int tmpX = x -1; tmpX <= x + 1; tmpX++)
                {
                    for(int tmpY = y - 1; tmpY <= y + 1; tmpY++)
                    {
                        if(tmpX == 0 && tmpY == 0)
                            continue;
                        push_points_lambda(tmpX, tmpY);
                    }
                }

                if( neighbour_heights.size() < 3 )
                    continue;

                const float avg_height = std::accumulate(heights.begin(), heights.end(), 0.0) /
                    (float) heights.size();
                const float avg_neighbour_height = std::accumulate(neighbour_heights.begin(), neighbour_heights.end(), 0.0) /
                    (float) neighbour_heights.size();

                if(std::fabs(avg_neighbour_height - avg_height) < 0.1 && j > 0 && j < m_ByteSize)
                {
                    m_map.data[j] = homer_mapnav_msgs::ModifyMap::DEPTH;
                    const float distance = ((worldX - laser.x) * (worldX - laser.x)) +  ((worldY - laser.y) * (worldY - laser.y));

                    if( distance < min_obstacle_distance )
                        min_obstacle_distance = distance;
                }
            }


            min_obstacle_distance = std::sqrt( min_obstacle_distance );
            min_obstacle_distance -= m_distance_offset;
            if( min_obstacle_distance < 0 )
                min_obstacle_distance = 0.0;
            std_msgs::Float32 min_obstacle_dist_msg;
            min_obstacle_dist_msg.data = min_obstacle_distance;
            m_maxDepthMoveDistancePublisher.publish(min_obstacle_dist_msg);
            ROS_INFO_STREAM("[depth_occupancy_map] min_obstacle_distance: " << min_obstacle_distance);
            publishMap( msg->header.stamp );
        }
        else
        {
            ROS_INFO_STREAM("[depth_occupancy_map] Found no transfom");
        }  //                   publishMap();
    }
    else
    {
        ROS_INFO_STREAM("[depth_occupancy_map] stamp error");
    }
}

void depth_occupancy_map::publishMap( ros::Time stamp )
{
    m_map.header.stamp = stamp;
    m_depthMapPublisher.publish(m_map);
}

void depth_occupancy_map::startNavigationCallback(const homer_mapnav_msgs::StartNavigation::ConstPtr& msg)
{
    m_navigating = true;
    ROS_INFO_STREAM("[depth_occupancy_map] Depth_Map navigating is set to " << m_navigating);
}

void depth_occupancy_map::moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    m_navigating = true;
    ROS_INFO_STREAM("[depth_occupancy_map] Depth_Map navigating is set to " << m_navigating);
}

void depth_occupancy_map::stopNavigationCallback(const std_msgs::Empty::ConstPtr& msg)
{
    m_navigating = false;
    ROS_INFO_STREAM("[depth_occupancy_map] Depth_Map navigating is set to " << m_navigating);
}

void depth_occupancy_map::navigateToPoiCallback(const homer_mapnav_msgs::NavigateToPOI::ConstPtr& msg)
{
    m_navigating = true;
    ROS_INFO_STREAM("[depth_occupancy_map] Depth_Map navigating is set to " << m_navigating);
}

void depth_occupancy_map::targetReachedCallback(const std_msgs::String& msg)
{
    m_navigating = false;
    ROS_INFO_STREAM("[depth_occupancy_map] Depth_Map navigating is set to " << m_navigating);
}

void depth_occupancy_map::targetUnreachableCallback(const homer_mapnav_msgs::TargetUnreachable& msg)
{
    m_navigating = false;
    ROS_INFO_STREAM("[depth_occupancy_map] Depth_Map navigating is set to " << m_navigating);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "depth_occupancy_map");
    ros::NodeHandle nh;

    depth_occupancy_map node(&nh);

    ros::Rate loopRate(10);
    ROS_INFO_STREAM("[depth_occupancy_map] depth_occupancy_map node started");

    while (ros::ok())
    {
        loopRate.sleep();
        ros::spinOnce();
    }

    return 0;
}

// ROS
#include <ros/rate.h>
#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <sstream>

#include <vector>

#include <geometry_msgs/Point.h>
#include <homer_mapnav_msgs/ModifyMap.h>
#include <homer_mapnav_msgs/StartNavigation.h>
#include <homer_mapnav_msgs/NavigateToPOI.h>
#include <homer_mapnav_msgs/TargetUnreachable.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <homer_nav_libs/tools.h>
#include <homer_tools/coordinate_converstion.h>
#include <homer_tools/loadRosConfig.h>

class depth_occupancy_map
{
        public:
                depth_occupancy_map(ros::NodeHandle* nh);

        private:
                void depth_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
                void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
                void doMappingCallback(const std_msgs::Bool::ConstPtr& msg);
                void timerCallback(const ros::TimerEvent&);
                void startNavigationCallback(const homer_mapnav_msgs::StartNavigation::ConstPtr& msg);
                void moveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
                void stopNavigationCallback(const std_msgs::Empty::ConstPtr& msg);
                void navigateToPoiCallback(const homer_mapnav_msgs::NavigateToPOI::ConstPtr& msg);
                void targetReachedCallback(const std_msgs::String& msg);
                void targetUnreachableCallback(const homer_mapnav_msgs::TargetUnreachable& msg);
                void updateDepthData(const sensor_msgs::PointCloud2::ConstPtr& msg);
                void publishMap( ros::Time stamp );

                nav_msgs::OccupancyGrid m_map;
                int m_ByteSize;

                tf::TransformListener m_tfListener;

                ros::Subscriber m_DoMappingSubscriber;
                ros::Subscriber m_MapSubscriber;
                ros::Subscriber m_StartNavigationSubscriber;
                ros::Subscriber m_MoveBaseSimpleGoalSubscriber;
                ros::Subscriber m_StopNavigationSubscriber;
                ros::Subscriber m_NavigateToPoiSubscriber;
                ros::Subscriber m_TargetReachedSubscriber;
                ros::Subscriber m_TargetUnreachableSubscriber;

                ros::Subscriber m_DepthSubscriber;
                std::string m_depth_topic;
                bool m_depthSubscribed;

                ros::Publisher m_depthMapPublisher;
                ros::Publisher m_maxDeptMovePublisher;
		ros::Publisher m_refreshNavigationParamsPublisher;
		ros::Publisher m_maxDepthMoveDistancePublisher;

                ros::Timer m_timer;

                ros::NodeHandle* m_nh;

                bool m_do_mapping;
                bool m_navigating;
                bool m_got_map_size;
                const double MAX_DISTANCE;
                std::string m_laser_frame;
};

# homer_navigation

## Introduction 

This is the navigation package. Requirements are a robotic pose (`/pose`), a map (`/map`) and laser range measurements (`/scan`).

`roslaunch homer_navigation homer_navigation.launch`

## Parameter 


### homer_navigation
* `/homer_navigation/safe_path_weight:`               1.2  # factor weight for safer path in relation to shortest path
* `/homer_navigation/waypoint_sampling_threshold:`    1.5  # factor of how dense the path waypoints are sampled regarding the obstacle_distance of the last or next waypoint 
* `/homer_navigation/frontier_safeness_factor:`       1.4  # factor of min_allowed_obstacle_distance to an obstacle of a cell which is considered safe

### cost calculation parameters
* `/homer_navigation/allowed_obstacle_distance/min:`  0.3  # m robot must stay further away than this from obstacles
* `/homer_navigation/allowed_obstacle_distance/max:`  5.0  # m not used at the moment
* `/homer_navigation/safe_obstacle_distance/min:`     0.7  # m if possible robot should move further away than this from obstacles
* `/homer_navigation/safe_obstacle_distance/max:`     1.5  # m further away than this from obstacles doesn't give a lesser cost addition

### collision Avoidance parameters
* `/homer_navigation/collision_distance:`             0.3  # m distance to obstacle from robotFront in which the obstacle avoidance will be executed
* `/homer_navigation/collision_distance_near_target:` 0.2  # m distance to obstacle from robotFront where obstacle avoidance won't be executed when near the target
* `/homer_navigation/backward_collision_distance:`    0.5  # m distance behind robot in which the robot won't back up into while doing collision avoidance 
* `/homer_navigation/min_y:`                          0.27 # m half robot width for max_move_distance calculation 
* `/homer_navigation/min_x:`                          0.3  # m distance from base_link to robot front for max_move_distance calculation 

### check path on map update
* `/homer_navigation/check_path:`                     true # bool toggles if the calculated path will be checked for obstacles while navigating
* `/homer_navigation/check_path_max_distance:`        2    # m maximal distance from robot position in which the path is being checked for obstacles

### speed parameters
* `/homer_navigation/min_turn_angle:`                 0.15 # rad values lower than this angle will let the navigation assume reaching the designated position   
* `/homer_navigation/max_turn_speed:`                 0.6  # rad/s max turn velocity the navigation can send
* `/homer_navigation/min_turn_speed:`                 0.3  # rad/s min turn speed for Final Turn so the Robot doesn't stop turning
* `/homer_navigation/max_drive_angle:`                0.6  # rad threshold for driving and turning - if above that value only turn

* `/homer_navigation/max_move_speed:`                 0.4  # m/s   max move speed the navigation can send

### caution factors values near 0 mean high caution values greater values mean less caution
### if any factor equals 0 the robot can't follow paths !!
* `/homer_navigation/map_speed_factor:`               1.2  # factor for the max speed calculation of the obstacleDistancemap
* `/homer_navigation/waypoint_speed_factor:`          1.2  # factor for the max speed calculation with the distance to the next waypoint
* `/homer_navigation/obstacle_speed_factor:`          1.0  # factor for the max speed calculation with the last laser may movement distance

* `/homer_navigation/callback_error_duration:`        0.3  # s max duration between pose and laser callbacks before error handling is executed

* `/homer_navigation/use_ptu:` 				          false# bool toggles if the ptu is being used to look at the next Waypoint during navigation

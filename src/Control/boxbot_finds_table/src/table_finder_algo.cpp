#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boxbot_finds_table/FibonacciAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <vector>
#include <set>
#include <utility> // pair
#include <math.h> // Trigonometry
#include <table_finder_algo.h>

#include <clocale>
#include <iostream>

// NOTE: This algorithm only works if the table's width and depth are smaller than the robot's width

TableFinder::TableFinder()
{
  ros::NodeHandle nh_private("~"); // Param nodehandle
  ros::Rate r(10);

  ros::Duration(0.5).sleep();
  std::cout << "\nINITIALISING..." << std::flush;;
  ros::Duration(0.5).sleep();

  // Init
  init_parameters(nh);

  scan_sub = nh.subscribe(scan_topic, 10, &TableFinder::scan_callback, this);
  command_pub = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);


  for (int i = 0; i < 3; i++)
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce(); // Clear junk data from Gazebo
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }

  std::cout << "\r";
  ROS_INFO("Node initialised!");

  std::cout << "\n . . . [ \u3065\u25E1\uFE4F\u25E1|\u3065.  \u252C\u2500\u2500\u2500\u252C\n\n";
  std::cout << "-= BOXBOT LOVES THEIR TABLE =-\n\n";

  // Calculate buffer distance
  buffer_distance = sqrt(pow(boxbot_x, 2) + pow(boxbot_y, 2)) / 2 + boxbot_buffer;

  while (ros::ok())
  {
    if (!scan_vector.empty())
      {

        state_flag = -1;

/*
  == ALGORITHM PSEUDOCODE ==

  NB: Assumptions
  - Robot LiDAR is 360 with large range
  - The table is the only object in room

  NB: (If we're not just isolating a table, then we need to filter out non table stuff, but that's outside of the scope)

  == STATE MACHINE ==

  = init =
  Upkeep

  NB: Upkeep includes the following:
    If robot is under the table:
      Set goal_table_ to true (in Upkeep)
    Else:
      Set goal_table_ to false (in Upkeep)

  = I = while robot is too close to table and in_table_ is false:
  Upkeep
  Move away from table_centroid

  = II = while robot is too far from table and in_table_ is false:
  Upkeep
  Move toward table_centroid (For better angle resolution)

  = III = while robot is not aligned and in_table_ is false:
  Align (orient until two pairs of points have equal x or y values (within tolerance to some limit))

  = IV = while robot is not clear of table, not near the table_centroid.y and in_table_ is false:
  NB: (Clear if average, absolute x value of the table_centroid is less than boxbot_x + boxbot_buffer)

  Upkeep
  If average is positive:
    Move backward
  Else:
    Move forward

  = V = while robot is not near the table_centroid.y
  Upkeep
  Move sideways toward table_centroid.y

  = VI = while goal_table_ is false:
  Upkeep
  Move to table_centroid

  = VII = while robot is under table and aligned_table_fine_ is false:
  Upkeep

  If goal_table_ is true:
    Finely align
  Else:
    Move to table_centroid

  If robot is at goal:
    Set goal_table_ to true (in Upkeep)
  Else:
    Set goal_table_ to false (in Upkeep)

  = END = ▔|▔ [ ^ ﹏ ^ ]づ ▔|▔  ～☆♪

*/

        // Init
        upkeep();

        if (goal_table_)
        {
          std::cout <<  "\r     \u252C\u2500 [ \u25E1\uFE4F\u25E1 ] \u2500\u252C \uFF5E\u2606\u266A       " << std::flush;
        }
        else
        {
          std::cout << "\r . . . [ \u3065\u00D4 \u25A1 \u00D4|\u3065 ! \u252C\u2500\u2500\u2500\u252C   " << std::flush;
        }

        // I : CLEAR TABLE
        while(closest_centroid_distance <= buffer_distance * 1.5 && !in_table_ && ros::ok())
        {
          state_flag = 1;

          // If we're lagging, stop the robot
          if (ros::Time::now().toSec() - last_command_time > 0.400)
          {
            stop_base();
          }

          upkeep();

          twist.linear.x = -vel_direction.first * linear_vel;
          twist.linear.y = -vel_direction.second * linear_vel;
          twist.linear.z = 0;

          twist.angular.x = 0;
          twist.angular.y = 0;
          twist.angular.z = 0;

          assert_velocity(twist);
          command_pub.publish(twist);

          last_command_time = ros::Time::now().toSec();
        }

        // II : APPROACH
        while(closest_centroid_distance > buffer_distance * 2 & !in_table_ && ros::ok())
        {
          state_flag = 2;

          // If we're lagging, stop the robot
          if (ros::Time::now().toSec() - last_command_time > 0.400)
          {
            stop_base();
          }

          upkeep();

          twist.linear.x = vel_direction.first * linear_vel;
          twist.linear.y = vel_direction.second * linear_vel;
          twist.linear.z = 0;

          twist.angular.x = 0;
          twist.angular.y = 0;
          twist.angular.z = 0;

          assert_velocity(twist);
          command_pub.publish(twist);

          last_command_time = ros::Time::now().toSec();
        }

        // III : ALIGN
        while(!aligned_table_ && !in_table_ && ros::ok())
        {
          state_flag = 3;

          // If we're lagging, stop the robot
          if (ros::Time::now().toSec() - last_command_time > 0.400)
          {
            stop_base();
          }

          upkeep();

          twist.linear.x = 0;
          twist.linear.y = 0;
          twist.linear.z = 0;

          twist.angular.x = 0;
          twist.angular.y = 0;
          twist.angular.z = ang_vel;

          assert_velocity(twist);
          command_pub.publish(twist);

          last_command_time = ros::Time::now().toSec();
        }

        // IV : CLEAR TABLE SHORT
        while(fabs(table_centroid.first) < boxbot_x + boxbot_buffer
              && fabs(table_centroid.second) > table_alignment_tolerance
              && !in_table_ && ros::ok())
        {
          state_flag = 4;

          // If we're lagging, stop the robot
          if (ros::Time::now().toSec() - last_command_time > 0.400)
          {
            stop_base();
          }

          upkeep();

          if (table_centroid.first > 0)
          {
            twist.linear.x = -linear_vel;
          }
          else
          {
            twist.linear.x = linear_vel;
          }

          twist.linear.y = 0;
          twist.linear.z = 0;

          twist.angular.x = 0;
          twist.angular.y = 0;
          twist.angular.z = 0;

          assert_velocity(twist);
          command_pub.publish(twist);

          last_command_time = ros::Time::now().toSec();
        }

        // V : MOVE TO TABLE_CENTROID Y
        while(fabs(table_centroid.second) > table_alignment_tolerance && !in_table_ && ros::ok())
        {
          state_flag = 5;

          // If we're lagging, stop the robot
          if (ros::Time::now().toSec() - last_command_time > 0.400)
          {
            stop_base();
          }

          upkeep();

          twist.linear.x = 0;
          twist.linear.y = vel_direction.second * linear_vel;
          twist.linear.z = 0;

          twist.angular.x = 0;
          twist.angular.y = 0;
          twist.angular.z = 0;

          assert_velocity(twist);
          command_pub.publish(twist);

          last_command_time = ros::Time::now().toSec();
        }

        // VI : MOVE TO GOAL
        while(!goal_table_ && ros::ok())
        {
          state_flag = 6;

          // If we're lagging, stop the robot
          if (ros::Time::now().toSec() - last_command_time > 0.400)
          {
            stop_base();
          }

          upkeep();

          twist.linear.x = vel_direction.first * linear_vel;
          twist.linear.y = vel_direction.second * linear_vel;
          twist.linear.z = 0;

          twist.angular.x = 0;
          twist.angular.y = 0;
          twist.angular.z = 0;

          assert_velocity(twist);
          command_pub.publish(twist);

          last_command_time = ros::Time::now().toSec();
        }


        // VII : MOVE TO GOAL AND FINELY ALIGN
        while(in_table_ && !aligned_table_fine_ && ros::ok())
        {
          state_flag = 7;

          // If we're lagging, stop the robot
          if (ros::Time::now().toSec() - last_command_time > 0.400)
          {
            stop_base();
          }

          upkeep();

          if(!goal_table_)
          {

            twist.linear.x = vel_direction.first * linear_vel;
            twist.linear.y = vel_direction.second * linear_vel;
            twist.linear.z = 0;

            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;
          }
          else if (!aligned_table_fine_) // Finely align
          {
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;

            twist.angular.x = 0;
            twist.angular.y = 0;

            if (aligned_table_fine_error > 0)
            {
              twist.angular.z = -tanh(aligned_table_fine_error) * 0.5 - 0.025;
            }
            else
            {
              twist.angular.z = -tanh(aligned_table_fine_error) * 0.5 + 0.025;
            }
          }

          assert_velocity(twist);
          command_pub.publish(twist);

          last_command_time = ros::Time::now().toSec();
        }

        stop_base();
    }
  }
}

/////////////////////////
// ALGORITHM FUNCTIONS //
/////////////////////////

void TableFinder::upkeep()
{
  // Get data from ROS
  scan_angle_squashed = squash_scan(scan_vector, start_angle, scan_increment);
  scan_angle_clusters = filter_scan(scan_vector, start_angle, scan_increment);

  rect_scan = polar_to_rect(scan_angle_squashed);
  leg_centroids = find_leg_centroids(scan_angle_clusters);
  table_centroid = find_centroid(leg_centroids);
  table_centroid_distance = sqrt(pow(table_centroid.first, 2) + pow(table_centroid.second, 2));
  closest_centroid_distance = closest_centroid_dist(leg_centroids);

  // Publish TFs
  publish_leg_transforms(leg_centroids);
  publish_table_transform(table_centroid);

  // Update variables
  if (use_p_control)
  {
    linear_vel = max_linear_vel / 8 + 3 * max_linear_vel / 4 * table_centroid_distance * kp;
  }

  ang_vel = max_ang_vel;

  vel_direction = get_direction_unit_vector(table_centroid);
  assert_vel_direction(vel_direction); // Just in case

  // Update flags
  in_table_ = in_table(leg_centroids);
  goal_table_ = reached_goal(table_centroid);
  aligned_table_ = aligned_table(leg_centroids);
  aligned_table_fine_error = aligned_table_fine(leg_centroids);
  aligned_table_fine_ = fabs(aligned_table_fine_error) < table_alignment_tolerance / 5;

  ros::spinOnce();

  ros::Rate r(10);
  r.sleep();
}


std::vector<f_tuple> TableFinder::squash_scan(const std::vector<float> &scan_vector,
                                      double start_angle,
                                      double scan_increment)
{
  std::vector<f_tuple> squash_output;
  //squash_output.clear();

  int squash_factor = scan_increment / 0.0087265;

  for (int i = 0; i < scan_vector.size(); i += 2)
  {
    squash_output.push_back(std::make_pair (scan_vector.at(i),
                                     start_angle + scan_increment * i));
  }

  return squash_output;
}

// Returns a vector of isolated 'clusters' of points
std::vector<std::vector<f_tuple > > TableFinder::filter_scan(const std::vector<float> &scan_vector,
                                                  double start_angle,
                                                  double scan_increment)
{
  std::vector<std::vector<f_tuple > > scan_angle_filtered;
  std::vector<f_tuple> sub_vector;
  bool first_flag(false);
  bool last_flag(false);

  for (int i = 0; i < scan_vector.size(); i++)
  {
    if (isinf(scan_vector.at(i)))
    {
      if (sub_vector.size() > 1)
      {
        scan_angle_filtered.push_back(sub_vector);
        sub_vector.clear();
      }
    }
    else
    {
      sub_vector.push_back(std::make_pair (scan_vector.at(i),
                                           start_angle + scan_increment * i));
      if (i == 0)
      {
        first_flag = true;
      }

      if (i == scan_vector.size() - 1)
      {
        last_flag = true;
      }
    }
  }

  if (sub_vector.size() > 1)
  {
    scan_angle_filtered.push_back(sub_vector);
  }

  while (scan_angle_filtered.size() > 4)
  {
    scan_angle_filtered.pop_back();
  }

  return scan_angle_filtered;
}

std::vector<d_tuple> TableFinder::polar_to_rect(const std::vector<f_tuple> &scan_angle_vector)
{
  std::vector<d_tuple> rect_scan;

  for (int i = 0; i < scan_angle_vector.size(); i++)
  {
    double range(scan_angle_vector.at(i).first);
    double angle(scan_angle_vector.at(i).second);

    rect_scan.push_back(std::make_pair (range * cos(angle),
                                        range * sin(angle)));
  }

  return rect_scan;
}

d_tuple TableFinder::find_centroid(const std::vector<d_tuple> &rect_scan)
{
  double x_sum;
  double y_sum;
  int skip_counter(0);

  for (int i = 0; i < rect_scan.size(); i++)
  {

    if (!isinf(rect_scan.at(i).first) && !isinf(rect_scan.at(i).second))
    {
      x_sum += rect_scan.at(i).first;
      y_sum += rect_scan.at(i).second;
    }
    else
    {
      skip_counter += 1;
    }
  }

  int points_counted = rect_scan.size() - skip_counter;

  return std::make_pair (x_sum / points_counted, y_sum / points_counted);
}

std::vector<d_tuple> TableFinder::find_leg_centroids(const std::vector<std::vector<f_tuple > > &leg_scan_angle_clusters)
{
  std::vector<d_tuple> leg_output;
  //leg_output.clear();

  for (int i = 0; i < leg_scan_angle_clusters.size(); i++)
  {
    std::vector<d_tuple> leg_points(polar_to_rect(leg_scan_angle_clusters.at(i)));
    leg_output.push_back(find_centroid(leg_points));
  }

  return leg_output;
}

int TableFinder::angle_quadrant(const d_tuple point)
{
  //      1
  //    2 | 1
  // 2 ---+--- 4
  //    3 | 4
  //      3

  double pi(3.14159265359);
  double angle = atan2(point.first, point.second);

  if( angle > 0 && angle <= 0.5 * pi)
  {
    return 1;
  }
  else if( angle > 0.5 * pi && angle <= pi)
  {
    return 2;
  }
  else if(angle > -2 * pi && angle <= -0.5 * pi)
  {
    return 3;
  }
  else if (angle > -0.5 * pi && angle < 0)
  {
    return 4;
  }
}

bool TableFinder::in_table(const std::vector<d_tuple> &leg_centroids)
{
  std::set<int> quadrant_set;

  for (int i = 0; i < leg_centroids.size(); i++)
  {
    quadrant_set.insert(angle_quadrant(leg_centroids.at(i)));
  }

  if (quadrant_set.size() == 4) return true; else return false;
}

bool TableFinder::aligned_table(const std::vector<d_tuple> &leg_centroids)
{
  bool align_flag_x(false);
  bool align_flag_y(false);

  d_tuple comparison_point = leg_centroids.at(0);
  d_tuple current_point;
  d_tuple closest_point;

  double distance;
  double closest_point_dist(999999.9);

  for (int i = 1; i < leg_centroids.size(); i++)
  {
    current_point = leg_centroids.at(i);
    distance = sqrt(pow(comparison_point.first - current_point.first, 2)
                    + pow(comparison_point.second - current_point.second, 2));

    if (distance < closest_point_dist)
    {
      closest_point = current_point;
      closest_point_dist = distance;
    }

    if (fabs(comparison_point.first - current_point.first) <= table_alignment_tolerance)
    {
      align_flag_y = true;
    }

    if (fabs(comparison_point.second - current_point.second) <= table_alignment_tolerance)
    {
      align_flag_x = true;
    }
  }

  if (enter_on_long_end_only)
  {
    // If the closest point is aligned horizontally, the robot will try to enter on the short end
    if (fabs(comparison_point.first - closest_point.first) <= table_alignment_tolerance)
    {
      return false;
    }
}

  if ((align_flag_x == true) && (align_flag_y == true)) return true; else return false;
}

double TableFinder::aligned_table_fine(const std::vector<d_tuple> &leg_centroids)
{
  bool align_flag_x(false);
  bool align_flag_y(false);

  d_tuple comparison_point = leg_centroids.at(0);
  d_tuple current_point;
  d_tuple closest_point;

  double distance;
  double closest_point_dist(999999.9);

  // Find closest point
  //
  // x        x
  // |   or   |
  // x        x
  //
  //

  for (int i = 1; i < leg_centroids.size(); i++)
  {
    current_point = leg_centroids.at(i);
    distance = sqrt(pow(comparison_point.first - current_point.first, 2)
                    + pow(comparison_point.second - current_point.second, 2));

    if (distance < closest_point_dist)
    {
      closest_point = current_point;
      closest_point_dist = distance;
    }
  }

  // 1    3
  //
  //
  // 2    4
  //

  double difference(comparison_point.second - closest_point.second);

  if (comparison_point.first < 0) // If comparison is on the left
  {
    if (comparison_point.second > 0) // 1
    {
      return -difference;
    }
    else // 2
    {
      return difference;
    }
  }
  else // Otherwise if it's on the right
  {
    if (comparison_point.second > 0) // 3
    {
      return -difference;
    }
    else // 4
    {
      return difference;
    }
  }


}

d_tuple TableFinder::get_direction_unit_vector(const d_tuple centroid)
{
  double vector_magnitude = sqrt(pow(centroid.first, 2) + pow(centroid.second, 2));

  return std::make_pair (centroid.first / vector_magnitude, centroid.second / vector_magnitude);
}

bool TableFinder::reached_goal(const d_tuple table_centroid)
{
  double displacement(sqrt(pow(table_centroid.first, 2) + pow(table_centroid.second, 2)));

  if (displacement < table_goal_tolerance) return true; else return false;
}

double TableFinder::closest_centroid_dist(const std::vector<d_tuple> &leg_centroids)
{
  double smallest_dist(999999);
  double distance;

  for (int i = 0; i < leg_centroids.size(); i++)
  {
    distance = sqrt(pow(leg_centroids.at(i).first, 2)
                    + pow(leg_centroids.at(i).second, 2));

    if (distance < smallest_dist)
    {
      smallest_dist = distance;
    }
  }

  return distance;
}

/////////////////////////
// ASSERTION FUNCTIONS //
/////////////////////////

void TableFinder::assert_vel_direction(const d_tuple vel_direction)
{
  assert(("ERROR: nan target x direction. Restart gazebo and try again!", !isnan(vel_direction.first)));
  assert(("ERROR: nan target y direction. Restart gazebo and try again!", !isnan(vel_direction.second)));

  assert(("ERROR: inf target x direction. Restart gazebo and try again!", !isinf(vel_direction.first)));
  assert(("ERROR: inf target y direction. Restart gazebo and try again!", !isinf(vel_direction.second)));

  assert(("ERROR: invalid target x direction. Restart gazebo and try again!", fabs(vel_direction.first) < 1.1));
  assert(("ERROR: invalid target y direction. Restart gazebo and try again!", fabs(vel_direction.second) < 1.1));
}

void TableFinder::assert_velocity(const geometry_msgs::Twist twist)
{
  assert(("ERROR: nan target x velocity. Restart gazebo and try again!", !isnan(twist.linear.x)));
  assert(("ERROR: nan target y velocity. Restart gazebo and try again!", !isnan(twist.linear.y)));
  assert(("ERROR: nan target angular velocity. Restart gazebo and try again!", !isnan(twist.angular.z)));

  assert(("ERROR: inf target x velocity. Restart gazebo and try again!", !isinf(twist.linear.x)));
  assert(("ERROR: inf target y velocity. Restart gazebo and try again!", !isinf(twist.linear.y)));
  assert(("ERROR: inf target angular velocity. Restart gazebo and try again!", !isinf(twist.angular.z)));

  assert(("ERROR: invalid target x velocity. Restart gazebo and try again!", fabs(twist.linear.x) < 50));
  assert(("ERROR: invalid target y velocity. Restart gazebo and try again!", fabs(twist.linear.y) < 50));
  assert(("ERROR: invalid target angular velocity. Restart gazebo and try again!", fabs(twist.angular.z) < 50));
}

///////////////////
// ROS FUNCTIONS //
///////////////////

void TableFinder::init_parameters(ros::NodeHandle &nh)
{
  nh.param("autonomous_mode", autonomous_mode, false);

  nh.param("boxbot_x_size", boxbot_x, 3.0);
  nh.param("boxbot_y_size", boxbot_y, 2.0);
  nh.param("boxbot_buffer_size", boxbot_buffer, 0.5);

  nh.param("table_long_length", table_long, 3.0);
  nh.param("table_short_length", table_short, 2.5);

  nh.param("table_goal_tolerance", table_goal_tolerance, 0.01);
  nh.param("table_alignment_tolerance", table_alignment_tolerance, 0.2);

  nh.param("max_linear_vel", max_linear_vel, 0.5);
  nh.param("max_angular_vel", max_ang_vel, 0.3);
  nh.param("enter_on_long_end_only", enter_on_long_end_only, true);
  nh.param("use_p_control", use_p_control, true);
  nh.param("k_p", kp, 1.0);

  nh.param("min_range", min_range, 0.1);
  nh.param("max_range", max_range, 10.0);

  nh.param<std::string>("command_topic", cmd_topic, "cmd_vel");
  nh.param<std::string>("scan_topic", scan_topic, "scan");
}

void TableFinder::scan_callback(const sensor_msgs::LaserScan& scan_msg)
{
  start_angle = scan_msg.angle_min;
  scan_increment = scan_msg.angle_increment;

  scan_vector = scan_msg.ranges; // This works!
}

void TableFinder::publish_leg_transforms(const std::vector<d_tuple> &leg_centroids)
{
  for (int i = 0; i < leg_centroids.size(); i++)
  {
    geometry_msgs::TransformStamped leg_tf;

    leg_tf.header.frame_id = "base_link";

    char buffer[10];
    sprintf(buffer, "leg_%i", i + 1);

    leg_tf.child_frame_id = buffer;

    geometry_msgs::Quaternion leg_quat(tf::createQuaternionMsgFromYaw(0.0));

    leg_tf.transform.translation.x = leg_centroids.at(i).first;
    leg_tf.transform.translation.y = leg_centroids.at(i).second;
    leg_tf.transform.translation.z = 0.0;

    leg_tf.transform.rotation = leg_quat;
    leg_tf.header.stamp = ros::Time::now();
    table_tf_broadcaster.sendTransform(leg_tf);
  }
}

void TableFinder::publish_table_transform(const d_tuple table_centroid)
{
  geometry_msgs::TransformStamped table_tf;

  table_tf.header.frame_id = "base_link";
  table_tf.child_frame_id = "table_centroid";

  geometry_msgs::Quaternion table_quat(tf::createQuaternionMsgFromYaw(0.0));

  table_tf.transform.translation.x = table_centroid.first;
  table_tf.transform.translation.y = table_centroid.second;
  table_tf.transform.translation.z = 0.0;

  table_tf.transform.rotation = table_quat;
  table_tf.header.stamp = ros::Time::now();
  table_tf_broadcaster.sendTransform(table_tf);
}

void TableFinder::stop_base()
{
  geometry_msgs::Twist twist;

  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;

  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;

  command_pub.publish(twist);
}

/*
class FibonacciAction
{
protected:

  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<simple_action_example::FibonacciAction> as_;
  std::string action_name_;

  // create messages that are used to published feedback/result
  simple_action_example::FibonacciFeedback feedback_;
  simple_action_example::FibonacciResult result_;

public:

  // Our constructor (with a cool initialisation list!)
  // https://stackoverflow.com/questions/2785612/c-what-does-the-colon-after-a-constructor-mean
  FibonacciAction(std::string name) :
    // Bind the callback to the action server. False is for thread spinning
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    // Start the action server
    as_.start();
  }

  // Destructor
  ~FibonacciAction(void)
  {
  }

  // Execute action callback (passing the goal via reference)
  void executeCB(const simple_action_example::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action (i <= goal->order, as goal is a pointer)
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }

      // Add the number to the feedback to be fed back
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  // Create an action server object and spin ROS
  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}

*/

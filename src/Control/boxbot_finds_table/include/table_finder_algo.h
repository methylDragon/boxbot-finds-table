#ifndef TABLE_FINDER_ALGO_H
#define TABLE_FINDER_ALGO_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boxbot_finds_table/FibonacciAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <vector>
#include <set>
#include <utility> // Pair
#include <math.h> // Trigonometry

typedef std::pair<float, float> f_tuple;
typedef std::pair<double, double> d_tuple;

class TableFinder
{
private:
  ////////////////
  // Parameters //
  ////////////////

  bool autonomous_mode;

  double boxbot_x, boxbot_y, boxbot_buffer;
  double table_short, table_long;
  double table_goal_tolerance, table_alignment_tolerance;

  double max_linear_vel, max_ang_vel;
  bool enter_on_long_end_only;
  bool use_p_control;
  double kp;

  double min_range, max_range;

  std::string cmd_topic, scan_topic;

  ///////////////
  // Variables //
  ///////////////

  int state_flag;

  bool in_table_;
  bool goal_table_;
  bool aligned_table_;
  bool aligned_table_fine_;

  double buffer_distance;

  double aligned_table_fine_error;

  std::vector<float> scan_vector; // Laser scan ranges
  double start_angle;
  double scan_increment;

  // The pair is range, angle. I.E. (SCAN, ANGLE)
  std::vector<f_tuple> scan_angle_squashed; // Squashed laser scan ranges with angles
  std::vector<d_tuple> rect_scan; // Laser scan points in rectangular coordinates

  std::vector<std::vector<f_tuple > > scan_angle_clusters; // Scan hits only

  d_tuple table_centroid;
  std::vector<d_tuple> leg_centroids; // List of table leg centroids

  double table_centroid_distance;
  double closest_centroid_distance;

  geometry_msgs::Twist twist;
  d_tuple vel_direction;
  double linear_vel, ang_vel;
  double last_command_time;

  /////////////////////////
  // ALGORITHM FUNCTIONS //
  /////////////////////////

  void upkeep();

  std::vector<f_tuple> squash_scan(const std::vector<float> &scan_vector,
                           double start_angle,
                           double scan_increment);

  std::vector<std::vector<f_tuple > > filter_scan(const std::vector<float> &scan_vector,
                                        double start_angle,
                                        double scan_increment);

  std::vector<d_tuple> polar_to_rect(const std::vector<f_tuple> &scan_angle_vector);

  d_tuple find_centroid(const std::vector<d_tuple> &rect_scan);
  std::vector<d_tuple> find_leg_centroids(const std::vector<std::vector<f_tuple > > &leg_scan_angle_clusters);

  d_tuple get_direction_unit_vector(const d_tuple centroid);

  int angle_quadrant(d_tuple point);
  bool in_table(const std::vector<d_tuple> &leg_centroids);
  bool aligned_table(const std::vector<d_tuple> &leg_centroids);
  double aligned_table_fine(const std::vector<d_tuple> &leg_centroids);
  bool reached_goal(const d_tuple table_centroid);
  double closest_centroid_dist(const std::vector<d_tuple> &leg_centroids);

  /////////////////////////
  // ASSERTION FUNCTIONS //
  /////////////////////////

  void assert_vel_direction(const d_tuple vel_direction);
  void assert_velocity(const geometry_msgs::Twist twist);

  ///////////////////
  // ROS FUNCTIONS //
  ///////////////////

  void init_parameters(ros::NodeHandle &nh);
  void scan_callback(const sensor_msgs::LaserScan& scan_msg);

  void publish_leg_transforms(const std::vector<d_tuple> &leg_centroids);
  void publish_table_transform(const d_tuple table_centroid);

  void stop_base();


protected:
  ros::NodeHandle nh; // Pubsub nodehandle
  ros::Subscriber scan_sub;
  ros::Publisher command_pub;

  tf::TransformBroadcaster table_tf_broadcaster;

public:
  TableFinder();
};

#endif

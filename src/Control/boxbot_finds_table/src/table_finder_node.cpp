#include <ros/ros.h>
#include <table_finder_algo.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "table_finder");

  // Create an action server object
  TableFinder table_finder;

  return 0;
}

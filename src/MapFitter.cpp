// Include the ROS C++ APIs
#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map/GridMapRosConverter.hpp"
#include "grid_map_msgs/GridMap.h"
#include <Eigen/Core>

using namespace std;
using namespace grid_map;

void GridMap_callback(const grid_map_msgs::GridMap message)
{
  ROS_INFO("Recieved my first grid_map.");
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(message, map);
  grid_map::Matrix& data = map["elevation"];
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      cout << "The value at index " << index.transpose() << " is " << data(index(0), index(1)) << endl;
  }
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "cloud_fitter");

  ROS_INFO("Hello, world I'm ready to match some grid_maps.");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<grid_map_msgs::GridMap>("elevation_mapping_long_range/elevation_map", 1, GridMap_callback);

  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();

  return 0; 
}

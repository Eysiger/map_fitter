// Include the ROS C++ APIs
#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Core>

using namespace std;
using namespace grid_map;

float Correlation(GridMap shifted_map, GridMap reference_map) {
  float correlation = 1;
  //TO_DO
  return correlation;
}

GridMap Shift(int x, int y, float theta) {
  GridMap shifted_map;

  return shifted_map;
}

void ExhaustiveSearch(GridMap map, GridMap reference_map) {//should later return shift param
  float x_length = reference_map.getLength().x();
  float y_length = reference_map.getLength().y();
  grid_map::Length length = reference_map.getLength();
  int x_size = reference_map.getSize()(0);
  int y_size = reference_map.getSize()(1);
  ROS_INFO("reference_map has x_length %f and y_length %f resp. %i x %i tiles", x_length, y_length, x_size, y_size);
  grid_map::Position position = reference_map.getPosition();
  float resolution = reference_map.getResolution();

  float spacing = 5;
  float angle = 5;

  float correlation[int(x_size/spacing)][int(y_size/spacing)][int(360/angle)];

  for (int x = spacing/2; x < x_size; x+=spacing)
  {
	for (int y = spacing/2; y < y_size; y+=spacing)
	{
		for (float theta = 0; theta < 360; theta+=angle)
		{
			//ROS_INFO("parameters: (%i, %i, %f)", x, y, theta);
			correlation[int(x/spacing)][int(y/spacing)][int(theta/angle)] = Correlation(Shift(x, y, theta), reference_map);
		}  
	}
  }
  ROS_INFO("done");
  
}

void GridMap_callback(const grid_map_msgs::GridMap message) {
  ROS_INFO("Recieved my first grid_map.");
  GridMap map;
  GridMapRosConverter::fromMessage(message, map);
  grid_map::Matrix& data = map["elevation"];
  /*for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      cout << "The value at index " << index.transpose() << " is " << data(index(0), index(1)) << endl;
  }*/
  
  GridMap reference_map;
  GridMapRosConverter::loadFromBag("/home/parallels/rosbags/reference_map_last.bag", "/uav_elevation_mapping/uav_elevation_map", reference_map);
  

  //ExhaustiveSearch(map, reference_map);   //should later return shift param
  grid_map::Vector shift;
  shift(0) = 0.05;
  shift(1) = 0.05;
  grid_map::Position position = map.getPosition();
  map.setPosition(position + shift);

  float correlation = Correlation(map, reference_map);
  ROS_INFO("Correlation for shift (%f,%f) is: %f", shift(0), shift(1), correlation);
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

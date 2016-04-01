#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator5.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


using namespace std;
using namespace grid_map;

ros::Publisher shifted_pub;
ros::Publisher correlation_pub;

void tfBroadcast(tf::TransformBroadcaster broadcaster) {
  broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
					ros::Time::now(),"/world", "/map"));
}

float Correlation(GridMap shifted_map, GridMap reference_map, grid_map::Position position, float theta) {
  int every = 5;
  float req_overlap = 0.5;

  float correlation = 0;
  float score = 0;
  int points = 0;
  int matches = 0;

  grid_map::Matrix& data = shifted_map["elevation"];
  for (grid_map::GridMapIterator5 iterator(shifted_map, every); !iterator.isPastEnd(); ++iterator) {
      	const Index index(*iterator);
	float shifted = data(index(0), index(1));
	if (shifted == shifted) {
		points += 1;
		grid_map::Position xy_position;
		shifted_map.getPosition(index, xy_position);

		tf::Transform transform = tf::Transform(tf::Quaternion(0, 0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), tf::Vector3(position(0), position(1), 0.0));
		tf::Vector3 xy_vector = tf::Vector3(xy_position(0), xy_position(1), 0.0);
		tf::Vector3 map_vector = transform*(xy_vector);
		grid_map::Position map_position;
		map_position(0) = map_vector.getX();
		map_position(1) = map_vector.getY();
	
        	if (reference_map.isInside(map_position)) {
			float reference = reference_map.atPosition("elevation", map_position);
			if (reference == reference) {//if nan f!= f 
				matches += 1;
				float xy_score = abs(shifted - reference);
				score += xy_score;
                		//cout << "Elevation at (" << index(0) << "," << index(1) <<"): " << shifted << ", reference at (" << xy_position(0) << "," << xy_position(1) << "): " << reference << ", diff.: " << xy_score << endl; 
			}
		}
	}
  }

  if (matches > points*req_overlap) {
	cout << matches <<" matching points. ";
	return score/matches;
  }
  else {
	cout << "Not enough matching points: " << matches <<" matches. ";
	return 1.0;
  }
}

GridMap Shift(grid_map::Position position, float theta, GridMap shifted_map,   tf::TransformBroadcaster broadcaster) {
  grid_map::Position zero_position;
  zero_position(0) = 0;
  zero_position(1) = 0;
  shifted_map.setPosition(zero_position);

  broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), 
		tf::Vector3(position(0), position(1), 0.0)), ros::Time::now(), "/map", "/map_rotated"));
  shifted_map.setFrameId("map_rotated");

  grid_map_msgs::GridMap shifted_msg;
  GridMapRosConverter::toMessage(shifted_map, shifted_msg);
  shifted_msg.info.header.stamp = ros::Time::now();
  shifted_pub.publish(shifted_msg);

  return shifted_map;
}

void ExhaustiveSearch(GridMap map, GridMap reference_map, tf::TransformBroadcaster broadcaster) {//should later return shift param
  int angle = 10;
  int every = 5;
  float correlation[reference_map.getSize()(0)][reference_map.getSize()(1)][360/angle];

  GridMap correlation_map({"correlation","rotation"});
  correlation_map.setGeometry(reference_map.getLength(), reference_map.getResolution()*every,
                              reference_map.getPosition());
  correlation_map.setFrameId("map");

  for (grid_map::GridMapIterator5 iterator(reference_map, every); !iterator.isPastEnd(); ++iterator) {
      	const Index index(*iterator);
	grid_map::Position xy_position;
	reference_map.getPosition(index, xy_position);
	for (int theta = 0; theta < 360; theta+=angle)
	{
		correlation[index(0)][index(1)][theta/angle] = Correlation(Shift(xy_position, theta, map, broadcaster), reference_map, xy_position, theta);
  		cout << "Correlation for " << xy_position.transpose() <<", " << theta <<" :" << correlation[index(0)][index(1)][theta/angle] << endl;
		
		if (correlation_map.isInside(xy_position)) {
			Index correlation_index;
			correlation_map.getIndex(xy_position, correlation_index);
			if ((correlation_map.isValid(correlation_index, "correlation") == false || correlation[index(0)][index(1)][theta/angle] < correlation_map.at("correlation", correlation_index)) && correlation[index(0)][index(1)][theta/angle] != 1) {
				correlation_map.at("correlation", correlation_index) = correlation[index(0)][index(1)][theta/angle]*10;
				correlation_map.at("rotation", correlation_index) = theta;
			}
		}
	}
	grid_map_msgs::GridMap correlation_msg;
  	GridMapRosConverter::toMessage(correlation_map, correlation_msg);
  	correlation_pub.publish(correlation_msg);
  }

  ROS_INFO("done");
}

void GridMap_callback(const grid_map_msgs::GridMap::ConstPtr& msg, tf::TransformBroadcaster broadcaster) {
  ROS_INFO("Recieved my first grid_map.");
  GridMap map;
  GridMapRosConverter::fromMessage(*msg, map);
  
  GridMap reference_map;
  GridMapRosConverter::loadFromBag("/home/parallels/rosbags/reference_map_last.bag", "/uav_elevation_mapping/uav_elevation_map", reference_map);

  ExhaustiveSearch(map, reference_map, broadcaster);   //should later return shift param
  /*grid_map::Vector shift;
  shift(0) = 1.0;
  shift(1) = 1.0;
  grid_map::Position position = map.getPosition();
  map.setPosition(position + shift);

  // advertise shifted_map to be displayed in rviz
  grid_map_msgs::GridMap shifted_msg;
  GridMapRosConverter::toMessage(map, shifted_msg);
  pub.publish(shifted_msg);
  
  float correlation = Correlation(map, reference_map);
  ROS_INFO("Correlation for shift (%f,%f) is: %f", shift(0), shift(1), correlation);*/
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "cloud_fitter");

  ROS_INFO("Hello, world I'm ready to match some grid_maps.");

  ros::NodeHandle nh;

  tf::TransformBroadcaster broadcaster;
  ros::Timer timer = nh.createTimer(ros::Duration(0.005), boost::bind(tfBroadcast, broadcaster), false, false);
  timer.start();

  shifted_pub = nh.advertise<grid_map_msgs::GridMap>("elevation_mapping_long_range/shifted_map",1);
  correlation_pub = nh.advertise<grid_map_msgs::GridMap>("correlation_best_rotation/correlation_map",1);

  ros::Subscriber sub = nh.subscribe<grid_map_msgs::GridMap>("elevation_mapping_long_range/elevation_map", 1, boost::bind(GridMap_callback, _1, broadcaster));

  ros::spin();

  return 0; 
}

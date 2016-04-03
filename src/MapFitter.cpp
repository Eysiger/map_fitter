#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIteratorSparse.hpp>
#include <grid_map_core/iterators/SubmapIteratorSparse.hpp>
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

float CorrelationSAD(GridMap shifted_map, GridMap reference_map, grid_map::Position position, float theta) {	// Sum of Absolute Differences divided by matching points
  int every = 5;		// VAR sparsity of match
  float req_overlap = 0.75;	// VAR required matches of all used template points

  float correlation = 0;	// initialization
  float score = 0;
  int points = 0;
  int matches = 0;

  // iterate sparsely through template points
  grid_map::Matrix& data = shifted_map["elevation"];
  for (grid_map::GridMapIteratorSparse iterator(shifted_map, every); !iterator.isPastEnd(); ++iterator) {
      	const Index index(*iterator);
	float shifted = data(index(0), index(1));
	if (shifted == shifted) { 	// check if point is defined, if nan f!= f
		points += 1;	  	// increase number of valid points
		grid_map::Position xy_position;
		shifted_map.getPosition(index, xy_position);	// get coordinates
		tf::Vector3 xy_vector = tf::Vector3(xy_position(0), xy_position(1), 0.0);

		// transform coordinates from /map_rotated to /map
		tf::Transform transform = tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), tf::Vector3(position(0), position(1), 0.0));
		tf::Vector3 map_vector = transform*(xy_vector);	// apply transformation
		grid_map::Position map_position;
		map_position(0) = map_vector.getX();
		map_position(1) = map_vector.getY();
		
		// check if point is within reference_map
        	if (reference_map.isInside(map_position)) {
			float reference = reference_map.atPosition("elevation", map_position);
			if (reference == reference) {   // check if point is defined, if nan f!= f 
				matches += 1;		// increase number of matched points
				float xy_score = abs(shifted - reference);
				score += xy_score;	// sum up absolute differneces 
			}
		}
	}
  }
  // check if required overlap is fulfilled
  if (matches > points*req_overlap) { return score/matches; }
  else { return 1.0; }
}

GridMap Shift(grid_map::Position position, float theta, GridMap shifted_map,   tf::TransformBroadcaster broadcaster) {		// returns shifted_map at origin
  grid_map::Position zero_position;
  zero_position(0) = 0.0;
  zero_position(1) = 0.0;
  shifted_map.setPosition(zero_position);	// set position to origin

  // broadcast transformation from /map to /map_rotated
  broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), 
		tf::Vector3(position(0), position(1), 0.0)), ros::Time::now(), "/map", "/map_rotated"));
  shifted_map.setFrameId("map_rotated");	// set frame to /map_rotated

  grid_map_msgs::GridMap shifted_msg;
  GridMapRosConverter::toMessage(shifted_map, shifted_msg);
  shifted_msg.info.header.stamp = ros::Time::now();
  shifted_pub.publish(shifted_msg);		// publish shifted_map
  
  return shifted_map;
}

void ExhaustiveSearch(GridMap map, GridMap reference_map, tf::TransformBroadcaster broadcaster) {	// searchs transformation with best fit, TODO should return shift param
  int angle = 45;		// VAR angle resolution
  int every = 5;		// VAR sparsity of search
  grid_map::Position position;
  position(0) = 6;		// VAR max x considered for search, TODO automate
  position(1) = 3;		// VAR max y considered for search, TODO automate 
  Index startIndex;
  reference_map.getIndex(position, startIndex);
  Size size;
  size(0) = (position(0) - (-1))/reference_map.getResolution();	// VAR min x, TODO automate
  size(1) = (position(1) - (-1))/reference_map.getResolution(); // VAR min y, TODO automate
  //float correlation[reference_map.getSize()(0)/every][reference_map.getSize()(1)/every][360/angle];

  ros::Time time = ros::Time::now();	// initialization
  float best_corr = 1;
  grid_map::Position best_position;
  float best_theta;
  GridMap correlation_map({"correlation","rotation"});
  correlation_map.setGeometry(reference_map.getLength(), reference_map.getResolution()*every,
                              reference_map.getPosition());	//TODO only use search submap
  correlation_map.setFrameId("map");

  // iterate sparsely through search area
  for (grid_map::SubmapIteratorSparse iterator(reference_map, startIndex, size, every); !iterator.isPastEnd(); ++iterator) {
      	const Index index(*iterator);
	grid_map::Position xy_position;
	reference_map.getPosition(index, xy_position);		// get coordinates
	for (float theta = 0; theta < 360; theta+=angle)	// iterate over rotation
	{
		float corr = CorrelationSAD(Shift(xy_position, theta, map, broadcaster), reference_map, xy_position, theta);		// get correlation of shifted_map
		//correlation[index(0)][index(1)][int(theta/angle)] = corr;
		
		if (correlation_map.isInside(xy_position)) {
			Index correlation_index;
			correlation_map.getIndex(xy_position, correlation_index);
			bool valid = correlation_map.isValid(correlation_index, "correlation");
			// if no value so far or correlation smaller and correlation valid
			if (((valid == false) || (corr*10 < correlation_map.at("correlation", correlation_index) )) && corr != 1) {
				correlation_map.at("correlation", correlation_index) = corr*10;	//set correlation
				correlation_map.at("rotation", correlation_index) = theta;		//set theta
				if (corr < best_corr) {	// if best correlation store it
					best_corr = corr;
					best_position = xy_position;
					best_theta = theta;
				}
			}
		}
	}
	// publish current correlation_map
	grid_map_msgs::GridMap correlation_msg;
  	GridMapRosConverter::toMessage(correlation_map, correlation_msg);
  	correlation_pub.publish(correlation_msg);
  }
  // output best correlation and time used
  cout << "Best correlation " << best_corr << " at " << best_position.transpose() << " and theta " << best_theta << endl;
  ros::Duration duration = ros::Time::now() - time;
  cout << "Time used: " << duration.toSec() << " Sekunden"<< endl;
  ROS_INFO("done");
}

void GridMap_callback(const grid_map_msgs::GridMap::ConstPtr& msg, tf::TransformBroadcaster broadcaster) {
  ROS_INFO("Recieved grid_map.");
  GridMap map;
  GridMapRosConverter::fromMessage(*msg, map);	// get grid_map from msg
  
  GridMap reference_map;
  GridMapRosConverter::loadFromBag("/home/parallels/rosbags/reference_map_last.bag", "/uav_elevation_mapping/uav_elevation_map", reference_map);	// get grid_map from bag, TODO subscribe to reference_map

  ExhaustiveSearch(map, reference_map, broadcaster);   // TODO should return shift param
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "cloud_fitter");
  ROS_INFO("Hello, world I'm ready to match some grid_maps.");
  ros::NodeHandle nh;

  tf::TransformBroadcaster broadcaster;
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), boost::bind(tfBroadcast, broadcaster), false, false);	// broadcast transform from /world to /map
  timer.start();

  shifted_pub = nh.advertise<grid_map_msgs::GridMap>("elevation_mapping_long_range/shifted_map",1);		// publisher for shifted_map
  correlation_pub = nh.advertise<grid_map_msgs::GridMap>("correlation_best_rotation/correlation_map",1);		// publisher for correlation_map

  ros::Subscriber sub = nh.subscribe<grid_map_msgs::GridMap>("elevation_mapping_long_range/elevation_map", 1, boost::bind(GridMap_callback, _1, broadcaster)); 
  // subscriber to StarlETH_map

  ros::spin();

  return 0; 
}

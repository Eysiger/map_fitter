/*
 * map_fitter_node.cpp
 *
 *  Created on: Apr 04, 2016
 *      Author: Roman Käslin
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#include <ros/ros.h>
#include <map_fitter/MapFitter.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "map_fitter");
  ros::NodeHandle nodeHandle("~");

  map_fitter::MapFitter mapFitter(nodeHandle);

  ros::spin();
  return 0; 
}

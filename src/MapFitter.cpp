/*
 * MapFitter.cpp
 *
 *  Created on: Apr 04, 2016
 *      Author: Roman Käslin
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#include <map_fitter/MapFitter.h>

namespace map_fitter {

MapFitter::MapFitter(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), isActive_(false)
{
  ROS_INFO("Map fitter node started, ready to match some grid maps.");
  readParameters();
  shiftedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(shiftedMapTopic_,1);   // publisher for shifted_map
  correlationPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(correlationMapTopic_,1);    // publisher for correlation_map
  activityCheckTimer_ = nodeHandle_.createTimer(activityCheckDuration_,
                                                &MapFitter::updateSubscriptionCallback,
                                                this);
  broadcastTimer_ = nodeHandle_.createTimer(ros::Duration(0.01), &MapFitter::tfBroadcast, this);
}

MapFitter::~MapFitter()
{
}

bool MapFitter::readParameters()
{
  nodeHandle_.param("map_topic", mapTopic_, std::string("/elevation_mapping_long_range/elevation_map"));
  nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/uav_elevation_mapping/uav_elevation_map"));
  nodeHandle_.param("shifted_map_topic", shiftedMapTopic_, std::string("/elevation_mapping_long_range/shifted_map"));
  nodeHandle_.param("correlation_map_topic", correlationMapTopic_, std::string("/correlation_best_rotation/correlation_map"));

  nodeHandle_.param("angle_increment", angleIncrement_, 45);
  nodeHandle_.param("position_increment_search", searchIncrement_, 5);
  nodeHandle_.param("position_increment_correlation", correlationIncrement_, 5);
  nodeHandle_.param("required_overlap", requiredOverlap_, float(0.75));

  double activityCheckRate;
  nodeHandle_.param("activity_check_rate", activityCheckRate, 1.0);
  activityCheckDuration_.fromSec(1.0 / activityCheckRate);
}

void MapFitter::updateSubscriptionCallback(const ros::TimerEvent&)
{
  if (!isActive_) {
    mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &MapFitter::callback, this);
    isActive_ = true;
    ROS_DEBUG("Subscribed to grid map at '%s'.", mapTopic_.c_str());
  }
}

void MapFitter::callback(const grid_map_msgs::GridMap& message)
{
  ROS_INFO("Map fitter received a map (timestamp %f) for matching.",
            message.info.header.stamp.toSec());
  grid_map::GridMapRosConverter::fromMessage(message, map_);

  grid_map::GridMapRosConverter::loadFromBag("/home/parallels/rosbags/reference_map_last.bag", referenceMapTopic_, referenceMap_);
  exhaustiveSearch();
}

void MapFitter::exhaustiveSearch()
{
  ros::Time time = ros::Time::now();  // initialization
  ros::Duration transform_dur;
  ros::Duration correlation_dur;
  float best_corr = 1;
  grid_map::Position best_position;
  float best_theta;
  grid_map::Position correct_position = map_.getPosition();

  grid_map::Position position;
  position(0) = 6;    // VAR max x considered for search, TODO automate
  position(1) = 3;    // VAR max y considered for search, TODO automate 
  grid_map::Index startIndex;
  referenceMap_.getIndex(position, startIndex);
  grid_map::Size size;
  size(0) = (position(0) - (-1))/referenceMap_.getResolution(); // VAR min x, TODO automate
  size(1) = (position(1) - (-1))/referenceMap_.getResolution(); // VAR min y, TODO automate

  grid_map::GridMap correlationMap_({"correlation","rotation"});
  correlationMap_.setGeometry(referenceMap_.getLength(), referenceMap_.getResolution()*searchIncrement_,
                              referenceMap_.getPosition()); //TODO only use submap
  correlationMap_.setFrameId("map");

  // iterate sparsely through search area
  for (grid_map::SubmapIteratorSparse iterator(referenceMap_, startIndex, size, searchIncrement_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position xy_position;     // TODO check if isInside necessary
    referenceMap_.getPosition(index, xy_position);    // get coordinates
    for (float theta = 0; theta < 360; theta+=angleIncrement_) {  // iterate over rotation 
      //float corr = CorrelationSAD(Shift(xy_position, theta, map, broadcaster), reference_map, xy_position, theta);    // get correlation of shifted_map
      //float corr = CorrelationSSD(Shift(xy_position, theta, map, broadcaster), reference_map, xy_position, theta);    // get correlation of shifted_map
      ros::Time transform_time = ros::Time::now();
      shift(xy_position, theta);
      transform_dur += ros::Time::now() - transform_time;

      ros::Time correlation_time = ros::Time::now();
      float corr = correlationNCC(xy_position, theta);  // get correlation of shifted_map
      //correlation[index(0)][index(1)][int(theta/angle)] = corr;
    
      if (correlationMap_.isInside(xy_position)) {
        grid_map::Index correlation_index;
        correlationMap_.getIndex(xy_position, correlation_index);
        bool valid = correlationMap_.isValid(correlation_index, "correlation");
        // if no value so far or correlation smaller and correlation valid
        if (((valid == false) || (corr*1 < correlationMap_.at("correlation", correlation_index) )) && corr != 1) {
          correlationMap_.at("correlation", correlation_index) = corr*1;  //set correlation
          correlationMap_.at("rotation", correlation_index) = theta;    //set theta
          if (corr < best_corr) { // if best correlation store it
            best_corr = corr;
            best_position = xy_position;
            best_theta = theta;
          }
        }
      }
      correlation_dur += ros::Time::now() - correlation_time;
    }
    // publish current correlation_map
    grid_map_msgs::GridMap correlation_msg;
    grid_map::GridMapRosConverter::toMessage(correlationMap_, correlation_msg);
    correlationPublisher_.publish(correlation_msg);
  }
  ros::Duration duration = ros::Time::now() - time;
  // TODO calculate z alignment
  // output best correlation and time used
  std::cout << "Best correlation " << best_corr << " at " << best_position.transpose() << " and theta " << best_theta << std::endl;
  std::cout << "Correct position " << correct_position.transpose() << " and theta 0" << std::endl;
  std::cout << "Time used: " << duration.toSec() << " Sekunden, (transform, corr)" << transform_dur.toSec() << ", " << correlation_dur.toSec() << std::endl;
  ROS_INFO("done");
  isActive_ = false;
}

void MapFitter::shift(grid_map::Position position, int theta)
{
  grid_map::Position zero_position;
  zero_position(0) = 0.0;
  zero_position(1) = 0.0;
  map_.setPosition(zero_position);  // set position to origin

  // broadcast transformation from /map to /map_rotated
  broadcaster_.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), 
    tf::Vector3(position(0), position(1), 0.0)), ros::Time::now(), "/map", "/map_rotated"));
  map_.setFrameId("map_rotated"); // set frame to /map_rotated

  grid_map_msgs::GridMap shifted_msg;
  grid_map::GridMapRosConverter::toMessage(map_, shifted_msg);
  shifted_msg.info.header.stamp = ros::Time::now();
  shiftedPublisher_.publish(shifted_msg);   // publish shifted_map
}
float MapFitter::correlationNCC(grid_map::Position position, int theta)
{
  float correlation = 0;  // initialization
  float shifted_mean = 0;
  float reference_mean = 0;
  std::vector<float> xy_shifted;
  std::vector<float> xy_reference;
  float shifted_normal = 0;
  float reference_normal = 0;
  int points = 0;
  int matches = 0;


  grid_map::Matrix& data = map_["elevation"];
  // iterate sparsely through template points
  for (grid_map::GridMapIteratorSparse iterator(map_, correlationIncrement_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    float shifted = data(index(0), index(1));
    if (shifted == shifted) {   // check if point is defined, if nan f!= f
      points += 1;    // increase number of valid points
      grid_map::Position xy_position;
      map_.getPosition(index, xy_position);  // get coordinates
      tf::Vector3 xy_vector = tf::Vector3(xy_position(0), xy_position(1), 0.0);

      // transform coordinates from /map_rotated to /map
      tf::Transform transform = tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), tf::Vector3(position(0), position(1), 0.0));
      tf::Vector3 map_vector = transform*(xy_vector); // apply transformation
      grid_map::Position map_position;
      map_position(0) = map_vector.getX();
      map_position(1) = map_vector.getY();

      // check if point is within reference_map
      if (referenceMap_.isInside(map_position)) {
        float reference = referenceMap_.atPosition("elevation", map_position);
        if (reference == reference) {   // check if point is defined, if nan f!= f 
          matches += 1;   // increase number of matched points
          shifted_mean += shifted;
          reference_mean += reference;
          xy_shifted.push_back(shifted);
          xy_reference.push_back(reference);
        }
      }
    }
  }
  // calculate Normalized Cross Correlation (NCC)
  if (matches > points*requiredOverlap_) 
  { 
    shifted_mean = shifted_mean/matches;
    reference_mean = reference_mean/matches;
    for (int i = 0; i < matches; i++) {
      float shifted_corr = (xy_shifted[i]-shifted_mean);
      float reference_corr = (xy_reference[i]-reference_mean);
      correlation += shifted_corr*reference_corr;
      shifted_normal += shifted_corr*shifted_corr;
      reference_normal += reference_corr*reference_corr;
    }
    correlation = correlation/sqrt(shifted_normal*reference_normal);
    return 1 - correlation; 
  }
  else { return 1.0; }

  // check if required overlap is fulfilled
  
}

void MapFitter::tfBroadcast(const ros::TimerEvent&) {
  broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
          ros::Time::now(),"/world", "/map"));
}

} /* namespace */
/*
 * MapFitter.h
 *
 *  Created on: Apr 04, 2016
 *      Author: Roman Käslin
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#ifndef MAPFITTER_H
#define MAPFITTER_H

#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIteratorSparse.hpp>
#include <grid_map_core/iterators/SubmapIteratorSparse.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Core>

namespace map_fitter {

class MapFitter
{
public:
	  /*!
   	 * Constructor.
   	 * @param nodeHandle the ROS node handle.
   	 */
  	MapFitter(ros::NodeHandle& nodeHandle);

  	/*!
   	 * Destructor.
   	 */
  	virtual ~MapFitter();

  	/*!
   	 * Callback function for the grid map.
   	 * @param message the grid map message to be visualized.
   	 */
  	void callback(const grid_map_msgs::GridMap& message);

    void exhaustiveSearch();

    void shift(grid_map::Position position, int theta);

    float correlationNCC(grid_map::Position position, int theta);

private:
    /*!
     * Read parameters from ROS.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Check if visualizations are active (subscribed to),
     * and accordingly cancels/activates the subscription to the
     * grid map to safe bandwidth.
     * @param timerEvent the timer event.
     */
    void updateSubscriptionCallback(const ros::TimerEvent& timerEvent);

    void tfBroadcast(const ros::TimerEvent& timerEvent);


    //! ROS nodehandle.
  	ros::NodeHandle& nodeHandle_;

  	//! Topic name of the grid map to be matched.
 	  std::string mapTopic_;

    //! Topic name of the grid map to be matched to.
    std::string referenceMapTopic_;


    std::string correlationMapTopic_;


    std::string shiftedMapTopic_;

    //! Duration of checking the activity of the visualizations.
    ros::Duration activityCheckDuration_;

    //! Timer to check the activity of the visualizations.
    ros::Timer activityCheckTimer_;

    ros::Timer broadcastTimer_;

    //! If the grid map visualization is subscribed to the grid map.
    bool isActive_;

    //! ROS subscriber to the grid map.
    ros::Subscriber mapSubscriber_;

    //! Template grid_map
    grid_map::GridMap map_;

    //! Reference grid_map
    grid_map::GridMap referenceMap_;


    int angleIncrement_;


    int searchIncrement_;


    int correlationIncrement_;
    

    float requiredOverlap_;


    tf::TransformBroadcaster broadcaster_;


    //! Grid map publisher.
    ros::Publisher shiftedPublisher_;

    //! Grid map publisher.
    ros::Publisher correlationPublisher_;


};

} /* namespace */

#endif
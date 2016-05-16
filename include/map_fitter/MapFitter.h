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
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Core>
#include <geometry_msgs/PointStamped.h>

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

    float findZ(float x, float y, int theta);

    bool findMatches(grid_map::Matrix& data, grid_map::Matrix& variance_data, grid_map::Matrix& reference_data, grid_map::Index reference_index, float theta);

    float mutualInformation();

    float weightedMutualInformation();   

    float errorSAD();

    float weightedErrorSAD();

    float errorSSD();

    float weightedErrorSSD();

    float correlationNCC();

    float weightedCorrelationNCC();

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


    float corrThreshold_;

    float SSDThreshold_;

    float SADThreshold_;

    float MIThreshold_;


    tf::TransformBroadcaster broadcaster_;

    std::vector< std::vector<float> > weightedHist_;

    float map_min_;
    float map_max_;
    float reference_min_;
    float reference_max_;


    //! Grid map publisher.
    ros::Publisher shiftedPublisher_;

    //! Grid map publisher.
    ros::Publisher correlationPublisher_;

    ros::Publisher corrPointPublisher_;
    ros::Publisher SSDPointPublisher_;
    ros::Publisher SADPointPublisher_;
    ros::Publisher MIPointPublisher_;
    ros::Publisher correctPointPublisher_;


    float cumulativeErrorCorr_;
    float cumulativeErrorSSD_;
    float cumulativeErrorSAD_;
    float cumulativeErrorMI_;
    int correctMatchesCorr_;
    int correctMatchesSSD_;
    int correctMatchesSAD_;
    int correctMatchesMI_;

    float shifted_mean_;
    float reference_mean_;
    int matches_;
    std::vector<float> xy_shifted_;
    std::vector<float> xy_reference_;
    std::vector<float> xy_shifted_var_;
    std::vector<float> xy_reference_var_;

    float templateRotation_;

    ros::Duration duration1_;
    ros::Duration duration2_;

};

} /* namespace */

#endif

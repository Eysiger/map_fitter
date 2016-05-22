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
  corrPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/corrPoint",1);
  SSDPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/SSDPoint",1);
  SADPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/SADPoint",1);
  MIPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/MIPoint",1);
  correctPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/correctPoint",1);
}

MapFitter::~MapFitter()
{
}

bool MapFitter::readParameters()
{
  nodeHandle_.param("map_topic", mapTopic_, std::string("/elevation_mapping_long_range/elevation_map"));
  nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/uav_elevation_mapping/uav_elevation_map"));
  //nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/elevation_mapping/elevation_map"));
  nodeHandle_.param("shifted_map_topic", shiftedMapTopic_, std::string("/elevation_mapping_long_range/shifted_map"));
  nodeHandle_.param("correlation_map_topic", correlationMapTopic_, std::string("/correlation_best_rotation/correlation_map"));

  nodeHandle_.param("angle_increment", angleIncrement_, 5);
  nodeHandle_.param("position_increment_search", searchIncrement_, 5);
  nodeHandle_.param("position_increment_correlation", correlationIncrement_, 5);
  nodeHandle_.param("required_overlap", requiredOverlap_, float(0.70));
  nodeHandle_.param("correlation_threshold", corrThreshold_, float(0)); //0.65 weighted, 0.75 unweighted
  nodeHandle_.param("SSD_threshold", SSDThreshold_, float(10));
  nodeHandle_.param("SAD_threshold", SADThreshold_, float(10));
  nodeHandle_.param("MI_threshold", MIThreshold_, float(-10));

  double activityCheckRate;
  nodeHandle_.param("activity_check_rate", activityCheckRate, 1.0);
  activityCheckDuration_.fromSec(1.0 / activityCheckRate);
  cumulativeErrorCorr_ = 0;
  cumulativeErrorSSD_ = 0;
  cumulativeErrorSAD_ = 0;
  cumulativeErrorMI_ = 0;
  correctMatchesCorr_ = 0;
  correctMatchesSSD_ = 0;
  correctMatchesSAD_ = 0;
  correctMatchesMI_ = 0;

  const int numberOfBins = 256;
  weightedHist_.resize(numberOfBins, std::vector<float>(numberOfBins));
  for (int i=0; i < numberOfBins; i++)
  {
    for (int j=i; j < numberOfBins; j++)
    {
      weightedHist_[i][j] = float( ((j-i)+1) )/12;
      weightedHist_[j][i] = float( ((j-i)+1) )/12;
    }
  }
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

  grid_map::GridMapRosConverter::loadFromBag("/home/roman/rosbags/reference_map_last.bag", referenceMapTopic_, referenceMap_);
  //grid_map::GridMapRosConverter::loadFromBag("/home/roman/rosbags/source/asl_walking_uav/uav_reference_map.bag", referenceMapTopic_, referenceMap_);

  exhaustiveSearch();
}

void MapFitter::exhaustiveSearch()
{
  // initialize correlationMap
  grid_map::GridMap correlationMap({"correlation","rotationNCC","SSD","rotationSSD","SAD","rotationSAD", "MI", "rotationMI"});
  correlationMap.setGeometry(referenceMap_.getLength(), referenceMap_.getResolution()*searchIncrement_,
                              referenceMap_.getPosition()); //TODO only use submap
  correlationMap.setFrameId("grid_map");

  //initialize parameters
  grid_map::Size reference_size = referenceMap_.getSize();
  int rows = reference_size(0);
  int cols = reference_size(1);
  grid_map::Matrix acceptedThetas = grid_map::Matrix::Constant(reference_size(0), reference_size(1), 0);

  float best_corr[int(360/angleIncrement_)];
  int corr_row[int(360/angleIncrement_)];
  int corr_col[int(360/angleIncrement_)];

  float best_SSD[int(360/angleIncrement_)];
  int SSD_row[int(360/angleIncrement_)];
  int SSD_col[int(360/angleIncrement_)];

  float best_SAD[int(360/angleIncrement_)];
  int SAD_row[int(360/angleIncrement_)];
  int SAD_col[int(360/angleIncrement_)];

  float best_MI[int(360/angleIncrement_)];
  int MI_row[int(360/angleIncrement_)];
  int MI_col[int(360/angleIncrement_)];

  grid_map::Position correct_position = map_.getPosition();
  grid_map::Position position = referenceMap_.getPosition();
  grid_map::Size size = map_.getSize();
  grid_map::Index start_index = map_.getStartIndex();
  grid_map::Index reference_start_index = referenceMap_.getStartIndex();

  map_min_ = map_.get("elevation").minCoeffOfFinites();
  map_max_ = map_.get("elevation").maxCoeffOfFinites();
  reference_min_ = referenceMap_.get("elevation").minCoeffOfFinites();
  reference_max_ = referenceMap_.get("elevation").maxCoeffOfFinites();

  //float correlation[referenceMapImage_.rows][referenceMapImage_.cols][int(360/angleIncrement_)];

  ros::Time time = ros::Time::now();
  duration1_.sec = 0;
  duration1_.nsec = 0;
  duration2_.sec = 0;
  duration2_.nsec = 0;

  grid_map::Matrix& reference_data = referenceMap_["elevation"];
  grid_map::Matrix& data = map_["elevation"];
  grid_map::Matrix& variance_data = map_["variance"];

  grid_map::Index submap_start_index;
  grid_map::Size submap_size;
  referenceMap_.getDataBoundingSubmap("elevation", submap_start_index, submap_size);
  //std::cout << reference_start_index.transpose() << " reference_size: "<< reference_size.transpose() << "submap" << submap_start_index.transpose() << " size " << submap_size.transpose() << std::endl;

  templateRotation_ = rand() %360;

  for (float theta = templateRotation_; theta < 360 + templateRotation_; theta+=angleIncrement_)
  {
    best_corr[int((theta-templateRotation_)/angleIncrement_)] = -1;
    best_SSD[int((theta-templateRotation_)/angleIncrement_)] = 10;
    best_SAD[int((theta-templateRotation_)/angleIncrement_)] = 10;
    best_MI[int((theta-templateRotation_)/angleIncrement_)] = -10;

    float sin_theta = sin(theta/180*M_PI);
    float cos_theta = cos(theta/180*M_PI);
    // iterate sparsely through search area
    for (grid_map::SubmapIteratorSparse iterator(referenceMap_, submap_start_index, submap_size, searchIncrement_); !iterator.isPastEnd(); ++iterator) 
      //for (grid_map::GridMapIteratorSparse iterator(referenceMap_, searchIncrement_); !iterator.isPastEnd(); ++iterator) 
    {
      grid_map::Index index(*iterator);
      //index = grid_map::getIndexFromBufferIndex(index, reference_size, reference_start_index);
      float errSAD = 10;
      float errSSD = 10;
      float corrNCC = -1;
      float mutInfo = -10;

      bool success = findMatches(data, variance_data, reference_data, index, sin_theta, cos_theta );

      if (success) 
      {
        errSAD = errorSAD();
        errSSD = errorSSD();
        corrNCC = correlationNCC();
        //mutInfo = mutualInformation();

        //errSAD = weightedErrorSAD();
        //errSSD = weightedErrorSSD();
        //corrNCC = weightedCorrelationNCC();
        mutInfo = weightedMutualInformation();
        /*for (int i = 0; i < matches_; i++)
        {
          std::cout << xy_reference_[i] << std::endl;
        }*/


        acceptedThetas(index(0), index(1)) += 1;

        grid_map::Position xy_position;
        referenceMap_.getPosition(index, xy_position);
        if (correlationMap.isInside(xy_position))
        {
          grid_map::Index correlation_index;
          correlationMap.getIndex(xy_position, correlation_index);

          bool valid = correlationMap.isValid(correlation_index, "correlation");
          // if no value so far or correlation smaller or correlation higher than for other thetas
          if (((valid == false) || (corrNCC+1.5 > correlationMap.at("correlation", correlation_index) ))) 
          {
            correlationMap.at("correlation", correlation_index) = corrNCC+1.5;  //set correlation
            correlationMap.at("rotationNCC", correlation_index) = theta;    //set theta
          }

          valid = correlationMap.isValid(correlation_index, "SSD");
          // if no value so far or correlation smaller or correlation higher than for other thetas
          if (((valid == false) || (errSSD*5 < correlationMap.at("SSD", correlation_index) ))) 
          {
            correlationMap.at("SSD", correlation_index) = errSSD*5;  //set correlation
            correlationMap.at("rotationSSD", correlation_index) = theta;    //set theta
          }

          valid = correlationMap.isValid(correlation_index, "SAD");
          // if no value so far or correlation smaller or correlation higher than for other thetas
          if (((valid == false) || (errSSD*5 < correlationMap.at("SAD", correlation_index) ))) 
          {
            correlationMap.at("SAD", correlation_index) = errSAD*5;  //set correlation
            correlationMap.at("rotationSAD", correlation_index) = theta;    //set theta
          }

          valid = correlationMap.isValid(correlation_index, "MI");
          // if no value so far or correlation smaller or correlation higher than for other thetas
          if (((valid == false) || (mutInfo > correlationMap.at("MI", correlation_index) ))) 
          {
            correlationMap.at("MI", correlation_index) = mutInfo;  //set correlation
            correlationMap.at("rotationMI", correlation_index) = theta;    //set theta
          }
        }
                  
        // save best correlation for each theta
        if (corrNCC > best_corr[int((theta-templateRotation_)/angleIncrement_)])
        {
          best_corr[int((theta-templateRotation_)/angleIncrement_)] = corrNCC;
          corr_row[int((theta-templateRotation_)/angleIncrement_)] = index(0);
          corr_col[int((theta-templateRotation_)/angleIncrement_)] = index(1);
        }
        if (errSSD < best_SSD[int((theta-templateRotation_)/angleIncrement_)])
        {
          best_SSD[int((theta-templateRotation_)/angleIncrement_)] = errSSD;
          SSD_row[int((theta-templateRotation_)/angleIncrement_)] = index(0);
          SSD_col[int((theta-templateRotation_)/angleIncrement_)] = index(1);
        }
        if (errSAD < best_SAD[int((theta-templateRotation_)/angleIncrement_)])
        {
          best_SAD[int((theta-templateRotation_)/angleIncrement_)] = errSAD;
          SAD_row[int((theta-templateRotation_)/angleIncrement_)] = index(0);
          SAD_col[int((theta-templateRotation_)/angleIncrement_)] = index(1);
        }
        if (mutInfo > best_MI[int((theta-templateRotation_)/angleIncrement_)])
        {
          best_MI[int((theta-templateRotation_)/angleIncrement_)] = mutInfo;
          MI_row[int((theta-templateRotation_)/angleIncrement_)] = index(0);
          MI_col[int((theta-templateRotation_)/angleIncrement_)] = index(1);
        }
      }
    }
    // publish correlationMap for each theta
    grid_map_msgs::GridMap correlation_msg;
    grid_map::GridMapRosConverter::toMessage(correlationMap, correlation_msg);
    correlationPublisher_.publish(correlation_msg);
  }
  
  //find highest correlation over all theta
  float bestCorr = -1.0;
  float bestSSD = 10;
  float bestSAD = 10;
  float bestMI = -10;
  int bestThetaCorr;
  int bestThetaSSD;
  int bestThetaSAD;
  int bestThetaMI;
  float bestXCorr;
  float bestYCorr;
  float bestXSSD;
  float bestYSSD;
  float bestXSAD;
  float bestYSAD;
  float bestXMI;
  float bestYMI;

  for (int i = 0; i < int(360/angleIncrement_); i++)
  {
    if (best_corr[i] > bestCorr && best_corr[i] >= corrThreshold_) 
    {
      //std::cout << int(acceptedThetas(corr_row[i], corr_col[i])) << " " << int(360/angleIncrement_) << std::endl;
      if (int(acceptedThetas(corr_row[i], corr_col[i])) == int(360/angleIncrement_))
      {
        bestCorr = best_corr[i];
        bestThetaCorr = i*angleIncrement_;
        grid_map::Position best_pos;
        referenceMap_.getPosition(grid_map::Index(corr_row[i], corr_col[i]), best_pos);
        bestXCorr = best_pos(0);
        bestYCorr = best_pos(1);
      }
    }
    if (best_SSD[i] < bestSSD && best_SSD[i] <= SSDThreshold_) 
    {
      if (acceptedThetas(SSD_row[i], SSD_col[i]) == int(360/angleIncrement_))
      {
        bestSSD = best_SSD[i];
        bestThetaSSD = i*angleIncrement_;
        grid_map::Position best_pos;
        referenceMap_.getPosition(grid_map::Index(SSD_row[i], SSD_col[i]), best_pos);
        bestXSSD = best_pos(0);
        bestYSSD = best_pos(1);
      }
    }
    if (best_SAD[i] < bestSAD && best_SAD[i] <= SADThreshold_) 
    {
      if (acceptedThetas(SAD_row[i], SAD_col[i]) == int(360/angleIncrement_))
      {
        bestSAD = best_SAD[i];
        bestThetaSAD = i*angleIncrement_;
        grid_map::Position best_pos;
        referenceMap_.getPosition(grid_map::Index(SAD_row[i], SAD_col[i]), best_pos);
        bestXSAD = best_pos(0);
        bestYSAD = best_pos(1);
      }
    }
    if (best_MI[i] > bestMI && best_MI[i] >= MIThreshold_) 
    {
      if (acceptedThetas(MI_row[i], MI_col[i]) == int(360/angleIncrement_))
      {
        bestMI = best_MI[i];
        bestThetaMI = i*angleIncrement_;
        grid_map::Position best_pos;
        referenceMap_.getPosition(grid_map::Index(MI_row[i], MI_col[i]), best_pos);
        bestXMI = best_pos(0);
        bestYMI = best_pos(1);
      }
    }
  }
  float z = findZ(bestXCorr, bestYCorr, bestThetaCorr);

  ros::Time pubTime = ros::Time::now();
  // output best correlation and time used
  if (bestCorr != -1) 
  {
    cumulativeErrorCorr_ += sqrt((bestXCorr - correct_position(0))*(bestXCorr - correct_position(0)) + (bestYCorr - correct_position(1))*(bestYCorr - correct_position(1)));
    if (sqrt((bestXCorr - correct_position(0))*(bestXCorr - correct_position(0)) + (bestYCorr - correct_position(1))*(bestYCorr - correct_position(1))) < 0.5 && fabs(bestThetaCorr - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesCorr_ += 1;}
    geometry_msgs::PointStamped corrPoint;
    corrPoint.point.x = bestXCorr;
    corrPoint.point.y = bestYCorr;
    corrPoint.point.z = bestThetaCorr;
    corrPoint.header.stamp = pubTime;
    corrPointPublisher_.publish(corrPoint);
  }
  if (bestSSD != 10) 
  {
    cumulativeErrorSSD_ += sqrt((bestXSSD - correct_position(0))*(bestXSSD - correct_position(0)) + (bestYSSD - correct_position(1))*(bestYSSD - correct_position(1)));
    if (sqrt((bestXSSD - correct_position(0))*(bestXSSD - correct_position(0)) + (bestYSSD - correct_position(1))*(bestYSSD - correct_position(1))) < 0.5 && fabs(bestThetaSSD - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesSSD_ += 1;}
    geometry_msgs::PointStamped SSDPoint;
    SSDPoint.point.x = bestXSSD;
    SSDPoint.point.y = bestYSSD;
    SSDPoint.point.z = bestThetaSSD;
    SSDPoint.header.stamp = pubTime;
    SSDPointPublisher_.publish(SSDPoint);
  }
  if (bestSAD != 10) 
  {
    cumulativeErrorSAD_ += sqrt((bestXSAD - correct_position(0))*(bestXSAD - correct_position(0)) + (bestYSAD - correct_position(1))*(bestYSAD - correct_position(1)));
    if (sqrt((bestXSAD - correct_position(0))*(bestXSAD - correct_position(0)) + (bestYSAD - correct_position(1))*(bestYSAD - correct_position(1))) < 0.5 && fabs(bestThetaSAD - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesSAD_ += 1;}
    geometry_msgs::PointStamped SADPoint;
    SADPoint.point.x = bestXSAD;
    SADPoint.point.y = bestYSAD;
    SADPoint.point.z = bestThetaSAD;
    SADPoint.header.stamp = pubTime;
    SADPointPublisher_.publish(SADPoint);
  }
  if (bestMI != 0) 
  {
    cumulativeErrorMI_ += sqrt((bestXMI - correct_position(0))*(bestXMI - correct_position(0)) + (bestYMI - correct_position(1))*(bestYMI - correct_position(1)));
    if (sqrt((bestXMI - correct_position(0))*(bestXMI - correct_position(0)) + (bestYMI - correct_position(1))*(bestYMI - correct_position(1))) < 0.5 && fabs(bestThetaMI - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesMI_ += 1;}
    geometry_msgs::PointStamped MIPoint;
    MIPoint.point.x = bestXMI;
    MIPoint.point.y = bestYMI;
    MIPoint.point.z = bestThetaMI;
    MIPoint.header.stamp = pubTime;
    MIPointPublisher_.publish(MIPoint);
  }
  geometry_msgs::PointStamped correctPoint;
  correctPoint.point.x = correct_position(0);
  correctPoint.point.y = correct_position(1);
  correctPoint.point.z = templateRotation_;
  correctPoint.header.stamp = pubTime;
  correctPointPublisher_.publish(correctPoint);


  ros::Duration duration = ros::Time::now() - time;
  std::cout << "Best correlation " << bestCorr << " at " << bestXCorr << ", " << bestYCorr << " and theta " << bestThetaCorr << " and z: " << z << std::endl;
  std::cout << "Best SSD " << bestSSD << " at " << bestXSSD << ", " << bestYSSD << " and theta " << bestThetaSSD << std::endl;
  std::cout << "Best SAD " << bestSAD << " at " << bestXSAD << ", " << bestYSAD << " and theta " << bestThetaSAD << std::endl;
  std::cout << "Best MI " << bestMI << " at " << bestXMI << ", " << bestYMI << " and theta " << bestThetaMI << std::endl;
  std::cout << "Correct position " << correct_position.transpose() << " and theta " << (360-int(templateRotation_))%360 << std::endl;
  std::cout << "Time used: " << duration.toSec() << " Sekunden" << " 1: " << duration1_.toSec() << " 2: " << duration2_.toSec() << std::endl;
  std::cout << "Cumulative error NCC: " << cumulativeErrorCorr_ << " matches: " << correctMatchesCorr_ << " SSD: " << cumulativeErrorSSD_ << " matches: " << correctMatchesSSD_ << " SAD: " << cumulativeErrorSAD_ << " matches: " << correctMatchesSAD_ << " MI: " << cumulativeErrorMI_ << " matches: " << correctMatchesMI_ << std::endl;
  ROS_INFO("done");
  isActive_ = false;
}

float MapFitter::findZ(float x, float y, int theta)
{
  // initialize
  float shifted_mean = 0;
  float reference_mean = 0;
  int matches = 0;

  grid_map::Matrix& data = map_["elevation"];
  for (grid_map::GridMapIteratorSparse iterator(map_, correlationIncrement_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    float shifted = data(index(0), index(1));
    if (shifted == shifted) {   // check if point is defined, if nan f!= f 
      grid_map::Position xy_position;
      map_.getPosition(index, xy_position);  // get coordinates
      tf::Vector3 xy_vector = tf::Vector3(xy_position(0), xy_position(1), 0.0);

      // transform coordinates from /map_rotated to /grid_map
      tf::Transform transform = tf::Transform(tf::Quaternion(0.0, 0.0, sin(theta/180*M_PI/2), cos(theta/180*M_PI/2)), tf::Vector3(x, y, 0.0));
      tf::Vector3 map_vector = transform*(xy_vector); // apply transformation
      grid_map::Position map_position;
      map_position(0) = map_vector.getX();
      map_position(1) = map_vector.getY();

      // check if point is within reference_map
      if (referenceMap_.isInside(map_position)) {
        float reference = referenceMap_.atPosition("elevation", map_position);
        if (reference == reference) {   // check if point is defined, if nan f!= f 
          matches += 1;
          shifted_mean += shifted;
          reference_mean += reference;
        }
      }
    }
  }
  // calculate mean
  shifted_mean = shifted_mean/matches;
  reference_mean = reference_mean/matches;

  return reference_mean - shifted_mean;
}

bool MapFitter::findMatches(grid_map::Matrix& data, grid_map::Matrix& variance_data, grid_map::Matrix& reference_data, grid_map::Index reference_index, float sin_theta, float cos_theta)
{
  // initialize
  int points = 0;
  matches_ = 0;

  shifted_mean_ = 0;
  reference_mean_ = 0;
  xy_shifted_.clear();
  xy_reference_.clear();
  xy_shifted_var_.clear();
  //xy_reference_var_.clear();

  Eigen::Array2i size = map_.getSize();
  int size_x = size(0);
  int size_y = size(1);
  Eigen::Array2i start_index = map_.getStartIndex();
  int start_index_x = start_index(0);
  int start_index_y = start_index(1);
  Eigen::Array2i reference_size = referenceMap_.getSize();
  int reference_size_x = reference_size(0);
  int reference_size_y = reference_size(1);
  Eigen::Array2i reference_start_index = referenceMap_.getStartIndex();
  int reference_start_index_x = reference_start_index(0);
  int reference_start_index_y = reference_start_index(1);
  int reference_index_x = reference_index(0);
  int reference_index_y = reference_index(1);

  for (int i = 0; i <= size_x-correlationIncrement_; i += correlationIncrement_)
  {
    for (int j = 0; j<= size_y-correlationIncrement_; j += correlationIncrement_)
    {
      int index_x = (start_index_x + i) % size_x;
      int index_y = (start_index_y + j) % size_y;

      float mapHeight = data(index_x, index_y);
        ros::Time time1 = ros::Time::now();
      if (mapHeight == mapHeight)
      {
        points += 1;
        int reference_buffer_index_x = reference_size_x - reference_start_index_x + reference_index_x;
        int reference_buffer_index_y = reference_size_y - reference_start_index_y + reference_index_y;

              ros::Time time2 = ros::Time::now();
        int shifted_index_x = reference_buffer_index_x % reference_size_x - round(cos_theta*(float(size_x)/2-i) - sin_theta*(float(size_y)/2-j));
        int shifted_index_y = reference_buffer_index_y % reference_size_y - sin_theta*(float(size_x)/2-i) - cos_theta*(float(size_y)/2-j);
              duration2_ += ros::Time::now() - time2;
        if (shifted_index_x >= 0 && shifted_index_x < reference_size_x && shifted_index_y >= 0 && shifted_index_y < reference_size_y )
        {
          shifted_index_x = (shifted_index_x + reference_start_index_x) % reference_size_x;
          shifted_index_y = (shifted_index_y + reference_start_index_y) % reference_size_y;
          float referenceHeight = reference_data(shifted_index_x, shifted_index_y);
          //std::cout << referenceHeight << " " << shifted_index_x <<", " << shifted_index_y << std::endl;
          if (referenceHeight == referenceHeight)
          {
            matches_ += 1;
            shifted_mean_ += mapHeight;
            reference_mean_ += referenceHeight;
            xy_shifted_.push_back(mapHeight);
            xy_reference_.push_back(referenceHeight);
            float mapVariance = variance_data(index_x, index_y);
            xy_shifted_var_.push_back(1 / mapVariance);
          }
        }
      }
       duration1_ += ros::Time::now() - time1;
    }
  }


  // check if required overlap is fulfilled
  if (matches_ > points*requiredOverlap_) 
  { 
    shifted_mean_ = shifted_mean_/matches_;
    reference_mean_ = reference_mean_/matches_;
    return true; 
  }
  else { return false; }
}

float MapFitter::mutualInformation()
{
  const int numberOfBins = 256;

  float minHeight = map_min_ - shifted_mean_;
  if ((reference_min_ - reference_mean_) < minHeight)
  {
    minHeight = reference_min_ - reference_mean_;
  }
  float maxHeight = map_max_ - shifted_mean_;
  if ((reference_max_ - reference_mean_) > maxHeight)
  {
    maxHeight = reference_max_ - reference_mean_;
  }
  float binWidth = (maxHeight - minHeight)/(numberOfBins - 1);
  float hist[numberOfBins] = {0.0};
  float referenceHist[numberOfBins] = {0.0};
  float jointHist[numberOfBins][numberOfBins] = {0.0};

  for (int i = 0; i < matches_; i++)
  {
    float i1 = xy_shifted_[i] - shifted_mean_;
    float i2 = xy_reference_[i] - reference_mean_;
    //std::cout << int((i1-minHeight) / binWidth) << " " << int((i2-minHeight) / binWidth) << std::endl;
    hist[int((i1-minHeight) / binWidth)] += 1.0/matches_;
    referenceHist[int((i2-minHeight) / binWidth)] += 1.0/matches_;
    jointHist[int((i1-minHeight) / binWidth)][int((i2-minHeight) / binWidth)] += 1.0/matches_;
  }
  
  float entropy = 0;
  float referenceEntropy = 0;
  float jointEntropy = 0;

  for (int i = 0; i < numberOfBins; i++)
  {
    if (hist[i]!=0.0) {entropy += fabs(hist[i])*log(fabs(hist[i]));}
    if (referenceHist[i]!=0.0) {referenceEntropy += fabs(referenceHist[i])*log(fabs(referenceHist[i]));}

    for (int j = 0; j < numberOfBins; j++)
    {
      if (jointHist[i][j]!=0.0) {jointEntropy += fabs(jointHist[i][j])*log(fabs(jointHist[i][j]));}
    }
  }

  entropy = -1*entropy;;
  referenceEntropy = -1*referenceEntropy;
  jointEntropy = -1*jointEntropy;

  //std::cout << " template entropy: " << entropy << " reference entropy: " << referenceEntropy << " joint entropy: " << jointEntropy << " Mutual information: " << entropy+referenceEntropy-jointEntropy <<std::endl;

  return (entropy+referenceEntropy-jointEntropy);
}

float MapFitter::weightedMutualInformation()
{
  const int numberOfBins = 256;

  float minHeight = map_min_ - shifted_mean_;
  if ((reference_min_ - reference_mean_) < minHeight)
  {
    minHeight = reference_min_ - reference_mean_;
  }
  float maxHeight = map_max_ - shifted_mean_;
  if ((reference_max_ - reference_mean_) > maxHeight)
  {
    maxHeight = reference_max_ - reference_mean_;
  }

  float binWidth = (maxHeight - minHeight)/(numberOfBins - 1);
  float hist[numberOfBins] = {0.0};
  float referenceHist[numberOfBins] = {0.0};
  float jointHist[numberOfBins][numberOfBins] = {0.0};

  for (int i = 0; i < matches_; i++)
  {
    float i1 = xy_shifted_[i] - shifted_mean_;
    float i2 = xy_reference_[i] - reference_mean_;
    //std::cout << int((i1-minHeight) / binWidth) << " " << int((i2-minHeight) / binWidth) << std::endl;
    hist[int((i1-minHeight) / binWidth)] += 1.0/matches_;
    referenceHist[int((i2-minHeight) / binWidth)] += 1.0/matches_;
    jointHist[int((i1-minHeight) / binWidth)][int((i2-minHeight) / binWidth)] += 1.0/matches_;
  }
  
  float entropy = 0;
  float referenceEntropy = 0;
  float jointEntropy = 0;

  for (int i = 0; i < numberOfBins; i++)
  {
    if (hist[i]!=0.0) {entropy += fabs(hist[i])*log(fabs(hist[i]));}
    if (referenceHist[i]!=0.0) {referenceEntropy += fabs(referenceHist[i])*log(fabs(referenceHist[i]));}

    for (int j = 0; j < numberOfBins; j++)
    {
      if (jointHist[i][j]!=0.0) {jointEntropy += weightedHist_[i][j]*fabs(jointHist[i][j])*log(fabs(jointHist[i][j]));}
    }
  }

  entropy = -1*entropy;;
  referenceEntropy = -1*referenceEntropy;
  jointEntropy = -1*jointEntropy;

  //std::cout << " template entropy: " << entropy << " reference entropy: " << referenceEntropy << " joint entropy: " << jointEntropy << " Mutual information: " << entropy+referenceEntropy-jointEntropy <<std::endl;
  return (entropy+referenceEntropy-jointEntropy);
}

float MapFitter::errorSAD()
{
  float error = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_)/std::numeric_limits<unsigned short>::max();
    float reference = (xy_reference_[i]-reference_mean_)/std::numeric_limits<unsigned short>::max();
    error += fabs(shifted-reference);
  }
  // divide error by number of matches
  //std::cout << error/matches_ <<std::endl;
  return error/matches_;
}

float MapFitter::weightedErrorSAD()
{
  float error = 0;
  float normalization = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_)/std::numeric_limits<unsigned short>::max();
    float reference = (xy_reference_[i]-reference_mean_)/std::numeric_limits<unsigned short>::max();
    error += fabs(shifted-reference) * xy_shifted_var_[i];// * xy_reference_var_[i];
    normalization += xy_shifted_var_[i];// * (xy_reference_var_[i]/std::numeric_limits<unsigned short>::max());
  }
  // divide error by number of matches
  //std::cout << error/normalization <<std::endl;
  return error/normalization;
}

float MapFitter::errorSSD()
{
  float error = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_)/std::numeric_limits<unsigned short>::max();
    float reference = (xy_reference_[i]-reference_mean_)/std::numeric_limits<unsigned short>::max();
    error += sqrt(fabs(shifted-reference)); //(shifted-reference)*(shifted-reference); //sqrt(fabs(shifted-reference)) instead of (shifted-reference)*(shifted-reference), since values are in between 0 and 1
  }
  // divide error by number of matches
  //std::cout << error/matches_ <<std::endl;
  return error/matches_;
}

float MapFitter::weightedErrorSSD()
{
  float error = 0;
  float normalization = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_)/std::numeric_limits<unsigned short>::max();
    float reference = (xy_reference_[i]-reference_mean_)/std::numeric_limits<unsigned short>::max();
    error += sqrt(fabs(shifted-reference)) * xy_shifted_var_[i]*xy_shifted_var_[i];// * xy_reference_var_[i]; //sqrt(fabs(shifted-reference)) instead of (shifted-reference)*(shifted-reference), since values are in between 0 and 1
    normalization += xy_shifted_var_[i]*xy_shifted_var_[i];// * (xy_reference_var_[i]/std::numeric_limits<unsigned short>::max());
  }
  // divide error by number of matches
  //std::cout << error/normalization <<std::endl;
  return error/normalization;
}

float MapFitter::correlationNCC()
{
  float shifted_normal = 0;
  float reference_normal = 0;
  float correlation = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted_corr = (xy_shifted_[i]-shifted_mean_);
    float reference_corr = (xy_reference_[i]-reference_mean_);
    correlation += shifted_corr*reference_corr;
    shifted_normal += shifted_corr*shifted_corr;
    reference_normal += reference_corr*reference_corr;
  }
  return correlation/sqrt(shifted_normal*reference_normal);
}

float MapFitter::weightedCorrelationNCC()
{
  // calculate Normalized Cross Correlation (NCC)
  float shifted_normal = 0;
  float reference_normal = 0;
  float correlation = 0;
  for (int i = 0; i < matches_; i++) 
  {
    //for 1/variance
    float shifted_corr = (xy_shifted_[i]-shifted_mean_); // * xy_shifted_var_[i]
    float reference_corr = (xy_reference_[i]-reference_mean_);// * xy_reference_var_[i];
    correlation += shifted_corr*reference_corr * xy_shifted_var_[i];
    shifted_normal += xy_shifted_var_[i]*shifted_corr*shifted_corr;// shifted_corr*shifted_corr * xy_shifted_var_[i];
    reference_normal += xy_shifted_var_[i]*reference_corr*reference_corr;// reference_corr*reference_corr * xy_shifted_var_[i];
    
    //for 1 - variance
    /*float shifted_corr = (xy_shifted_[i]-shifted_mean_)* xy_shifted_var_[i];
    float reference_corr = (xy_reference_[i]-reference_mean_);// * xy_reference_var_[i];
    correlation += shifted_corr*reference_corr;
    shifted_normal += shifted_corr * shifted_corr;
    reference_normal += reference_corr * reference_corr;*/
  }
  return correlation/sqrt(shifted_normal*reference_normal);
}

void MapFitter::tfBroadcast(const ros::TimerEvent&) {
  broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
          ros::Time::now(),"/map", "/grid_map"));
}

} /* namespace */

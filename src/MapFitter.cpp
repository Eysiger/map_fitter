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
    : nodeHandle_(nodeHandle), isActive_(false), isFirst_(true)
{
  ROS_INFO("Map fitter node started, ready to match some grid maps.");
  readParameters();
  shiftedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(shiftedMapTopic_,1);   // publisher for shifted_map
  correlationPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(correlationMapTopic_,1);    // publisher for correlation_map
  referencePublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/uav_elevation_mapping/uav_elevation_map",1); // change back to referenceMapTopic_
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
  //nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/uav_elevation_mapping/uav_elevation_map"));
  nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/elevation_mapping/elevation_map"));
  nodeHandle_.param("shifted_map_topic", shiftedMapTopic_, std::string("/elevation_mapping_long_range/shifted_map"));
  nodeHandle_.param("correlation_map_topic", correlationMapTopic_, std::string("/correlation_best_rotation/correlation_map"));

  nodeHandle_.param("angle_increment", angleIncrement_, 5);
  nodeHandle_.param("position_increment_search", searchIncrement_, 5);
  nodeHandle_.param("position_increment_correlation", correlationIncrement_, 5);
  nodeHandle_.param("required_overlap", requiredOverlap_, float(0.75));
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

  //grid_map::GridMapRosConverter::loadFromBag("/home/parallels/rosbags/reference_map_last.bag", referenceMapTopic_, referenceMap_);
  //referenceMap_.move(grid_map::Position(2.75,1));

  //grid_map::GridMap extendMap;
  //extendMap.setGeometry(grid_map::Length(10.5,7.5), referenceMap_.getResolution(), referenceMap_.getPosition());
  //extendMap.setFrameId("grid_map");
  //referenceMap_.extendToInclude(extendMap);
  
  grid_map::GridMapRosConverter::loadFromBag("/home/parallels/rosbags/source/asl_walking_uav/uav_reference_map.bag", referenceMapTopic_, referenceMap_);

  exhaustiveSearch();
}

void MapFitter::exhaustiveSearch()
{
  // initialize correlationMap
  grid_map::GridMap correlationMap({"correlation","rotationNCC","SSD","rotationSSD","SAD","rotationSAD", "MI", "rotationMI"});
  correlationMap.setGeometry(referenceMap_.getLength(), referenceMap_.getResolution(),
                              referenceMap_.getPosition()); //TODO only use submap
  correlationMap.setFrameId("grid_map");

  //initialize parameters
  grid_map::Size reference_size = referenceMap_.getSize();
  int rows = reference_size(0);
  int cols = reference_size(1);
  //grid_map::Matrix acceptedThetas = grid_map::Matrix::Constant(reference_size(0), reference_size(1), 0);

  std::vector<float> SAD;
  std::vector<float> SSD;
  std::vector<float> NCC;
  std::vector<float> MI;
  SAD.clear();
  SSD.clear();
  NCC.clear();
  MI.clear();

  grid_map::Position previous_position = correct_position_;
  correct_position_ = map_.getPosition();
  grid_map::Position position = referenceMap_.getPosition();
  grid_map::Size size = map_.getSize();
  grid_map::Index start_index = map_.getStartIndex();
  grid_map::Index reference_start_index = referenceMap_.getStartIndex();

  map_min_ = map_.get("elevation").minCoeffOfFinites();
  map_max_ = map_.get("elevation").maxCoeffOfFinites();
  reference_min_ = referenceMap_.get("elevation").minCoeffOfFinites();
  reference_max_ = referenceMap_.get("elevation").maxCoeffOfFinites();

  ros::Time time = ros::Time::now();
  duration1_.sec = 0;
  duration1_.nsec = 0;
  duration2_.sec = 0;
  duration2_.nsec = 0;

  std::normal_distribution<float> distribution(0.0,3.0);

  grid_map::Matrix& reference_data = referenceMap_["elevation"];
  grid_map::Matrix& data = map_["elevation"];
  grid_map::Matrix& variance_data = map_["variance"];

  grid_map::Index submap_start_index;
  grid_map::Size submap_size;
  referenceMap_.getDataBoundingSubmap("elevation", submap_start_index, submap_size);
  //std::cout << reference_start_index.transpose() << " reference_size: "<< reference_size.transpose() << "submap" << submap_start_index.transpose() << " size " << submap_size.transpose() << std::endl;


  /*for (grid_map::GridMapIterator iterator(referenceMap_); !iterator.isPastEnd(); ++iterator) 
  {
    grid_map::Index index(*iterator);
    grid_map::Index shaped_index = grid_map::getIndexFromBufferIndex(index, reference_size, reference_start_index);
    grid_map::Index shaped_submap_start_index = grid_map::getIndexFromBufferIndex(submap_start_index, reference_size, reference_start_index);
    bool outside_submap = (shaped_index(0) < shaped_submap_start_index(0) || shaped_index(1) < shaped_submap_start_index(1) || shaped_index(0) > shaped_submap_start_index(0)+submap_size(0) || shaped_index(1) > shaped_submap_start_index(1)+submap_size(1));
    if ( outside_submap )
    {
      referenceMap_.at("elevation", index) = -0.75;
    }
  }*/
  grid_map_msgs::GridMap reference_msg;
  grid_map::GridMapRosConverter::toMessage(referenceMap_, reference_msg);
  referencePublisher_.publish(reference_msg);

  // initialize particles
  if (isFirst_)
  {
    numberOfParticles_ = 0;
    for (float theta = 0; theta < 360; theta += angleIncrement_)
    {
      for (grid_map::SubmapIteratorSparse iterator(referenceMap_, submap_start_index, submap_size, searchIncrement_); !iterator.isPastEnd(); ++iterator) 
      //for (grid_map::GridMapIteratorSparse iterator(referenceMap_, searchIncrement_); !iterator.isPastEnd(); ++iterator) 
      {
        grid_map::Index index(*iterator);
        particleRow_.push_back(index(0));
        particleCol_.push_back(index(1));
        particleTheta_.push_back(theta);
        numberOfParticles_ += 1;
      }
    }
    templateRotation_ = static_cast <float> (rand() / static_cast <float> (RAND_MAX/360)); //rand() %360;

    std::cout <<"Number of particles: " << numberOfParticles_ << std::endl;
    isFirst_ = false;
  }
  else
  {
    //std::transform(particleRow_.begin(), particleRow_.end(), particleRow_.begin(), std::bind2nd(std::plus<int>(), round( -(correct_position_(0) - previous_position(0)) / referenceMap_.getResolution() + rows ) ));
    //std::transform(particleRow_.begin(), particleRow_.end(), particleRow_.begin(), std::bind2nd(std::modulus<int>(), rows));

    //std::transform(particleCol_.begin(), particleCol_.end(), particleCol_.begin(), std::bind2nd(std::plus<int>(), round( -(correct_position_(1) - previous_position(1)) / referenceMap_.getResolution() + cols ) ));
    //std::transform(particleCol_.begin(), particleCol_.end(), particleCol_.begin(), std::bind2nd(std::modulus<int>(), cols));

    templateRotation_ = fmod(templateRotation_ + distribution(generator_)/2 + 360, 360);
  }

  for (int i = 0; i < numberOfParticles_; i++)
  {
    int row = particleRow_[i];
    int col = particleCol_[i];
    grid_map::Index index = grid_map::Index(row, col);
    int theta = particleTheta_[i];

    float sin_theta = sin((theta+templateRotation_)/180*M_PI);
    float cos_theta = cos((theta+templateRotation_)/180*M_PI);

    bool success = findMatches(data, variance_data, reference_data, index, sin_theta, cos_theta );
    ros::Time time2 = ros::Time::now();
    if (success) 
    {
      
      float errSAD = errorSAD();
      float errSSD = errorSSD();
      float corrNCC = correlationNCC();
      //float mutInfo = mutualInformation();

      //float errSAD = weightedErrorSAD();
      //float errSSD = weightedErrorSSD();
      //float corrNCC = weightedCorrelationNCC();
      //float mutInfo = weightedMutualInformation();


      SAD.push_back(errSAD);
      SSD.push_back(errSSD);
      NCC.push_back(corrNCC);
      //MI.push_back(mutInfo);

      //acceptedThetas(index(0), index(1)) += 1;

      grid_map::Position xy_position;
      referenceMap_.getPosition(index, xy_position);
      if (correlationMap.isInside(xy_position))
      {
        grid_map::Index correlation_index;
        correlationMap.getIndex(xy_position, correlation_index);

        bool valid = correlationMap.isValid(correlation_index, "correlation");
        // if no value so far or correlation smaller or correlation higher than for other thetas
        if (((valid == false) || (corrNCC+1 > correlationMap.at("correlation", correlation_index) )))// && fabs(theta - (360-int(templateRotation_))%360) < angleIncrement_) 
        {
          correlationMap.at("correlation", correlation_index) = corrNCC+1;  //set correlation
          correlationMap.at("rotationNCC", correlation_index) = theta;    //set theta
        }

        valid = correlationMap.isValid(correlation_index, "SSD");
        // if no value so far or correlation smaller or correlation higher than for other thetas
        if (((valid == false) || (errSSD*1000 < correlationMap.at("SSD", correlation_index) ))) 
        {
          correlationMap.at("SSD", correlation_index) = errSSD*1000;  //set correlation
          correlationMap.at("rotationSSD", correlation_index) = theta;    //set theta
        }

        valid = correlationMap.isValid(correlation_index, "SAD");
        // if no value so far or correlation smaller or correlation higher than for other thetas
        if (((valid == false) || (errSSD*1000000 < correlationMap.at("SAD", correlation_index) ))) 
        {
          correlationMap.at("SAD", correlation_index) = errSAD*1000000;  //set correlation
          correlationMap.at("rotationSAD", correlation_index) = theta;    //set theta
        }

        /*valid = correlationMap.isValid(correlation_index, "MI");
        // if no value so far or correlation smaller or correlation higher than for other thetas
        if (((valid == false) || (mutInfo > correlationMap.at("MI", correlation_index) ))) 
        {
          correlationMap.at("MI", correlation_index) = mutInfo;  //set correlation
          correlationMap.at("rotationMI", correlation_index) = theta;    //set theta
        }*/
      }
    }
    else
    {
      SAD.push_back(10);
      SSD.push_back(10);
      NCC.push_back(-1);
      MI.push_back(-10);
    }
    duration2_ += ros::Time::now() - time2;
  }

  grid_map_msgs::GridMap correlation_msg;
  grid_map::GridMapRosConverter::toMessage(correlationMap, correlation_msg);
  correlationPublisher_.publish(correlation_msg);
  
  // search best score for all error measures
  grid_map::Position best_pos;

  std::vector<float>::iterator SADit = std::min_element(SAD.begin(), SAD.end());
  int bestSADparticle = std::distance(SAD.begin(), SADit);
  float bestSAD = SAD[bestSADparticle];
  referenceMap_.getPosition(grid_map::Index(particleRow_[bestSADparticle], particleCol_[bestSADparticle]), best_pos);
  float bestXSAD = best_pos(0);
  float bestYSAD = best_pos(1);
  int bestThetaSAD = particleTheta_[bestSADparticle];

  std::vector<float>::iterator SSDit = std::min_element(SSD.begin(), SSD.end());
  int bestSSDparticle = std::distance(SSD.begin(), SSDit);
  float bestSSD = SSD[bestSSDparticle];
  referenceMap_.getPosition(grid_map::Index(particleRow_[bestSSDparticle], particleCol_[bestSSDparticle]), best_pos);
  float bestXSSD = best_pos(0);
  float bestYSSD = best_pos(1);
  int bestThetaSSD = particleTheta_[bestSSDparticle];

  std::vector<float>::iterator NCCit = std::max_element(NCC.begin(), NCC.end());
  int bestNCCparticle = std::distance(NCC.begin(), NCCit);
  float bestNCC = NCC[bestNCCparticle];
  referenceMap_.getPosition(grid_map::Index(particleRow_[bestNCCparticle], particleCol_[bestNCCparticle]), best_pos);
  float bestXNCC = best_pos(0);
  float bestYNCC = best_pos(1);
  int bestThetaNCC = particleTheta_[bestNCCparticle];;

  /*std::vector<float>::iterator MIit = std::max_element(MI.begin(), MI.end());
  int bestMIparticle = std::distance(MI.begin(), MIit);
  float bestMI = MI[bestMIparticle];
  referenceMap_.getPosition(grid_map::Index(particleRow_[bestMIparticle], particleCol_[bestMIparticle]), best_pos);
  float bestXMI = best_pos(0);
  float bestYMI = best_pos(1);
  int bestThetaMI = particleTheta_[bestMIparticle];*/


  // Calculate z alignement
  float z = findZ(data, reference_data, bestXNCC, bestYNCC, bestThetaNCC);

  ros::Time pubTime = ros::Time::now();
  // output best correlation and time used
  if (bestNCC != -1) 
  {
    cumulativeErrorCorr_ += sqrt((bestXNCC - correct_position_(0))*(bestXNCC - correct_position_(0)) + (bestYNCC - correct_position_(1))*(bestYNCC - correct_position_(1)));
    if (sqrt((bestXNCC - correct_position_(0))*(bestXNCC - correct_position_(0)) + (bestYNCC - correct_position_(1))*(bestYNCC - correct_position_(1))) < 0.5 && fabs(bestThetaNCC - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesCorr_ += 1;}
    geometry_msgs::PointStamped corrPoint;
    corrPoint.point.x = bestXNCC;
    corrPoint.point.y = bestYNCC;
    corrPoint.point.z = bestThetaNCC;
    corrPoint.header.stamp = pubTime;
    corrPointPublisher_.publish(corrPoint);
  }
  if (bestSSD != 10) 
  {
    cumulativeErrorSSD_ += sqrt((bestXSSD - correct_position_(0))*(bestXSSD - correct_position_(0)) + (bestYSSD - correct_position_(1))*(bestYSSD - correct_position_(1)));
    if (sqrt((bestXSSD - correct_position_(0))*(bestXSSD - correct_position_(0)) + (bestYSSD - correct_position_(1))*(bestYSSD - correct_position_(1))) < 0.5 && fabs(bestThetaSSD - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesSSD_ += 1;}
    geometry_msgs::PointStamped SSDPoint;
    SSDPoint.point.x = bestXSSD;
    SSDPoint.point.y = bestYSSD;
    SSDPoint.point.z = bestThetaSSD;
    SSDPoint.header.stamp = pubTime;
    SSDPointPublisher_.publish(SSDPoint);
  }
  if (bestSAD != 10) 
  {
    cumulativeErrorSAD_ += sqrt((bestXSAD - correct_position_(0))*(bestXSAD - correct_position_(0)) + (bestYSAD - correct_position_(1))*(bestYSAD - correct_position_(1)));
    if (sqrt((bestXSAD - correct_position_(0))*(bestXSAD - correct_position_(0)) + (bestYSAD - correct_position_(1))*(bestYSAD - correct_position_(1))) < 0.5 && fabs(bestThetaSAD - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesSAD_ += 1;}
    geometry_msgs::PointStamped SADPoint;
    SADPoint.point.x = bestXSAD;
    SADPoint.point.y = bestYSAD;
    SADPoint.point.z = bestThetaSAD;
    SADPoint.header.stamp = pubTime;
    SADPointPublisher_.publish(SADPoint);
  }
  /*if (bestMI != 0) 
  {
    cumulativeErrorMI_ += sqrt((bestXMI - correct_position_(0))*(bestXMI - correct_position_(0)) + (bestYMI - correct_position_(1))*(bestYMI - correct_position_(1)));
    if (sqrt((bestXMI - correct_position_(0))*(bestXMI - correct_position_(0)) + (bestYMI - correct_position_(1))*(bestYMI - correct_position_(1))) < 0.5 && fabs(bestThetaMI - (360-int(templateRotation_))%360) < angleIncrement_) {correctMatchesMI_ += 1;}
    geometry_msgs::PointStamped MIPoint;
    MIPoint.point.x = bestXMI;
    MIPoint.point.y = bestYMI;
    MIPoint.point.z = bestThetaMI;
    MIPoint.header.stamp = pubTime;
    MIPointPublisher_.publish(MIPoint);
  }*/
  geometry_msgs::PointStamped correctPoint;
  correctPoint.point.x = correct_position_(0);
  correctPoint.point.y = correct_position_(1);
  correctPoint.point.z = (360.0-templateRotation_);
  correctPoint.header.stamp = pubTime;
  correctPointPublisher_.publish(correctPoint);

  ros::Duration duration = ros::Time::now() - time;
  std::cout << "Best NCC " << bestNCC << " at " << bestXNCC << ", " << bestYNCC << " and theta " << bestThetaNCC << " and z: " << z << std::endl;
  std::cout << "Best SSD " << bestSSD << " at " << bestXSSD << ", " << bestYSSD << " and theta " << bestThetaSSD << std::endl;
  std::cout << "Best SAD " << bestSAD << " at " << bestXSAD << ", " << bestYSAD << " and theta " << bestThetaSAD << std::endl;
  //std::cout << "Best MI " << bestMI << " at " << bestXMI << ", " << bestYMI << " and theta " << bestThetaMI << std::endl;
  std::cout << "Correct position " << correct_position_.transpose() << " and theta " << (360.0-templateRotation_) << std::endl;
  std::cout << "Time used: " << duration.toSec() << " Sekunden" << " 1: " << duration1_.toSec() << " 2: " << duration2_.toSec() << std::endl;
  std::cout << "Cumulative error NCC: " << cumulativeErrorCorr_ << " matches: " << correctMatchesCorr_ << " SSD: " << cumulativeErrorSSD_ << " matches: " << correctMatchesSSD_ << " SAD: " << cumulativeErrorSAD_ << " matches: " << correctMatchesSAD_ << " MI: " << cumulativeErrorMI_ << " matches: " << correctMatchesMI_ << std::endl;

  //update & resample particles
  std::vector<float> beta;
  beta.clear();
  for (int i = 0; i < numberOfParticles_; i++)
  {
      if (NCC[i] >= 0.75*bestNCC) { beta.push_back(NCC[i]); }   //threshold for acceptance?
      else { beta.push_back(0.0); }
  }
  float sum = std::accumulate(beta.begin(), beta.end(), 0.0);
  if (sum == 0.0 && bestNCC != -1)                            // fix for second dataset with empty template update
  {
    particleRow_.clear();
    particleCol_.clear();
    particleTheta_.clear();
    isFirst_ = true;
    ROS_INFO("particle Filter reinitialized");
  }
  else if(sum != 0.0)
  {
    std::transform(beta.begin(), beta.end(), beta.begin(), std::bind1st(std::multiplies<float>(), 1.0/sum));
    std::partial_sum(beta.begin(), beta.end(), beta.begin());

    std::vector< std::vector<int> > newParticles;
    newParticles.clear();

    if (numberOfParticles_ < 4000) { numberOfParticles_ = 4000; }
    for (int i = 0; i < numberOfParticles_; i++)
    {
      float randNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * (beta.back()-beta[0]) + beta[0];
      int ind = std::upper_bound(beta.begin(), beta.end(), randNumber) - beta.begin() -1;

      std::vector<int> particle;
      particle.clear();

      particle.push_back( int(particleRow_[ind] + round(distribution(generator_)) + rows) % rows );
      particle.push_back( int(particleCol_[ind] + round(distribution(generator_)) + cols) % cols );
      particle.push_back( int( particleTheta_[ind] + round(distribution(generator_)) + 360) % 360);
      newParticles.push_back(particle);
    }
    
    std::sort(newParticles.begin(), newParticles.end());
    auto last = std::unique(newParticles.begin(), newParticles.end());
    newParticles.erase(last, newParticles.end());

    numberOfParticles_ = newParticles.size();
    particleRow_.clear();
    particleCol_.clear();
    particleTheta_.clear();

    for (int i = 0; i < numberOfParticles_; i++)
    {
      particleRow_.push_back(newParticles[i][0]);
      particleCol_.push_back(newParticles[i][1]);
      particleTheta_.push_back(newParticles[i][2]);
    }
  }


  std::cout <<"New number of particles: " << numberOfParticles_ << std::endl;
  ROS_INFO("done");
  isActive_ = false;
}

float MapFitter::findZ(grid_map::Matrix& data, grid_map::Matrix& reference_data, float x, float y, int theta)
{
  grid_map::Index reference_index;
  referenceMap_.getIndex(grid_map::Position(x,y), reference_index);
  float sin_theta = sin((theta+templateRotation_)/180*M_PI);
  float cos_theta = cos((theta+templateRotation_)/180*M_PI);

  // initialize
  float shifted_mean = 0;
  float reference_mean = 0;
  int matches = 0;

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

  for (int i = 0; i < size_x; i ++)
  {
    for (int j = 0; j< size_y; j ++)
    {
      int index_x = (start_index_x + i) % size_x;
      int index_y = (start_index_y + j) % size_y;

      float mapHeight = data(index_x, index_y);
      if (mapHeight == mapHeight)
      {
        int reference_buffer_index_x = reference_size_x - reference_start_index_x + reference_index_x;
        int reference_buffer_index_y = reference_size_y - reference_start_index_y + reference_index_y;

        int shifted_index_x = reference_buffer_index_x % reference_size_x - round(cos_theta*(float(size_x)/2-i) - sin_theta*(float(size_y)/2-j));
        int shifted_index_y = reference_buffer_index_y % reference_size_y - round(sin_theta*(float(size_x)/2-i) + cos_theta*(float(size_y)/2-j));
              
        if (shifted_index_x >= 0 && shifted_index_x < reference_size_x && shifted_index_y >= 0 && shifted_index_y < reference_size_y )
        {
          shifted_index_x = (shifted_index_x + reference_start_index_x) % reference_size_x;
          shifted_index_y = (shifted_index_y + reference_start_index_y) % reference_size_y;
          float referenceHeight = reference_data(shifted_index_x, shifted_index_y);
          //std::cout << referenceHeight << " " << shifted_index_x <<", " << shifted_index_y << std::endl;
          if (referenceHeight == referenceHeight)
          {
            matches += 1;
            shifted_mean += mapHeight;
            reference_mean += referenceHeight;
          }
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
  ros::Time time1 = ros::Time::now();
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
      if (mapHeight == mapHeight)
      {
        points += 1;
        int reference_buffer_index_x = reference_size_x - reference_start_index_x + reference_index_x;
        int reference_buffer_index_y = reference_size_y - reference_start_index_y + reference_index_y;

        int shifted_index_x = reference_buffer_index_x % reference_size_x - round(cos_theta*(float(size_x)/2-i) - sin_theta*(float(size_y)/2-j));
        int shifted_index_y = reference_buffer_index_y % reference_size_y - round(sin_theta*(float(size_x)/2-i) + cos_theta*(float(size_y)/2-j));
              
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
      
    }
  }

 duration1_ += ros::Time::now() - time1;
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
  if ((reference_min_ - reference_mean_) < minHeight) { minHeight = reference_min_ - reference_mean_; }

  float maxHeight = map_max_ - shifted_mean_;
  if ((reference_max_ - reference_mean_) > maxHeight) { maxHeight = reference_max_ - reference_mean_; }

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
    if (hist[i]!=0.0) {entropy += -hist[i]*log(hist[i]);}
    if (referenceHist[i]!=0.0) {referenceEntropy += -referenceHist[i]*log(referenceHist[i]);}

    for (int j = 0; j < numberOfBins; j++)
    {
      if (jointHist[i][j]!=0.0) {jointEntropy += -jointHist[i][j]*log(jointHist[i][j]);}
    }
  }

  //std::cout << " template entropy: " << entropy << " reference entropy: " << referenceEntropy << " joint entropy: " << jointEntropy << " Mutual information: " << entropy+referenceEntropy-jointEntropy <<std::endl;
  return (entropy+referenceEntropy-jointEntropy);
}

float MapFitter::weightedMutualInformation()
{
  const int numberOfBins = 256;

  float minHeight = map_min_ - shifted_mean_;
  if ((reference_min_ - reference_mean_) < minHeight) { minHeight = reference_min_ - reference_mean_; }

  float maxHeight = map_max_ - shifted_mean_;
  if ((reference_max_ - reference_mean_) > maxHeight) { maxHeight = reference_max_ - reference_mean_; }

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
    if (hist[i]!=0.0) { entropy += -hist[i]*log(hist[i]); }
    if (referenceHist[i]!=0.0) { referenceEntropy += -referenceHist[i]*log(referenceHist[i]); }

    for (int j = 0; j < numberOfBins; j++)
    {
      if (jointHist[i][j]!=0.0) { jointEntropy += -float(abs(j-i)+1)/12*jointHist[i][j]*log(jointHist[i][j]); }
    }
  }

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
    normalization += xy_shifted_var_[i];// * (xy_reference_var_[i]
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
    normalization += xy_shifted_var_[i]*xy_shifted_var_[i];// * (xy_reference_var_[i]
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
  }
  return correlation/sqrt(shifted_normal*reference_normal);
}

void MapFitter::tfBroadcast(const ros::TimerEvent&) 
{
  broadcaster_.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
          ros::Time::now(),"/map", "/grid_map"));
}

} /* namespace */

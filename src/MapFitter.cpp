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
    : nodeHandle_(nodeHandle), isActive_(false), initializeSAD_(false), initializeSSD_(false), initializeNCC_(false), initializeMI_(false)
{
  ROS_INFO("Map fitter node started, ready to match some grid maps.");
  readParameters();
  initialization();
  shiftedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(shiftedMapTopic_,1);   // publisher for shifted_map
  correlationPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(correlationMapTopic_,1);    // publisher for correlation_map
  referencePublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/uav_elevation_mapping/uav_elevation_map",1); // change back to referenceMapTopic_
  activityCheckTimer_ = nodeHandle_.createTimer(activityCheckDuration_, &MapFitter::updateSubscriptionCallback, this);
  broadcastTimer_ = nodeHandle_.createTimer(ros::Duration(0.01), &MapFitter::tfBroadcast, this);
  listenerTimer_  = nodeHandle_.createTimer(ros::Duration(0.01), &MapFitter::tfListener, this);
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
  set_ = "set1";
  weighted_ = true;
  resample_ = true;

  SAD_ = true;
  SSD_ = true;
  NCC_ = true;
  MI_ = true;

  nodeHandle_.param("map_topic", mapTopic_, std::string("/elevation_mapping_long_range/elevation_map"));
  if (set_ == "set1") { nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/uav_elevation_mapping/uav_elevation_map")); }
  if (set_ == "set2") { nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/elevation_mapping/elevation_map")); }
  nodeHandle_.param("shifted_map_topic", shiftedMapTopic_, std::string("/elevation_mapping_long_range/shifted_map"));
  nodeHandle_.param("correlation_map_topic", correlationMapTopic_, std::string("/correlation_best_rotation/correlation_map"));

  nodeHandle_.param("angle_increment", angleIncrement_, 5);
  nodeHandle_.param("position_increment_search", searchIncrement_, 5);
  nodeHandle_.param("position_increment_correlation", correlationIncrement_, 5);
  nodeHandle_.param("required_overlap", requiredOverlap_, float(0.25));
  if (weighted_)
  {
    nodeHandle_.param("SAD_threshold", SADThreshold_, float(0.05));
    nodeHandle_.param("SSD_threshold", SSDThreshold_, float(0.008));
    nodeHandle_.param("NCC_threshold", NCCThreshold_, float(0.65));
    nodeHandle_.param("MI_threshold", MIThreshold_, float(0));
  }
  else
  {
    nodeHandle_.param("SAD_threshold", SADThreshold_, float(10));
    nodeHandle_.param("SSD_threshold", SSDThreshold_, float(10));
    nodeHandle_.param("NCC_threshold", NCCThreshold_, float(0.65));
    nodeHandle_.param("MI_threshold", MIThreshold_, float(0));
  }
  double activityCheckRate;
  nodeHandle_.param("activity_check_rate", activityCheckRate, 1.0);
  activityCheckDuration_.fromSec(1.0 / activityCheckRate);
}

bool MapFitter::initialization()
{
  if (SAD_ == true) { initializeSAD_ = true; }
  if (SSD_ == true) { initializeSSD_ = true; }
  if (NCC_ == true) { initializeNCC_ = true; }
  if (MI_ == true) { initializeMI_ = true; }

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
  ROS_INFO("Map fitter received a map (timestamp %f) for matching.", message.info.header.stamp.toSec());
  grid_map::GridMapRosConverter::fromMessage(message, map_);

  grid_map::Index submap_start_index;
  grid_map::Size submap_size;
  if (set_ == "set1")
  { 
    grid_map::GridMapRosConverter::loadFromBag("/home/parallels/rosbags/reference_map_last.bag", referenceMapTopic_, referenceMap_);
    referenceMap_.move(grid_map::Position(2.75,1));

    //grid_map::GridMap extendMap;
    //extendMap.setGeometry(grid_map::Length(10.5,7.5), referenceMap_.getResolution(), referenceMap_.getPosition());
    //extendMap.setFrameId("grid_map");
    //referenceMap_.extendToInclude(extendMap);

    referenceMap_.getDataBoundingSubmap("elevation", submap_start_index, submap_size);

    grid_map::Size reference_size = referenceMap_.getSize();
    grid_map::Index reference_start_index = referenceMap_.getStartIndex();

    for (grid_map::GridMapIterator iterator(referenceMap_); !iterator.isPastEnd(); ++iterator) 
    {
      grid_map::Index index(*iterator);
      grid_map::Index shaped_index = grid_map::getIndexFromBufferIndex(index, reference_size, reference_start_index);
      grid_map::Index shaped_submap_start_index = grid_map::getIndexFromBufferIndex(submap_start_index, reference_size, reference_start_index);
      bool outside_submap = (shaped_index(0) < shaped_submap_start_index(0)-0 || shaped_index(1) < shaped_submap_start_index(1)-0 || shaped_index(0) > shaped_submap_start_index(0)+submap_size(0)+0 || shaped_index(1) > shaped_submap_start_index(1)+submap_size(1)+0);
      if ( outside_submap )
      {
        referenceMap_.at("elevation", index) = -0.75; //static_cast <float> (rand()) / static_cast <float> (RAND_MAX) -1;
      }
    }
  }
  
  if (set_ == "set2") 
  { 
    grid_map::GridMapRosConverter::loadFromBag("/home/parallels/rosbags/source/asl_walking_uav/uav_reference_map.bag", referenceMapTopic_, referenceMap_); 
    
    referenceMap_.getDataBoundingSubmap("elevation", submap_start_index, submap_size);
  }

  exhaustiveSearch(submap_start_index, submap_size);
}

void MapFitter::exhaustiveSearch(grid_map::Index submap_start_index, grid_map::Size submap_size)
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
  float resolution = referenceMap_.getResolution();
  int subresolution = resolution / 0.01;

  grid_map::Position previous_position = map_position_;
  map_position_ = map_.getPosition();
  grid_map::Position position = referenceMap_.getPosition();
  grid_map::Size size = map_.getSize();
  grid_map::Index start_index = map_.getStartIndex();
  grid_map::Index reference_start_index = referenceMap_.getStartIndex();

  grid_map::Matrix& reference_data = referenceMap_["elevation"];
  grid_map::Matrix& data = map_["elevation"];
  grid_map::Matrix& variance_data = map_["variance"];

  grid_map_msgs::GridMap reference_msg;
  grid_map::GridMapRosConverter::toMessage(referenceMap_, reference_msg);
  referencePublisher_.publish(reference_msg);

  tf::StampedTransform correct_position;
  try { listener_.lookupTransform("/map", "/footprint", ros::Time(0), correct_position); }
  catch (tf::TransformException ex) { ROS_ERROR("%s",ex.what()); }
  tf::Matrix3x3 m(correct_position.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  float previous_templateRotation = templateRotation_;
  templateRotation_ = 360 - fmod(yaw/M_PI*180+360,360);
  grid_map::Position shift = grid_map::Position(map_position_(0)-correct_position.getOrigin().x(), map_position_(1)-correct_position.getOrigin().y() );

  ros::Time time = ros::Time::now();
  duration1_.sec = 0;
  duration1_.nsec = 0;
  duration2_.sec = 0;
  duration2_.nsec = 0;

  std::normal_distribution<float> distribution(0.0,2.0*subresolution);

  // initialize particles
  if (initializeSAD_ || initializeSSD_ || initializeNCC_ || initializeMI_)
  {
    int numberOfParticles = 0;
    for (float theta = 0; theta < 360; theta += angleIncrement_)
    {
      for (grid_map::SubmapIteratorSparse iterator(referenceMap_, submap_start_index, submap_size, searchIncrement_); !iterator.isPastEnd(); ++iterator) 
      {
        grid_map::Index index(*iterator);
      //grid_map::Index index;
      //referenceMap_.getIndex(map_position_, index);
        if (initializeSAD_)
        {
          particleRowSAD_.push_back(index(0)*subresolution);
          particleColSAD_.push_back(index(1)*subresolution);
          particleThetaSAD_.push_back(theta);
        }
        if (initializeSSD_)
        {
          particleRowSSD_.push_back(index(0)*subresolution);
          particleColSSD_.push_back(index(1)*subresolution);
          particleThetaSSD_.push_back(theta);
        }
        if (initializeNCC_)
        {
          particleRowNCC_.push_back(index(0)*subresolution);
          particleColNCC_.push_back(index(1)*subresolution);
          particleThetaNCC_.push_back(theta);
        }
        if (initializeMI_)
        {
          particleRowMI_.push_back(index(0)*subresolution);
          particleColMI_.push_back(index(1)*subresolution);
          particleThetaMI_.push_back(theta);
        }
        numberOfParticles += 1;
      }
    }
    //templateRotation_ = static_cast <float> (rand() / static_cast <float> (RAND_MAX/360)); //rand() %360;

    if (initializeSAD_) { std::cout <<"Number of particles SAD: " << numberOfParticles << std::endl; }
    if (initializeSSD_) { std::cout <<"Number of particles SSD: " << numberOfParticles << std::endl; }
    if (initializeNCC_) { std::cout <<"Number of particles NCC: " << numberOfParticles << std::endl; }
    if (initializeMI_) { std::cout <<"Number of particles MI: " << numberOfParticles << std::endl; }

    initializeSAD_ = false;
    initializeSSD_ = false;
    initializeNCC_ = false;
    initializeMI_ = false;
  }
  else
  {
    if (SAD_)
    {
      std::transform(particleRowSAD_.begin(), particleRowSAD_.end(), particleRowSAD_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(0) - previous_position(0)) / resolution + rows)*subresolution + distribution(generator_)/2) ));
      std::transform(particleRowSAD_.begin(), particleRowSAD_.end(), particleRowSAD_.begin(), std::bind2nd(std::modulus<int>(), rows*subresolution));
      std::transform(particleColSAD_.begin(), particleColSAD_.end(), particleColSAD_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(1) - previous_position(1)) / resolution + cols)*subresolution + distribution(generator_)/2) ));
      std::transform(particleColSAD_.begin(), particleColSAD_.end(), particleColSAD_.begin(), std::bind2nd(std::modulus<int>(), cols*subresolution));
      std::transform(particleThetaSAD_.begin(), particleThetaSAD_.end(), particleThetaSAD_.begin(), std::bind2nd(std::plus<int>(), round( -(templateRotation_ - previous_templateRotation) + 360 + distribution(generator_)/2) ));
      std::transform(particleThetaSAD_.begin(), particleThetaSAD_.end(), particleThetaSAD_.begin(), std::bind2nd(std::modulus<int>(), 360));
    }
    if (SSD_)
    {
      std::transform(particleRowSSD_.begin(), particleRowSSD_.end(), particleRowSSD_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(0) - previous_position(0)) / resolution + rows)*subresolution + distribution(generator_)/2) ));
      std::transform(particleRowSSD_.begin(), particleRowSSD_.end(), particleRowSSD_.begin(), std::bind2nd(std::modulus<int>(), rows*subresolution));
      std::transform(particleColSSD_.begin(), particleColSSD_.end(), particleColSSD_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(1) - previous_position(1)) / resolution + cols)*subresolution + distribution(generator_)/2) ));
      std::transform(particleColSSD_.begin(), particleColSSD_.end(), particleColSSD_.begin(), std::bind2nd(std::modulus<int>(), cols*subresolution));
      std::transform(particleThetaSSD_.begin(), particleThetaSSD_.end(), particleThetaSSD_.begin(), std::bind2nd(std::plus<int>(), round( -(templateRotation_ - previous_templateRotation) + 360 + distribution(generator_)/2) ));
      std::transform(particleThetaSSD_.begin(), particleThetaSSD_.end(), particleThetaSSD_.begin(), std::bind2nd(std::modulus<int>(), 360));
    }
    if (NCC_)
    {
      std::transform(particleRowNCC_.begin(), particleRowNCC_.end(), particleRowNCC_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(0) - previous_position(0)) / resolution + rows)*subresolution + distribution(generator_)/2) ));
      std::transform(particleRowNCC_.begin(), particleRowNCC_.end(), particleRowNCC_.begin(), std::bind2nd(std::modulus<int>(), rows*subresolution));
      std::transform(particleColNCC_.begin(), particleColNCC_.end(), particleColNCC_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(1) - previous_position(1)) / resolution + cols)*subresolution + distribution(generator_)/2) ));
      std::transform(particleColNCC_.begin(), particleColNCC_.end(), particleColNCC_.begin(), std::bind2nd(std::modulus<int>(), cols*subresolution));
      std::transform(particleThetaNCC_.begin(), particleThetaNCC_.end(), particleThetaNCC_.begin(), std::bind2nd(std::plus<int>(), round( -(templateRotation_ - previous_templateRotation) + 360 + distribution(generator_)/2) ));
      std::transform(particleThetaNCC_.begin(), particleThetaNCC_.end(), particleThetaNCC_.begin(), std::bind2nd(std::modulus<int>(), 360));
    }
    if (MI_)
    {
      std::transform(particleRowMI_.begin(), particleRowMI_.end(), particleRowMI_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(0) - previous_position(0)) / resolution + rows)*subresolution + distribution(generator_)/2) ));
      std::transform(particleRowMI_.begin(), particleRowMI_.end(), particleRowMI_.begin(), std::bind2nd(std::modulus<int>(), rows*subresolution));
      std::transform(particleColMI_.begin(), particleColMI_.end(), particleColMI_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(1) - previous_position(1)) / resolution + cols)*subresolution + distribution(generator_)/2) ));
      std::transform(particleColMI_.begin(), particleColMI_.end(), particleColMI_.begin(), std::bind2nd(std::modulus<int>(), cols*subresolution));
      std::transform(particleThetaMI_.begin(), particleThetaMI_.end(), particleThetaMI_.begin(), std::bind2nd(std::plus<int>(), round( -(templateRotation_ - previous_templateRotation) + 360 + distribution(generator_)/2) ));
      std::transform(particleThetaMI_.begin(), particleThetaMI_.end(), particleThetaMI_.begin(), std::bind2nd(std::modulus<int>(), 360));
    }
  }

  ros::Time pubTime = ros::Time::now();
  if (SAD_)
  {
    std::vector<float> SAD;
    SAD.clear();

    for (int i = 0; i < particleRowSAD_.size(); i++)
    {
      float row = float(particleRowSAD_[i])/subresolution;
      float col = float(particleColSAD_[i])/subresolution;
      grid_map::Index index = grid_map::Index(int(round(row)), int(round(col)));
      int theta = particleThetaSAD_[i];

      float sin_theta = sin((theta+templateRotation_)/180*M_PI);
      float cos_theta = cos((theta+templateRotation_)/180*M_PI);

      bool success = findMatches(data, variance_data, reference_data, row, col, sin_theta, cos_theta );
      if (success) 
      {
        float errSAD;
        if (!weighted_) { errSAD = errorSAD(); }
        else { errSAD = weightedErrorSAD(); }

        SAD.push_back(errSAD);
 
        grid_map::Position xy_position;
        referenceMap_.getPosition(index, xy_position);
        if (correlationMap.isInside(xy_position))
        {
          grid_map::Index correlation_index;
          correlationMap.getIndex(xy_position, correlation_index);

          bool valid = correlationMap.isValid(correlation_index, "SAD");
          // if no value so far or correlation smaller or correlation higher than for other thetas
          if (((valid == false) || (errSAD*10 < correlationMap.at("SAD", correlation_index) ))) 
          {
            correlationMap.at("SAD", correlation_index) = errSAD*10;  //set correlation
            correlationMap.at("rotationSAD", correlation_index) = theta;    //set theta
          }
        }
      }
      else { SAD.push_back(10); }
    }
      // search best score 
    grid_map::Position best_pos;

    std::vector<float>::iterator SADit = std::min_element(SAD.begin(), SAD.end());
    int bestSADparticle = std::distance(SAD.begin(), SADit);
    float bestSAD = SAD[bestSADparticle];
    //referenceMap_.getPosition(grid_map::Index(particleRowSAD_[bestSADparticle], particleColSAD_[bestSADparticle]), best_pos);
    //float bestXSAD = best_pos(0);
    //float bestYSAD = best_pos(1);
    int bestRow = int(round(particleRowSAD_[bestSADparticle]/subresolution)) % rows;
    int bestCol = int(round(particleColSAD_[bestSADparticle]/subresolution)) % cols;
    referenceMap_.getPosition(grid_map::Index(bestRow, bestCol), best_pos);
    float bestXSAD = best_pos(0) - ( particleRowSAD_[bestSADparticle]-int(round(particleRowSAD_[bestSADparticle])) )*referenceMap_.getResolution()/subresolution; 
    float bestYSAD = best_pos(1) - ( particleColSAD_[bestSADparticle]-int(round(particleColSAD_[bestSADparticle])) )*referenceMap_.getResolution()/subresolution;
    int bestThetaSAD = particleThetaSAD_[bestSADparticle];

    // Calculate z alignement
    float z = findZ(data, reference_data, bestXSAD, bestYSAD, bestThetaSAD);
    if (bestSAD != 10) 
    {
      cumulativeErrorSAD_ += sqrt((bestXSAD - map_position_(0))*(bestXSAD - map_position_(0)) + (bestYSAD - map_position_(1))*(bestYSAD - map_position_(1)));
      if (sqrt((bestXSAD - map_position_(0))*(bestXSAD - map_position_(0)) + (bestYSAD - map_position_(1))*(bestYSAD - map_position_(1))) < 0.5 && (fabs(bestThetaSAD - (360-templateRotation_)) < angleIncrement_ || fabs(bestThetaSAD - (360-templateRotation_)) > 360-angleIncrement_)) {correctMatchesSAD_ += 1;}
      geometry_msgs::PointStamped SADPoint;
      SADPoint.point.x = bestXSAD - shift(0);
      SADPoint.point.y = bestYSAD - shift(1);
      SADPoint.point.z = bestThetaSAD;
      SADPoint.header.stamp = pubTime;
      SADPointPublisher_.publish(SADPoint);
    }
    std::cout << "Best SAD " << bestSAD << " at " << bestXSAD << ", " << bestYSAD << " , theta " << bestThetaSAD << " and z: " << z << std::endl;
    std::cout << "Cumulative error SAD: " << cumulativeErrorSAD_ << " matches: " << correctMatchesSAD_ << std::endl;

    if (resample_)
    {
      //update & resample particles
      std::vector<float> beta;
      beta.clear();
      for (int i = 0; i < particleRowSAD_.size(); i++)
      {
          if (SAD[i] <= 1.25*bestSAD) { beta.push_back(1.25*bestSAD-SAD[i]); }
          else { beta.push_back(0.0); }
      }
      float sum = std::accumulate(beta.begin(), beta.end(), 0.0);
      if ((sum == 0.0 || bestSAD > SADThreshold_) && bestSAD != 10)                           // fix for second dataset with empty template update
      {
        particleRowSAD_.clear();
        particleColSAD_.clear();
        particleThetaSAD_.clear();
        initializeSAD_ = true;
        ROS_INFO("particle Filter SAD reinitialized");
      }
      else if(sum != 0.0)
      {
        std::transform(beta.begin(), beta.end(), beta.begin(), std::bind1st(std::multiplies<float>(), 1.0/sum));
        std::partial_sum(beta.begin(), beta.end(), beta.begin());

        std::vector< std::vector<int> > newParticles;
        newParticles.clear();
        int numberOfParticles = particleRowSAD_.size();
        if (numberOfParticles < 4000) { numberOfParticles = 4000; }
        for (int i = 0; i < numberOfParticles; i++)
        {
          float randNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * (beta.back()-beta[0]) + beta[0];
          int ind = std::upper_bound(beta.begin(), beta.end(), randNumber) - beta.begin() -1;

          std::vector<int> particle;
          particle.clear();

          particle.push_back( int(particleRowSAD_[ind] + round(distribution(generator_)) + rows*subresolution) % (rows*subresolution) );
          particle.push_back( int(particleColSAD_[ind] + round(distribution(generator_)) + cols*subresolution) % (cols*subresolution) );
          particle.push_back( int(particleThetaSAD_[ind] + round(distribution(generator_)) + 360) % 360);
          newParticles.push_back(particle);
        }
        
        std::sort(newParticles.begin(), newParticles.end());
        auto last = std::unique(newParticles.begin(), newParticles.end());
        newParticles.erase(last, newParticles.end());

        numberOfParticles = newParticles.size();
        particleRowSAD_.clear();
        particleColSAD_.clear();
        particleThetaSAD_.clear();

        for (int i = 0; i < numberOfParticles; i++)
        {
          particleRowSAD_.push_back(newParticles[i][0]);
          particleColSAD_.push_back(newParticles[i][1]);
          particleThetaSAD_.push_back(newParticles[i][2]);
        }
        std::cout <<"New number of particles SAD: " << numberOfParticles << std::endl;
      }
    }
  }

  if (SSD_)
  {
    std::vector<float> SSD;
    SSD.clear();

    for (int i = 0; i < particleRowSSD_.size(); i++)
    {
      float row = float(particleRowSSD_[i])/subresolution;
      float col = float(particleColSSD_[i])/subresolution;
      grid_map::Index index = grid_map::Index(int(round(row)), int(round(col)));
      int theta = particleThetaSSD_[i];

      float sin_theta = sin((theta+templateRotation_)/180*M_PI);
      float cos_theta = cos((theta+templateRotation_)/180*M_PI);

      bool success = findMatches(data, variance_data, reference_data, row, col, sin_theta, cos_theta );
      if (success) 
      {
        float errSSD;
        if (!weighted_) { errSSD = errorSSD(); }
        else { errSSD = weightedErrorSSD(); }

        SSD.push_back(errSSD);

        grid_map::Position xy_position;
        referenceMap_.getPosition(index, xy_position);
        if (correlationMap.isInside(xy_position))
        {
          grid_map::Index correlation_index;
          correlationMap.getIndex(xy_position, correlation_index);

          bool valid = correlationMap.isValid(correlation_index, "SSD");
          // if no value so far or correlation smaller or correlation higher than for other thetas
          if (((valid == false) || (errSSD*50 < correlationMap.at("SSD", correlation_index) ))) 
          {
            correlationMap.at("SSD", correlation_index) = errSSD*50;  //set correlation
            correlationMap.at("rotationSSD", correlation_index) = theta;    //set theta
          }
        }
      }
      else { SSD.push_back(10); }
    }
      // search best score 
    grid_map::Position best_pos;

    std::vector<float>::iterator SSDit = std::min_element(SSD.begin(), SSD.end());
    int bestSSDparticle = std::distance(SSD.begin(), SSDit);
    float bestSSD = SSD[bestSSDparticle];
    //referenceMap_.getPosition(grid_map::Index(particleRowSSD_[bestSSDparticle], particleColSSD_[bestSSDparticle]), best_pos);
    //float bestXSSD = best_pos(0);
    //float bestYSSD = best_pos(1);
    int bestRow = int(round(particleRowSSD_[bestSSDparticle]/subresolution)) % rows;
    int bestCol = int(round(particleColSSD_[bestSSDparticle]/subresolution)) % cols;
    referenceMap_.getPosition(grid_map::Index(bestRow, bestCol), best_pos);
    float bestXSSD = best_pos(0) - ( particleRowSSD_[bestSSDparticle]-int(round(particleRowSSD_[bestSSDparticle])) )*referenceMap_.getResolution()/subresolution; 
    float bestYSSD = best_pos(1) - ( particleColSSD_[bestSSDparticle]-int(round(particleColSSD_[bestSSDparticle])) )*referenceMap_.getResolution()/subresolution;
    int bestThetaSSD = particleThetaSSD_[bestSSDparticle];

    // Calculate z alignement
    float z = findZ(data, reference_data, bestXSSD, bestYSSD, bestThetaSSD);
    if (bestSSD != 10) 
    {
      cumulativeErrorSSD_ += sqrt((bestXSSD - map_position_(0))*(bestXSSD - map_position_(0)) + (bestYSSD - map_position_(1))*(bestYSSD - map_position_(1)));
      if (sqrt((bestXSSD - map_position_(0))*(bestXSSD - map_position_(0)) + (bestYSSD - map_position_(1))*(bestYSSD - map_position_(1))) < 0.5 && (fabs(bestThetaSSD - (360-templateRotation_)) < angleIncrement_ || fabs(bestThetaSSD - (360-templateRotation_)) > 360-angleIncrement_)) {correctMatchesSSD_ += 1;}
      geometry_msgs::PointStamped SSDPoint;
      SSDPoint.point.x = bestXSSD - shift(0);
      SSDPoint.point.y = bestYSSD - shift(1);
      SSDPoint.point.z = bestThetaSSD;
      SSDPoint.header.stamp = pubTime;
      SSDPointPublisher_.publish(SSDPoint);
    }
    std::cout << "Best SSD " << bestSSD << " at " << bestXSSD << ", " << bestYSSD << " , theta " << bestThetaSSD << " and z: " << z << std::endl;
    std::cout << "Cumulative error SSD: " << cumulativeErrorSSD_ << " matches: " << correctMatchesSSD_ << std::endl;

    if (resample_)
    {
      //update & resample particles
      std::vector<float> beta;
      beta.clear();
      for (int i = 0; i < particleRowSSD_.size(); i++)
      {
          if (SSD[i] <= 1.25*bestSSD) { beta.push_back(1.25*bestSSD-SSD[i]); }
          else { beta.push_back(0.0); }
      }
      float sum = std::accumulate(beta.begin(), beta.end(), 0.0);
      if ((sum == 0.0 || bestSSD > SSDThreshold_) && bestSSD != 10)                            // fix for second dataset with empty template update
      {
        particleRowSSD_.clear();
        particleColSSD_.clear();
        particleThetaSSD_.clear();
        initializeSSD_ = true;
        ROS_INFO("particle Filter SSD reinitialized");
      }
      else if(sum != 0.0)
      {
        std::transform(beta.begin(), beta.end(), beta.begin(), std::bind1st(std::multiplies<float>(), 1.0/sum));
        std::partial_sum(beta.begin(), beta.end(), beta.begin());

        std::vector< std::vector<int> > newParticles;
        newParticles.clear();
        int numberOfParticles = particleRowSSD_.size();
        if (numberOfParticles < 4000) { numberOfParticles = 4000; }
        for (int i = 0; i < numberOfParticles; i++)
        {
          float randNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * (beta.back()-beta[0]) + beta[0];
          int ind = std::upper_bound(beta.begin(), beta.end(), randNumber) - beta.begin() -1;

          std::vector<int> particle;
          particle.clear();

          particle.push_back( int(particleRowSSD_[ind] + round(distribution(generator_)) + rows*subresolution) % (rows*subresolution) );
          particle.push_back( int(particleColSSD_[ind] + round(distribution(generator_)) + cols*subresolution) % (cols*subresolution) );
          particle.push_back( int(particleThetaSSD_[ind] + round(distribution(generator_)) + 360) % 360);
          newParticles.push_back(particle);
        }
        
        std::sort(newParticles.begin(), newParticles.end());
        auto last = std::unique(newParticles.begin(), newParticles.end());
        newParticles.erase(last, newParticles.end());

        numberOfParticles = newParticles.size();
        particleRowSSD_.clear();
        particleColSSD_.clear();
        particleThetaSSD_.clear();

        for (int i = 0; i < numberOfParticles; i++)
        {
          particleRowSSD_.push_back(newParticles[i][0]);
          particleColSSD_.push_back(newParticles[i][1]);
          particleThetaSSD_.push_back(newParticles[i][2]);
        }
        std::cout <<"New number of particles SSD: " << numberOfParticles << std::endl;
      }
    }
  }

  if (NCC_)
  {
    std::vector<float> NCC;
    NCC.clear();

    for (int i = 0; i < particleRowNCC_.size(); i++)
    {
      float row = float(particleRowNCC_[i])/subresolution;
      float col = float(particleColNCC_[i])/subresolution;
      grid_map::Index index = grid_map::Index(int(round(row)), int(round(col)));
      int theta = particleThetaNCC_[i];

      float sin_theta = sin((theta+templateRotation_)/180*M_PI);
      float cos_theta = cos((theta+templateRotation_)/180*M_PI);

      bool success = findMatches(data, variance_data, reference_data, row, col, sin_theta, cos_theta );
      if (success) 
      {
        float corrNCC;
        if (!weighted_) { corrNCC = correlationNCC(); }
        else { corrNCC = weightedCorrelationNCC(); }

        NCC.push_back(corrNCC);

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
        }
      }
      else { NCC.push_back(-1); }
    }
      // search best score 
    grid_map::Position best_pos;

    std::vector<float>::iterator NCCit = std::max_element(NCC.begin(), NCC.end());
    int bestNCCparticle = std::distance(NCC.begin(), NCCit);
    float bestNCC = NCC[bestNCCparticle];
    //referenceMap_.getPosition(grid_map::Index(particleRowNCC_[bestNCCparticle], particleColNCC_[bestNCCparticle]), best_pos);
    //float bestXNCC = best_pos(0);
    //float bestYNCC = best_pos(1);
    int bestRow = int(round(particleRowNCC_[bestNCCparticle]/subresolution)) % rows;
    int bestCol = int(round(particleColNCC_[bestNCCparticle]/subresolution)) % cols;
    referenceMap_.getPosition(grid_map::Index(bestRow, bestCol), best_pos);
    float bestXNCC = best_pos(0) - ( particleRowNCC_[bestNCCparticle]-int(round(particleRowNCC_[bestNCCparticle])) )*referenceMap_.getResolution()/subresolution; 
    float bestYNCC = best_pos(1) - ( particleColNCC_[bestNCCparticle]-int(round(particleColNCC_[bestNCCparticle])) )*referenceMap_.getResolution()/subresolution;
    int bestThetaNCC = particleThetaNCC_[bestNCCparticle];

    // Calculate z alignement
    float z = findZ(data, reference_data, bestXNCC, bestYNCC, bestThetaNCC);
    if (bestNCC != -1) 
    {
      cumulativeErrorCorr_ += sqrt((bestXNCC - map_position_(0))*(bestXNCC - map_position_(0)) + (bestYNCC - map_position_(1))*(bestYNCC - map_position_(1)));
      if (sqrt((bestXNCC - map_position_(0))*(bestXNCC - map_position_(0)) + (bestYNCC - map_position_(1))*(bestYNCC - map_position_(1))) < 0.5 && (fabs(bestThetaNCC - (360-templateRotation_)) < angleIncrement_ || fabs(bestThetaNCC - (360-templateRotation_)) > 360-angleIncrement_)) {correctMatchesCorr_ += 1;}
      geometry_msgs::PointStamped corrPoint;
      corrPoint.point.x = bestXNCC - shift(0);
      corrPoint.point.y = bestYNCC - shift(1);
      corrPoint.point.z = bestThetaNCC;
      corrPoint.header.stamp = pubTime;
      corrPointPublisher_.publish(corrPoint);
    }
    std::cout << "Best NCC " << bestNCC << " at " << bestXNCC << ", " << bestYNCC << " , theta " << bestThetaNCC << " and z: " << z << std::endl;
    std::cout << "Cumulative error NCC: " << cumulativeErrorCorr_ << " matches: " << correctMatchesCorr_ << std::endl;

    if (resample_)
    {
      //update & resample particles
      std::vector<float> beta;
      beta.clear();
      for (int i = 0; i < particleRowNCC_.size(); i++)
      {
          if (NCC[i] > 0.75*bestNCC) { beta.push_back(NCC[i]-0.75*bestNCC); }
          else { beta.push_back(0.0); }
      }
      float sum = std::accumulate(beta.begin(), beta.end(), 0.0);
      if ((sum == 0.0 || bestNCC < NCCThreshold_) && bestNCC != -1)                           // fix for second dataset with empty template update
      {
        particleRowNCC_.clear();
        particleColNCC_.clear();
        particleThetaNCC_.clear();
        initializeNCC_ = true;
        ROS_INFO("particle Filter NCC reinitialized");
      }
      else if(sum != 0.0)
      {
        std::transform(beta.begin(), beta.end(), beta.begin(), std::bind1st(std::multiplies<float>(), 1.0/sum));
        std::partial_sum(beta.begin(), beta.end(), beta.begin());

        std::vector< std::vector<int> > newParticles;
        newParticles.clear();
        int numberOfParticles = particleRowNCC_.size();
        if (numberOfParticles < 4000) { numberOfParticles = 4000; }
        for (int i = 0; i < numberOfParticles; i++)
        {
          float randNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * (beta.back()-beta[0]) + beta[0];
          int ind = std::upper_bound(beta.begin(), beta.end(), randNumber) - beta.begin() -1;

          std::vector<int> particle;
          particle.clear();

          particle.push_back( int(particleRowNCC_[ind] + round(distribution(generator_)) + rows*subresolution) % (rows*subresolution) );
          particle.push_back( int(particleColNCC_[ind] + round(distribution(generator_)) + cols*subresolution) % (cols*subresolution) );
          particle.push_back( int(particleThetaNCC_[ind] + round(distribution(generator_)) + 360) % 360);
          newParticles.push_back(particle);
        }
        
        std::sort(newParticles.begin(), newParticles.end());
        auto last = std::unique(newParticles.begin(), newParticles.end());
        newParticles.erase(last, newParticles.end());

        numberOfParticles = newParticles.size();
        particleRowNCC_.clear();
        particleColNCC_.clear();
        particleThetaNCC_.clear();

        for (int i = 0; i < numberOfParticles; i++)
        {
          particleRowNCC_.push_back(newParticles[i][0]);
          particleColNCC_.push_back(newParticles[i][1]);
          particleThetaNCC_.push_back(newParticles[i][2]);
        }
        std::cout <<"New number of particles NCC: " << numberOfParticles << std::endl;
      }
    }
  }

  if (MI_)
  {
    std::vector<float> MI;
    MI.clear();
    map_min_ = map_.get("elevation").minCoeffOfFinites();
    map_max_ = map_.get("elevation").maxCoeffOfFinites();
    reference_min_ = referenceMap_.get("elevation").minCoeffOfFinites();
    reference_max_ = referenceMap_.get("elevation").maxCoeffOfFinites();

    for (int i = 0; i < particleRowMI_.size(); i++)
    {
      float row = float(particleRowMI_[i])/subresolution;
      float col = float(particleColMI_[i])/subresolution;
      grid_map::Index index = grid_map::Index(int(round(row)), int(round(col)));
      int theta = particleThetaMI_[i];

      float sin_theta = sin((theta+templateRotation_)/180*M_PI);
      float cos_theta = cos((theta+templateRotation_)/180*M_PI);

      bool success = findMatches(data, variance_data, reference_data, row, col, sin_theta, cos_theta );
      if (success) 
      {
        float mutInfo;
        if (!weighted_) { mutInfo = mutualInformation(); }
        else { mutInfo = weightedMutualInformation(); }

        MI.push_back(mutInfo);

        grid_map::Position xy_position;
        referenceMap_.getPosition(index, xy_position);
        if (correlationMap.isInside(xy_position))
        {
          grid_map::Index correlation_index;
          correlationMap.getIndex(xy_position, correlation_index);

          bool valid = correlationMap.isValid(correlation_index, "MI");
          // if no value so far or correlation smaller or correlation higher than for other thetas
          if (((valid == false) || (mutInfo > correlationMap.at("MI", correlation_index) ))) 
          {
            correlationMap.at("MI", correlation_index) = mutInfo;  //set correlation
            correlationMap.at("rotationMI", correlation_index) = theta;    //set theta
          }
        }
      }
      else { MI.push_back(-10); }
    }
      // search best score 
    grid_map::Position best_pos;

    std::vector<float>::iterator MIit = std::max_element(MI.begin(), MI.end());
    int bestMIparticle = std::distance(MI.begin(), MIit);
    float bestMI = MI[bestMIparticle];
    //referenceMap_.getPosition(grid_map::Index(particleRowMI_[bestMIparticle], particleColMI_[bestMIparticle]), best_pos);
    //float bestXMI = best_pos(0);
    //float bestYMI = best_pos(1);
    int bestRow = int(round(particleRowMI_[bestMIparticle]/subresolution)) % rows;
    int bestCol = int(round(particleColMI_[bestMIparticle]/subresolution)) % cols;
    referenceMap_.getPosition(grid_map::Index(bestRow, bestCol), best_pos);
    float bestXMI = best_pos(0) - ( particleRowMI_[bestMIparticle]-int(round(particleRowMI_[bestMIparticle])) )*referenceMap_.getResolution()/subresolution; 
    float bestYMI = best_pos(1) - ( particleColMI_[bestMIparticle]-int(round(particleColMI_[bestMIparticle])) )*referenceMap_.getResolution()/subresolution;
    int bestThetaMI = particleThetaMI_[bestMIparticle];

    // Calculate z alignement
    float z = findZ(data, reference_data, bestXMI, bestYMI, bestThetaMI);
    if (bestMI != -10) 
    {
      cumulativeErrorMI_ += sqrt((bestXMI - map_position_(0))*(bestXMI - map_position_(0)) + (bestYMI - map_position_(1))*(bestYMI - map_position_(1)));
      if (sqrt((bestXMI - map_position_(0))*(bestXMI - map_position_(0)) + (bestYMI - map_position_(1))*(bestYMI - map_position_(1))) < 0.5 && (fabs(bestThetaMI - (360-templateRotation_)) < angleIncrement_ || fabs(bestThetaMI - (360-templateRotation_)) > 360-angleIncrement_)) {correctMatchesMI_ += 1;}
      geometry_msgs::PointStamped MIPoint;
      MIPoint.point.x = bestXMI - shift(0);
      MIPoint.point.y = bestYMI - shift(1);
      MIPoint.point.z = bestThetaMI;
      MIPoint.header.stamp = pubTime;
      MIPointPublisher_.publish(MIPoint);
    }
    std::cout << "Best MI " << bestMI << " at " << bestXMI << ", " << bestYMI << " , theta " << bestThetaMI << " and z: " << z << std::endl;
    std::cout << "Cumulative error MI: " << cumulativeErrorMI_ << " matches: " << correctMatchesMI_ << std::endl;

    if (resample_)
    {
      //update & resample particles
      std::vector<float> beta;
      beta.clear();
      for (int i = 0; i < particleRowMI_.size(); i++)
      {
          if (MI[i] > 0.90*bestMI) { beta.push_back(MI[i]-0.90*bestMI); }
          else { beta.push_back(0.0); }
      }
      float sum = std::accumulate(beta.begin(), beta.end(), 0.0);
      if ((sum == 0.0 || bestMI < MIThreshold_) && bestMI != -10 )                           // fix for second dataset with empty template update
      {
        particleRowMI_.clear();
        particleColMI_.clear();
        particleThetaMI_.clear();
        initializeMI_ = true;
        ROS_INFO("particle Filter MI reinitialized");
      }
      else if(sum != 0.0)
      {
        std::transform(beta.begin(), beta.end(), beta.begin(), std::bind1st(std::multiplies<float>(), 1.0/sum));
        std::partial_sum(beta.begin(), beta.end(), beta.begin());

        std::vector< std::vector<int> > newParticles;
        newParticles.clear();
        int numberOfParticles = particleRowMI_.size();
        if (numberOfParticles < 4000) { numberOfParticles = 4000; }
        for (int i = 0; i < numberOfParticles; i++)
        {
          float randNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * (beta.back()-beta[0]) + beta[0];
          int ind = std::upper_bound(beta.begin(), beta.end(), randNumber) - beta.begin() -1;

          std::vector<int> particle;
          particle.clear();

          particle.push_back( int(particleRowMI_[ind] + round(distribution(generator_)) + rows*subresolution) % (rows*subresolution) );
          particle.push_back( int(particleColMI_[ind] + round(distribution(generator_)) + cols*subresolution) % (cols*subresolution) );
          particle.push_back( int(particleThetaMI_[ind] + round(distribution(generator_)) + 360) % 360);
          newParticles.push_back(particle);
        }
        
        std::sort(newParticles.begin(), newParticles.end());
        auto last = std::unique(newParticles.begin(), newParticles.end());
        newParticles.erase(last, newParticles.end());

        numberOfParticles = newParticles.size();
        particleRowMI_.clear();
        particleColMI_.clear();
        particleThetaMI_.clear();

        for (int i = 0; i < numberOfParticles; i++)
        {
          particleRowMI_.push_back(newParticles[i][0]);
          particleColMI_.push_back(newParticles[i][1]);
          particleThetaMI_.push_back(newParticles[i][2]);
        }
        std::cout <<"New number of particles MI: " << numberOfParticles << std::endl;
      }
    }
  }

  grid_map_msgs::GridMap correlation_msg;
  grid_map::GridMapRosConverter::toMessage(correlationMap, correlation_msg);
  correlationPublisher_.publish(correlation_msg); 

  std::cout << "Correct position " << map_position_.transpose() << " and theta " << (360-templateRotation_) << std::endl;

  ros::Duration duration = ros::Time::now() - time;
  std::cout << "Time used: " << duration.toSec() << " Sekunden" << " 1: " << duration1_.toSec() << " 2: " << duration2_.toSec() << std::endl;
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

bool MapFitter::findMatches(grid_map::Matrix& data, grid_map::Matrix& variance_data, grid_map::Matrix& reference_data, float row, float col, float sin_theta, float cos_theta)
{
  // initialize
  int points = 0;
  matches_ = 0;

  shifted_mean_ = 0;
  reference_mean_ = 0;
  xy_shifted_.clear();
  xy_reference_.clear();
  xy_shifted_var_.clear();

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
  float reference_index_x = row; //reference_index(0);
  float reference_index_y = col; //reference_index(1);

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
        float reference_buffer_index_x = reference_size_x - reference_start_index_x + reference_index_x;
        float reference_buffer_index_y = reference_size_y - reference_start_index_y + reference_index_y;

        int shifted_index_x = round(fmod(reference_buffer_index_x, reference_size_x) - (cos_theta*(float(size_x)/2-i) - sin_theta*(float(size_y)/2-j)) );
        int shifted_index_y = round(fmod(reference_buffer_index_y, reference_size_y) - (sin_theta*(float(size_x)/2-i) + cos_theta*(float(size_y)/2-j)) );
              
        if (shifted_index_x >= 0 && shifted_index_x < reference_size_x && shifted_index_y >= 0 && shifted_index_y < reference_size_y )
        {
          shifted_index_x = (shifted_index_x + reference_start_index_x) % reference_size_x;
          shifted_index_y = (shifted_index_y + reference_start_index_y) % reference_size_y;
          float referenceHeight = reference_data(shifted_index_x, shifted_index_y);
          if (referenceHeight == referenceHeight)
          {
            matches_ += 1;
            shifted_mean_ += mapHeight;
            reference_mean_ += referenceHeight;
            xy_shifted_.push_back(mapHeight);
            xy_reference_.push_back(referenceHeight);
            float mapVariance = variance_data(index_x, index_y);
            if (mapVariance < 1e-6) { mapVariance = 1e-6; }
            xy_shifted_var_.push_back(mapVariance);
          }
        }
      }
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

float MapFitter::errorSAD()
{
  float error = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_);
    float reference = (xy_reference_[i]-reference_mean_);
    error += fabs(shifted-reference);
  }
  return error/matches_;
}

float MapFitter::weightedErrorSAD()
{
  float error = 0;
  float normalization = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_);
    float reference = (xy_reference_[i]-reference_mean_);
    error += fabs(shifted-reference) / xy_shifted_var_[i];
    normalization += 1.0/xy_shifted_var_[i];
  }
  return error/normalization;
}

float MapFitter::errorSSD()
{
  float error = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_);
    float reference = (xy_reference_[i]-reference_mean_);
    error += (shifted-reference)*(shifted-reference); //sqrt(fabs(shifted-reference)) instead of (shifted-reference)*(shifted-reference)
  }
  return error/matches_;
}

float MapFitter::weightedErrorSSD()
{
  float error = 0;
  float normalization = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted = (xy_shifted_[i]-shifted_mean_);
    float reference = (xy_reference_[i]-reference_mean_);
    //error += sqrt(fabs(shifted-reference) * xy_shifted_var_[i]);
    //normalization += sqrt(xy_shifted_var_[i]);
    error += (shifted-reference)*(shifted-reference) / (xy_shifted_var_[i]*xy_shifted_var_[i]);
    normalization += 1/(xy_shifted_var_[i]*xy_shifted_var_[i]);
  }
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
  float shifted_normal = 0;
  float reference_normal = 0;
  float correlation = 0;
  for (int i = 0; i < matches_; i++) 
  {
    float shifted_corr = (xy_shifted_[i]-shifted_mean_);
    float reference_corr = (xy_reference_[i]-reference_mean_);
    correlation += shifted_corr*reference_corr / xy_shifted_var_[i];
    shifted_normal += shifted_corr*shifted_corr / xy_shifted_var_[i];
    reference_normal += reference_corr*reference_corr / xy_shifted_var_[i];
  }
  return correlation/sqrt(shifted_normal*reference_normal);
}

float MapFitter::mutualInformation()
{
  const int numberOfBins = 128;

  float minHeight = map_min_ - shifted_mean_;
  if ((reference_min_ - reference_mean_) < minHeight) { minHeight = reference_min_ - reference_mean_; }

  float maxHeight = map_max_ - shifted_mean_;
  if ((reference_max_ - reference_mean_) > maxHeight) { maxHeight = reference_max_ - reference_mean_; }

  float binWidth = (maxHeight - minHeight + 1e-6) / numberOfBins;
  float hist[numberOfBins] = {0.0};
  float referenceHist[numberOfBins] = {0.0};
  float jointHist[numberOfBins][numberOfBins] = {0.0};

  for (int i = 0; i < matches_; i++)
  {
    int i1 = (xy_shifted_[i] - shifted_mean_ - minHeight) / binWidth;
    int i2 = (xy_reference_[i] - reference_mean_ - minHeight) / binWidth;
    hist[i1] += 1.0/matches_;
    referenceHist[i2] += 1.0/matches_;
    jointHist[i1][i2] += 1.0/matches_;
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
  ros::Time time1 = ros::Time::now();

  float minHeight = map_min_ - shifted_mean_;
  if ((reference_min_ - reference_mean_) < minHeight) { minHeight = reference_min_ - reference_mean_; }

  float maxHeight = map_max_ - shifted_mean_;
  if ((reference_max_ - reference_mean_) > maxHeight) { maxHeight = reference_max_ - reference_mean_; }

  const int numberOfBins = 128;

  float binWidth = (maxHeight - minHeight + 1e-6) / numberOfBins;
  float hist[numberOfBins] = {0.0};
  float referenceHist[numberOfBins] = {0.0};
  float jointHist[numberOfBins][numberOfBins] = {0.0};

  for (int i = 0; i < matches_; i++)
  {

    int i1 = (xy_shifted_[i] - shifted_mean_ - minHeight) / binWidth;
    int i2 = (xy_reference_[i] - reference_mean_ - minHeight) / binWidth;
    hist[i1] += 1.0/matches_;
    referenceHist[i2] += 1.0/matches_;
    jointHist[i1][i2] += 1.0/matches_;
  }

  float entropy = 0;
  float referenceEntropy = 0;
  float jointEntropy = 0;
  float weightSum = 0;

  for (int i = 0; i < numberOfBins; i++)
  {
    if (hist[i]!=0.0) { entropy += -hist[i]*log(hist[i]); }
    if (referenceHist[i]!=0.0) { referenceEntropy += -referenceHist[i]*log(referenceHist[i]); }

    for (int j = 0; j < numberOfBins; j++)
    {
      if (jointHist[i][j]!=0.0) 
      { 
        jointEntropy += -jointHist[i][j]*log(jointHist[i][j]);
        weightSum += (numberOfBins-abs(i-j))*(numberOfBins-abs(i-j))*jointHist[i][j];
      }
    }
  }
  jointEntropy = jointEntropy / ( weightSum / (numberOfBins*numberOfBins) );
  
  duration1_ += ros::Time::now() - time1;

  //std::cout << " template entropy: " << entropy << " reference entropy: " << referenceEntropy << " joint entropy: " << jointEntropy << " Mutual information: " << entropy+referenceEntropy-jointEntropy <<std::endl;
  return (entropy+referenceEntropy-jointEntropy); //-jointEntropy;
}

void MapFitter::tfBroadcast(const ros::TimerEvent&) 
{
  broadcaster_.sendTransform( tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"/map", "/grid_map"));
}

void MapFitter::tfListener(const ros::TimerEvent&)
{
  tf::StampedTransform position;
  try { listener_.lookupTransform("/map", "/footprint", ros::Time(0), position); }
  catch (tf::TransformException ex) { //ROS_ERROR("%s",ex.what()); 
  return; }
  tf::Matrix3x3 m(position.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ros::Time pubTime = ros::Time::now();
  geometry_msgs::PointStamped correctPoint;
  correctPoint.point.x = position.getOrigin().x();
  correctPoint.point.y = position.getOrigin().y();
  correctPoint.point.z = fmod(yaw/M_PI*180+360, 360);
  correctPoint.header.stamp = pubTime;
  correctPointPublisher_.publish(correctPoint);
}

} /* namespace */

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
  correlationPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(correlationMapTopic_,1);    // publisher for correlation_map
  referencePublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/uav_elevation_mapping/uav_elevation_map",1); // change back to referenceMapTopic_
  activityCheckTimer_ = nodeHandle_.createTimer(activityCheckDuration_, &MapFitter::updateSubscriptionCallback, this);
  broadcastTimer_ = nodeHandle_.createTimer(ros::Duration(0.01), &MapFitter::tfBroadcast, this);
  listenerTimer_  = nodeHandle_.createTimer(ros::Duration(0.01), &MapFitter::tfListener, this);
  NCCPointPublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("/NCCPoint",1);
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

  nodeHandle_.param("rho_SAD", rhoSAD_, float(-0.0025));
  nodeHandle_.param("rho_SSD", rhoSSD_, float(-0.0004));
  nodeHandle_.param("rho_NCC", rhoNCC_, float(0.025));
  nodeHandle_.param("rho_MI", rhoMI_, float(0.015));
  nodeHandle_.param("number_of_particles", numberOfParticles_, 4000);

  nodeHandle_.param("map_topic", mapTopic_, std::string("/elevation_mapping_long_range/elevation_map"));
  if (set_ == "set1") { nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/uav_elevation_mapping/uav_elevation_map")); }
  if (set_ == "set2") { nodeHandle_.param("reference_map_topic", referenceMapTopic_, std::string("/elevation_mapping/elevation_map")); }
  nodeHandle_.param("correlation_map_topic", correlationMapTopic_, std::string("/correlation_best_rotation/correlation_map"));

  nodeHandle_.param("angle_increment", angleIncrement_, 5);
  nodeHandle_.param("position_increment_search", searchIncrement_, 5);
  nodeHandle_.param("position_increment_correlation", correlationIncrement_, 5);
  nodeHandle_.param("required_overlap", requiredOverlap_, float(0.25));
  if (weighted_)
  {
    nodeHandle_.param("SAD_threshold", SADThreshold_, float(0.05));
    nodeHandle_.param("SSD_threshold", SSDThreshold_, float(0.008));
    nodeHandle_.param("NCC_threshold", NCCThreshold_, float(0.6));
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

  cumulativeErrorNCC_ = 0;
  cumulativeErrorSSD_ = 0;
  cumulativeErrorSAD_ = 0;
  cumulativeErrorMI_ = 0;
  correctMatchesNCC_ = 0;
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
    grid_map::GridMapRosConverter::loadFromBag("/home/roman/rosbags/reference_map_last.bag", referenceMapTopic_, referenceMap_);
    referenceMap_.move(grid_map::Position(2.75,1));

    grid_map::GridMap extendMap;
    extendMap.setGeometry(grid_map::Length(10.5,8.5), referenceMap_.getResolution(), referenceMap_.getPosition());
    extendMap.setFrameId("grid_map");
    referenceMap_.extendToInclude(extendMap);

    referenceMap_.getDataBoundingSubmap("elevation", submap_start_index, submap_size);

    grid_map::Size reference_size = referenceMap_.getSize();
    grid_map::Index reference_start_index = referenceMap_.getStartIndex();

    for (grid_map::GridMapIterator iterator(referenceMap_); !iterator.isPastEnd(); ++iterator) 
    {
      grid_map::Index index(*iterator);
      grid_map::Index shaped_index = grid_map::getIndexFromBufferIndex(index, reference_size, reference_start_index);
      grid_map::Index shaped_submap_start_index = grid_map::getIndexFromBufferIndex(submap_start_index, reference_size, reference_start_index);
      bool outside_submap = (shaped_index(0) < shaped_submap_start_index(0)-10 || shaped_index(1) < shaped_submap_start_index(1)-15 || shaped_index(0) > shaped_submap_start_index(0)+submap_size(0)+0 || shaped_index(1) > shaped_submap_start_index(1)+submap_size(1)+0);
      if ( outside_submap )
      {
        referenceMap_.at("elevation", index) = -0.75 + static_cast <float> (rand()) / static_cast <float> (RAND_MAX)/20;
      }
    }

    grid_map_msgs::GridMap reference_msg;
    grid_map::GridMapRosConverter::toMessage(referenceMap_, reference_msg);
    referencePublisher_.publish(reference_msg);
  }
  
  if (set_ == "set2") 
  { 
    grid_map::GridMapRosConverter::loadFromBag("/home/roman/rosbags/source/asl_walking_uav/uav_reference_map.bag", referenceMapTopic_, referenceMap_); 

    grid_map_msgs::GridMap reference_msg;
    grid_map::GridMapRosConverter::toMessage(referenceMap_, reference_msg);
    referencePublisher_.publish(reference_msg);
    
    referenceMap_.getDataBoundingSubmap("elevation", submap_start_index, submap_size);

    submap_start_index = grid_map::Index((submap_start_index(0) + 50 + referenceMap_.getSize()(0) ) % referenceMap_.getSize()(0), (submap_start_index(1) + 20 + referenceMap_.getSize()(1) ) % referenceMap_.getSize()(1));
    submap_size = submap_size - grid_map::Size(125,40);
  }

  exhaustiveSearch(submap_start_index, submap_size);
}

void MapFitter::exhaustiveSearch(grid_map::Index submap_start_index, grid_map::Size submap_size)
{
  // initialize correlationMap
  grid_map::GridMap correlationMap({"NCC","rotationNCC","SSD","rotationSSD","SAD","rotationSAD", "MI", "rotationMI"});
  correlationMap.setGeometry(referenceMap_.getLength(), referenceMap_.getResolution(),
                              referenceMap_.getPosition()); //TODO only use submap
  correlationMap.setFrameId("grid_map");

  //initialize parameters
  grid_map::Size reference_size = referenceMap_.getSize();
  int rows = reference_size(0);
  int cols = reference_size(1);
  float resolution = referenceMap_.getResolution();
  int subresolution = 1;//resolution / 0.01;

  grid_map::Position previous_position = map_position_;
  map_position_ = map_.getPosition();
  grid_map::Position position = referenceMap_.getPosition();
  grid_map::Size size = map_.getSize();
  grid_map::Index start_index = map_.getStartIndex();
  grid_map::Index reference_start_index = referenceMap_.getStartIndex();

  grid_map::Matrix& reference_data = referenceMap_["elevation"];
  grid_map::Matrix& data = map_["elevation"];
  grid_map::Matrix& variance_data = map_["variance"];

  tf::StampedTransform correct_position;
  try { listener_.lookupTransform("/map", "/base", ros::Time(0), correct_position); }
  catch (tf::TransformException ex) { ROS_ERROR("%s",ex.what()); }
  tf::Matrix3x3 m(correct_position.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  float previous_templateRotation = templateRotation_;
  templateRotation_ = 360.0 - fmod(yaw/M_PI*180+360,360);
  grid_map::Position shift = grid_map::Position(map_position_(0)-correct_position.getOrigin().x(), map_position_(1)-correct_position.getOrigin().y() );

  ros::Time pubTime = ros::Time::now();
  ros::Time time = ros::Time::now();
  duration1_.sec = 0;
  duration1_.nsec = 0;
  duration2_.sec = 0;
  duration2_.nsec = 0;

  std::normal_distribution<float> distribution(0.0,2.0*subresolution);

  bool initialized_all = false;
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
    if (initializeSAD_ && initializeSSD_ && initializeNCC_ && initializeMI_) { initialized_all = true; }
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
    if (SAD_ && resample_)
    {
      std::transform(particleRowSAD_.begin(), particleRowSAD_.end(), particleRowSAD_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(0) - previous_position(0)) / resolution + rows)*subresolution + distribution(generator_)/2) ));
      std::transform(particleRowSAD_.begin(), particleRowSAD_.end(), particleRowSAD_.begin(), std::bind2nd(std::modulus<int>(), rows*subresolution));
      std::transform(particleColSAD_.begin(), particleColSAD_.end(), particleColSAD_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(1) - previous_position(1)) / resolution + cols)*subresolution + distribution(generator_)/2) ));
      std::transform(particleColSAD_.begin(), particleColSAD_.end(), particleColSAD_.begin(), std::bind2nd(std::modulus<int>(), cols*subresolution));
      std::transform(particleThetaSAD_.begin(), particleThetaSAD_.end(), particleThetaSAD_.begin(), std::bind2nd(std::plus<int>(), round( -(templateRotation_ - previous_templateRotation) + 360 + distribution(generator_)/2) ));
      std::transform(particleThetaSAD_.begin(), particleThetaSAD_.end(), particleThetaSAD_.begin(), std::bind2nd(std::modulus<int>(), 360));
    }
    if (SSD_ && resample_)
    {
      std::transform(particleRowSSD_.begin(), particleRowSSD_.end(), particleRowSSD_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(0) - previous_position(0)) / resolution + rows)*subresolution + distribution(generator_)/2) ));
      std::transform(particleRowSSD_.begin(), particleRowSSD_.end(), particleRowSSD_.begin(), std::bind2nd(std::modulus<int>(), rows*subresolution));
      std::transform(particleColSSD_.begin(), particleColSSD_.end(), particleColSSD_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(1) - previous_position(1)) / resolution + cols)*subresolution + distribution(generator_)/2) ));
      std::transform(particleColSSD_.begin(), particleColSSD_.end(), particleColSSD_.begin(), std::bind2nd(std::modulus<int>(), cols*subresolution));
      std::transform(particleThetaSSD_.begin(), particleThetaSSD_.end(), particleThetaSSD_.begin(), std::bind2nd(std::plus<int>(), round( -(templateRotation_ - previous_templateRotation) + 360 + distribution(generator_)/2) ));
      std::transform(particleThetaSSD_.begin(), particleThetaSSD_.end(), particleThetaSSD_.begin(), std::bind2nd(std::modulus<int>(), 360));
    }
    if (NCC_ && resample_)
    {
      std::transform(particleRowNCC_.begin(), particleRowNCC_.end(), particleRowNCC_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(0) - previous_position(0)) / resolution + rows)*subresolution + distribution(generator_)/2) ));
      std::transform(particleRowNCC_.begin(), particleRowNCC_.end(), particleRowNCC_.begin(), std::bind2nd(std::modulus<int>(), rows*subresolution));
      std::transform(particleColNCC_.begin(), particleColNCC_.end(), particleColNCC_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(1) - previous_position(1)) / resolution + cols)*subresolution + distribution(generator_)/2) ));
      std::transform(particleColNCC_.begin(), particleColNCC_.end(), particleColNCC_.begin(), std::bind2nd(std::modulus<int>(), cols*subresolution));
      std::transform(particleThetaNCC_.begin(), particleThetaNCC_.end(), particleThetaNCC_.begin(), std::bind2nd(std::plus<int>(), round( -(templateRotation_ - previous_templateRotation) + 360 + distribution(generator_)/2) ));
      std::transform(particleThetaNCC_.begin(), particleThetaNCC_.end(), particleThetaNCC_.begin(), std::bind2nd(std::modulus<int>(), 360));
    }
    if (MI_ && resample_)
    {
      std::transform(particleRowMI_.begin(), particleRowMI_.end(), particleRowMI_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(0) - previous_position(0)) / resolution + rows)*subresolution + distribution(generator_)/2) ));
      std::transform(particleRowMI_.begin(), particleRowMI_.end(), particleRowMI_.begin(), std::bind2nd(std::modulus<int>(), rows*subresolution));
      std::transform(particleColMI_.begin(), particleColMI_.end(), particleColMI_.begin(), std::bind2nd(std::plus<int>(), round( (-(map_position_(1) - previous_position(1)) / resolution + cols)*subresolution + distribution(generator_)/2) ));
      std::transform(particleColMI_.begin(), particleColMI_.end(), particleColMI_.begin(), std::bind2nd(std::modulus<int>(), cols*subresolution));
      std::transform(particleThetaMI_.begin(), particleThetaMI_.end(), particleThetaMI_.begin(), std::bind2nd(std::plus<int>(), round( -(templateRotation_ - previous_templateRotation) + 360 + distribution(generator_)/2) ));
      std::transform(particleThetaMI_.begin(), particleThetaMI_.end(), particleThetaMI_.begin(), std::bind2nd(std::modulus<int>(), 360));
    }
  }

  ros::Duration duration3;
  ros::Duration duration4;
  ros::Time time1 = ros::Time::now();
  if ((resample_ || !(SAD_ && SSD_ && NCC_ && MI_)) && !initialized_all)
  {
    if (particleRowSAD_.size() < 4000) { correlationIncrement_ = 1; }
    else { correlationIncrement_ = 5; }
    if (SAD_)
    {
      std::vector<float> SAD; SAD.clear();
      iterateParticles("SAD",subresolution,data,variance_data,reference_data,SAD,correlationMap,shift);

      std::vector<float> bestPos; bestPos.clear();
      bestPos = findBestPos("SAD", SAD, subresolution);

      // Calculate z alignement
      float z = findZ(data, reference_data, bestPos[0], bestPos[1], bestPos[2]);
      if (bestPos[3] != noneSAD_) 
      {
        cumErrorAndCorrMatches("SAD", bestPos);
        publishPoint("SAD", bestPos, shift, pubTime);
      }
      std::cout << "Best SAD " << bestPos[3] << " at " << bestPos[0] << ", " << bestPos[1] << " , theta " << bestPos[2] << " and z: " << z << std::endl;
      std::cout << "Cumulative error SAD: " << cumulativeErrorSAD_ << " matches: " << correctMatchesSAD_ << std::endl;

      if (resample_) { resample("SAD", bestPos, SAD, distribution, subresolution); }
    }

    duration1_ += ros::Time::now() - time1;
    ros::Time time2 = ros::Time::now();

    if (particleRowSSD_.size() < 4000) { correlationIncrement_ = 1; }
    else { correlationIncrement_ = 5; }
    if (SSD_)
    {
      std::vector<float> SSD; SSD.clear();
      iterateParticles("SSD",subresolution,data,variance_data,reference_data,SSD,correlationMap,shift);

      std::vector<float> bestPos; bestPos.clear();
      bestPos = findBestPos("SSD", SSD, subresolution);

      // Calculate z alignement
      float z = findZ(data, reference_data, bestPos[0], bestPos[1], bestPos[2]);
      if (bestPos[3] != noneSSD_) 
      {
        cumErrorAndCorrMatches("SSD", bestPos);
        publishPoint("SSD", bestPos, shift, pubTime);
      }
      std::cout << "Best SSD " << bestPos[3] << " at " << bestPos[0] << ", " << bestPos[1] << " , theta " << bestPos[2] << " and z: " << z << std::endl;
      std::cout << "Cumulative error SSD: " << cumulativeErrorSSD_ << " matches: " << correctMatchesSSD_ << std::endl;

      if (resample_) { resample("SSD", bestPos, SSD, distribution, subresolution); }
    }

    duration2_ += ros::Time::now() - time2;
    ros::Time time3 = ros::Time::now();

    if (particleRowNCC_.size() < 4000) { correlationIncrement_ = 1; }
    else { correlationIncrement_ = 5; }
    if (NCC_)
    {
      std::vector<float> NCC; NCC.clear();
      iterateParticles("NCC",subresolution,data,variance_data,reference_data,NCC,correlationMap,shift);

      std::vector<float> bestPos;
      bestPos.clear();
      bestPos = findBestPos("NCC", NCC, subresolution);

      // Calculate z alignement
      float z = findZ(data, reference_data, bestPos[0], bestPos[1], bestPos[2]);
      if (bestPos[3] != noneNCC_) 
      {
        cumErrorAndCorrMatches("NCC", bestPos);
        publishPoint("NCC", bestPos, shift, pubTime);
      }
      std::cout << "Best NCC " << bestPos[3] << " at " << bestPos[0] << ", " << bestPos[1] << " , theta " << bestPos[2] << " and z: " << z << std::endl;
      std::cout << "Cumulative error NCC: " << cumulativeErrorNCC_ << " matches: " << correctMatchesNCC_ << std::endl;

      if (resample_) { resample("NCC", bestPos, NCC, distribution, subresolution); }
    }

    duration3 = ros::Time::now() - time3;
    ros::Time time4 = ros::Time::now();

    if (particleRowMI_.size() < 4000) { correlationIncrement_ = 1; }
    else { correlationIncrement_ = 5; }
    if (MI_)
    {
      std::vector<float> MI; MI.clear();
      map_min_ = map_.get("elevation").minCoeffOfFinites();
      map_max_ = map_.get("elevation").maxCoeffOfFinites();
      reference_min_ = referenceMap_.get("elevation").minCoeffOfFinites();
      reference_max_ = referenceMap_.get("elevation").maxCoeffOfFinites();

      iterateParticles("MI",subresolution,data,variance_data,reference_data,MI,correlationMap,shift);

      std::vector<float> bestPos; bestPos.clear();
      bestPos = findBestPos("MI", MI, subresolution);

      // Calculate z alignement
      float z = findZ(data, reference_data, bestPos[0], bestPos[1], bestPos[2]);
      if (bestPos[3] != noneMI_) 
      {
        cumErrorAndCorrMatches("MI", bestPos);
        publishPoint("MI", bestPos, shift, pubTime);
      }
      std::cout << "Best MI " << bestPos[3] << " at " << bestPos[0] << ", " << bestPos[1] << " , theta " << bestPos[2] << " and z: " << z << std::endl;
      std::cout << "Cumulative error MI: " << cumulativeErrorMI_ << " matches: " << correctMatchesMI_ << std::endl;

      if (resample_) { resample("MI", bestPos, MI, distribution, subresolution); }
    }
    duration4 = ros::Time::now() - time4;
  }
  else if (SAD_ && SSD_ && NCC_ && MI_) // just to speed up
  {
    std::vector<float> SAD; SAD.clear();
    std::vector<float> SSD; SSD.clear();
    std::vector<float> NCC; NCC.clear();
    std::vector<float> MI; MI.clear();
    map_min_ = map_.get("elevation").minCoeffOfFinites();
    map_max_ = map_.get("elevation").maxCoeffOfFinites();
    reference_min_ = referenceMap_.get("elevation").minCoeffOfFinites();
    reference_max_ = referenceMap_.get("elevation").maxCoeffOfFinites();


    for (int i = 0; i < particleRowSAD_.size(); i++)
    {
      float row = float(particleRowSAD_[i])/subresolution;
      float col = float(particleColSAD_[i])/subresolution;
      grid_map::Index index = grid_map::Index(int(round(row)), int(round(col)));
      int theta = particleThetaSAD_[i];

      float sin_theta = sin((theta+templateRotation_)/180*M_PI);
      float cos_theta = cos((theta+templateRotation_)/180*M_PI);

      bool success = findMatches(data, variance_data, reference_data, row, col, sin_theta, cos_theta, true );

      calculateSimilarity(success,"SAD",index,theta,SAD,correlationMap,shift);
      calculateSimilarity(success,"SSD",index,theta,SSD,correlationMap,shift);
      calculateSimilarity(success,"NCC",index,theta,NCC,correlationMap,shift);
      calculateSimilarity(success,"MI",index,theta,MI,correlationMap,shift);
    }

    std::vector<float> bestPosSAD; bestPosSAD.clear();
    bestPosSAD = findBestPos("SAD", SAD, subresolution);
 
    std::vector<float> bestPosSSD; bestPosSSD.clear();
    bestPosSSD = findBestPos("SSD", SSD, subresolution);

    std::vector<float> bestPosNCC; bestPosNCC.clear();
    bestPosNCC = findBestPos("NCC", NCC, subresolution);

    std::vector<float> bestPosMI; bestPosMI.clear();
    bestPosMI = findBestPos("MI", MI, subresolution);

    // Calculate z alignement
    float z = findZ(data, reference_data, bestPosSAD[0], bestPosSAD[1], bestPosSAD[2]);
    if (bestPosSAD[3] != noneSAD_) 
    {
      cumErrorAndCorrMatches("SAD", bestPosSAD);
      publishPoint("SAD", bestPosSAD, shift, pubTime);
    }
    std::cout << "Best SAD " << bestPosSAD[3] << " at " << bestPosSAD[0] << ", " << bestPosSAD[1] << " , theta " << bestPosSAD[2] << " and z: " << z << std::endl;
    std::cout << "Cumulative error SAD: " << cumulativeErrorSAD_ << " matches: " << correctMatchesSAD_ << std::endl;

    z = findZ(data, reference_data, bestPosSSD[0], bestPosSSD[1], bestPosSSD[2]);
    if (bestPosSSD[3] != noneSSD_) 
    {
      cumErrorAndCorrMatches("SSD", bestPosSSD);
      publishPoint("SSD", bestPosSSD, shift, pubTime);
    }
    std::cout << "Best SSD " << bestPosSSD[3] << " at " << bestPosSSD[0] << ", " << bestPosSSD[1] << " , theta " << bestPosSSD[2] << " and z: " << z << std::endl;
    std::cout << "Cumulative error SSD: " << cumulativeErrorSSD_ << " matches: " << correctMatchesSSD_ << std::endl;

    z = findZ(data, reference_data, bestPosNCC[0], bestPosNCC[1], bestPosNCC[2]);
    if (bestPosNCC[3] != noneNCC_) 
    {
      cumErrorAndCorrMatches("NCC", bestPosNCC);
      publishPoint("NCC", bestPosNCC, shift, pubTime);
    }
    std::cout << "Best NCC " << bestPosNCC[3] << " at " << bestPosNCC[0] << ", " << bestPosNCC[1] << " , theta " << bestPosNCC[2] << " and z: " << z << std::endl;
    std::cout << "Cumulative error NCC: " << cumulativeErrorNCC_ << " matches: " << correctMatchesNCC_ << std::endl;

    z = findZ(data, reference_data, bestPosMI[0], bestPosMI[1], bestPosMI[2]);
    if (bestPosMI[3] != noneMI_) 
    {
      cumErrorAndCorrMatches("MI", bestPosMI);
      publishPoint("MI", bestPosMI, shift, pubTime);
    }
    std::cout << "Best MI " << bestPosMI[3] << " at " << bestPosMI[0] << ", " << bestPosMI[1] << " , theta " << bestPosMI[2] << " and z: " << z << std::endl;
    std::cout << "Cumulative error MI: " << cumulativeErrorMI_ << " matches: " << correctMatchesMI_ << std::endl;

    if (resample_)
    {
      resample("SAD", bestPosSAD, SAD, distribution, subresolution);
      resample("SSD", bestPosSSD, SSD, distribution, subresolution);
      resample("NCC", bestPosNCC, NCC, distribution, subresolution);
      resample("MI", bestPosMI, MI, distribution, subresolution);
    }
  }

  grid_map_msgs::GridMap correlation_msg;
  grid_map::GridMapRosConverter::toMessage(correlationMap, correlation_msg);
  correlationPublisher_.publish(correlation_msg); 

  std::cout << "Correct position " << map_position_.transpose() << " and theta " << (360-templateRotation_) << std::endl;

  ros::Duration duration = ros::Time::now() - time;
  std::cout << "Time used: " << duration.toSec() << " Sekunden" << " 1: " << duration1_.toSec() << " 2: " << duration2_.toSec() << " 3: " << duration3.toSec() << " 4: " << duration4.toSec() << std::endl;
  ROS_INFO("done");
  isActive_ = false;
}

void MapFitter::iterateParticles(std::string score,int subresolution,grid_map::Matrix& data,grid_map::Matrix& variance_data,grid_map::Matrix& reference_data,std::vector<float>& scores,grid_map::GridMap& correlationMap,grid_map::Position& shift)
{
  std::map <std::string,std::vector<int>> rowMap;
  rowMap["SAD"] = particleRowSAD_; rowMap["SSD"] = particleRowSSD_; rowMap["NCC"] = particleRowNCC_; rowMap["MI"] = particleRowMI_;
  std::map <std::string,std::vector<int>> colMap;
  colMap["SAD"] = particleColSAD_; colMap["SSD"] = particleColSSD_; colMap["NCC"] = particleColNCC_; colMap["MI"] = particleColMI_;
  std::map <std::string,std::vector<int>> thetaMap;
  thetaMap["SAD"] = particleThetaSAD_; thetaMap["SSD"] = particleThetaSSD_; thetaMap["NCC"] = particleThetaNCC_; thetaMap["MI"] = particleThetaMI_;

  for (int i = 0; i < rowMap[score].size(); i++)
  {
    float row = float(rowMap[score][i])/subresolution;
    float col = float(colMap[score][i])/subresolution;
    grid_map::Index index = grid_map::Index(int(round(row)), int(round(col)));
    int theta = thetaMap[score][i];

    float sin_theta = sin((theta+templateRotation_)/180*M_PI);
    float cos_theta = cos((theta+templateRotation_)/180*M_PI);

    bool success;
    if (score=="MI") { success = findMatches(data, variance_data, reference_data, row, col, sin_theta, cos_theta, true ); }
    else { success = findMatches(data, variance_data, reference_data, row, col, sin_theta, cos_theta, false ); }
    
    calculateSimilarity(success,score,index,theta,scores,correlationMap,shift);
  }
}

void MapFitter::calculateSimilarity(bool success,std::string score,grid_map::Index index,int theta,std::vector<float>& scores,grid_map::GridMap& correlationMap,grid_map::Position& shift)
{
  if (success) 
    {
      float value;
      if (!weighted_) 
      { 
        if(score=="SAD") {value = errorSAD();}
        if(score=="SSD") {value = errorSSD();}
        if(score=="NCC") {value = correlationNCC();}
        if(score=="MI") {value = mutualInformation();}
      }
      else 
      { 
        if(score=="SAD") {value = weightedErrorSAD();}
        if(score=="SSD") {value = weightedErrorSSD();}
        if(score=="NCC") {value = weightedCorrelationNCC();}
        if(score=="MI") {value = normalizedMutualInformation();}
      }

      scores.push_back(value);

      grid_map::Position xy_position;
      referenceMap_.getPosition(index, xy_position);
      if (correlationMap.isInside(xy_position-shift))
      {
        grid_map::Index correlation_index;
        correlationMap.getIndex(xy_position-shift, correlation_index);

        bool valid = correlationMap.isValid(correlation_index, score);
        // if no value so far or correlation smaller or correlation higher than for other thetas
        if (((valid == false) || (value < correlationMap.at(score, correlation_index) ))) 
        {
          float alpha = 1;
          if(score=="SAD") {alpha = 10;}
          if(score=="SSD") {alpha = 50;}
          correlationMap.at(score, correlation_index) = alpha*value;  //set correlation
          correlationMap.at("rotation"+score, correlation_index) = theta;    //set theta
        }
      }
    }
    else 
    { 
      if(score=="SAD") {scores.push_back(noneSAD_);}
      if(score=="SSD") {scores.push_back(noneSSD_);}
      if(score=="NCC") {scores.push_back(noneNCC_);}
      if(score=="MI") {scores.push_back(noneMI_);}
    }
}

std::vector<float> MapFitter::findBestPos(std::string score, std::vector<float> scores, int subresolution)
{
  std::map <std::string,std::vector<int>> rowMap;
  rowMap["SAD"] = particleRowSAD_; rowMap["SSD"] = particleRowSSD_; rowMap["NCC"] = particleRowNCC_; rowMap["MI"] = particleRowMI_;
  std::map <std::string,std::vector<int>> colMap;
  colMap["SAD"] = particleColSAD_; colMap["SSD"] = particleColSSD_; colMap["NCC"] = particleColNCC_; colMap["MI"] = particleColMI_;
  std::map <std::string,std::vector<int>> thetaMap;
  thetaMap["SAD"] = particleThetaSAD_; thetaMap["SSD"] = particleThetaSSD_; thetaMap["NCC"] = particleThetaNCC_; thetaMap["MI"] = particleThetaMI_;

  grid_map::Size reference_size = referenceMap_.getSize();
  int rows = reference_size(0);
  int cols = reference_size(1);
  grid_map::Position best_pos;

  int bestParticle;
  if (score == "SAD" || score == "SSD")
  {
    std::vector<float>::iterator it = std::min_element(scores.begin(), scores.end());
    bestParticle = std::distance(scores.begin(), it);
  }
  else
  {
    std::vector<float>::iterator it = std::max_element(scores.begin(), scores.end());
    bestParticle = std::distance(scores.begin(), it);
  }

  float value = scores[bestParticle];
  int bestRow = int(round(float(rowMap[score][bestParticle])/subresolution)) % rows;
  int bestCol = int(round(float(colMap[score][bestParticle])/subresolution)) % cols;
  referenceMap_.getPosition(grid_map::Index(bestRow, bestCol), best_pos);
  float bestX = best_pos(0) - ( float(rowMap[score][bestParticle])/subresolution-int(round(float(rowMap[score][bestParticle])/subresolution)) )*referenceMap_.getResolution(); 
  float bestY = best_pos(1) - ( float(colMap[score][bestParticle])/subresolution-int(round(float(colMap[score][bestParticle])/subresolution)) )*referenceMap_.getResolution();
  int bestTheta = thetaMap[score][bestParticle];

  std::vector<float> bestPos;
  bestPos.clear();
  bestPos.push_back(bestX);
  bestPos.push_back(bestY);
  bestPos.push_back(bestTheta);
  bestPos.push_back(value);
  return bestPos;
}

void MapFitter::publishPoint(std::string score, std::vector<float>& bestPos, grid_map::Position& shift, ros::Time pubTime)
{
  std::map <std::string,ros::Publisher> pubMap;
  pubMap["SAD"] = SADPointPublisher_; pubMap["SSD"] = SSDPointPublisher_; pubMap["NCC"] = NCCPointPublisher_; pubMap["MI"] = MIPointPublisher_;

  geometry_msgs::PointStamped Point;
  Point.point.x = bestPos[0] - shift(0);
  Point.point.y = bestPos[1] - shift(1);
  Point.point.z = bestPos[2];
  Point.header.stamp = pubTime;
  pubMap[score].publish(Point);
}

void MapFitter::cumErrorAndCorrMatches(std::string score, std::vector<float> bestPos)
{
  float distError = sqrt((bestPos[0] - map_position_(0))*(bestPos[0] - map_position_(0)) + (bestPos[1] - map_position_(1))*(bestPos[1] - map_position_(1)) );
  if (score == "SAD") { cumulativeErrorSAD_ += distError; }
  if (score == "SSD") { cumulativeErrorSSD_ += distError; }
  if (score == "NCC") { cumulativeErrorNCC_ += distError; }
  if (score == "MI") { cumulativeErrorMI_ += distError; }

  if (distError < 0.5 && (fabs(bestPos[2] - (360-templateRotation_)) < angleIncrement_ || fabs(bestPos[2] - (360-templateRotation_)) > 360-angleIncrement_)) 
  {
    if (score == "SAD") { correctMatchesSSD_ += 1; }
    if (score == "SSD") { correctMatchesSAD_ += 1; }
    if (score == "NCC") { correctMatchesNCC_ += 1; }
    if (score == "MI") { correctMatchesMI_ += 1; }
  }
}

void MapFitter::resample(std::string score, std::vector<float> bestPos, std::vector<float> scores, std::normal_distribution<float>& distribution, int subresolution)
{
  std::map <std::string,std::vector<int>> rowMap;
  rowMap["SAD"] = particleRowSAD_; rowMap["SSD"] = particleRowSSD_; rowMap["NCC"] = particleRowNCC_; rowMap["MI"] = particleRowMI_;
  std::map <std::string,std::vector<int>> colMap;
  colMap["SAD"] = particleColSAD_; colMap["SSD"] = particleColSSD_; colMap["NCC"] = particleColNCC_; colMap["MI"] = particleColMI_;
  std::map <std::string,std::vector<int>> thetaMap;
  thetaMap["SAD"] = particleThetaSAD_; thetaMap["SSD"] = particleThetaSSD_; thetaMap["NCC"] = particleThetaNCC_; thetaMap["MI"] = particleThetaMI_;
  std::map <std::string, float> thresMap;
  thresMap["SAD"] = SADThreshold_; thresMap["SSD"] = SSDThreshold_; thresMap["NCC"] = NCCThreshold_; thresMap["MI"] = MIThreshold_;
  std::map <std::string, int> noneMap;
  noneMap["SAD"] = noneSAD_; noneMap["SSD"] = noneSSD_; noneMap["NCC"] = noneNCC_; noneMap["MI"] = noneMI_;
  std::map <std::string, float> rhoMap;
  rhoMap["SAD"] = rhoSAD_; rhoMap["SSD"] = rhoSSD_; rhoMap["NCC"] = rhoNCC_; rhoMap["MI"] = rhoMI_;

  grid_map::Size reference_size = referenceMap_.getSize();
  int rows = reference_size(0);
  int cols = reference_size(1);

  std::vector<float> beta;
  beta.clear();
  for (int i = 0; i < rowMap[score].size(); i++)
  {
      beta.push_back(exp(scores[i]/rhoMap[score]));
  }
  float sum = std::accumulate(beta.begin(), beta.end(), 0.0);
  if ( ( ((score == "SAD" || score == "SSD") && (sum == 0.0 || bestPos[3] > thresMap[score])) || ((score == "NCC" || score == "MI") && (sum == 0.0 || bestPos[3] < thresMap[score])) ) && bestPos[3] != noneMap[score] )                           // fix for second dataset with empty template update
  {
    rowMap[score].clear();
    colMap[score].clear();
    thetaMap[score].clear();
    initializeSAD_ = true;
    std::cout << "particle Filter " << score <<" reinitialized" << std::endl;
  }
  else if(bestPos[3] != noneMap[score])
  {
    std::transform(beta.begin(), beta.end(), beta.begin(), std::bind1st(std::multiplies<float>(), 1.0/sum));
    std::partial_sum(beta.begin(), beta.end(), beta.begin());

    std::vector< std::vector<int> > newParticles;
    newParticles.clear();

    for (int i = 0; i < numberOfParticles_; i++)
    {
      float randNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * (beta.back()-beta[0]) + beta[0];
      int ind = std::upper_bound(beta.begin(), beta.end(), randNumber) - beta.begin() -1;

      std::vector<int> particle;
      particle.clear();

      particle.push_back( int(rowMap[score][ind] + round(distribution(generator_)) + rows*subresolution) % (rows*subresolution) );
      particle.push_back( int(colMap[score][ind] + round(distribution(generator_)) + cols*subresolution) % (cols*subresolution) );
      particle.push_back( int(thetaMap[score][ind] + round(distribution(generator_)) + 360) % 360);
      newParticles.push_back(particle);
    }
    
    std::sort(newParticles.begin(), newParticles.end());
    auto last = std::unique(newParticles.begin(), newParticles.end());
    newParticles.erase(last, newParticles.end());

    int numberOfParticles = newParticles.size();
    if (score == "SAD") 
    { 
      particleRowSAD_.clear(); particleColSAD_.clear(); particleThetaSAD_.clear();
      for (int i = 0; i < numberOfParticles; i++)
      {
        particleRowSAD_.push_back(newParticles[i][0]);
        particleColSAD_.push_back(newParticles[i][1]);
        particleThetaSAD_.push_back(newParticles[i][2]);
      }
      std::cout <<"New number of particles SAD: " << numberOfParticles << std::endl;
    }
    if (score == "SSD") 
    { 
      particleRowSSD_.clear(); particleColSSD_.clear(); particleThetaSSD_.clear(); 
      for (int i = 0; i < numberOfParticles; i++)
      {
        particleRowSSD_.push_back(newParticles[i][0]);
        particleColSSD_.push_back(newParticles[i][1]);
        particleThetaSSD_.push_back(newParticles[i][2]);
      }
      std::cout <<"New number of particles SSD: " << numberOfParticles << std::endl;
    }
    if (score == "NCC") 
    { 
      particleRowNCC_.clear(); particleColNCC_.clear(); particleThetaNCC_.clear(); 
      for (int i = 0; i < numberOfParticles; i++)
      {
        particleRowNCC_.push_back(newParticles[i][0]);
        particleColNCC_.push_back(newParticles[i][1]);
        particleThetaNCC_.push_back(newParticles[i][2]);
      }
      std::cout <<"New number of particles NCC: " << numberOfParticles << std::endl;
    }
    if (score == "MI") 
    { 
      particleRowMI_.clear(); particleColMI_.clear(); particleThetaMI_.clear(); 
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

bool MapFitter::findMatches(grid_map::Matrix& data, grid_map::Matrix& variance_data, grid_map::Matrix& reference_data, float row, float col, float sin_theta, float cos_theta, bool equal)
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
    if (equal)
    {
      //assure that we always have the same number of points
      for (int f =0; f < size_x*size_y; f++)
      {
        int index_x = round(static_cast <float> (rand() / static_cast <float> (RAND_MAX/(size_x-1)))); //rand() %size_x;
        int index_y = round(static_cast <float> (rand() / static_cast <float> (RAND_MAX/(size_y-1)))); //rand() %size_y;
        int i = (index_x - start_index_x + size_x) % size_x; 
        int j = (index_y - start_index_y + size_y) % size_y; 
        float mapHeight = data(index_x, index_y);
        if (mapHeight == mapHeight)
        {
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
        if (matches_ == points)
        {
          shifted_mean_ = shifted_mean_/matches_;
          reference_mean_ = reference_mean_/matches_;
          return true; 
        }
      }
      return false;
    }
    else
    {
      shifted_mean_ = shifted_mean_/matches_;
      reference_mean_ = reference_mean_/matches_;
      return true;
    }
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
  float minHeight = map_min_;
  if (reference_min_ < minHeight) { minHeight = reference_min_; }

  float maxHeight = map_max_;
  if (reference_max_ > maxHeight) { maxHeight = reference_max_; }

  int numberOfBins = ceil((maxHeight - minHeight)/0.03);

  float binWidth = (maxHeight - minHeight + 1e-6) / numberOfBins;

  std::vector <float> hist;
  std::vector <float> referenceHist;
  std::vector< std::vector <float> > jointHist;
  hist.clear();
  referenceHist.clear();
  jointHist.clear();
  for (int i = 0; i < numberOfBins; i++)
  {
    hist.push_back(0.0);
    referenceHist.push_back(0.0);
  }
  for (int i = 0; i < numberOfBins; i++)
  {
    jointHist.push_back(hist);
  }

  for (int i = 0; i < matches_; i++)
  {
    int i1 = (xy_shifted_[i] - minHeight) / binWidth;
    int i2 = (xy_reference_[i] - minHeight) / binWidth;
    hist[i1] += 1.0/matches_;
    referenceHist[i2] += 1.0/matches_;
    jointHist[i1][i2] += 1.0/matches_;
  }

  float entropy = 0;
  float referenceEntropy = 0;
  float jointEntropy = 0;

  for (int i = 0; i < numberOfBins; i++)
  {
    if (hist[i]!=0.0) { entropy += -hist[i]*log2(hist[i]); }
    if (referenceHist[i]!=0.0) { referenceEntropy += -referenceHist[i]*log2(referenceHist[i]); }

    for (int j = 0; j < numberOfBins; j++)
    {
      if (jointHist[i][j]!=0.0) 
      { 
        jointEntropy += -jointHist[i][j]*log2(jointHist[i][j]);
      }
    }
  }
  return (entropy+referenceEntropy)-jointEntropy;
}

float MapFitter::normalizedMutualInformation()
{
  float minHeight = map_min_;
  if (reference_min_ < minHeight) { minHeight = reference_min_; }

  float maxHeight = map_max_;
  if (reference_max_ > maxHeight) { maxHeight = reference_max_; }

  int numberOfBins = ceil((maxHeight - minHeight)/0.08); //128

  float binWidth = (maxHeight - minHeight + 1e-6) / numberOfBins;

  std::vector <float> hist;
  std::vector <float> referenceHist;
  std::vector< std::vector <float> > jointHist;
  hist.clear();
  referenceHist.clear();
  jointHist.clear();
  for (int i = 0; i < numberOfBins; i++)
  {
    hist.push_back(0.0);
    referenceHist.push_back(0.0);
  }
  for (int i = 0; i < numberOfBins; i++)
  {
    jointHist.push_back(hist);
  }

  for (int i = 0; i < matches_; i++)
  {
    int i1 = (xy_shifted_[i] - minHeight) / binWidth;
    int i2 = (xy_reference_[i] - minHeight) / binWidth;
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
    if (hist[i]!=0.0) { entropy += -hist[i]*log2(hist[i]); }
    if (referenceHist[i]!=0.0) { referenceEntropy += -referenceHist[i]*log2(referenceHist[i]); }

    for (int j = 0; j < numberOfBins; j++)
    {
      if (jointHist[i][j]!=0.0) 
      { 
        jointEntropy += -jointHist[i][j]*log2(jointHist[i][j]);
        weightSum += (numberOfBins-abs(i-j))*jointHist[i][j];
      }
    }
  }
  return (entropy+referenceEntropy)/jointEntropy;
}

void MapFitter::tfBroadcast(const ros::TimerEvent&) 
{
  broadcaster_.sendTransform( tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"/map", "/grid_map"));
}

void MapFitter::tfListener(const ros::TimerEvent&)
{
  tf::StampedTransform position;
  try { listener_.lookupTransform("/map", "/base", ros::Time(0), position); }
  catch (tf::TransformException ex) { //ROS_ERROR("%s",ex.what()); 
  return; }
  tf::Matrix3x3 m(position.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ros::Time pubTime2 = ros::Time::now();
  geometry_msgs::PointStamped correctPoint;
  correctPoint.point.x = position.getOrigin().x();
  correctPoint.point.y = position.getOrigin().y();
  correctPoint.point.z = fmod(yaw/M_PI*180+360, 360);
  correctPoint.header.stamp = pubTime2;
  correctPointPublisher_.publish(correctPoint);
}

} /* namespace */

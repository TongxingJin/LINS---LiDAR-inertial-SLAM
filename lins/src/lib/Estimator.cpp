// This file is part of LINS.
//
// Copyright (C) 2020 Chao Qin <cscharlesqin@gmail.com>,
// Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
// The Hong Kong University of Science and Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.

#include <Estimator.h>

namespace fusion {

int Scan::scan_counter_ = 0;

LinsFusion::LinsFusion(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {}

LinsFusion::~LinsFusion() { delete estimator; }

void LinsFusion::run() { initialization(); }

void LinsFusion::initialization() {
  // Implement an iterative-ESKF Kalman filter class
  estimator = new StateEstimator();

  // Subscribe to IMU, segmented point clouds, and map-refined odometry feedback
  subMapOdom_ = pnh_.subscribe<nav_msgs::Odometry>(
      LIDAR_MAPPING_TOPIC, 5, &LinsFusion::mapOdometryCallback, this);// TODO:只是进行了坐标系的统一，没有使用啊
  subImu = pnh_.subscribe<sensor_msgs::Imu>(IMU_TOPIC, 100,
                                            &LinsFusion::imuCallback, this);// 主要的内容
  subLaserCloud = pnh_.subscribe<sensor_msgs::PointCloud2>(
      "/segmented_cloud", 2, &LinsFusion::laserCloudCallback, this);// 缓存在map中
  subLaserCloudInfo = pnh_.subscribe<cloud_msgs::cloud_info>(
      "/segmented_cloud_info", 2, &LinsFusion::laserCloudInfoCallback, this);
  subOutlierCloud = pnh_.subscribe<sensor_msgs::PointCloud2>(
      "/outlier_cloud", 2, &LinsFusion::outlierCloudCallback, this);

  // Set publishers
  pubUndistortedPointCloud =
      pnh_.advertise<sensor_msgs::PointCloud2>("/undistorted_point_cloud", 1);

  pubCornerPointsSharp =
      pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 1);
  pubCornerPointsLessSharp =
      pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 1);
  pubSurfPointsFlat =
      pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 1);
  pubSurfPointsLessFlat =
      pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 1);

  pubLaserCloudCornerLast =
      pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
  pubLaserCloudSurfLast =
      pnh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
  pubOutlierCloudLast =
      pnh_.advertise<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2);
  pubLaserOdometry =
      pnh_.advertise<nav_msgs::Odometry>(LIDAR_ODOMETRY_TOPIC, 5);

  // Set types of the point cloud
  distortedPointCloud.reset(new pcl::PointCloud<PointType>());
  outlierPointCloud.reset(new pcl::PointCloud<PointType>());

  // Allocate measurement buffers for sensors
  imuBuf_.allocate(500);
  pclBuf_.allocate(3);
  outlierBuf_.allocate(3);
  cloudInfoBuf_.allocate(3);

  // Initialize IMU propagation parameters
  isImuCalibrated = CALIBARTE_IMU;
  ba_init_ = INIT_BA;
  bw_init_ = INIT_BW;
  ba_tmp_.setZero();
  bw_tmp_.setZero();
  sample_counter_ = 0;

  duration_ = 0.0;
  scan_counter_ = 0;

  ROS_INFO_STREAM("Subscribe to \033[1;32m---->\033[0m " << IMU_TOPIC);
  ROS_INFO_STREAM("Subscribe to \033[1;32m---->\033[0m " << LIDAR_TOPIC);
}

void LinsFusion::laserCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  // Add a new segmented point cloud
  pclBuf_.addMeas(laserCloudMsg, laserCloudMsg->header.stamp.toSec());
}
void LinsFusion::laserCloudInfoCallback(
    const cloud_msgs::cloud_infoConstPtr& cloudInfoMsg) {
  // Add segmentation information of the point cloud
  cloudInfoBuf_.addMeas(*cloudInfoMsg, cloudInfoMsg->header.stamp.toSec());
}

void LinsFusion::outlierCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  outlierBuf_.addMeas(laserCloudMsg, laserCloudMsg->header.stamp.toSec());
}

void LinsFusion::mapOdometryCallback(
    const nav_msgs::Odometry::ConstPtr& odometryMsg) {
  geometry_msgs::Quaternion geoQuat = odometryMsg->pose.pose.orientation;
  V3D t_yzx(odometryMsg->pose.pose.position.x,
            odometryMsg->pose.pose.position.y,
            odometryMsg->pose.pose.position.z);
  Q4D q_yzx(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
  V3D t_xyz = estimator->Q_yzx_to_xyz * t_yzx;// 只是坐标系之间存在一个变换
  Q4D q_xyz =
      estimator->Q_yzx_to_xyz * q_yzx * estimator->Q_yzx_to_xyz.inverse();// 坐标系之间和本地之间都存在

  V3D rpy = math_utils::Q2rpy(q_xyz);
}

void LinsFusion::imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg) {
  // Align IMU measurements from IMU frame to vehicle frame
  // two frames share same roll and pitch angles, but with a small
  // misalign-angle in the yaw direction
  acc_raw_ << imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y,
      imuMsg->linear_acceleration.z;
  gyr_raw_ << imuMsg->angular_velocity.x, imuMsg->angular_velocity.y,
      imuMsg->angular_velocity.z;
  misalign_euler_angles_ << deg2rad(0.0), deg2rad(0.0),
      deg2rad(IMU_MISALIGN_ANGLE);// 从IMU坐标系转换到车体坐标系
  alignIMUtoVehicle(misalign_euler_angles_, acc_raw_, gyr_raw_, acc_aligned_,
                    gyr_aligned_);// 把测量从IMU坐标系转换到车体坐标系

  // Add a new IMU measurement
  Imu imu(imuMsg->header.stamp.toSec(), acc_aligned_, gyr_aligned_);
  imuBuf_.addMeas(imu, imuMsg->header.stamp.toSec());// 测量增加到buffer（有的数据可能来不及处理而被直接删掉）

  // Trigger the Kalman filter
  performStateEstimation();
}

void LinsFusion::processFirstPointCloud() {
  // Use the most recent poing cloud to initialize the estimator
  pclBuf_.getLastTime(scan_time_);// 该imu信息也会是在scan_time后的第一条imu信息

  sensor_msgs::PointCloud2::ConstPtr pclMsg;
  pclBuf_.getLastMeas(pclMsg);
  distortedPointCloud->clear();
  pcl::fromROSMsg(*pclMsg, *distortedPointCloud);

  sensor_msgs::PointCloud2::ConstPtr outlierMsg;
  outlierBuf_.getLastMeas(outlierMsg);
  outlierPointCloud->clear();
  pcl::fromROSMsg(*outlierMsg, *outlierPointCloud);

  cloud_msgs::cloud_info cloudInfoMsg;
  cloudInfoBuf_.getLastMeas(cloudInfoMsg);// TODO:不需要考虑时间戳是否一致么？

  // The latest IMU measurement records the inertial information when the new
  // point cloud is recorded
  Imu imu;
  imuBuf_.getLastMeas(imu);

  // Initialize the iterative-ESKF by the first PCL
  estimator->processPCL(scan_time_, imu, distortedPointCloud, cloudInfoMsg,
                        outlierPointCloud);// 既有成员共享指针的拷贝，也有深拷贝

  // Clear all the PCLs before the initialization PCL
  pclBuf_.clean(estimator->getTime());// 在回调中push操作时也可能有pop的操作，这里在处理的时候也会进行pop
  cloudInfoBuf_.clean(estimator->getTime());
  outlierBuf_.clean(estimator->getTime());
}

void LinsFusion::publishTopics() {
  if (pubLaserCloudCornerLast.getNumSubscribers() != 0) {
    publishCloudMsg(pubLaserCloudCornerLast,
                    estimator->scan_last_->cornerPointsLessSharpYZX_,
                    ros::Time().fromSec(scan_time_), "/camera");
  }
  if (pubLaserCloudSurfLast.getNumSubscribers() != 0) {
    publishCloudMsg(pubLaserCloudSurfLast,
                    estimator->scan_last_->surfPointsLessFlatYZX_,
                    ros::Time().fromSec(scan_time_), "/camera");
  }
  if (pubOutlierCloudLast.getNumSubscribers() != 0) {
    publishCloudMsg(pubOutlierCloudLast,
                    estimator->scan_last_->outlierPointCloudYZX_,
                    ros::Time().fromSec(scan_time_), "/camera");
  }

  // Publish the estimated 6-DOF odometry by a YZX-frame convention (e.g. camera
  // frame convention), where Z points forward, X poins leftward, and Y poitns
  // upwards.
  // Notive that the estimator is performed in a XYZ-frame convention,
  // where X points forward, Y...leftward, Z...upward. Therefore, we have to
  // transforme the odometry from XYZ-convention to YZX-convention to meet the
  // mapping module's requirement.
  publishOdometryYZX(scan_time_);
}

// 至少是第2帧
bool LinsFusion::processPointClouds() {
  // 解析第一帧更新的激光点云
  // Obtain the next PCL
  pclBuf_.itMeas_ = pclBuf_.measMap_.upper_bound(estimator->getTime());// 第一个大于estimator的点云，可能存在多个
  sensor_msgs::PointCloud2::ConstPtr pclMsg = pclBuf_.itMeas_->second;
  scan_time_ = pclBuf_.itMeas_->first;
  distortedPointCloud->clear();
  pcl::fromROSMsg(*pclMsg, *distortedPointCloud);

  outlierBuf_.itMeas_ = outlierBuf_.measMap_.upper_bound(estimator->getTime());
  sensor_msgs::PointCloud2::ConstPtr outlierMsg = outlierBuf_.itMeas_->second;
  outlierPointCloud->clear();
  pcl::fromROSMsg(*outlierMsg, *outlierPointCloud);

  cloudInfoBuf_.itMeas_ =
      cloudInfoBuf_.measMap_.upper_bound(estimator->getTime());
  cloud_msgs::cloud_info cloudInfoMsg = cloudInfoBuf_.itMeas_->second;

  imuBuf_.getLastTime(last_imu_time_);
<<<<<<< HEAD
  if (last_imu_time_ < scan_time_) { // 最新的imu数据，要涵盖scan。如果不足以完成scan的一个周期，直接返回。
=======
  if (last_imu_time_ < scan_time_) { // 如果imu不能覆盖第一帧新点云，退出。TODO:应该在外面就提前退出的
>>>>>>> 256e42b4f2f0d6a16024af18f4b14fa2b94bcce7
    // ROS_WARN("Wait for more IMU measurement!");
    return false;
  }

  // Propagate IMU measurements between two consecutive scans
  int imu_couter = 0;
  // 如果estimator的时间戳还没有达到点云的时间戳
  // 而且有更新的imu数据（上面已经保证了）
  // 目的在于将estimator更新到点云的时刻
  while (estimator->getTime() < scan_time_ &&
         (imuBuf_.itMeas_ = imuBuf_.measMap_.upper_bound(
              estimator->getTime())) != imuBuf_.measMap_.end()) {
<<<<<<< HEAD
    // 计算更新时长dt，一般是imu的周期，横跨两个点云的imu会被使用两次
=======
    // 积分时长从上一时刻开始，到imu和点云中的较小时刻为止
    // 因此，跨点云的imu消息会被两次积分
>>>>>>> 256e42b4f2f0d6a16024af18f4b14fa2b94bcce7
    double dt =
        std::min(imuBuf_.itMeas_->first, scan_time_) - estimator->getTime();
    Imu imu = imuBuf_.itMeas_->second;
<<<<<<< HEAD
    estimator->processImu(dt, imu.acc, imu.gyr);// 车体坐标系下的imu
  }// 到这里把scan_time前的imu数据都用于预积分了
=======
    estimator->processImu(dt, imu.acc, imu.gyr);// 这里的imu已经是车体坐标系下的了
  }
>>>>>>> 256e42b4f2f0d6a16024af18f4b14fa2b94bcce7

  Imu imu;
  imuBuf_.getLastMeas(imu);

  // Update the iterative-ESKF using a new PCL
  estimator->processPCL(scan_time_, imu, distortedPointCloud, cloudInfoMsg,
                        outlierPointCloud);

  // Clear all measurements before the current time stamp
  imuBuf_.clean(estimator->getTime());
  pclBuf_.clean(estimator->getTime());
  cloudInfoBuf_.clean(estimator->getTime());
  outlierBuf_.clean(estimator->getTime());

  return true;
}

// 每次IMU的回调函数中都会尝试执行
void LinsFusion::performStateEstimation() {
  // 需要IMU和点云数据都具备才会执行
  if (imuBuf_.empty() || pclBuf_.empty() || cloudInfoBuf_.empty() ||
      outlierBuf_.empty())
    return;// 如果imu的时间戳没能覆盖第一个点云，也应该及时返回，不要再往下处理了
  
  if (!estimator->isInitialized()) {
    processFirstPointCloud();
    return;
  }

  // Iterate all PCL measurements in the buffer
<<<<<<< HEAD
  pclBuf_.getLastTime(last_scan_time_);// TODO:如果此时存在多个scan，直接处理最后一个
  while (!pclBuf_.empty() && estimator->getTime() < last_scan_time_) {
=======
  pclBuf_.getLastTime(last_scan_time_);// TODO:在处理的过程中可能收到新的点云，但并没有再更新// 如果存在至少一帧新激光，那么estimator需要被更新
  while (!pclBuf_.empty() && estimator->getTime() < last_scan_time_) { 
>>>>>>> 256e42b4f2f0d6a16024af18f4b14fa2b94bcce7
    TicToc ts_total;
    if (!processPointClouds()) break;
    double time_total = ts_total.toc();
    duration_ = (duration_ * scan_counter_ + time_total) / (scan_counter_ + 1);
    scan_counter_++;
    // ROS_INFO_STREAM("Pure-odometry processing time: " << duration_);
    publishTopics();

    // if (VERBOSE) {
    //   cout << "ba: " << estimator->globalState_.ba_.transpose() << endl;
    //   cout << "bw: " << estimator->globalState_.bw_.transpose() << endl;
    //   cout << "gw: " << estimator->globalState_.gn_.transpose() << endl;
    //   duration_ = (duration_ * scan_counter_ + time_total) / (scan_counter_ +
    //   1); scan_counter_++; ROS_INFO_STREAM(); cout << "Odometry: time: " <<
    //   duration_ << endl;
    // }
  }
}

void LinsFusion::alignIMUtoVehicle(const V3D& rpy, const V3D& acc_in,
                                   const V3D& gyr_in, V3D& acc_out,
                                   V3D& gyr_out) {
  M3D R = rpy2R(rpy);
  acc_out = R.transpose() * acc_in;
  gyr_out = R.transpose() * gyr_in;
}

void LinsFusion::publishOdometryYZX(double timeStamp) {
  laserOdometry.header.frame_id = "/camera_init";
  laserOdometry.child_frame_id = "/laser_odom";
  laserOdometry.header.stamp = ros::Time().fromSec(timeStamp);
  laserOdometry.pose.pose.orientation.x = estimator->globalStateYZX_.qbn_.x();
  laserOdometry.pose.pose.orientation.y = estimator->globalStateYZX_.qbn_.y();
  laserOdometry.pose.pose.orientation.z = estimator->globalStateYZX_.qbn_.z();
  laserOdometry.pose.pose.orientation.w = estimator->globalStateYZX_.qbn_.w();
  laserOdometry.pose.pose.position.x = estimator->globalStateYZX_.rn_[0];
  laserOdometry.pose.pose.position.y = estimator->globalStateYZX_.rn_[1];
  laserOdometry.pose.pose.position.z = estimator->globalStateYZX_.rn_[2];
  pubLaserOdometry.publish(laserOdometry);

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;
  laserOdometryTrans.frame_id_ = "/camera_init";
  laserOdometryTrans.child_frame_id_ = "/laser_odom";
  laserOdometryTrans.stamp_ = ros::Time().fromSec(timeStamp);
  laserOdometryTrans.setRotation(tf::Quaternion(
      estimator->globalStateYZX_.qbn_.x(), estimator->globalStateYZX_.qbn_.y(),
      estimator->globalStateYZX_.qbn_.z(),
      estimator->globalStateYZX_.qbn_.w()));
  laserOdometryTrans.setOrigin(tf::Vector3(estimator->globalStateYZX_.rn_[0],
                                           estimator->globalStateYZX_.rn_[1],
                                           estimator->globalStateYZX_.rn_[2]));
  tfBroadcaster.sendTransform(laserOdometryTrans);
}

// void LinsFusion::performImuBiasEstimation() {
//   Imu imu;
//   while (imuBuf_.getSize() != 0) {
//     imuBuf_.getFirstMeas(imu);
//     ba_tmp_ += imu.acc - V3D(0, 0, G0);
//     bw_tmp_ += imu.gyr;
//     sample_counter_++;
//     if (sample_counter_ == AVERAGE_NUMS) {
//       ba_init_ = ba_tmp_ * (1. / sample_counter_);
//       bw_init_ = bw_tmp_ * (1. / sample_counter_);
//       isImuCalibrated = true;
//       ROS_INFO_STREAM("Estimated IMU acceleration bias: \n "
//                       << ba_init_.transpose() << " and gyroscope bias: \n"
//                       << bw_init_.transpose());
//       ba_tmp_.setZero();
//       bw_tmp_.setZero();
//       sample_counter_ = 0;
//       break;
//     }
//     imuBuf_.clean(imu.time);
//     pclBuf_.clean(imu.time);
//     outlierBuf_.clean(imu.time);
//     cloudInfoBuf_.clean(imu.time);
//   }
// }

}  // namespace fusion

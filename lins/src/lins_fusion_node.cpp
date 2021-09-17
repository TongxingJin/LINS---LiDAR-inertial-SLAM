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

#include <ros/ros.h>

#include <parameters.h>
#include <Estimator.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lins_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("\033[1;32m---->\033[0m LINS Fusion Started.");

  // pnh通过参数服务器读取参数到params.cpp，所有include params.h的文件都可以访问
  // 所以，通过pnh可以一次性把所有参数都读到params声明的外部变量中，任何功能文件（例如estimator）所需的参数不再需要通过pnh获取
  // 只有fusion::LinsFusion lins由于需要回调消息，需要保存nodehandle。其成员变量estimator的初始化不再需要从nodehandle获取参数
  parameter::readParameters(pnh);

  fusion::LinsFusion lins(nh, pnh);
  lins.run();

  ros::spin();
  return 0;
}

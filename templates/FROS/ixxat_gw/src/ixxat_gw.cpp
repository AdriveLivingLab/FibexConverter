/**
 * @file
 * @brief
 * @authors
 * @date
 * */

#include "ros/ros.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "datahandler.h"
#include "datahandler.cpp"

using namespace ixxat_gw_{{channel_name}};


int main(int argc, char **argv)
{

  /* Initialize node */
  ros::init(argc, argv, "ixxat_gw_{{channel_name}}");
  ros::NodeHandle ixxat_nh("~");
  
  /* Init important node parameters */
  std::string port_num_gw;
  std::string ip_address_gw;
  
  if (!ixxat_nh.getParam("port_num_gw", port_num_gw))
  {
    ROS_INFO("Missing Portnumber to Gateway");
  }
  if (!ixxat_nh.getParam("ip_address_gw", ip_address_gw))
  {
    ROS_INFO("Missing IP Address of Gateway");
  }

  boost::asio::io_service io_service;

  DataHandler *dh = new DataHandler(port_num_gw, ip_address_gw, io_service);
  dh->SetRosNodeAndAdvertise(&ixxat_nh);
  dh->EstablishConnection();
  ros::Time::init();
  ROS_INFO("'------------------------------'");
  ROS_INFO("Success: Data decoder is running");
  ROS_INFO("'------------------------------'");
  
  while (ros::ok()) {
    dh->ReceiveGatewayData();
  }
  return 0;
}
/**
 * @file
 * @brief
 * @authors
 * @date
 * */

#include "rclcpp/rclcpp.hpp"
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
  rclcpp::init(argc, argv);
  boost::asio::io_service io_service;
  const rclcpp::NodeOptions options;

  auto node = std::make_shared<ixxat_gw_mlbevo_flexray::DataHandler>(io_service);
  node->SetRosNodeAndAdvertise();
  node->EstablishConnection();
  
  while (rclcpp::ok()) {
    node->ReceiveGatewayData();
  }
  return 0;
}
/** @file epos_node.cpp
 * Node file.
 *
 * This file contians impelmentation of node used to controle EPOS
 * controler and maxon servomotor in ALLO project
 */
 
#include "epos_driver/epos_driver.hpp"
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "EPOS_controler");
  ros::NodeHandle parameters("~");
  eposdriver eDriver(parameters);
  eDriver.On();
  eDriver.Main();
  return 0;
}

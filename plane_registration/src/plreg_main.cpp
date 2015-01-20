#include <iostream>
#include <ros/ros.h>

#include "plane_registration.h"

int main(int argc, char *argv[])
{
  using namespace KDL;

  std::cout << "kdl tester" << std::endl;
  ros::init(argc, argv, "plane_registration");
  ros::NodeHandle node;
  ros::Rate rate(30);   // 50 hz

  PlaneRegistration pr(&node);

  while (ros::ok())
  {
    ros::spinOnce();
    pr.update();
    rate.sleep();
  }

  return 0;
}

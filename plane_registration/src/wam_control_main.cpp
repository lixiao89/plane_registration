#include <iostream>
#include <ros/ros.h>

#include "wam_control.h"
// base frame  ^ x                           cutter frame     / y
//             |                                             /
//             |                                            /
//             |-------> z                    x <----------|
//            /                                            |
//           /                                             |
//          /                                              |
//         V y                                             v z




int main(int argc, char *argv[])
{
  using namespace KDL;

  std::cout << "kdl tester" << std::endl;
  ros::init(argc, argv, "kdl_parser_node");
  ros::NodeHandle node;
  ros::Rate rate(100);   // 50 hz

  WamMove ctrl(&node);

  while (ros::ok())
    {
      ros::spinOnce();
      ctrl.update();
      rate.sleep();
    }

  return 0;
}

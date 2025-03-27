#include <ow_core/types.h>
#include <ros/ros.h>

// need include here
#include <ow_msgs/DCMPointSetList.h>

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "test_publisher");
  ROS_INFO("starting test_publisher");
  
  ros::NodeHandle nh;

  ros::Publisher dcm_point_set_pub = nh.advertise<ow_msgs::DCMPointSet>("x_lin", 10);

  // create DCMPointSet
  ow::DCMPointSet dcm_point_set;
  dcm_point_set.dcm() << 1, 2, 3; 
 
  // DCMPointSet -> msg
  ow_msgs::DCMPointSet dcm_point_set_msg;
  dcm_point_set_msg = dcm_point_set;

  // msg -> DCMPointSet
  ow::DCMPointSet dcm_point_set_test;
  dcm_point_set_test = dcm_point_set_msg;

  //----------------------------------------------------------------------------

  // a list of steps
  ow::DCMPointSetList dcm_list;
  dcm_list.push_back(dcm_point_set);

  // list -> msg
  ow_msgs::DCMPointSetList dcm_list_msgs;
    for(size_t i = 0; i < dcm_list.size(); ++i)
      dcm_list_msgs.point_sets[i] = dcm_list[i];

  //----------------------------------------------------------------------------

  ros::Rate rate(30);
  while(ros::ok())
  { 
    dcm_point_set_pub.publish(dcm_point_set_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

#include <ow_core/types.h>
#include <ros/ros.h>
#include <ow_core/common/thread.h>

class Publisher : 
  public ow::Thread
{
public:
  typedef ow::Thread Base;

protected:
  bool stopped_;

public:
  Publisher() :
    Base(),
    stopped_(false)
  {
  }

  virtual ~Publisher()
  {
    ROS_WARN("Publisher destructor");
  }

  bool init(ros::NodeHandle& nh)
  {
    ROS_INFO("Publisher::init call");
    Base::start();
    return true;
  }

  virtual void run()
  {
    ROS_INFO("Publisher::run begin");

    ros::Rate rate(30);
    while(ros::ok() && !stopped_)
    { 
      ROS_INFO("Publisher::run looping");

      ros::spinOnce();
      rate.sleep();
    }

    ROS_WARN("Publisher::run end");
  }

  bool stop()
  {
    ROS_WARN("Publisher::stop call");
    if(Base::isRunning())
    {
      stopped_ = true;
    }
    return true;
  }

};

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "test_publisher");
  ROS_INFO("starting test_publisher");
  ros::NodeHandle nh;

  Publisher publisher;

  publisher.init(nh);

  ros::Rate rate(30);
  
  int i = 0;
  bool run = true;
  while(ros::ok() && run)
  { 
    ROS_INFO("main looping");

    if( i == 60 )
    {
      //publisher.stop();
      run = false;
    }

    ros::spinOnce();
    rate.sleep();

    i++;
  }

  publisher.stop();
  ROS_WARN("stopping test_publisher");
  return 0;
}

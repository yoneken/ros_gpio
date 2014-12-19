#include "ros/ros.h"
#include "ros_gpio/gpio.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ros_gpio");
  if (argc != 2)
  {
    ROS_INFO("usage: test_ros_gpio pin");
    return 1;
  }

  ros::NodeHandle n;

  ros::ServiceClient clientOpen = n.serviceClient<ros_gpio::OpenGpio>("open");
  ros_gpio::OpenGpio srvOpen;
  srvOpen.request.pin = atoi(argv[1]);
  if (!clientOpen.call(srvOpen))
  {
    ROS_ERROR("Failed to call service open(%d)", srvOpen.request.pin);
    return 1;
  }

  ros::ServiceClient clientSetDir = n.serviceClient<ros_gpio::SetGpioDir>("set_dir");
  ros_gpio::SetGpioDir srvSetDir;
  srvSetDir.request.pin = atoi(argv[1]);
  srvSetDir.request.direction = "out";
  if (!clientSetDir.call(srvSetDir))
  {
    ROS_ERROR("Failed to call service set_dir(%d)", srvSetDir.request.pin);
    return 1;
  }

  ros::ServiceClient clientWrite = n.serviceClient<ros_gpio::WriteGpio>("write");
  ros_gpio::WriteGpio srvWrite;
  srvWrite.request.pin = atoi(argv[1]);

  ros::Rate loop_rate(1);

  int count = 0;
  while(ros::ok){
    srvWrite.request.value = count%2;
    if (!clientWrite.call(srvWrite))
    {
      ROS_ERROR("Failed to call service write(%d)", srvWrite.request.pin);
      return 1;
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  ros::ServiceClient clientClose = n.serviceClient<ros_gpio::CloseGpio>("close");
  ros_gpio::CloseGpio srvClose;
  srvClose.request.pin = atoi(argv[1]);
  if (!clientClose.call(srvClose))
  {
    ROS_ERROR("Failed to call service close(%d)", srvClose.request.pin);
    return 1;
  }

  return 0;
}


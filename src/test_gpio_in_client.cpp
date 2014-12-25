#include "ros/ros.h"
#include "ros_gpio/service.h"
#include <cstdlib>

void stateChanged(const ros_gpio::GpioState &state)
{
  ROS_INFO("Pin %d state changed to %d", state.pin, state.value);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ros_gpio");
  if (argc != 2)
  {
    ROS_INFO("usage: test_ros_gpio pin");
    return 1;
  }

  ros::NodeHandle n;

  ros::ServiceClient clientOpen = n.serviceClient<ros_gpio::OpenGpio>("open_gpio");
  ros_gpio::OpenGpio srvOpen;
  srvOpen.request.pin = atoi(argv[1]);
  if (!clientOpen.call(srvOpen))
  {
    ROS_ERROR("Failed to call service open(%d)", srvOpen.request.pin);
    return 1;
  }

  ros::ServiceClient clientSetDir = n.serviceClient<ros_gpio::SetGpioDir>("set_gpio_dir");
  ros_gpio::SetGpioDir srvSetDir;
  srvSetDir.request.pin = atoi(argv[1]);
  srvSetDir.request.direction = "in";
  if (!clientSetDir.call(srvSetDir))
  {
    ROS_ERROR("Failed to call service set_dir(%d)", srvSetDir.request.pin);
    return 1;
  }

  ros::ServiceClient clientSetMode = n.serviceClient<ros_gpio::SetGpioMode>("set_gpio_mode");
  ros_gpio::SetGpioMode srvSetMode;
  srvSetMode.request.pin = atoi(argv[1]);
  srvSetMode.request.mode = "pullup";
  if (!clientSetMode.call(srvSetMode))
  {
    ROS_ERROR("Failed to call service set_mode(%d)", srvSetMode.request.pin);
    return 1;
  }

  ros::ServiceClient clientRead = n.serviceClient<ros_gpio::ReadGpio>("read_gpio");
  ros_gpio::ReadGpio srvRead;
  srvRead.request.pin = atoi(argv[1]);
  if (!clientRead.call(srvRead))
  {
    ROS_ERROR("Failed to call service read(%d)", srvRead.request.pin);
    return 1;
  }

  ros::Subscriber sub = n.subscribe("gpio_state", 1000, stateChanged);

  ros::spin();

  ros::ServiceClient clientClose = n.serviceClient<ros_gpio::CloseGpio>("close_gpio");
  ros_gpio::CloseGpio srvClose;
  srvClose.request.pin = atoi(argv[1]);
  if (!clientClose.call(srvClose))
  {
    ROS_ERROR("Failed to call service close(%d)", srvClose.request.pin);
    return 1;
  }

  return 0;
}


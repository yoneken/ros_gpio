#include "ros/ros.h"
#include "ros_gpio/service.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pwm_client");
  if (argc != 2)
  {
    ROS_INFO("usage: test_pwm_client pin");
    return 1;
  }

  ros::NodeHandle n;

  ros::ServiceClient clientOpen = n.serviceClient<ros_gpio::OpenPwm>("open_pwm");
  ros_gpio::OpenPwm srvOpen;
  srvOpen.request.pin = atoi(argv[1]);
  if (!clientOpen.call(srvOpen))
  {
    ROS_ERROR("Failed to call service open_pwm(%d)", srvOpen.request.pin);
    return 1;
  }

  ros::ServiceClient clientSetDir = n.serviceClient<ros_gpio::SetPwmPeriod>("set_pwm_period");
  ros_gpio::SetPwmPeriod srvSetDir;
  srvSetDir.request.pin = atoi(argv[1]);
  srvSetDir.request.us = 20*1000;
  if (!clientSetDir.call(srvSetDir))
  {
    ROS_ERROR("Failed to call service set_pwm_period(%d)", srvSetDir.request.pin);
    return 1;
  }

  ros::ServiceClient clientStart = n.serviceClient<ros_gpio::StartPwm>("start_pwm");
  ros_gpio::StartPwm srvStart;
  srvStart.request.pin = atoi(argv[1]);
  if (!clientStart.call(srvStart))
  {
    ROS_ERROR("Failed to call service start_pwm(%d)", srvStart.request.pin);
    return 1;
  }

  ros::ServiceClient clientWrite = n.serviceClient<ros_gpio::WritePwm>("write_pwm");
  ros_gpio::WritePwm srvWrite;
  srvWrite.request.pin = atoi(argv[1]);

  ros::Rate loop_rate(10);

  int count = 0;
  while(ros::ok){
    srvWrite.request.percent = 1.0/15.0 + 1.0/15.0/20*count;
    if (!clientWrite.call(srvWrite))
    {
      ROS_ERROR("Failed to call service write_pwm(%d)", srvWrite.request.pin);
      return 1;
    }

    ros::spinOnce();

    loop_rate.sleep();
    if(count < 20){
      ++count;
    }else{
      count = 0;
    }
  }

  ros::ServiceClient clientStop = n.serviceClient<ros_gpio::StopPwm>("stop_pwm");
  ros_gpio::StopPwm srvStop;
  srvStop.request.pin = atoi(argv[1]);
  if (!clientStop.call(srvStop))
  {
    ROS_ERROR("Failed to call service start_pwm(%d)", srvStop.request.pin);
    return 1;
  }

  ros::ServiceClient clientClose = n.serviceClient<ros_gpio::ClosePwm>("close_pwm");
  ros_gpio::ClosePwm srvClose;
  srvClose.request.pin = atoi(argv[1]);
  if (!clientClose.call(srvClose))
  {
    ROS_ERROR("Failed to call service close_pwm(%d)", srvClose.request.pin);
    return 1;
  }

  return 0;
}


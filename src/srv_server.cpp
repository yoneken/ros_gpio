/*
 * Copyright (C) 2014, Kenta Yonekura<yoneken@ieee.org>.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_gpio/service.h"
#include "ros_gpio/gpio.h"
#include "ros_gpio/pwm.h"
#include "ros_gpio/uart.h"
#include "ros_gpio/internal.h"
#include <map>
#include <mraa/gpio.hpp>
#include <mraa/common.hpp>

const char *funcname[] = {"Aio", "Gpio", "i2c", "Pwm", "Spi", "Uart"};

std::map<int, int> pin_manager;

bool checkDuplicate(int pin)
{
  std::map<int, int>::const_iterator it = pin_manager.find(pin);
  if(it != pin_manager.end()){
   	ROS_ERROR("Port %d is already initialized as %s.", pin, funcname[(int)it->second]);
    return true;
  }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpio_server");

  ros::NodeHandle n;

// %Tag(SERVICE)%
  ros::ServiceServer srvOpenGpio = n.advertiseService("open_gpio", openGpio);
  ros::ServiceServer srvCloseGpio = n.advertiseService("close_gpio", closeGpio);
  ros::ServiceServer srvWriteGpio = n.advertiseService("write_gpio", writeGpio);
  ros::ServiceServer srvReadGpio = n.advertiseService("read_gpio", readGpio);
  ros::ServiceServer srvSetGpioDir = n.advertiseService("set_gpio_dir", setGpioDir);
  ros::ServiceServer srvSetGpioMode = n.advertiseService("set_gpio_mode", setGpioMode);

  ros::ServiceServer srvOpenPwm = n.advertiseService("open_pwm", openPwm);
  ros::ServiceServer srvClosePwm = n.advertiseService("close_pwm", closePwm);
  ros::ServiceServer srvWritePwm = n.advertiseService("write_pwm", writePwm);
  ros::ServiceServer srvReadPwm = n.advertiseService("read_pwm", readPwm);
 	ros::ServiceServer srvSetPwmPeriod = n.advertiseService("set_pwm_period", setPwmPeriod);
  ros::ServiceServer srvSetPwmPulseWidth = n.advertiseService("set_pwm_pulsewidth", setPwmPulseWidth);
 	ros::ServiceServer srvSetPwmDuty_ms = n.advertiseService("set_pwm_duty_ms", setPwmDuty_ms);
  ros::ServiceServer srvSetPwmDuty_percent = n.advertiseService("set_pwm_duty_percent", setPwmDuty_percent);
  ros::ServiceServer srvStartPwm = n.advertiseService("start_pwm", startPwm);
  ros::ServiceServer srvStopPwm = n.advertiseService("stop_pwm", stopPwm);

  ros::ServiceServer srvOpenUart = n.advertiseService("open_uart", openUart);
  ros::ServiceServer srvCloseUart = n.advertiseService("close_uart", closeUart);
  // %EndTag(SERVICE)%

// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

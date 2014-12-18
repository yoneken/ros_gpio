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
#include "ros_gpio/ros_gpio.h"
#include <map>
#include <mraa/gpio.hpp>
#include <mraa/common.hpp>

std::map<int, mraa::Gpio*> gpios;

bool openGpio(ros_gpio::OpenGpio::Request &req,
              ros_gpio::OpenGpio::Response &res)
{
  mraa::Gpio *gpio = new mraa::Gpio((int)req.pin);
  if(gpio == NULL){
    res.result = MRAA_ERROR_UNSPECIFIED;
    ROS_ERROR("failed to open gpio(%d)", (int)req.pin);
    return false;
  }
  gpios.insert(std::map<int, mraa::Gpio*>::value_type((int)req.pin, gpio));
  res.result = MRAA_SUCCESS;

  ROS_INFO("opened gpio(%d)", (int)req.pin);
  return true;
}

bool closeGpio(ros_gpio::CloseGpio::Request &req,
               ros_gpio::CloseGpio::Response &res)
{
  std::map<int, mraa::Gpio*>::const_iterator it = gpios.find((int)req.pin);
  if(it == gpios.end()){
    ROS_WARN("tried to close un-initialized gpio(%d)", (int)req.pin);
  }else{
    gpios.erase((*it).first);
  }

  ROS_INFO("closed gpio(%d)", (int)req.pin);
  return true;
}

bool writeGpio(ros_gpio::WriteGpio::Request &req,
               ros_gpio::WriteGpio::Response &res)
{
  std::map<int, mraa::Gpio*>::const_iterator it = gpios.find((int)req.pin);
  if(it == gpios.end()){
    ROS_ERROR("tried to write un-initialized gpio(%d)", (int)req.pin);
    return false;
  }else{
    res.result = (*it).second->write((int)req.value);
    ROS_INFO("wrote gpio(%d) : %d", (int)req.pin, (int)req.value);
  }
  return true;
}

bool readGpio(ros_gpio::ReadGpio::Request &req,
              ros_gpio::ReadGpio::Response &res)
{
  std::map<int, mraa::Gpio*>::const_iterator it = gpios.find((int)req.pin);
  if(it == gpios.end()){
    ROS_ERROR("tried to read un-initialized gpio(%d)", (int)req.pin);
    return false;
  }else{
    res.result = (*it).second->read();
    ROS_INFO("read gpio(%d) : %d", (int)req.pin, (int)res.result);
  }
  return true;
}

bool setGpioMode(ros_gpio::SetGpioMode::Request &req,
                 ros_gpio::SetGpioMode::Response &res)
{
  std::map<int, mraa::Gpio*>::const_iterator it = gpios.find((int)req.pin);
  if(it == gpios.end()){
    if(req.mode == "strong"){
      res.result = (*it).second->mode(mraa::MODE_STRONG);
      ROS_INFO("set gpio mode as strong(%d)", (int)req.pin);
   }else if(req.mode == "pullup"){
      res.result = (*it).second->mode(mraa::MODE_PULLUP);
      ROS_INFO("set gpio mode as pull up(%d)", (int)req.pin);
    }else if(req.mode == "pulldown"){
      res.result = (*it).second->mode(mraa::MODE_PULLDOWN);
      ROS_INFO("set gpio mode as pull down(%d)", (int)req.pin);
    }else if(req.mode == "hiz"){
      res.result = (*it).second->mode(mraa::MODE_HIZ);
      ROS_INFO("set gpio mode as hi-impedance(%d)", (int)req.pin);
    }else{
      ROS_ERROR("Could not set gpio mode except \"strong\", \"pullup\", \"pulldown\", or \"hiz\" : gpio(%d)", (int)req.pin);
      return false;
    }
  }
  return true;
}

bool setGpioDir(ros_gpio::SetGpioDir::Request &req,
                ros_gpio::SetGpioDir::Response &res)
{
  std::map<int, mraa::Gpio*>::const_iterator it = gpios.find((int)req.pin);
  if(it == gpios.end()){
    ROS_ERROR("tried to read un-initialized gpio(%d)", (int)req.pin);
    return false;
  }else{
    if(req.direction == "in"){
      res.result = (*it).second->dir(mraa::DIR_IN);
      ROS_INFO("set gpio direction as input(%d)", (int)req.pin);
    }else if(req.direction == "out"){
      res.result = (*it).second->dir(mraa::DIR_OUT);
      ROS_INFO("set gpio direction as output(%d)", (int)req.pin);
    }else{
      ROS_ERROR("Could not set gpio direction except \"in\" or \"out\" : gpio(%d)", (int)req.pin);
      return false;
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpio_server");

  ros::NodeHandle n;

// %Tag(SERVICE)%
  ros::ServiceServer srvOpen = n.advertiseService("open", openGpio);
  ros::ServiceServer srvClose = n.advertiseService("close", closeGpio);
  ros::ServiceServer srvWrite = n.advertiseService("write", writeGpio);
  ros::ServiceServer srvRead = n.advertiseService("read", readGpio);
  ros::ServiceServer srvSetDir = n.advertiseService("set_dir", setGpioDir);
  ros::ServiceServer srvSetMode = n.advertiseService("set_mode", setGpioMode);
// %EndTag(SERVICE)%

// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

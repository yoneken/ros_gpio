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
#include "ros_gpio/service.h"
#include "ros_gpio/gpio.h"
#include "ros_gpio/internal.h"
#include <map>
#include <mraa/gpio.hpp>
#include <mraa/common.hpp>

extern std::map<int, int> pin_manager;

std::map<int, mraa::Gpio*> gpios;

ros::Publisher state_pub;

void changedGpioState(void *arg)
{
  mraa::Gpio *dev = (mraa::Gpio *)arg;

  ros_gpio::GpioState state;
  state.pin = dev->getPin();
  state.value = dev->read();
  ROS_INFO("Published: pin %d state changed to %d", state.pin, state.value);

  state_pub.publish(state);
}

bool openGpio(ros_gpio::OpenGpio::Request &req,
              ros_gpio::OpenGpio::Response &res)
{
  if(checkDuplicate((int)req.pin)) return false;

   mraa::Gpio *gpio = new mraa::Gpio((int)req.pin);
  if(gpio == NULL){
    res.result = MRAA_ERROR_UNSPECIFIED;
    ROS_ERROR("failed to open gpio(%d)", (int)req.pin);
    return false;
  }
  gpios.insert(std::map<int, mraa::Gpio*>::value_type((int)req.pin, gpio));
  res.result = MRAA_SUCCESS;

  pin_manager.insert(std::map<int, int>::value_type((int)req.pin, FUNC_GPIO));

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
    it->second->isrExit();

    gpios.erase(it->first);

    pin_manager.erase((int)req.pin);
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
    res.result = it->second->write((int)req.value);
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
    res.result = it->second->read();
    ROS_INFO("read gpio(%d) : %d", (int)req.pin, (int)res.result);

    it->second->isr(mraa::EDGE_BOTH, changedGpioState, (void *)it->second);
  }
  return true;
}

bool setGpioMode(ros_gpio::SetGpioMode::Request &req,
                 ros_gpio::SetGpioMode::Response &res)
{
  std::map<int, mraa::Gpio*>::const_iterator it = gpios.find((int)req.pin);
  if(it == gpios.end()){
    ROS_ERROR("tried to set mode un-initialized gpio(%d)", (int)req.pin);
    return false;
  }else{
    if(req.mode == "strong"){
      res.result = it->second->mode(mraa::MODE_STRONG);
      ROS_INFO("set gpio mode as strong(%d)", (int)req.pin);
    }else if(req.mode == "pullup"){
      res.result = it->second->mode(mraa::MODE_PULLUP);
      ROS_INFO("set gpio mode as pull up(%d)", (int)req.pin);
    }else if(req.mode == "pulldown"){
      res.result = it->second->mode(mraa::MODE_PULLDOWN);
      ROS_INFO("set gpio mode as pull down(%d)", (int)req.pin);
    }else if(req.mode == "hiz"){
      res.result = it->second->mode(mraa::MODE_HIZ);
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
    ROS_ERROR("tried to set direction un-initialized gpio(%d)", (int)req.pin);
    return false;
  }else{
    if(req.direction == "in"){
      res.result = it->second->dir(mraa::DIR_IN);
      ROS_INFO("set gpio direction as input(%d)", (int)req.pin);
			it->second->useMmap(true);
    }else if(req.direction == "out"){
      res.result = it->second->dir(mraa::DIR_OUT);
      ROS_INFO("set gpio direction as output(%d)", (int)req.pin);
			it->second->useMmap(true);
    }else{
      ROS_ERROR("Could not set gpio direction except \"in\" or \"out\" : gpio(%d)", (int)req.pin);
      return false;
    }
  }
  return true;
}
// %EndTag(FULLTEXT)%


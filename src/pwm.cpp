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
#include "ros_gpio/pwm.h"
#include "ros_gpio/internal.h"
#include <map>
#include <mraa/pwm.hpp>
#include <mraa/common.hpp>

extern std::map<int, int> pin_manager;

std::map<int, mraa::Pwm*> pwms;

bool openPwm(ros_gpio::OpenPwm::Request &req,
             ros_gpio::OpenPwm::Response &res)
{
	if(checkDuplicate((int)req.pin)) return false;

  mraa::Pwm *pwm = new mraa::Pwm((int)req.pin);
  if(pwm == NULL){
    res.result = MRAA_ERROR_UNSPECIFIED;
    ROS_ERROR("failed to open pwm(%d)", (int)req.pin);
    return false;
  }
  pwms.insert(std::map<int, mraa::Pwm*>::value_type((int)req.pin, pwm));
  res.result = MRAA_SUCCESS;

	pin_manager.insert(std::map<int, int>::value_type((int)req.pin, FUNC_PWM));

  ROS_INFO("opened pwm(%d)", (int)req.pin);
  return true;
}

bool closePwm(ros_gpio::ClosePwm::Request &req,
              ros_gpio::ClosePwm::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_WARN("tried to close un-initialized pwm(%d)", (int)req.pin);
  }else{
    pwms.erase(it->first);

	  pin_manager.erase((int)req.pin);
   }

  ROS_INFO("closed pwm(%d)", (int)req.pin);
  return true;
}

bool writePwm(ros_gpio::WritePwm::Request &req,
              ros_gpio::WritePwm::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_ERROR("tried to write un-initialized pwm(%d)", (int)req.pin);
    return false;
  }else{
    res.result = it->second->write((float)req.percent);
    ROS_INFO("wrote pwm(%d) : %f", (int)req.pin, (float)req.percent);
  }
  return true;
}

bool readPwm(ros_gpio::ReadPwm::Request &req,
             ros_gpio::ReadPwm::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_ERROR("tried to read un-initialized pwm(%d)", (int)req.pin);
    return false;
  }else{
    res.percent = it->second->read();
    ROS_INFO("read pwm(%d) : %f", (int)req.pin, (float)res.percent);
  }
  return true;
}

bool setPwmPeriod(ros_gpio::SetPwmPeriod::Request &req,
                  ros_gpio::SetPwmPeriod::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_ERROR("tried to set pwm period un-initialized pwm(%d)", (int)req.pin);
    return false;
  }else{
    res.result = it->second->period_us((unsigned int)req.us);
    ROS_INFO("set pwm period(%d) : %d [us]", (int)req.pin, (unsigned int)req.us);
  }
  return true;
}

bool setPwmPulseWidth(ros_gpio::SetPwmPulseWidth::Request &req,
                      ros_gpio::SetPwmPulseWidth::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_ERROR("tried to set pulse width un-initialized pwm(%d)", (int)req.pin);
    return false;
  }else{
    res.result = it->second->pulsewidth_us((unsigned int)req.us);
    ROS_INFO("set pwm pulse width(%d) : %d [us]", (int)req.pin, (unsigned int)req.us);
  }
  return true;
}

bool setPwmDuty_ms(ros_gpio::SetPwmDuty_ms::Request &req,
                   ros_gpio::SetPwmDuty_ms::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_ERROR("tried to set duty ms un-initialized pwm(%d)", (int)req.pin);
    return false;
  }else{
    res.result = it->second->config_ms((unsigned int)req.period_ms, (float)req.duty_ms);
    ROS_INFO("set pwm duty(%d) : %f [ms] / %d [ms]", (int)req.pin, (float)req.duty_ms, (unsigned int)req.period_ms);
  }
  return true;
}

bool setPwmDuty_percent(ros_gpio::SetPwmDuty_percent::Request &req,
                        ros_gpio::SetPwmDuty_percent::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_ERROR("tried to set duty percent un-initialized pwm(%d)", (int)req.pin);
    return false;
  }else{
    res.result = it->second->config_percent((unsigned int)req.period_ms, (float)req.percent);
    ROS_INFO("set pwm pulse width(%d) : %f \%/ %d [us]", (int)req.pin, (float)req.percent, (unsigned int)req.period_ms);
  }
  return true;
}

bool startPwm(ros_gpio::StartPwm::Request &req,
              ros_gpio::StartPwm::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_ERROR("tried to start un-initialized pwm(%d)", (int)req.pin);
    return false;
  }else{
    res.result = it->second->enable(true);
    ROS_INFO("start pwm (%d)", (int)req.pin);
  }
  return true;
}

bool stopPwm(ros_gpio::StopPwm::Request &req,
             ros_gpio::StopPwm::Response &res)
{
  std::map<int, mraa::Pwm*>::const_iterator it = pwms.find((int)req.pin);
  if(it == pwms.end()){
    ROS_ERROR("tried to stop un-initialized pwm(%d)", (int)req.pin);
    return false;
  }else{
    res.result = it->second->enable(false);
    ROS_INFO("stop pwm (%d)", (int)req.pin);
  }
  return true;
}
// %EndTag(FULLTEXT)%


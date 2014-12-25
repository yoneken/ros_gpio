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
#include "ros_gpio/uart.h"
#include "ros_gpio/internal.h"
#include <map>
#include <mraa/uart.hpp>
#include <mraa/common.hpp>

extern std::map<int, int> pin_manager;

std::map<int, mraa::Uart*> uarts;

bool openUart(ros_gpio::OpenUart::Request &req,
              ros_gpio::OpenUart::Response &res)
{
  if(checkDuplicate((int)req.pin)) return false;

  mraa::Uart *uart = new mraa::Uart((int)req.pin);
  if(uart == NULL){
    res.result = MRAA_ERROR_UNSPECIFIED;
    ROS_ERROR("failed to open uart(%d)", (int)req.pin);
    return false;
  }
  uarts.insert(std::map<int, mraa::Uart*>::value_type((int)req.pin, uart));
  res.result = MRAA_SUCCESS;

  pin_manager.insert(std::map<int, int>::value_type((int)req.pin, FUNC_UART));

  ROS_INFO("opened uart(%d)", (int)req.pin);
  return true;
}

bool closeUart(ros_gpio::CloseUart::Request &req,
               ros_gpio::CloseUart::Response &res)
{
  std::map<int, mraa::Uart*>::const_iterator it = uarts.find((int)req.pin);
  if(it == uarts.end()){
    ROS_WARN("tried to close un-initialized uart(%d)", (int)req.pin);
  }else{
    uarts.erase(it->first);

    pin_manager.erase((int)req.pin);
  }

  ROS_INFO("closed uart(%d)", (int)req.pin);
  return true;
}
// %EndTag(FULLTEXT)%


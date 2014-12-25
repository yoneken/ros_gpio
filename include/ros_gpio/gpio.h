#pragma once

void changedGpioState(void *arg);

bool openGpio(ros_gpio::OpenGpio::Request &req,
              ros_gpio::OpenGpio::Response &res);
bool closeGpio(ros_gpio::CloseGpio::Request &req,
               ros_gpio::CloseGpio::Response &res);
bool writeGpio(ros_gpio::WriteGpio::Request &req,
               ros_gpio::WriteGpio::Response &res);
bool readGpio(ros_gpio::ReadGpio::Request &req,
              ros_gpio::ReadGpio::Response &res);
bool setGpioMode(ros_gpio::SetGpioMode::Request &req,
                 ros_gpio::SetGpioMode::Response &res);
bool setGpioDir(ros_gpio::SetGpioDir::Request &req,
                ros_gpio::SetGpioDir::Response &res);


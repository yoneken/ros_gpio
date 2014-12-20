#pragma once

bool openPwm(ros_gpio::OpenPwm::Request &req,
             ros_gpio::OpenPwm::Response &res);

bool closePwm(ros_gpio::ClosePwm::Request &req,
              ros_gpio::ClosePwm::Response &res);

bool writePwm(ros_gpio::WritePwm::Request &req,
              ros_gpio::WritePwm::Response &res);

bool readPwm(ros_gpio::ReadPwm::Request &req,
             ros_gpio::ReadPwm::Response &res);

bool setPwmPeriod(ros_gpio::SetPwmPeriod::Request &req,
                  ros_gpio::SetPwmPeriod::Response &res);

bool setPwmPulseWidth(ros_gpio::SetPwmPulseWidth::Request &req,
                      ros_gpio::SetPwmPulseWidth::Response &res);

bool setPwmDuty_ms(ros_gpio::SetPwmDuty_ms::Request &req,
                   ros_gpio::SetPwmDuty_ms::Response &res);

bool setPwmDuty_percent(ros_gpio::SetPwmDuty_percent::Request &req,
                        ros_gpio::SetPwmDuty_percent::Response &res);

bool startPwm(ros_gpio::StartPwm::Request &req,
              ros_gpio::StartPwm::Response &res);

bool stopPwm(ros_gpio::StopPwm::Request &req,
             ros_gpio::StopPwm::Response &res);


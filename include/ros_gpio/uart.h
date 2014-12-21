#pragma once

bool openUart(ros_gpio::OpenUart::Request &req,
              ros_gpio::OpenUart::Response &res);

bool closeUart(ros_gpio::CloseUart::Request &req,
               ros_gpio::CloseUart::Response &res);


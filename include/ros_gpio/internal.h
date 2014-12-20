#pragma once

enum{
	FUNC_AIO = 0,
	FUNC_GPIO,
	FUNC_I2C,
	FUNC_PWM,
	FUNC_SPI,
	FUNC_UART
};

bool checkDuplicate(int pin);


#ifndef __PinConfig_HAL__
#define __PinConfig_HAL__
#include "gpio.hpp"

struct PinConfig {
    GPIO_TypeDef          *port;
    uint8_t                pin;
    GPIO::AlternateFunction af = GPIO::AlternateFunction::AF7;
};
#endif

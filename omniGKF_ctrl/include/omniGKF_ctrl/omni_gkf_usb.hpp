#pragma once

#include <iostream>

#include "serial/serial.h"

#define OMNI_GKF_USB_HEAD

namespace omni_gkf
{
    class OmniGKFUSB
    {
    public:
        OmniGKFUSB();
        ~OmniGKFUSB();

        void init(const std::string &portName, int baudRate);
        void read();
        void write(uint8_t cmd, int16_t data);

    private:
        serial::Serial port_;
    };
} // namespace omni_gkf
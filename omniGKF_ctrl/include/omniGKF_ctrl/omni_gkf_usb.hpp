#pragma once

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
        void init();
        void read();
        void write();

    private:
        serial::Serial port_;
    };
} // namespace omni_gkf
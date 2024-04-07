#pragma once

#include <libusb-1.0/libusb.h>

#define OMNI_GKF_USB_HEAD

namespace omni_gkf
{
    class OmniGKFUSB
    {
    public:
        OmniGKFUSB();
        ~OmniGKFUSB();

        void init();
        void read();
        void write();

    private:
    };
} // namespace omni_gkf
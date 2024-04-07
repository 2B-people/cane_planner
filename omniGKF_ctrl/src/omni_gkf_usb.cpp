#include "omniGKF_ctrl/omni_gkf_usb.hpp"

namespace omni_gkf
{
    OmniGKFUSB::OmniGKFUSB()
    {
    }

    OmniGKFUSB::~OmniGKFUSB()
    {
        if (port_.isOpen())
        {
            port_.close();
        }
    }

    void OmniGKFUSB::init(const std::string &portName, int baudRate)
    {
        port_.setPort(portName);
        port_.setBaudrate(baudRate);
        port_.open();
    }

    void OmniGKFUSB::read()
    {
    }

    void OmniGKFUSB::write()
    {
    }
} // namespace omni_gkf

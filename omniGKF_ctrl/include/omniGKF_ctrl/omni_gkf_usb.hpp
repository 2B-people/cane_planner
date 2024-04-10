#pragma once

#include <iostream>

#include "serial/serial.h"

// 帧头43->大写C
#define CMD_FRAME_HEADER 0x43
// cmd
#define CMD_VEL  0x01
#define CMD_POS  0x02
#define CMD_STOP 0x03
#define CMD_UPDATE 0x04


namespace omni_gkf
{
    class OmniGKFUSB
    {
    public:
        OmniGKFUSB();
        ~OmniGKFUSB();

        void init(const std::string &portName, int baudRate);
        void update();
        void write(uint8_t cmd, int16_t data);

        bool isAvailable();

      //float getHeading() const { return gkf_heading_; }
        int getHeading() const { return encode_; }
        std::vector<int16_t> getVelocity() const { return gkf_velocity_; }

    private:
        serial::Serial port_;
        float gkf_heading_;
        int encode_;
        std::vector<int16_t> gkf_velocity_;
        int read_flag_;

    };
} // namespace omni_gkf
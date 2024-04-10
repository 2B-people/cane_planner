#pragma once

#include <iostream>

#include "serial/serial.h"
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

// 帧头43->大写C
#define CMD_FRAME_HEADER 0x43
// cmd
#define CMD_VEL 0x01
#define CMD_POS 0x02
#define CMD_STOP 0x03
#define CMD_UPDATE 0x04

#define GKF_DATA_LEN 4

namespace omni_gkf
{
    class OmniGKFUSB
    {
    public:
        OmniGKFUSB();
        ~OmniGKFUSB();

        void update();
        void Set(uint8_t cmd, int16_t data);
        void haveData();
        bool isAvailable();

        virtual void init(const std::string &portName, int baudRate);

        float getHeading() const { return gkf_heading_; }
        int getEncoder() const { return encode_; }
        std::vector<int16_t> getVelocity() const { return gkf_velocity_; }

    protected:
        float gkf_heading_;
        int encode_;
        std::vector<int16_t> gkf_velocity_;
        int read_flag_;

    private:
        serial::Serial port_;

        virtual size_t write(std::vector<uint8_t> frame);
        virtual size_t read(std::string &line);
    };

    class OmniGKFUSB_LS : public OmniGKFUSB
    {
    public:
        OmniGKFUSB_LS() : OmniGKFUSB(){};
        ~OmniGKFUSB_LS(){};
        void init(const std::string &portName, int baudRate);

    private:
        LibSerial::SerialPort port_ls_;

        size_t write(std::vector<uint8_t> frame);
        size_t read(std::string &line);
    };
} // namespace omni_gkf
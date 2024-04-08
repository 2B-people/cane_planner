#include "omniGKF_ctrl/omni_gkf_usb.hpp"

namespace omni_gkf
{
    OmniGKFUSB::OmniGKFUSB()
    {
    }

    OmniGKFUSB::~OmniGKFUSB()
    {
        gkf_heading_ = 0.0;
        gkf_velocity_.clear();
        write(CMD_STOP, 0); // 停止运动

        if (port_.isOpen())
        {
            port_.flush();
            port_.close();
        }
    }

    void OmniGKFUSB::init(const std::string &portName, int baudRate)
    {
        port_.setPort(portName);
        port_.setBaudrate(baudRate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        port_.setTimeout(to);
        std::cout << "open port: " << portName << " baudrate: " << baudRate << std::endl;
        try
        {
            port_.open();
            if (port_.isOpen())
            {
                std::cout << "Serial Port initialized" << std::endl;
            }
        }
        catch (serial::IOException &e)
        {
            std::cout << "Unable to open port " << portName << std::endl;
        }

        read_flag_ = 0;
        gkf_heading_ = 0.0;
        gkf_velocity_.resize(2);
        gkf_velocity_[0] = 0;
        gkf_velocity_[1] = 0;

        // begin get data
        port_.flush();

        write(CMD_UPDATE, 0);
    }

    bool OmniGKFUSB::isAvailable()
    {
        if (read_flag_ > 2)
        {
            read_flag_ = 0;
            return true;
        }
        return false;
    }

    void OmniGKFUSB::write(uint8_t cmd, int16_t data)
    {
        std::vector<uint8_t> frame;
        frame.push_back(0x43);          // 帧头
        frame.push_back(cmd);           // 命令码
        if (cmd == 0x01 || cmd == 0x02) // 如果命令码是0x01或0x02，添加数据内容
        {
            frame.push_back((data >> 8) & 0xFF); // 数据高字节
            frame.push_back(data & 0xFF);        // 数据低字节
        }
        port_.write(frame);
    }

    void OmniGKFUSB::update()
    {
        std::string line;

        try
        {
            if (port_.readline(line))
            {
                if (line.size() < 3) // 如果数据太短，忽略
                    return;
                char id = line[0]; // 数据标识
                float value = 0.0;
                switch (id)
                {
                case 'E': // 编码器脉冲数
                    // 处理编码器脉冲数
                    value = std::stof(line.substr(2)); // 值
                    break;
                case 'A': // 编码器角度
                    // 处理编码器角度
                    gkf_heading_ = std::stof(line.substr(2)); // 值
                    read_flag_++;
                    break;
                case 'S': // 电机转子速度
                    // 处理电机转子速度
                    gkf_velocity_[0] = (float)std::stoi(line.substr(2)); // 值
                    read_flag_++;
                    break;
                case 'M': // 电机转子速度
                    // 处理电机转子速度
                    gkf_velocity_[1] = (float)std::stoi(line.substr(2)); // 值
                    read_flag_++;
                    break;
                }

                // test code
                // std::cout << "id: " << id << " value: " << value << std::endl;
            }
        }
        catch (serial::IOException &e)
        {
            std::cout << "Error " << e.what() << std::endl;
        }
    }
} // namespace omni_gkf

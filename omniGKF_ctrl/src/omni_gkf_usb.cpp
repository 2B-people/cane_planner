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

    void OmniGKFUSB::read()
    {
        std::string line;
        if (port_.readline(line)) // 读取一行数据
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
                value = std::stof(line.substr(2)); // 值
                break;
            case 'S': // 电机转子速度
                // 处理电机转子速度
                value = (float)std::stoi(line.substr(2)); // 值
                break;
            }

            // test code
            std::cout << "id: " << id << " value: " << value << std::endl;
        }
    }
} // namespace omni_gkf

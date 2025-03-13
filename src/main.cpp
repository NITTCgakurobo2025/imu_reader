#define _USE_MATH_DEFINES

#include <fcntl.h>
#include <limits.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>

#include "map_data.hpp"

using namespace std::chrono_literals;

class IMUReader : public rclcpp::Node {
public:
    IMUReader() : Node("imu_reader") {
        this->declare_parameter("output_topic", "imu");
        this->declare_parameter("frame_id", "imu_link");
        auto topic_name = get_parameter("output_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();

        this->timer_ = this->create_wall_timer(1ms, std::bind(&IMUReader::timerCallback, this));
        this->pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name, 10);

        if (!openSerialPort()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port");
            return;
        }
    }

    ~IMUReader() {
        // シリアルポートをクローズ
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    bool openSerialPort() {
        this->declare_parameter("serial_name", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 115200);
        auto device = get_parameter("serial_name").as_string();
        auto baudrate = get_parameter("baudrate").as_int();

        RCLCPP_INFO(this->get_logger(), "device  : %s", device.c_str());
        RCLCPP_INFO(this->get_logger(), "baudrate: %ld", baudrate);

        bool is_same_rate = false;
        for (const auto &rate : baudrate_list) {
            if (baudrate == rate) {
                is_same_rate = true;
                break;
            }
        }

        if (!is_same_rate) {
            RCLCPP_ERROR(this->get_logger(), "Baudrate is incorrect");
            return false;
        }

        // シリアルポートをオープン
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port");
            return false;
        }

        // シリアル通信の設定
        struct termios serial;
        tcgetattr(fd_, &serial);
        if (-1 == tcgetattr(fd_, &serial)) {
            close(fd_);
            RCLCPP_ERROR(this->get_logger(), "tcgetattr error!");
            return false;
        }
        int UART_BAUDRATE = baudrate_map.at(baudrate);
        struct termios term;

        if (-1 == tcgetattr(fd_, &term)) {
            close(fd_);
            RCLCPP_ERROR(this->get_logger(), "tcsetattr error!");
            return false;
        }

        cfsetospeed(&term, UART_BAUDRATE);
        cfsetispeed(&term, UART_BAUDRATE);
        term.c_cflag &= ~CSIZE;
        term.c_cflag |= CS8;      // データビット8
        term.c_cflag &= ~CRTSCTS; // ハードウェアフロー制御なし
        term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        term.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
        term.c_oflag &= ~OPOST;
        term.c_cc[VMIN] = 0; //	ノンブロッキング
        term.c_cc[VTIME] = 0;

        if (-1 == tcsetattr(fd_, TCSANOW, &term)) {
            close(fd_);
            RCLCPP_ERROR(this->get_logger(), "tcsetattr error!");
            return false;
        }
        return true;
    }

    void timerCallback() {
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
            return;
        }

        char tmp[LINE_MAX];

        int n = read(fd_, tmp, LINE_MAX - 1);

        if (n > 0) {
            // 前ループの残りのデータと結合
            tmp[n] = '\0';
            buffer_ += tmp;

            auto datas = splitString(buffer_, '\r');

            for (size_t i = 0; i < datas.size() - 1; ++i) {
                publishData(datas[i]);
            }

            buffer_ = datas.back();
        }
    }

    void publishData(const std::string &data) {
        constexpr double gravity = 9.8;
        constexpr double linear_const = gravity * 0.061 / 1000;        // [m/s^2]
        constexpr double angular_const = (M_PI / 180.0) * 35.0 / 1000; // [rad/s]

        // std::cout << "publishData: " << visualizeNewlines(data) << std::endl;

        // ','で分割してstd::vector<int>に格納
        std::vector<int> result;
        std::stringstream ss(data);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try {
                result.push_back(std::stoi(token));
            } catch (const std::invalid_argument &e) {
                std::cerr << "Invalid number: " << token << std::endl;
            } catch (const std::out_of_range &e) {
                std::cerr << "Number out of range: " << token << std::endl;
            }
        }

        if (result.size() < 6) {
            RCLCPP_ERROR(this->get_logger(), "Invalid data length: %ld", result.size());
            return;
        }

        sensor_msgs::msg::Imu pub_msg;
        pub_msg.header.stamp = this->get_clock()->now();
        pub_msg.header.frame_id = frame_id_;

        pub_msg.linear_acceleration.x = result[0] * linear_const;
        pub_msg.linear_acceleration.y = result[1] * linear_const;
        pub_msg.linear_acceleration.z = result[2] * linear_const;
        pub_msg.angular_velocity.x = result[3] * angular_const;
        pub_msg.angular_velocity.y = result[4] * angular_const;
        pub_msg.angular_velocity.z = result[5] * angular_const;

        // RCLCPP_INFO(this->get_logger(), "linear  x: %f y: %f z: %f", pub_msg.linear_acceleration.x,
        //             pub_msg.linear_acceleration.y, pub_msg.linear_acceleration.z);
        // RCLCPP_INFO(this->get_logger(), "angular x: %f y: %f z: %f", pub_msg.angular_velocity.x,
        //             pub_msg.angular_velocity.y, pub_msg.angular_velocity.z);

        pub_->publish(pub_msg);
    }

    std::vector<std::string> splitString(const std::string &input, char delimiter) {
        std::vector<std::string> result;
        size_t start = 0, end = 0;
        while ((end = input.find(delimiter, start)) != std::string::npos) {
            result.push_back(input.substr(start, end - start));
            start = end + 1;
        }
        if (start < input.size()) {
            result.push_back(input.substr(start));
        }
        return result;
    }

    // デバッグ用
    std::string visualizeNewlines(const std::string &input) {
        std::string output;
        for (char c : input) {
            if (c == '\n') {
                output += "[LF]";
            } else if (c == '\r') {
                output += "[CR]";
            } else {
                output += c;
            }
        }
        return output;
    }

    int fd_; // シリアルポートのファイルディスクリプタ
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    std::string frame_id_;
    std::string buffer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUReader>());
    rclcpp::shutdown();

    return 0;
}
#ifndef SERIAL_EXAMPLE_HPP
#define SERIAL_EXAMPLE_HPP

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"

#if defined(_MSC_VER)
#define PACKED_BEGIN __pragma(pack(push, 1))
#define PACKED_END   __pragma(pack(pop))
#define PACKED
#else
#define PACKED_BEGIN
#define PACKED_END
#define PACKED __attribute__((packed))
#endif

namespace io
{
    PACKED_BEGIN
    struct PACKED GimbalToVision
    {
        uint8_t head[2] = { 'S', 'W' };
        uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
        float q[4];    // wxyz顺序
        float yaw;
        float yaw_vel;
        float pitch;
        float pitch_vel;
        float bullet_speed;
        uint16_t bullet_count;  // 子弹累计发送次数
        uint16_t crc16;
    };
    PACKED_END

    static_assert(sizeof(GimbalToVision) <= 64);

    PACKED_BEGIN
    struct PACKED VisionToGimbal
    {
        uint8_t head[2] = { 'S', 'W' };
        uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
        float yaw;
        float yaw_vel;
        float yaw_acc;
        float pitch;
        float pitch_vel;
        float pitch_acc;
        uint16_t crc16;
    };
    PACKED_END

    static_assert(sizeof(VisionToGimbal) <= 64);

#undef PACKED_BEGIN
#undef PACKED_END
#undef PACKED

    enum class GimbalMode
    {
        IDLE,        // 空闲
        AUTO_AIM,    // 自瞄
        SMALL_BUFF,  // 小符
        BIG_BUFF     // 大符
    };

    struct GimbalState
    {
        float yaw;
        float yaw_vel;
        float pitch;
        float pitch_vel;
        float bullet_speed;
        uint16_t bullet_count;
    };

    class Gimbal
    {
    public:
        Gimbal(const std::string& config_path);

        ~Gimbal();

        GimbalMode mode() const;
        GimbalState state() const;
        std::string str(GimbalMode mode) const;

        void send(
            bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
            float pitch_acc);

        void send(io::VisionToGimbal VisionToGimbal);

    private:
        serial::Serial serial_;

        std::thread thread_;
        std::atomic<bool> quit_ = false;
        mutable std::mutex mutex_;

        GimbalToVision rx_data_;
        VisionToGimbal tx_data_;

        GimbalMode mode_ = GimbalMode::IDLE;
        GimbalState state_;

        bool read(uint8_t* buffer, size_t size);
        void read_thread();
        void reconnect();
    };

}  // namespace io

#endif  // SERIAL_EXAMPLE_HPP
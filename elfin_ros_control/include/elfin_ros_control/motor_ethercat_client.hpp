#pragma once

#include <stdint.h>
#include <ros/ros.h>
#include "elfin_ethercat_driver/elfin_ethercat_manager.h"

namespace elfin_ros_control
{
typedef struct
{
    std::string name;             //!< 轮子名字
    double directions;            //!< 旋转的正方向。
    int32_t slave_no;             //!< 电机slave id号
    double reduction_ratios;      //!< 减速机的速比，由外部给定。
    int32_t encoder_resolutions;  //!< 连接总线后读SDO获取。
    //!< 舵轮的零位直接写在电机驱动器里，驱动器返回的是扣除零位读数的值，因此此处zero_counts的值为0，即直接换算成角度。
    int32_t steering_zero_counts_;

    uint16_t kvp;
    uint16_t kvi;

    int32_t target_velocity;    // 0x60FF0020，速度模式使用。
    uint16_t control_word;      // 0x60400010
    int32_t actual_position;    // 0x60630020
    int32_t actual_velocity;    // 0x606C0020
    uint16_t status_word;       // 0x60410010
    uint16_t error_code_1;      // 0x26010010
    uint16_t error_code_2;      // 0x26020010

} motor_handle_t;

struct __attribute__((packed)) RxPDO
{
    int32_t target_velocity;    // 0x60FF0020，速度模式使用。
    uint16_t control_word;      // 0x60400010
};

struct __attribute__((packed)) TxPDO
{
    int32_t actual_position;    // 0x60630020
    int32_t actual_velocity;    // 0x606C0020
    uint16_t status_word;       // 0x60410010
    uint16_t error_code_1;      // 0x26010010
    uint16_t error_code_2;      // 0x26020010
};

class MotorEthercatClient
{
public:
    MotorEthercatClient(
        elfin_ethercat_driver::EtherCatManager* ecat_manager, motor_handle_t mh,
        int slave_no, const ros::NodeHandle& node, int rate);
    ~MotorEthercatClient();
    bool configure();
    bool enable();
    bool disable();
    void stopMotor();
    bool clearMotorFault();
    void setMotorCommand(double command);
    double getMotorVelState();
    uint32_t getMotorPosState();
    uint16_t getMotorStateWord();

private:
    elfin_ethercat_driver::EtherCatManager* ecat_manager_;
    int slave_no_;
    ros::NodeHandle node_;
    motor_handle_t motor_handle_;
    int rate_;
    bool is_enabled_, is_configured_;

    double count2velocity(int32_t count, int32_t encoder_resolution, double reduction_ratio);
    int32_t velocity2count(double velocity, int32_t encoder_resolution, double reduction_ratio);
    double getMotorCommand();
    void setMotorControlWord(uint16_t cw);
    uint16_t getMotorControlWord();

    template <typename T>
    T readInput(uint32_t index);
    template <typename T>
    T readOutput(uint32_t index);
    template <typename T>
    void write(uint32_t index, T value);
};
} // namespace moying_mor_hardware
#include "elfin_ros_control/motor_ethercat_client.hpp"
#include "elfin_ethercat_driver/elfin_ethercat_manager.h"
#include <ros/ros.h>

#include <cstring>
#include <thread>
#include <chrono>
#include <vector>

namespace elfin_ros_control
{
MotorEthercatClient::MotorEthercatClient(elfin_ethercat_driver::EtherCatManager* ecat_manager, motor_handle_t mh,
    int slave_no, const ros::NodeHandle& node, int rate)
  : ecat_manager_(ecat_manager),
    motor_handle_(mh),
    slave_no_(slave_no),
    node_(node),
    rate_(rate),
    is_enabled_(false),
    is_configured_(false)
{
  if(ecat_manager_ == nullptr)
  {
    ROS_ERROR(  "Cannot open EtherCat Manager.");
    exit(1);
  }
  // 获取编码器分辨率，配置电机参数时需要知道编码器分辨率。
  std::vector<uint8_t> bytes_encoder_wheel = ecat_manager_->readSDO(slave_no_, 0x6410, 3, 4);
  motor_handle_.encoder_resolutions = *(reinterpret_cast<uint32_t *>(bytes_encoder_wheel.data()));
  ROS_INFO(  "Slave %d Client encoder_resolutions: %d,", slave_no_, motor_handle_.encoder_resolutions);

  configure();
}

MotorEthercatClient::~MotorEthercatClient()
{
  disable();
}

bool MotorEthercatClient::configure()
{
  uint16_t kvp = motor_handle_.kvp;
  uint16_t kvi = motor_handle_.kvi;

  // 走轮控制模式。
  std::vector<uint8_t> bytes_mode_wheel(1);
  int8_t mode_wheel = -3;
  bytes_mode_wheel[0] = *reinterpret_cast<uint8_t *>(&mode_wheel);
  ecat_manager_->writeSDO(slave_no_, 0x6060, 0, bytes_mode_wheel);

  // 走轮速度环的比例增益。
  uint8_t * ptr_kvp_wheel = reinterpret_cast<uint8_t *>(&kvp);
  std::vector<uint8_t> bytes_kvp_wheel(2);
  bytes_kvp_wheel[0] = ptr_kvp_wheel[0];
  bytes_kvp_wheel[1] = ptr_kvp_wheel[1];
  ecat_manager_->writeSDO(slave_no_, 0x60F9, 1, bytes_kvp_wheel);

  // 走轮速度环的积分增益。
  uint8_t * ptr_kvi_wheel = reinterpret_cast<uint8_t *>(&kvi);
  std::vector<uint8_t> bytes_kvi_wheel(2);
  bytes_kvi_wheel[0] = ptr_kvi_wheel[0];
  bytes_kvi_wheel[1] = ptr_kvi_wheel[1];
  ecat_manager_->writeSDO(slave_no_, 0x60F9, 2, bytes_kvi_wheel);

  is_configured_ = true;
  return true;
}

bool MotorEthercatClient::enable()
{
  if (is_enabled_ == true)
  {
    ROS_WARN(  "Motor is already enabled.");
    return true;
  }
  if (is_configured_ != true)
  {
    return false;
  }
  stopMotor();
  clearMotorFault();

  // 上电
  uint16_t poweron = 0x0f;
  write(offsetof(RxPDO, control_word), poweron);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 查询状态字，使能位是否为1。
  bool all_operation_enable = false;
  int check_times = 10 * 1000 / rate_;  // 查询10个通信周期。
  while (check_times)
  {
    if (!(readInput<int16_t>(offsetof(TxPDO, status_word)) & 0x0004))
    {
      all_operation_enable = true;
      break;
    }
    check_times--;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / rate_));
  }

  if (all_operation_enable)
  {
    is_enabled_ = true;
    ROS_INFO(  "Enable Moying diff-drive vehicle.");
    stopMotor();
  }
  else
  {
    is_enabled_ = false;
    ROS_ERROR( 
                 "Cannot enable %d Slave Motor, not all joints reached Operation Enable status.", slave_no_);
    exit(1);
  }
  ROS_INFO(  " Slave%d Client Create.", slave_no_);
  return true;
}

bool MotorEthercatClient::disable()
{
  uint16_t poweroff=0x06;
  write(offsetof(RxPDO, control_word), poweroff);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 查询状态字，使能状态位是否为0。
  bool all_operation_disable = false;
  int check_times = 100;
  while (check_times)
  {
    if (!(readInput<uint16_t>(offsetof(TxPDO, status_word)) & 0x0004))
    {
      all_operation_disable = true;
      break;
    }
    check_times--;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (all_operation_disable)
  {
    is_enabled_ = false;
    ROS_INFO(  "Disable Slave %d Motor.", slave_no_);
    return true;
  }
  else
  {
    ROS_ERROR(  "Failed to disable Slave %d Motor.", slave_no_);
    return false;
  }
}

bool MotorEthercatClient::clearMotorFault()
{
  uint16_t clearfault = 0x86;
  write(offsetof(RxPDO, control_word), clearfault);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 查询状态字，Fault位是否为0。
  bool all_fault_cleared = false;
  int check_times = 100;
  while (check_times)
  {
    if (!(getMotorStateWord() & 0x0008))
    {
      all_fault_cleared = true;
      break;
    }
    check_times--;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return all_fault_cleared;
}

void MotorEthercatClient::setMotorCommand(double vel_cmd)
{
  // if (is_configured_ == false)
  // {
  //   ROS_WARN(  "Failed to set command, Slave %d Motor is not configured.", slave_no_);
  //   return;
  // }
  // if (is_enabled_ == false)
  // {
  //   ROS_WARN(  "Failed to set command, Slave %d Motor is not enabled.", slave_no_);
  //   return;
  // }
  int32_t act_vel = velocity2count(
      motor_handle_.directions * vel_cmd, motor_handle_.encoder_resolutions, motor_handle_.reduction_ratios);
  write(offsetof(RxPDO, target_velocity), act_vel);
}

double MotorEthercatClient::getMotorCommand()
{
  int32_t target_velocity = readOutput<int32_t>(offsetof(RxPDO, target_velocity));
  double vel = motor_handle_.directions * count2velocity(
      target_velocity, motor_handle_.encoder_resolutions, motor_handle_.reduction_ratios);
  return vel;
}

void MotorEthercatClient::setMotorControlWord(uint16_t cw)
{
  write(offsetof(RxPDO, control_word), cw);
}

uint16_t MotorEthercatClient::getMotorControlWord()
{
  return readOutput<uint16_t>(offsetof(RxPDO, control_word));
}

double MotorEthercatClient::getMotorVelState()
{
  uint32_t act_vel = readInput<uint32_t>(offsetof(TxPDO, actual_velocity));
  double vel = motor_handle_.directions * count2velocity(
      act_vel, motor_handle_.encoder_resolutions, motor_handle_.reduction_ratios);
  return vel;
}

uint32_t MotorEthercatClient::getMotorPosState()
{
  uint32_t act_pos = readInput<uint32_t>(offsetof(TxPDO, actual_position));
  return act_pos;
}

uint16_t MotorEthercatClient::getMotorStateWord()
{
  return readInput<uint16_t>(offsetof(TxPDO, status_word));
}

void MotorEthercatClient::stopMotor()
{
  setMotorCommand(0.0);
}

double MotorEthercatClient::count2velocity(int32_t count, int32_t encoder_resolution, double reduction_ratio)
{
  double coeff = reduction_ratio * (60.0 / (2.0 * M_PI) * 512.0 * encoder_resolution / 1875.0);
  return count / coeff;
}

int32_t MotorEthercatClient::velocity2count(double velocity, int32_t encoder_resolution, double reduction_ratio)
{
  double coeff = reduction_ratio * (60.0 / (2.0 * M_PI) * 512.0 * encoder_resolution / 1875.0);
  return std::lround(velocity * coeff);
}

template <typename T>
void MotorEthercatClient::write(uint32_t index, T value)
{
  uint8_t value8[sizeof(T)];
  memcpy(value8, &value, sizeof(T));
  for (size_t i = 0; i < sizeof(T); i++)
  {
    ecat_manager_->write(slave_no_, index + i, value8[i]);
  }
}

template <typename T>
T MotorEthercatClient::readInput(uint32_t index)
{
  uint8_t value8[sizeof(T)];
  for (size_t i = 0; i < sizeof(T); i++)
  {
    value8[i] = ecat_manager_->readInput(slave_no_, index + i);
  }
  return *(T*)value8;
}

template <typename T>
T MotorEthercatClient::readOutput(uint32_t index)
{
  uint8_t value8[sizeof(T)];
  for (size_t i = 0; i < sizeof(T); i++)
  {
    value8[i] = ecat_manager_->readOutput(slave_no_, index + i);
  }
  return *(T*)value8;
}

} // namespace moying_mor_hardware
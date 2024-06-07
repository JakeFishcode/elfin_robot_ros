#pragma once

#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "elfin_ethercat_driver/elfin_ethercat_manager.h"
#include "elfin_ros_control/motor_ethercat_client.hpp"

namespace elfin_ros_control
{
struct wheel_handle_t
{
  std::shared_ptr<MotorEthercatClient> motor_client;
  std::string name;
  double cmd;
  double vel_state;
  double effort_state;
  double pos_state;
}; 

class ChassisEthercatDriver
{
public:
  ChassisEthercatDriver(elfin_ethercat_driver::EtherCatManager* ecat_manager,
    const ros::NodeHandle& node);
  ~ChassisEthercatDriver();
  bool configure(const std::string & config);
  std::vector<wheel_handle_t> wheel_handles;

private:
  elfin_ethercat_driver::EtherCatManager* em_;
  ros::NodeHandle node_;
  int rate_;
};

} // namespace moying_mor_hardware
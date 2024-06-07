#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "elfin_ethercat_driver/elfin_ethercat_manager.h"
#include "elfin_ros_control/motor_ethercat_client.hpp"
#include "elfin_ros_control/chassis_ethercat_driver.hpp"

namespace elfin_ros_control 
{

ChassisEthercatDriver::ChassisEthercatDriver(elfin_ethercat_driver::EtherCatManager *ecat_manager,
    const ros::NodeHandle& node):
    em_(ecat_manager), node_(node), rate_(1000)
{
  bool is_configure_from_yaml=false;
  if(is_configure_from_yaml == false){
    wheel_handles.clear();
    std::vector<std::string> names = {"forward_right_wheel_joint", "forward_left_wheel_joint", "back_right_wheel_joint", "back_left_wheel_joint"};
    std::vector<double> dir = {1.0, -1.0, 1.0, -1.0};
    std::vector<int> slave_nos = {1, 2, 3, 4};

    motor_handle_t motor_settings;
    for (size_t i = 0; i < 4; i++)
    {
      // motor settings
      motor_settings.directions = dir[i];
      motor_settings.reduction_ratios = 21.0;
      motor_settings.encoder_resolutions = 10000;
      motor_settings.slave_no = slave_nos[i];
      motor_settings.kvp = 15;
      motor_settings.kvi = 1;
      std::shared_ptr<MotorEthercatClient> motor_client =
        std::make_shared<MotorEthercatClient>(ecat_manager, motor_settings, slave_nos[i], node_, rate_);

      // motor command init
      wheel_handle_t wheel_handle;
      wheel_handle.name = names[i];
      wheel_handle.cmd = 0.0;
      wheel_handle.vel_state = 0.0;
      wheel_handle.pos_state = 0.0;
      wheel_handle.motor_client = motor_client;
      wheel_handles.push_back(wheel_handle);
    }
  }
}

ChassisEthercatDriver::~ChassisEthercatDriver()
{
  // 资源清理
}

bool ChassisEthercatDriver::configure(const std::string & config)
{
  // 配置代码
  return true; 
}

} // namespace moying_mor_hardware
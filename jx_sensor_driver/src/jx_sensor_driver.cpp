#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/format.hpp>

#include "modbus_rtu_master.h"
#include "jx_sensor_msg/JXSensorMsg.h"

class JXSensorDriver
{
public:
  JXSensorDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~JXSensorDriver();
  void loop();

private:
  ros::NodeHandle nh_, private_nh_;
  jx_sensor_msg::JXSensorMsg msg_;
  ros::Publisher sensor_msg_pub_;
  int32_t baudrate_;
  std::string sensor1_com_;
  std::string sensor2_com_;
  ModbusRTUMaster *sensor_1_;
  ModbusRTUMaster *sensor_2_;
};

JXSensorDriver::JXSensorDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  private_nh_.param<int>("baudrate", baudrate_, 9600);
  private_nh_.param<std::string>("sensor1_com", sensor1_com_, "485_1");
  private_nh_.param<std::string>("sensor2_com", sensor2_com_, "485_2");
  
  sensor_1_ = new ModbusRTUMaster(sensor1_com_, baudrate_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  sensor_2_ = new ModbusRTUMaster(sensor2_com_, baudrate_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  sensor_msg_pub_ = nh_.advertise<jx_sensor_msg::JXSensorMsg>("jx_sensor_data", 10);
}

JXSensorDriver::~JXSensorDriver()
{
  free(sensor_1_);
  sensor_1_ = nullptr;

  free(sensor_2_);
  sensor_2_ = nullptr;
}

void JXSensorDriver::loop()
{
  uint16_t data[256] = {0};
  uint8_t ret = 0;

  msg_.header.stamp = ros::Time::now();

  // get CO  ppm
  ret = sensor_1_->GetMultipleRegisters(0x01, 0x0006, 0x0002, data);
  if (!ret) ROS_WARN("get co ppm faild!!");
  else msg_.co_ppm = uint32_t((data[0] << 16) | data[1]);

  // get CH4 ppm
  ret = sensor_1_->GetMultipleRegisters(0x02, 0x0006, 0x0002, data);
  if (!ret) ROS_WARN("get ch4 ppm faild!!");
  else msg_.ch4_ppm = uint32_t((data[0] << 16) | data[1]);

  // get CO2  ppm
  ret = sensor_1_->GetMultipleRegisters(0x03, 0x0005, 0x0002, data);
  if (!ret) ROS_WARN("get co2 ppm faild!!");
  else msg_.co2_ppm = uint32_t((data[0] << 16) | data[1]);

  // get pm2.5 and pm10 vol
  ret = sensor_2_->GetMultipleRegisters(0x01, 0x0004, 0x0006, data);
  if (!ret) ROS_WARN("get pm2.5 vol faild!!");
  else {
    msg_.pm2_5_vol =  float(data[0]);
    msg_.gas_concen = float(data[2]);
    msg_.pm10_vol =   float(data[5]);
  } 

  // get Atmospheric humidity and temperature
  ret = sensor_2_->GetMultipleRegisters(0x02, 0x0000, 0x0002, data);
  if (!ret) ROS_WARN("get Atmospheric humidity&temperature faild!!");
  else {
    msg_.atm_humid = float(data[0] / 10.0);
    msg_.atm_temp = float(data[1] / 10.0);
  }
  
  sensor_msg_pub_.publish(msg_);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jx_sensor_driver");
  ros::NodeHandle nh, private_nh("~");

  JXSensorDriver jx(nh, private_nh);

  ros::Rate r(1);

  while (ros::ok())
  {
    ros::spinOnce();
    jx.loop();
    r.sleep();
  }

  ROS_INFO("All finish");

  return 0;
}
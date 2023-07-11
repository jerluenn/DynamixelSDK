#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 37

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               11               // DXL1 ID
#define DXL2_ID               12               // DXL2 ID
#define DXL3_ID               31              // DXL2 ID
#define DXL4_ID               32               // DXL2 ID
#define DXL5_ID               41               // DXL2 ID
#define DXL6_ID               42               // DXL2 ID
#define BAUDRATE              1000000          // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;

void set_reference_velocity_callback(const std_msgs::Float64MultiArray& msg)

{

    std::cout << msg->data << "\n";

}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  ros::init(argc, argv, "velocity_controller_node");
  ros::NodeHandle nh;
  ros::Subscriber set_reference_velocity_sub = nh.subscribe("/set_reference_velocity", 10, set_reference_velocity_callback);
  ros::spin();

  portHandler->closePort();
  return 0;

  }

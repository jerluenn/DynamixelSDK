#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <deque>
#include <algorithm> 
#include <stdlib.h>


using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_MOVING_SPEED     32
#define ADDR_PRESENT_SPEED    39

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

#define Kp                     0.2
#define Ki                     0.05
#define Kd                     0.0

PortHandler * portHandler;
PacketHandler * packetHandler;
int error = 0; 
int prev_error = 0;
int ref_speed = 0;
int input = 0;
double time_step = 0.01;
double accum_error = 0; 
int16_t current_speed = 0;
int current_speed_filtered = 0;
std::deque<int> current_speed_sorted{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
std::deque<int> current_speed_unsorted{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  

void set_reference_velocity_callback(const std_msgs::Float64MultiArray& msg)

{

    ref_speed = msg.data[0];

}

void integral_wind_up()

{



}

void map_des_input_to_dynamixel_input_and_run(int input_value) 

{

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    int dynamixel_input;

    std::cout << "input value: " << input_value << "\n\n";

    if (input_value > 1023) 
    
    {

        input_value = 1022; 

    } 

    else if (input_value < -1023) 
    
    {

        input_value = -1022; 

    }

    if (input_value < 0)
    
    {

        dynamixel_input = 1023 - input_value;

    }

    else 
    
    {

        dynamixel_input = input_value;

    }

    std::cout << "dynamixel_input" << dynamixel_input << "\n\n";

    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler, DXL6_ID, ADDR_MOVING_SPEED, dynamixel_input, &dxl_error);

}

void runPID()
{

    prev_error = error; 
    error = ref_speed - current_speed_filtered;
    accum_error += error*time_step; 
    input = Kd*((error - prev_error)/time_step) + Ki*accum_error + Kp*error;

    map_des_input_to_dynamixel_input_and_run(input);

    std::cout << "Current speed: " << current_speed_filtered << "\n\n";
    // std::cout << "Input: " << input << "\n\n";
    // std::cout << "DER: " << Kd*((error - prev_error)/time_step) << "\n\n";
    // std::cout << "I: " << Ki*accum_error << "\n\n";
    // std::cout << "ERROR: " << Kp*error << "\n\n";



} 

void run_median_filter() 

{

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result  = packetHandler->read2ByteTxRx(
        portHandler, DXL6_ID, ADDR_PRESENT_SPEED, (uint16_t*)&current_speed, &dxl_error);

    std::cout << current_speed << "\n\n";

    current_speed_unsorted.pop_back();
    current_speed_unsorted.push_front(int(current_speed)); 
    current_speed_sorted = current_speed_unsorted; 
    std::sort(current_speed_sorted.begin(), current_speed_sorted.end());
    current_speed_filtered = current_speed_sorted[12];

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
    portHandler, DXL6_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL6_ID);
    return -1;
  }

  ros::init(argc, argv, "velocity_controller_node");
  ros::NodeHandle nh;
  ros::Subscriber set_reference_velocity_sub = nh.subscribe("/set_reference_velocity", 10, set_reference_velocity_callback);

    while (ros::ok()) 

    {
    
        run_median_filter();
        runPID(); 
        ros::spinOnce();
        
    }

  portHandler->closePort();
  return 0;

  }

/*
 * read_write.cpp
 *
 *  Created on: 2016. 2. 21.
 *      Author: leon
 */

//
// *********     Read and Write Example      *********
//
//
// Available DXL model on this example : All models using Protocol 1.0
// This example is designed for using a Dynamixel MX-28, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(support.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 1000000 [1M])
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk/dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30 //31
#define ADDR_MX_PRESENT_POSITION        36 //37
#define ADDR_MOVING_SPEED               32 //33
#define ADDR_PRESENT_SPEED              38 //39

//Add table
#define ADDR_MARGING_CW                 26 
#define ADDR_MARGING_CCW                27
#define ADDR_SLOPE_CW                   28
#define ADDR_SLOPE_CCW                  29
#define ADDR_PUNCH                      48 //49


// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define DXL_ID2                         2                  // Dynamixel ID: 2

// sync
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define PUNCH_VALUE                       100                 // Dynamixel will rotate with this value

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      400                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      600                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b                //ESC

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

#include <iostream>
#include <string.h>

void check_comm(dynamixel::PacketHandler *packetHandler, int dxl_comm_result, uint8_t dxl_error, std::string str){
  if (dxl_comm_result != COMM_SUCCESS)
  {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
  }
  else if (dxl_error != 0)
  {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
  }
  else
  {
    std::cout << str << std::endl;
    }
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  //Groupsyncwrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);


  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;                // addParam result

  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t param_goal_position[2];

  uint16_t dxl_present_position = 0, dxl2_present_position =0;              // Present position

  // Open port
  if (portHandler->openPort()){ printf("Succeeded to open the port!\n");
  }else{
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){ printf("Succeeded to change the baudrate!\n");
  }else{
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  //Dyna1
  // Enable DXL Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"Dynamixel has been successfully connected \n");

  // Change Punch
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PUNCH, PUNCH_VALUE, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"Punch has been succesfully change\n");

  // Change CW/CCW Marging
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_SLOPE_CW, 0x84, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"CW Slope has been succesfully change\n");
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_SLOPE_CCW, 0x20, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"CCW Slope has been succesfully change\n");

  // Change CW/CCW Marging
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MARGING_CW, 0x01, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"CW margin has been succesfully change\n");
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MARGING_CCW, 0x01, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"CCW margin has been succesfully change\n");

  //Dyna2
  // Enable DXL Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"Dynamixel has been successfully connected \n");

  // Change Punch
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_PUNCH, PUNCH_VALUE, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"Punch has been succesfully change\n");

  // Change CW/CCW Marging
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_SLOPE_CW, 0x84, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"CW Slope has been succesfully change\n");
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_SLOPE_CCW, 0x20, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"CCW Slope has been succesfully change\n");

  // Change CW/CCW Marging
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MARGING_CW, 0x01, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"CW margin has been succesfully change\n");
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MARGING_CCW, 0x01, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"CCW margin has been succesfully change\n");

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    param_goal_position[0] = DXL_LOBYTE(dxl_goal_position[index]);
    param_goal_position[1] = DXL_HIBYTE(dxl_goal_position[index]);
    // Write goal position
    // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
    // check_comm(packetHandler,dxl_comm_result,dxl_error,"");

    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID);
      return 0;
    }

    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID2, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID2);
      return 0;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    do
    {
      // Read present position
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
      check_comm(packetHandler,dxl_comm_result,dxl_error,"");

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);

      // Read present position2
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_PRESENT_POSITION, &dxl2_present_position, &dxl_error);
      check_comm(packetHandler,dxl_comm_result,dxl_error,"");

      printf("[ID2:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID2, dxl_goal_position[index], dxl2_present_position);

    }while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD) || (abs(dxl_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index == 0){
      index = 1;
    }else{
      index = 0;
    }
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"");

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  check_comm(packetHandler,dxl_comm_result,dxl_error,"");

  // Close port
  portHandler->closePort();

  return 0;
}


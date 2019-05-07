#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "dynamixel_sdk.h"   
#include "saictime.h"                               // Uses Dynamixel SDK library
#include "shm.h"
// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

#define STDIN_FILENO          0
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132     // 4 Bytes
#define ADDR_PRO_PRESENT_VELOCITY       128     // 4 Bytes
#define ADDR_PRO_PRESENT_PWM        124     // 2 Bytes
#define ADDR_PRO_PRESENT_CURRENT        126     // 2 Bytes

#define ADDR_PRO_GOAL_VELOCITY      104     // 4 Bytes
#define ADDR_PRO_OPERATING_MODE     11      // 1 Byte
#define ADDR_PRO_VELOCITY_LIMIT         44      // 4 Bytes

#define VELOCITY_MODE                 1
#define POSITION_MODE                 3
#define EXTENDED_POSITION_MODE      4

#define ADDR_PRO_Position_P_Gain    84      // 2 Bytes

#define ADDR_PRO_BULK_READ        124
#define LEN_PRO_BULK_READ       12

#define LEN_PRO_GOAL_POSITION     4
#define LEN_PRO_PRESENT_POSITION    4
#define LEN_PRO_PRESENT_VELOCITY    4
#define LEN_PRO_PRESENT_PWM       2
#define LEN_PRO_PRESENT_CURRENT     2


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID2                         10                   //
                
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2KQC4K-if00-port0"      // Check which port is being used on your controller

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      1024            // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      3072
              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{

  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;

}

int kbhit(void)
{

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
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position
  int32_t dxl_present_velocity = 0;  
 // uint16_t dxl_present_current = 0;               // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel2 has been successfully connected \n");
  }
  
  CTime timer;
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error);

  while(1)
  {
 //   printf("Press any key to continue! (or press ESC to quit!)\n");
 //   if (getch() == ESC_ASCII_VALUE)
 //     break;
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error);

printf("rate: %ehz\n", 1/timer.get_delta_t());
   // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error);

    // Change goal position
   
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}

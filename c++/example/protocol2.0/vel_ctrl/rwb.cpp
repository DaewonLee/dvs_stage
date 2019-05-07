// echo 1 | sudo tee -a /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "time.h"

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132			// 4 Bytes
#define ADDR_PRO_PRESENT_VELOCITY       128			// 4 Bytes
#define ADDR_PRO_PRESENT_PWM    	      124			// 2 Bytes
#define ADDR_PRO_PRESENT_CURRENT        126			// 2 Bytes
#define ADDR_PRO_GOAL_VELOCITY          104     // 4 Bytes

#define POSITION_P_GAIN                 84      // 2 Bytes default 800
#define OPERATING_MODE                  11      // 1 Byte

#define VELOCITY_LIMIT                  44      // 4 Bytes default 285

#define RETURN_DELAY                    9       // 1 Byte

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define NUM_OF_MOTORS			1

// Default setting
                
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2KQC4K-if00-port0"      // Check 
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
//#define DXL_MINIMUM_POSITION_VALUE      1024            // Dynamixel will rotate between this value
//#define DXL_MAXIMUM_POSITION_VALUE      3072

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

int print_results(int dxl_comm_result, int dxl_error, int id, dynamixel::PacketHandler *packetHandler)
{
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("[%d] %s\n",id, packetHandler->getTxRxResult(dxl_comm_result));
		return 0;
	}
	else if (dxl_error != 0)
	{
		printf("[%d] %s\n",id, packetHandler->getRxPacketError(dxl_error));
		return 0;
	}
	return 1;
}

int main()
{

  CTime time;
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 1;
  int index_vel = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int DXL_MINIMUM_POSITION_VALUE[NUM_OF_MOTORS] = {1024};
  int DXL_MAXIMUM_POSITION_VALUE[NUM_OF_MOTORS] = {3072};
  int dxl_goal_position[NUM_OF_MOTORS][2];         // Goal position
  int dxl_goal_velocity[NUM_OF_MOTORS][2];
  int vel_limit = 1023;
  int MODE = 1;
  int delay_time = 10;

for (int i=0; i < NUM_OF_MOTORS; i++)
{
  dxl_goal_position[i][0] = DXL_MINIMUM_POSITION_VALUE[i];
  dxl_goal_position[i][1] = DXL_MAXIMUM_POSITION_VALUE[i];
}

for (int i=0; i < NUM_OF_MOTORS; i++)
{
  dxl_goal_velocity[i][0] = -50;
  dxl_goal_velocity[i][1] = 50;
}

  int dxl_id[NUM_OF_MOTORS] = {10};

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position[NUM_OF_MOTORS] = {0};               // Present position
  int32_t dxl_present_velocity[NUM_OF_MOTORS] = {0};  
  uint16_t dxl_present_current[NUM_OF_MOTORS] = {0};
  uint16_t dxl_present_pwm[NUM_OF_MOTORS] = {0};
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

  //disable torque to change the control mode

  for (int i = 0; i<NUM_OF_MOTORS; i++)
{
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
  {
    printf("Dynamixel[%d] torque disabled \n",dxl_id[i]);
  } 
}


  // Set return delay time
 for (int i = 0; i<NUM_OF_MOTORS; i++)
{
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], RETURN_DELAY, delay_time, &dxl_error);
  if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
  {
    printf("Dynamixel[%d] delay time is set to %d \n",dxl_id[i], delay_time);
  } 
}


  // set control mode
  // 0: current, 1: velocity, 3: position, 4: extended position, 5: current-based, 16: PWM
  

  for (int i = 0; i<NUM_OF_MOTORS; i++)
{
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], OPERATING_MODE, MODE, &dxl_error);
  if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
  {
    printf("Dynamixel[%d] mode changed to %d \n",dxl_id[i], MODE);
  } 
}

  // set velocity limit 0~1023
  for (int i = 0; i<NUM_OF_MOTORS; i++)
{
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], VELOCITY_LIMIT, vel_limit, &dxl_error);
  if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
  {
    printf("Dynamixel[%d] velocity limit changed to %d \n",dxl_id[i], vel_limit);
  } 
}

   // set position p gain
  for (int i = 0; i<NUM_OF_MOTORS; i++)
{
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], POSITION_P_GAIN, 800, &dxl_error);
  if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
  {
    printf("Dynamixel[%d] position p gain changed \n",dxl_id[i]);
  } 
}



  // Enable Dynamixel Torque

for (int i = 0; i<NUM_OF_MOTORS; i++)
{
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
  {
    printf("Dynamixel[%d] torque enabled \n",dxl_id[i]);
  }	
}

  int desired_vel = 0;
  while(1)
  {

      if (getch() == ESC_ASCII_VALUE)
        break;

      while((abs(dxl_goal_position[0][index] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD))
      {
          printf("rate: %ehz\n",1/time.get_delta_t());
          printf("desired vel: %d\n",desired_vel);

          printf("dxl_goal_position[0][index]: %d\n",dxl_goal_position[0][index]);
          printf("dxl_present_position[0]: %d\n",dxl_present_position[0]);

          printf("error: %d\n",(dxl_goal_position[0][index] - dxl_present_position[0] ));

        if((dxl_goal_position[0][index] - dxl_present_position[0] ) > 0 )
        {
          desired_vel = 50;
        }
        else desired_vel = -50;

  if(MODE == 3) //position control
  {
      for (int i = 0; i<NUM_OF_MOTORS; i++)
      {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_POSITION, dxl_goal_position[i][index], &dxl_error);
        if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
        {
         // printf("Pos\n");
        }
      }
  }
  else if(MODE == 1) // velocity control
  {
    for (int i = 0; i<NUM_OF_MOTORS; i++)
      {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_VELOCITY, desired_vel, &dxl_error);
        if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
        {
        //  printf("Vel\n");
        }
      }
  }
      	
     for (int i=0; i<NUM_OF_MOTORS; i++)
    {  
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position[i], &dxl_error);
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity[i], &dxl_error);
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_PRESENT_PWM, (uint16_t*)&dxl_present_pwm[i], &dxl_error);
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_current[i], &dxl_error); 
      print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler);
    }
  	
  //printf("dt: %e",time.get_delta_t());
  printf("pos: %d\n", dxl_present_position[0]);
  printf("vel: %d\n", dxl_present_velocity[0]);

  }

  if(MODE == 1) // velocity control
  {
    desired_vel = 0;
    for (int i = 0; i<NUM_OF_MOTORS; i++)
      {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_VELOCITY, desired_vel, &dxl_error);
        if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
        {
        //  printf("Vel\n");
        }
      }
  }

  if (index == 0)
  {
    index = 1;
  }
  else
  {
    index = 0;
  }

   
}

  // Disable Dynamixel Torque

for (int i=0; i<NUM_OF_MOTORS; i++)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
	  {
		 printf("%d is off\n", dxl_id[i]);
	  }
}

  portHandler->closePort();

  return 0;
}


#ifndef CMOTOR_H_
#define CMOTOR_H_

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "saictime.h"

#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132			// 4 Bytes
#define ADDR_PRO_PRESENT_VELOCITY       128			// 4 Bytes
#define ADDR_PRO_PRESENT_PWM    	    124			// 2 Bytes
#define ADDR_PRO_PRESENT_CURRENT        126			// 2 Bytes
#define ADDR_PRO_GOAL_VELOCITY          104     // 4 Bytes

#define POSITION_P_GAIN                 84      // 2 Bytes default 800
#define OPERATING_MODE                  11      // 1 Byte
#define VELOCITY_LIMIT                  44      // 4 Bytes default 285
#define RETURN_DELAY                    9       // 1 Byte

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
#define NUM_OF_MOTORS					2
                
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2KQC4K-if00-port0"      // Check 
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     50                  // Dynamixel moving status threshold
#define ESC_ASCII_VALUE                 0x1b

#define CURRENT_MODE					0
#define VELOCITY_MODE					1
#define POSITION_MODE					3
#define EXTENDED_POSITION_MODE			4
#define CURRENT_BASED_MODE				5
#define PWM_MODE						16

  // 0: current, 1: velocity, 3: position, 4: extended position, 5: current-based, 16: PWM
  


class CMotor
{
public:
	CMotor();
	~CMotor(){}

	int index;
	int index_vel;
	int dxl_comm_result;             // Communication result
	int dxl_goal_position[NUM_OF_MOTORS][2];         // Goal position
	int dxl_goal_velocity[NUM_OF_MOTORS][2];
	int vel_limit;
	int MODE;
	int delay_time;
	int dxl_id[NUM_OF_MOTORS];

	int error_position[NUM_OF_MOTORS];
	int error_velocity[NUM_OF_MOTORS];

	int goal_position[NUM_OF_MOTORS];
	int goal_velocity[NUM_OF_MOTORS];

	int error_thresh;

	//dynamixel::PortHandler *portHandler;// = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  	//dynamixel::PacketHandler *packetHandler;// = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


	uint8_t dxl_error;                          // Dynamixel error
	int32_t dxl_present_position[NUM_OF_MOTORS];               // Present position
	int32_t dxl_present_velocity[NUM_OF_MOTORS];  
	uint16_t dxl_present_current[NUM_OF_MOTORS];
	uint16_t dxl_present_pwm[NUM_OF_MOTORS];

	int getch();
	int print_results(int dxl_comm_result, int dxl_error, int id, dynamixel::PacketHandler *packetHandler);
	int setup(int control_mode);
	void initialize();
	int read();
	void calc_error();
	int run(int* goal_vel, int* goal_pos);

	
};

#endif
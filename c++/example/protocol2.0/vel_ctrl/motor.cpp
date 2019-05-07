#include "motor.h"
static dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
static dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	
// echo 1 | sudo tee -a /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 
CMotor::CMotor() 
{
	
}

void CMotor::initialize()
{
	index = 1;
	index_vel = 0;
	dxl_comm_result = COMM_TX_FAIL;             // Communication result

	vel_limit = 1023;
	MODE = 1;
	delay_time = 10;
	dxl_id[0] = 10;
	dxl_id[1] = 11;

	error_thresh = 20;

	for (int i=0; i < NUM_OF_MOTORS; i++)
	{
	  goal_position[i] = 2048;
	  goal_velocity[i] = 0;
	}

	for (int i=0; i < NUM_OF_MOTORS; i++)
	{
	  dxl_goal_velocity[i][0] = -50;
	  dxl_goal_velocity[i][1] = 50;
	}
	dxl_error = 0;                          // Dynamixel error

	for (int i = 0; i<NUM_OF_MOTORS; i++)
	{
		dxl_present_position[NUM_OF_MOTORS] = {0};               // Present position
		dxl_present_velocity[NUM_OF_MOTORS] = {0};  
		dxl_present_current[NUM_OF_MOTORS] = {0};
		dxl_present_pwm[NUM_OF_MOTORS] = {0};
	}

	printf("Initialization Done!\n");

}

int CMotor::getch()
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


int CMotor::print_results(int dxl_comm_result, int dxl_error, int id, dynamixel::PacketHandler *packetHandler)
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

int CMotor::setup(int control_mode)
{
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
		printf("CMotor::dxl_id[i]: %d\n",CMotor::dxl_id[i]);
		printf("dxl_id[i]: %d\n",dxl_id[i]);
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, CMotor::dxl_id[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
		if (print_results(dxl_comm_result, dxl_error,  CMotor::dxl_id[i], packetHandler))
		{
			printf("Dynamixel[%d] torque disabled \n",CMotor::dxl_id[i]);
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

	for (int i = 0; i<NUM_OF_MOTORS; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], OPERATING_MODE, control_mode, &dxl_error);
		if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
		{
			printf("Dynamixel[%d] mode changed to %d \n",dxl_id[i], MODE);
		} 
	}
	if(control_mode == VELOCITY_MODE)
	{
		// set velocity limit 0~1023
		for (int i = 0; i<NUM_OF_MOTORS; i++)
		{
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], VELOCITY_LIMIT, vel_limit, &dxl_error);
			if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
			{
				printf("Dynamixel[%d] velocity limit changed to %d \n",dxl_id[i], vel_limit);
			} 
		}

	}
	else if(control_mode == POSITION_MODE)
	{
		// set position p gain
		for (int i = 0; i<NUM_OF_MOTORS; i++)
		{
			dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], POSITION_P_GAIN, 800, &dxl_error);
			if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
			{
				printf("Dynamixel[%d] position p gain changed \n",dxl_id[i]);
			} 
		}
	}
	else
	{
		printf("You CAN NOT use this control mode!\n");
		return -1;
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



  	return 1;
}

int CMotor::read()
{
	for (int i=0; i<NUM_OF_MOTORS; i++)
    {  
    	
		dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position[i], &dxl_error);
		dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity[i], &dxl_error);
		dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_PRESENT_PWM, (uint16_t*)&dxl_present_pwm[i], &dxl_error);
		dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_current[i], &dxl_error); 
		//printf("[%d]pos: %d\n", dxl_id[i], dxl_present_position[i]);
		//printf("[%d]vel: %d\n", dxl_id[i], dxl_present_velocity[i]);
    }
	return 0;
}

void CMotor::calc_error()
{
	read();
	for (int i=0; i<NUM_OF_MOTORS; i++)
    {  
    	error_position[i] = goal_position[i] - dxl_present_position[i];
    	error_velocity[i] = goal_velocity[i] - dxl_present_velocity[i];
		//printf("[%d]error_pos: %d\n", dxl_id[i], error_position[i]);
		//printf("[%d]error_vel: %d\n", dxl_id[i], error_velocity[i]);
    }
}

int CMotor::run(int* goal_vel, int* goal_pos)
{
	for (int i=0; i<NUM_OF_MOTORS; i++)
    {  
    	goal_position[i] = goal_pos[i];
    	goal_velocity[i] = goal_vel[i];
    	//printf("[%d]goal_pos: %d\n", dxl_id[i], goal_pos[i]);
		//printf("[%d]goal_vel: %d\n", dxl_id[i], goal_vel[i]);
    }
    printf("New Goal\n");
	calc_error();
	for (int i=0; i<NUM_OF_MOTORS; i++)
    {  
    	if (error_position[i] > 0)
    	{
    		goal_velocity[i] = goal_vel[i];
    	}
    	else
    	{
    		goal_velocity[i] = -goal_vel[i];
    	}
    }

    int done[NUM_OF_MOTORS] = {0};
    int cnt = 0;
    while(!done[0])
    {
    	cnt++;
	    for (int i=0; i<NUM_OF_MOTORS; i++)
	    { 
	

	    	calc_error();

	    	printf("abs(error_position[i]): %d\n",abs(error_position[i]));
	    	if(abs(error_position[i]) < error_thresh) 
	    	{

	    		done[i] = 1;
	    		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_VELOCITY, 0, &dxl_error);
	    		printf("Done[%d]\n",i);
	    	}
	    	else
	    	{

	    		done[i] = 0;
	    		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_VELOCITY, goal_velocity[i], &dxl_error);
	    	}

		}
		printf("[%d]done: %d, %d\n",cnt,done[0], done[1]);
		//printf("sum: %d\n",done[0]+done[1]);
		printf("\n\ngoal: %d, %d\n", goal_position[0], goal_position[1]);
		printf("curr: %d, %d\n", dxl_present_position[0], dxl_present_position[1]);
		printf("erro: %d, %d\n", error_position[0], error_position[1]);

		printf("done: %d, %d\n",done[0], done[1]);
	}

	for (int i=0; i<NUM_OF_MOTORS; i++)
    { 
		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_VELOCITY, 0, &dxl_error);
	}

	return 1;

}




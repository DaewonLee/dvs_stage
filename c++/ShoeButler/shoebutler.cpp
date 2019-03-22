#include <sched.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "dynamixel_sdk.h"                                  
#include "saictime.h"
#include "shm.h"
#include "traj.h"

#define STDIN_FILENO 					0
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132			// 4 Bytes
#define ADDR_PRO_PRESENT_VELOCITY       128			// 4 Bytes
#define ADDR_PRO_PRESENT_PWM    		124			// 2 Bytes
#define ADDR_PRO_PRESENT_CURRENT        126			// 2 Bytes

#define ADDR_PRO_Position_P_Gain		84 			// 2 Bytes

#define ADDR_PRO_BULK_READ				124
#define LEN_PRO_BULK_READ				12

#define LEN_PRO_GOAL_POSITION			4
#define LEN_PRO_PRESENT_POSITION		4
#define LEN_PRO_PRESENT_VELOCITY		4
#define LEN_PRO_PRESENT_PWM				2
#define LEN_PRO_PRESENT_CURRENT			2

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

#define NUM_OF_MOTORS					2
 
#define BAUDRATE                        115200
#define DEVICENAME                      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2GXCUN-if00-port0"      // Check 

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL_MOVING_STATUS_THRESHOLD     20                 // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define NUM_THREADS 					3

bool SYSTEM_RUN = true;
bool new_control = true;
int dxl_goal_position[NUM_OF_MOTORS] = {2048, 2048};         // Goal position
int dxl_current_position[NUM_OF_MOTORS];         // Goal position
pthread_mutex_t data_acq_mutex;
pthread_cond_t data_acq_cv;
pthread_mutex_t mutex;
CShm recv("/robot_input", 4096);
CShm send("/robot_output", 4096);




int ch;

int getch(void)
{
    struct termios oldattr, newattr;
    
    tcgetattr( STDIN_FILENO, &oldattr );

    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
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


/////////////////////////  THREADS  BIGIN   ////////////////////////////////////////

void *buffer(void *thread_id)
{
	CTime t_traj;
	DVS_trajectory traj;
	while(SYSTEM_RUN)
	{
		//dxl_goal_position[0] = 2000 + traj.sample(t_traj.get_time(), 1.) * 200;
		dxl_goal_position[0] = 2000 + traj.random_sample(t_traj.get_time()) * 200;
		dxl_goal_position[1] = 2000 + traj.random_sample(t_traj.get_time()) * 200;
		printf("dxl_goal_position[1]: %f\n",dxl_goal_position[1]);
		usleep(100000);
	}
	
    pthread_exit(NULL);
   
}



void *command_input(void *thread_id)
{
	unsigned char command_key;
	while(SYSTEM_RUN)
	{
		command_key = getch();
		if(command_key == 'q') SYSTEM_RUN = false;
		else if(command_key == 'a')
		{
			dxl_goal_position[0] -= 600;
		}
		else if(command_key == 'd')
		{
			dxl_goal_position[0] += 600;
		}
		else if(command_key == 'w')
		{
			dxl_goal_position[1] -= 600;
		}
		else if(command_key == 's')
		{
			dxl_goal_position[1] += 600;
		}


	}

    pthread_exit(NULL);
}



void *motor_control(void *thread_id)
{

  CTime time;
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupBulkWrite instance
  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);


	int index = 0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	int dxl_id[NUM_OF_MOTORS] = {5,6};

	bool dxl_addparam_result = false;               // addParam result
	bool dxl_getdata_result = false;                // GetParam result

	uint8_t dxl_error = 0;                          // Dynamixel error
	int32_t dxl_present_position[NUM_OF_MOTORS] = {0};               // Present position
	int32_t dxl_present_velocity[NUM_OF_MOTORS] = {0};  
	uint16_t dxl_present_current[NUM_OF_MOTORS] = {0};
	uint16_t dxl_present_pwm[NUM_OF_MOTORS] = {0};
	uint8_t dxl_present_bulk[12];
	uint8_t dxl_present_bulk2[12];

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

	for (int i = 0; i<NUM_OF_MOTORS; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
		{
			printf("Dynamixel[%d] has been successfully connected \n",dxl_id[i]);
		}	
		else
		{
			printf("Dynamixel[%d]: Failed to connect!! \n",dxl_id[i]);
		}	
	}

	//dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[0], ADDR_PRO_Position_P_Gain, 100, &dxl_error);
		


	for (int i = 0; i<NUM_OF_MOTORS; i++)
	{
		dxl_addparam_result = groupBulkRead.addParam(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed\n", dxl_id[i]);
			return 0;
		}
	}



	while(SYSTEM_RUN)
	{	

		dxl_comm_result = groupBulkRead.txRxPacket();
		for (int i=0; i<NUM_OF_MOTORS; i++)
		{
			dxl_getdata_result = groupBulkRead.isAvailable(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ);
		      if (dxl_getdata_result != true)
		      {
			fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", dxl_id[i]);
			return 0;
		      }
		}
		for (int i=0; i<NUM_OF_MOTORS; i++)
		{
			int res = groupBulkRead.getBulkData(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ, dxl_present_bulk);
	
			memcpy(&dxl_present_pwm[i], dxl_present_bulk,2);
			memcpy(&dxl_present_current[i], dxl_present_bulk+2,2);
			memcpy(&dxl_present_velocity[i], dxl_present_bulk+4,4);
			memcpy(&dxl_present_position[i], dxl_present_bulk+8,4);
			
			//memcpy(&dxl_goal_position[i], dxl_present_bulk2+8,4);
			

		}


		if(new_control == true)
		{
			for (int i = 0; i<NUM_OF_MOTORS; i++)
			{
				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_POSITION, dxl_goal_position[i], &dxl_error);
				if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler));
			}

		}
		groupBulkWrite.clearParam();
		dxl_comm_result = groupBulkRead.txRxPacket();

		for (int i=0; i<NUM_OF_MOTORS; i++)
		{
			dxl_getdata_result = groupBulkRead.isAvailable(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ);
			if (dxl_getdata_result != true)
			{
				fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", dxl_id[i]);
				return 0;
			}
		}
		int8_t temp[255];
		
		for (int i=0; i<NUM_OF_MOTORS; i++)
		{
			int res = groupBulkRead.getBulkData(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ, dxl_present_bulk);
			memcpy(&dxl_current_position[i], dxl_present_bulk+8,4);
			//memcpy(temp+i*12, dxl_present_bulk, 12);

		}

		printf("     pwm,    torque,   velocity,   position\n");
		printf("[0]: %d pwm,  %d,  %f deg/s,  %f deg\n",(signed short int)dxl_present_pwm[0], (signed short int)dxl_present_current[0], dxl_present_velocity[0]*0.229*6., (dxl_present_position[0]-2048)*0.088);
		printf("[1]: %d pwm,  %d,  %f deg/s,  %f deg\n",(signed short int)dxl_present_pwm[1], (signed short int)dxl_present_current[1], dxl_present_velocity[1]*0.229*6., (dxl_present_position[1]-2048)*0.088);

		//send.write8(temp,60);
		//usleep(10000);



													
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

}

/////////////////////////  THREADS  END   ////////////////////////////////////////

int main(int argc, char **argv)
{

	

	pthread_t threads[NUM_THREADS];
	pthread_attr_t attr;
	struct sched_param	param;

	int fifo_max_prio, fifo_min_prio;
	int numT = 0;

	pthread_mutex_init(&data_acq_mutex, NULL);


	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
	fifo_min_prio = sched_get_priority_min(SCHED_FIFO);

	param.sched_priority = fifo_max_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, motor_control, (void *) 0);

	// Medium priority for vicon
	param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, buffer, (void *) 1);

	// Lower priority for vicon
	param.sched_priority = fifo_min_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, command_input, (void *) 2);

	// Wait for all threads to complete
	for (int i = 0; i < numT; i++)
	{
		pthread_join(threads[i], NULL);
		printf("joining %d thread\n", i);
	}

	printf("close destroy pthread attr\n");
	pthread_attr_destroy(&attr);
	printf("close destroying mutex\n");
	pthread_mutex_destroy(&data_acq_mutex);
	return 0;


}

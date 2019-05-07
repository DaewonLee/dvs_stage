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
#include <assert.h>
//#include "cyusb.h"
#include <ctype.h>
#include <getopt.h>
#include <cmath>

#define STDIN_FILENO 					0
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132			// 4 Bytes
#define ADDR_PRO_PRESENT_VELOCITY       128			// 4 Bytes
#define ADDR_PRO_PRESENT_PWM    		124			// 2 Bytes
#define ADDR_PRO_PRESENT_CURRENT        126			// 2 Bytes

#define ADDR_PRO_GOAL_VELOCITY 			104			// 4 Bytes
#define ADDR_PRO_OPERATING_MODE			11			// 1 Byte
#define ADDR_PRO_VELOCITY_LIMIT         44			// 4 Bytes

#define VELOCITY_MODE	                1
#define POSITION_MODE	                3
#define EXTENDED_POSITION_MODE			4

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

#define NUM_OF_MOTORS					1
 
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT2KQC4K-if00-port0"      // Check 

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL_MOVING_STATUS_THRESHOLD     20                 // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define NUM_THREADS 					5

bool SYSTEM_RUN = true;
bool new_control = true;
int dxl_goal_position[NUM_OF_MOTORS] = {2048};         // Goal position
int dxl_current_position[NUM_OF_MOTORS];         // Goal position
pthread_mutex_t data_acq_mutex;
pthread_cond_t data_acq_cv;
pthread_mutex_t mutex;
CShm recv("/robot_input", 4096);
CShm send("/robot_output", 4096);

///////////

char sprintf_buffer[200000000];
int sprintf_buffer_loc = 0;
double time_current = 0.;

bool run_dvs = true;

//////////////////////////////DVS SRART////////////////////////////////


static const char * program_name;
static const char *const short_options = "?hdsl:";
static const struct option long_options[] = {
		{ "help",		0,		NULL,		'h'},
		{ "load",		1,		NULL,		'l',},
		{ "stream",		1,		NULL,		's',},
		{ "debug",		1,		NULL,		'd',},
		{ NULL,			0,		NULL,		 0}
};

static int next_option;

static void print_usage(FILE *stream, int exit_code)
{
	fprintf(stream, "Usage: %s options\n", program_name);
	fprintf(stream, 
		"  -h  --help        Display this usage information.\n"
		"  -l  --load        filename Load I2C script.\n"
		"  -s  --stream      Stream data.\n"
		"  -d  --debug       Print debug information.\n");

	exit(exit_code);
}

static FILE *fp = stdout;
static int timeout_provided;
static int timeout = 1000;
static int debug = false;


const int buflen = 1024;
const int I2C_VALUE_LEN = 2;
const int I2C_SLAVE_ADDR = 0x60;	//96;	// 0x60

const int	I2C_SLAVE_ADDR_DVSL = 0x20;
const int	I2C_SLAVE_ADDR_DVSR = 0x30;
const int	I2C_SLAVE_ADDR_D2FX = 0x40;
const int	I2C_SLAVE_ADDR_M2PR = 0x1A;
const int	I2C_SLAVE_ADDR_M2PL = 0x1C;
const int	I2C_VALUE_LEN_DVSL = 1;
const int	I2C_VALUE_LEN_DVSR = 1;
const int	I2C_VALUE_LEN_D2FX = 1;
const int	I2C_VALUE_LEN_M2PR = 2;
const int	I2C_VALUE_LEN_M2PL = 2;

struct Node {
	void *data;
	int len;
	struct Node* next;
};
struct Node* front = NULL;
struct Node* rear = NULL;
pthread_mutex_t queue_mutex;

CTime global_timer;

static void validate_inputs(void)
{
	if ( (timeout_provided) && (timeout < 0) ) {
		fprintf(stderr,"Must provide a positive value for timeout in seconds\n");
		print_usage(stdout, 1);
	}
}

int I2cSlaveAddr (int index) {
	switch(index){
		case 0 : return(I2C_SLAVE_ADDR_D2FX);
		case 1 : return(I2C_SLAVE_ADDR_DVSL);
		case 2 : return(I2C_SLAVE_ADDR_DVSR);
		case 3 : return(I2C_SLAVE_ADDR_M2PL);
		case 4 : return(I2C_SLAVE_ADDR_M2PR);
		default : return(I2C_SLAVE_ADDR_D2FX);
	}
}

int I2cValueLen (int slvAddr) {
	switch(slvAddr){
		case I2C_SLAVE_ADDR_D2FX : return(I2C_VALUE_LEN_D2FX);
		case I2C_SLAVE_ADDR_DVSL : return(I2C_VALUE_LEN_DVSL);
		case I2C_SLAVE_ADDR_DVSR : return(I2C_VALUE_LEN_DVSR);
		case I2C_SLAVE_ADDR_M2PL : return(I2C_VALUE_LEN_M2PL);
		case I2C_SLAVE_ADDR_M2PR : return(I2C_VALUE_LEN_M2PR);
		default : return(I2C_VALUE_LEN_D2FX);
	}
}

int readI2cReg(int slvAddr, int addr)
{

	return 1;
}

int writeI2cReg(int slvAddr, int addr, int val)
{

	return 0;
}

int htoi(char s[], int *i)
{
	int hexdigit;
	int n = 0;
	char c;

	while (true) {
		c = s[*i];

		if(c >='0' && c <='9')
			hexdigit = c - '0';
		else if(c >='a' && c <='f')
			hexdigit = c -'a' + 10;
		else if(c >='A' && c <='F')
			hexdigit = c -'A' + 10;
		else
			return n;

		n = 16 * n + hexdigit;

		*i = *i + 1;
	}
}

int parseString(char *s, int *slvAddr, int *adr, int *val) {
	int i = 0;
	int *dst;
	int mode = 10, ret;

	dst = slvAddr;
	while (true) {
		switch (tolower(s[i])) {
			case 0   : return mode;
			case '\n': return mode;
			case '/' : return mode;

			case ' ' : break;
			case '\t': break;
			case ':' : break;
			case '=' : break;

			case 'w':
				if (tolower(s[i+1]) == 'a' && tolower(s[i+2]) == 'i' && tolower(s[i+3]) == 't') {
					i = i + 3;
					mode = 20;
					dst = val;
					break;
				} else {
					return -1;
				}

			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case 'a':
			case 'b':
			case 'c':
			case 'd':
			case 'e':
			case 'f':
				*dst = htoi(s, &i);
				if (debug) printf("[I] Mode(%d): Value(%X) : (%X, %X, %X)\n", mode, *dst, *slvAddr, *adr, *val);
				switch (mode) {
					case 10 :
						mode = 11;
						dst = adr;
						break;
					case 11 :
						mode = 12;
						dst = val;
						break;
					case 12 :
						mode = 1;
						break;
					case 20 :
						mode = 2;
						break;
					default :
						break;
				}
				break;

			default: return -1;
		}
		i++;
	}
}

void loadScript(char *s)
{
	int slvAddr, adr, val;

	if (debug) printf("[I] loadScript(%s)\n", s);

	const int buflen = 1000;
	char buf[buflen];
	FILE * file;

	file = fopen(s , "r");
	if (!file) {
		printf("Error opening file %s\n", s);
		return;
	}

	while (fgets(buf, buflen, file)!=NULL) {

		//if (debug) printf("%s",buf);

		switch (parseString(buf, &slvAddr, &adr, &val)) {
			case 1:
				if (debug) printf("[I] I2C (%X,%X,%X)\n", slvAddr, adr, val);
				writeI2cReg(slvAddr, adr, val);
				break;
			case 2:
				if (debug) printf("[I] WAIT (%d)\n", val);
				//usleep(val*1000);
				break;
			default: continue;
		}
	}

	fclose(file);
}

int ahtoi(char s[])
{
	int hexdigit;
	int inhex = 1;
	int n = 0;
	int i = 0;

	if (s[i] == '0')
	{
		++i;
		if(s[i] == 'x' || s[i] == 'X') ++i;
		else return atoi(s);
	} else return atoi(s);

	return htoi(s, &i);
}

void Enqueue(void *pkt, int len) {
	pthread_mutex_lock(&queue_mutex);

	struct Node* temp = 
		(struct Node*)malloc(sizeof(struct Node));
	temp->data = pkt;
	temp->len = len;
	temp->next = NULL;

	if(front == NULL && rear == NULL){
		front = rear = temp;
	} else {
		rear->next = temp;
		rear = temp;
	}

	pthread_mutex_unlock(&queue_mutex);

	if (debug) printf("[I] Enqueued packet, len=%d\n", len);
}

void *Dequeue(int *len) {

	*len = 0;
	void *data = NULL;
	pthread_mutex_lock(&queue_mutex);
	int wha=0;


	struct Node* temp = front;
	if(front == NULL) {
		//printf("Queue is Empty\n");
		wha=1;
			//return NULL;
		
	} else {
		wha=0;
		if(front == rear) {
			data = front->data;
			*len = front->len;
			front = rear = NULL;
		} else {
			front = front->next;
			data = front->data;
			*len = front->len;
		}
		free(temp);
	}
	pthread_mutex_unlock(&queue_mutex);

	if (wha==1){
	return NULL;
	}
	return data;
}

void DecodePacket(unsigned char *pkt, int pktlen, FILE *fp) {
//	printf("he");
	//FILE *fifi = fopen("multi_T","wb");
	if (pktlen < 4) return;	// Unexpected invalid packet
	if (pktlen % 4) pktlen = (pktlen / 4) * 4;

	for (int i = 0; i < pktlen; i += 4) {
        	
		for (int j=0; j < 4; j+=1){
			//printf("%p",pkt+(i+j));
            		fwrite((pkt+(i+j)),sizeof(unsigned char),1,fp);
            	}
		
		switch (pkt[i]) {

			case (0x66) :
				printf ("T");
				break;
			case (0x99) :
				printf ("G");
				break;
			case (0xcc) :
				printf ("E");
				break;
			default :
				break;

		}

	}
	//printf (" %d\n", pktlen/4);

}


////////////////////////////////DVS END/////////////////////////////



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

void SatVal(int upper, int lower, int* val){

	if (*val > upper)
	    *val = upper;
	else if (*val < lower)
	    *val = lower;

} 


/////////////////////////  THREADS  BIGIN   ////////////////////////////////////////

void *trajectory(void *thread_id)
{
	CTime t_traj;
	DVS_trajectory traj;
	float pan_limit_upper = 21; //degree
	float tilt_limit_upper =4;
	float pan_limit_lower = -16; //degree
	float tilt_limit_lower =-7;  

	float pan_Hz = 0.3; //Hz
	float tilt_Hz = 0.2;  

	float pan_degree = 25.; //degree
	float tilt_degree = 5.;

	// pan limit: -16, 21 deg
	// tile limit: -7, 4 deg

	while(SYSTEM_RUN)
	{
	
		dxl_goal_position[0] = 2048 + traj.sample(t_traj.get_time(),pan_Hz) * pan_degree/0.088;
		dxl_goal_position[1] = 2048 + traj.sample(t_traj.get_time(),tilt_Hz) * tilt_degree/0.088;

		//printf("dxl_goal_position[0]: %d\n", dxl_goal_position[0]);

		

		//dxl_goal_position[0] = 2000 + traj.random_sample(t_traj.get_time()) * 200;
		//dxl_goal_position[1] = 2000 + traj.random_sample(t_traj.get_time()) * 200;

		SatVal(2048+(int)pan_limit_upper/0.088, 2048+(int)pan_limit_lower/0.088, &dxl_goal_position[0]);
		SatVal(2048+(int)tilt_limit_upper/0.088, 2048+(int)tilt_limit_lower/0.088, &dxl_goal_position[1]);
		//printf("dxl_goal_position[1]: %f\n",dxl_goal_position[1]);
		
		usleep((int)(abs((1000*traj.randn(0.,5.))+200000)));
		//usleep(-1);
		if(t_traj.get_time()>45) SYSTEM_RUN = false;
	}
	
    pthread_exit(NULL);
   
}



void *command_input(void *thread_id)
{
	unsigned char command_key;
	//while(SYSTEM_RUN)
	while(0)
	{
		command_key = getch();  
		if(command_key == 'q') SYSTEM_RUN = false;
		else if(command_key == 'a')
		{
			dxl_goal_position[0] -= 10;
		}
		else if(command_key == 'd')
		{
			dxl_goal_position[0] += 10;
		}
		else if(command_key == 'w')
		{
			dxl_goal_position[1] -= 10;
		}
		else if(command_key == 's')
		{
			dxl_goal_position[1] += 10;
		}


	}

    pthread_exit(NULL);
}



void *motor_control(void *thread_id)
{

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  FILE *file;
  file = fopen("motor_data.txt","w");

  // Initialize GroupBulkWrite instance
  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);


	int index = 0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	int dxl_id[NUM_OF_MOTORS] = {10};

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
		printf("Failed to open the port!@@@@\n");
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

	// Disable Dynamixel Torque

	for (int i = 0; i<NUM_OF_MOTORS; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
		if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
		{
			printf("Dynamixel[%d] has been successfully disables \n",dxl_id[i]);
		}	
		else
		{
			printf("Dynamixel[%d]: Failed to disable!! \n",dxl_id[i]);
		}	
	}

	

	for (int i = 0; i<NUM_OF_MOTORS; i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_OPERATING_MODE , POSITION_MODE, &dxl_error);
		if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
		{
			printf("Dynamixel[%d] is velocity control mode! \n",dxl_id[i]);
		}	
		else
		{
			printf("Dynamixel[%d]: Failed to enter velocity control mode \n",dxl_id[i]);
		}	
		//usleep(10000);
		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_VELOCITY_LIMIT , 600 , &dxl_error);
		if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler))
		{
			printf("Dynamixel[%d] velocity limit set ! \n",dxl_id[i]);
		}	
		else
		{
			printf("Dynamixel[%d]: Failed to set velocity limit \n",dxl_id[i]);
		}
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

	for (int i = 0; i<NUM_OF_MOTORS; i++)
	{
		dxl_addparam_result = groupBulkRead.addParam(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed\n", dxl_id[i]);
			return 0;
		}
	}
//usleep(2000000);

	while(SYSTEM_RUN)
	{	
		printf("system rate: %f (Hz)\n",1./global_timer.get_delta_t());
		dxl_comm_result = groupBulkRead.txRxPacket();
		//for (int i=0; i<NUM_OF_MOTORS; i++)
		//{
		//	dxl_getdata_result = groupBulkRead.isAvailable(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ);
		//     if (dxl_getdata_result != true)
		//      {
		//	fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", dxl_id[i]);
		//	return 0;
		//      }
		//}
		for (int i=0; i<NUM_OF_MOTORS; i++)
		{
			int res = groupBulkRead.getBulkData(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ, dxl_present_bulk);
	
			memcpy(&dxl_present_pwm[i], dxl_present_bulk,2);
			memcpy(&dxl_present_current[i], dxl_present_bulk+2,2);
			memcpy(&dxl_present_velocity[i], dxl_present_bulk+4,4);
			memcpy(&dxl_present_position[i], dxl_present_bulk+8,4);


		}


		if(new_control == true)
		{
			for (int i = 0; i<NUM_OF_MOTORS; i++)
			{
				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_POSITION, dxl_goal_position[i], &dxl_error);
				if (print_results(dxl_comm_result, dxl_error,  dxl_id[i], packetHandler));
			}

		}

		if(0)	
		{
			for (int i = 0; i<NUM_OF_MOTORS; i++)
			{
				dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PRO_GOAL_VELOCITY, 200, &dxl_error);
				//dxl_comm_result =packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_VELOCITY, DXL_GOAL_VELOCITY, &dxl_error);
				printf("here\n");
			}

		}

		
		//groupBulkWrite.clearParam();
		//dxl_comm_result = groupBulkRead.txRxPacket();

		//for (int i=0; i<NUM_OF_MOTORS; i++)
		//{
		//	dxl_getdata_result = groupBulkRead.isAvailable(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ);
		//	if (dxl_getdata_result != true)
		//	{
		//		fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", dxl_id[i]);
		//		return 0;
		//	}
		//}
		time_current = global_timer.get_time();
		//int8_t temp[255];
		
		//for (int i=0; i<NUM_OF_MOTORS; i++)
		//{
		//	int res = groupBulkRead.getBulkData(dxl_id[i], ADDR_PRO_BULK_READ, LEN_PRO_BULK_READ, dxl_present_bulk);
		//	memcpy(&dxl_current_position[i], dxl_present_bulk+8,4);
			//memcpy(temp+i*12, dxl_present_bulk, 12);

		//}

		//printf("%fs     pwm,    torque,   velocity,   position\n",global_timer.get_time());
		//printf("[0]: %d pwm,  %d,  %f deg/s,  %f deg\n",(signed short int)dxl_present_pwm[0], (signed short int)dxl_present_current[0], dxl_present_velocity[0]*0.229*6., (dxl_present_position[0]-2048)*0.088);
		//printf("[1]: %d pwm,  %d,  %f deg/s,  %f deg\n",(signed short int)dxl_present_pwm[1], (signed short int)dxl_present_current[1], dxl_present_velocity[1]*0.229*6., (dxl_present_position[1]-2048)*0.088);
		//printf("time: %f\n",global_timer.get_time());
		if(sprintf_buffer_loc < sizeof(sprintf_buffer))
        {
            int sprintf_size = sprintf(sprintf_buffer+sprintf_buffer_loc,"\
            %e %e %e %e %e \n",
            
            time_current, dxl_present_velocity[0]*0.229*6., dxl_present_velocity[1]*0.229*6., (dxl_present_position[0]-2048)*0.088, (dxl_present_position[1]-2048)*0.088 );
            sprintf_buffer_loc+=sprintf_size;
        }
        else if(sprintf_buffer_loc >= sizeof(sprintf_buffer))
        {
            printf("Warning: sprintf_buffer is full! \n");
		}


	}
	printf("======================================================================================= I'M DYING HERE\n\n\n\n\n\n");
    printf("Opening text file.....  \n");
    printf("Saving buffer to a file.....  \n");
    fwrite(sprintf_buffer, 1, sprintf_buffer_loc,file);
    printf("free buffer file.....  \n");
	fclose(file);

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


// dvs thread 

static void *reader(void *thread_id)
{

    pthread_exit(NULL);
}

// dvs thread

static void *processor(void *thread_id)
{
	

    pthread_exit(NULL);
}


/////////////////////////  THREADS  END   ////////////////////////////////////////

int main(int argc, char **argv)
{

	int r;
	char user_input = 'n';
	pthread_t tidStream, tidProcess;
	int slvAddr = 0, addr = 0;
	int val = 0;
	int cmd = 0;
	char *filename;
	//FILE *fifi = fopen("dvsfastdump.txt","w");
	program_name = argv[0];

	if (argc < 2) {
		printf("here");
		print_usage (stdout, 1);
	}




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

	param.sched_priority = fifo_min_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, motor_control, (void *) 0);

	// Medium priority for vicon
	param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, trajectory, (void *) 1);

	// Lower priority for vicon
	param.sched_priority = fifo_min_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, command_input, (void *) 2);

	// dvs thread reader
	param.sched_priority = fifo_max_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, reader, (void *) 3);

	// dvs thread process
	param.sched_priority = fifo_max_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[numT++], &attr, processor, (void *) 4);

	// Wait for all threads to complete
	for (int i = 0; i < numT; i++)
	{
		pthread_join(threads[i], NULL);
		printf("joining %d thread\n", i);
	}
	printf("bye\n");
	//cyusb_close();

	printf("close destroy pthread attr\n");
	pthread_attr_destroy(&attr);
	printf("close destroying mutex\n");
	pthread_mutex_destroy(&data_acq_mutex);
	printf("you made it");
	return 0;


}

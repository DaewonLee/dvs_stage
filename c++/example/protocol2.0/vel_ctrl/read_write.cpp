// echo 1 | sudo tee -a /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "saictime.h"
#include "motor.h"

int main()
{
  CMotor motor;
  motor.initialize();
  motor.setup(VELOCITY_MODE);
  motor.read();
  motor.calc_error();
 
  //motor.goal_velocity[0]=150;
  int goal_pos[2] = {3000, 2000};
  int goal_vel[2] = {150, 100};
  int goal_pos1_[5] = {2300, 2500, 2300, 2500,2300};
  int goal_vel1_[5] = {50, 50, 50, 50,50};

  int goal_pos2_[5] = {2300, 2500, 2300, 2500,2300};
  int goal_vel2_[5] = {50, 50, 50, 50,50};


  for (int i =0; i<5; i++)
  {
    goal_vel[0] = goal_vel1_[i];
    goal_pos[0] = goal_pos1_[i];
    goal_vel[1] = goal_vel2_[i];
    goal_pos[1] = goal_pos2_[i];
    motor.run(goal_vel, goal_pos);
  }
  

  printf("hello\n");



  return 0;
}

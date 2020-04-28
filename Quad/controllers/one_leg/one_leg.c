#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/keyboard.h>

#include "webotsInterface.h"
#include "controller.h"
#include <math.h>

extern robotTypeDef robot;


int main(int argc, char **argv) {
  
  wb_robot_init();
  webots_device_init();                          
  robot_init();                                  
  while (wb_robot_step(TIME_STEP) != -1)
 {
    updateRobotState();                             
    robot_control();                              
  }
  wb_robot_cleanup();
  return 0;
}





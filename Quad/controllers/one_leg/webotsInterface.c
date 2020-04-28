#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>
#include <webots/accelerometer.h>
#include <webots/supervisor.h>
  
#include "webotsInterface.h"

WbDeviceTag FL0_motor;
WbDeviceTag FL1_motor;
WbDeviceTag FL2_motor;
WbDeviceTag FR0_motor;
WbDeviceTag FR1_motor;
WbDeviceTag FR2_motor;

WbDeviceTag BL0_motor;
WbDeviceTag BL1_motor;
WbDeviceTag BL2_motor;
WbDeviceTag BR0_motor;
WbDeviceTag BR1_motor;
WbDeviceTag BR2_motor;

WbDeviceTag FL0_pos_sensor;
WbDeviceTag FL1_pos_sensor;
WbDeviceTag FL2_pos_sensor;
WbDeviceTag FR0_pos_sensor;
WbDeviceTag FR1_pos_sensor;
WbDeviceTag FR2_pos_sensor;

WbDeviceTag BL0_pos_sensor;
WbDeviceTag BL1_pos_sensor;
WbDeviceTag BL2_pos_sensor;
WbDeviceTag BR0_pos_sensor;
WbDeviceTag BR1_pos_sensor;
WbDeviceTag BR2_pos_sensor;

/*足底触碰开关*/
WbDeviceTag FL_touch_sensor;
WbDeviceTag FR_touch_sensor;
WbDeviceTag BL_touch_sensor;
WbDeviceTag BR_touch_sensor;

WbDeviceTag IMU;
WbDeviceTag Accelerometer;
WbNodeRef   Robot_node     ;
WbFieldRef  trans_field    ;           
void webots_device_init()
{
  FL0_motor          = wb_robot_get_device("FL0 rotational motor");
  FL1_motor          = wb_robot_get_device("FL1 rotational motor");
  FL2_motor          = wb_robot_get_device("FL2 rotational motor");
  FR0_motor          = wb_robot_get_device("FR0 rotational motor");
  FR1_motor          = wb_robot_get_device("FR1 rotational motor");
  FR2_motor          = wb_robot_get_device("FR2 rotational motor");
  
  BL0_motor          = wb_robot_get_device("BL0 rotational motor");
  BL1_motor          = wb_robot_get_device("BL1 rotational motor");
  BL2_motor          = wb_robot_get_device("BL2 rotational motor");
  BR0_motor          = wb_robot_get_device("BR0 rotational motor");
  BR1_motor          = wb_robot_get_device("BR1 rotational motor");
  BR2_motor          = wb_robot_get_device("BR2 rotational motor");
  
  FL0_pos_sensor     = wb_robot_get_device("FL0 position sensor");
  FL1_pos_sensor     = wb_robot_get_device("FL1 position sensor");
  FL2_pos_sensor     = wb_robot_get_device("FL2 position sensor");  
  FR0_pos_sensor     = wb_robot_get_device("FR0 position sensor");
  FR1_pos_sensor     = wb_robot_get_device("FR1 position sensor");
  FR2_pos_sensor     = wb_robot_get_device("FR2 position sensor");  
  
  BL0_pos_sensor     = wb_robot_get_device("BL0 position sensor");
  BL1_pos_sensor     = wb_robot_get_device("BL1 position sensor");
  BL2_pos_sensor     = wb_robot_get_device("BL2 position sensor");  
  BR0_pos_sensor     = wb_robot_get_device("BR0 position sensor");
  BR1_pos_sensor     = wb_robot_get_device("BR1 position sensor");
  BR2_pos_sensor     = wb_robot_get_device("BR2 position sensor");  
  
  IMU                = wb_robot_get_device("inertial unit");
  Accelerometer      = wb_robot_get_device("ACCELEROMETER");
 
  
  wb_position_sensor_enable(FL0_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(FL1_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(FL2_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(FR0_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(FR1_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(FR2_pos_sensor, TIME_STEP);
  
  wb_position_sensor_enable(BL0_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(BL1_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(BL2_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(BR0_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(BR1_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(BR2_pos_sensor, TIME_STEP);
  
  
   //get device  FB T sensor
  FL_touch_sensor    = wb_robot_get_device("FL touch sensor");
  FR_touch_sensor    = wb_robot_get_device("FR touch sensor");
  BL_touch_sensor    = wb_robot_get_device("BL touch sensor");
  BR_touch_sensor    = wb_robot_get_device("BR touch sensor");
  wb_touch_sensor_enable(FL_touch_sensor, TIME_STEP);
  wb_touch_sensor_enable(FR_touch_sensor, TIME_STEP);
  wb_touch_sensor_enable(BL_touch_sensor, TIME_STEP);
  wb_touch_sensor_enable(BR_touch_sensor, TIME_STEP);
  
  wb_inertial_unit_enable(IMU, TIME_STEP);
  
  wb_accelerometer_enable(Accelerometer,TIME_STEP);
   
  wb_keyboard_enable(TIME_STEP);
  
  Robot_node   = wb_supervisor_node_get_from_def("Robot"); 
  trans_field  = wb_supervisor_node_get_field(Robot_node,"translation");
}

void set_motor_torque(motorNameTypeDef motorName, float torque)
{
  if(torque >  1800)torque =  1800;
  if(torque < -1800)torque = -1800;
  
  switch (motorName){
  case FL_M0:  {  wb_motor_set_torque(FL0_motor, torque);break; }
  case FL_M1:  {  wb_motor_set_torque(FL1_motor, torque);break; }
  case FL_M2:  {  wb_motor_set_torque(FL2_motor, torque);break; }
  case FR_M0:  {  wb_motor_set_torque(FR0_motor, torque);break; }
  case FR_M1:  {  wb_motor_set_torque(FR1_motor, torque);break; }
  case FR_M2:  {  wb_motor_set_torque(FR2_motor, torque);break; }
  
  case BL_M0:  {  wb_motor_set_torque(BL0_motor, torque);break; }
  case BL_M1:  {  wb_motor_set_torque(BL1_motor, torque);break; }
  case BL_M2:  {  wb_motor_set_torque(BL2_motor, torque);break; }
  case BR_M0:  {  wb_motor_set_torque(BR0_motor, torque);break; }
  case BR_M1:  {  wb_motor_set_torque(BR1_motor, torque);break; }
  case BR_M2:  {  wb_motor_set_torque(BR2_motor, torque);break; }
  default:break;
  }
}

void set_motor_position(motorNameTypeDef motorName, float pos)
{
 if(pos >  2*PI/3)pos = 2*PI/3;
 if(pos < -2*PI/3)pos =-2*PI/3;
  int sp = 45;
  switch (motorName){
  case FL_M0:  {  wb_motor_set_control_pid(FL0_motor, sp, 1, 0);wb_motor_set_position(FL0_motor, pos);break; }
  case FL_M1:  {  wb_motor_set_control_pid(FL1_motor, 370,1, 0);wb_motor_set_position(FL1_motor, pos);break; }
  case FL_M2:  {  wb_motor_set_control_pid(FL2_motor, 370,1, 0);wb_motor_set_position(FL2_motor, pos);break; }
  case FR_M0:  {  wb_motor_set_control_pid(FR0_motor, sp, 1, 0);wb_motor_set_position(FR0_motor, pos);break; }
  case FR_M1:  {  wb_motor_set_control_pid(FR1_motor, 370,1, 0);wb_motor_set_position(FR1_motor, pos);break; }
  case FR_M2:  {  wb_motor_set_control_pid(FR2_motor, 370,1, 0);wb_motor_set_position(FR2_motor, pos);break; }
  
  case BL_M0:  {  wb_motor_set_control_pid(BL0_motor, sp, 1, 0);wb_motor_set_position(BL0_motor, pos);break; }
  case BL_M1:  {  wb_motor_set_control_pid(BL1_motor, 370,1, 0);wb_motor_set_position(BL1_motor, pos);break; }
  case BL_M2:  {  wb_motor_set_control_pid(BL2_motor, 370,1, 0);wb_motor_set_position(BL2_motor, pos);break; }
  case BR_M0:  {  wb_motor_set_control_pid(BR0_motor, sp, 1, 0);wb_motor_set_position(BR0_motor, pos);break; }
  case BR_M1:  {  wb_motor_set_control_pid(BR1_motor, 370,1, 0);wb_motor_set_position(BR1_motor, pos);break; }
  case BR_M2:  {  wb_motor_set_control_pid(BR2_motor, 370,1, 0);wb_motor_set_position(BR2_motor, pos);break; }
  default:break;
  }
}

      
float get_motor_angle(motorNameTypeDef motorName)
{
  float angle = 0;
  switch (motorName){
  case FL_M0:  { angle = wb_position_sensor_get_value(FL0_pos_sensor);break; }
  case FL_M1:  { angle = wb_position_sensor_get_value(FL1_pos_sensor);break; }
  case FL_M2:  { angle = wb_position_sensor_get_value(FL2_pos_sensor);break; }
  case FR_M0:  { angle = wb_position_sensor_get_value(FR0_pos_sensor);break; }
  case FR_M1:  { angle = wb_position_sensor_get_value(FR1_pos_sensor);break; }
  case FR_M2:  { angle = wb_position_sensor_get_value(FR2_pos_sensor);break; }
  
  case BL_M0:  { angle = wb_position_sensor_get_value(BL0_pos_sensor);break; }
  case BL_M1:  { angle = wb_position_sensor_get_value(BL1_pos_sensor);break; }
  case BL_M2:  { angle = wb_position_sensor_get_value(BL2_pos_sensor);break; }
  case BR_M0:  { angle = wb_position_sensor_get_value(BR0_pos_sensor);break; }
  case BR_M1:  { angle = wb_position_sensor_get_value(BR1_pos_sensor);break; }
  case BR_M2:  { angle = wb_position_sensor_get_value(BR2_pos_sensor);break; }
  }
  return angle*180.0f/PI;
}

bool is_foot_touching(legNameTypeDef legName)
{
  if(legName == FL)
     return wb_touch_sensor_get_value(FL_touch_sensor);
  if(legName == FR)
     return wb_touch_sensor_get_value(FR_touch_sensor); 
  if(legName == BL)
     return wb_touch_sensor_get_value(BL_touch_sensor);
  if(legName == BR)
     return wb_touch_sensor_get_value(BR_touch_sensor); 
  return true;
}

double Force_touching(legNameTypeDef legName)
{
  double Force;
  if(legName == FL)
     return Force = wb_touch_sensor_get_value(FL_touch_sensor);
  if(legName == FR)
     return Force = wb_touch_sensor_get_value(FR_touch_sensor); 
  if(legName == BL)
     return Force = wb_touch_sensor_get_value(BL_touch_sensor);
  if(legName == BR)
     return Force = wb_touch_sensor_get_value(BR_touch_sensor); 
  return 1;
}

eulerAngleTypeDef get_IMU_Angle()
{
  const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  eulerAngleTypeDef eulerAngle;
  eulerAngle.roll  = data[1];
  eulerAngle.pitch = data[0];
  eulerAngle.yaw   = data[2];
  return eulerAngle;
}

acceleroTypeDef get_ACCs()
{
  const double* data = wb_accelerometer_get_values(Accelerometer);
  acceleroTypeDef accAxis;
  accAxis.x  = data[2];
  accAxis.y  = data[0];
  accAxis.z  = data[1];
  return accAxis;
}

v3TypeDef get_sf_vec3f()
{
  const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
  v3TypeDef p;
  p.x = trans[2];
  p.y = trans[0];
  p.z = trans[1];
  return p;
}

int get_keyboard()
{
  return wb_keyboard_get_key();
}

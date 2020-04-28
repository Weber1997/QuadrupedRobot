#ifndef _WEBOTSINTERFACE_H_
#define _WEBOTSINTERFACE_H_

#define PI           (3.141592654f)
#define TIME_STEP    (3)

typedef enum
{
  FL_M0   = 0x00,
  FL_M1   = 0x01,
  FL_M2   = 0x02,
  FR_M0   = 0x03,
  FR_M1   = 0x04,
  FR_M2   = 0x05,
  BL_M0   = 0x06,
  BL_M1   = 0x07,
  BL_M2   = 0x08,
  BR_M0   = 0x09,
  BR_M1   = 0x10,
  BR_M2   = 0x11,
  
}motorNameTypeDef;
   
typedef enum
{
  FL = 0x00,   
  FR = 0x01,   
  BR = 0x02,   
  BL = 0x03,  
}legNameTypeDef;

typedef struct
{
  double roll;       //横滚，x轴
  double pitch;      //俯仰，z轴
  double yaw;        //偏航，y轴
}eulerAngleTypeDef;

typedef struct
{
  double x;
  double y;
  double z;
}acceleroTypeDef;
 
typedef struct
{
  double x;
  double y;
  double z;
}v3TypeDef;


extern void              webots_device_init                                          ();
extern void              set_motor_torque    (motorNameTypeDef motorName, float torque);
extern void              set_motor_position     (motorNameTypeDef motorName, float pos);
extern float             get_motor_angle                   (motorNameTypeDef motorName);
extern bool              is_foot_touching                      (legNameTypeDef legName);
extern double            Force_touching                        (legNameTypeDef legName);
extern eulerAngleTypeDef get_IMU_Angle                                               ();
extern acceleroTypeDef   get_ACCs                                                    ();
extern int               get_keyboard                                                ();
extern v3TypeDef         get_sf_vec3f                                                ();
#endif
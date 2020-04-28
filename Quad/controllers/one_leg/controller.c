#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <conio.h>
#include <webots/keyboard.h>

#include "easyMat.h"
#include "controller.h"
#include "webotsInterface.h"

robotTypeDef robot;
//----------------------------------------------------------declaration
void update_vxyd();
void update_IMU();
void update_ACC();
void update_foot_touch_sensor();
void phase_swap();
void update_theta();
void forwardKinematics();
void update_toe0_dtoe0();
void estimate_vxy();
void create_T_transJ(matTypeDef* transJ, legNameTypeDef leg);
void Raibert_Planning(vect3TypeDef* Exp, vect3TypeDef* toe_0);
void invertKinematics(matTypeDef* Theta, vect3TypeDef* Exp, legNameTypeDef leg);
void RunCurve(vect3TypeDef toe,legNameTypeDef leg);
void F_Create(matTypeDef* F);
void J_6_create(matTypeDef* J_Mat, matTypeDef* jf, matTypeDef* jb, legNameTypeDef f_leg, legNameTypeDef b_leg);


void robot_init()
{

    robot.zh = 0.04f;   //摆线最高点;
    robot.Tf = 0.25;
    robot.Ts = 0.5;

    //支撑端 F T 参数
    robot.Kvx  = 200 ;   
    robot.Kh   = 8000;   
    robot.Kdh  = 800 ;   
    robot.Krx  = -600;   
    robot.Kdrx = -50 ;    
    robot.Kry  = -500;   
    robot.Kdry = -50 ;     
    robot.Kwz  = -500;   
    robot.wzd  = 0;
    robot.vxd  = 0.02; 
    
    robot.xT     = 0.00    ;
    robot.yT     = 0       ;
    robot.hd     = 0.346   ;  
//------------------------------------------------------------------------模型参数
    robot.a1   = 0.2        ;
    robot.a2   = 0.2        ;  
    robot.M    = 50        ;     //总质量
    robot.L    = 0.25       ;
    robot.w    = 0.15       ;
//------------------------------------------------------------------------状态参数    
   robot.t = 0;       
   robot.is_touching[FL]  = false;  //足底传感器接触标志量
   robot.is_touching[FR]  = false;  //足底传感器接触标志量
   robot.is_touching[BL]  = false;  //足底传感器接触标志量
   robot.is_touching[BR]  = false;  //足底传感器接触标志量
   robot.F_sw = FL;       
   robot.F_st = FR;      
   robot.B_sw = BR;      
   robot.B_st = BL;      
   robot.F_sw_toe0.x         = 0;
   robot.F_sw_toe0.y         = 0;
   robot.F_sw_toe0.z         = 0;
   robot.F_st_toe0.x         = 0;
   robot.F_st_toe0.y         = 0;
   robot.F_st_toe0.z         = 0;
   
   robot.B_sw_toe0.x         = 0;
   robot.B_sw_toe0.y         = 0;
   robot.B_sw_toe0.z         = 0;
   robot.B_st_toe0.x         = 0;
   robot.B_st_toe0.y         = 0;
   robot.B_st_toe0.z         = 0;
   
   robot.F_dtoe0.x        = 0;
   robot.F_dtoe0.y        = 0;
   robot.F_dtoe0.z        = 0;
   robot.B_dtoe0.x        = 0;
   robot.B_dtoe0.y        = 0;
   robot.B_dtoe0.z        = 0;

}

void updateRobotState()
{
    update_vxyd();
    update_IMU();
    update_ACC();
    update_foot_touch_sensor();
    phase_swap();
    update_theta();
    forwardKinematics();
    update_toe0_dtoe0();
    estimate_vxy();
    
    robot.t += 0.001 * TIME_STEP;
}
//---------------------------------------------------- updateRobotState()

void update_vxyd()
{
  switch(get_keyboard())
  {
    case WB_KEYBOARD_UP:
    {
      break;
    }
     case WB_KEYBOARD_DOWN:
    {
      break;
    }
    default:
    {
      break;
    }
  }
}

 void update_IMU()
 {
    static eulerAngleTypeDef pre_eulerAngle = {0,0,0};
    eulerAngleTypeDef eulerAngle = get_IMU_Angle();
    robot.pitch = eulerAngle.pitch;
    robot.roll  = eulerAngle.roll;
    robot.yaw   = eulerAngle.yaw;
    
    static float pre_dpitch = 0;
    static float pre_droll  = 0;
    static float pre_dyaw   = 0;
    robot.dpitch = (eulerAngle.pitch - pre_eulerAngle.pitch)/((float)TIME_STEP/1000.0);
    robot.droll  = (eulerAngle.roll - pre_eulerAngle.roll)  /((float)TIME_STEP/1000.0);
    robot.dyaw   = (eulerAngle.yaw - pre_eulerAngle.yaw)    /((float)TIME_STEP/1000.0);
    
    robot.dpitch = robot.dpitch*0.1 + pre_dpitch*0.9;
    robot.droll  = robot.droll*0.1  + pre_droll*0.9;
    robot.dyaw   = robot.dyaw*0.1   + pre_dyaw*0.9;
    
    pre_dpitch = robot.dpitch;
    pre_droll  = robot.droll;
    pre_dyaw   = robot.dyaw;
    
    pre_eulerAngle = eulerAngle;
 }

  void update_ACC()
  {
    acceleroTypeDef acc = get_ACCs();
    robot.ax  = acc.x;
    robot.ay  = acc.y;
    robot.az  = acc.z;
  }
  
   void update_foot_touch_sensor()
  {
    robot.is_touching[FL] = is_foot_touching(FL);
    robot.is_touching[FR] = is_foot_touching(FR);
    robot.is_touching[BL] = is_foot_touching(BL);
    robot.is_touching[BR] = is_foot_touching(BR);
  }
  
  
  void phase_swap()
{
  if(robot.t > 0.75*robot.Tf)  
  {
   if(robot.F_st == FR && robot.B_st == BL)  
    {
      if(robot.is_touching[BR] || robot.is_touching[FL] )
      {
        robot.F_st = FL;
        robot.F_sw = FR;
        robot.B_st = BR;
        robot.B_sw = BL;
        robot.t = 0;      
      }
    }
    else if(robot.F_st == FL && robot.B_st == BR) 
    {
      if(robot.is_touching[BL] || robot.is_touching[FR])
      {
        robot.F_st = FR;
        robot.F_sw = FL;
        robot.B_st = BL;
        robot.B_sw = BR;
        robot.t = 0;     
      }
    }
  
  }
}
   void update_theta()
   {
    robot.theta[FL][0] = get_motor_angle(FL_M0);
    robot.theta[FL][1] = get_motor_angle(FL_M1);
    robot.theta[FL][2] = get_motor_angle(FL_M2);
    robot.theta[FR][0] = get_motor_angle(FR_M0);
    robot.theta[FR][1] = get_motor_angle(FR_M1);
    robot.theta[FR][2] = get_motor_angle(FR_M2);
    
    robot.theta[BL][0] = get_motor_angle(BL_M0);
    robot.theta[BL][1] = get_motor_angle(BL_M1);
    robot.theta[BL][2] = get_motor_angle(BL_M2);
    robot.theta[BR][0] = get_motor_angle(BR_M0);
    robot.theta[BR][1] = get_motor_angle(BR_M1);
    robot.theta[BR][2] = get_motor_angle(BR_M2);
   }
   
   void forwardKinematics()
   {
    float a1 = robot.a1;
    float a2 = robot.a2;
    float L  = robot.L;
    float w  = robot.w;
    int   delta     = 1;
    int   u         = 1;
    float plx,ply,plz;
     
    //足底位置
    for (legNameTypeDef legIdx = FL; legIdx <= BL; legIdx++)
    {
        float s0 =  sin(robot.theta[legIdx][0] / 180.0 * PI);
        float c0 =  cos(robot.theta[legIdx][0] / 180.0 * PI);
        float s1 =  sin(robot.theta[legIdx][1] / 180.0 * PI);
        float c1 =  cos(robot.theta[legIdx][1] / 180.0 * PI);
        float s12 = sin(robot.theta[legIdx][1] / 180.0 * PI + robot.theta[legIdx][2] / 180.0 * PI);
        float c12 = cos(robot.theta[legIdx][1] / 180.0 * PI + robot.theta[legIdx][2] / 180.0 * PI);
        
          switch (legIdx)
           {
             case FL:  {  delta = 1; u = 1   ;break;  }
             case FR:  {  delta = 1; u = -1  ;break;  } 
             case BR:  {  delta = -1; u = -1 ;break;  }
             case BL:  {  delta = -1; u = 1  ;break;  }
              default:break;
            }
                plx = c0*(a1*c1+a2*c12);
                ply = s0*(a1*c1+a2*c12);
                plz = -a1*s1-a2*s12;
            
              robot.toe[legIdx].x = plz  + delta*L ; 
              robot.toe[legIdx].y = ply  + u*w;      
              robot.toe[legIdx].z =  -plx   ;       
           }
   }
   
   
   void update_toe0_dtoe0()
  {
    if (robot.t <= 0.003) 
    {
        robot.F_sw_toe0 = robot.toe[robot.F_sw]; 
        robot.B_sw_toe0 = robot.toe[robot.B_sw]; 
   
    }
      
  }
  void estimate_vxy()
{
   static v3TypeDef pre_p = {0,0,0};
          v3TypeDef     p;  
          p = get_sf_vec3f();
        if(robot.t != 0)
        {
         robot.v.x = (p.x - pre_p.x) / (0.001f * (float)TIME_STEP);
         robot.v.y = (p.y - pre_p.y) / (0.001f * (float)TIME_STEP);
        }
       robot.v.x =1*robot.v.x;
       robot.v.y =1*robot.v.y;
       pre_p = p;
}

void robot_control()
{
    vect3TypeDef F_st_toe = {0,0,0};
    vect3TypeDef B_st_toe = {0,0,0};;
    if (robot.F_st == FR || robot.B_st == BL)  
    {
        F_st_toe = robot.toe[FR];
        B_st_toe = robot.toe[BL];
    
    }
    else if(robot.F_st == FL || robot.B_st == BR)  
    {
        F_st_toe = robot.toe[FL];
        B_st_toe = robot.toe[BR];
    }
// ------------------------------------------------ ----------------------  Standing Motion  
   
      matTypeDef Q,J, Jf, Jb,P,Tau,F;
      easyMat_create(&Q,  6, 6);
      easyMat_create(&J,  6, 6);
      easyMat_create(&Jf, 3, 3);
      easyMat_create(&Jb, 3, 3);
      easyMat_create(&P,  6, 6);
      easyMat_create(&Tau,6, 1);
      easyMat_create(&F,  6, 1);
      F_Create(&F);
      J_6_create(&J, &Jf, &Jb, robot.F_st, robot.B_st);
      easyMat_mult_k(-1.0f, &J);
      Q_Inv_Create(&Q, F_st_toe.x, F_st_toe.y, F_st_toe.z, B_st_toe.x, B_st_toe.y, B_st_toe.z);
      easyMat_mult(&P, &J, &Q);
      easyMat_mult(&Tau, &P, &F);
      
     if (robot.F_st == FR)  
     {
         set_motor_torque(FR_M0, (&Tau)->data[0][0]);
         set_motor_torque(FR_M1, (&Tau)->data[1][0]);
         set_motor_torque(FR_M2, (&Tau)->data[2][0]);
         set_motor_torque(BL_M0, (&Tau)->data[3][0]);
         set_motor_torque(BL_M1, (&Tau)->data[4][0]);
         set_motor_torque(BL_M2, (&Tau)->data[5][0]);
     }
     else if (robot.F_st == FL)
     {
         set_motor_torque(FL_M0, (&Tau)->data[0][0]);
         set_motor_torque(FL_M1, (&Tau)->data[1][0]);
         set_motor_torque(FL_M2, (&Tau)->data[2][0]);
         set_motor_torque(BR_M0, (&Tau)->data[3][0]);
         set_motor_torque(BR_M1, (&Tau)->data[4][0]);
         set_motor_torque(BR_M2, (&Tau)->data[5][0]);
     }  
         easyMat_free(&J);
         easyMat_free(&Jf);
         easyMat_free(&Jb);
         easyMat_free(&Q);
         easyMat_free(&P);
         easyMat_free(&F);
         easyMat_free(&Tau);
 
   if(robot.F_sw == FL)
   {
       RunCurve(robot.F_sw_toe0,FL);
       RunCurve(robot.B_sw_toe0,BR);
   }
   else if(robot.F_sw == FR)
   {
       RunCurve(robot.F_sw_toe0,FR);
       RunCurve(robot.B_sw_toe0,BL);
   }
   else
    printf("No leg \n");
}


  void Raibert_Planning(vect3TypeDef* Exp, vect3TypeDef* toe_0)
{
    float la = 0.5;
    float si;
    float kp = 0.001,ki = 0.001;
    float theta0,theta,dev;
    static float  sum = 0;
    si = 2 * PI * robot.t/ (la * robot.Ts) ;
    
    robot.xT = robot.Ts/2 *robot.vxd;
    sum += robot.v.y; 
    theta0 = atan(robot.v.y*robot.Ts/(2*0.346));
    theta  = theta0 + kp*robot.v.y + ki*sum;  
    dev = 0.346*tan(theta);
    if(robot.F_sw == FL)
      robot.yT = 0.15 - robot.F_sw_toe0.y + dev;
    if(robot.F_sw == FR)
      robot.yT = -0.15 - robot.F_sw_toe0.y + dev;
    Exp->x = robot.xT * (si-sin(si)) / (2 * PI) + toe_0->x;
    Exp->y = robot.yT * (si-sin(si)) / (2 * PI) + toe_0->y;
    Exp->z = robot.zh * (1 - cos(si))/ 2 + toe_0->z;
}
   
  void invertKinematics(matTypeDef* Theta, vect3TypeDef* Exp, legNameTypeDef leg)
{
    float c2, s2, mid, c_1, s_1; 
    int delta = 1;
    int u     = 1;
    switch (leg)
      {
         case FL:  {  delta = 1; u = 1   ;break;  }
         case FR:  {  delta = 1; u = -1  ;break;  } 
         case BR:  {  delta = -1; u = -1 ;break;  }
         case BL:  {  delta = -1; u = 1  ;break;  }
         default:break;
       }
    
 
    float a1 = robot.a1;
    float a2 = robot.a2;
    float px = Exp->x - delta*robot.L;
    float py = Exp->y -     u*robot.w;
    float pz = Exp->z;

    mid = px;
    px  = -pz;
    pz  = mid;
 
    c2 = (px * px + py * py + pz * pz - a1 * a1 - a2 * a2) / (2 * a1 * a2);
    s2 = -sqrtf(1 - c2 * c2);
    Theta->data[2][0] = atan2(s2, c2);
    s_1 = -(a1 + a2 * c2) * pz                      - a2 * s2 * sqrt(py * py + px * px) ;
    c_1 = (a1 + a2 * c2) * sqrt(py * py + px * px)  - a2 * s2 * pz;

     Theta->data[1][0] = atan2(s_1 , c_1);
     Theta->data[0][0] = atan2(py, px) ;
     
}
   
   void RunCurve(vect3TypeDef toe,legNameTypeDef leg)
   {
     vect3TypeDef  Exp;
     matTypeDef Theta_sw;
     easyMat_create(&Theta_sw, 3, 1);
     Raibert_Planning(&Exp, &toe);
     invertKinematics(&Theta_sw, &Exp,leg);
        if (robot.F_sw == FL)  
       {
        set_motor_position(FL_M0, (&Theta_sw)->data[0][0]);  
        set_motor_position(FL_M1, (&Theta_sw)->data[1][0]);
        set_motor_position(FL_M2, (&Theta_sw)->data[2][0]);
        set_motor_position(BR_M0, (&Theta_sw)->data[0][0]);  
        set_motor_position(BR_M1, (&Theta_sw)->data[1][0]);
        set_motor_position(BR_M2, (&Theta_sw)->data[2][0]);
       
       }
         if (robot.F_sw == FR)  
       {
        set_motor_position(FR_M0, (&Theta_sw)->data[0][0]);  
        set_motor_position(FR_M1, (&Theta_sw)->data[1][0]);
        set_motor_position(FR_M2, (&Theta_sw)->data[2][0]);  
        set_motor_position(BL_M0, (&Theta_sw)->data[0][0]);  
        set_motor_position(BL_M1, (&Theta_sw)->data[1][0]);
        set_motor_position(BL_M2, (&Theta_sw)->data[2][0]);  
        
       }
    easyMat_free(&Theta_sw);   
   }
  
  void create_T_transJ(matTypeDef* transJ, legNameTypeDef leg)
  {
    float a1 = robot.a1;
    float a2 = robot.a2;

    float c0 = cos(robot.theta[leg][0] / 180.0 * PI);
    float s0 = sin(robot.theta[leg][0] / 180.0 * PI);
    float c1 = cos(robot.theta[leg][1] / 180.0 * PI);
    float s1 = sin(robot.theta[leg][1] / 180.0 * PI);
    float c12 = cos(robot.theta[leg][1] / 180.0 * PI + robot.theta[leg][2] / 180.0 * PI);
    float s12 = sin(robot.theta[leg][1] / 180.0 * PI + robot.theta[leg][2] / 180.0 * PI);

   transJ->data[0][0] =  0;
   transJ->data[0][1] =  c0*(a1*c1 + a2*c12); 
   transJ->data[0][2] =  s0*(a1*c1 + a2*c12); 
   transJ->data[1][0] = -a1*c1     - a2*c12;
   transJ->data[1][1] = -s0*(a1*s1 + a2*s12);
   transJ->data[1][2] =  c0*(a1*s1 + a2*s12);
   transJ->data[2][0] = -a2*c12;
   transJ->data[2][1] = -a2*s0*s12;
   transJ->data[2][2] =  a2*c0*s12;
  }
  
void J_6_create(matTypeDef* J_Mat, matTypeDef* jf, matTypeDef* jb, legNameTypeDef f_leg, legNameTypeDef b_leg)
{
    create_T_transJ(jf, f_leg);
    create_T_transJ(jb, b_leg);
    float q[6][6] = {  {jf->data[0][0],jf->data[0][1],jf->data[0][2],0,0,0},
                       {jf->data[1][0],jf->data[1][1],jf->data[1][2],0,0,0},
                       {jf->data[2][0],jf->data[2][1],jf->data[2][2],0,0,0},
                       { 0, 0, 0, jb->data[0][0], jb->data[0][1], jb->data[0][2]},
                       { 0, 0, 0, jb->data[1][0], jb->data[1][1], jb->data[1][2]},
                       { 0, 0, 0, jb->data[2][0], jb->data[2][1], jb->data[2][2]} };
     easyMat_init(J_Mat, *q);   
}

void  F_Create(matTypeDef* F)
{
    static float pre_h = 0.346; 
    static float pre_psiy = 0;
    float deltaz = (robot.toe[robot.F_st].z - robot.toe[robot.B_st].z);
    float deltax = (robot.toe[robot.F_st].x - robot.toe[robot.B_st].x);
    robot.psiy   = atan(deltaz / deltax);
    robot.dpsiy  = (robot.psiy - pre_psiy) / (0.001*(float)TIME_STEP);
    robot.h      = -(robot.toe[robot.F_st].z + robot.toe[robot.B_st].z) / 2;
    robot.dh     = (robot.h - pre_h) / (0.001*(float)TIME_STEP);
    
 
    pre_psiy     = robot.psiy;
    pre_h        = robot.h; 
    float Fz = robot.Kh*( robot.hd - robot.h) - robot.Kdh*robot.dh;
    F->data[0][0] = robot.Kvx*(robot.vxd - robot.v.x) - robot.M *g*sin(robot.pitch);
    F->data[1][0] = Fz + robot.M*g*cos(robot.pitch);
    
    F->data[2][0] = robot.Krx * robot.roll + robot.Kdrx * robot.droll;
    F->data[3][0] = robot.Kry*robot.psiy +  robot.Kdry* robot.dpsiy;
    F->data[4][0] = robot.Kwz*(robot.dyaw - robot.wzd);
    F->data[5][0] = 0;
}



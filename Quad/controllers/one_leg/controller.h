#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdbool.h>

#include "easyMat.h"
#include "webotsInterface.h"

#define g        (9.8)
 
 
typedef struct
{
  float x;
  float y;
  float z;
}vect3TypeDef;

typedef struct
{
   float    Tf    ;
   float    Ts    ;
   float    xT    ;
   float    yT    ;
   float    wzd   ;
   float    zh    ;
   float    h     ;  
   float    dh    ; 
   float    hd    ;  
   float    psiy  ; 
   float    dpsiy ; 
   float    pitch ; 
   float    dpitch;  
   float    roll  ;  
   float    droll ; 
   float    yaw   ; 
   float    dyaw  ; 
   float    vx    ;
   float    vxd   ;
   //支撑端 F T 参数
   float    Kvx   ;    //Fx;
   float    Kh    ;    //Fz;
   float    Kdh   ;     //Fz;
   float    Krx   ;   //Tx;
   float    Kdrx  ;     //Tx;
   float    Kry   ;   //Ty;
   float    Kdry  ;     //Ty;
   float    Kwz   ;    //Tz;
//------------------------------------------------------------------------模型参数   
   float M  ;  // 50;    
   float a1 ;  // 0.4;   
   float a2 ;  // 0.4;   
   float L  ;  // 0.25;      
   float w  ;  // 0.15;    
   
   float             t              ;   
   legNameTypeDef    F_sw           ;   
   legNameTypeDef    F_st           ;  
   legNameTypeDef    B_sw           ;   
   legNameTypeDef    B_st           ;   
   float             theta[4][3]    ;   
   vect3TypeDef      toe[4]         ;  //4个足底坐标
   vect3TypeDef      v              ;
   vect3TypeDef      F_sw_toe0      ;
   vect3TypeDef      F_st_toe0      ;
   vect3TypeDef      B_sw_toe0      ;
   vect3TypeDef      B_st_toe0      ;
   vect3TypeDef      F_dtoe0        ;
   vect3TypeDef      B_dtoe0        ;
   bool              is_touching[4] ;   
   double            force_touching[4];
   double            ax             ;
   double            ay             ;
   double            az             ;

}robotTypeDef;

extern robotTypeDef robot;

extern void robot_init           ();
extern void updateRobotState     ();
extern void robot_control        ();

#endif
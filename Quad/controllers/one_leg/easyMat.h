#ifndef _EASYMAT_H_
#define _EASYMAT_H_

#include <stdint.h>

#define PI           (3.141592654f)
typedef struct
{
  uint16_t row;
  uint16_t col;
  float** data;
}matTypeDef;

extern void easyMat_create    (matTypeDef* mat, uint16_t row, uint16_t col)              ;
extern void easyMat_free      (matTypeDef* mat)                                          ;
extern void easyMat_init      (matTypeDef* mat, float* data)                             ;
extern void easyMat_mult      (matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)   ;
extern void easyMat_mult_k    (float k, matTypeDef* srcMat)                              ;
extern void easyMat_printf(matTypeDef* mat);
extern void Q_Inv_Create(matTypeDef* Qi, float xf, float yf, float zf, float xb, float yb, float zb);
#endif
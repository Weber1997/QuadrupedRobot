#include <stdio.h>
#include <stdarg.h> 
#include <stdlib.h >
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "easyMat.h"

void easyMat_create(matTypeDef* mat, uint16_t row, uint16_t col)
{
  mat->row = row;
  mat->col = col;
  mat->data = (float **)malloc(row*sizeof(float *));    //指针的指针
  for (int i = 0; i < row; i++)
  {
    mat->data[i] = (float *)malloc(col*sizeof(float));  //每行数据的指针
  }
}

void easyMat_free(matTypeDef* mat)
{
  for (uint16_t i = 0; i < mat->row; i++)
  {
    free(mat->data[i]);
  }
  free(mat->data);
}
void easyMat_init(matTypeDef* mat, float* data)
{
  for (uint16_t i = 0; i < mat->row; i++)
  {
    for (uint16_t j = 0; j < mat->col; j++)
    {
      mat->data[i][j] = data[i*mat->col+j];
    }
  }
}
void easyMat_mult(matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)
{
  for (uint16_t i = 0; i < mat1->row; i++)
  {
    for (uint16_t j = 0; j < mat2->col; j++)
    {
      outMat->data[i][j] = 0;
      for (uint16_t m = 0; m < mat1->col; m++)
      {
        outMat->data[i][j] += mat1->data[i][m] * mat2->data[m][j];
      }
    }
  }
}
void easyMat_mult_k(float k, matTypeDef* srcMat)
{
  for (uint16_t i = 0; i < srcMat->row; i++)
  {
    for (uint16_t j = 0; j < srcMat->col; j++)
    {
      srcMat->data[i][j] *= k;
    }
  }
}

void easyMat_printf(matTypeDef* mat)
{
    int i, j;
    for (i = 0; i < mat->row; i++)
    {
        for (j = 0; j < mat->col; j++)
        {
          printf(" %g ", mat->data[i][j]);
        }
       printf("\n");
    }
}
 
 
 void Q_Inv_Create(matTypeDef* Qi, float xf, float yf, float zf, float xb, float yb, float zb)
{
    float Txz = (2 * (xb * zf - xf * zb));
    float yTxz = (yb - yf) * Txz;

    float p11 = (xb * yb * zf + xb * yf * zb - 2 * xf * yb * zb - xf * yb * zf + xf * yf * zb) / yTxz;
    //P(2,1) = p51
    float p21 = (yb * zf - yf * zb) / Txz;
    //P(3,1) = -p61
    float p31 = -((yb * zf - yf * zb) * (zb + zf)) / yTxz;
    float p41 =  (xb * yb * zf - xb * yf * zb - 2 * xb * yf * zf + xf * yb * zf + xf * yf * zb) / yTxz;
    //P(1,2) = -p42
    float p12 = -(xb * xb * yf - xf * xf * yb - xb * xf * yb + xb * xf * yf) / yTxz;
    //P(2,2) = p52
    float p22 = (xb * yf - xf * yb) / Txz;  
    //P(3,2)
    float p32 = -(xb * yf * zf - xf * yb * zf - 2 * xb * yb * zf + xb * yf * zb + xf * yb * zb) / yTxz;
    float p62 = -(xf * yb * zb - xb * yf * zb + xb * yf * zf + xf * yb * zf - 2 * xf * yf * zb) / yTxz;
    //P(1,3) = -p43
    float p13 =  ((xb + xf) * (xb - xf)) / yTxz;
    float p23 = -(xb - xf) / Txz;
    //P(3,3) = -p63
    float p33 =  ((xb + xf) * (zb - zf)) / yTxz;
    float p53 = -(xb - xf) / Txz;
    //P(1,4) (-p44)
    float p14 =  (xb + xf) / Txz;
    //P(2,4) = p54 
    float p24 = -(yb - yf) / Txz;
    //P(3,4) =-p64
    float p34 =  (zb + zf) / Txz;
    //P(1,5) = -p45
    float p15 = ((zb + zf) * (xb - xf)) / yTxz;
    //P(2,5) = p55
    float p25 = -(zb - zf) / Txz;
    //P(3,5) = -p65
    float p35 = ((zb + zf) * (zb - zf)) / yTxz;

    float q[6][6] = { {p11,p12,p13,p14,p15,0},
                      {p21,p22,p23,p24,p25,0},
                      {p31,p32,p33,p34,p35,0},
                      {p41,-p12,-p13,-p14,-p15,0},
                      {p21,p22,p53,p24,p25,0},
                      {-p31,p62,-p33,-p34,-p35,0}};
     easyMat_init(Qi,*q) ;                  
 }
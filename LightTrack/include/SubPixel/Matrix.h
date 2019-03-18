#ifndef	MATRIX_H
#define MATRIX_H



//用Gauss-Jordan法对矩阵a求逆，得到的逆矩阵仍存入a矩阵
void Matrix_Inverse(double *a,int n);

//m*s的矩阵leftMatrix右乘s*n的矩阵rightMatrix，结果存放在m*n的矩阵resultMatrix中
void  Matrix_Multiply(double *leftMatrix,double *rightMatrix,double *resultMatrix,int m,int s,int n);

#endif
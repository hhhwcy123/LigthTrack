#ifndef	MATRIX_H
#define MATRIX_H



//��Gauss-Jordan���Ծ���a���棬�õ���������Դ���a����
void Matrix_Inverse(double *a,int n);

//m*s�ľ���leftMatrix�ҳ�s*n�ľ���rightMatrix����������m*n�ľ���resultMatrix��
void  Matrix_Multiply(double *leftMatrix,double *rightMatrix,double *resultMatrix,int m,int s,int n);

#endif
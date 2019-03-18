#pragma once
#ifndef _GEOBASE
#define	_GEOBASE

#include <vector>
using namespace std;

#include "opencv2\opencv.hpp"
using namespace cv;


#ifndef max
#define max(a,b)   (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)   (((a) < (b)) ? (a) : (b))
#endif

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#define POLYORDER 9


////////////////////////////////////////////////////////
// ��ؽṹ��
////////////////////////////////////////////////////////
//////////////////////////////
// λ�ú���̬�ṹ��
//////////////////////////////
struct StrPos
{
	double T[3];			// ƽ����
	double Eulor[3];		// ��ת��
	double R[9];			// ��ת����
	bool isuse;				// �Ƿ�ʹ�õı�־
	int index;
	// ��ʼ��
	StrPos()
	{
		memset(T, 0, sizeof(double)*3);
		memset(Eulor, 0, sizeof(double)*3);
		memset(R, 0, sizeof(double)*9);
		R[0] = R[4] = R[8] = 1.0;
		isuse = false;
	}
};


//////////////////////////////
// ���λ�˽ṹ��
//////////////////////////////
struct StrCamPos
{
	string camname;				// �������
	double f[2], C[2];			// ��������
	double k[3], p[2];			// ����ϵ��
	double skew;				// ������ת
	double w, h;				// ���泤��
	StrPos pos;					// �����ʼλ�ú���̬(����ڲο�ϵ)
	// ��ʼ��
	StrCamPos()		
	{ 
		camname = "cam";
		f[0] = f[1] = 4000;
		w = 1280;
		h = 1024;

		C[0] = w/2;
		C[1] = h/2;
		memset(k, 0, sizeof(double)*3);
		memset(p, 0, sizeof(double)*2);
		skew = 0;
	}
	// ��������������
	void DistortionCal(double &x, double &y)
	{
		double x1, y1, x2, y2, r2, r4, r6, x1y1, tmp;
		y1 = (y-C[1])/f[1];
		x1 = (x-C[0])/f[0]-skew*y1;
		r2 = x1*x1 + y1*y1;
		r4 = r2*r2;
		r6 = r4*r2;
		tmp = 1 + k[0]*r2 + k[1]*r4 + k[2]*r6;
		x1y1 = x1*y1;
		x2 = x1*tmp + 2*p[0]*x1y1 + p[1]*(r2 + 2*x1*x1);
		y2 = y1*tmp + p[0]*(r2 + 2*y1*y1) + 2*p[1]*x1y1;
		x = f[0]*x2 + C[0];
		y = f[1]*y2 + C[1];
	}
};


//////////////////////////////
// �궨��ṹ��
// ������Ϊ0һ��Ϊ�ο��궨��
// һ������ֻ����һ���ο��궨��
//////////////////////////////
struct StrBoard
{
	bool isMove;		// �궨���Ƿ�Ϊ�˶���, ���ڲο��궨��, һ��Ϊfalse
	StrPos pos;			// �궨��λ�ú���̬,�����ż�֡����
	// ��ʼ��
	StrBoard()		
	{ 
		isMove = false;
	}
};


//////////////////////////////
// ֡�ṹ��
//////////////////////////////
struct StrFrame
{
	// ÿ֡ת����λ�ú���̬,�����ż�֡����
	// Y = RX + T
	// YΪת���������ϵ����
	// XΪת��ǰ������ϵ����
	vector<StrPos> pos;
	// ��ʼ��
	~StrFrame()	{ pos.clear(); }
};


//////////////////////////////
// ����ƽ���õ��񷽺��﷽������
//////////////////////////////
struct StrPoint
{
	double x[2];		// ����������(�����Ŵ�0��ʼ����),��x��y
	double X[3];		// �����̸�ֲ�����ϵ�е��﷽����(X[2]�����Ϻ�Ϊ0) 
	int iFrame;			// ����Ӧ��֡������
	int iCam;			// ����Ӧ�����������
	int iBoard;			// ����Ӧ�ı궨��������
};


// ���л��������ĺ���(ע���ǽ��޻�������ת�����񷽲���ֵ)
void DistortionCal(StrCamPos *cam, double &x, double &y);

// ����RT����
// ����ͬһ��Ӱ���������̸񣬷ֱ�֪���������̸����Ӱ���RT
// ������̸�1�����̸�2��RT��ģ������
// Y = R1*X1 + T1
// Y = R2*X2 + T2
// X2 = R2T*(R1*X1+T1-T2)
//    = R2T*R1*X1 + R2T*(T1-T2)
void CrossRT(StrPos pos1, StrPos pos2, StrPos &pos3);

// ����RT����
// ģ�����£�
// Y1 = R1*X + T1
// Y2 = R2*X + T2
// Y2 = R2*R1T*Y1 + T2 - R2*R1T1
void CrossRT2(StrPos pos1, StrPos pos2, StrPos &pos3);

// ����RT����, �õ���pos1����ϵ�任��pos2����ϵ��RT
// X2 = R2*(R1*X1 + T1) + T2 = R2*R1*X1 + R2*T1 + T2
void TransportRT(StrPos pos1, StrPos pos2, StrPos &pos3);

// ����RT�任, Y=RX+T, ��֪X��Y
void TransformRT(double *X, double *R, double *T);

// ����RT���任, Y=RX+T, ��֪Y��X
void TransformRTinv(double *Y, double *R, double *T);




///////////////////////////////////////////////////////////////
// ͨ�ú�����
///////////////////////////////////////////////////////////////
class GeoBase
{
public:
	GeoBase(void);
	virtual ~GeoBase(void);

public:
	//////////////////////////////////////////////////////////////////////////
	// ͨ�ú���
	//////////////////////////////////////////////////////////////////////////
	// ��һ���ļ����а���ͨ���ȥ�������������������ļ�(����)
	void DiretorySearch(string dirPath, vector<string> &outPath, string dirCode);
	// ��һ���ļ����������������ļ���(����)
	void DiretorySearchFolder(string dirPath, vector<string> &folder);

public:
	//////////////////////////////////////////////////////////////////////////
	// ��ת������ŷ���ǵ��໥�任
	//////////////////////////////////////////////////////////////////////////
	// ����ת������ŷ����
	void Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3);
	// ��ŷ���ǻ����ת����
	void Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R);

public:
	//////////////////////////////////////////////////////////////////////////
	// �������ת
	//////////////////////////////////////////////////////////////////////////
	// ��X��ת��angle����ת����
	void RotationX(double angle, double *R);
	// ��Y��ת��angle����ת����
	void RotationY(double angle, double *R);
	// ��Z��ת��angle����ת����
	void RotationZ(double angle, double *R);
	
public:
	//////////////////////////////////////////////////////////////////////////
	// �������������
	//////////////////////////////////////////////////////////////////////////
	// ����
	void pNormal(double *a,int n,double b,double *aa, double *ab,double p);
	// ��˹���
	int Gauss(double *ATA,double *ATL,int n);
	// 3*3�ĸ�˹���
	bool solve33(double *A, double *al);
	//�����޵ȣ������������������ڲ������ݴ����е�Ӧ��
	void GaussExt(double *ATA,double *ATL,double *x,int n);
	// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪB 
	void Transpose(double *A,double *B, int m,int n);
	// �����ת�ã��β�mΪ�У�nΪ��,Aת�ú��ΪA
	void Transpose(double *A,int m,int n);
	// ��������,A����Ϊ[m,p],B����Ϊ[p,n],CΪ[m,n] 
	void Multi(double *A,double *B,double *C ,int m,int p,int n);
	// �����ͳ������,A����Ϊ[m,n] 
	void Multi(double *A, int m, int n, double p);
	// ��������,A����Ϊ[m,n],B����Ϊ[m,n],��B����ֵ�ӵ�A������
	void Add(double *A,double *B, int m,int n);
	// ��������,A����Ϊ[m,n],B����Ϊ[m,n],C����Ϊ[m,n],��AB����ֵ�ӵ�C������
	void Add(double *A,double *B, double *C, int m,int n);
	// ��������,A����Ϊ[m,n],B����Ϊ[l, k],��B����ֵ�ӵ�A������,��ӵ����Ͻ�Ϊ(p, q)
	void Add(double *A,double *B, int m,int n, int l, int k, int p, int q);
	// ��������,A����Ϊ[m,n],B����Ϊ[m,n],��B����ֵ�ӵ�A������
	void Minus(double *A,double *B, int m,int n);
	// ��������,A����Ϊ[m,n],B����Ϊ[m,n],C����Ϊ[m,n],��AB����ֵ�ӵ�C������
	void Minus(double *A,double *B, double *C, int m,int n);
	// ��������,A����Ϊ[m,n],B����Ϊ[l, k],��B����ֵ�ӵ�A������,��ӵ����Ͻ�Ϊ(p, q)
	void Minus(double *A,double *B, int m,int n, int l, int k, int p, int q);
	// ���󷴺�,A����Ϊ[m,n],B����Ϊ[m,n]
	void RevSig(double *A,double *B, int m,int n);
	// ���������ʽ
	double Det(double *A,int m);
	// ��A�������C 
	void Inv(double *A,double *C,int m);
	// ��A�������,ֱ�Ӹı�Aֵ
	void Inv33(double *A,double *C,int m);
	// ���������й�һ��
	void NormVector(double *R, int num);
	// ��ȡ������ģ
	double Norm(double *R, int num);
	// �����������
	double Dot(double *A, double *B, int num);
	// �����������
	void CrossMult(double *u, double *v, double *w);

	// ����ʽ���
	void PolynominalFitting(double *x,double *y,int n,int order,double *p);
	// ����ʽ��ϼ��侫��
	void PolynominalFittingError(double *x,double *y,int n,int order,double *error,double *p);
};


#endif


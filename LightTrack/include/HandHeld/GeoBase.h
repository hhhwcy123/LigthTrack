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
// 相关结构体
////////////////////////////////////////////////////////
//////////////////////////////
// 位置和姿态结构体
//////////////////////////////
struct StrPos
{
	double T[3];			// 平移量
	double Eulor[3];		// 旋转量
	double R[9];			// 旋转矩阵
	bool isuse;				// 是否使用的标志
	int index;
	// 初始化
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
// 相机位姿结构体
//////////////////////////////
struct StrCamPos
{
	string camname;				// 相机名称
	double f[2], C[2];			// 主点主距
	double k[3], p[2];			// 畸变系数
	double skew;				// 像面旋转
	double w, h;				// 像面长宽
	StrPos pos;					// 相机初始位置和姿态(相对于参考系)
	// 初始化
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
	// 进行正向畸变计算
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
// 标定板结构体
// 索引号为0一定为参考标定板
// 一个场景只能有一个参考标定板
//////////////////////////////
struct StrBoard
{
	bool isMove;		// 标定板是否为运动的, 对于参考标定板, 一定为false
	StrPos pos;			// 标定板位置和姿态,索引号即帧索引
	// 初始化
	StrBoard()		
	{ 
		isMove = false;
	}
};


//////////////////////////////
// 帧结构体
//////////////////////////////
struct StrFrame
{
	// 每帧转动的位置和姿态,索引号即帧索引
	// Y = RX + T
	// Y为转动后的坐标系坐标
	// X为转动前的坐标系坐标
	vector<StrPos> pos;
	// 初始化
	~StrFrame()	{ pos.clear(); }
};


//////////////////////////////
// 用于平差用的像方和物方点坐标
//////////////////////////////
struct StrPoint
{
	double x[2];		// 像方像素坐标(索引号从0开始计数),先x再y
	double X[3];		// 在棋盘格局部坐标系中的物方坐标(X[2]理论上恒为0) 
	int iFrame;			// 像点对应的帧索引号
	int iCam;			// 像点对应的相机索引号
	int iBoard;			// 物点对应的标定版索引号
};


// 进行畸变修正的函数(注意是将无畸变坐标转化到像方测量值)
void DistortionCal(StrCamPos *cam, double &x, double &y);

// 进行RT传递
// 比如同一张影像两个棋盘格，分别知道两个棋盘格相对影像的RT
// 求从棋盘格1到棋盘格2的RT，模型如下
// Y = R1*X1 + T1
// Y = R2*X2 + T2
// X2 = R2T*(R1*X1+T1-T2)
//    = R2T*R1*X1 + R2T*(T1-T2)
void CrossRT(StrPos pos1, StrPos pos2, StrPos &pos3);

// 进行RT传递
// 模型如下：
// Y1 = R1*X + T1
// Y2 = R2*X + T2
// Y2 = R2*R1T*Y1 + T2 - R2*R1T1
void CrossRT2(StrPos pos1, StrPos pos2, StrPos &pos3);

// 进行RT传递, 得到从pos1坐标系变换到pos2坐标系的RT
// X2 = R2*(R1*X1 + T1) + T2 = R2*R1*X1 + R2*T1 + T2
void TransportRT(StrPos pos1, StrPos pos2, StrPos &pos3);

// 进行RT变换, Y=RX+T, 已知X求Y
void TransformRT(double *X, double *R, double *T);

// 进行RT反变换, Y=RX+T, 已知Y求X
void TransformRTinv(double *Y, double *R, double *T);




///////////////////////////////////////////////////////////////
// 通用函数类
///////////////////////////////////////////////////////////////
class GeoBase
{
public:
	GeoBase(void);
	virtual ~GeoBase(void);

public:
	//////////////////////////////////////////////////////////////////////////
	// 通用函数
	//////////////////////////////////////////////////////////////////////////
	// 从一个文件夹中按照通配符去搜索所有满足条件的文件(单层)
	void DiretorySearch(string dirPath, vector<string> &outPath, string dirCode);
	// 从一个文件夹中搜索所有子文件夹(单层)
	void DiretorySearchFolder(string dirPath, vector<string> &folder);

public:
	//////////////////////////////////////////////////////////////////////////
	// 旋转矩阵与欧拉角的相互变换
	//////////////////////////////////////////////////////////////////////////
	// 从旋转矩阵获得欧拉角
	void Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3);
	// 从欧拉角获得旋转矩阵
	void Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R);

public:
	//////////////////////////////////////////////////////////////////////////
	// 绕轴的旋转
	//////////////////////////////////////////////////////////////////////////
	// 绕X轴转角angle的旋转矩阵
	void RotationX(double angle, double *R);
	// 绕Y轴转角angle的旋转矩阵
	void RotationY(double angle, double *R);
	// 绕Z轴转角angle的旋转矩阵
	void RotationZ(double angle, double *R);
	
public:
	//////////////////////////////////////////////////////////////////////////
	// 矩阵的若干运算
	//////////////////////////////////////////////////////////////////////////
	// 法化
	void pNormal(double *a,int n,double b,double *aa, double *ab,double p);
	// 高斯求解
	int Gauss(double *ATA,double *ATL,int n);
	// 3*3的高斯求解
	bool solve33(double *A, double *al);
	//王新洲等，谱修正迭代法及其在测量数据处理中的应用
	void GaussExt(double *ATA,double *ATL,double *x,int n);
	// 求矩阵转置，形参m为行，n为列,A转置后存为B 
	void Transpose(double *A,double *B, int m,int n);
	// 求矩阵转置，形参m为行，n为列,A转置后存为A
	void Transpose(double *A,int m,int n);
	// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
	void Multi(double *A,double *B,double *C ,int m,int p,int n);
	// 求矩阵和常数相乘,A矩阵为[m,n] 
	void Multi(double *A, int m, int n, double p);
	// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],将B矩阵值加到A矩阵上
	void Add(double *A,double *B, int m,int n);
	// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
	void Add(double *A,double *B, double *C, int m,int n);
	// 求矩阵相加,A矩阵为[m,n],B矩阵为[l, k],将B矩阵值加到A矩阵上,其加的左上角为(p, q)
	void Add(double *A,double *B, int m,int n, int l, int k, int p, int q);
	// 求矩阵相减,A矩阵为[m,n],B矩阵为[m,n],将B矩阵值加到A矩阵上
	void Minus(double *A,double *B, int m,int n);
	// 求矩阵相减,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
	void Minus(double *A,double *B, double *C, int m,int n);
	// 求矩阵相减,A矩阵为[m,n],B矩阵为[l, k],将B矩阵值加到A矩阵上,其加的左上角为(p, q)
	void Minus(double *A,double *B, int m,int n, int l, int k, int p, int q);
	// 矩阵反号,A矩阵为[m,n],B矩阵为[m,n]
	void RevSig(double *A,double *B, int m,int n);
	// 求矩阵行列式
	double Det(double *A,int m);
	// 求A的逆矩阵C 
	void Inv(double *A,double *C,int m);
	// 求A的逆矩阵,直接改变A值
	void Inv33(double *A,double *C,int m);
	// 对向量进行归一化
	void NormVector(double *R, int num);
	// 求取向量的模
	double Norm(double *R, int num);
	// 两个向量点乘
	double Dot(double *A, double *B, int num);
	// 三阶向量叉乘
	void CrossMult(double *u, double *v, double *w);

	// 多项式拟合
	void PolynominalFitting(double *x,double *y,int n,int order,double *p);
	// 多项式拟合及其精度
	void PolynominalFittingError(double *x,double *y,int n,int order,double *error,double *p);
};


#endif


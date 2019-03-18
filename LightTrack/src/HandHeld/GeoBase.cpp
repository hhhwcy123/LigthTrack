#include "HandHeld/GeoBase.h"
#include <math.h>
#include <stdio.h>
#include <string>
#include <map>
using namespace std;

#ifdef WIN32
#include <io.h>
#else
//#include <dirent.h>
#endif


////////////////////////////////////////////////////////
// 进行畸变修正的函数(注意是将无畸变坐标转化到像方测量值)
////////////////////////////////////////////////////////
void DistortionCal(StrCamPos *cam, double &x, double &y)
{
	double x1, y1, x2, y2, r2, r4, r6, x1y1, tmp;
	y1 = (y-cam->C[1])/cam->f[1];
	x1 = (x-cam->C[0])/cam->f[0]-cam->skew*y1;
	r2 = x1*x1 + y1*y1;
	r4 = r2*r2;
	r6 = r4*r2;
	tmp = 1 + cam->k[0]*r2 + cam->k[1]*r4 + cam->k[2]*r6;
	x1y1 = x1*y1;
	x2 = x1*tmp + 2*cam->p[0]*x1y1 + cam->p[1]*(r2 + 2*x1*x1);
	y2 = y1*tmp + cam->p[0]*(r2 + 2*y1*y1) + 2*cam->p[1]*x1y1;
	x = cam->f[0]*x2 + cam->C[0];
	y = cam->f[1]*y2 + cam->C[1];
}


////////////////////////////////////////////////////////
// 进行RT传递
// 比如同一张影像两个棋盘格，分别知道两个棋盘格相对影像的RT
// 求从棋盘格1到棋盘格2的RT，模型如下
// Y = R1*X1 + T1
// Y = R2*X2 + T2
// X2 = R2T*(R1*X1+T1-T2)
//    = R2T*R1*X1 + R2T*(T1-T2)
////////////////////////////////////////////////////////
void CrossRT(StrPos pos1, StrPos pos2, StrPos &pos3)
{
	GeoBase m_base;
	double RT[9], T[3];
	m_base.Transpose(pos2.R, RT, 3, 3);
	m_base.Multi(RT, pos1.R, pos3.R, 3, 3, 3);
	m_base.Minus(pos1.T, pos2.T, T, 3, 1);
	m_base.Multi(RT, T, pos3.T, 3, 3, 1);
}



////////////////////////////////////////////////////////
// 进行RT传递
// 模型如下：
// Y1 = R1*X + T1
// Y2 = R2*X + T2
// Y2 = R2*R1T*Y1 + T2 - R2*R1T1
////////////////////////////////////////////////////////
void CrossRT2(StrPos pos1, StrPos pos2, StrPos &pos3)
{
	GeoBase m_base;
	double R1T[9], T[3];
	m_base.Transpose(pos1.R, R1T, 3, 3);
	m_base.Multi(pos2.R, R1T, pos3.R, 3, 3, 3);
	m_base.Multi(pos3.R, pos1.T, T, 3, 3, 1);
	m_base.Minus(pos2.T, T, pos3.T, 3, 1);
}


////////////////////////////////////////////////////////
// 进行RT传递, 得到从pos1坐标系变换到pos2坐标系的RT
// X2 = R2*(R1*X1 + T1) + T2 = R2*R1*X1 + R2*T1 + T2
////////////////////////////////////////////////////////
void TransportRT(StrPos pos1, StrPos pos2, StrPos &pos3)
{
	GeoBase m_base;
	double T[3];
	m_base.Multi(pos2.R, pos1.R, pos3.R, 3, 3, 3);
	m_base.Multi(pos2.R, pos1.T, T, 3, 3, 1);
	m_base.Add(T, pos2.T, pos3.T, 3, 1);
}


////////////////////////////////////////////////////////
// 进行RT变换, Y=RX+T, 已知X求Y
////////////////////////////////////////////////////////
void TransformRT(double *X, double *R, double *T)
{
	GeoBase m_base;
	double P[3];
	m_base.Multi(R, X, P, 3, 3, 1);
	m_base.Add(P, T, X, 3, 1);
}


////////////////////////////////////////////////////////
// 进行RT反变换, Y=RX+T, 已知Y求X
////////////////////////////////////////////////////////
void TransformRTinv(double *Y, double *R, double *T)
{
	GeoBase m_base;
	double P1[3], P2[3], RT[9];
	m_base.Minus(Y, T, P1, 3, 1);
	m_base.Transpose(R, RT, 3, 3);
	m_base.Multi(RT, P1, Y, 3, 3, 1);
}



//////////////////////////////////////////////////////////////////////////
// 构造/析构函数
//////////////////////////////////////////////////////////////////////////
GeoBase::GeoBase(void)	{}
GeoBase::~GeoBase(void)	{}

//////////////////////////////////////////////////////////////////////////
// 通用函数
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：从一个文件夹中按照通配符去搜索所有满足条件的文件(单层文件)
// 输入:
//		string dirPath:			搜索的文件夹路径
//		vector<string> &outPath:满足条件的文件列表
//		string dirCode：		通配符,如"*.txt"
// 返回值：
//		void
//////////////////////////////////////
/*
void GeoBase::DiretorySearch(string dirPath, vector<string> &outPath, string dirCode)
{
#ifdef WIN32
	int i;
	outPath.clear();
	// begin search
	struct _finddata_t filefind;
	char temp[2]="\\";
	char curr[1024],curr2[1024];
	char currtemp[1024];
	strcpy(curr,dirPath.c_str());
	strcat(curr,temp);
	strcpy(curr2,curr);
	strcat(curr,dirCode.c_str());
	int done=0,handle;
	if((handle=_findfirst(curr,&filefind))==-1)
		return;
	strcpy(currtemp,curr2);
	strcat(currtemp,filefind.name);
	outPath.emplace_back((string)currtemp);
	while(!(done=_findnext(handle,&filefind)))
	{
		if(!strcmp(filefind.name,".."))
			continue;
		strcpy(currtemp,curr2);
		strcat(currtemp,filefind.name);
		outPath.emplace_back((string)currtemp);
	}
	_findclose(handle);
#else

    if(dirPath.size()< 1 || dirPath == "" || dirCode.size() <1 || dirCode == "")
		return;
	DIR* pdir;
	int i;
    struct dirent*	pdir_ent;
    string str_tmp;
    string last_baking;
    int dot_pos;
    if(dirPath[dirPath.size()-1] != '/')
		dirPath += '/';
    int getcount = 0;
	outPath.clear();

    if((pdir = opendir(dirPath.c_str())) == NULL)
    {
        printf("Open Folder %s Failed\n", dirPath.c_str());
        return;
    }
    while((pdir_ent = readdir(pdir)) != NULL)
    {
		str_tmp = pdir_ent->d_name;
		if(pdir_ent->d_type == DT_DIR)
        {
            continue;
        }
        if(strcmp(pdir_ent->d_name, ".") == 0 || strcmp(pdir_ent->d_name, "..")==0 )
        {
            continue;
        }
        if(str_tmp.find(dirCode) == string::npos )
        {
            continue;
        }
        bool add = true;
        for(i=0; i<dirCode.size(); i++)
        {
        	if(str_tmp[str_tmp.size()-dirCode.size()+i]!=dirCode[i])
        		add = false;
        }
        if(add==true)
        	outPath.emplace_back(dirPath + str_tmp);
    }//while
    closedir(pdir);

#endif
}



//////////////////////////////////////////////////////////
// 从一个文件夹中搜索所有子文件夹(单层)
//////////////////////////////////////////////////////////
void GeoBase::DiretorySearchFolder(string dirPath, vector<string> &folder)
{
	folder.clear();
	//文件句柄 
	long  hFile  =  0; 
	//文件信息 
	struct _finddata_t fileinfo; 
	string p; 
	if((hFile = _findfirst(p.assign(dirPath).append("\\*").c_str(),&fileinfo)) != -1) 
	{ 
		do
		{  
			if((fileinfo.attrib & _A_SUBDIR)) 
			{ 
				if(strcmp(fileinfo.name,".") != 0 && strcmp(fileinfo.name,"..") != 0) 
				{
					folder.emplace_back(fileinfo.name);
				}           
			}  
		}while(_findnext(hFile, &fileinfo) == 0); 
		_findclose(hFile); 
	} 
}

*/

////////////////////////////////////////////////////////
// 旋转矩阵与欧拉角的相互变换
////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：从旋转矩阵获得欧拉角
// 输入:
//		double *R：		旋转矩阵3*3，行优先	
//		int ratateOrder:欧拉角转序
// 输出：
//		double &eulor1:	欧拉角1
//		double &eulor2:	欧拉角2
//		double &eulor3:	欧拉角3
// 返回值：
//		void
///////////////////////////////////////
void GeoBase::Matrix2Eulor(double *R, int rotateOrder, double &eulor1, double &eulor2, double &eulor3)
{
	switch(rotateOrder)
	{
		// 第一类:第一次和第三次转动是绕同类坐标轴进行的,第二次转动是绕另两轴中的一类进行的
	case 121:  // 1
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[0]);					double temp = sin(eulor2);
			eulor1 = atan2(R[1]*temp, -R[2]*temp);	eulor3 = atan2(R[3]*temp, R[6]*temp);	break;
		}
	case 131:	// 2
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[0]);					double temp = sin(eulor2);
			eulor1 = atan2(R[2]*temp, R[1]*temp);	eulor3 = atan2(R[6]*temp, -R[3]*temp);	break;
		}
	case 212:	// 3
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[4]);					double temp = sin(eulor2);
			eulor1 = atan2(R[3]*temp, R[5]*temp);	eulor3 = atan2(R[1]*temp, -R[7]*temp);	break;
		}
	case 232:	// 4
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[4]);					double temp = sin(eulor2);
			eulor1 = atan2(R[5]*temp, -R[3]*temp);	eulor3 = atan2(R[7]*temp, R[1]*temp);	break;
		}
	case 313:	// 5
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[8]);					double temp = sin(eulor2);
			eulor1 = atan2(R[6]*temp, -R[7]*temp);	eulor3 = atan2(R[2]*temp, R[5]*temp);	break;
		}
	case 323:	// 6
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			eulor2 = acos(R[8]);					double temp = sin(eulor2);
			eulor1 = atan2(R[7]*temp, R[6]*temp);	eulor3 = atan2(R[5]*temp, -R[2]*temp);	break;
		}
		// 第二类:每次转动是绕不同类别的坐标轴进行的
	case 123:	// 7
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(R[6]);					double temp = cos(eulor2);
			eulor1 = atan2(-R[7]*temp, R[8]*temp);	eulor3 = atan2(-R[3]*temp, R[0]*temp);	break;
		}
	case 132:	// 8
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(-R[3]);					double temp = cos(eulor2);
			eulor1 = atan2(R[5]*temp, R[4]*temp);	eulor3 = atan2(R[6]*temp, R[0]*temp);	break;
		}
	case 213:	// 9
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(-R[7]);					double temp = cos(eulor2);
			eulor1 = atan2(R[6]*temp, R[8]*temp);	eulor3 = atan2(R[1]*temp, R[4]*temp);	break;
		}
	case 231:	// 10
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(R[1]);					double temp = cos(eulor2);
			eulor1 = atan2(-R[2]*temp, R[0]*temp);	eulor3 = atan2(-R[7]*temp, R[4]*temp);	break;
		}
	case 312:	// 11
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(R[5]);					double temp = cos(eulor2);
			eulor1 = atan2(-R[3]*temp, R[4]*temp);	eulor3 = atan2(-R[2]*temp, R[8]*temp);	break;
		}
	case 321:	// 12
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			eulor2 = asin(-R[2]);					double temp = cos(eulor2);
			eulor1 = atan2(R[1]*temp, R[0]*temp);	eulor3 = atan2(R[5]*temp, R[8]*temp);	break;
		}
	}
}


//////////////////////////////////////
// 功能：从欧拉角获得旋转矩阵
// 输入:
//		double &eulor1:	欧拉角1
//		double &eulor2:	欧拉角2
//		double &eulor3:	欧拉角3
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
//		int ratateOrder:欧拉角转序	
// 返回值：
//		void
///////////////////////////////////////
void GeoBase::Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double *R)
{
	double R1[9], R2[9], R3[9], Rtemp[9];
	switch(rotateOrder)
	{
		// 第一类:第一次和第三次转动是绕同类坐标轴进行的,第二次转动是绕另两轴中的一类进行的
	case 121:	// 1
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationX(eulor1, R1);	RotationY(eulor2, R2);	RotationX(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 131:	// 2
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationX(eulor1, R1);	RotationZ(eulor2, R2);	RotationX(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 212:	// 3
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationY(eulor1, R1);	RotationX(eulor2, R2);	RotationY(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 232:	// 4
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationY(eulor1, R1);	RotationZ(eulor2, R2);	RotationY(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 313:	// 5
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationZ(eulor1, R1);	RotationX(eulor2, R2);	RotationZ(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 323:	// 6
		{	// eulor1值域-pi到pi，eulor2值域0到pi，eulor3值域-pi到pi
			RotationZ(eulor1, R1);	RotationY(eulor2, R2);	RotationZ(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
		// 第二类:每次转动是绕不同类别的坐标轴进行的
	case 123:	// 7
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationX(eulor1, R1);	RotationY(eulor2, R2);	RotationZ(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 132:	// 8
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationX(eulor1, R1);	RotationZ(eulor2, R2);	RotationY(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 213:	// 9
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationY(eulor1, R1);	RotationX(eulor2, R2);	RotationZ(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 231:	// 10
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationY(eulor1, R1);	RotationZ(eulor2, R2);	RotationX(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 312:	// 11
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationZ(eulor1, R1);	RotationX(eulor2, R2);	RotationY(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	case 321:	// 12
		{	// eulor1值域-pi到pi，eulor2值域-pi/2到pi/2，eulor3值域-pi到pi
			RotationZ(eulor1, R1);	RotationY(eulor2, R2);	RotationX(eulor3, R3);
			Multi(R2, R1, Rtemp, 3, 3, 3);	Multi(R3, Rtemp, R, 3, 3, 3);	break;
		}
	default:
		{
			printf("Eulor2Matrix Error!\n");	break;	// 没有此种转序存在!
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// 绕轴的旋转
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////
// 功能：绕X轴转角angle的旋转矩阵
// [ 1          0         0     ]
// [ 0    cos(angle)  sin(angle)]
// [ 0   -sin(angle)  cos(angle)]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		void
//////////////////////////////////////
void GeoBase::RotationX(double angle, double *R)
{
	memset(R, 0, sizeof(double)*9);
	R[0] = 1.0;
	R[8] = R[4] = cos(angle);
	R[5] = sin(angle);
	R[7] = -R[5];
}


//////////////////////////////////////
// 功能：绕Y轴转角angle的旋转矩阵
// [cos(angle)  0    -sin(angle)]
// [     0      1         0     ]
// [sin(angle)  0     cos(angle)]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		void
//////////////////////////////////////
void GeoBase::RotationY(double angle, double *R)
{
	memset(R, 0, sizeof(double)*9);
	R[8] = R[0] = cos(angle);
	R[4] = 1.0;
	R[2] = -sin(angle);
	R[6] = -R[2];
}


//////////////////////////////////////
// 功能：绕Z轴转角angle的旋转矩阵
// [ cos(angle)  sin(angle)  0]
// [-sin(angle)  cos(angle)  0]
// [     0            0      1]
// 输入:
//		double angle:	转过的角(弧度)
// 输出：
//		double *R：		旋转矩阵3*3，行优先	
// 返回值：
//		void
//////////////////////////////////////
void GeoBase::RotationZ(double angle, double *R)
{
	memset(R, 0, sizeof(double)*9);
	R[4] = R[0] = cos(angle);
	R[8] = 1.0;
	R[1] = sin(angle);
	R[3] = -R[1];
}



////////////////////////////////////////////////////////////////////////////////////
// 矩阵的若干运算
////////////////////////////////////////////////////////////////////////////////////
// 求矩阵转置，形参m为行，n为列,A转置后存为B 
void GeoBase::Transpose(double *A,double *B, int m,int n)
{
    for (int i=0;i<n;i++)
       for (int j=0;j<m;j++)
           B[i*m+j]=A[j*n+i];
}

// 求矩阵转置，形参m为行，n为列,A转置后存为A 
void GeoBase::Transpose(double *A, int m, int n)
{
	long size = m*n;
	double *B = new double[size];
    for (int i=0;i<n;i++)
       for (int j=0;j<m;j++)
           B[i*m+j]=A[j*n+i];
	memcpy(A, B, sizeof(double)*size);
	if(B!=NULL) { delete []B; B=NULL; };
}

// 求矩阵相乘,A矩阵为[m,p],B矩阵为[p,n],C为[m,n] 
void GeoBase::Multi(double *A,double *B,double *C ,int m,int p,int n)
{
     for (int i=0;i<m;i++)
        for (int j=0;j<n;j++)
        {
            double sum=0;
            for (int k=0;k<p;k++)
                sum=sum+A[i*p+k]*B[k*n+j];
            C[i*n+j]=sum;
        }
}

// 求矩阵和常数相乘,A矩阵为[m,n] 
void GeoBase::Multi(double *A, int m, int n, double p)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			A[i*n+j] *= p;
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],将B矩阵值加到A矩阵上
void GeoBase::Add(double *A,double *B, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			A[i*n+j] += B[i*n+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
void GeoBase::Add(double *A,double *B, double *C, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			C[i*n+j] = A[i*n+j] + B[i*n+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[l, k],将B矩阵值加到A矩阵上,其加的左上角为(p, q)
void GeoBase::Add(double *A,double *B, int m,int n, int l, int k, int p, int q)
{
	for(int i=0; i<l; i++)
	{
		for(int j=0; j<k; j++)
		{
			A[(i+p)*n+j+q] += B[i*k+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],将B矩阵值加到A矩阵上
void GeoBase::Minus(double *A,double *B, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			A[i*n+j] -= B[i*n+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[m,n],C矩阵为[m,n],将AB矩阵值加到C矩阵上
void GeoBase::Minus(double *A,double *B, double *C, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			C[i*n+j] = A[i*n+j] - B[i*n+j];
		}
	}
}


// 求矩阵相加,A矩阵为[m,n],B矩阵为[l, k],将B矩阵值加到A矩阵上,其加的左上角为(p, q)
void GeoBase::Minus(double *A,double *B, int m,int n, int l, int k, int p, int q)
{
	for(int i=0; i<l; i++)
	{
		for(int j=0; j<k; j++)
		{
			A[(i+p)*n+j+q] -= B[i*k+j];
		}
	}
}


// 矩阵反号,A矩阵为[m,n],B矩阵为[m,n]
void GeoBase::RevSig(double *A,double *B, int m,int n)
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			B[i*n+j] = -A[i*n+j];
		}
	}
}


// 求矩阵行列式
double GeoBase::Det(double *A,int m)
{
    int i=0,ii=0,j=0,jj=0,k=0,t=0,tt=1;
    double det=1, mk=0;
    double *pA=new double [m*m];
    double *pB=new double [m];
    for (i=0;i<m;i++)
	{
       pB[i]=0;
       for (j=0;j<m;j++)
          pA[i*m+j]=A[i*m+j];
	}
    for (k=0;k<m;k++)
	{
       for (j=k;j<m;j++)
          if (pA[k*m+j])
		  {
             for (i=0;i<m;i++)
			 {
                  pB[i]=pA[i*m+k];
                  pA[i*m+k]=pA[i*m+j];
                  pA[i*m+j]=pB[i];
			 }
             if (j-k)
                  tt=tt*(-1);
             t=t+1;
             break ;
		  } 
       if (t)
	   {
           for(ii=k+1;ii<m;ii++)
		   {
               mk=(-1)*pA[ii*m+k]/pA[k*m+k];
               pA[ii*m + k] = 0;
               for (jj=k+1;jj<m;jj++)
                   pA[ii*m+jj]=pA[ii*m+jj]+mk*pA[k*m+jj];
		   }
           det=det*pA[k*m+k];
           t=0;
	   }
       else
	   {
           det=0;
           break ;
	   }
	}
    det=det*tt;
    delete pA;	pA = NULL;
    delete pB;	pB = NULL;
    return det;
}

// 求A的逆矩阵C 
void GeoBase::Inv(double *A, double *C, int m)
{
     int i,j,x0,y0;
     double M=0;
     double *SP=new double [m*m];
     double *AB=new double [m*m];
     double *B=new double [m*m];
     M=Det(A,m);
     if(M==0.0)
         return;
     M=1/M;
     for(i=0;i<m;i++)
	 {
         for(j=0;j<m;j++)
		 {
            for(x0=0;x0<m;x0++)
                for (y0=0;y0<m;y0++)
                     B[x0*m+y0]=A[x0*m+y0];
            for(x0=0;x0<m;x0++)
                 B[x0*m+j]=0;
            for(y0=0;y0<m;y0++)
                 B[i*m+y0]=0;
            B[i*m+j]=1;
            SP[i*m+j]=Det(B,m);
            SP[i*m+j]=SP[i*m+j];
            AB[i*m+j]=M*SP[i*m+j];
		 }
	 }
     Transpose(AB,C,m,m);
     delete SP;		SP=NULL;
     delete AB;		AB=NULL;
     delete B;		B=NULL;   
}


// 对向量进行归一化
void GeoBase::NormVector(double *R, int num)
{
	double retVal = 0.0;
	for (int i=0; i < num; i++)
		retVal += pow(R[i],2);
	retVal = sqrt( retVal);
	for (int i=0; i < num; i++)
		R[i] /= retVal;
}


// 求取向量的模
double GeoBase::Norm(double *R, int num)
{
	double retVal = 0.0;
	for (int i=0; i<num; i++)
		retVal += pow(R[i],2);
	return sqrt(retVal);
}


// 两个向量点乘
double GeoBase::Dot(double *A, double *B, int num)
{
	double retVal = 0.0;
	for (int i=0; i < num; i++)
		retVal += A[i]*B[i];
	return retVal;
}


// 三阶向量叉乘
void GeoBase::CrossMult(double *u, double *v, double *w)
{
	w[0] = u[1]*v[2] - u[2]*v[1];
	w[1] = u[2]*v[0] - u[0]*v[2];
	w[2] = u[0]*v[1] - u[1]*v[0];
}

// 法化
void GeoBase::pNormal(double *a, int n, double b, double *aa, double *ab, double p)
{
	int i,j;
	for(i=0; i<n; i++) 
	{
		for (j=0; j<n; j++) 
			*aa++ += p*a[i]*a[j];
		*ab++ += p*a[i]*b;
	}
}

// 高斯求解
int GeoBase::Gauss(double *ATA,double *ATL,int n)
{
	double *ATAinv = new double[n*n];
	double *temp = new double[n];
	Inv(ATA, ATAinv, n);
	Multi(ATAinv, ATL, temp, n, n, 1);
	memcpy(ATL, temp, sizeof(double)*n);
	delete []ATAinv;	ATAinv = NULL;
	delete []temp;		temp = NULL;
	return 1;
}

// 3*3的高斯求解
bool GeoBase::solve33(double *A, double *al)
{
	double calresult[3];
	if(fabs(A[0])<1e-30)
		return false;
	double L10 = A[3]/A[0];
	double L20 = A[6]/A[0];
	double U11 = A[4]-L10*A[1];
	if(fabs(U11)<1e-30)
		return false;
	double L21 = (A[7]-(A[1]*L20))/U11;
	double U12 = A[5]-L10*A[2];
	double U22 = A[8]-L20*A[2]-L21*U12;
	double b0 = al[0];
	double b1 = al[1]-b0*L10;
	double b2 = al[2]-b0*L20-b1*L21;
	if(fabs(U22)<1e-30)
		return false;
	calresult[2] = b2/U22;
	calresult[1] = (b1-U12*calresult[2])/U11;
	calresult[0] = (b0-A[1]*calresult[1]-A[2]*calresult[2])/A[0];
	memcpy(al,calresult,sizeof(double)*3);
	return true;
}


//王新洲等，谱修正迭代法及其在测量数据处理中的应用
void GeoBase::GaussExt(double *ATA, double *ATL, double *x, int n)
{
	double *ATAinv = new double[n*n];
	long i;
	int num=0;
	for(int i=0;i<n;i++)
	{
		ATA[i*n+i]+=1;
	}
	Inv(ATA, ATAinv, n);	
	double *temp=new double[n];
	double *temp1=new double[n];
	double dx0=1e10,dx=1e10,dxx=0;
	do 
	{
		dx0=dx;
		memcpy(temp1,x,sizeof(double)*n);
		for(i=0;i<n;i++)
			temp[i]=ATL[i]+x[i];        
		Multi(ATAinv, temp, x, n, n, 1);		
		dx=0;
		for(i=0;i<n;i++)
		{
			dx+=(x[i]-temp1[i])*(x[i]-temp1[i]);
		}		
		num++;	
	} while(num<10000&&dx<dx0);;
	delete []temp;		temp = NULL;
	delete []temp1;		temp1 = NULL;
	delete []ATAinv;	ATAinv = NULL; 
}



///////////////////////////////////////
// 多项式拟合
//	在这里,一般x为时间,y为其对应的函数
//	n为观测值个数,order是多项式拟合的阶数
//  p为输出的多项式系数
///////////////////////////////////////
void GeoBase::PolynominalFitting(double *x, double *y, int n, int order, double *p)
{
	long i, j;
	double *a=new double[order];
	double *aa=new double[order*order];
	double *al=new double[order];
	double *altmp = new double[order];
	memset(aa,0,sizeof(double)*order*order);
	memset(al,0,sizeof(double)*order);
	memset(altmp,0,sizeof(double)*order);
	for(i=0; i<n; i++)
	{
		a[0] = 1.0;
		for(j=1; j<order; j++)
		{
			a[j]=a[j-1]*x[i];
		}
		pNormal(a, order, y[i], aa, al, 1.0);		
	}
	GaussExt(aa, al, altmp, order);
	memcpy(p, altmp, sizeof(double)*order);

	delete []a;a=NULL;
	delete []aa;aa=NULL;
	delete []al;al=NULL;
}

///////////////////////////////////////
// 多项式拟合及其精度
///////////////////////////////////////
void GeoBase::PolynominalFittingError(double *x,double *y,int n,int order,double *error,double *p)
{
	PolynominalFitting(x, y, n, order, p);
    double *a=new double[order];
	long i, j;
	for(i=0;i<n;i++)
	{
		a[0] = 1.0;
		for(j=1; j<order; j++)
		{
			a[j]=a[j-1]*x[i];
		}
		error[i]=0;
		for(j=0; j<order; j++)
		{
			error[i]+=p[j]*a[j];
		}
		error[i]-=y[i];
	}
	delete []a;	a=NULL;
}



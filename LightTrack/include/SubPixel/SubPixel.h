/*
* Copyright (c) 2007,天津大学国家重点实验室305室
* All right reserved
*
* 文件名称: subpixel.h
* 摘    要: 获取实际轮廓上逐点的亚象素级坐标
*
* 当前版本: 1.1
* 作    者: powerground
* 完成时期: 2007年9月4日
*/

#ifndef SUBPIXEL_H
#define SUBPIXEL_H

// #include "OwnArray.h"
// #include "DIBAPI.H"
// #include "MainFrm.h"

// typedef struct  
// {
// 	int point_X;
// 	int point_Y;
// 	int gradsdirection;
// }   PixelPoint; 

// typedef struct  
// {
// 	float point_X;
// 	float point_Y;
// }   SubPixelPoint; 
#include <opencv2/opencv.hpp>
typedef struct tag_PIXELPOINT
{
	int Height;
	int Width;
}PIXELPOINT, *PPIXELPOINT;

typedef struct tag_REALPOINT
{
	double x;
	double y;
} REALPOINT, *PREALPOINT;

typedef struct tag_CIRPARAMETER
{
    REALPOINT	circleCentre;
	double		radius;
} CIRPARAMETER, *PCIRPARAMETER;

typedef struct tag_EDGEGRAY
{
	unsigned  char  a;
    unsigned  char  b;
	unsigned  char  c;
	unsigned  char  d;
	unsigned  char  e;
	unsigned  char  f;
	unsigned  char  g;
	double          x;
} EDGEGRAY, *PEDGEGRAY;

enum DIB{source,division,contour,contourminus,trace0,trace1,trace2,trace3,trace4,trace5,trace6,trace7};

//梯度只能取四个方向之一，分别是水平，45度，垂直，135度四个方向 
enum Gradsdirection{d0,d45,d90,d135,d180,d225,d270,d315};

class CSubPixel
{
public:
	CSubPixel();

    ~CSubPixel();
/*	{
	    LocalUnlock(lpDivision);
	    LocalFree(hDivision);	
        LocalUnlock(lpTrace1);
		LocalFree(hTrace1);
        LocalUnlock(lpTrace2);
        LocalFree(hTrace2);
	}*/

	/*
public:
	HDIB  m_hSource,m_hDivision,m_hContour,m_hContourMinus;
	CPalette*  m_palDIB;
	DIB		m_dibsign;
	CIRPARAMETER m_circleParameter[100];
	int m_circleParameterNO;
	COwnArray<HDIB, HDIB&>  m_hTrace;
	//LONG  lWidth, lHeight;
	//LPSTR lpDIBBits;
	//LPSTR lpDivision,lpEdge,lpTrace1,lpTrace2;
	//COwnArray<COwnArray<PIXELPOINT, PIXELPOINT&>, COwnArray<PIXELPOINT, PIXELPOINT&>&>  m_realPointContour;
    COwnArray<COwnArray<REALPOINT, REALPOINT&>, COwnArray<REALPOINT, REALPOINT&>&>  m_realPointContour;
	COwnArray<COwnArray<double, double&>, COwnArray<double, double&>&>  m_gradDirectionContour;
    COwnArray<COwnArray<EDGEGRAY, EDGEGRAY&>, COwnArray<EDGEGRAY, EDGEGRAY&>&>  m_edgeGrayContour;
    //COwnArray<COwnArray<REALPOINT, REALPOINT&>, COwnArray<REALPOINT, REALPOINT&>&>  m_subIMPixleContour;
	//COwnArray<COwnArray<REALPOINT, REALPOINT&>, COwnArray<REALPOINT, REALPOINT&>&>  m_translationContour;
    //COwnArray<COwnArray<REALPOINT, REALPOINT&>, COwnArray<REALPOINT, REALPOINT&>&>  m_rotationContour;
    //COwnArray<COwnArray<REALPOINT, REALPOINT&>, COwnArray<REALPOINT, REALPOINT&>&>  m_demarcationContour;
    REALPOINT  m_gearAcme[20];
private: 
	void SelectSort(double *pData, int Count);
	
	void  ThresholdDIB(LPSTR lpSourceBits, LPSTR lpDivisionBits,LONG lWidth,LONG lHeight);
    void  ContourDIB(LPSTR lpDivisionBits,LPSTR lpContourBits,LONG lWidth,LONG lHeight);
	BOOL  TraceDIB(LPSTR lpContourBits,LPSTR lpTraceBits,LONG lWidth,LONG lHeight,int lowerLimit,BOOL& isBlank,BOOL& isDirty,LONG& startLine);
    //Gradsdirection   GradsDirection(PIXELPOINT pixelPoint);
	//FLOAT GradsValue(PIXELPOINT pixelPoint,int iTempH,int iTempW,int iTempMX,int iTempMY,FLOAT * fpArray,FLOAT fTempC);
	//void  BSpline(PIXELPOINT pixelPoint,Gradsdirection gradsdrt);
	
	BOOL  GetThresholdDIB();
    BOOL  GetContourDIB();
	BOOL  GetTraceDIB();

    void  GetCircleEquationMatrix(int edgeNO,double* matrixCoefficient,double* rightVector);
	void  CircleFit(int edgeNO);
	
	double  GetRotationRAD(int iNewOrigin,int iNewCoordinates);
	
    void    GetGearAcme(REALPOINT* pGearAcme,int gearNum,int edgeNO);
	double  GetGearRotationRAD(int gearNum,int edgeNO);

	double  LineFit(COwnArray<REALPOINT, REALPOINT&>& line);
	double  GetQinZongLunRotationRAD();

	void    TransAndRot(REALPOINT& transValue,double rotationRAD,double resolution);
	//void  GetSubCircleEquationMatrix(int edgeNO,double* matrixCoefficient,double* rightVector);
	//void  Rotation(double rotationRAD);
	//void  Demarcation(double resolution);
	//void  Translation(int iNewOrigin);
	//void  Translation(double newOriginX,double newOriginY);
	//double  GetRotationRAD(double newOriginX,double newOriginY,double newCoordinatesX,double newCoordinatesY);
    //void  SubCircleFit(int edgeNO);
	//void  IMSubCircleFit(int edgeNO);


	//法线法求亚像素边缘点
	REALPOINT  GetNormalSubPixelValue(PIXELPOINT pixelPoint,int circleNO);
	void  GetDataPoint(double gradsdirection,PIXELPOINT * pData);
	void  GetM(double* X,unsigned char* Y,double* M);
	double  NormalDirection(PIXELPOINT pixelPoint,int circleNO);
    
	//梯度法求亚像素边缘
	REALPOINT  GetGradsSubPixelValue(PIXELPOINT pixelPoint,signed int* kirschArray,COwnArray<double, double&>& gradsDirectionEdge,COwnArray<EDGEGRAY, EDGEGRAY&>& edgeGrayEdge);
    double  GradsDirection(PIXELPOINT pixelPoint,signed int* kirschArray,COwnArray<double, double&>& gradsDirectionEdge);
	//REALPOINT  GetIMSubPixelValue(PIXELPOINT pixelPoint,int circleNO);
	//int  GradsValue(PIXELPOINT pixelPoint,int iTempH,int iTempW,signed int * pArray);

	//空间矩法求亚像素边缘
	REALPOINT GetSpatialSubPixelValue(PIXELPOINT pixelPoint,double* tempMatrix);
	//friend class CContourCompare;

public:
	CIRPARAMETER CircleParameter[100];
	void CircleParameterSave();
	BOOL  ReadDIB();
	BOOL  Process();
	BOOL  ShowDIB(CDC* pDC,DIB dib);

	void  NormalSubContour();
    void  GradsSubContour();
    void  SpatialMomentSubContour();

	void  Dingdanghuang();
	void  Qinzonglun1();
    void  Shangtiaolun();
    void  Qinzonglun2();
	void  ContourDataSave();
	void  DIBSave(HDIB hdib);
	void  AcmeSave();
	void  AcdisSave();

	*/
};
#endif

REALPOINT  GetGradsSubPixelValue(
	/*unsigned char *imageData,*/
	cv::Mat& imageData,
	PIXELPOINT pixelPoint,
	signed int* kirschArray);

	
	


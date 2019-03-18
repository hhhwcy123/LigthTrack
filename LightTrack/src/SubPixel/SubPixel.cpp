#include <math.h>
#include "SubPixel\SubPixel.h"
#include "SubPixel\OwnArray.h"
#include "SubPixel\Matrix.h"
//得到部分插值样点坐标值。（另一部分对称选取即可得到）
//参数gradsdirection：梯度方向或法线方向（作为插值方向）
//参数pData：一个数组，用于存放由该函数求出的样点坐标值
void  GetDataPoint(double gradsdirection, PIXELPOINT * pData)
{
	if (gradsdirection >= 0 && gradsdirection <= PI / 4)
	{
		double  ym = 0;
		double  d1 = 0, d2 = 0;
		for (int i = 1; i < 4; i++)
		{
			pData[i].Width = i;
			ym = i*tan(gradsdirection);
			d1 = ym - pData[i - 1].Height;
			d2 = pData[i - 1].Height + 1 - ym;
			if ((d1 - d2) > 0)
			{
				pData[i].Height = pData[i - 1].Height + 1;
			}
			else
			{
				pData[i].Height = pData[i - 1].Height;
			}
		}
	}
	else
		return;
// 	else
// 		AfxMessageBox("角度越界");
}

//求梯度方向
double  GradsDirection(
	/*unsigned char* imageData,*/
	cv::Mat& imageData,
	PIXELPOINT pixelPoint, signed int* kirschArray, COwnArray<double, double&>& gradsDirectionEdge)
{
	//int  grads_X =  GradsValue(pixelPoint,3,3,pArray1);
	//int  grads_Y =  GradsValue(pixelPoint,3,3,pArray2);

	//double  detX=pixelPoint.Width - m_circleParameter[circleNO].circleCentre.x;
	//double  detY=pixelPoint.Height - m_circleParameter[circleNO].circleCentre.y;

	//yhl 
	long i, j, m;
	//指向原图像的指针
	unsigned char*	lpSrc;
	//保存计算结果
	int fResult = 0;
// 	LPSTR lpSource = (LPSTR) ::GlobalLock((HGLOBAL)m_hSource);
// 	LPSTR lpSourceBits = ::FindDIBBits(lpSource);
//	long lWidth = (long) ::DIBWidth(lpSource);
	//long lWidth_ = width/**8*/;
	int  maxValue = 0;
	int  indexM = 0;
	for (m = 0; m < 8; m++)
	{
		fResult = 0;
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				//指向DIB（原图像）中第pixelPoint.Height + iTempMY - i行，pixelPoint.Width - iTempMX + j列像素的指针
				//lpSrc = imageData/*(char*)lpSourceBits*/ + lWidth_ * (pixelPoint.Height + 1 - i) + pixelPoint.Width - 1 + j;
				//保存像素值
				//fResult=*lpSrc;
				//unsigned char k = (unsigned char)(*lpSrc);

				unsigned char k = imageData.at<unsigned char>(pixelPoint.Height + 1 - i, pixelPoint.Width - 1 + j);
				fResult += k * (*(kirschArray + 9 * m + i * 3 + j));
			}
		}
		if (fResult >= maxValue)
		{
			maxValue = fResult;
			indexM = m;
		}
	}
	double RAD;
	switch (indexM)
	{
	case 0:
		RAD = 3 * PI / 2;
		break;
	case 1:
		RAD = 5 * PI / 4;
		break;
	case 2:
		RAD = PI;
		break;
	case 3:
		RAD = 3 * PI / 4;
		break;
	case 4:
		RAD = PI / 2;
		break;
	case 5:
		RAD = PI / 4;
		break;
	case 6:
		RAD = 0;
		break;
	case 7:
		RAD = 7 * PI / 4;
		break;
	}
	//gradsDirectionEdge.Add(RAD);
	return RAD;
}

//法线法求某个亚像素边缘点坐标
//将整个坐标平面平均分成8个区域，并且在计算其他7个区域上的数据点时都将其转化为第1个区域即0-PI/4进行计算(不等距)
// REALPOINT  GetNormalSubPixelValue(PIXELPOINT pixelPoint,int circleNO)
// {
//     PIXELPOINT  pData[4];
//     pData[0].Height=0;
// 	pData[0].Width=0;
// 	pData[1].Height=0;
// 	pData[1].Width=0;
// 	pData[2].Height=0;
// 	pData[2].Height=0;
// 	pData[3].Height=0;
// 	pData[3].Width=0;
//     //存储转换到0~PI/4区域上的样点坐标值
// 	PIXELPOINT  pData_[4];
//     pData_[0].Height=0;
// 	pData_[0].Width=0;
// 	pData_[1].Height=0;
// 	pData_[1].Width=0;
// 	pData_[2].Height=0;
// 	pData_[2].Height=0;
// 	pData_[3].Height=0;
// 	pData_[3].Width=0;
//     int  i=0;
// 	PIXELPOINT  pFitPoint[7];
// 	double  gradsdirection=NormalDirection(pixelPoint,circleNO);
//     double  gradsdirection_;
// 	/*CString  str;
// 	str.Format("%f",gradsdirection);
// 	AfxMessageBox(str);*/
// 
// 	if(gradsdirection>=0&&gradsdirection<PI/4)
//     {
//         GetDataPoint(gradsdirection,pData);
//         
// 	}
//     else if (gradsdirection>PI/4&&gradsdirection<=PI/2)
//     {
// 		gradsdirection_=PI/2-gradsdirection;
// 		GetDataPoint(gradsdirection_,pData_);
//         for (i=1;i<4;i++)
//         {
// 			//int  temp;
// 			//temp=pData[i].Height;
//             pData[i].Height=pData_[i].Width;
//             pData[i].Width=pData_[i].Height;
//         }
//     }
//     else if (gradsdirection>=PI/2&&gradsdirection<3*PI/4)
//     {
// 		gradsdirection_=gradsdirection-PI/2;
//         GetDataPoint(gradsdirection_,pData_);
// 		for (i=1;i<4;i++)
//         {
// 			//int  temp;
// 			//temp=pData[i].Height;
//             pData[i].Height=pData_[i].Width;
//             pData[i].Width=-1*pData_[i].Height;
//             //pData[i].Width=-1*pData[i].Width;
//         }
//     }
// 	else if (gradsdirection>=3*PI/4&&gradsdirection<PI)
// 	{
//         gradsdirection_=PI-gradsdirection;
//         GetDataPoint(gradsdirection_,pData_);
//         for (i=1;i<4;i++)
//         {
//             pData[i].Width=-1*pData_[i].Width;
// 			pData[i].Height=pData_[i].Height;
//         }
// 	}
// 	else if (gradsdirection>=PI&&gradsdirection<5*PI/4)
// 	{
// 		gradsdirection_=gradsdirection-PI;
//         GetDataPoint(gradsdirection_,pData_);
//         for (i=1;i<4;i++)
//         {
//             pData[i].Width=-1*pData_[i].Width;
// 			pData[i].Height=-1*pData_[i].Height;
//         }
// 	}
// 	else if (gradsdirection>=5*PI/4&&gradsdirection<3*PI/2)
// 	{
//         gradsdirection_=3*PI/2-gradsdirection;
//         GetDataPoint(gradsdirection_,pData_);
//         for (i=1;i<4;i++)
//         {   
// 			//int  temp;
// 			//temp=pData[i].Height;
//             pData[i].Height=-1*pData_[i].Width;
//             pData[i].Width=-1*pData_[i].Height;
//             //pData[i].Width=-1*pData[i].Width;
// 			//pData[i].Height=-1*pData[i].Height;
//         }
// 	}
//     else if (gradsdirection>=3*PI/2&&gradsdirection<7*PI/4)
//     {
//         gradsdirection_=gradsdirection-3*PI/2;
//         GetDataPoint(gradsdirection_,pData_);
//         for (i=1;i<4;i++)
//         {   
// 			//int  temp;
// 			//temp=pData[i].Height;
//             pData[i].Height=-1*pData_[i].Width;
//             pData[i].Width=pData_[i].Height;
//             //pData[i].Width=pData[i].Width;
// 			//pData[i].Height=-1*pData[i].Height;
//         }
//     }
// 	else 
// 	{
//         gradsdirection_=2*PI-gradsdirection;
//         GetDataPoint(gradsdirection_,pData_);
//          for (i=1;i<4;i++)
//         {   
//             pData[i].Width=pData_[i].Width;
// 			pData[i].Height=-1*pData_[i].Height;
//         }
// 	}
// 
// 	pFitPoint[3].Width=pixelPoint.Width;
//     pFitPoint[3].Height=pixelPoint.Height;
// 
//     for (i=1;i<4;i++)
//     {
// 		pFitPoint[3+i].Width=pixelPoint.Width+pData[i].Width;
//         pFitPoint[3+i].Height=pixelPoint.Height+pData[i].Height;
// 		pFitPoint[3-i].Width=pixelPoint.Width-pData[i].Width;
//         pFitPoint[3-i].Height=pixelPoint.Height-pData[i].Height;
//     }
// 
//     double  X[7];
// 	
// 	X[3]=0.0;
// 	int  detWidth=0;
// 	int  detHeigh=0;
// 	for (i=1;i<4;i++)
// 	{
//         //X[3+i]=sqrt((pData[i].Width-pData[i-1].Width)*(pData[i].Width-pData[i-1].Width) + (pData[i].Height-pData[i-1].Height)*(pData[i].Height-pData[i-1].Height));
// 		//X[3-i]=-1*X[3+i];
// 		detWidth=pData[i].Width-pData[i-1].Width;
// 		detHeigh=pData[i].Height-pData[i-1].Height;
// 		if (fabs(detWidth+detHeigh)==1)
// 		{
//             X[3+i]=X[3+i-1]+1;
// 		}
// 		else
// 		{
//             X[3+i]=X[3+i-1]+sqrt(2.0);
// 		}
//         X[3-i]=-1*X[3+i];
// 	}
// 
// 	unsigned char  Y[7];
// // 	LPSTR lpSource = (LPSTR) ::GlobalLock((HGLOBAL) m_hSource);
// // 	LPSTR lpSourceBits = ::FindDIBBits(lpSource);
// 	    
// 	//long lWidth=(long) WIDTHBYTES(::DIBWidth(lpContour)*8);
// 	long lWidth=(long) ::DIBWidth(lpSource);
// 	long lHeight=(long) ::DIBHeight(lpSource);
// 	long lWidth_=(long)WIDTHBYTES(lWidth*8);
// 
// 	for (i=0;i<7;i++)
// 	{
// 		LPSTR  lpSrc = (char *)lpSourceBits + lWidth_ * (long)pFitPoint[i].Height + (long)pFitPoint[i].Width;
// 			
// 		//取得当前指针处的像素值，注意要转换为unsigned char型
// 		Y[i] = (unsigned char)*lpSrc;
//         unsigned char  k=Y[i];	
// 	}
// 
//     GlobalUnlock((HGLOBAL) m_hSource);
// 
// 	/*unsigned char  Z[7];
// 	for (i=0;i<6;i++)
// 	{
// 		LPSTR lpSource = (LPSTR) ::GlobalLock((HGLOBAL) m_hSource);
// 	    LPSTR lpSourceBits = ::FindDIBBits(lpSource);
// 	    
// 	    //long lWidth=(long) WIDTHBYTES(::DIBWidth(lpContour)*8);
// 	    long lWidth=(long) ::DIBWidth(lpSource);
// 	    long lHeight=(long) ::DIBHeight(lpSource);
// 	    long lWidth_=(long)WIDTHBYTES(lWidth*8);
// 
// 		LPSTR  lpSrc = (char *)lpSourceBits + lWidth_ * (pFitPoint[6].Width+1) + pFitPoint[i].Height;
// 			
// 		//取得当前指针处的像素值，注意要转换为unsigned char型
// 	
//         Z[i]=(unsigned char)*lpSrc;
// 		GlobalUnlock((HGLOBAL) m_hSource);
//         
// 	}*/
// 
// 	/*for (i=0;i<7;i++)
// 	{
// 		CString  str;
// 		str.Format("%d,%d",pFitPoint[i].Width,pFitPoint[i].Height);
// 		AfxMessageBox(str);
// 	}*/
// /*	for (i=0;i<7;i++)
// 	{
// 		CString  str;
// 		str.Format("%,f%d",X[i],Y[i]);
// 		AfxMessageBox(str);
// 	}*/
//         
//     double  M[7];
// 	GetM(X,Y,M);
// 	//double  X2d0[6];
// 	double  Y1dM=0;
// 	double  Y1d;
// 	double  X2d0;
// 	double  XSub;
// 	REALPOINT  Sub;
// 	for (i=1;i<7;i++)
// 	{
//         X2d0= (X[i-1]*M[i]-X[i]*M[i-1])/(M[i]-M[i-1]);
// 		if (X2d0>=X[i-1]&&X2d0<=X[i])
// 		{
//             Y1d=(-1*(X[i]-X2d0)*(X[i]-X2d0)*M[i-1]+(X2d0-X[i-1])*(X2d0-X[i-1])*M[i])/(X[i]-X[i-1])/2+(Y[i]-Y[i-1])/(X[i]-X[i-1])+(X[i]-X[i-1])*(M[i-1]-M[i])/6;
// 	        Y1d=fabs(Y1d);
// 			if (Y1d>Y1dM)
// 			{
// 				Y1dM=Y1d;
//                 XSub=X2d0;
// 			}
// 		}
// 	}
// 
//     /*if (XSub<0)
//     {
// 		if (gradsdirection<PI)
// 		{
//             Sub.x=XSub*cos(gradsdirection+PI);
// 			Sub.y=XSub*sin(gradsdirection+PI);
// 		}
// 		if (gradsdirection>=PI)
// 		{
//             Sub.x=XSub*cos(gradsdirection-PI);
// 			Sub.y=XSub*sin(gradsdirection-PI);
// 		}
//     }*/
// 	//else
// 	//{
//         //double k=180*gradsdirection/PI;
//         //Sub.x=XSub*cos(gradsdirection);
// 		//Sub.y=XSub*sin(gradsdirection);
// 	//}
//    // if (gradsdirection>=0&&gradsdirection<=PI/4)
// 
//     //假设处在0-45度之间。先判断在哪个区间段，再判断该区间是水平的还是斜45度的，用该区间起始点坐标加上该段在x，y轴的投影即得到Sub.x，Sub.y
// 		i=0;
// 		bool  flag=false;
// 		while (i<3&&flag==false)
// 		{
//             if (fabs(XSub)>=X[i+3]&&fabs(XSub)<=X[i+4])
// 			{
// 				double detX=fabs(XSub)-X[i+3];
//                 if ((pData_[i+1].Height-pData_[i].Height)==0)
//                 {
//                     Sub.x=(double)pData_[i].Width+detX;
// 					Sub.y=(double)pData_[i].Height;
//                 }
// 				else
// 				{
//                     Sub.x=(double)pData_[i].Width+detX*cos(PI/4);
// 					Sub.y=(double)pData_[i].Height+detX*sin(PI/4);
// 				}
// 				flag=true;
// 			}
// 			else
// 			{
// 				i++;
// 			}
// 		}
//     
// 	//根据法线方向所在角度范围，做相应变换求得真正的Sub.x，Sub.y
// 	if (gradsdirection>=0&&gradsdirection<=PI/4)
// 	{
// 
// 	}
// 	
// 	else if (gradsdirection>PI/4&&gradsdirection<=PI/2)
// 	{
//         double temp;
// 		temp=Sub.x;
// 		Sub.x=Sub.y;
// 		Sub.y=temp;
// 	}
// 	else if (gradsdirection>PI/2&&gradsdirection<=3*PI/4)
// 	{
//         double temp;
// 		temp=Sub.x;
// 		Sub.x=Sub.y;
// 		Sub.y=temp;
//         Sub.x=-1*Sub.x;
// 	}
// 	else if (gradsdirection>3*PI/4&&gradsdirection<=PI)
// 	{
// 		Sub.x=-1*Sub.x;
// 	}
// 	else if (gradsdirection>PI&&gradsdirection<=5*PI/4)
// 	{
//         Sub.x=-1*Sub.x;
// 		Sub.y=-1*Sub.y;
// 	}
// 	else if (gradsdirection>5*PI/4&&gradsdirection<=3*PI/2)
// 	{
//         double temp;
// 		temp=Sub.x;
// 		Sub.x=Sub.y;
// 		Sub.y=temp;
//         Sub.x=-1*Sub.x;
// 		Sub.y=-1*Sub.y;
// 	}
// 	else if (gradsdirection>3*PI/2&&gradsdirection<=7*PI/4)
// 	{
//         double temp;
// 		temp=Sub.x;
// 		Sub.x=Sub.y;
// 		Sub.y=temp;
//         Sub.y=-1*Sub.y;
// 	}
// 	else 
// 	{
// 		Sub.y=-1*Sub.y;
// 	}
//     //Sub.x=(double)pixelPoint.Width+Sub.x;
// 	//Sub.y=(double)pixelPoint.Height+Sub.y;
//     //return  Sub;
// 	if (XSub<0)
// 	{
//         Sub.x=-1*Sub.x;
// 		Sub.y=-1*Sub.y;
// 	}
// 	Sub.x=(double)pixelPoint.Width+Sub.x;
// 	Sub.y=(double)pixelPoint.Height+Sub.y;
//     return  Sub;
// }

//根据各值节点值和对应函数值求得三次样条曲线M表达式的各M值
void  GetM(double* X, unsigned char* Y, double* M)
{
	double  d[7];
	double  c[49] = { 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0 };
	double  h[7] = { 0, 0, 0, 0, 0, 0, 0 };

	for (int i = 1; i < 7; i++)
	{
		h[i] = X[i] - X[i - 1];
	}

	c[0] = 2;
	c[1] = 1;

	d[0] = 6 * (Y[1] - Y[0]) / h[1] / h[1];

	for (int i = 1; i < 6; i++)
	{
		for (int j = i - 1; j < i + 2; j++)
		{
			if (j == i - 1)
			{
				c[i * 7 + j] = h[i] / (h[i] + h[i + 1]);
			}
			else if (j == i)
			{
				c[i * 7 + j] = 2;
			}
			else
			{
				c[i * 7 + j] = h[i + 1] / (h[i] + h[i + 1]);
			}
		}

		d[i] = 6 / (h[i] + h[i + 1])*((Y[i + 1] - Y[i]) / h[i + 1] - (Y[i] - Y[i - 1]) / h[i]);
	}

	c[47] = 1;
	c[48] = 2;

	d[6] = -1 * 6 * (Y[6] - Y[5]) / h[6] / h[6];
	double  b = 0;
	double  a = 1 / b;
	Matrix_Inverse(c, 7);
	Matrix_Multiply(c, d, M, 7, 7, 1);
}

//梯度法求某个亚像素边缘点坐标
//将整个坐标平面平均分成8个区域，并且在计算其他7个区域上的数据点时都将其转化为第1个区域即0-PI/4进行计算(不等距)
REALPOINT  GetGradsSubPixelValue(
	/*unsigned char *imageData,*/
	cv::Mat& imageData,
// 	int width,
// 	int height,
	PIXELPOINT pixelPoint,
	signed int* kirschArray)
{
	COwnArray<double, double&>  gradsDirectionEdge;
	COwnArray<EDGEGRAY, EDGEGRAY&>  edgeGrayEdge;

    PIXELPOINT  pData[4];
    pData[0].Height=0;
	pData[0].Width=0;
	pData[1].Height=0;
	pData[1].Width=0;
	pData[2].Height=0;
	pData[2].Width = 0;
	pData[3].Height=0;
	pData[3].Width=0;
    int  i=0;
	PIXELPOINT  pFitPoint[7];
	double  gradsdirection = GradsDirection(imageData,pixelPoint, kirschArray, gradsDirectionEdge);
    double  gradsdirection_;
	/*CString  str;
	str.Format("%f",gradsdirection);
	AfxMessageBox(str);*/

	if(gradsdirection>=0&&gradsdirection<=PI/4)
    {
        GetDataPoint(gradsdirection,pData);
	}
    else if (gradsdirection>PI/4&&gradsdirection<=PI/2)
    {
		gradsdirection_=0;
		GetDataPoint(gradsdirection_,pData);
        for (i=1;i<4;i++)
        {
			int  temp;
			temp=pData[i].Height;
            pData[i].Height=pData[i].Width;
            pData[i].Width=temp;
        }
    }
    else if (gradsdirection>PI/2&&gradsdirection<=3*PI/4)
    {
		gradsdirection_=PI/4;
        GetDataPoint(gradsdirection_,pData);
		for (i=1;i<4;i++)
        {
			int  temp;
			temp=pData[i].Height;
            pData[i].Height=pData[i].Width;
            pData[i].Width=temp;
            pData[i].Width=-1*pData[i].Width;
        }
    }
	else if (gradsdirection>3*PI/4&&gradsdirection<=PI)
	{
        gradsdirection_=0;
        GetDataPoint(gradsdirection_,pData);
        for (i=1;i<4;i++)
        {
            pData[i].Width=-1*pData[i].Width;
        }
	}
	else if (gradsdirection>PI&&gradsdirection<=5*PI/4)
	{
		gradsdirection_=PI/4;
        GetDataPoint(gradsdirection_,pData);
        for (i=1;i<4;i++)
        {
            pData[i].Width=-1*pData[i].Width;
			pData[i].Height=-1*pData[i].Height;
        }
	}
	else if (gradsdirection>5*PI/4&&gradsdirection<=3*PI/2)
	{
        gradsdirection_=0;
        GetDataPoint(gradsdirection_,pData);
        for (i=1;i<4;i++)
        {   
			int  temp;
			temp=pData[i].Height;
            pData[i].Height=pData[i].Width;
            pData[i].Width=temp;
            pData[i].Width=-1*pData[i].Width;
			pData[i].Height=-1*pData[i].Height;
        }
	}
    else if (gradsdirection>3*PI/2&&gradsdirection<=7*PI/4)
    {
        gradsdirection_=PI/4;
        GetDataPoint(gradsdirection_,pData);
        for (i=1;i<4;i++)
        {   
			int  temp;
			temp=pData[i].Height;
            pData[i].Height=pData[i].Width;
            pData[i].Width=temp;
            pData[i].Width=pData[i].Width;
			pData[i].Height=-1*pData[i].Height;
        }
    }
	else 
	{
        gradsdirection_=2*PI-gradsdirection;
        GetDataPoint(gradsdirection_,pData);
         for (i=1;i<4;i++)
        {   
            pData[i].Width=pData[i].Width;
			pData[i].Height=-1*pData[i].Height;
        }
	}

	pFitPoint[3].Width=pixelPoint.Width;
    pFitPoint[3].Height=pixelPoint.Height;

    for (i=1;i<4;i++)
    {
		pFitPoint[3+i].Width=pixelPoint.Width+pData[i].Width;
        pFitPoint[3+i].Height=pixelPoint.Height+pData[i].Height;
		pFitPoint[3-i].Width=pixelPoint.Width-pData[i].Width;
        pFitPoint[3-i].Height=pixelPoint.Height-pData[i].Height;
    }

    double  X[7];
	
	X[3]=0.0;
	int  detWidth=0;
	int  detHeigh=0;
	for (i=1;i<4;i++)
	{
        //X[3+i]=sqrt((pData[i].Width-pData[i-1].Width)*(pData[i].Width-pData[i-1].Width) + (pData[i].Height-pData[i-1].Height)*(pData[i].Height-pData[i-1].Height));
		//X[3-i]=-1*X[3+i];
		detWidth=pData[i].Width-pData[i-1].Width;
		detHeigh=pData[i].Height-pData[i-1].Height;
		if (abs(detWidth+detHeigh)==1)
		{
            X[3+i]=X[3+i-1]+1;
		}
		else
		{
            X[3+i]=X[3+i-1]+sqrt(2.0);
		}
        X[3-i]=-1*X[3+i];
	}

	unsigned char  Y[7];
	EDGEGRAY  edgeGray;

	//yhl 
	//  	LPSTR lpSource = (LPSTR) ::GlobalLock((HGLOBAL) m_hSource);
	//  	LPSTR lpSourceBits = ::FindDIBBits(lpSource);
	//		long lWidth=(long) WIDTHBYTES(::DIBWidth(lpContour)*8);
	//  	long lWidth=(long) ::DIBWidth(lpSource);
	//  	long lHeight=(long) ::DIBHeight(lpSource);
	//  	long lWidth_=(long)WIDTHBYTES(lWidth*8);
 	for (i=0;i<7;i++)
 	{
//  		char*  lpSrc = (char *)imageData + width * (long)pFitPoint[i].Height + (long)pFitPoint[i].Width;
//  			
//  		//取得当前指针处的像素值，注意要转换为unsigned char型
//  		Y[i] = (unsigned char)*lpSrc;
//          unsigned char  k=Y[i];	


// 		if (pFitPoint[i].Height < 0 || pFitPoint[i].Height >= imageData.rows ||
// 			pFitPoint[i].Width < 0 || pFitPoint[i].Width >= imageData.cols)
// 		{
// 			continue;
// 		}


		Y[i]  = imageData.at<unsigned char>((long)pFitPoint[i].Height, (long)pFitPoint[i].Width);
		unsigned char k = Y[i];
 	}

	/*unsigned char  Z[7];
	for (i=0;i<6;i++)
	{
		LPSTR lpSource = (LPSTR) ::GlobalLock((HGLOBAL) m_hSource);
	    LPSTR lpSourceBits = ::FindDIBBits(lpSource);
	    
	    //long lWidth=(long) WIDTHBYTES(::DIBWidth(lpContour)*8);
	    long lWidth=(long) ::DIBWidth(lpSource);
	    long lHeight=(long) ::DIBHeight(lpSource);
	    long lWidth_=(long)WIDTHBYTES(lWidth*8);

		LPSTR  lpSrc = (char *)lpSourceBits + lWidth_ * (pFitPoint[6].Width+1) + pFitPoint[i].Height;
			
		//取得当前指针处的像素值，注意要转换为unsigned char型
	
        Z[i]=(unsigned char)*lpSrc;
		GlobalUnlock((HGLOBAL) m_hSource);
        
	}*/

	/*for (i=0;i<7;i++)
	{
		CString  str;
		str.Format("%d,%d",pFitPoint[i].Width,pFitPoint[i].Height);
		AfxMessageBox(str);
	}*/

	/*	for (i=0;i<7;i++)
	{
		CString  str;
		str.Format("%,f%d",X[i],Y[i]);
		AfxMessageBox(str);
	}*/
        
    double  M[7];
	GetM(X,Y,M);
	//double  X2d0[6];
	double  Y1dM=0;
	double  Y1d;
	double  X2d0;
	double  XSub=0;
	REALPOINT  Sub;
	for (i=1;i<7;i++)
	{
        X2d0= (X[i-1]*M[i]-X[i]*M[i-1])/(M[i]-M[i-1]);
		if (X2d0>=X[i-1]&&X2d0<=X[i])
		{
            Y1d=(-1*(X[i]-X2d0)*(X[i]-X2d0)*M[i-1]+(X2d0-X[i-1])*(X2d0-X[i-1])*M[i])/(X[i]-X[i-1])/2+(Y[i]-Y[i-1])/(X[i]-X[i-1])+(X[i]-X[i-1])*(M[i-1]-M[i])/6;
	        Y1d=fabs(Y1d);
			if (Y1d>Y1dM)
			{
				Y1dM=Y1d;
                XSub=X2d0;
			}
		}
	}
    edgeGray.a=Y[0];
	edgeGray.b=Y[1];
	edgeGray.c=Y[2];
	edgeGray.d=Y[3];
	edgeGray.e=Y[4];
	edgeGray.f=Y[5];
	edgeGray.g=Y[6];
    edgeGray.x=XSub;
    //edgeGrayEdge.Add(edgeGray);
    /*if (XSub<0)
    {
		if (gradsdirection<PI)
		{
            Sub.x=XSub*cos(gradsdirection+PI);
			Sub.y=XSub*sin(gradsdirection+PI);
		}
		if (gradsdirection>=PI)
		{
            Sub.x=XSub*cos(gradsdirection-PI);
			Sub.y=XSub*sin(gradsdirection-PI);
		}
    }*/
	//else
	//{
        double k=180*gradsdirection/PI;
        Sub.x=XSub*cos(gradsdirection);
		Sub.y=XSub*sin(gradsdirection);
	//}

    Sub.x=(double)pixelPoint.Width+Sub.x;
	Sub.y=(double)pixelPoint.Height+Sub.y;
    return  Sub;
}

 
#ifndef	OWNARRAY_H
#define OWNARRAY_H
/*#include <afxtempl.h> */
#include <math.h>

#define  PI  3.141592653589
template <class TYPE, class ARG_TYPE>
class COwnArray //: public CArray <TYPE, ARG_TYPE> 
{
public:
	COwnArray<TYPE, ARG_TYPE>(void);
	COwnArray<TYPE, ARG_TYPE>(const COwnArray<TYPE, ARG_TYPE> &rhs);
	void operator=(const COwnArray<TYPE, ARG_TYPE> &rhs);
};
//自定义拷贝构造函数
template <class TYPE, class ARG_TYPE>
COwnArray<TYPE, ARG_TYPE>::COwnArray<TYPE, ARG_TYPE>(const COwnArray<TYPE, ARG_TYPE> &rhs)
{
	RemoveAll();
	Append(rhs);
}
//自定义无参构造函数
template <class TYPE, class ARG_TYPE>
COwnArray<TYPE, ARG_TYPE>::COwnArray<TYPE, ARG_TYPE>(void)
{

}
//自定义赋值构造函数
template <class TYPE, class ARG_TYPE>
void COwnArray<TYPE, ARG_TYPE>::operator=(const COwnArray<TYPE, ARG_TYPE> &rhs)
{
	RemoveAll();
	Append(rhs);
}

//用Gauss-Jordan法对矩阵a求逆，得到的逆矩阵仍存入a矩阵
/*void Matrix_Inverse(double *a,int n)
{
	  int   *is,*js,i,j,k,l,u,v;   
	  double  d,p;   
    
	  is   =   new   int[n*sizeof(int)];   
	  js   =   new   int[n*sizeof(int)];   
    
	  for(k=0;k<n;k++)   
	  {   
	  d=0.0;   
	  for(i=k;i<n;i++)   
	  for(j=k;j<n;j++)   
	  {   
	  l=i*n+j; p=fabs(a[l]);   
	  if(p>d) {   d=p;   is[k]=i;   js[k]=j;   }   
	  }   
    
	  if(d+1.0==1.0)   
	  {   
	  delete   is;   delete   js;   
	  AfxMessageBox("error   **not   inverse",MB_OK);   
	  //return(0);   
	  }   
    
	  if(is[k]!=k)   
	  for(j=0;j<n;j++)   
	  {   
	  u=k*n+j;   v=is[k]*n+j;   
	  p=a[u];   a[u]=a[v];   a[v]=p;   
	  }   
    
	  if(js[k]!=k)   
	  for(i=0;i<n;i++)   
	  {   
	  u=i*n+k;   v=i*n+js[k];   
	  p=a[u];   a[u]=a[v];   a[v]=p;   
	  }   
    
	  l=k*n+k;   
	  a[l]=1.0/a[l];   
    
	  for(j=0;j<n;j++)   
	  if(j!=k)   
	  {   
	  u=k*n+j;   a[u]=a[u]*a[l];   
	  }   
    
	  for(i=0;i<n;i++)   
	  if(i!=k)   
	  for(j=0;j<n;j++)   
	  if(j!=k)   
	  {   
	  u=i*n+j;   
	  a[u]=a[u]-a[i*n+k]*a[k*n+j];   
	  }   
    
	  for(i=0;i<n;i++)   
	  if(i!=k)   
	  {   
	  u=i*n+k;   a[u]=-a[u]*a[l];   
	  }   
	  }   
    
	  for(k=n-1;k>=0;k--)   
	  {   
	  if(js[k]!=k)   
	  for(j=0;j<n;j++)   
	  {   
	  u=k*n+j;   v=js[k]*n+j;   
	  p=a[u];   a[u]=a[v];   a[v]=p;   
	  }   
	  if(is[k]!=k)   
	  for(i=0;i<n;i++)   
	  {   
	  u=i*n+k;   v=i*n+is[k];   
	  p=a[u];   a[u]=a[v];   a[v]=p;   
	  }   
	  }   
    
	  delete   is;   delete   js;   
	  //return(1);   
}
//m*s的矩阵leftMatrix右乘s*n的矩阵rightMatrix，结果存放在m*n的矩阵resultMatrix中
void  Matrix_Multiply(double *leftMatrix,double *rightMatrix,double *resultMatrix,int m,int s,int n)
{
	for (int i=0;i<m;i++)
	{
		for (int j=0;j<n;j++)
		{
            resultMatrix[i*n+j]=0;
			for (int k=0;k<s;k++)
			{
                resultMatrix[i*n+j]=resultMatrix[i*n+j]+leftMatrix[i*s+k]*rightMatrix[k*n+j];
			}
		}
	}
}*/
#endif
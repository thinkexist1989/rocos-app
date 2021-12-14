


#pragma once
#include "robotMath.h"

/***************************************
 * nearest integer function
 **************************************/
//int round(double x)
//{
//	double temp = x - floor(x);
//
//	if (temp > 0.5)
//	{
//		return floor(x) + 1;
//	}
//	else
//	{
//		return floor(x);
//	}
//}

/***************************************
 * factorial function
 **************************************/
int factorial(int x)
{
	int i;
	int y=1;
	for(i=1;i<=x;i++){
		y = y*i;
	}
	return y;
}

/***************************************
 * function for norm distance in R3
 **************************************/
double norm3(double* a)
{
	double c;
	c = sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);

	return c;
}

/***************************************
 * function for distance from a[3] to b[3]
 **************************************/
double dis3(double* a, double* b)
{
	double c;

	c = sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));

	return c;
}

/*********************************************
 * function for dot multiplication y=x1.x2
 ********************************************/
double dot3(double* x1, double* x2)
{
	double y = x1[0]*x2[0] + x1[1]*x2[1] + x1[2]*x2[2];
	return y;
}

/*********************************************
 * function for cross multiplication y=x1×x2
 ********************************************/
void cross3(double* y, double* x1, double* x2)
{
    y[0] = x1[1]*x2[2] - x1[2]*x2[1];
    y[1] = x1[2]*x2[0] - x1[0]*x2[2];
    y[2] = x1[0]*x2[1] - x1[1]*x2[0];

}



/******************************************************************************************
 * 求解一元二次方程最大实数根：		a*x^2 + b*x + c = 0
 * Input:
 * a,b,c:				方程系数
 *
 * Output:
 * x:					方程根
 *
 * return:				”0“——正确，”-1“——错误
 * ***************************************************************************************/
int root2(double a, double b, double c, double* x1, double* x2)
{
	if (a != 0)
	{
		double A = b*b - 4*a*c;
		double B =-b/(2*a);
		double C;
		//cout << "A = " << A << endl;
		if (A < 0)
		{
			//DEBUG("error: no real root in <root2>\n");
			return -1;
		}
		else
		{
			C = sqrt(A)/(2*a);
			*x1 = B + C;
			*x2 = B - C;
			return 0;
		}
	}
	else
	{
		DEBUG("error: not a quadratic equation in <root2>\n");
		return -1;
	}
}


/******************************************************************************************
 * 求解一元三次方程最大实数根：		a*x^3 + b*x^2 + c*x + d = 0
 * Input:
 * a,b,c,d:				方程系数
 *
 * Output:
 * x:					方程根
 *
 * return:				”0“——正确，”-1“——错误
 * ***************************************************************************************/
int root3(double a, double b, double c, double d, double* z)
{
	if (a != 0)
	{
		double A = b*b - 3*a*c;
		double B = b*c - 9*a*d;
		double C = c*c - 3*b*d;
		double D = B*B - 4*A*C;

		if ((0 == A)&&(0 == B))
		{
			*z = -b/(3*a);
			//cout << "1 1" << endl;
			return 0;
		}
		if(D > 0)
		{
			double y1 = A*b + 1.5*a*(-B + sqrt(B*B - 4*A*C));
			double y2 = A*b + 1.5*a*(-B - sqrt(B*B - 4*A*C));
			//cout << "y1 = " << y1 << endl;
			//cout << "y2 = " << y2 << endl;
			if(y1 > 0 && y2 > 0)
			{
				*z = (-b - (pow(y1,1/3.0) + pow(y2,1/3.0)))/(3*a);
			}
			if(y1 <= 0 && y2 <= 0)
			{
				*z = (-b - (-pow(-y1,1/3.0) +(-pow(-y2,1/3.0))))/(3*a);
			}
			if(y1 > 0 && y2 <= 0)
			{
				*z = (-b - (pow(y1,1/3.0) + (-pow(-y2,1/3.0))))/(3*a);
			}
			if(y1 <= 0 && y2 > 0)
			{
				*z = (-b - (-pow(-y1,1/3.0) + pow(y2,1/3.0)))/(3*a);
			}
			//cout << "1 2" << endl;
			return 0;
		}
		if ((0 == D)&&(A != 0))
		{
			double k = B/A;
			//z = -k/2;
			*z = -b/a + k;
			//cout << "1 3" << endl;
			return 0;
		}
		if ((D < 0)&&(A > 0))
		{
			double T = (2*A*b - 3*a*B)/(2*sqrt(A)*sqrt(A)*sqrt(A));
			double num = 0.0;

			if(T<=-1.0000000000000){
				num=PI;
			}else if(T>=1.0000000000000){
				num=0;
			}else{
				num=acos(T);
			}

			//z = (-b - 2*sqrt(A)*cos(acos(T)/3))/(3*a);
			//z = (-b + sqrt(A)*(cos(acos(T)/3) - sqrt(3)*sin(acos(T)/3)))/(3*a);
			//z = (-b + sqrt(A)*(cos(acos(T)/3) + sqrt(3)*sin(acos(T)/3)))/(3*a);
			*z = (-b + sqrt(A)*(cos(num/3.0) + sqrt(3.0)*sin(num/3.0)))/(3.0*a);

			return 0;
		}
		return 0;
	}
	else
	{
		//cout << "The equation is not a threeEquation !!" << endl;
		DEBUG("error: not a cubic equation in <root3>\n");
		return -1;
	}
}


/******************************************************************************************
 * 求解一元四次方程实数根：		a0*x^4 + b0*x^3 + c0*x^2 + d0*x + e0 = 0
 * Input:
 * a0,b0,c0,d0,e0:		方程系数
 *
 * Output:
 * x:					方程根
 *
 * return:				方程根个数	"-1"——错误，"0"——没有实根
 * ***************************************************************************************/
int root4(double a0,double b0,double c0,double d0,double e0,double* x)
{
	int size = 0;
	double x1,x2,x3,x4;

	if (a0 != 0)
	{
		// 四次方项系数化为一
		// double a = 1;
		double b = b0/a0;
		double c = c0/a0;
		double d = d0/a0;
		double e = e0/a0;

		double a3 = 1;
		double b3 = -c;
		double c3 = b*d - 4*e;
		double d3 = 4*e*c - d*d - e*b*b;
		// 求三次方程的任一实根z
		double z;
		if (root3(a3,b3,c3,d3,&z)==0)
		{
			//cout << "z = " << z << endl;
		}
		//求第一个二次方程的实根x1,x2
		double y = z;

		if((0.25*b*b + y - c)==0)
		{
			//cout <<"0.5by-d = " << 0.5*b*y - d << endl;
			//cout <<"0.25y*y - e = " << 0.25*y*y - e << endl;
			double a2 = 1;
			double b2 = 0.5*b;
			double c2 = 0.5*y + sqrt(0.25*y*y - e);
			if (root2(a2,b2,c2,&x1,&x2)==0)
			{
				//cout << " OK 1 1" << endl;
				size = size + 2;
				x[0] = x1;
				x[1] = x2;
			}
			b2 = 0.5*b;
			c2 = 0.5*y - sqrt(0.25*y*y - e);
			if (root2(a2,b2,c2,&x3,&x4)==0)
			{
				//	cout << " OK 1 2" << endl;
				size = size + 2;

				if(size>=4){
					x[2] = x3;
					x[3] = x4;
				}
				else{
					x[0] = x3;
					x[1] = x4;
				}
			}
		}
		else
		{

			double a2 = 1;
			double b2 = 0.5*b - sqrt(0.25*b*b + y - c);
			double c2 = 0.5*y - (0.25*b*y - 0.5*d)/sqrt(0.25*b*b + y - c);
			if (root2(a2,b2,c2,&x1,&x2)==0)
			{
				//cout << " OK 2 1" << endl;
				size = size + 2;
				x[0] = x1;
				x[1] = x2;
			}
			else{
				;
			}
			//求第二个二次方程的实根x3,x4
			b2 = 0.5*b + sqrt(0.25*b*b + y - c);
			c2 = 0.5*y + (0.25*b*y - 0.5*d)/sqrt(0.25*b*b + y - c);
			if (root2(a2,b2,c2,&x3,&x4)==0)
			{
				size = size + 2;

				if(size>=4){
					x[2] = x3;
					x[3] = x4;
				}
				else{
					x[0] = x3;
					x[1] = x4;
				}
			}
			else{
				;
			}
		}

		if (size==0){
			DEBUG("error: no real root in <root4>\n");
			return 0;
		}
		else{
			return size;
		}

	}
	else{
		DEBUG("error: not a quartic equation in <root4>\n");
		return -1;
	}
}


/******************************************************************************************
 * 追赶法求解三对角线性齐次方程组
 * Input:
 * n:				方程个数
 * mid:				系数矩阵中间对角元素组成向量
 * up:				系数矩阵中间以上对角元素组成向量
 * down:			系数矩阵中间以下对角元素组成向量
 * b:				常系数向量
 *
 * Output:
 * x:				方程根
 *
 * return:
 * ***************************************************************************************/
int crout(double* mid, double* up, double* down, double* b, double* x, int n)
{
	int i;

	//double p[n],q[n-1],y[n];
	double* p = new double[n];
	double* q = new double[n-1];
	double* y = new double[n];

	p[0] = mid[0];

	for(i=0;i<n-1;i++){
		q[i] = up[i]/p[i];
		p[i+1] = mid[i+1]-down[i]*q[i];
	}

	y[0] = b[0]/p[0];

	for(i=1;i<n;i++){
		y[i] = (b[i]-down[i-1]*y[i-1])/p[i];
	}

	x[n-1] = y[n-1];

	for(i=n-2;i>=0;i--){
		x[i] = y[i] - q[i]*x[i+1];
	}

	// 求解L、U矩阵
/*
	double L[n][n],U[n][n];
	memset(L,0,sizeof(L));
	memset(U,0,sizeof(L));
	for(i=0;i<n;i++){
		L[i][i] = p[i];
		U[i][i] = 1;
	}

	for(i=0;i<n-1;i++){
		L[i+1][i] = d[i];
		U[i][i+1] = q[i];
	}
*/
	return 0;
}

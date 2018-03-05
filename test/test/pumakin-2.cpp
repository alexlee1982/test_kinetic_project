/*****************************************************************
* Description: pumakins.c
*   Kinematics for puma typed robots
*   Set the params using HAL to fit your robot
*
*   Derived from a work by Fred Proctor
*
* Author: 
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
* $Revision: 1.4 $
* $Author: alex_joni $
* $Date: 2007/03/03 22:34:32 $
*******************************************************************
*/ 
#include<math.h>
#include <stdio.h>
#include "posemath.h"
#include "pumakins.h"
#include "kinematics.h"             /* decls for kinematicsForward, etc. */

#define DBL_MIN  0.000001


int isEqual(double v1, double v2)
{
	if (fabs(v1 - v2) < 1e-6)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

int cartIsEqual(PmCartesian c1, PmCartesian c2)
{
	if ((0 == isEqual(c1.x, c2.x)) && (0 == isEqual(c1.y, c2.y)) && (0 == isEqual(c1.z, c2.z)))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}


int matricIsEqual(PmRotationMatrix m1, PmRotationMatrix m2)
{
	if ((0 == cartIsEqual(m1.x, m2.x)) && (0 == cartIsEqual(m1.y, m2.y)) && (0 == cartIsEqual(m1.z, m2.z)))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}



  	 double PUMA_A1=100.0;
	 double PUMA_A2=300.0;
	 double PUMA_A3=20.0;
	 double PUMA_D4=300.0;
	 double PUMA_D6=100.0;
	 double PUMA_D3 = 10.0;

	 double L2 = 20.0;
	 double L3 = 30.0;
	 double L4 = 40.0;
	 double L5 = 50.0;


int kinematicsForward2( double * joint,
                      PmRotationMatrix* Ehom_M,PmCartesian* Ehom_P,
                       KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
   
		double s1, s2, s3, s4, s5, s6;
		double c1, c2, c3, c4, c5, c6;
//		double f[16],h[16],g[16];
		double th1, th2, th3, th4 ,th5, th6;
        PmHomogeneous hom;
		double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;
 //   world->tran.x= joint[0];
 //   world->tran.y= joint[1];
 //   world->tran.z= joint[2];
 //   world->a=joint[3] ;
 //   world->b=joint[4] ;
  //  world->c=joint[5] ;




		joint[1] = joint[1] - 90;
		joint[2] = joint[2] - 90;
		/* Calculate sin of joints for future use */




		s1 = sin(joint[0]*PM_PI/180);
		s2 = sin(joint[1]*PM_PI/180);
		s3 = sin(joint[2]*PM_PI/180);
		s4 = sin(joint[3]*PM_PI/180);
		s5 = sin(joint[4]*PM_PI/180);
		s6 = sin(joint[5]*PM_PI/180);

		/* Calculate cos of joints for future use */
		c1 = cos(joint[0]*PM_PI/180);
		c2 = cos(joint[1]*PM_PI/180);
		c3 = cos(joint[2]*PM_PI/180);
		c4 = cos(joint[3]*PM_PI/180);
		c5 = cos(joint[4]*PM_PI/180);
		c6 = cos(joint[5]*PM_PI/180);


		nx = c2*s6 - c6*(c5*(c3*c4*s2 - s2*s3*s4) - s5*(c3*s2*s4 + c4*s2*s3));
		ny = s1*s2*s6 - c6*(c5*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3)) + s5*(c4*(c1*c3 + c2*s1*s3) - s4*(c1*s3 - c2*c3*s1)));
		nz = c2*s6 - c6*(c5*(c3*c4*s2 - s2*s3*s4) - s5*(c3*s2*s4 + c4*s2*s3));

		ox = c1*c6*s2 - s6*(c5*(c4*(s1*s3 + c1*c2*c3) + s4*(c3*s1 - c1*c2*s3)) + s5*(c4*(c3*s1 - c1*c2*s3) - s4*(s1*s3 + c1*c2*c3)));
		oy = s6*(c5*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3)) + s5*(c4*(c1*c3 + c2*s1*s3) - s4*(c1*s3 - c2*c3*s1))) + c6*s1*s2;
		oz = c2*c6 + s6*(c5*(c3*c4*s2 - s2*s3*s4) - s5*(c3*s2*s4 + c4*s2*s3));

		ax = c5*(c4*(c3*s1 - c1*c2*s3) - s4*(s1*s3 + c1*c2*c3)) - s5*(c4*(s1*s3 + c1*c2*c3) + s4*(c3*s1 - c1*c2*s3));
		ay = s5*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3)) - c5*(c4*(c1*c3 + c2*s1*s3) - s4*(c1*s3 - c2*c3*s1));
		az = c5*(c3*s2*s4 + c4*s2*s3) + s5*(c3*c4*s2 - s2*s3*s4);

		px = L2*(c5*(c4*(s1*s3 + c1*c2*c3) + s4*(c3*s1 - c1*c2*s3)) + s5*(c4*(c3*s1 - c1*c2*s3) - s4*(s1*s3 + c1*c2*c3))) - L5*s1 + L4*(s1*s3 + c1*c2*c3) + L3*(c4*(s1*s3 + c1*c2*c3) + s4*(c3*s1 - c1*c2*s3));
		py = L5*c1 - L2*(c5*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3)) + s5*(c4*(c1*c3 + c2*s1*s3) - s4*(c1*s3 - c2*c3*s1))) - L4*(c1*s3 - c2*c3*s1) - L3*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3));
		pz = -L2*(c5*(c3*c4*s2 - s2*s3*s4) - s5*(c3*s2*s4 + c4*s2*s3)) - L3*(c3*c4*s2 - s2*s3*s4) - L4*c3*s2;

		hom.rot.x.x = nx;
		hom.rot.x.y = ny;
		hom.rot.x.z = nz;

		hom.rot.y.x = ox;
		hom.rot.y.y = oy;
		hom.rot.y.z = oz;

		hom.rot.z.x = ax;
		hom.rot.z.y = ay;
		hom.rot.z.z = az;

		hom.tran.x = px;
		hom.tran.y = py;
		hom.tran.z = pz;
		*Ehom_M = hom.rot;
		*Ehom_P = hom.tran;



	return 0;

}
int kinematicsForward_axis5(double * joint,
							RobotPose* world,
							PmRotationMatrix* Ehom,
							KINEMATICS_FORWARD_FLAGS * fflags,
							KINEMATICS_INVERSE_FLAGS * iflags)
{ 
	double s1, s2, s3, s4, s5, s345, s34;
	double c1, c2, c3, c4, c5, c345, c34;
	double theta345,theta34;
	PmHomogeneous hom;

	double L1, L2, L3, L4;
	L1 = 100;
	L2 = 345;
	L3 = 330;
	L4 = 107.5;

	joint[1] = joint[1] - 90;
	joint[2] = joint[2] - 90;

	theta34 = joint[2] + joint[3];
	theta345 = joint[2] + joint[3] + joint[4];

	s1 = sin(joint[0] * PM_PI / 180);
	s2 = sin(joint[1] * PM_PI / 180);
	s3 = sin(joint[2] * PM_PI / 180);
	s4 = sin(joint[3] * PM_PI / 180);
	s5 = sin(joint[4] * PM_PI / 180);
	s34 = sin(theta34 * PM_PI / 180);
	s345= sin(theta345 * PM_PI / 180);



	c1 = cos(joint[0] * PM_PI / 180);
	c2 = cos(joint[1] * PM_PI / 180);
	c3 = cos(joint[2] * PM_PI / 180);
	c4 = cos(joint[3] * PM_PI / 180);
	c5 = cos(joint[4] * PM_PI / 180);
	c34 = cos(theta34 * PM_PI / 180);
	c345 = cos(theta345 * PM_PI / 180);


	hom.rot.x.x = c1*c2*c345 + s1*s345;
	hom.rot.x.y = s1*c2*c345 - c1*s345;
	hom.rot.x.z = -1 * s2*c345 ;

	hom.rot.y.x = -1 * c1*c2*s345 + s1*c345;
	hom.rot.y.y = -1 * s1*c2*s345 - c1*c345;
	hom.rot.y.z = s2*s345;

	hom.rot.z.x = -c1*s2;
	hom.rot.z.y = -s1*s2;
	hom.rot.z.z = -c2;

	hom.tran.x = (c1*c2*c34+s1*s34)*L2+(c1*c2*c3+s1*s3)*L3-s1*L4;
	hom.tran.y = (s1*c2*c34-c1*s34)*L2+(s1*c2*c3-c1*s3)*L3+c1*L4;
	hom.tran.z = -s2*c34*L2-s2*c3*L3;
	

	*Ehom = hom.rot;
	world->tran = hom.tran;

	joint[1] = joint[1] + 90;
	joint[2] = joint[2] + 90;

	return 0;
}

int kinematicsInverse_axis5(RobotPose * world, PmHomogeneous* hom,
	double * joint, double *pre_joint,
	int  flag)
{
	double goal[8][5];//顺序存储八组解
	double th1, th2, th3, th4, th5;//存储各关节的弧度值
	double TH1[2], TH2[2], TH4[2]; //关节1,2,4分别有两个值
	double s1, s2, s3, s4, s5, s345;//各关节的正弦值	
	double c1, c2, c3, c4, c5, c345;//各关节的余弦值
	int i, j, k, l, m, n;		//循环迭代
	double A, B, M, N; //计算的中间值

	double L1, L2, L3, L4;
	double c34,s34;
	double a,b,c;
	
	double dsum[8];
	double u1, u2, u;
	double j5;
	L1 = 100;
	L2 = 345;
	L3 = 330;
	L4 = 107.5;
	int optFlag = 0;

	TH1[0] = atan2(hom->rot.z.y, hom->rot.z.x);
	TH1[1] = atan2(-1*(hom->rot.z.y), -1*(hom->rot.z.x));


	for (i = 0; i < 2; i++)
	{
		c1=cos(TH1[i]);
		s1 = sin(TH1[i]);	
		goal[i * 4][0] = TH1[i];
		TH2[0] = atan2((c1*hom->rot.z.x+s1*hom->rot.z.y), hom->rot.z.z);
		TH2[1] = atan2(-(c1*hom->rot.z.x+s1*hom->rot.z.y),-hom->rot.z.z);
		for (j = 0; j < 2; j++)
		{
			c2 = cos(TH2[j]);
			s2 = sin(TH2[j]);
			goal[i * 4 + j * 2][1] = TH2[j];
			A = c1*c2*hom->tran.x + s1*c2*hom->tran.y - s2*hom->tran.z;
			B = -1 * s1*hom->tran.x + c1*hom->tran.y - L4;
			for (k = 0; k < 2; k++)
			{
				c4 = (A*A + B*B - L2*L2 - L3*L3) / (2 * L2*L3);
				s4 = sqrt(1 - c4*c4);
				TH4[0] = atan2(s4, c4);
				TH4[1] = atan2(-s4, c4);
				s4 = sin(TH4[k]);



				M = c4*L2 + L3;
				N = s4*L2;

				//th3 = atan2(-1 * B, A) - atan2(N, M);
				
				if (fabs(A + M - 0) > DBL_MIN)
				{
					u1 = (-N + sqrt(N*N + M*M - A*A)) / (A + M);
					u2 = (-N - sqrt(N*N + M*M - A*A)) / (A + M);
					s3=2*u1/(1+u1*u1);
					c3=(1-u1*u1)/(1+u1*u1);
					//if (fabs(-M*s3 - N*c3 - B) > DBL_MIN || fabs(M*c3 - N*s3 - A) > DBL_MIN)
					//{
					//	s3=2*u2/(1+u2*u2);
					//	c3=(1-u2*u2)/(1+u2*u2);
					//}
					th3 = atan2(s3, c3);
				}
				else
				{
					th3 = PM_PI;
				}



				//if (fabs(A + M - 0) > DBL_MIN)
				//{
				//	u1 = (-N + sqrt(N*N + M*M - A*A)) / (A + M);
				//	u2 = (-N - sqrt(N*N + M*M - A*A)) / (A + M);
				//	s3=2*u1/(1+u1*u1);
				//	c3=(1-u1*u1)/(1+u1*u1);
				//	if (fabs(-M*s3 - N*c3 - B) > DBL_MIN || fabs(M*c3 - N*s3 - A) > DBL_MIN)
				//	{
				//		s3=2*u2/(1+u2*u2);
				//		c3=(1-u2*u2)/(1+u2*u2);
				//	}
				//	
				//}
				//else
				//{
				//	double s3_1, s3_2, c3_1, c3_2;
				//	u1 = (M + sqrt(M*M + N*N - B*B)) / (N - B);
				//	u2 = (M - sqrt(M*M + N*N - B*B)) / (N - B);
				//	s3_1 = 2 * u1 / (1 + u1*u1);
				//	c3_1 = (1 - u1*u1) / (1 + u1*u1);
				//	
				//	s3_2 = 2 * u2 / (1 + u2*u2);
				//	c3_2 = (1 - u2*u2) / (1 + u2*u2);

				//	if (fabs(s3_1*s3_1 + c3_1*c3_1 - 1) > fabs(s3_2*s3_2 + c3_2*c3_2 - 1))
				//	{
				//		s3 = s3_2;
				//		c3 = c3_2;
				//	}
				//	else
				//	{
				//		s3 = s3_1;
				//		c3 = c3_1;
				//	}
				//}

				
				


		/*		s3 = (-A*N - B*M) / (M*M + N*N);
				c3 = (A*M - B*N) / (M*M + N*N);

				double a=M*c3-N*s3;
				double b=-M*s3-N*c3;*/
				

				//c34=c3*c4-s3*s4;
				//s34=s3*c4+c3*s4;
				//a=c34;
				//b=-1*s34;
				//c=c1*c2*hom->rot.x.x + s1*c2*hom->rot.x.y - s2*hom->rot.x.z;
				//u1=
				//th3 = atan2(s3, c3);

				//th3 -= PM_PI / 2;
				//th5 = atan2(s1*hom->rot.x.x - c1*hom->rot.x.y, s1*hom->rot.y.x-c1*hom->rot.y.y)-th3-TH4[k];
				
				th5 = atan2(-1*(c1*c2*hom->rot.y.x + s1*c2*hom->rot.y.y - s2*hom->rot.y.z),c1*c2*hom->rot.x.x+s1*c2*hom->rot.x.y-s2*hom->rot.x.z) - th3 - TH4[k];

				if (fabs(th5*180.0 / PM_PI - pre_joint[4]) > fabs(th5*180.0 / PM_PI - pre_joint[4] + 360.0))
				{
					th5 += PM_2_PI;
				}

				goal[i * 4 + j * 2 + k][0] = TH1[i];
				goal[i * 4 + j * 2 + k][1] = TH2[j];
				goal[i * 4 + j * 2 + k][2] = th3;
				goal[i * 4 + j * 2 + k][3] = TH4[k];
				goal[i * 4 + j * 2 + k][4] = th5;
			}
		}
	}
	//for (i = 0; i < 8; i++)
	//{
	//	for (j = 0; j < 5; j++)
	//	{
	//		goal[i][j] = (goal[i][j] * 180 / PM_PI);
	//		if(1==j)
	//		{
	//			goal[i][1]=goal[i][1]-90;
	//			
	//		}
	//		if(2==j)
	//		{
	//			goal[i][2]=goal[i][2]-90;
	//		}

	//		joint[j]=goal[flag][j];
	//	}
	//
	//}
	//
	for (i = 0; i < 8; i++)
	{
		dsum[i] = 0.0;
		for (j = 0; j < 5; j++)
		{
			goal[i][j] = (goal[i][j] * 180 / PM_PI);
			if (1 == j)
			{
				goal[i][1] = goal[i][1] - 90;

			}
			if (2 == j)
			{
				goal[i][2] = goal[i][2] - 90;
			}
			//joint[j]=goal[0][j];
			dsum[i] += (goal[i][j] - pre_joint[j])*(goal[i][j] - pre_joint[j]);
		}
		if (dsum[i] - dsum[optFlag]<0)
		{
			optFlag = i;
		}
	}
	for (i = 0; i<5; i++)
	{
		joint[i] = goal[optFlag][i];
	}
	//joint[4] += 180.0;



	return 0;
}


int kinematicsForward( double * joint,
                      RobotPose* world,
                       PmRotationMatrix* Ehom,
                       KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
   
		double s1, s2, s3, s4, s5, s6;
		double c1, c2, c3, c4, c5, c6;
		double f[16],h[16],g[16];
        PmHomogeneous hom;
		double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;
	   joint[1] = joint[1] - 90;
	   joint[2] = joint[2] - 90;
	   s1 = sin(joint[0] * PM_PI / 180);
	   s2 = sin(joint[1] * PM_PI / 180);
	   s3 = sin(joint[2] * PM_PI / 180);
	   s4 = sin(joint[3] * PM_PI / 180);
	   s5 = sin(joint[4] * PM_PI / 180);
	   s6 = sin(joint[5] * PM_PI / 180);

	   c1 = cos(joint[0] * PM_PI / 180);
	   c2 = cos(joint[1] * PM_PI / 180);
	   c3 = cos(joint[2] * PM_PI / 180);
	   c4 = cos(joint[3] * PM_PI / 180);
	   c5 = cos(joint[4] * PM_PI / 180);
	   c6 = cos(joint[5] * PM_PI / 180);
	   
	   nx = c6*(c5*(c4*(s1*s3 + c1*c2*c3) + s4*(c3*s1 - c1*c2*s3)) + s5*(c4*(c3*s1 - c1*c2*s3) - s4*(s1*s3 + c1*c2*c3))) + c1*s2*s6;
	   ny = s1*s2*s6 - c6*(c5*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3)) + s5*(c4*(c1*c3 + c2*s1*s3) - s4*(c1*s3 - c2*c3*s1)));
	   nz = c2*s6 - c6*(c5*(c3*c4*s2 - s2*s3*s4) - s5*(c3*s2*s4 + c4*s2*s3));


	   ox = c1*c6*s2 - s6*(c5*(c4*(s1*s3 + c1*c2*c3) + s4*(c3*s1 - c1*c2*s3)) + s5*(c4*(c3*s1 - c1*c2*s3) - s4*(s1*s3 + c1*c2*c3)));
	   oy = s6*(c5*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3)) + s5*(c4*(c1*c3 + c2*s1*s3) - s4*(c1*s3 - c2*c3*s1))) + c6*s1*s2;
	   oz = c2*c6 + s6*(c5*(c3*c4*s2 - s2*s3*s4) - s5*(c3*s2*s4 + c4*s2*s3));

	   ax = c5*(c4*(c3*s1 - c1*c2*s3) - s4*(s1*s3 + c1*c2*c3)) - s5*(c4*(s1*s3 + c1*c2*c3) + s4*(c3*s1 - c1*c2*s3));
	   ay = s5*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3)) - c5*(c4*(c1*c3 + c2*s1*s3) - s4*(c1*s3 - c2*c3*s1));
	   az = c5*(c3*s2*s4 + c4*s2*s3) + s5*(c3*c4*s2 - s2*s3*s4);


	   px = L4*(s1*s3 + c1*c2*c3) - L5*s1 + L3*(c4*(s1*s3 + c1*c2*c3) + s4*(c3*s1 - c1*c2*s3));
	   py = L5*c1 - L4*(c1*s3 - c2*c3*s1) - L3*(c4*(c1*s3 - c2*c3*s1) + s4*(c1*c3 + c2*s1*s3));
	   pz = -L3*(c3*c4*s2 - s2*s3*s4) - L4*c3*s2;

	   world->tran.x = px;
	   world->tran.y = py;
	   world->tran.z = pz;

	   Ehom->x.x = nx;
	   Ehom->x.y = ny;
	   Ehom->x.z = nz;

	   Ehom->y.x = ox;
	   Ehom->y.y = oy;
	   Ehom->y.z = oz;

	   Ehom->z.x = ax;
	   Ehom->z.y = ay;
	   Ehom->z.z = az;


	   joint[1] = joint[1] + 90;
	   joint[2] = joint[2] + 90;

	return 0;

}

int kinematicsInverse(RobotPose * world, PmHomogeneous* hom,
	double * joint, double *pre_joint,
	int  flag)
{
	int i, r, n;
	int k, j, m;//循环变量	
	//double goal[8][6];//顺序存储八组解
	//double testInverse[8][6];

	double dsum[8] = { 0 }; //计算8组解的同前一组角度的误差
	int MN;
	//	int err[8];	
	unsigned char errFlag;
	int skipFlag[8];
	int Flag = 0;
	double goal[32][6];
	double th1, th2, th3, th4, th5, th6, th345;//存储各关节的弧度值
	double TH1[2], TH2[2], TH3[2], TH4[2], TH5[2], TH6[2]; //th1 th2 th5 th6各有两种解法	
	double s1, s2, s3, s4, s5, s6;//各关节的正弦值	
	double c1, c2, c3, c4, c5, c6;//各关节的余弦值
	double s345, c345;
	double K, L;
	double A, B, C;				//计算2关节角度值
	double A3, B3, C3;			//计算关节3角度值
	double px, py, pz, nx, ny, nz, ox, oy, oz, ax, ay, az;


	errFlag = 0;   //错误提示？


	px = hom->tran.x;
	py = hom->tran.y;
	pz = hom->tran.z;

	nx = hom->rot.x.x;
	ny = hom->rot.x.y;
	nz = hom->rot.x.z;

	ox = hom->rot.y.x;
	oy = hom->rot.y.y;
	oz = hom->rot.y.z;

	ax = hom->rot.z.x;
	ay = hom->rot.z.y;
	az = hom->rot.z.z;


	//求解th1,th2
	
	
	TH1[1] = atan2(az*px- pz*ax, pz*ay - az*py);
	TH1[0] = atan2(-1 * az*px+ pz*ax, -1 * pz*ay + az*py);
	 
	for (int i = 0; i < 2; i++)
	{
		th1 = TH1[i];
		s1 = sin(th1);
		c1 = cos(th1);
		if ((fabs(az) < DBL_MIN) && (fabs(ax*c1 + ay*s1) < DBL_MIN))
		{
			if ((fabs(pz) < DBL_MIN) && (fabs(px*c1 + py*s1) < DBL_MIN))
			{
				return -1;
			}
			else
			{
				TH2[0] = atan2(pz, -1 * (px*c1 + py*s1));
				TH2[1] = atan2(-1 * pz, (px*c1 + py*s1));
			}
		}
		else
		{
			TH2[0] = atan2(az, -1 * (ax*c1 + ay*s1));
			TH2[1] = atan2(-1 * az, (ax*c1 + ay*s1));
		}
		TH6[0] = atan2(ox*s1 - oy*c1, ny*c1 - nx*s1);
		TH6[1] = atan2(-1 * ox*s1 + oy*c1, -1 * ny*c1 + nx*s1);
		
		for (int j = 0; j < 2; j++)
		{
			th2 = TH2[j];
			s2 = sin(th2);
			c2 = cos(th2);
			c4 = ((px*c1*c2 - pz*s2 + py*c2*s1)*(px*c1*c2 - pz*s2 + py*c2*s1) + (py*c1 - L5 - px*s1)*(py*c1 - L5 - px*s1) - L3*L3 - L4*L4) / (2 * L3*L4);
			if (c4 > 1)
			{
				TH4[0] = 1.7976931348623158e+308;
				TH4[1] = 1.7976931348623158e+308;
			}
			else
			{
				s4 = sqrt(1 - c4*c4);
				TH4[0] = atan2(s4, c4);
				TH4[1] = atan2(-1 * s4, c4);
			}
			for (int k = 0; k < 2; k++)
			{
				th4 = TH4[k];
				s4 = sin(th4);
				c4 = cos(th4);
				A3 = L3*c4 + L4;
				B3 = L3*s4;
				C3 = px*c1*c2 - pz*s2 + py*c2*s1;
				TH3[0] = atan2(A3, B3) - atan2(C3, sqrt(A3*A3 + B3*B3 - C3*C3));
				TH3[1] = atan2(A3, B3) - atan2(C3, -1 * sqrt(A3*A3 + B3*B3 - C3*C3));
				for (int r = 0; r < 2; r++)
				{
					th6 = TH6[r];
					s6 = sin(th6);
					c6 = cos(th6);
					s345 = ax*c1*c2 - az*s2 + ay*c2*s1;
					c345 = c6*(c1*c2*nx - nz*s2 + c2*ny*s1) - s6*(c1*c2*ox - oz*s2 + c2*oy*s1);
					
					for (int p = 0; p < 2; p++)
					{
						th3 = TH3[p];
						th345 = atan2(-1*s345, c345);
						th5 = th345 - th3 - th4;
						goal[i * 16 + j * 8 + k * 4 + r * 2 + p][0] = TH1[i] * 180 / PM_PI;
						goal[i * 16 + j * 8 + k * 4 + r * 2 + p][1] = TH2[j] * 180 / PM_PI - 90;
						goal[i * 16 + j * 8 + k * 4 + r * 2 + p][2] = TH3[p] * 180 / PM_PI - 90;
						goal[i * 16 + j * 8 + k * 4 + r * 2 + p][3] = TH4[k] * 180 / PM_PI;
						goal[i * 16 + j * 8 + k * 4 + r * 2 + p][4] = th5 * 180 / PM_PI;
						goal[i * 16 + j * 8 + k * 4 + r * 2 + p][5] = TH6[r] * 180 / PM_PI;
						for (int m = 0; m < 6; m++)
						{
							if (goal[i * 16 + j * 8 + k * 4 + r * 2 + p][m] < -180)
							{
								goal[i * 16 + j * 8 + k * 4 + r * 2 + p][m] += 360.0;
							}
						}
					}
					
				}
			}
		}

	}
	


	return 0;

}







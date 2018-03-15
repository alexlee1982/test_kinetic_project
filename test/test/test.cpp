// test.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include "kinematics.h"

KINEMATICS_FORWARD_FLAGS fflags=1;
KINEMATICS_INVERSE_FLAGS iflags=1;


int _tmain(int argc, _TCHAR* argv[])
{
	double joint[6];
	double pre_joint[6];
	int i = 0;
	double inv_joint[6];
	RobotPose world;
	world.tran.x=0;
	world.tran.y=0;
	world.tran.z=0;
	PmRotationMatrix Ehom;
	PmHomogeneous hom;

	PmCartesian tran;


	//Ehom.x.x=0;
	//Ehom.x.y=1;
	//Ehom.x.z=0;

	//Ehom.y.x=0;
	//Ehom.y.y=0;
	//Ehom.y.z=1;

	//Ehom.z.x=1;
	//Ehom.z.y=0;
	//Ehom.z.z=0;

	//world.tran.x = 0;
	//world.tran.y = 782.4;
	//world.tran.z = 0;

	


	joint[0] = 89 ;
	joint[1] = 89 ;
	joint[2] = 0;
	joint[3] = 0;
	joint[4] = 89;
	joint[5] = 89;
	for (i = 0; i < 6; i++)
	{
		pre_joint[i] = joint[i];
	}

	kinematicsForward(joint, &world, &Ehom, &fflags, &iflags);

	hom.tran = world.tran;
	hom.rot = Ehom;
	kinematicsInverse(&world,&hom, inv_joint,pre_joint,1);

	//kinematicsForward_axis5(joint,&world,&Ehom,&fflags,&iflags);
	//
	//hom.tran=world.tran;
	//hom.rot=Ehom;
	//for (int i = 0; i < 8; )
	//{
	//	kinematicsInverse_axis5(&world,&hom,joint, pre_joint,i);

	//	kinematicsForward_axis5(joint,&world,&Ehom,&fflags,&iflags);
	//	i++;
	//}

	return 0;
}


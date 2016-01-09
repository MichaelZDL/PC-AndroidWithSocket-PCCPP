#include "stdafx.h"
#include "Laser_Obstacle_Avoidance.h"
#include <math.h>
#include "MotionOutput.h"
#include "LMS_Control_Michael.h"
#include <stdio.h>
extern CMotionOutput m_MotionOutput;


void LaserObAvoid::MoveAndAovidObs(int bufer[],double RT)
{
	SearchOKRange(bufer,RT);
	if(abs(BestAngle-30)>5)
	{
		TurnForAvoid((30-BestAngle)*3,20);
	}
	else m_MotionOutput.Drive(0.15,0,0);
}


void LaserObAvoid::SearchOKRange(int bufer[],double RT)
{
	int RangNum;
	int OKSmallRange[60];//测试用={1,2,3,4,5,6,7,9,11,12,13,20,21,'\0'};
	int i=0,j=0,m_txt=0;
	int ChordLenCdt;
	/*FILE * fp=fopen("C:\\Michael.txt","w");
	for (m_txt=0;m_txt<181;m_txt++)
	{
		fprint(fp,"%d",bufer[m_txt]);
	}*/
	//step1:找出满足阈值条件的小区间
	for (RangNum=0;RangNum<60;RangNum++)
	{
		if((bufer[RangNum*3]>RT)&&
			(bufer[RangNum*3+1]>RT)&&
			(bufer[RangNum*3+2]>RT))
		{
			OKSmallRange[i]=RangNum;
			i++;
		}
	}
	OKSmallRange[i]=0xffff;

	//step2:把相邻的小区间合成大区间
	for (i=0,j=0;OKSmallRange[i]!=0xffff;j++)
	{
		BigRange[j][0]=OKSmallRange[i];
		for (;OKSmallRange[i]!=0xffff;i++)
		{
			if(OKSmallRange[i+1]!=(OKSmallRange[i]+1))
			{
				BigRange[j][1]=OKSmallRange[i];
				i++;
				break;
			}
		}
	}
	BigRangeNum=j;//保存大区间的个数

	//step3:弦长条件LT=X*0.5=0.4m=400mm
	ChordLenCdt=2*((asin(200/RT))*(180/M_PI));//300/RT=sin(ChordLenCdt/2)
	for (i=0,j=0;i<BigRangeNum;i++)
	{
		if(((BigRange[i][1]-BigRange[i][0]+1)*3)>ChordLenCdt)
		{
			OKBigRange[j]=i;
			j++;
		}
	}
	OKBigRangeNum=j;//可用的大区间的个数
	OKBigRange[j]=0xffff;

	//step4:矩形条件

	//选择最优大区间
	if (OKBigRangeNum>1)
	{
		int EndChs=0;
		int Wch_DistToAng90[1][2]={0,0};
		int Maxbuf[30];
		int MaxbufNum;

		for (i=0;i<OKBigRangeNum;i++)
		{
			Maxbuf[i]=bufer[BigRange[OKBigRange[i]][0]*3];
			for(int j=BigRange[OKBigRange[i]][0]*3;
				j<=BigRange[OKBigRange[i]][1]*3+2;j++)
			{
				if (bufer[j]>Maxbuf[i])
				{
					Maxbuf[i]=bufer[j];
				}
			}
		}

		MaxbufNum=0;
		for (i=0;i<OKBigRangeNum;i++)
		{
			if (Maxbuf[i]>Maxbuf[MaxbufNum])
			{
				MaxbufNum=i;
			}
		}
		BestBigRange[0][0]=BigRange[OKBigRange[MaxbufNum]][0];
		BestBigRange[0][1]=BigRange[OKBigRange[MaxbufNum]][1];
		BestAngle=(BestBigRange[0][0]+BestBigRange[0][1])/2;

			//if(BigRange[OKBigRange[i]][1]-
			//	BigRange[OKBigRange[i]][0]>29)
			//{
			//	BestBigRange[0][0]=BigRange[OKBigRange[i]][0];
			//	BestBigRange[0][1]=BigRange[OKBigRange[i]][1];
			//	EndChs=1;
			//}
	

		///*if (EndChs!=1)
		//{
		//	Wch_DistToAng90[0][1]=abs(((BigRange[OKBigRange[0]][1]+
		//		BigRange[OKBigRange[0]][0])
		//		/2)-30);
		//	Wch_DistToAng90[0][0]=OKBigRange[0];
		//	for (i=1;OKBigRange[i]!=0xffff;i++)
		//	{
		//		if(abs(((BigRange[OKBigRange[i]][1]+
		//			BigRange[OKBigRange[i]][0])
		//			/2)-30)<Wch_DistToAng90[0][1])
		//		{
		//			Wch_DistToAng90[0][1]=abs(((BigRange[OKBigRange[i]][1]+
		//				BigRange[OKBigRange[i]][0])
		//				/2)-30);
		//			Wch_DistToAng90[0][0]=OKBigRange[i];
		//		}
		//	}
		//}
		//BestBigRange[0][0]=BigRange[Wch_DistToAng90[0][0]][0];
		//BestBigRange[0][1]=BigRange[Wch_DistToAng90[0][0]][1];*/
	}
	else if(OKBigRangeNum==1)
	{
		BestAngle=(BigRange[OKBigRange[0]][1]+
					BigRange[OKBigRange[0]][0])/2;
	}
	else
	{
		AfxMessageBox(_T("找不到"));
	}

	
}

UINT ThreadDushuju(LPVOID lpParam);
int kaishidu;
void LaserObAvoid::TurnForAvoid(float Angle,float Vel)
{
	Vel=abs(Angle)/Angle*abs(Vel);
	m_MotionOutput.Drive(0,0,-Vel);

	HANDLE DushuThread;
	DWORD ThreadID;
	kaishidu=1;
	DushuThread = CreateThread(NULL,
		0,
		(LPTHREAD_START_ROUTINE)ThreadDushuju,
		this,
		0,
		&ThreadID);

	Sleep((abs(Angle/Vel))*1000);
	kaishidu=0;
	m_MotionOutput.Drive(0,0,0);

	WaitForSingleObject(DushuThread,INFINITE);


}
extern LMSControl Laser;
extern int bufer[MAXPACKET];
extern int Length;

UINT ThreadDushuju(LPVOID lpParam)
{	
	//threadInfo* pInfo = (threadInfo*)lpParam;
	while(kaishidu)
	{
		Length=Laser.ReadLMSData(bufer,POLARZuoBiao);
	}
	
	return 0;
}
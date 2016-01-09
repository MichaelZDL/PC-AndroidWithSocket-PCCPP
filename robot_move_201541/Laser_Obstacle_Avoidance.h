#pragma once
#define M_PI       3.14159265358979323846

class LaserObAvoid  
{
public:
	//data
	int BigRangeNum;
	int OKBigRangeNum;
	int BigRange[30][2];//������={{0,2},{3,10},{20,40},{45,70}};//��С������ʼ��ţ�С������ֹ���)
	int OKBigRange[30];
	int BestBigRange[1][2];
	int BestAngle;
public:
	//functions
	void SearchOKRange(int*,double);
	void MoveAndAovidObs(int*,double);
	void TurnForAvoid(float,float);
};

#pragma once
#define M_PI       3.14159265358979323846

class LaserObAvoid  
{
public:
	//data
	int BigRangeNum;
	int OKBigRangeNum;
	int BigRange[30][2];//测试用={{0,2},{3,10},{20,40},{45,70}};//（小区间起始序号，小区间终止序号)
	int OKBigRange[30];
	int BestBigRange[1][2];
	int BestAngle;
public:
	//functions
	void SearchOKRange(int*,double);
	void MoveAndAovidObs(int*,double);
	void TurnForAvoid(float,float);
};

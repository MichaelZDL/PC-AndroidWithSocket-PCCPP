#pragma once
#include <afxmt.h>

#include "IMTCompass_LP3300.h"
#pragma comment(lib,"CompassLib.lib")
#include "Motion.h"
using namespace IMTMotionControlCard;

void SerialCallback(void* pParam, char* buff,int nLen,COMPASS_RECEINFO _CompassReceInfo);
void _stdcall ReceFun(void* pParam,int type,float* value,int len);

/*!
	机器人信息结构体\n
	机器人坐标系X轴正方向指向机器人的正前方，Y轴正方向指向机器人的左侧，Z轴正方向垂直于机器人向上
*/
struct MOTIONINFO
{
	float fXVelocity;///<x轴速度
	float fYVelocity;///<y轴速度
	float fZVelocity;///<z轴速度
	float fXDistance;///<相对于机器人本体的X轴方向的绝对位移
	float fYDistance;///<相对于机器人本体的Y轴方向的绝对位移
	float fZDistance;///<相对于机器人本体的Z轴方向的绝对位移
	float fXSpan;///<X轴相对位移
	float fYSpan;///<Y轴相对位移
	float fZSpan;///<Z轴相对位移
	bool bReceInfo;///<是否有数据接收
	float fX;//世界坐标系下机器人X轴方向的位移
	float fY;//世界坐标系下机器人Y轴方向的位移
	float fZ;//世界坐标系下机器人Z轴方向的位移
	float fLDis;
	float fRDis;
	MOTIONINFO()
	{
		fXVelocity = 0.0;
		fYVelocity = 0.0;
		fZVelocity = 0.0;
		fXDistance = 0.0;
		fYDistance = 0.0;
		fZDistance = 0.0;
		fXSpan = 0.0;
		fYSpan = 0.0;
		fZSpan = 0.0;
		bReceInfo = false;
		fX = 0.0;
		fY = 0.0;
		fZ = 0.0;
		fLDis = 0.0;
		fRDis = 0.0;
	}
};

/*!
	MTR的PID信息结构体，只有MTR机器人才有在线调试PID及LoadPID的功能
*/
struct PIDINFO//
{
	float flKp;;///<左轮比例值
	float flKi;;///<左轮积分值
	float flKd;;///<左轮微分值
	float frKp;;///<右轮比例值
	float frKi;;///<右轮积分值
	float frKd;;///<右轮微分值
	bool bReceInfo;
	PIDINFO()
	{
		flKp = 0.0;
		flKi = 0.0;
		flKd = 0.0;
		frKp = 0.0;
		frKi = 0.0;
		frKd = 0.0;
		bReceInfo = false;
	}
};

namespace CRESULAT
{
	enum CHECKRESULT
	{
		WAITING = 0,
		CHECKING = 1,
		ABNORMAL = 2,
		NORMAL = 4
	};
}

/*!
	MTR机器人自检信息结构体，只有MTR机器人有自检功能
*/
struct SELFCHECKINFO
{
	CRESULAT::CHECKRESULT m_LeftMotorRes;///<左电机检测结果
	CRESULAT::CHECKRESULT m_RightMotorRes;///<右电机检测结果
	CRESULAT::CHECKRESULT m_SonarRes[16];///<超声波检测结果
	CRESULAT::CHECKRESULT m_PSDRes[16];///<PSD检测结果
};

/*!
	MTFR机器人信息结构体，适合MTFR履带机器人 
*/
struct MTFROTHERINFO
{
	float m_fArmAngle;///<摆臂角度
	float m_fLeftCurrent;///<左轮电流
	float m_fRightCurrent;///<右轮电流
	float m_fArmCurrent;///<摆臂电流
	int   m_LeftStatus;///<左轮状态
	int   m_RightStatus;///<右轮状态
	int   m_ArmStatus;///<摆臂状态
	int   m_SystemStatus;///<系统状态
	bool bReceInfo;
	MTFROTHERINFO()
	{
		m_fArmAngle = 0.0;
		m_fLeftCurrent = 0.0;
		m_fRightCurrent = 0.0;
		m_fArmCurrent = 0.0;
		m_LeftStatus = 0;
		m_RightStatus = 0;
		m_ArmStatus = 0;
		m_SystemStatus = 0;
		bReceInfo = false;
	}
};

/*!
机器人接收信息结构体
*/
struct MOTIONRECEINFO
{
	MOTIONINFO _MotionInfo;
	PIDINFO _PidInfo;
	float m_fSonarDis[16];
	float m_fPSDDis[16];
	int m_iDI[16];
	SELFCHECKINFO _SelfCheckInfo;
	MTFROTHERINFO _MTFROtherInfo;
	MOTIONRECEINFO()
	{
		for(int i=0;i<16;i++)
		{
			m_fSonarDis[i] = 0.0;
			m_fPSDDis[i] = 0.0;
			m_iDI[i] = 0;
			_SelfCheckInfo.m_SonarRes[i] = CRESULAT::CHECKRESULT::WAITING;
			_SelfCheckInfo.m_PSDRes[i] = CRESULAT::CHECKRESULT::WAITING;
		}
		_SelfCheckInfo.m_LeftMotorRes = CRESULAT::CHECKRESULT::WAITING;
		_SelfCheckInfo.m_RightMotorRes = CRESULAT::CHECKRESULT::WAITING;
	}
};

class CMotionOutput
{
public:
	CMotionOutput(void);
	~CMotionOutput(void);
public:
	IMTMotionControlCard::CMotion::DeviceProc m_DeviceProc;
	
	CMutex m_Mutex;///<互斥体
	MOTIONRECEINFO m_ReceInfo;///<机器人接收信息结构体对象
	bool bRece;///<是否有数据接收
	int m_MotionPort;///<本体的串口号
	int m_SonarPort;///<超声串口号
	int m_PSDPort;	///<PSD串口号
	int m_DIPort;	///<DI串口号
	int m_DOPort;	///<DO串口号
	int m_DAPort;	///<DA串口号
	bool bClear;	///<是否被清零
	bool bAdmin;	///<是否是管理员
	ROBOTINFO m_RobotInfo;	///<ROBOTINFO结构体对象。机器人信息结构体。

	void Init();
	bool InputUserPassword(IMTMotionControlCard::GROUP _group,char* _userName,char* _password); 
	int connect_compass(int port, long baudrate);
	int connect_Motionserial(int port, long baudrate);//连接运动设备
	int connect_SonarSerial(int port,long baudrate);//连接超声设备
	int connect_PSDSerial(int port,long baudrate);//连接PSD设备
	int connect_DISerial(int port,long baudrate);//连接DI数字量输入设备
	int connect_DOSerial(int port,long baudrate);//连接DO数字量输出设备
	int connect_DASerial(int port,long baudrate);//连接DA数字量转模拟量设备
	void disconnect_serial();//关闭所有串口
	void Drive(float Vel_x,float Vel_y,float Vel_z);//控制运动设备，控制模式为速度闭环
	void OpenSonarChannel(int iChannel,bool bEnable);//打开超声设备
    void SetModeSelect(int modesel,float xDis,float yDis,float zDis);//设置模式选择
	void setVel(double velocity);//设置运行速度
	void setVel2(double leftVelocity,double rightVelocity);//设置左右轮速度
	void setRotVel(double velocity);//设置旋转速度
	void move(double distance,float basicVel);//移动的位移
	void setRotAngle(double angle,float basicVel,float &t);//设置旋转角度
	void setRotRadius(double radius,float basicVel);//设置旋转半径
	void stop();//机器人停止运动

	void SetAccelerate(float xAcc,float yAcc,float zAcc);//设置加速度值
	void SetPidSelect(int pidsel,float Kp[2],float Ki[2],float Kd[2]);//设置PID选择
	void StartCheckSelf(bool bEnable);//启动自检，检测电机，超声，PSD
	void SavePID(float Kp[2],float Ki[2],float Kd[2]);//保存PID
	void LookOverPIDParam();//查看PID参数
	void EnableEncoderDev(MTRParamInfo::ENCODERTYPE Type,bool bEnable);//启动码盘设备

	const char* GetRobotType();
	void OpenPSDChannel(int iChannel,bool bEnable);//打开PSD设备
	void OpenDIChannel(int iChannel,bool bEnable);//启动DI设备
	void OpenDOChannel(int iChannel,bool bEnable);//启动DO设备
	void OpenDAChannel(int iChannel,int value,bool bEnable);//启动DA设备
	bool LoadXMLFile();
	bool SaveXMLFile();

	void SetControlModeSel(MTORIParamInfo::CONTROL controlmode);
	void SetDribblingSpeed(float m_lSpeed,float m_rSpeed);//设置盘球电机速度
	void SetKickStrength(int m_Strength);//设置踢球的力量

	void Check();
	void GetComInfo(bool &bMotionserial,bool &bSonarserial,bool &PSDserial,bool &bDIserial,bool &bDOserial,bool &bDAserial);

	void ArmCalibrate();
	void ControlArm(float m_fSpeed,float m_fAngle);

	void Clear();

public:
	char* m_receBuff;///<接收数据缓存 
	int nReceBuffLen;///<接收数据缓存大小
	COMPASS_RECEINFO m_CompassReceInfo;

	IMTCompass_LP3300 m_Compass;
	bool bReceCompass;
	CString strCompass;
	void disconnect_compass();

};

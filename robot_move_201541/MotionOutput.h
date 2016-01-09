#pragma once
#include <afxmt.h>

#include "IMTCompass_LP3300.h"
#pragma comment(lib,"CompassLib.lib")
#include "Motion.h"
using namespace IMTMotionControlCard;

void SerialCallback(void* pParam, char* buff,int nLen,COMPASS_RECEINFO _CompassReceInfo);
void _stdcall ReceFun(void* pParam,int type,float* value,int len);

/*!
	��������Ϣ�ṹ��\n
	����������ϵX��������ָ������˵���ǰ����Y��������ָ������˵���࣬Z��������ֱ�ڻ���������
*/
struct MOTIONINFO
{
	float fXVelocity;///<x���ٶ�
	float fYVelocity;///<y���ٶ�
	float fZVelocity;///<z���ٶ�
	float fXDistance;///<����ڻ����˱����X�᷽��ľ���λ��
	float fYDistance;///<����ڻ����˱����Y�᷽��ľ���λ��
	float fZDistance;///<����ڻ����˱����Z�᷽��ľ���λ��
	float fXSpan;///<X�����λ��
	float fYSpan;///<Y�����λ��
	float fZSpan;///<Z�����λ��
	bool bReceInfo;///<�Ƿ������ݽ���
	float fX;//��������ϵ�»�����X�᷽���λ��
	float fY;//��������ϵ�»�����Y�᷽���λ��
	float fZ;//��������ϵ�»�����Z�᷽���λ��
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
	MTR��PID��Ϣ�ṹ�壬ֻ��MTR�����˲������ߵ���PID��LoadPID�Ĺ���
*/
struct PIDINFO//
{
	float flKp;;///<���ֱ���ֵ
	float flKi;;///<���ֻ���ֵ
	float flKd;;///<����΢��ֵ
	float frKp;;///<���ֱ���ֵ
	float frKi;;///<���ֻ���ֵ
	float frKd;;///<����΢��ֵ
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
	MTR�������Լ���Ϣ�ṹ�壬ֻ��MTR���������Լ칦��
*/
struct SELFCHECKINFO
{
	CRESULAT::CHECKRESULT m_LeftMotorRes;///<���������
	CRESULAT::CHECKRESULT m_RightMotorRes;///<�ҵ�������
	CRESULAT::CHECKRESULT m_SonarRes[16];///<�����������
	CRESULAT::CHECKRESULT m_PSDRes[16];///<PSD�����
};

/*!
	MTFR��������Ϣ�ṹ�壬�ʺ�MTFR�Ĵ������� 
*/
struct MTFROTHERINFO
{
	float m_fArmAngle;///<�ڱ۽Ƕ�
	float m_fLeftCurrent;///<���ֵ���
	float m_fRightCurrent;///<���ֵ���
	float m_fArmCurrent;///<�ڱ۵���
	int   m_LeftStatus;///<����״̬
	int   m_RightStatus;///<����״̬
	int   m_ArmStatus;///<�ڱ�״̬
	int   m_SystemStatus;///<ϵͳ״̬
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
�����˽�����Ϣ�ṹ��
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
	
	CMutex m_Mutex;///<������
	MOTIONRECEINFO m_ReceInfo;///<�����˽�����Ϣ�ṹ�����
	bool bRece;///<�Ƿ������ݽ���
	int m_MotionPort;///<����Ĵ��ں�
	int m_SonarPort;///<�������ں�
	int m_PSDPort;	///<PSD���ں�
	int m_DIPort;	///<DI���ں�
	int m_DOPort;	///<DO���ں�
	int m_DAPort;	///<DA���ں�
	bool bClear;	///<�Ƿ�����
	bool bAdmin;	///<�Ƿ��ǹ���Ա
	ROBOTINFO m_RobotInfo;	///<ROBOTINFO�ṹ����󡣻�������Ϣ�ṹ�塣

	void Init();
	bool InputUserPassword(IMTMotionControlCard::GROUP _group,char* _userName,char* _password); 
	int connect_compass(int port, long baudrate);
	int connect_Motionserial(int port, long baudrate);//�����˶��豸
	int connect_SonarSerial(int port,long baudrate);//���ӳ����豸
	int connect_PSDSerial(int port,long baudrate);//����PSD�豸
	int connect_DISerial(int port,long baudrate);//����DI�����������豸
	int connect_DOSerial(int port,long baudrate);//����DO����������豸
	int connect_DASerial(int port,long baudrate);//����DA������תģ�����豸
	void disconnect_serial();//�ر����д���
	void Drive(float Vel_x,float Vel_y,float Vel_z);//�����˶��豸������ģʽΪ�ٶȱջ�
	void OpenSonarChannel(int iChannel,bool bEnable);//�򿪳����豸
    void SetModeSelect(int modesel,float xDis,float yDis,float zDis);//����ģʽѡ��
	void setVel(double velocity);//���������ٶ�
	void setVel2(double leftVelocity,double rightVelocity);//�����������ٶ�
	void setRotVel(double velocity);//������ת�ٶ�
	void move(double distance,float basicVel);//�ƶ���λ��
	void setRotAngle(double angle,float basicVel,float &t);//������ת�Ƕ�
	void setRotRadius(double radius,float basicVel);//������ת�뾶
	void stop();//������ֹͣ�˶�

	void SetAccelerate(float xAcc,float yAcc,float zAcc);//���ü��ٶ�ֵ
	void SetPidSelect(int pidsel,float Kp[2],float Ki[2],float Kd[2]);//����PIDѡ��
	void StartCheckSelf(bool bEnable);//�����Լ죬�������������PSD
	void SavePID(float Kp[2],float Ki[2],float Kd[2]);//����PID
	void LookOverPIDParam();//�鿴PID����
	void EnableEncoderDev(MTRParamInfo::ENCODERTYPE Type,bool bEnable);//���������豸

	const char* GetRobotType();
	void OpenPSDChannel(int iChannel,bool bEnable);//��PSD�豸
	void OpenDIChannel(int iChannel,bool bEnable);//����DI�豸
	void OpenDOChannel(int iChannel,bool bEnable);//����DO�豸
	void OpenDAChannel(int iChannel,int value,bool bEnable);//����DA�豸
	bool LoadXMLFile();
	bool SaveXMLFile();

	void SetControlModeSel(MTORIParamInfo::CONTROL controlmode);
	void SetDribblingSpeed(float m_lSpeed,float m_rSpeed);//�����������ٶ�
	void SetKickStrength(int m_Strength);//�������������

	void Check();
	void GetComInfo(bool &bMotionserial,bool &bSonarserial,bool &PSDserial,bool &bDIserial,bool &bDOserial,bool &bDAserial);

	void ArmCalibrate();
	void ControlArm(float m_fSpeed,float m_fAngle);

	void Clear();

public:
	char* m_receBuff;///<�������ݻ��� 
	int nReceBuffLen;///<�������ݻ����С
	COMPASS_RECEINFO m_CompassReceInfo;

	IMTCompass_LP3300 m_Compass;
	bool bReceCompass;
	CString strCompass;
	void disconnect_compass();

};

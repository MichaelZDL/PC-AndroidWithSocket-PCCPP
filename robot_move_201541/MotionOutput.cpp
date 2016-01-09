/*!

@file MotionOutput.cpp
@brief CMotionOutput��ʵ���ļ����������˶���������PSD���豸�ӿڡ�
@note
*/

#include "StdAfx.h"
#include ".\MotionOutput.h"
IMTMotionControlCard::CMotion m_Motion;

CMotionOutput::CMotionOutput(void)
{
	bRece = false;
	m_MotionPort = 0;
	m_SonarPort = 0;
	m_PSDPort = 0;
	m_DIPort = 0;
	m_DOPort = 0;
	m_DAPort = 0;
	bClear = false;
	bAdmin = false;
}

CMotionOutput::~CMotionOutput(void)
{
}

//-------------------------------------------------------------------------
/*!��������: Init

@param  ��
@return ��
@note  ��ȡXML�ļ���ø��豸�Ķ˿ںŲ����˶��豸�˿�
*/
void CMotionOutput::Init()
{
	m_Motion.InputUserPassword(USER,"User","User");
	if(m_Motion.LoadXMLFile())
	{
		m_RobotInfo = m_Motion.m_RobotInfo;
		for(int i=0;i<m_Motion.m_RobotInfo.m_Sensor.size();i++)
		{
			if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"�����ּ�����"))
			{
				sscanf(m_Motion.m_RobotInfo.m_Sensor[i].strCom,"COM%d",&m_MotionPort);
			}
			else if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"����"))
			{
				sscanf(m_Motion.m_RobotInfo.m_Sensor[i].strCom,"COM%d",&m_SonarPort);
			}
			else if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"PSD"))//���PSD�豸��Ӧ�Ķ˿ں�
			{
				sscanf(m_Motion.m_RobotInfo.m_Sensor[i].strCom,"COM%d",&m_PSDPort);
			}
			else if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"DI"))
			{
				sscanf(m_Motion.m_RobotInfo.m_Sensor[i].strCom,"COM%d",&m_DIPort);
			}
			else if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"DO"))
			{
				sscanf(m_Motion.m_RobotInfo.m_Sensor[i].strCom,"COM%d",&m_DOPort);
			}
			else if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"DA"))
			{
				sscanf(m_Motion.m_RobotInfo.m_Sensor[i].strCom,"COM%d",&m_DAPort);
			}
		}
		if (m_RobotInfo.iRobotType==MTR)
		{
#define MTRROBOT
		}
		else if (m_RobotInfo.iRobotType==MTFR)
		{
#define MTFRROBOT
		}
		else if (m_RobotInfo.iRobotType==MTOR)
		{
#define MTORROBOT
		}
		else if (m_RobotInfo.iRobotType==MTHOME)
		{
#define MTHOMEROBOT
		}
		if(m_Motion.m_RobotInfo.m_Sensor.size()>0)
		{
			m_Motion.connect_Motionserial(m_MotionPort,19200);
			m_DeviceProc = ReceFun;
			m_Motion.SetReceProc(m_DeviceProc,this);//�����¼��������
		}
	}
}

void SerialCallback(void* pParam, char* buff,int nLen,COMPASS_RECEINFO _CompassReceInfo)
{
	CMotionOutput* pThis = (CMotionOutput*)pParam;
	char s[256];
	pThis->strCompass.Empty();
	pThis->m_receBuff = buff;
	pThis->nReceBuffLen = nLen;
	pThis->m_CompassReceInfo = _CompassReceInfo;
	for(int i = 0;i<pThis->nReceBuffLen;i++)
	{
		if(((pThis->m_receBuff[i]&0x80)>>7)==1)
		{
			sprintf(s,"%X ",(BYTE)pThis->m_receBuff[i]);
		}
		else
		{
			if((pThis->m_receBuff[i]&0x7F)<15)
			{
				sprintf(s,"0%X ",(BYTE)pThis->m_receBuff[i]);
			}
			else
			{
				sprintf(s,"%X ",(BYTE)pThis->m_receBuff[i]);
			}
		}
		pThis->strCompass+=s;
	}
	pThis->bReceCompass = true;
}

//-------------------------------------------------------------------------
/*!��������: ReceFun

@param pParam ָ�����ṹ���ָ��
@param type ����
@param value ����ֵ
@param len ���鳤��
@return ��
@note  �ص����������ڽ��յ����ݽ��н�����ɺ��֪ͨ�ú�����
*/
void _stdcall ReceFun(void* pParam,int type,float* value,int len)
{
	CMotionOutput* pThis = (CMotionOutput*)pParam;
	CSingleLock sLock(&pThis->m_Mutex);
	int m_StateInfo[3][16]={0};//�����ŷ�����ģ��(���֡����֡��ڱۣ���Ӧ��16�ִ���״̬��Ϣ
	float m_value[3]={0};//�����ŷ�����ģ��״̬��Ϣδ������
	int m_iCanStatus=0;//����Can״̬
	int m_iCanStatus2=0;//����Can״̬
	int m_iCanStatus3=0;//�ڱ�Can״̬
	int m_StopButtonStatus=0;//��ͣ״̬
	bool bException= false;//�쳣��־
	static bool bSend = false;//�����쳣ʱ�跢��ָֹͣ��ñ�־�������Ʒ���һ��ָֹͣ�ֱ���쳣״̬��ʧ���ٴγ����쳣ʱʹ��
	sLock.Lock();
	
	switch(type)
	{
	case MOTION:
		pThis->m_ReceInfo._MotionInfo.fXVelocity = value[0];//X���ٶ�
		pThis->m_ReceInfo._MotionInfo.fYVelocity = value[1];//Y���ٶ�
		pThis->m_ReceInfo._MotionInfo.fZVelocity = value[2];//Z���ٶ�
		pThis->m_ReceInfo._MotionInfo.fXDistance = value[3];//X��λ��
		pThis->m_ReceInfo._MotionInfo.fYDistance = value[4];//Y��λ��
		pThis->m_ReceInfo._MotionInfo.fZDistance = value[5];//Z��λ��
		pThis->m_ReceInfo._MotionInfo.fLDis = value[6];//����λ��
		pThis->m_ReceInfo._MotionInfo.fRDis = value[7];//����λ��
		pThis->m_ReceInfo._MotionInfo.bReceInfo = true;//�ṩ���ⲿ�����ʹ�ã������ж��Ƿ���յ���������
		break;
	case SONAR:
		pThis->m_ReceInfo.m_fSonarDis[len] = *value;//����������������ֵ
		break;
	case PID:
		pThis->m_ReceInfo._PidInfo.flKp = value[0];//RAM�ڴ洢�����ֱ���ֵ
		pThis->m_ReceInfo._PidInfo.flKi = value[1];//RAM�ڴ洢�����ֻ���ֵ
		pThis->m_ReceInfo._PidInfo.flKd = value[2];//RAM�ڴ洢������΢��ֵ
		pThis->m_ReceInfo._PidInfo.frKp = value[3];
		pThis->m_ReceInfo._PidInfo.frKi = value[4];
		pThis->m_ReceInfo._PidInfo.frKd = value[5];
		pThis->m_ReceInfo._PidInfo.bReceInfo = true;
		break;
	case PSD:
		if(*value>80)
		{
			*value = 80;
		}
		pThis->m_ReceInfo.m_fPSDDis[len] = *value;//PSD���⴫��������ֵ
		break;
	case DI:
		for(int i=0;i<16;i++)
		{
			pThis->m_ReceInfo.m_iDI[i] = (int)value[i];//���������뷵��ֵ
		}
		break;
	case ENCODER_RELA:
		pThis->m_ReceInfo._MotionInfo.fXSpan = value[0];//X�����λ��
		pThis->m_ReceInfo._MotionInfo.fYSpan = value[1];//Y�����λ��
		pThis->m_ReceInfo._MotionInfo.fZSpan = value[2];//Z�����λ��
		if(pThis->bClear)//�Ƿ�����ı�־
		{
			pThis->bClear = false;
			pThis->m_ReceInfo._MotionInfo.fX = 0.0;//��������ϵ�µ�Xֵ
			pThis->m_ReceInfo._MotionInfo.fY = 0.0;//��������ϵ�µ�Yֵ
			pThis->m_ReceInfo._MotionInfo.fZ = 0.0;//��������ϵ�µ�Zֵ�Ƕ�ֵ
		}
		pThis->m_ReceInfo._MotionInfo.fZ += value[2];
		if(pThis->m_ReceInfo._MotionInfo.fZ>360)
		{
			pThis->m_ReceInfo._MotionInfo.fZ -= 360;
		}
		else if(pThis->m_ReceInfo._MotionInfo.fZ<-360)
		{
			pThis->m_ReceInfo._MotionInfo.fZ += 360;
		}
		pThis->m_ReceInfo._MotionInfo.fX += (value[0] * cos(pThis->m_ReceInfo._MotionInfo.fZ*PI/180) - value[1] * sin(pThis->m_ReceInfo._MotionInfo.fZ*PI/180));//��������������������ת������������ϵ�����������ϵXYZ�᷽����ͬ
		pThis->m_ReceInfo._MotionInfo.fY += (value[0] * sin(pThis->m_ReceInfo._MotionInfo.fZ*PI/180) + value[1] * cos(pThis->m_ReceInfo._MotionInfo.fZ*PI/180));
		pThis->m_ReceInfo._MotionInfo.bReceInfo = true;
		break;
	case CHECK://�������Լ�ʱ�ķ���ֵ���Ĵ�������û�и����
		CRESULAT::CHECKRESULT Res;
		if(value[2]==1)
		{
			Res = CRESULAT::CHECKRESULT::CHECKING;//���ڼ��
		}
		else if(value[2]==2)
		{
			Res = CRESULAT::CHECKRESULT::ABNORMAL;//����쳣
		}
		else if(value[2]==4)
		{
			Res = CRESULAT::CHECKRESULT::NORMAL;//�������
		}
		if(value[0]==1)//���������ֵ�����״̬
		{
			if(value[1]==0)//����״̬
			{
				pThis->m_ReceInfo._SelfCheckInfo.m_LeftMotorRes = Res;
			}
			else//�ҵ��״̬
			{
				pThis->m_ReceInfo._SelfCheckInfo.m_RightMotorRes = Res;
			}
		}
		else if(value[0]==3)//���س��������������״̬
		{
			pThis->m_ReceInfo._SelfCheckInfo.m_SonarRes[(int)value[1]] = Res;
		}
		else if(value[0]==5)//����PSD���⴫�������״̬
		{
			pThis->m_ReceInfo._SelfCheckInfo.m_PSDRes[(int)value[1]] = Res;
		}
		break;
	case MTFRINFO://�����Ĵ������˵�һЩ״̬��Ϣ
		pThis->m_ReceInfo._MTFROtherInfo.m_fArmAngle = value[0];//�ڱ۽Ƕ�
		pThis->m_ReceInfo._MTFROtherInfo.m_fLeftCurrent = value[1];//���ֵ���
		pThis->m_ReceInfo._MTFROtherInfo.m_fRightCurrent = value[2];//���ֵ���
		pThis->m_ReceInfo._MTFROtherInfo.m_fArmCurrent = value[3];//�ڱ۵���
		pThis->m_ReceInfo._MTFROtherInfo.m_LeftStatus = value[4];//����״̬
		pThis->m_ReceInfo._MTFROtherInfo.m_RightStatus = value[5];//����״̬
		pThis->m_ReceInfo._MTFROtherInfo.m_ArmStatus = value[6];//�ڱ�״̬
		pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus = value[7];//ϵͳ״̬
		pThis->m_ReceInfo._MTFROtherInfo.bReceInfo = true;
		
		m_value[0] = (float)pThis->m_ReceInfo._MTFROtherInfo.m_LeftStatus;
		m_value[1] = (float)pThis->m_ReceInfo._MTFROtherInfo.m_RightStatus;
		m_value[2] = (float)pThis->m_ReceInfo._MTFROtherInfo.m_ArmStatus;
		for(int j=0;j<3;j++)
		{
			for(int i=0;i<16;i++)
			{
				m_StateInfo[j][15-i] = (((int)m_value[j])&(1<<i))>>i;//�ֱ�Ϊ���֡����֡��ڱ�״̬��Ϣ,��16��״̬��Ϣ
				//�ֱ��ӦDsp�쳣��λ����ֵ�����������쳣������λ������λ�����¡�ʵ���ٶ��쳣��ϵͳ�ϵ縴λ����̬������
				//���ݱ����쳣�������ɼ��쳣������ͨѶ�쳣��Ƿѹ����쳣�������ٶ��쳣��λ�òɼ��쳣�������λ
				if(m_StateInfo[j][15-i]==1)
				{
					bException = true; 
				}
			}
		}
		
		m_iCanStatus = (((int)pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus)&0x02)>>1;//����can����״̬
	    m_iCanStatus2 = (((int)pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus)&0x04)>>2;//����can����״̬
	    m_iCanStatus3 = (((int)pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus)&0x08)>>3;//�ڱ�can����״̬
	    m_StopButtonStatus = ((pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus)&0x01);//��ӳ��ͣ����״̬
		if(m_iCanStatus==1)
		{
			bException = true;
		}
		if(m_iCanStatus2==1)
		{
			bException = true;
		}
		if(m_iCanStatus3==1)
		{
			bException = true;
		}
		if(m_StopButtonStatus==1)
		{
			bException = true;
		}
		
		if(bException)//һ�������κ�һ���쳣��֪ͨ������ֹͣ����
		{
			if(!bSend)//�����ظ��򴮿ڷ���ָֹͣ��
			{
				pThis->Drive(0,0,0);
				bSend = true;
			}
		}
		else
		{
			bSend = false;
		}
		break;
	}
	sLock.Unlock();
}
 
//-------------------------------------------------------------------------
/*!��������: InputUserPassword

@param _group  ��������ΪADMIN����Ա��USER�û�
@param _userName  �û���
@param _password  ���� 
@return m_Motion.InputUserPassword(_group,_userName,_password)�ķ���ֵ
@note �����û���������
*/
bool CMotionOutput::InputUserPassword(IMTMotionControlCard::GROUP _group,char* _userName,char* _password)
{
	if(_group==ADMIN)
	{
		bAdmin = true;
	}
	else if(_group==USER)
	{
		bAdmin = false;
	}
	return m_Motion.InputUserPassword(_group,_userName,_password);
}

int CMotionOutput::connect_compass(int port, long baudrate)
{
	bool bOpen = m_Compass.Init(port,baudrate);

	if ( bOpen )
	{
		m_Compass.SetSerialCallback(SerialCallback,this);
	}

	return bOpen;
}
//-------------------------------------------------------------------------
/*!��������: connect_Motionserial

@param   port �˿ں�
@param baudrate ������
@return m_Motion.connect_Motionserial(port,baudrate)�ķ���ֵ
@note �����˶��豸
*/
int CMotionOutput::connect_Motionserial(int port, long baudrate)
{
	return m_Motion.connect_Motionserial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!��������: connect_SonarSerial

@param   port �˿ں�
@param baudrate ������
@return m_Motion.connect_SonarSerial(port,baudrate)�ķ���ֵ
@note ���ӳ����豸
*/
int CMotionOutput::connect_SonarSerial(int port,long baudrate)
{
	return m_Motion.connect_SonarSerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!��������: connect_PSDSerial

@param   port �˿ں�
@param baudrate ������
@return m_Motion.connect_PSDSerial(port,baudrate)�ķ���ֵ
@note   ����PSD�豸
*/
int CMotionOutput::connect_PSDSerial(int port,long baudrate)
{
	return m_Motion.connect_PSDSerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!��������: connect_DISerial

@param   port �˿ں�
@param baudrate ������
@return m_Motion.connect_DISerial(port,baudrate)�ķ���ֵ
@note  ����DI�����������豸
*/
int CMotionOutput::connect_DISerial(int port,long baudrate)
{
	return m_Motion.connect_DISerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!��������: connect_DOSerial

@param   port �˿ں�
@param baudrate ������
@return m_Motion.connect_DOSerial(port,baudrate)�ķ���ֵ
@note ����DO����������豸
*/
int CMotionOutput::connect_DOSerial(int port,long baudrate)
{
	return m_Motion.connect_DOSerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!��������: connect_DASerial

@param   port �˿ں�
@param baudrate ������  
@return m_Motion.connect_DASerial(port,baudrate)�ķ���ֵ
@note ����DA������תģ�����豸
*/
int CMotionOutput::connect_DASerial(int port,long baudrate)
{
	return m_Motion.connect_DASerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!��������: disconnect_serial

@param  ��
@return m_Motion.disconnect_serial()�ķ���ֵ
@note �ر����д���
*/
void CMotionOutput::disconnect_serial()
{
	m_Motion.disconnect_serial();
}
 
void CMotionOutput::disconnect_compass()
{
	m_Compass.End();
}
//-------------------------------------------------------------------------
/*!��������: Drive

@param Vel_x X���ٶ�
@param Vel_y Y���ٶ�,�������ֻ�������Y�᷽����û���ٶȣ����趨��ֵΪ0
@param Vel_z Z���ٶ�
@return ��
@note  �����˶��豸������ģʽĬ��Ϊ�ٶȱջ�������Ҫ����λ�ñջ�ʱ���ڸú���ǰ��SetModeSelect�����趨
*/
void CMotionOutput::Drive(float Vel_x,float Vel_y,float Vel_z)
{
	m_Motion.Drive(Vel_x,Vel_y,Vel_z);
}
 
//-------------------------------------------------------------------------
/*!��������: OpenSonarChannel

@param  iChannel ����ͨ���ţ��������Ϲ���16������ͨ������ֵ��ΧΪ��0~15���������˳���ǰĬ�ϻ�������ǰ���ĳ���ͨ����Ϊ0����ʱ����תͨ��������
@param  bEnable  Ϊtrue��ʾ�򿪸ó���ͨ������֮�ر�
@return ��
@note   �򿪳����豸    
*/
void CMotionOutput::OpenSonarChannel(int iChannel,bool bEnable)
{
	m_Motion.OpenSonarChannel(iChannel,bEnable);
}
 
//-------------------------------------------------------------------------
/*!��������: SetModeSelect

@param  modesel ģʽ��������ȡֵ��\n
			MTRParamInfo::POS_CLλ�ñջ�\n
			MTRParamInfo::VEL_CL�ٶȱջ�
@param xDis ���ֻ�����ʱ��ֵ��Ӧ����λ�ƣ����ֻ�����ʱ��Ӧ����X���λ�ƣ�ģʽΪ�ٶȱջ�ʱ��ֵ��Ϊ0
@param yDis ���ֻ�����ʱ��ֵ��Ӧ����λ�ƣ����ֻ�����ʱ��Ӧ����Y���λ�ƣ�ģʽΪ�ٶȱջ�ʱ��ֵ��Ϊ0
@param zDis ���ֻ�����ʱ��ֵΪ0�����ֻ�����ʱ��Ӧ����Z���λ�ƣ�ģʽΪ�ٶȱջ�ʱ��ֵ��Ϊ0
@return ��
@note ����ģʽѡ��  
*/
void CMotionOutput::SetModeSelect(int modesel,float xDis,float yDis,float zDis)//
{
	m_Motion.SetModeSelect(modesel,xDis,yDis,zDis);
}


/********************************************************************
**                                                   **

********************************************************************/
//-------------------------------------------------------------------------
/*!��������: SetAccelerate

@param  xAcc x������ٶ�
@param  yAcc y������ٶ�
@param  zAcc z������ٶ�
@return ��
@note ���ü��ٶ�ֵ  
*/
void CMotionOutput::SetAccelerate(float xAcc,float yAcc,float zAcc)
{
	#ifdef MTRROBOT//��Ӧ������ʽ������
	m_Motion.SetAccelerate(xAcc,yAcc,zAcc);
#endif
}
 
//-------------------------------------------------------------------------
/*!��������: SetPidSelect

@param   pidsel		PIDģʽ��PID_DEFĬ��PID��PID_DEBUG���ߵ���PID��PID_LOADΪʹ��E2PROM�ڱ���õ�PIDֵ
@param   Kp		Kp[0]���ֱ���ֵ��Kp[1]���ֱ���ֵ
@param   Ki		Ki[0]���ֻ���ֵ��Ki[1]���ֻ���ֵ
����4��Kd[0]����΢��ֵ��Kd[1]����΢��ֵ
@return ��
@note ����PIDѡ��          
*/
void CMotionOutput::SetPidSelect(int pidsel,float Kp[2],float Ki[2],float Kd[2])
{
	#ifdef MTRROBOT//��Ӧ������ʽ������
	m_Motion.SetPidSelect(pidsel,Kp,Ki,Kd);
#endif
}

 
//-------------------------------------------------------------------------
/*!��������: StartCheckSelf

@param  bEnable Ϊtrue��ʾ�����Լ죬��ֹ֮ͣ�Լ�
@return ��
@note �����Լ죬�������������PSD 
*/
void CMotionOutput::StartCheckSelf(bool bEnable)
{
	#ifdef MTRROBOT//��Ӧ������ʽ������
	m_Motion.StartCheckSelf(bEnable);
#endif
}
 
//-------------------------------------------------------------------------
/*!��������: SavePID

@param  Kp  Kp[0]���ֱ���ֵ��Kp[1]���ֱ���ֵ
@param  Ki  Ki[0]���ֻ���ֵ��Ki[1]���ֻ���ֵ
@param  Kd  Kd[0]����΢��ֵ��Kd[1]����΢��ֵ
@return ��
@note  ����PID 
*/
void CMotionOutput::SavePID(float Kp[2],float Ki[2],float Kd[2])
{
	#ifdef MTRROBOT//��Ӧ������ʽ������
	m_Motion.SavePID(Kp,Ki,Kd);
#endif
}
 
//-------------------------------------------------------------------------
/*!��������: LookOverPIDParam

@param  ��
@return ��
@note �鿴PID����
*/
void CMotionOutput::LookOverPIDParam()
{
	#ifdef MTRROBOT//��Ӧ������ʽ������
	m_Motion.LookOverPIDParam();
#endif
}
 
//-------------------------------------------------------------------------
/*!��������: EnableEncoderDev

@param  Type �������ͣ�ENCODER_ABSOLUTE���ؾ���ֵ�������ݣ�ENCODER_RELATIVE�������ֵ��������
@param  bEnable Ϊtrue��ʾ�������̷��ع��ܣ�����Ҫ�������ֵ��������ʱ��Ҫ���һ��ʱ�䷢��һ�θú���
@return ��
@note ���������豸
*/
void CMotionOutput::EnableEncoderDev(MTRParamInfo::ENCODERTYPE Type,bool bEnable)
{
	#ifdef MTRROBOT//��Ӧ������ʽ������
	m_Motion.EnableEncoderDev(Type,bEnable);
	#endif
}
 
//-------------------------------------------------------------------------
/*!��������: GetRobotType

@param  ��
@return  ���ػ��������ͣ�����ֵΪMTR˵���������Ӧ������ʽ�����˵������ΪMTFR˵��������ʺ��Ĵ�������ʹ��
@note ��û��������� 
*/
const char* CMotionOutput::GetRobotType()
{
	return m_Motion.GetRobotType();
}
 
//-------------------------------------------------------------------------
/*!��������: OpenPSDChannel

@param  iChannel ͨ���ţ��������Ϲ���16��PSDͨ������ֵ��ΧΪ��0~15���������˳���ǰĬ�ϻ�������ǰ����PSDͨ����Ϊ0����ʱ����תͨ��������
@param  bEnable  Ϊtrue��ʾ��ͨ������֮�ر�ͨ��
@return ��
@note  ��PSD�豸  
*/
void CMotionOutput::OpenPSDChannel(int iChannel,bool bEnable)
{
	m_Motion.OpenPSDChannel(iChannel,bEnable);
}
 
//-------------------------------------------------------------------------
/*!��������: OpenDIChannel

@param iChannel ͨ���ţ��������Ϲ���16��DIͨ������ֵ��ΧΪ��0~15��
@param bEnable Ϊtrue��ʾ��ͨ������֮�ر�ͨ��
@return ��
@note ��DI�豸       
*/
void CMotionOutput::OpenDIChannel(int iChannel,bool bEnable)
{
	m_Motion.OpenDIChannel(iChannel,bEnable);
}
 
//-------------------------------------------------------------------------
/*!��������: OpenDOChannel

@param iChannel ͨ���ţ��������Ϲ���16��DOͨ������ֵ��ΧΪ��0~15��
@param bEnable  true��ʾ��ͨ������֮�ر�ͨ��
@return ��
@note ��DO�豸     
*/
void CMotionOutput::OpenDOChannel(int iChannel,bool bEnable)
{
	m_Motion.OpenDOChannel(iChannel,bEnable);
}
 
//-------------------------------------------------------------------------
/*!��������: OpenDAChannel

@param  iChannel ͨ���ţ��������Ϲ���6��DAͨ����DA1��DA2ͨ���ѱ�ռ�ã���˸�ֵ��ΧΪ��3~6��
@param  value �����ֵ��0~255��
@param  bEnable Ϊtrue��ʾ��ͨ������֮�ر�ͨ��
@return ��
@note  ��DA�豸 
*/
void CMotionOutput::OpenDAChannel(int iChannel,int value,bool bEnable)
{
	m_Motion.OpenDAChannel(iChannel,value,bEnable);
}
 
//-------------------------------------------------------------------------
/*!��������: LoadXMLFile

@param  ��
@return װ�سɹ�������true
@note װ��XML�ļ���MTRobotConfig.xml��  
*/
bool CMotionOutput::LoadXMLFile()
{
	return m_Motion.LoadXMLFile();
}
 
//-------------------------------------------------------------------------
/*!��������: SaveXMLFile

@param  ��
@return ����ɹ�������true
@note ����XML�ļ���MTRobotConfig.xml��
*/
bool CMotionOutput::SaveXMLFile()
{
	m_Motion.m_RobotInfo = m_RobotInfo;
	return m_Motion.SaveXMLFile();
}
 
//-------------------------------------------------------------------------
/*!��������: SetControlModeSel

@param  controlmode VECTOR_COMPOSITEʸ���ϳɣ�CONTROL_AXIS�������
@return ��
@note ���ÿ���ģʽ
*/
void CMotionOutput::SetControlModeSel(MTORIParamInfo::CONTROL controlmode)
{
	#ifdef MTORROBOT//����ȫ�������
	m_Motion.SetControlModeSel(controlmode);
#endif
}
 
//-------------------------------------------------------------------------
/*!��������: SetDribblingSpeed

@param  m_lSpeed ������ٶ�
@param  m_rSpeed �Ҳ����ٶ�
@return ��
@note �����������ٶ�
*/
void CMotionOutput::SetDribblingSpeed(float m_lSpeed,float m_rSpeed)
{
	#ifdef MTORROBOT//����ȫ�������
	m_Motion.SetDribblingSpeed(m_lSpeed,m_rSpeed);
#endif
}
 
//-------------------------------------------------------------------------
/*!��������: SetKickStrength

@param m_Strength ������������
@return ��
@note ������������� 
*/
void CMotionOutput::SetKickStrength(int m_Strength)
{
	#ifdef MTORROBOT//����ȫ�������
	m_Motion.SetKickStrength(m_Strength);
	#endif
}
 
//-------------------------------------------------------------------------
/*!��������: Check

@param  ��
@return ��
@note ��ѯ״̬��Ϣ��ÿ����һ�η���һ��״̬��Ϣ
*/
void CMotionOutput::Check()
{
	m_Motion.Check();
}
 
//-------------------------------------------------------------------------
/*!��������: GetComInfo

@param bMotionserial [out]�˶��˿��Ƿ񱻴򿪵ı�־
@param bSonarserial [out]�����˿��Ƿ񱻴򿪵ı�־
@param PSDserial [out]PSD�˿��Ƿ񱻴򿪵ı�־
@param bDIserial [out]DI�˿��Ƿ񱻴򿪵ı�־
@param bDOserial [out]DO�˿��Ƿ񱻴򿪵ı�־
@param bDAserial [out]DA�˿��Ƿ񱻴򿪵ı�־
@return ��
@note ��ô�����Ϣ 
*/
void CMotionOutput::GetComInfo(bool &bMotionserial,bool &bSonarserial,bool &PSDserial,bool &bDIserial,bool &bDOserial,bool &bDAserial)
{
	m_Motion.GetComInfo(bMotionserial,bSonarserial,PSDserial,bDIserial,bDOserial,bDAserial);
}


 
//-------------------------------------------------------------------------
/*!��������: ArmCalibrate

@param  ��
@return ��
@note �ڱ۱궨��� 
*/
void CMotionOutput::ArmCalibrate()
{
	#ifdef MTFRROBOT//�Ĵ�������
	m_Motion.ArmCalibrate();
#endif
}
 
//-------------------------------------------------------------------------
/*!��������: ControlArm

@param  m_fSpeed �ڱ��ٶ�(deg/s)
@param  m_fAngle �ڱ۽Ƕ�(deg)
@return ��
@note ���ưڱ��˶�    
*/
void CMotionOutput::ControlArm(float m_fSpeed,float m_fAngle)
{
	#ifdef MTFRROBOT//�Ĵ�������
	m_Motion.ControlArm(m_fSpeed,m_fAngle);
#endif
}
 
//-------------------------------------------------------------------------
/*!��������: Clear

@param  ��
@return ��
@note   ���㣨����ŷ�����ģ��״̬��Ϣ��
*/
void CMotionOutput::Clear()
{
	m_Motion.Clear();
}
 
//-------------------------------------------------------------------------
/*!��������: setVel

@param  velocity ֱ���ٶ�(-0.9m/s~0.9m/s)
@return 
@note ���������ٶ�
*/
void CMotionOutput::setVel(double velocity)
{
	SetModeSelect(MTRParamInfo::VEL_CL,0.0,0.0,0.0);
	Drive(velocity,0.0,0.0);
}

//-------------------------------------------------------------------------
/*!��������: setVel2

@param  leftVelocity  �����ٶ�(-0.9m/s~0.9m/s)
@param  rightVelocity �����ٶ�(-0.9m/s~0.9m/s)
@return ��
@note �����������ٶ�
*/
void CMotionOutput::setVel2(double leftVelocity,double rightVelocity)
{
	double xVel = 0.0;
	double wVel = 0.0;
	xVel = (leftVelocity+rightVelocity)/2;
	wVel = (rightVelocity-leftVelocity)/0.523;
	SetModeSelect(MTRParamInfo::VEL_CL,0.0,0.0,0.0);
	Drive(xVel,0.0,wVel*180/PI);
}
 
//-------------------------------------------------------------------------
/*!��������: setRotVel

@param velocity  ��ת�ٶ�(-90deg/s~90deg/s)
@return 
@note  ������ת�ٶ�  
*/
void CMotionOutput::setRotVel(double velocity)
{
	SetModeSelect(MTRParamInfo::VEL_CL,0.0,0.0,0.0);
	Drive(0.0,0.0,velocity);
}
 
//-------------------------------------------------------------------------
/*!��������: move

@param distance λ��(-15~15m)
@param basicVel �ٶ�
@return ��
@note ��λ�ñջ��ƶ���λ��
*/
void CMotionOutput::move(double distance,float basicVel)
{
	SetModeSelect(MTRParamInfo::POS_CL,distance,distance,0.0);
	Drive(basicVel,0.0,0.0);
}
 
//-------------------------------------------------------------------------
/*!��������: setRotAngle

@param angle     �Ƕ�
@param basicVel  �ٶ�
@param t 
@return 
@note ������ת�Ƕ�  
*/
void CMotionOutput::setRotAngle(double angle,float basicVel,float &t)
{
	float sl = 0.0,sr = 0.0;
	if(angle!=0)
	{
		sl = (2*PI*(0-0.523/2))/(360/angle);
		sr = (2*PI*(0+0.523/2))/(360/angle);
		if(fabs(sl)>fabs(sr))
		{
			t = fabs(sl)/basicVel;
		}
		else
		{
			t = fabs(sr)/basicVel;
		}
		if(t>0)
		{
			SetModeSelect(MTRParamInfo::POS_CL,sl,sr,0.0);
			Drive(basicVel,0.0,0.0);
		}
	}
}
 
//-------------------------------------------------------------------------
/*!��������: setRotRadius

@param  radius  �뾶��m��
@param  basicVel  �ٶ�
@return ��
@note  ������ת�뾶     
*/
void CMotionOutput::setRotRadius(double radius,float basicVel)
{
	float sl = 0.0,sr = 0.0;
	float t = 0.0;
	sl = 2*PI*(radius-0.523/2);
	sr = 2*PI*(radius+0.523/2);
	if(fabs(sl)>fabs(sr))
	{
		t = fabs(sl)/basicVel;
	}
	else
	{
		t = fabs(sr)/basicVel;
	}
    if(t>0)
	{
		/*SetModeSelect(MTRParamInfo::POS_CL,sl,sr,0.0);
		Drive(basicVel,0.0,0);*/
		setVel2(sl/t,sr/t);
	}
}
 
//-------------------------------------------------------------------------
/*!��������: stop

@param  ��
@return ��
@note ������ֹͣ�˶�   
*/
void CMotionOutput::stop()
{
	Drive(0.0,0.0,0.0);
}
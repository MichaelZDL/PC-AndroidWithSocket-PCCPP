/*!

@file MotionOutput.cpp
@brief CMotionOutput类实现文件，机器人运动、超声、PSD等设备接口。
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
/*!函数名称: Init

@param  无
@return 无
@note  读取XML文件获得各设备的端口号并打开运动设备端口
*/
void CMotionOutput::Init()
{
	m_Motion.InputUserPassword(USER,"User","User");
	if(m_Motion.LoadXMLFile())
	{
		m_RobotInfo = m_Motion.m_RobotInfo;
		for(int i=0;i<m_Motion.m_RobotInfo.m_Sensor.size();i++)
		{
			if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"主动轮及码盘"))
			{
				sscanf(m_Motion.m_RobotInfo.m_Sensor[i].strCom,"COM%d",&m_MotionPort);
			}
			else if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"超声"))
			{
				sscanf(m_Motion.m_RobotInfo.m_Sensor[i].strCom,"COM%d",&m_SonarPort);
			}
			else if(!strcmp(m_Motion.m_RobotInfo.m_Sensor[i].strName,"PSD"))//获得PSD设备对应的端口号
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
			m_Motion.SetReceProc(m_DeviceProc,this);//设置事件处理过程
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
/*!函数名称: ReceFun

@param pParam 指向类或结构体的指针
@param type 类型
@param value 返回值
@param len 数组长度
@return 无
@note  回调函数，串口接收到数据进行解析完成后会通知该函数体
*/
void _stdcall ReceFun(void* pParam,int type,float* value,int len)
{
	CMotionOutput* pThis = (CMotionOutput*)pParam;
	CSingleLock sLock(&pThis->m_Mutex);
	int m_StateInfo[3][16]={0};//三个伺服驱动模块(左轮、右轮、摆臂）对应的16种错误状态信息
	float m_value[3]={0};//三个伺服驱动模块状态信息未解析的
	int m_iCanStatus=0;//左轮Can状态
	int m_iCanStatus2=0;//右轮Can状态
	int m_iCanStatus3=0;//摆臂Can状态
	int m_StopButtonStatus=0;//急停状态
	bool bException= false;//异常标志
	static bool bSend = false;//出现异常时需发送停止指令，该标志用来控制发送一次停止指令，直到异常状态消失后再次出现异常时使用
	sLock.Lock();
	
	switch(type)
	{
	case MOTION:
		pThis->m_ReceInfo._MotionInfo.fXVelocity = value[0];//X轴速度
		pThis->m_ReceInfo._MotionInfo.fYVelocity = value[1];//Y轴速度
		pThis->m_ReceInfo._MotionInfo.fZVelocity = value[2];//Z轴速度
		pThis->m_ReceInfo._MotionInfo.fXDistance = value[3];//X轴位移
		pThis->m_ReceInfo._MotionInfo.fYDistance = value[4];//Y轴位移
		pThis->m_ReceInfo._MotionInfo.fZDistance = value[5];//Z轴位移
		pThis->m_ReceInfo._MotionInfo.fLDis = value[6];//左轮位移
		pThis->m_ReceInfo._MotionInfo.fRDis = value[7];//右轮位移
		pThis->m_ReceInfo._MotionInfo.bReceInfo = true;//提供给外部类调用使用，用于判断是否接收到串口数据
		break;
	case SONAR:
		pThis->m_ReceInfo.m_fSonarDis[len] = *value;//超声波传感器返回值
		break;
	case PID:
		pThis->m_ReceInfo._PidInfo.flKp = value[0];//RAM内存储的左轮比例值
		pThis->m_ReceInfo._PidInfo.flKi = value[1];//RAM内存储的左轮积分值
		pThis->m_ReceInfo._PidInfo.flKd = value[2];//RAM内存储的左轮微分值
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
		pThis->m_ReceInfo.m_fPSDDis[len] = *value;//PSD红外传感器距离值
		break;
	case DI:
		for(int i=0;i<16;i++)
		{
			pThis->m_ReceInfo.m_iDI[i] = (int)value[i];//数字量输入返回值
		}
		break;
	case ENCODER_RELA:
		pThis->m_ReceInfo._MotionInfo.fXSpan = value[0];//X轴相对位移
		pThis->m_ReceInfo._MotionInfo.fYSpan = value[1];//Y轴相对位移
		pThis->m_ReceInfo._MotionInfo.fZSpan = value[2];//Z轴相对位移
		if(pThis->bClear)//是否被清零的标志
		{
			pThis->bClear = false;
			pThis->m_ReceInfo._MotionInfo.fX = 0.0;//世界坐标系下的X值
			pThis->m_ReceInfo._MotionInfo.fY = 0.0;//世界坐标系下的Y值
			pThis->m_ReceInfo._MotionInfo.fZ = 0.0;//世界坐标系下的Z值角度值
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
		pThis->m_ReceInfo._MotionInfo.fX += (value[0] * cos(pThis->m_ReceInfo._MotionInfo.fZ*PI/180) - value[1] * sin(pThis->m_ReceInfo._MotionInfo.fZ*PI/180));//机器人坐标与世界坐标转换，世界坐标系与机器人坐标系XYZ轴方向相同
		pThis->m_ReceInfo._MotionInfo.fY += (value[0] * sin(pThis->m_ReceInfo._MotionInfo.fZ*PI/180) + value[1] * cos(pThis->m_ReceInfo._MotionInfo.fZ*PI/180));
		pThis->m_ReceInfo._MotionInfo.bReceInfo = true;
		break;
	case CHECK://机器人自检时的返回值，履带机器人没有该项功能
		CRESULAT::CHECKRESULT Res;
		if(value[2]==1)
		{
			Res = CRESULAT::CHECKRESULT::CHECKING;//正在检测
		}
		else if(value[2]==2)
		{
			Res = CRESULAT::CHECKRESULT::ABNORMAL;//检测异常
		}
		else if(value[2]==4)
		{
			Res = CRESULAT::CHECKRESULT::NORMAL;//检测正常
		}
		if(value[0]==1)//返回左右轮电机检测状态
		{
			if(value[1]==0)//左电机状态
			{
				pThis->m_ReceInfo._SelfCheckInfo.m_LeftMotorRes = Res;
			}
			else//右电机状态
			{
				pThis->m_ReceInfo._SelfCheckInfo.m_RightMotorRes = Res;
			}
		}
		else if(value[0]==3)//返回超声波传感器检测状态
		{
			pThis->m_ReceInfo._SelfCheckInfo.m_SonarRes[(int)value[1]] = Res;
		}
		else if(value[0]==5)//返回PSD红外传感器检测状态
		{
			pThis->m_ReceInfo._SelfCheckInfo.m_PSDRes[(int)value[1]] = Res;
		}
		break;
	case MTFRINFO://返回履带机器人的一些状态信息
		pThis->m_ReceInfo._MTFROtherInfo.m_fArmAngle = value[0];//摆臂角度
		pThis->m_ReceInfo._MTFROtherInfo.m_fLeftCurrent = value[1];//左轮电流
		pThis->m_ReceInfo._MTFROtherInfo.m_fRightCurrent = value[2];//右轮电流
		pThis->m_ReceInfo._MTFROtherInfo.m_fArmCurrent = value[3];//摆臂电流
		pThis->m_ReceInfo._MTFROtherInfo.m_LeftStatus = value[4];//左轮状态
		pThis->m_ReceInfo._MTFROtherInfo.m_RightStatus = value[5];//右轮状态
		pThis->m_ReceInfo._MTFROtherInfo.m_ArmStatus = value[6];//摆臂状态
		pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus = value[7];//系统状态
		pThis->m_ReceInfo._MTFROtherInfo.bReceInfo = true;
		
		m_value[0] = (float)pThis->m_ReceInfo._MTFROtherInfo.m_LeftStatus;
		m_value[1] = (float)pThis->m_ReceInfo._MTFROtherInfo.m_RightStatus;
		m_value[2] = (float)pThis->m_ReceInfo._MTFROtherInfo.m_ArmStatus;
		for(int j=0;j<3;j++)
		{
			for(int i=0;i<16;i++)
			{
				m_StateInfo[j][15-i] = (((int)m_value[j])&(1<<i))>>i;//分别为左轮、右轮、摆臂状态信息,共16种状态信息
				//分别对应Dsp异常复位、峰值过流、驱动异常、正限位、负限位、过温、实际速度异常、系统断电复位、稳态过流、
				//数据保存异常、电流采集异常、总线通讯异常、欠压检测异常、期望速度异常、位置采集异常、软件限位
				if(m_StateInfo[j][15-i]==1)
				{
					bException = true; 
				}
			}
		}
		
		m_iCanStatus = (((int)pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus)&0x02)>>1;//左轮can总线状态
	    m_iCanStatus2 = (((int)pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus)&0x04)>>2;//右轮can总线状态
	    m_iCanStatus3 = (((int)pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus)&0x08)>>3;//摆臂can总线状态
	    m_StopButtonStatus = ((pThis->m_ReceInfo._MTFROtherInfo.m_SystemStatus)&0x01);//反映急停开关状态
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
		
		if(bException)//一旦出现任何一种异常将通知机器人停止运行
		{
			if(!bSend)//以免重复向串口发送停止指令
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
/*!函数名称: InputUserPassword

@param _group  组名，分为ADMIN管理员及USER用户
@param _userName  用户名
@param _password  密码 
@return m_Motion.InputUserPassword(_group,_userName,_password)的返回值
@note 输入用户名及密码
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
/*!函数名称: connect_Motionserial

@param   port 端口号
@param baudrate 波特率
@return m_Motion.connect_Motionserial(port,baudrate)的返回值
@note 连接运动设备
*/
int CMotionOutput::connect_Motionserial(int port, long baudrate)
{
	return m_Motion.connect_Motionserial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!函数名称: connect_SonarSerial

@param   port 端口号
@param baudrate 波特率
@return m_Motion.connect_SonarSerial(port,baudrate)的返回值
@note 连接超声设备
*/
int CMotionOutput::connect_SonarSerial(int port,long baudrate)
{
	return m_Motion.connect_SonarSerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!函数名称: connect_PSDSerial

@param   port 端口号
@param baudrate 波特率
@return m_Motion.connect_PSDSerial(port,baudrate)的返回值
@note   连接PSD设备
*/
int CMotionOutput::connect_PSDSerial(int port,long baudrate)
{
	return m_Motion.connect_PSDSerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!函数名称: connect_DISerial

@param   port 端口号
@param baudrate 波特率
@return m_Motion.connect_DISerial(port,baudrate)的返回值
@note  连接DI数字量输入设备
*/
int CMotionOutput::connect_DISerial(int port,long baudrate)
{
	return m_Motion.connect_DISerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!函数名称: connect_DOSerial

@param   port 端口号
@param baudrate 波特率
@return m_Motion.connect_DOSerial(port,baudrate)的返回值
@note 连接DO数字量输出设备
*/
int CMotionOutput::connect_DOSerial(int port,long baudrate)
{
	return m_Motion.connect_DOSerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!函数名称: connect_DASerial

@param   port 端口号
@param baudrate 波特率  
@return m_Motion.connect_DASerial(port,baudrate)的返回值
@note 连接DA数字量转模拟量设备
*/
int CMotionOutput::connect_DASerial(int port,long baudrate)
{
	return m_Motion.connect_DASerial(port,baudrate);
}
 
//-------------------------------------------------------------------------
/*!函数名称: disconnect_serial

@param  无
@return m_Motion.disconnect_serial()的返回值
@note 关闭所有串口
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
/*!函数名称: Drive

@param Vel_x X轴速度
@param Vel_y Y轴速度,对于两轮机器人在Y轴方向上没有速度，可设定该值为0
@param Vel_z Z轴速度
@return 无
@note  控制运动设备，控制模式默认为速度闭环，若需要运行位置闭环时需在该函数前加SetModeSelect进行设定
*/
void CMotionOutput::Drive(float Vel_x,float Vel_y,float Vel_z)
{
	m_Motion.Drive(Vel_x,Vel_y,Vel_z);
}
 
//-------------------------------------------------------------------------
/*!函数名称: OpenSonarChannel

@param  iChannel 超声通道号，机器人上共有16个超声通道，该值范围为（0~15），机器人出厂前默认机器人正前方的超声通道号为0，逆时针旋转通道号增加
@param  bEnable  为true表示打开该超声通道，反之关闭
@return 无
@note   打开超声设备    
*/
void CMotionOutput::OpenSonarChannel(int iChannel,bool bEnable)
{
	m_Motion.OpenSonarChannel(iChannel,bEnable);
}
 
//-------------------------------------------------------------------------
/*!函数名称: SetModeSelect

@param  modesel 模式，有以下取值：\n
			MTRParamInfo::POS_CL位置闭环\n
			MTRParamInfo::VEL_CL速度闭环
@param xDis 两轮机器人时该值对应左轮位移，三轮或四轮时对应的是X轴的位移，模式为速度闭环时该值可为0
@param yDis 两轮机器人时该值对应右轮位移，三轮或四轮时对应的是Y轴的位移，模式为速度闭环时该值可为0
@param zDis 两轮机器人时该值为0，三轮或四轮时对应的是Z轴的位移，模式为速度闭环时该值可为0
@return 无
@note 设置模式选择  
*/
void CMotionOutput::SetModeSelect(int modesel,float xDis,float yDis,float zDis)//
{
	m_Motion.SetModeSelect(modesel,xDis,yDis,zDis);
}


/********************************************************************
**                                                   **

********************************************************************/
//-------------------------------------------------------------------------
/*!函数名称: SetAccelerate

@param  xAcc x方向加速度
@param  yAcc y方向加速度
@param  zAcc z方向加速度
@return 无
@note 设置加速度值  
*/
void CMotionOutput::SetAccelerate(float xAcc,float yAcc,float zAcc)
{
	#ifdef MTRROBOT//对应两轮轮式机器人
	m_Motion.SetAccelerate(xAcc,yAcc,zAcc);
#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: SetPidSelect

@param   pidsel		PID模式，PID_DEF默认PID，PID_DEBUG在线调试PID，PID_LOAD为使用E2PROM内保存好的PID值
@param   Kp		Kp[0]左轮比例值，Kp[1]右轮比例值
@param   Ki		Ki[0]左轮积分值，Ki[1]右轮积分值
参数4：Kd[0]左轮微分值，Kd[1]右轮微分值
@return 无
@note 设置PID选择          
*/
void CMotionOutput::SetPidSelect(int pidsel,float Kp[2],float Ki[2],float Kd[2])
{
	#ifdef MTRROBOT//对应两轮轮式机器人
	m_Motion.SetPidSelect(pidsel,Kp,Ki,Kd);
#endif
}

 
//-------------------------------------------------------------------------
/*!函数名称: StartCheckSelf

@param  bEnable 为true表示启动自检，反之停止自检
@return 无
@note 启动自检，检测电机，超声，PSD 
*/
void CMotionOutput::StartCheckSelf(bool bEnable)
{
	#ifdef MTRROBOT//对应两轮轮式机器人
	m_Motion.StartCheckSelf(bEnable);
#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: SavePID

@param  Kp  Kp[0]左轮比例值，Kp[1]右轮比例值
@param  Ki  Ki[0]左轮积分值，Ki[1]右轮积分值
@param  Kd  Kd[0]左轮微分值，Kd[1]右轮微分值
@return 无
@note  保存PID 
*/
void CMotionOutput::SavePID(float Kp[2],float Ki[2],float Kd[2])
{
	#ifdef MTRROBOT//对应两轮轮式机器人
	m_Motion.SavePID(Kp,Ki,Kd);
#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: LookOverPIDParam

@param  无
@return 无
@note 查看PID参数
*/
void CMotionOutput::LookOverPIDParam()
{
	#ifdef MTRROBOT//对应两轮轮式机器人
	m_Motion.LookOverPIDParam();
#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: EnableEncoderDev

@param  Type 码盘类型，ENCODER_ABSOLUTE返回绝对值码盘数据，ENCODER_RELATIVE返回相对值码盘数据
@param  bEnable 为true表示启动码盘返回功能，当需要返回相对值码盘数据时需要间隔一段时间发送一次该函数
@return 无
@note 启动码盘设备
*/
void CMotionOutput::EnableEncoderDev(MTRParamInfo::ENCODERTYPE Type,bool bEnable)
{
	#ifdef MTRROBOT//对应两轮轮式机器人
	m_Motion.EnableEncoderDev(Type,bEnable);
	#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: GetRobotType

@param  无
@return  返回机器人类型，返回值为MTR说明该软件对应的是轮式机器人的软件，为MTFR说明该软件适合履带机器人使用
@note 获得机器人类型 
*/
const char* CMotionOutput::GetRobotType()
{
	return m_Motion.GetRobotType();
}
 
//-------------------------------------------------------------------------
/*!函数名称: OpenPSDChannel

@param  iChannel 通道号，机器人上共有16个PSD通道，该值范围为（0~15），机器人出厂前默认机器人正前方的PSD通道号为0，逆时针旋转通道号增加
@param  bEnable  为true表示打开通道，反之关闭通道
@return 无
@note  打开PSD设备  
*/
void CMotionOutput::OpenPSDChannel(int iChannel,bool bEnable)
{
	m_Motion.OpenPSDChannel(iChannel,bEnable);
}
 
//-------------------------------------------------------------------------
/*!函数名称: OpenDIChannel

@param iChannel 通道号，机器人上共有16个DI通道，该值范围为（0~15）
@param bEnable 为true表示打开通道，反之关闭通道
@return 无
@note 打开DI设备       
*/
void CMotionOutput::OpenDIChannel(int iChannel,bool bEnable)
{
	m_Motion.OpenDIChannel(iChannel,bEnable);
}
 
//-------------------------------------------------------------------------
/*!函数名称: OpenDOChannel

@param iChannel 通道号，机器人上共有16个DO通道，该值范围为（0~15）
@param bEnable  true表示打开通道，反之关闭通道
@return 无
@note 打开DO设备     
*/
void CMotionOutput::OpenDOChannel(int iChannel,bool bEnable)
{
	m_Motion.OpenDOChannel(iChannel,bEnable);
}
 
//-------------------------------------------------------------------------
/*!函数名称: OpenDAChannel

@param  iChannel 通道号，机器人上共有6个DA通道，DA1、DA2通道已被占用，因此该值范围为（3~6）
@param  value 输入的值（0~255）
@param  bEnable 为true表示打开通道，反之关闭通道
@return 无
@note  打开DA设备 
*/
void CMotionOutput::OpenDAChannel(int iChannel,int value,bool bEnable)
{
	m_Motion.OpenDAChannel(iChannel,value,bEnable);
}
 
//-------------------------------------------------------------------------
/*!函数名称: LoadXMLFile

@param  无
@return 装载成功，返回true
@note 装载XML文件（MTRobotConfig.xml）  
*/
bool CMotionOutput::LoadXMLFile()
{
	return m_Motion.LoadXMLFile();
}
 
//-------------------------------------------------------------------------
/*!函数名称: SaveXMLFile

@param  无
@return 保存成功，返回true
@note 保存XML文件（MTRobotConfig.xml）
*/
bool CMotionOutput::SaveXMLFile()
{
	m_Motion.m_RobotInfo = m_RobotInfo;
	return m_Motion.SaveXMLFile();
}
 
//-------------------------------------------------------------------------
/*!函数名称: SetControlModeSel

@param  controlmode VECTOR_COMPOSITE矢量合成，CONTROL_AXIS单轴控制
@return 无
@note 设置控制模式
*/
void CMotionOutput::SetControlModeSel(MTORIParamInfo::CONTROL controlmode)
{
	#ifdef MTORROBOT//三轮全向机器人
	m_Motion.SetControlModeSel(controlmode);
#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: SetDribblingSpeed

@param  m_lSpeed 左侧电机速度
@param  m_rSpeed 右侧电机速度
@return 无
@note 设置盘球电机速度
*/
void CMotionOutput::SetDribblingSpeed(float m_lSpeed,float m_rSpeed)
{
	#ifdef MTORROBOT//三轮全向机器人
	m_Motion.SetDribblingSpeed(m_lSpeed,m_rSpeed);
#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: SetKickStrength

@param m_Strength 设置踢球力量
@return 无
@note 设置踢球的力量 
*/
void CMotionOutput::SetKickStrength(int m_Strength)
{
	#ifdef MTORROBOT//三轮全向机器人
	m_Motion.SetKickStrength(m_Strength);
	#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: Check

@param  无
@return 无
@note 查询状态信息，每发送一次返回一次状态信息
*/
void CMotionOutput::Check()
{
	m_Motion.Check();
}
 
//-------------------------------------------------------------------------
/*!函数名称: GetComInfo

@param bMotionserial [out]运动端口是否被打开的标志
@param bSonarserial [out]超声端口是否被打开的标志
@param PSDserial [out]PSD端口是否被打开的标志
@param bDIserial [out]DI端口是否被打开的标志
@param bDOserial [out]DO端口是否被打开的标志
@param bDAserial [out]DA端口是否被打开的标志
@return 无
@note 获得串口信息 
*/
void CMotionOutput::GetComInfo(bool &bMotionserial,bool &bSonarserial,bool &PSDserial,bool &bDIserial,bool &bDOserial,bool &bDAserial)
{
	m_Motion.GetComInfo(bMotionserial,bSonarserial,PSDserial,bDIserial,bDOserial,bDAserial);
}


 
//-------------------------------------------------------------------------
/*!函数名称: ArmCalibrate

@param  无
@return 无
@note 摆臂标定零点 
*/
void CMotionOutput::ArmCalibrate()
{
	#ifdef MTFRROBOT//履带机器人
	m_Motion.ArmCalibrate();
#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: ControlArm

@param  m_fSpeed 摆臂速度(deg/s)
@param  m_fAngle 摆臂角度(deg)
@return 无
@note 控制摆臂运动    
*/
void CMotionOutput::ControlArm(float m_fSpeed,float m_fAngle)
{
	#ifdef MTFRROBOT//履带机器人
	m_Motion.ControlArm(m_fSpeed,m_fAngle);
#endif
}
 
//-------------------------------------------------------------------------
/*!函数名称: Clear

@param  无
@return 无
@note   清零（清除伺服驱动模块状态信息）
*/
void CMotionOutput::Clear()
{
	m_Motion.Clear();
}
 
//-------------------------------------------------------------------------
/*!函数名称: setVel

@param  velocity 直行速度(-0.9m/s~0.9m/s)
@return 
@note 设置运行速度
*/
void CMotionOutput::setVel(double velocity)
{
	SetModeSelect(MTRParamInfo::VEL_CL,0.0,0.0,0.0);
	Drive(velocity,0.0,0.0);
}

//-------------------------------------------------------------------------
/*!函数名称: setVel2

@param  leftVelocity  左轮速度(-0.9m/s~0.9m/s)
@param  rightVelocity 右轮速度(-0.9m/s~0.9m/s)
@return 无
@note 设置左右轮速度
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
/*!函数名称: setRotVel

@param velocity  旋转速度(-90deg/s~90deg/s)
@return 
@note  设置旋转速度  
*/
void CMotionOutput::setRotVel(double velocity)
{
	SetModeSelect(MTRParamInfo::VEL_CL,0.0,0.0,0.0);
	Drive(0.0,0.0,velocity);
}
 
//-------------------------------------------------------------------------
/*!函数名称: move

@param distance 位移(-15~15m)
@param basicVel 速度
@return 无
@note 以位置闭环移动的位移
*/
void CMotionOutput::move(double distance,float basicVel)
{
	SetModeSelect(MTRParamInfo::POS_CL,distance,distance,0.0);
	Drive(basicVel,0.0,0.0);
}
 
//-------------------------------------------------------------------------
/*!函数名称: setRotAngle

@param angle     角度
@param basicVel  速度
@param t 
@return 
@note 设置旋转角度  
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
/*!函数名称: setRotRadius

@param  radius  半径（m）
@param  basicVel  速度
@return 无
@note  设置旋转半径     
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
/*!函数名称: stop

@param  无
@return 无
@note 机器人停止运动   
*/
void CMotionOutput::stop()
{
	Drive(0.0,0.0,0.0);
}
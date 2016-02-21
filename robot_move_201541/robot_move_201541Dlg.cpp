// robot_move_201541Dlg.cpp : implementation file
//

#include "stdafx.h"
#include "robot_move_201541.h"
#include "robot_move_201541Dlg.h"
#include "Laser_Obstacle_Avoidance.h"
#include "opencv.hpp"


using namespace cv;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//窗口画图变量
CRect rect;
CDC *pDC;
HDC hDC;
CWnd *pwnd;

//机器人运动控制类
CMotionOutput m_MotionOutput;
threadInfo Info;

//线程终止开始标志位
int StartAvdObs=0;
int SetSICK=1;
int StartWatchdata=0;

#define COM_PORT 1//选择串口
LMSControl Laser;
int bufer[MAXPACKET];
int Length;

//WindowsSocket网络控制类
WindowsSocket windowsSocket;
Crobot_move_201541Dlg* CrobotDlg = NULL;
BOOLEAN OnlineOn=TRUE;
BOOLEAN threadFuncOverOK=TRUE;
#define CONNECT_SOCKET_OK 1008

void Crobot_move_201541Dlg::Drive(float m_fXVel,float m_fYVel,float m_fZVel)
{

	m_MotionOutput.Drive(m_fXVel,m_fYVel,m_fZVel);	
	//停止Drive(0,0,0);
	//前进Drive(0.0,0,0);
	//后退Drive(-0.05,0,0);
	//左转Drive(0,0,5);
	//右转Drive(0,0,-5);
}

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// Crobot_move_201541Dlg dialog




Crobot_move_201541Dlg::Crobot_move_201541Dlg(CWnd* pParent /*=NULL*/)
	: CDialog(Crobot_move_201541Dlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void Crobot_move_201541Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);

	DDX_Control(pDX, IDC_EDIT5, printfCEdit);
	DDX_Control(pDX, IDC_EDIT4, m_SendData);
}

BEGIN_MESSAGE_MAP(Crobot_move_201541Dlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON1, &Crobot_move_201541Dlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &Crobot_move_201541Dlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &Crobot_move_201541Dlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_SICK, &Crobot_move_201541Dlg::OnBnClickedSick)
	ON_BN_CLICKED(IDC_BUTTON5, &Crobot_move_201541Dlg::OnBnClickedButton5)
	ON_MESSAGE(WM_MICHAEL, &Crobot_move_201541Dlg::OnMichael)
	ON_BN_CLICKED(IDC_AVOID, &Crobot_move_201541Dlg::OnBnClickedAvoid)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON6, &Crobot_move_201541Dlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON7, &Crobot_move_201541Dlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON8, &Crobot_move_201541Dlg::OnBnClickedButton8)
END_MESSAGE_MAP()


// Crobot_move_201541Dlg message handlers

BOOL Crobot_move_201541Dlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	CrobotDlg=this;
	m_MotionOutput.Init();       //机器人运动初始化

	pwnd = GetDlgItem(IDC_Video);
	pDC =pwnd->GetDC();
	hDC= pDC->GetSafeHdc();
	pwnd->GetClientRect(&rect);

	if(SetSICK==1)
	{
		SetSICK=0;	
		char s[5];
		sprintf_s(s, "COM%d", COM_PORT);
		HANDLE j;
		//SICK configuration 
		j=Laser.ConfigureComPort((LPCSTR)s);
		//Laser.ChangeToMM();
		Laser.ChangeAngleRes(ANGLE_RES_180X1);
		Laser.StartContinuousOutput();
		Info.pClass=this;
		SetTimer(1, 500, NULL);//读数据，构图
		hThreadAvdObs = CreateThread(NULL,
			0,
			(LPTHREAD_START_ROUTINE)ThreadAvdObs,
			&Info,
			0,
			&ThreadIDAvdObs);
	}	

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void Crobot_move_201541Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void Crobot_move_201541Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR Crobot_move_201541Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void Crobot_move_201541Dlg::OnBnClickedButton1()
{
	// TODO: Add your control notification handler code here
	Drive(0.05,0,0);
}



void Crobot_move_201541Dlg::OnBnClickedButton2()
{
	// TODO: Add your control notification handler code here
	Drive(0,0,0);
}

void Crobot_move_201541Dlg::OnBnClickedButton3()
{
	// TODO: Add your control notification handler code here
	Drive(-0.05,0,0);
}

void Crobot_move_201541Dlg::OnBnClickedButton6()
{
	// TODO: Add your control notification handler code here
	Drive(0,0,8);
}

void Crobot_move_201541Dlg::OnBnClickedButton7()
{
	// TODO: Add your control notification handler code here
	Drive(0,0,-8);
}

void Crobot_move_201541Dlg::OnBnClickedSick()
{
	// TODO: Add your control notification handler code here 
	StartAvdObs=!StartAvdObs;

	
	if (StartAvdObs==1)
	{
	
	}
	else
	{
		Drive(0,0,0);
	}


}

void Crobot_move_201541Dlg::OnBnClickedButton5()//WatchData
{
	//// TODO: Add your control notification handler code here
	//Laser.PolarToRect(Length, bufer, recBuf);
	//windowsSocket.SendIntArray362ToTCPClient(recBuf);

}

UINT ThreadFunc(LPVOID lpParam)
{	
	threadFuncOverOK=FALSE;
	threadInfo* pInfo = (threadInfo*)lpParam;
	windowsSocket.ListenTCPClient();
	threadFuncOverOK=TRUE;
	return 0;
}



afx_msg LRESULT Crobot_move_201541Dlg::OnMichael(WPARAM wParam, LPARAM lParam)
{
	CString* rmsg = (CString*)lParam;
	
	if (*rmsg==(_T("MT-R move forward")))OnBnClickedButton1();
	if (*rmsg==(_T("MT-R move stop")))OnBnClickedButton2();
	if (*rmsg==(_T("MT-R move back")))OnBnClickedButton3();
	if (*rmsg==(_T("MT-R move left")))OnBnClickedButton6();
	if (*rmsg==(_T("MT-R move right")))OnBnClickedButton7();	

	PrintfMFC(*rmsg);
	PrintfMFC(_T("\n"));
	connectSocketOK=wParam;
	return 0;
}
void Crobot_move_201541Dlg::OnBnClickedAvoid()
{
	// TODO: Add your control notification handler code here
	StartAvdObs=0;
	SetSICK=1;
	OnlineOn=FALSE;
	windowsSocket.ShutDownBoth();
	if (threadFuncOverOK==FALSE)
	{
		PrintfMFC(_T("Waiting closing TCPListen...\r\n"));
		return;
	}
	PrintfMFC(_T("TCPListen closed"));
	Drive(0,0,0);
	KillTimer(1);
	CDialog::OnCancel();
}


UINT ThreadAvdObs(LPVOID lpParam)
{
	LaserObAvoid Av;
	threadInfo* pInfo = (threadInfo*)lpParam;
	

	while(SetSICK==0)
	{
		//读极坐标激光供全局使用
		Length=Laser.ReadLMSData(bufer,POLARZuoBiao);
		if (StartAvdObs==1)
		{
			Av.MoveAndAovidObs(bufer,600);
		}
		
	}
	m_MotionOutput.Drive(0,0,0);
	Laser.StopContinuousOutput();
	return 0;
}

void Crobot_move_201541Dlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	switch(nIDEvent)
	{
	case(1):
		{	
			//转换成直角坐标构图
			int Buf[MAXPACKET];
			Laser.PolarToRect(Length, bufer, Buf);

			if (connectSocketOK==CONNECT_SOCKET_OK)
			{
				windowsSocket.SendIntArray362ToTCPClient(Buf);

			}

			CvvImage m_CvvImage;
			Mat Sick_image=Mat::zeros(400,400,CV_8UC3);//黑色画布
			int center_x,center_y;
			int thickness=-1;
			int lineType=8;
			int threshold=55;//设置避障阈值 单位CM
	
			//画绿色球模拟机器人
			center_x=0+200;//以（200，0）为原点
			center_y=400-0;
			circle(Sick_image,                  //圆将被画到img图像上  
				Point(center_x, center_y),                   //圆心位置由center定义  
				threshold,                     //圆的半径  
				Scalar(0, 255, 0),      //圆的颜色，红色  
				thickness,                //圆的线宽定义为-1，所以圆会被填充  
				lineType);
			
			rectangle( Sick_image,  
				Point(center_x-30, center_y),  
				Point(center_x+30, center_y-10),  
				Scalar( 0, 0, 0 ),  
				-1,  
				lineType ); 

			//画激光扫描点
			for (int i=0; i < (Length); i++)
			{
				if((Buf[2*i]/10)>=200)
					center_x=200+200;
				else if((Buf[2*i]/10)<=(-200))
					center_x=(-200)+200;
				else
					center_x=(Buf[2*i]/10)+200;

				if((Buf[2*i+1]/10)>=400)
					center_y=400-400;
				else
					center_y=400-(Buf[2*i+1]/10);

				circle(Sick_image,                  //圆将被画到img图像上  
					Point(center_x, center_y),                   //圆心位置由center定义  
					2,                     //圆的半径  
					Scalar(0, 0, 255),      //圆的颜色，红色  
					thickness,                //圆的线宽定义为-1，所以圆会被填充  
					lineType);
			}

			//画到窗口去
			IplImage pFrame=Sick_image;
			m_CvvImage.CopyOf(&pFrame,1);
			if (true)
			{
				m_CvvImage.DrawToHDC(hDC,&rect);
			}

		}
	}
	CDialog::OnTimer(nIDEvent);
}

BOOL Crobot_move_201541Dlg::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		switch(pMsg->wParam)
		{
			case VK_SHIFT:OnBnClickedSick();break;
			case VK_UP:OnBnClickedButton1();break;
			case VK_CONTROL:OnBnClickedButton2();break;
			case VK_DOWN:OnBnClickedButton3();break;
			case VK_LEFT:OnBnClickedButton6();break;
			case VK_RIGHT:OnBnClickedButton7();break;
			case VK_ESCAPE:OnBnClickedAvoid();break;
		}
		
	}
	return   CDialog::PreTranslateMessage(pMsg);

}


void Crobot_move_201541Dlg::OnBnClickedButton8()
{
	// TODO: Add your control notification handler code here
	Info.pClass=this;
	hThread = CreateThread(NULL,
		0,
		(LPTHREAD_START_ROUTINE)ThreadFunc,
		&Info,
		0,
		&ThreadID);
}

void Crobot_move_201541Dlg::PrintfMFC(LPCTSTR str)
{
	int  nLength  =  printfCEdit.SendMessage(WM_GETTEXTLENGTH);
	printfCEdit.SetSel(nLength, nLength);
	printfCEdit.ReplaceSel(str);
}


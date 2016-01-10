// robot_move_201541Dlg.h : header file
//

#pragma once

#include "MotionOutput.h"
#include "LMS_Control_Michael.h"
#include <windows.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "CvvImage.h"
#include "afxwin.h"
#include <string>
#include "WindowsSocket.h"
using namespace std;

// Crobot_move_201541Dlg dialog
class Crobot_move_201541Dlg : public CDialog
{
// Construction
public:
	Crobot_move_201541Dlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_ROBOT_MOVE_201541_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
// Implementation
protected:
	HICON m_hIcon;
	HANDLE hThread;
	HANDLE hThreadAvdObs;
	DWORD ThreadID;
	DWORD ThreadIDAvdObs;
	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
	void Drive(float m_fXVel,float m_fYVel,float m_fZVel);//ËÙ¶È±Õ»·
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedSick();
	afx_msg void OnBnClickedButton5();
	afx_msg LRESULT OnMichael(WPARAM wParam, LPARAM lParam);

	afx_msg void OnBnClickedAvoid();
	afx_msg void OnTimer(UINT_PTR nIDEvent);

	BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnBnClickedButton6();
	afx_msg void OnBnClickedButton7();
	afx_msg void OnBnClickedButton8();
	void PrintfMFC(LPCTSTR);
	CEdit printfCEdit;

	afx_msg void OnBnClickedButton9();
	afx_msg void OnBnClickedButton10();
	CEdit m_SendData;
};

struct threadInfo
{
	int* pRangedata;
	Crobot_move_201541Dlg* pClass;
};

UINT ThreadFunc(LPVOID lpParam);
UINT ThreadAvdObs(LPVOID lpParam);
#define WM_MICHAEL WM_USER+100

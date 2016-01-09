// robot_move_201541.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// Crobot_move_201541App:
// See robot_move_201541.cpp for the implementation of this class
//

class Crobot_move_201541App : public CWinApp
{
public:
	Crobot_move_201541App();

// Overrides
	public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern Crobot_move_201541App theApp;
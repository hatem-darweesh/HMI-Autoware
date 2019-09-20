
// HMI_Win_Client.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CHMI_Win_ClientApp:
// See HMI_Win_Client.cpp for the implementation of this class
//

class CHMI_Win_ClientApp : public CWinApp
{
public:
	CHMI_Win_ClientApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CHMI_Win_ClientApp theApp;
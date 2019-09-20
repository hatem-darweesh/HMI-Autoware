
// HMI_Win_ClientDlg.h : header file
//



#pragma once

#include"CommToAuto.h"
#include "afxwin.h"
#include "afxcmn.h"


// CHMI_Win_ClientDlg dialog
class CHMI_Win_ClientDlg : public CDialogEx
{
// Construction
public:
	CHMI_Win_ClientDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_HMI_WIN_CLIENT_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton2();


protected:
	CommToAuto m_Comm;
public:
	CListBox m_MessagesList;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	CButton m_RightButton;
	CButton m_FrontButton;
	CButton m_LeftButton;
	afx_msg void OnNMDblclkDestinations(NMHDR *pNMHDR, LRESULT *pResult);
	CListCtrl m_DestinationList;
	CButton m_StartBtn;
	CButton m_StopBtn;
	afx_msg void OnBnClickedStart();
	afx_msg void OnBnClickedStop();
	afx_msg void OnBnClickedSlowdown();
	CIPAddressCtrl m_IP_Input;
	CEdit m_send_port;
	CEdit m_receive_port;
	CString m_send_port_str;
	CString m_receive_port_str;
	afx_msg void OnBnClickedButton5();
	CString m_ip_address_str;
};

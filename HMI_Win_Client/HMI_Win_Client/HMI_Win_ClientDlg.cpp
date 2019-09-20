
// HMI_Win_ClientDlg.cpp : implementation file
//

#include "stdafx.h"
#include "HMI_Win_Client.h"
#include "HMI_Win_ClientDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif



// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CHMI_Win_ClientDlg dialog



CHMI_Win_ClientDlg::CHMI_Win_ClientDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_HMI_WIN_CLIENT_DIALOG, pParent)
	, m_send_port_str(_T(""))
	, m_receive_port_str(_T(""))
	, m_ip_address_str(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	
}

void CHMI_Win_ClientDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_INFORMATION, m_MessagesList);
	DDX_Control(pDX, IDC_BUTTON1, m_RightButton);
	DDX_Control(pDX, IDC_BUTTON3, m_FrontButton);
	DDX_Control(pDX, IDC_BUTTON2, m_LeftButton);
	DDX_Control(pDX, IDC_DESTINATIONS, m_DestinationList);
	DDX_Control(pDX, IDC_START, m_StartBtn);
	DDX_Control(pDX, IDC_STOP, m_StopBtn);
	DDX_Control(pDX, IDC_EDIT2, m_send_port);
	DDX_Control(pDX, IDC_EDIT1, m_receive_port);
	DDX_Text(pDX, IDC_EDIT2, m_send_port_str);
	DDX_Text(pDX, IDC_EDIT1, m_receive_port_str);
	DDX_Text(pDX, IDC_EDIT3, m_ip_address_str);
}

BEGIN_MESSAGE_MAP(CHMI_Win_ClientDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CHMI_Win_ClientDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON3, &CHMI_Win_ClientDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON2, &CHMI_Win_ClientDlg::OnBnClickedButton2)
	ON_WM_TIMER()
	ON_NOTIFY(NM_DBLCLK, IDC_DESTINATIONS, &CHMI_Win_ClientDlg::OnNMDblclkDestinations)
	ON_BN_CLICKED(IDC_START, &CHMI_Win_ClientDlg::OnBnClickedStart)
	ON_BN_CLICKED(IDC_STOP, &CHMI_Win_ClientDlg::OnBnClickedStop)
	ON_BN_CLICKED(IDC_SLOWDOWN, &CHMI_Win_ClientDlg::OnBnClickedSlowdown)
	ON_BN_CLICKED(IDC_BUTTON5, &CHMI_Win_ClientDlg::OnBnClickedButton5)
END_MESSAGE_MAP()


// CHMI_Win_ClientDlg message handlers

BOOL CHMI_Win_ClientDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
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
	m_Comm.m_pForwardButton = &m_FrontButton;
	m_Comm.m_pLeftButton = &m_LeftButton;
	m_Comm.m_pRightButton = &m_RightButton;
	m_Comm.m_pDestinationList = &m_DestinationList;

	m_Comm.m_pImageList = new CImageList;

	m_Comm.m_pImageList->Create(128, 128, ILC_COLOR32 | ILC_MASK, 1, 1);
	m_Comm.m_pImageList->Add(AfxGetApp()->LoadIcon(IDI_SCHOOL));
	m_Comm.m_pImageList->Add(AfxGetApp()->LoadIcon(IDI_HOME));
	m_Comm.m_pImageList->Add(AfxGetApp()->LoadIcon(IDI_HOSPITAL));
	m_Comm.m_pImageList->Add(AfxGetApp()->LoadIcon(IDI_MALL));
	m_Comm.m_pImageList->Add(AfxGetApp()->LoadIcon(IDI_WORK));
	m_Comm.m_pImageList->Add(AfxGetApp()->LoadIcon(IDI_CLUB));
	
	m_Comm.m_pDestinationList->SetImageList(m_Comm.m_pImageList, LVSIL_NORMAL);

	/*m_StartBtn.SetIcon(::LoadIcon(NULL, "IDI_START"));
	m_StopBtn.SetIcon(::LoadIcon(NULL, "IDI_STOP"));*/

	m_ip_address_str = "192.168.179.9";
	m_send_port_str = "10004";
	m_receive_port_str = "10003";

	UpdateData(false);
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CHMI_Win_ClientDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CHMI_Win_ClientDlg::OnPaint()
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
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CHMI_Win_ClientDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CHMI_Win_ClientDlg::OnBnClickedButton1()
{
	HMI_MSG msg;
	msg.type = COMMAND_MSG;
	msg.options.push_back(RIGHT_TURN_ACTION);
	msg.bErr = 0;

	m_Comm.sendActionPackets(msg);
}


void CHMI_Win_ClientDlg::OnBnClickedButton3()
{
	HMI_MSG msg;
	msg.type = COMMAND_MSG;
	msg.options.push_back(FORWARD_ACTION);
	msg.bErr = 0;

	m_Comm.sendActionPackets(msg);
}


void CHMI_Win_ClientDlg::OnBnClickedButton2()
{
	HMI_MSG msg;
	msg.type = COMMAND_MSG;
	msg.options.push_back(LEFT_TURN_ACTION);
	msg.bErr = 0;

	m_Comm.sendActionPackets(msg);
}


void CHMI_Win_ClientDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default

	CDialogEx::OnTimer(nIDEvent);
}


void CHMI_Win_ClientDlg::OnNMDblclkDestinations(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	
	HMI_MSG msg;
	msg.type = COMMAND_MSG;
	msg.bErr = 0;
	msg.next_destination_id = m_DestinationList.GetSelectionMark();
	msg.options.push_back(CHANGE_DESTINATION);

	m_Comm.sendActionPackets(msg);

	*pResult = 0;
}


void CHMI_Win_ClientDlg::OnBnClickedStart()
{
	HMI_MSG msg;
	msg.type = COMMAND_MSG;
	msg.options.push_back(START_ACTION);
	msg.bErr = 0;

	m_Comm.sendActionPackets(msg);
}


void CHMI_Win_ClientDlg::OnBnClickedStop()
{
	HMI_MSG msg;
	msg.type = COMMAND_MSG;
	msg.options.push_back(STOP_ACTION);
	msg.bErr = 0;

	m_Comm.sendActionPackets(msg);
}


void CHMI_Win_ClientDlg::OnBnClickedSlowdown()
{
	HMI_MSG msg;
	msg.type = COMMAND_MSG;
	msg.options.push_back(SLOWDOWN_ACTION);
	msg.bErr = 0;

	m_Comm.sendActionPackets(msg);
}


void CHMI_Win_ClientDlg::OnBnClickedButton5()
{
	UpdateData(true);
	m_Comm.InitClient(m_ip_address_str.GetString(), m_send_port_str.GetString(), m_receive_port_str.GetString(), &m_MessagesList);
}


// YJHuangDlg.cpp : 實作檔
//

#include "stdafx.h"
#include "SHHuang.h"
#include "SHHuangDlg.h"
#include "CvvImage.h"
#include "Serial.h"
#include "Voronoi.cpp"
#include <opencv2/opencv.hpp> 

using namespace std;

#define WNU_THREAD_EXIT (WM_USER + 1)

VideoCapture L_Cam;
VideoCapture R_Cam;
CSerial Connect_I90;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// 對 App About 使用 CAboutDlg 對話方塊

//-----------初始化靜態變數-----------------
bool SHHuangDlg::carFLAG = true;
bool SHHuangDlg::check_updatedata = true;
bool SHHuangDlg::stop_everthing = true;
vector<CvPoint2D64f> SHHuangDlg::Path;
vector<Keep_Feature> SHHuangDlg::draw_feature;
vector <vector<Keep_Feature>> SHHuangDlg::feature_path;
double SHHuangDlg::scale;
double SHHuangDlg::initial_scale;
double  SHHuangDlg::Camera[6] = { 0 };
double SHHuangDlg::map_move[6] = { 0 };
double  SHHuangDlg::target_pos[3] = { 0 };
bool rotation_AMonte_Carlo = false;
bool SHHuangDlg::Click_Left_Button = false;
double SHHuangDlg::vr = 0, SHHuangDlg::vl = 0;
double SHHuangDlg::car_x = 0, SHHuangDlg::car_y = 0, SHHuangDlg::car_zdir = 0;
double  SHHuangDlg::vr_draw = 0, SHHuangDlg::vl_draw = 0;
vector <CPoint>  SHHuangDlg::jump_path_optimization;
vector <CPoint>  SHHuangDlg::jump_path_optimization_copy;
vector <draw_car> SHHuangDlg::local_draw_car;
vector <draw_car> SHHuangDlg::static_draw_car;
CPoint SHHuangDlg::jump_path_optimization_save;
CvPoint2D64f SHHuangDlg::orgin;
PoseData CPSocket::m_pose_data_2;
float CRSocket::m_lrf_data_2[SIZE_LRF_DATA];
int SHHuangDlg::sampleTime = 0;
int SHHuangDlg::jump_num = 30000;  //隨意給的
int SHHuangDlg::path_optimization_size_change;
int SHHuangDlg::state = 1;
int SHHuangDlg::d[2000];       // 紀錄起點到各個點的最短路徑長度
int SHHuangDlg::parent[2000];  // 紀錄各個點在最短路徑樹上的父親是誰
bool SHHuangDlg::visit[2000];  // 紀錄各個點是不是已在最短路徑樹之中
int SHHuangDlg::w[2000][2000];    // 一張有權重的圖
double SHHuangDlg::IDC_Map_Rect_Map[4];
vector <int> SHHuangDlg::show_path;
IplImage * SHHuangDlg::Read_LRF_Map_static = NULL;
IplImage * SHHuangDlg::Draw_Path_static = NULL;
CPoint SHHuangDlg::Wheel_mouse_pos;
CPoint SHHuangDlg::LButtonDown_mouse_pos;

int c_Synchronous = 51;  //I90補正
double Magnification = 1.2; //放大倍率
int sleep_time = 0;



//-----------初始化靜態變數-----------------

char I90_PWM_control[15] = {
	94,    //0  STX  0x5e|0x02
	2,		//1  STX
	1,		//2  RID
	0,		//3  Reserved
	5,		//4  DID  5的話是各輪子PWM控制
	6,		 //5  Length
	1,		//6   選擇輪子1(右輪)
	160,		//7   Low_8bit
	15,	//8   High_8bit(64就是16384，為中間值，右輪比16384小為前進)
	0,		 //9  選擇輪子0(左輪)
	160,		//10  Low_8bit
	15,	//11  High_8bit(64就是16384，為中間值，左輪比16384大為前進)
	0,		//12  Checksum
	94,	//13  ETX 0x5E|0X0D
	13 }; //14

struct THREAD_INFO_car_draw
{
	bool * Continue;//是否繼續執行
	HWND hWnd;//產生執行續的視窗物件
}Thread_Info_car_draw;

struct THREAD_INFO_TARGET_control
{
	bool * Continue;//是否繼續執行
	HWND hWnd;//產生執行續的視窗物件
}Thread_Info_TARGET_control;

struct THREAD_INFO_Path_Planning
{
	bool * Continue;//是否繼續執行
	HWND hWnd;//產生執行續的視窗物件
}Thread_Info_Path_Planning;

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

	// 對話方塊資料
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支援

														// 程式碼實作
protected:
	DECLARE_MESSAGE_MAP()
};

CPSocket::CPSocket()
{

	memset(&m_pose_data, 0, sizeof(PoseData));
}

CPSocket::~CPSocket() {}

CCRLSocket::CCRLSocket()
{
	int buffer_size = 1024 * 1024 * 16;
	SetSockOpt(SO_RCVBUF, &buffer_size, sizeof(int));
}


CCRLSocket::~CCRLSocket()
{
}

CMSocket::CMSocket()
{
	m_map_data = new MapData;
	int buffer_size = 1024 * 1024 * 16;
	SetSockOpt(SO_RCVBUF, &buffer_size, sizeof(int));
}

CRSocket::CRSocket() {}


CRSocket::~CRSocket() {}

CMSocket::~CMSocket()
{
	delete m_map_data;
}

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

// CYJHuangDlg 對話方塊

SHHuangDlg::SHHuangDlg(CWnd* pParent /*=NULL*/)
	: CDialog(SHHuangDlg::IDD, pParent)
	, m_SampleTime(0)
	, m_Time1(0)
	, m_Time2(0)
	, m_Time3(0)
	, m_Time4(0)
	, m_cameraSitaX(0)
	, m_cameraSitaY(0)
	, m_cameraSitaZ(0)
	, m_cameraX(0)
	, m_cameraY(0)
	, m_cameraZ(0)
	, m_Frequency(0)
	, m_Frequency2(0)
	, m_FeatureNum(0)
	, m_X_shift(0)
	, m_Y_shift(0)
	, m_Z_shift(0)
	, m_socket_map_port(0)
	, m_socket_lrf_port(0)
	, m_socket_pose_port(0)
	, fg_cmd_connected(0)
	, fg_map_connected(0)
	, fg_pose_connected(0)
	, fg_lrf_connected(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void SHHuangDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_L_SelectCamera, m_L_SelectCamera_c);
	DDX_Control(pDX, IDC_R_SelectCamera, m_R_SelectCamera_c);
	DDX_Control(pDX, IDC_L_Image_Live, m_L_Image_Live_c);
	DDX_Control(pDX, IDC_R_Image_Live, m_R_Image_Live_c);
	DDX_Control(pDX, IDC_L_Image_SURF, m_L_Image_SURF_c);
	DDX_Control(pDX, IDC_R_Image_SURF, m_R_Image_SURF_c);
	DDX_Control(pDX, IDC_SaveImage_Mode, m_SaveImage_Mode_c);
	DDX_Control(pDX, IDC_Start, m_Start_c);
	DDX_Control(pDX, IDC_Pause, m_Pause_c);
	DDX_Control(pDX, IDC_Stop, m_Stop_c);
	DDX_Control(pDX, IDC_Continue, m_Continue_c);
	DDX_Control(pDX, IDC_LoadImage_Mode, m_LoadImage_Mode_c);
	DDX_Text(pDX, IDC_SampleTime, m_SampleTime);
	DDX_Text(pDX, IDC_Frequency, m_Frequency);
	DDX_Text(pDX, IDC_Frequency2, m_Frequency2);
	DDX_Text(pDX, IDC_Time1, m_Time1);
	DDX_Text(pDX, IDC_Time2, m_Time2);
	DDX_Text(pDX, IDC_Time3, m_Time3);
	DDX_Text(pDX, IDC_Time4, m_Time4);
	DDX_Text(pDX, IDC_Time5, m_cameraSitaX);
	DDX_Text(pDX, IDC_Time6, m_cameraSitaY);
	DDX_Text(pDX, IDC_Time7, m_cameraSitaZ);
	DDX_Text(pDX, IDC_Time8, m_cameraX);
	DDX_Text(pDX, IDC_Time9, m_cameraY);
	DDX_Text(pDX, IDC_Time10, m_cameraZ);
	DDX_Control(pDX, IDC_Image_PhotoCount, m_Image_PhotoCount_c);
	DDX_Control(pDX, IDC_Map, m_Map_c);
	DDX_Control(pDX, IDC_OnLine_Mode, m_OnLine_Mode_c);
	DDX_Control(pDX, IDC_L_InitialCCD, m_L_InitialCCD_c);
	DDX_Control(pDX, IDC_R_InitialCCD, m_R_InitialCCD_c);
	DDX_Control(pDX, IDC_L_OptionCCD, m_L_OptionCCD_c);
	DDX_Control(pDX, IDC_R_OptionCCD, m_R_OptionCCD_c);
	DDX_Control(pDX, IDC_L_CloseCCD, m_L_CloseCCD_c);
	DDX_Control(pDX, IDC_R_CloseCCD, m_R_CloseCCD_c);
	DDX_Control(pDX, IDC_SOCKET_IP, m_socket_ip_c);
	DDX_Control(pDX, IDC_SOCKET_LOG, m_socket_log_c);
	DDX_Control(pDX, IDC_SOCKET_CONNECT, m_socket_connect_c);
	DDX_Control(pDX, IDC_BUTTON_ReadFeature, m_read_map_c);
	DDX_Control(pDX, IDC_CHECK_Follow, m_FollowMode_c);
	DDX_Control(pDX, IDC_LoadImage_Mode_Laser, m_LoadImage_Mode_Laser_c);
	DDX_Control(pDX, IDC_BUTTON_PathPlanning, m_PathPlanning_c);
	DDX_Text(pDX, IDC_SOCKET_PORT, m_socket_map_port);
	DDX_Text(pDX, IDC_SOCKET_PORT2, m_socket_lrf_port);
	DDX_Text(pDX, IDC_SOCKET_PORT3, m_socket_pose_port);
}

BEGIN_MESSAGE_MAP(SHHuangDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_L_InitialCCD, &SHHuangDlg::OnBnClickedLInitialccd)
	ON_BN_CLICKED(IDC_L_OptionCCD, &SHHuangDlg::OnBnClickedLOptionccd)
	ON_BN_CLICKED(IDC_L_CloseCCD, &SHHuangDlg::OnBnClickedLCloseccd)
	ON_BN_CLICKED(IDC_R_InitialCCD, &SHHuangDlg::OnBnClickedRInitialccd)
	ON_BN_CLICKED(IDC_R_OptionCCD, &SHHuangDlg::OnBnClickedROptionccd)
	ON_BN_CLICKED(IDC_R_CloseCCD, &SHHuangDlg::OnBnClickedRCloseccd)
	ON_BN_CLICKED(IDC_Start, &SHHuangDlg::OnBnClickedStart)
	ON_BN_CLICKED(IDC_Pause, &SHHuangDlg::OnBnClickedPause)
	ON_BN_CLICKED(IDC_Stop, &SHHuangDlg::OnBnClickedStop)
	ON_BN_CLICKED(IDC_Continue, &SHHuangDlg::OnBnClickedContinue)
	ON_BN_CLICKED(IDC_BUTTON1, &SHHuangDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON_ReadFeature, &SHHuangDlg::OnBnClickedButtonReadfeature)
	ON_BN_CLICKED(IDC_BUTTON_big, &SHHuangDlg::OnBnClickedButtonbig)
	ON_BN_CLICKED(IDC_BUTTON_small, &SHHuangDlg::OnBnClickedButtonsmall)
	ON_BN_CLICKED(IDC_BUTTON_UP, &SHHuangDlg::OnBnClickedButtonUp)
	ON_BN_CLICKED(IDC_BUTTON_DOWN, &SHHuangDlg::OnBnClickedButtonDown)
	ON_BN_CLICKED(IDC_BUTTON_LEFT, &SHHuangDlg::OnBnClickedButtonLeft)
	ON_BN_CLICKED(IDC_BUTTON_RIGHT, &SHHuangDlg::OnBnClickedButtonRight)
	ON_BN_CLICKED(IDC_BUTTON_Reset, &SHHuangDlg::OnBnClickedButtonReset)
	ON_BN_CLICKED(IDC_SOCKET_CONNECT, &SHHuangDlg::OnBnClickedSocketConnect)
	ON_BN_CLICKED(IDC_BUTTON_Connect_I90, &SHHuangDlg::OnBnClickedButtonConnectI90)
	ON_WM_KEYDOWN()
	ON_WM_MOUSEWHEEL()
	ON_WM_MOUSEMOVE()
	ON_BN_CLICKED(IDC_BUTTON_Simulation, &SHHuangDlg::OnBnClickedButtonSimulation)
	ON_WM_LBUTTONDOWN()
	ON_BN_CLICKED(IDC_BUTTON_PathPlanning, &SHHuangDlg::OnBnClickedButtonPathplanning)
END_MESSAGE_MAP()


// CYJHuangDlg 訊息處理常式

BOOL SHHuangDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// 將 [關於...] 功能表加入系統功能表。

	// IDM_ABOUTBOX 必須在系統命令範圍之中。
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

	// 設定此對話方塊的圖示。當應用程式的主視窗不是對話方塊時，
	// 框架會自動從事此作業
	SetIcon(m_hIcon, TRUE);			// 設定大圖示
	SetIcon(m_hIcon, FALSE);		// 設定小圖示

									// TODO: 在此加入額外的初始設定

	m_LoadImage_Mode_c.SetCheck(1);
	m_OnLine_Mode_c.SetCheck(0);
	m_SaveImage_Mode_c.SetCheck(0);


	m_L_SelectCamera_c.EnableWindow(1);
	m_L_InitialCCD_c.EnableWindow(1);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(1);
	m_R_InitialCCD_c.EnableWindow(1);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(1);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(1);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);


	IDC_Map_Rect_Map[0] = 687;  //左上點位置x
	IDC_Map_Rect_Map[1] = 215;  //左上點位置y
	IDC_Map_Rect_Map[2] = 600;  //水平範圍
	IDC_Map_Rect_Map[3] = 600;  //垂直範圍

								//影像貼圖視窗位置及大小設定
	m_L_Image_Live_c.SetWindowPos(&wndTop, 13, 90, 320, 240, SWP_SHOWWINDOW);
	m_L_Image_SURF_c.SetWindowPos(&wndTop, 13, 350, 320, 240, SWP_SHOWWINDOW);
	m_R_Image_Live_c.SetWindowPos(&wndTop, 350, 90, 320, 240, SWP_SHOWWINDOW);
	m_R_Image_SURF_c.SetWindowPos(&wndTop, 350, 350, 320, 240, SWP_SHOWWINDOW);
	//	m_Image_PhotoCount_c.SetWindowPos(&wndTop, 687, 190, 32, 16, SWP_SHOWWINDOW);
	m_Map_c.SetWindowPos(&wndTop, IDC_Map_Rect_Map[0], IDC_Map_Rect_Map[1], IDC_Map_Rect_Map[2], IDC_Map_Rect_Map[3], SWP_SHOWWINDOW);

	pWnd_map = GetDlgItem(IDC_Map);
	pWnd_map->GetWindowRect(rect_map);
	//*****************************************L_CCD選擇*******************************************************
	char L_sName[100];
	char L_CameraName[100];

	for (int i = 0; i < L_Cam.GetCaptureCount(); i++)
	{
		L_Cam.GetDeviceName(i, L_sName, 100);
		sprintf_s(L_CameraName, "%d %s", i, L_sName);
		m_L_SelectCamera_c.AddString((CString)L_CameraName);
	}

	m_L_SelectCamera_c.SetCurSel(0);
	//******************************************************************************************************



	//*****************************************R_CCD選擇*******************************************************
	char R_sName[100];
	char R_CameraName[100];

	for (int i = 0; i < R_Cam.GetCaptureCount(); i++)
	{
		R_Cam.GetDeviceName(i, R_sName, 100);
		sprintf_s(R_CameraName, "%d %s", i, R_sName);
		m_R_SelectCamera_c.AddString((CString)R_CameraName);
	}

	m_R_SelectCamera_c.SetCurSel(0);
	//******************************************************************************************************


	L_InitialCCD = false;
	R_InitialCCD = false;

	PhotoCount = 0;

	Continue_car_draw = TRUE;
	Thread_Info_car_draw.hWnd = m_hWnd;
	Thread_Info_car_draw.Continue = &Continue_car_draw;
	m_pThread_car_draw = AfxBeginThread(ThreadFun_car_draw, (LPVOID)&Thread_Info_car_draw);

	scale = 50;
	initial_scale = 50;

	AfxSocketInit();
	// Initial the Map Socket port
	m_socket_map.registerParent(this);
	m_socket_lrf.registerParent(this);
	m_socket_cmd.registerParent(this);
	m_socket_pose.registerParent(this);

	m_socket_ip_c.SetAddress(192, 168, 1, 210);

	// Setup the init Port for each socket
	m_socket_map_port = 25651;
	m_socket_lrf_port = 25650;
	m_socket_pose_port = 25652;

	UpdateData(false);

	return TRUE;  // 傳回 TRUE，除非您對控制項設定焦點
}

void SHHuangDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

// 如果將最小化按鈕加入您的對話方塊，您需要下列的程式碼，
// 以便繪製圖示。對於使用文件/檢視模式的 MFC 應用程式，
// 框架會自動完成此作業。

void SHHuangDlg::OnPaint() //地圖大小在txt檔
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 繪製的裝置內容

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 將圖示置中於用戶端矩形
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 描繪圖示
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}

}


// 當使用者拖曳最小化視窗時，
// 系統呼叫這個功能取得游標顯示。
HCURSOR SHHuangDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CALLBACK TimeProc(UINT uTimerID, UINT uMsg, DWORD dwUser, DWORD dw1, DWORD dw2)
{
	SHHuangDlg* pointer = (SHHuangDlg*)dwUser;
	pointer->DoEvent(); //要重複執行的函式
}

UINT SHHuangDlg::ThreadFun_car_draw(LPVOID lParam)
{
	THREAD_INFO_car_draw* Thread_Info = (THREAD_INFO_car_draw *)lParam;
	SHHuangDlg * hWnd = (SHHuangDlg *)CWnd::FromHandle((HWND)Thread_Info->hWnd);

	CWnd* pWnd_IDC_Map = (CWnd *)hWnd->GetDlgItem(IDC_Map);
	CRect Rect_Map;
	pWnd_IDC_Map->GetWindowRect(Rect_Map);
	CDC* dc = pWnd_IDC_Map->GetWindowDC();
	CString str_Value;
	CStatic * Static_num = (CStatic *)hWnd->GetDlgItem(IDC_Draw_Time);

	CvRect Rect1;
	orgin.x = IDC_Map_Rect_Map[2] / 2;
	orgin.y = IDC_Map_Rect_Map[3] / 2;
	int i = 0;
	int match_size = 0;
	int k = 0;
	int j = 0;
	int P1X, P1Y;
	int P2X, P2Y;
	char xx[100];
	float hessian_min = 1000000;
	LARGE_INTEGER tStart, tEnd, ts;
	CvFont Font1 = cvFont(1, 1);
	CvFont Font2 = cvFont(0.9, 1);
	IplImage * draw_data = NULL;
	draw_data = cvCreateImage(cvSize(Rect_Map.right - Rect_Map.left, Rect_Map.bottom - Rect_Map.top), IPL_DEPTH_8U, 3);
	CvPoint draw_car[3];
	vector <vector<Keep_Feature>> feature_path_temp;

	cv::Rect rect;
	IplImage * draw_data2 = NULL;
	draw_data2 = cvCreateImage(cvSize(4000, 4000), IPL_DEPTH_8U, 3);



	if (!Path.empty())
	{
		map_move[4] = cvRound(IDC_Map_Rect_Map[2] / 2 + Path[Path.size() - 1].x  *initial_scale);
		map_move[5] = cvRound(IDC_Map_Rect_Map[3] / 2 - Path[Path.size() - 1].y * initial_scale);
	}
	else
	{
		map_move[4] = -116;
		map_move[5] = -46;
		// 		map_move[4] = -0;
		// 		map_move[5] = -0;
	}

	while (1)
	{
		QueryPerformanceFrequency(&ts);
		QueryPerformanceCounter(&tStart);

		if (Draw_Path_static)
		{
			cvResize(Draw_Path_static, draw_data);  //利用OPENCV ROI取一個範圍的像，達到縮放的效果。
													// 			Rect1 = cvRect(100, 100, 100, 100);
													// 			cvSetImageROI(draw_data, Rect1);

			int half_image_height = (draw_data2->height - draw_data->height) / 2;

			for (int i = 0; i < draw_data->height; i++)
			{
				for (int j = 0; j < draw_data->width; j++)
				{

					//			cvSet2D(draw_data2, i, j, cvGet2D(draw_data, i, j));
					((uchar *)(draw_data2->imageData + (i + half_image_height)*draw_data2->widthStep))[(j + half_image_height)*draw_data2->nChannels + 0] = ((uchar *)(draw_data->imageData + i*draw_data->widthStep))[j*draw_data2->nChannels + 0];   //直接訪問，較快
					((uchar *)(draw_data2->imageData + (i + half_image_height)*draw_data2->widthStep))[(j + half_image_height)*draw_data2->nChannels + 1] = ((uchar *)(draw_data->imageData + i*draw_data->widthStep))[j*draw_data2->nChannels + 1];
					((uchar *)(draw_data2->imageData + (i + half_image_height)*draw_data2->widthStep))[(j + half_image_height)*draw_data2->nChannels + 2] = ((uchar *)(draw_data->imageData + i*draw_data->widthStep))[j*draw_data2->nChannels + 2];

				}
			}


			cv::Mat srcMat(draw_data2);
			cv::Mat srcMat2;

			rect.width = draw_data->width / scale * initial_scale;
			rect.height = draw_data->height / scale * initial_scale;
			rect.x = half_image_height - (rect.width - draw_data->width) / 2 - map_move[2] + map_move[4];
			rect.y = half_image_height - (rect.width - draw_data->width) / 2 - map_move[3] + map_move[5];


			if (draw_data2->width - rect.width - rect.x < 1 || draw_data2->height - rect.height - rect.y < 1) //還要再改，縮放錯誤
			{
				scale = scale * Magnification;

				rect.width = draw_data->width / scale * initial_scale;
				rect.height = draw_data->height / scale * initial_scale;
				rect.x = half_image_height - (rect.width - draw_data->width) / 2 - map_move[2] + map_move[4];
				rect.y = half_image_height - (rect.width - draw_data->width) / 2 - map_move[3] + map_move[5];
			}

			srcMat(rect).copyTo(srcMat2);
			IplImage iplimage = srcMat2;
			cvResize(&iplimage, draw_data);

			//			cvResize(draw_data2, draw_data);
			//			cvReleaseImage(&draw_data2);

			memset((unsigned char*)draw_data2->imageData, 0, draw_data2->imageSize);
		}

		cvLine(draw_data, cvPoint(orgin.x + map_move[0], -100 * scale + map_move[1]), cvPoint(orgin.x + map_move[0], Rect_Map.bottom + 100 * scale + map_move[1]), CV_RGB(125, 125, 125), 1);
		cvLine(draw_data, cvPoint(-100 * scale + map_move[0], orgin.y + map_move[1]), cvPoint(Rect_Map.right + 100 * scale + map_move[0], orgin.y + map_move[1]), CV_RGB(125, 125, 125), 1);

		if (Path.size() > 1)
		{

			for (i = 1; i < Path.size(); i++)  //畫路徑
			{
				int x1 = cvRound(orgin.x + Path[i - 1].x  * scale + map_move[0]);
				int y1 = cvRound(orgin.y - Path[i - 1].y * scale + map_move[1]);
				int x2 = cvRound(orgin.x + Path[i].x  * scale + map_move[0]);
				int y2 = cvRound(orgin.y - Path[i].y  * scale + map_move[1]);
				cvLine(draw_data, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(0, 255, 0), 1);
			}

			draw_car[0].x = cvRound(orgin.x + Path[i - 1].x  * scale + 9 * sin(-Camera[5])*scale / initial_scale) + map_move[0];
			draw_car[0].y = cvRound(orgin.y - Path[i - 1].y  * scale - 9 * cos(-Camera[5])*scale / initial_scale) + map_move[1];
			draw_car[1].x = cvRound(orgin.x + Path[i - 1].x  * scale - 5 * cos(-Camera[5])*scale / initial_scale - 5 * sin(-Camera[5])*scale / initial_scale) + map_move[0];
			draw_car[1].y = cvRound(orgin.y - Path[i - 1].y  * scale - 5 * sin(-Camera[5])*scale / initial_scale + 5 * cos(-Camera[5])*scale / initial_scale) + map_move[1];
			draw_car[2].x = cvRound(orgin.x + Path[i - 1].x  * scale + 5 * cos(-Camera[5])*scale / initial_scale - 5 * sin(-Camera[5])*scale / initial_scale) + map_move[0];
			draw_car[2].y = cvRound(orgin.y - Path[i - 1].y  * scale + 5 * sin(-Camera[5])*scale / initial_scale + 5 * cos(-Camera[5])*scale / initial_scale) + map_move[1];
			cvFillConvexPoly(draw_data, draw_car, 3, CV_RGB(255, 255, 0), CV_AA, 0);  //畫三角形車車

			if (feature_path.size() > 30)
			{
				feature_path_temp.assign(feature_path.end() - 20, feature_path.end());
				feature_path.clear();
				feature_path.reserve(30);
				feature_path.assign(feature_path_temp.begin(), feature_path_temp.end());
				feature_path_temp.clear();
			}

			for (k = feature_path.size() - 15; k < feature_path.size(); k++)  //畫特徵點軌跡
			{
				for (j = 0; j < feature_path[k].size(); j++)
				{
					P1X = cvRound(orgin.x + feature_path[k][j].X * scale + map_move[0]);
					P1Y = cvRound(orgin.y - feature_path[k][j].Y * scale + map_move[1]);

					if (k == feature_path.size() - 1)
					{
						float hessian_temp = feature_path[feature_path.size() - 1][j].hessian;
						if (hessian_min > hessian_temp)
							hessian_min = hessian_temp;

						if (!feature_path[feature_path.size() - 1][j].match)
							cvCircle(draw_data, cvPoint(P1X, P1Y), 1, CV_RGB(0, 255, 255), CV_FILLED, CV_FILLED, 0);
						else
						{
							cvCircle(draw_data, cvPoint(P1X, P1Y), 2, CV_RGB(255, 0, 0), CV_FILLED, CV_FILLED, 0);
							match_size++;
							sprintf_s(xx, "%d", feature_path[feature_path.size() - 1][j].num);
							cvPutText(draw_data, xx, cvPoint(P1X + 5, P1Y - 5), &Font2, CV_RGB(200, 255, 255));
						}
					}

					if (!feature_path[k][j].appear || feature_path[k][j].X == 0 || feature_path[k - 1][j].X == 0 || feature_path[k - 1][j].Y == 0 || feature_path[k][j].Y == 0 || !feature_path[k][j].match)
						continue;


					if (feature_path[k][j].num == feature_path[k - 1][j].num)
					{
						P2X = cvRound(orgin.x + feature_path[k - 1][j].X * scale + map_move[0]);
						P2Y = cvRound(orgin.y - feature_path[k - 1][j].Y * scale + map_move[1]);
						cvLine(draw_data, cvPoint(P2X, P2Y), cvPoint(P1X, P1Y), CV_RGB(150, 150, 255), 1);
					}
				}
			}
		}
		else
		{
			draw_car[0].x = cvRound(orgin.x) + map_move[0];
			draw_car[0].y = cvRound(orgin.y - 9) + map_move[1];
			draw_car[1].x = cvRound(orgin.x - 5) + map_move[0];
			draw_car[1].y = cvRound(orgin.y + 5) + map_move[1];
			draw_car[2].x = cvRound(orgin.x + 5) + map_move[0];
			draw_car[2].y = cvRound(orgin.y + 5) + map_move[1];
			cvFillConvexPoly(draw_data, draw_car, 3, CV_RGB(255, 255, 0), CV_AA, 0);

			for (j = 0; j < draw_feature.size(); j++)
			{
				P1X = cvRound(orgin.x + draw_feature[j].X * scale + map_move[0]);
				P1Y = cvRound(orgin.y - draw_feature[j].Y * scale + map_move[1]);

				float hessian_temp = draw_feature[j].hessian;
				if (hessian_min > hessian_temp)
					hessian_min = hessian_temp;

				if (!draw_feature[j].match)
					cvCircle(draw_data, cvPoint(P1X, P1Y), 1, CV_RGB(0, 255, 255), CV_FILLED, CV_FILLED, 0);
				else
				{
					cvCircle(draw_data, cvPoint(P1X, P1Y), 2, CV_RGB(255, 0, 0), CV_FILLED, CV_FILLED, 0);
					match_size++;
					sprintf_s(xx, "%d", draw_feature[j].num);
					cvPutText(draw_data, xx, cvPoint(P1X + 5, P1Y - 5), &Font2, CV_RGB(200, 255, 255));

				}
			}

		}

		int x_pos = Wheel_mouse_pos.x - IDC_Map_Rect_Map[0];
		int y_pos = Wheel_mouse_pos.y - IDC_Map_Rect_Map[1];

		if (x_pos > 0 && y_pos > 0 && x_pos < Rect_Map.right - Rect_Map.left && y_pos < Rect_Map.bottom - Rect_Map.top)
		{
			x_pos = (Wheel_mouse_pos.x - IDC_Map_Rect_Map[0]) - orgin.x - map_move[0];
			y_pos = orgin.y - (Wheel_mouse_pos.y - IDC_Map_Rect_Map[1]) + map_move[1];
		}
		else
		{
			x_pos = NULL;
			y_pos = NULL;
		}

		sprintf_s(xx, "(X=%0.2fcm, Y=%0.2fcm, rho=%0.2fcm)", (double)(x_pos*98.2 / scale), (double)(y_pos*98.2 / scale), sqrtf((x_pos*98.2 / scale)*(x_pos*98.2 / scale) + (y_pos*98.2 / scale)*(y_pos*98.2 / scale)));  //BUG暫時拿掉
		cvPutText(draw_data, xx, cvPoint(Rect_Map.Width() - 380, 20), &Font1, CV_RGB(255, 255, 255));

		sprintf_s(xx, "(%d, %d, %0.2f, %0.1f)", draw_feature.size(), match_size, hessian_min, 98.2 / scale);
		cvPutText(draw_data, xx, cvPoint(10, 20), &Font1, CV_RGB(255, 255, 255));

		char yy[30] = "Version V2.1";
		cvPutText(draw_data, yy, cvPoint(5, Rect_Map.bottom - 250), &Font2, CV_RGB(100, 100, 100));

		match_size = 0;

		CvvImage show1;



		show1.CopyOf(draw_data);
		if (check_updatedata)
			show1.Show(*dc, 0, 0, draw_data->width, draw_data->height);

		memset((unsigned char*)draw_data->imageData, 0, draw_data->imageSize);

		Sleep(40 - Path.size() / 500);

		QueryPerformanceCounter(&tEnd);
		float m_total_time = 1000 / ((tEnd.QuadPart - tStart.QuadPart) * 1000 / (double)(ts.QuadPart));
		str_Value.Format(_T("%0.1f"), m_total_time);
		Static_num->SetWindowText(str_Value);

	}
	*Thread_Info->Continue = false;
	::PostMessage(hWnd->m_hWnd, WNU_THREAD_EXIT, 0, 0);
	return(0);
}

UINT SHHuangDlg::ThreadFun_TARGET_control(LPVOID lParam)
{
	bool firstin = 1;
	THREAD_INFO_TARGET_control* Thread_Info = (THREAD_INFO_TARGET_control *)lParam;
	SHHuangDlg * hWnd = (SHHuangDlg *)CWnd::FromHandle((HWND)Thread_Info->hWnd);

	CString str_Value, str_Value2, str_Value3, str_Value4, str_Value5, str_Value6, str_Value7, str_Value8, str_Value9;
	CStatic * Static_num = (CStatic *)hWnd->GetDlgItem(IDC_STATIC_roh);
	CStatic * Static_num2 = (CStatic *)hWnd->GetDlgItem(IDC_STATIC_theta);
	CStatic * Static_num3 = (CStatic *)hWnd->GetDlgItem(IDC_STATIC_angle);
	CStatic * Static_num4 = (CStatic *)hWnd->GetDlgItem(IDC_STATIC_alpha);
	CStatic * Static_num5 = (CStatic *)hWnd->GetDlgItem(IDC_STATIC_beta);
	CStatic * Static_num6 = (CStatic *)hWnd->GetDlgItem(IDC_STATIC_state);
	CStatic * Static_num7 = (CStatic *)hWnd->GetDlgItem(IDC_STATIC_VR);
	CStatic * Static_num8 = (CStatic *)hWnd->GetDlgItem(IDC_STATIC_VL);
	CStatic * Static_num9 = (CStatic *)hWnd->GetDlgItem(IDC_control_Time);

	deque <int> testttte;

	

	LARGE_INTEGER tStart, tEnd, ts;
	target_pos[0] = 0;
	target_pos[1] = 0;
	target_pos[2] = 0;  //旋轉角

						//	int state = 1;
	double rho = 0;
	double theta = 0;
	double alpha = 0;
	double beta = 0;
	double phi = 0;
	double pixel2cm = 98.2 / scale;
	sampleTime = 50; //毫秒

	remove("control_output.txt");
	remove("control_output_m.txt");
	remove("control_pos_m.txt");
	fstream app_control("control_output.txt", ios::app);
	fstream app_control_m("control_output_m.txt", ios::app); //為了給matlab畫圖用的
	fstream app_pos_m("control_pos_m.txt", ios::app);

	while (stop_everthing && !jump_path_optimization_copy.empty())
	{
		QueryPerformanceFrequency(&ts);
		QueryPerformanceCounter(&tStart);

		if (jump_path_optimization_copy.size() < 3)
		{
			float goal = atan2((jump_path_optimization_copy[0].y - jump_path_optimization_copy[1].y), (jump_path_optimization_copy[1].x - jump_path_optimization_copy[0].x));
			target_pos[2] = goal;  //旋轉角
		}
		else
		{
			float goal = atan2((jump_path_optimization_copy[1].y - jump_path_optimization_copy[2].y), (jump_path_optimization_copy[2].x - jump_path_optimization_copy[1].x));
			target_pos[2] = goal;  //旋轉角
		}

							   //		float goal2degree = goal * 180 / CV_PI;

		if (!Path.empty())
		{
			double start_pointx = cvRound(IDC_Map_Rect_Map[2] / 2 + Path[Path.size() - 1].x  *initial_scale) + map_move[4];
			double start_pointy = cvRound(IDC_Map_Rect_Map[3] / 2 - Path[Path.size() - 1].y * initial_scale) + map_move[5];

			car_x = start_pointx - jump_path_optimization_copy[1].x  /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
			car_y = jump_path_optimization_copy[1].y - start_pointy  /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
			car_zdir = Camera[5] + CV_PI / 2 /*+ (double)rand() / (RAND_MAX + 1.0) * 6*/;
		}
		else
		{
			double start_pointx = cvRound(IDC_Map_Rect_Map[2] / 2) + map_move[4];
			double start_pointy = cvRound(IDC_Map_Rect_Map[3] / 2) + map_move[5];

			car_x = start_pointx - jump_path_optimization_copy[1].x  /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
			car_y = jump_path_optimization_copy[1].y - start_pointy   /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
			car_zdir = CPSocket::m_pose_data_2.pose_orientation[2] + CV_PI / 2 /*+ (double)rand() / (RAND_MAX + 1.0) * 6*/;
		}

		// 		car_x = CPSocket::m_pose_data_2.pose_position[0] - jump_path_optimization_copy[1].x /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
		// 		car_y = CPSocket::m_pose_data_2.pose_position[1] - jump_path_optimization_copy[1].y /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
		// 		car_zdir = CPSocket::m_pose_data_2.pose_orientation[2] + CV_PI / 2 /*+ (double)rand() / (RAND_MAX + 1.0) * 6*/;


		phi = car_zdir;
		if (phi > CV_PI)		phi = -2 * CV_PI + phi;
		if (phi < -CV_PI)		phi = 2 * CV_PI + phi;

		rho = sqrt((car_x - target_pos[0])*(car_x - target_pos[0]) + (car_y - target_pos[1])*(car_y - target_pos[1])) *pixel2cm / 100;
		theta = atan2(car_y *pixel2cm - target_pos[1], car_x*pixel2cm - target_pos[0]);


		alpha = -phi + theta + CV_PI;
		if (alpha > CV_PI)		alpha = -2 * CV_PI + alpha;
		if (alpha < -CV_PI)		alpha = 2 * CV_PI + alpha;

		beta = -(theta + CV_PI) + target_pos[2];
		if (beta > CV_PI)		beta = -2 * CV_PI + beta;
		if (beta < -CV_PI)		beta = 2 * CV_PI + beta;

		double theta_de = theta * 180 / CV_PI;
		double alpha_de = alpha * 180 / CV_PI;
		double beta_de = beta * 180 / CV_PI;



		hWnd->Control_Methods(1, rho, alpha, beta, phi, vr, vl, state);


		// 		if (vr > 10)
		// 			vr = 10;
		// 
		// 		if (vl > 10)
		// 			vl = 10;
		// 
		// 
		// 		if (vr < -10)
		// 			vr = -10;
		// 
		// 		if (vl < -10)
		// 			vl = -10;

		vr_draw = vr;
		vl_draw = vl;


		vr = vr * 3400;
		vl = vl * 3400;

		if (vr > 0)
			vr = vr + 8100;
		else
			vr = vr - 8100;

		if (vl > 0)
			vl = vl + 8100;
		else
			vl = vl - 8100;

		app_control_m
			<< setw(12) << setprecision(5) << rho
			<< setw(12) << setprecision(5) << theta
			<< setw(12) << setprecision(5) << phi
			<< setw(12) << setprecision(5) << alpha
			<< setw(12) << setprecision(5) << beta
			<< setw(12) << setprecision(5) << vl
			<< setw(12) << setprecision(5) << vr
			<< setw(12) << setprecision(5) << state
			<< endl;

		app_pos_m
			<< car_zdir << "  "
			<< car_x << "  "
			<< car_y << "  "
			<< endl;



		// 		if (state == 3)
		// 		{
		// 			if (vr > 0)
		// 				vr = vr -150;
		// 			else
		// 				vr = vr + 150;
		// 
		// 			if (vl > 0)
		// 				vl = vl - 150;
		// 			else
		// 				vl = vl + 150;
		// 		}

		I90_PWM_send(vl, vr);

		str_Value.Format(_T("%0.2f"), rho);
		Static_num->SetWindowText(str_Value);

		str_Value2.Format(_T("%0.2f"), theta * 180 / CV_PI);
		Static_num2->SetWindowText(str_Value2);

		str_Value3.Format(_T("%0.2f"), phi * 180 / CV_PI);
		Static_num3->SetWindowText(str_Value3);

		str_Value4.Format(_T("%0.2f"), alpha * 180 / CV_PI);
		Static_num4->SetWindowText(str_Value4);

		str_Value5.Format(_T("%0.2f"), beta * 180 / CV_PI);
		Static_num5->SetWindowText(str_Value5);

		str_Value6.Format(_T("%d"), state);
		Static_num6->SetWindowText(str_Value6);

		str_Value7.Format(_T("%0.2f"), vr);
		Static_num7->SetWindowText(str_Value7);

		str_Value8.Format(_T("%0.2f"), vl);
		Static_num8->SetWindowText(str_Value8);

		cout.flags(ios::left);

		// 		app_control
		// 			<< " |rho= " << setw(7) << setprecision(4) << rho << setw(6)
		// 			<< " |theta= " << setw(7) << setprecision(4) << theta << setw(6)
		// 			<< " |phi= " << setw(7) << setprecision(4) << phi << setw(6)
		// 			<< " |alpha= " << setw(10) << setprecision(4) << alpha << setw(6)
		// 			<< " |beta= " << setw(10) << setprecision(4) << beta << setw(4)
		// 			<< " |vl= " << setw(5) << setprecision(4) << vl << setw(4)
		// 			<< " |vr= " << setw(5) << setprecision(4) << vr
		// 			<< endl;
		//-----------------------------模擬數據----------------------------------------------------------------
		double rho_dot, alpha_dot, beta_dot;
		double x_here, y_here, zdir_here = 0, theta_here;
		double u1, u2;
		double target_pos_sim[3] = { 0 }, car_x_sim, car_y_sim, car_zdir_sim, phi_sim, rho_sim, theta_sim, alpha_sim, beta_sim;
		CPoint erase_path = { 0 };
		vector <double> x_save, y_save;
		int jump_draw = 0;
		//CvPoint draw_car[6];
		int carsize = 20;
		int initial_scale = 100;
		IplImage * draw_data = NULL;
		draw_data = cvCreateImage(cvSize(1440, 900), IPL_DEPTH_8U, 3);
		cvSetZero(draw_data);
		CvPoint draw_oringin[2];

		draw_oringin[0] = cvPoint(orgin.x, orgin.y);
		draw_oringin[1] = cvPoint(orgin.x, 700 - orgin.y);

		vector <CPoint>  jump_path_optimization_copy_sim;
		jump_path_optimization_copy_sim.assign(jump_path_optimization_copy.begin(), jump_path_optimization_copy.end());

		for (int path_num = 0; path_num < jump_path_optimization_copy_sim.size() + 10; path_num++)
		{

			jump_draw = 0;

			if (jump_path_optimization_copy_sim.size() < 2)
				break;
			else if (jump_path_optimization_copy_sim.size() < 3)
			{
				float goal_sim = atan2((jump_path_optimization_copy_sim[0].y - jump_path_optimization_copy_sim[1].y), (jump_path_optimization_copy_sim[1].x - jump_path_optimization_copy_sim[0].x));
				target_pos_sim[2] = goal_sim;  //旋轉角
			}
			else
			{
				float goal_sim = atan2((jump_path_optimization_copy_sim[1].y - jump_path_optimization_copy_sim[2].y), (jump_path_optimization_copy_sim[2].x - jump_path_optimization_copy_sim[1].x));
				target_pos_sim[2] = goal_sim;  //旋轉角
				
			}


			if (path_num > 0)
			{
				car_x_sim = x_here / pixel2cm * 100;
				car_y_sim = y_here / pixel2cm * 100;
				car_zdir_sim = zdir_here;
				target_pos_sim[0] = jump_path_optimization_copy_sim[1].x - jump_path_optimization_copy_sim[0].x + erase_path.x;
				target_pos_sim[1] = jump_path_optimization_copy_sim[0].y - jump_path_optimization_copy_sim[1].y + erase_path.y;

				erase_path.x = target_pos_sim[0] + erase_path.x;
				erase_path.y = target_pos_sim[1] + erase_path.y;
			}
			else
			{
				if (!Path.empty())
				{
					double start_pointx_sim = jump_path_optimization_copy_sim[0].x;
					double start_pointy_sim = jump_path_optimization_copy_sim[0].y;

					car_x_sim = start_pointx_sim - jump_path_optimization_copy_sim[1].x  /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
					car_y_sim = jump_path_optimization_copy_sim[1].y - start_pointy_sim  /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
					car_zdir_sim = CPSocket::m_pose_data_2.pose_orientation[2] + CV_PI / 2 /*+ (double)rand() / (RAND_MAX + 1.0) * 6*/;
				}
				else
				{
					double start_pointx_sim = jump_path_optimization_copy_sim[0].x;
					double start_pointy_sim = jump_path_optimization_copy_sim[0].y;

					// 				target_pos_sim[0] = jump_path_optimization_copy_sim[1].x;
					// 				target_pos_sim[1] = jump_path_optimization_copy_sim[1].y;

					car_x_sim = start_pointx_sim - jump_path_optimization_copy_sim[1].x  /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
					car_y_sim = -start_pointy_sim + jump_path_optimization_copy_sim[1].y /*+ (double)rand() / (RAND_MAX + 1.0) * 4*/;
					car_zdir_sim = CPSocket::m_pose_data_2.pose_orientation[2] + CV_PI / 2 /*+ (double)rand() / (RAND_MAX + 1.0) * 6*/;
				}
			}


			phi_sim = car_zdir_sim;
			if (phi_sim > CV_PI)		phi_sim = -2 * CV_PI + phi_sim;
			if (phi_sim < -CV_PI)		phi_sim = 2 * CV_PI + phi_sim;

			rho_sim = sqrt((car_x_sim - target_pos_sim[0])*(car_x_sim - target_pos_sim[0]) + (car_y_sim - target_pos_sim[1])*(car_y_sim - target_pos_sim[1])) *pixel2cm / 100;
			theta_sim = atan2(car_y_sim - target_pos_sim[1], car_x_sim - target_pos_sim[0]);

			alpha_sim = -phi_sim + theta_sim + CV_PI;
			if (alpha_sim > CV_PI)		alpha_sim = -2 * CV_PI + alpha_sim;
			if (alpha_sim < -CV_PI)		alpha_sim = 2 * CV_PI + alpha_sim;

			beta_sim = -(theta_sim + CV_PI) + target_pos_sim[2];
			if (beta_sim > CV_PI)		beta_sim = -2 * CV_PI + beta_sim;
			if (beta_sim < -CV_PI)		beta_sim = 2 * CV_PI + beta_sim;

			while (true)
			{
				hWnd->Control_Methods(1, rho_sim, alpha_sim, beta_sim, zdir_here, vr, vl, state);

				double degree_alpha = alpha_sim * 180 / CV_PI;
				double degree_beta = beta_sim * 180 / CV_PI;
				double degree_zdir_here = zdir_here * 180 / CV_PI;

				u1 = ((vr + vl) / 2);
				u2 = ((vr - vl) / 0.3);

				rho_dot = u1*(-cos(alpha_sim));
				alpha_dot = (sin(alpha_sim) / rho_sim)*u1 - u2;
				beta_dot = -(sin(alpha_sim) / rho_sim)*u1;

				rho_sim = rho_sim + rho_dot * (float)sampleTime / 1000;
				alpha_sim = alpha_sim + alpha_dot * (float)sampleTime / 1000;
				if (alpha_sim > CV_PI)		alpha_sim = -2 * CV_PI + alpha_sim;
				if (alpha_sim < -CV_PI)		alpha_sim = 2 * CV_PI + alpha_sim;

				beta_sim = beta_sim + beta_dot * (float)sampleTime / 1000;
				if (beta_sim > CV_PI)		beta_sim = -2 * CV_PI + beta_sim;
				if (beta_sim < -CV_PI)		beta_sim = 2 * CV_PI + beta_sim;

				theta_here = -CV_PI - beta_sim + target_pos_sim[2];  //加入 target_pos_sim[2]
				if (theta_here > CV_PI)		theta_here = -2 * CV_PI + theta_here;
				if (theta_here < -CV_PI)		theta_here = 2 * CV_PI + theta_here;

				float here_scale = 100;

				x_here = cos(theta_here) * rho_sim + target_pos_sim[0] * pixel2cm / 100;
				y_here = sin(theta_here) * rho_sim + target_pos_sim[1] * pixel2cm / 100;
				zdir_here = -beta_sim - alpha_sim + target_pos_sim[2];  //加入 target_pos_sim[2]
				jump_draw++;
				x_save.push_back(x_here* here_scale + orgin.x);
				y_save.push_back(700 - (y_here * here_scale + orgin.y));

				draw_car draw_car_temp;

				if (jump_draw % 50 == 1)
				{
					draw_car_temp.car[0] = cvPoint(x_here * here_scale + carsize * cos(zdir_here + 0.7854) + orgin.x, 700 - (y_here * here_scale + carsize * sin(zdir_here + 0.7854) + orgin.y));
					draw_car_temp.car[1] = cvPoint(x_here * here_scale + carsize * cos(0.7854 - zdir_here) + orgin.x, 700 - (y_here * here_scale - carsize * sin(0.7854 - zdir_here) + orgin.y));
					draw_car_temp.car[2] = cvPoint(x_here * here_scale - carsize * cos(zdir_here + 0.7854) + orgin.x, 700 - (y_here * here_scale - carsize * sin(zdir_here + 0.7854) + orgin.y));
					draw_car_temp.car[3] = cvPoint(x_here * here_scale - carsize * cos(0.7854 - zdir_here) + orgin.x, 700 - (y_here * here_scale + carsize * sin(0.7854 - zdir_here) + orgin.y));
					draw_car_temp.car[4] = cvPoint(x_here * here_scale + orgin.x, 700 - (y_here * here_scale + orgin.y));
					draw_car_temp.car[5] = cvPoint(x_here * here_scale + 40 * cos(zdir_here) + orgin.x, 700 - (y_here * here_scale + 40 * sin(zdir_here) + orgin.y));

					local_draw_car.push_back(draw_car_temp);

					cvLine(draw_data, draw_car_temp.car[0], draw_car_temp.car[1], CV_RGB(0, 255, 0), 2);
					cvLine(draw_data, draw_car_temp.car[1], draw_car_temp.car[2], CV_RGB(0, 255, 0), 2);
					cvLine(draw_data, draw_car_temp.car[2], draw_car_temp.car[3], CV_RGB(0, 255, 0), 2);
					cvLine(draw_data, draw_car_temp.car[3], draw_car_temp.car[0], CV_RGB(0, 255, 0), 2);
					cvLine(draw_data, draw_car_temp.car[4], draw_car_temp.car[5], CV_RGB(255, 0, 0), 2);
//					cvLine(draw_data, draw_oringin[0], draw_oringin[1], CV_RGB(255, 100, 255), 2);

				}



				if ((abs(x_here - target_pos_sim[0] * pixel2cm / 100) < 0.02  && abs(y_here - target_pos_sim[1] * pixel2cm / 100) < 0.02) || jump_draw > 600 /*|| zdir_here < 0.2*/)
				{			
					jump_path_optimization_copy_sim.erase(jump_path_optimization_copy_sim.begin(), jump_path_optimization_copy_sim.begin() + 1);
					break;
				}

			}

//			static_draw_car.assign(local_draw_car.begin(), local_draw_car.end());
			local_draw_car.clear();

			for (int i = 0; i < x_save.size() - 1; i++)
			{
				cvLine(draw_data, cvPoint((int)x_save[i], (int)y_save[i]), cvPoint((int)x_save[i + 1], (int)y_save[i + 1]), CV_RGB(150, 150, 255), 2);
			}

		}

		//-----------------------------模擬數據-----------------------------------------------------------------
//		cvNamedWindow("draw_data", CV_WND_PROP_AUTOSIZE);
		cvShowImage("draw_data", draw_data); // 顯示影像於視窗
		cvWaitKey(30); // 停留視窗
		cvReleaseImage(&draw_data);
		Sleep(sampleTime);

		QueryPerformanceCounter(&tEnd);
		float m_total_time = 1000 / ((tEnd.QuadPart - tStart.QuadPart) * 1000 / (double)(ts.QuadPart));
		str_Value9.Format(_T("%0.1f"), m_total_time);
		Static_num9->SetWindowText(str_Value9);
	}




	*Thread_Info->Continue = false;
	::PostMessage(hWnd->m_hWnd, WNU_THREAD_EXIT, 0, 0);
	return(0);
}

UINT SHHuangDlg::ThreadFun_Path_Planning(LPVOID lParam)
{

	// 	CWnd* CW_vo = (CWnd *)GetDlgItem(IDC_STATIC_show2);
	// 	CDC* pDC2 = CW_vo->GetWindowDC();

	THREAD_INFO_Path_Planning* Thread_Info = (THREAD_INFO_Path_Planning *)lParam;
	SHHuangDlg * hWnd = (SHHuangDlg *)CWnd::FromHandle((HWND)Thread_Info->hWnd);

	CString str_Value;
	CStatic * Static_num = (CStatic *)hWnd->GetDlgItem(IDC_Path_Time);
	//	remove("路徑輸出.txt");

	IplConvKernel *pKernel_small = NULL;
	IplConvKernel *pKernel_small2 = NULL;
	pKernel_small = cvCreateStructuringElementEx(11, 11, 6, 6, CV_SHAPE_RECT, NULL);
	vector<vector<bool>>  sca_image;  //縮圖後二值化的結果
	vector <int> path_optimization;
	vector <cv::Point> save_coner;
	vector <CPoint> all_point_map;
	vector <CvPoint2D64f> all_point_map_original;
	int corner_count = 0;
	int line_count = 0;  //有多少VD線段
	int new_input_index = 0;
	bool image_change = 0;
	double Data[8000];
	//	template <double T, size_t N>
	CElement* pElement = 0;
	CvPoint2D32f *point2 = 0;
	CvPoint2D64f savepoint1[3000] = { 0.0 }, savepoint2[3000] = { 0.0 };
	CvPoint2D64f new_savepoint1[3000] = { 0.0 }, new_savepoint2[3000] = { 0.0 };
	LARGE_INTEGER tStart, tEnd, ts;

	IplImage * pGrayImg = NULL;
	IplImage * check_change = NULL;
	IplImage * show_data = NULL; //縮小十倍的矩陣
	IplImage * read_data_old = NULL;
	IplImage * Draw_Path = NULL;

	//	Read_LRF_Map_static = cvLoadImage("path\\L0.png", 0);
	Read_LRF_Map_static = cvLoadImage("20160626_090600/map_20160626_090600.png", 0);
	read_data_old = cvCreateImage(cvGetSize(Read_LRF_Map_static), Read_LRF_Map_static->depth, 1);
	pGrayImg = cvCreateImage(cvGetSize(Read_LRF_Map_static), Read_LRF_Map_static->depth, 1);
	Draw_Path = cvCreateImage(cvGetSize(Read_LRF_Map_static), Read_LRF_Map_static->depth, 3);
	check_change = cvCreateImage(cvGetSize(Read_LRF_Map_static), Read_LRF_Map_static->depth, 1);
	show_data = cvCreateImage(cvSize(60, 60), IPL_DEPTH_8U, 1);
	Draw_Path_static = cvCreateImage(cvGetSize(Read_LRF_Map_static), Read_LRF_Map_static->depth, 3);
	char read_name[100];
	char write_name[100];
	int photo_conunt = 0;
	int jump_conunt = 0;
	CvPoint start_point, end_point;
	// 	start_point.x = 140;  //路徑起始與終點，請參照圖片給定
	// 	start_point.y = 399;
	end_point.x = 0;
	end_point.y = 0;

	bool choose1 = false, choose2 = false, only_first = true;
	while (stop_everthing)
	{

		QueryPerformanceFrequency(&ts);
		QueryPerformanceCounter(&tStart);

		if ((choose1 || choose2) && only_first || Click_Left_Button)
		{
			int x_pos = LButtonDown_mouse_pos.x - IDC_Map_Rect_Map[0];
			int y_pos = LButtonDown_mouse_pos.y - IDC_Map_Rect_Map[1];

			if (x_pos > 0 && y_pos > 0 && x_pos < IDC_Map_Rect_Map[2] && y_pos < IDC_Map_Rect_Map[3] && choose1)
			{
				start_point.x = x_pos;  //路徑起始與終點，請參照圖片給定
				start_point.y = y_pos;
				choose1 = false;
				x_pos = 0;
				y_pos = 0;
				LButtonDown_mouse_pos.x = 0;
				LButtonDown_mouse_pos.y = 0;

				if (!choose2)
					only_first = false;
			}
			if (x_pos > 0 && y_pos > 0 && x_pos < IDC_Map_Rect_Map[2] && y_pos < IDC_Map_Rect_Map[3] && choose2)
			{
				end_point.x = x_pos + map_move[4] - map_move[0];
				end_point.y = y_pos + map_move[5] - map_move[1];
				choose2 = false;
				x_pos = 0;
				y_pos = 0;
				LButtonDown_mouse_pos.x = 0;
				LButtonDown_mouse_pos.y = 0;

				if (!choose1)
					only_first = false;
			}
			Sleep(30);
		}

		if (!Path.empty())
		{
			start_point.x = cvRound(IDC_Map_Rect_Map[2] / 2 + Path[Path.size() - 1].x  *initial_scale) + map_move[4];
			start_point.y = cvRound(IDC_Map_Rect_Map[3] / 2 - Path[Path.size() - 1].y * initial_scale) + map_move[5];
		}
		else
		{
			start_point.x = cvRound(IDC_Map_Rect_Map[2] / 2) + map_move[4];
			start_point.y = cvRound(IDC_Map_Rect_Map[3] / 2) + map_move[5];
		}

		choose1 = false;
		choose2 = true;
		Click_Left_Button = false;


		//		end_point.x -= 10; //模擬移動

		sprintf_s(read_name, "20160626_090600/map_20160626_090600.png", photo_conunt);
		sprintf_s(write_name, "path\\R%d.png", photo_conunt);
		//		sprintf_s(read_name, "path\\L%d.png", photo_conunt);
		fstream in_image0(read_name, ios::in);

		if (!in_image0)
		{
			//			Draw_Path_static = NULL;
			break;
		}

		Read_LRF_Map_static = cvLoadImage(read_name, 0);
		Sleep(40);

		cvAbsDiff(Read_LRF_Map_static, read_data_old, check_change);
		cvResize(Read_LRF_Map_static, read_data_old, CV_INTER_NN);
		cvResize(check_change, show_data, CV_INTER_NN);

		image_change = 0;
		for (int i = 0; i < show_data->height; i++)
		{
			for (int j = 0; j < show_data->width; j++)
			{
				int something = ((uchar *)(show_data->imageData + i*show_data->widthStep))[j];  //直接訪問，較快
																								//				int something = cvGet2D(show_data, i, j).val[0];  //間接訪問，較慢
				if (something != 0)
				{
					image_change = 1;
					break;
				}
			}
			if (image_change)
				break;
			else
				image_change = 0;
		}

		//		jump_conunt++;
		if (jump_conunt == 2)
		{
			photo_conunt++;
			jump_conunt = 0;
			Sleep(1000);
		}
		photo_conunt++;
		if (1)
		{


			//-------------------------清除數據------------------------------
			save_coner.clear();
			sca_image.clear();
			path_optimization.clear();
			all_point_map.clear();
			all_point_map_original.clear();
			show_path.clear();
			memset((unsigned char*)Draw_Path->imageData, 0, Draw_Path->imageSize);
			memset(d, 0, sizeof(d));
			memset(parent, 0, sizeof(parent));
			memset(visit, 0, sizeof(visit));
			memset(w, 0, sizeof(w));

			//---------------------------------------------------------------------


			cvResize(Read_LRF_Map_static, pGrayImg, CV_INTER_NN);//讀黑白影像用的
																 //		cvCvtColor(read_data, pGrayImg, CV_RGB2GRAY);  //讀彩色影像用的
#if 0
			cvErode(pGrayImg, pGrayImg, pKernel_small, 2);  //侵蝕的相反(因為是白底)
			cvDilate(pGrayImg, pGrayImg, pKernel_small, 1);  //膨脹的相反
#else
			cvDilate(pGrayImg, pGrayImg, pKernel_small, 2);  //膨脹
			cvErode(pGrayImg, pGrayImg, pKernel_small, 1);  //侵蝕
#endif			

			cvCvtColor(pGrayImg, Draw_Path, CV_GRAY2RGB);


			//			cvSaveImage("給連通物件用的.bmp", pGrayImg);

			//數值要依據縮小倍率與格點pixel數決定


			cvResize(pGrayImg, show_data, CV_INTER_LANCZOS4);


			//輸入圖片，輸出二值資料
			hWnd->binarization(show_data, sca_image);
			//輸入二值資料，輸出角點
			hWnd->find_coner(sca_image, save_coner, 4);
			//將角點轉換為準備要丟入Voronoi運算的格式
			hWnd->trans2Voronoi(sca_image, save_coner, Data, 2);
			//計算狹義Voronoi，輸入角點資料與邊界，輸出兩個矩陣
			hWnd->Voronoi_calculate(Data, show_data->width, show_data->height, savepoint1, savepoint2, line_count);
			//計算廣義Voronoi，待改
			hWnd->Generalized_Voronoi(sca_image, savepoint1, savepoint2, line_count, new_input_index, new_savepoint1, new_savepoint2);
			//VD點會破碎，將其重新聚合
			hWnd->Match_point(line_count, new_input_index, new_savepoint1, new_savepoint2, 2.5);
			//Dijkstra路徑搜尋，輸入點連接資訊跟數量
			hWnd->Dijkstra_path_planning(start_point, end_point, new_savepoint1, new_savepoint2, new_input_index, all_point_map, all_point_map_original);
			//路徑優化，輸入二值資訊與原本路徑
			hWnd->Path_Optimization(sca_image, all_point_map_original, path_optimization);



			//-------------------------------------------繪圖---------------------------------------

			for (int i = 0; i < save_coner.size(); i++)  //角點圖
			{
				//				cvLine(Draw_Path, cvPoint(save_coner[i].x * 10, save_coner[i].y * 10), cvPoint(save_coner[i].x * 10, save_coner[i].y * 10), CV_RGB(0, 250, 0), 6);
			}
			for (int i = 0; i < line_count; i++)   //VD圖
			{
				//				cvLine(Draw_Path, cvPoint(savepoint1[i].x * 10, savepoint1[i].y * 10), cvPoint(savepoint2[i].x * 10, savepoint2[i].y * 10), CV_RGB(0, 0, 255), 2);
			}

			for (int i = 0; i < new_input_index; i++)  //GVD圖
			{
				cvLine(Draw_Path, cvPoint(new_savepoint1[i].x * 10, new_savepoint1[i].y * 10), cvPoint(new_savepoint2[i].x * 10, new_savepoint2[i].y * 10), CV_RGB(255, 100, 100), 2);
			}

			for (int path_index = 0; path_index < show_path.size() - 1; path_index++) //畫出路徑圖
			{
				cvLine(Draw_Path, cvPoint(all_point_map[show_path[path_index]].x, all_point_map[show_path[path_index]].y), cvPoint(all_point_map[show_path[path_index + 1]].x, all_point_map[show_path[path_index + 1]].y), CV_RGB(0, 0, 255), 2);
			}

			float close_path = 200;
			int jump_num2 = 20000;
			int path_opt = 0;

			for (path_opt = 0; path_opt < path_optimization.size() - 1; path_opt++) //畫出路徑優化圖
			{
				if (path_optimization_size_change != path_optimization.size())
				{
					jump_num = 3000000;
				}

				if (close_path > 16)  //16
				{
					//					first_in = sqrtf(pow((all_point_map[path_optimization[0]].x - all_point_map[path_optimization[1]].x), 2) + pow((all_point_map[path_optimization[0]].y - all_point_map[path_optimization[1]].y), 2)) / 2;
					jump_num2 = path_optimization[path_opt];

					if (jump_num2 != jump_num)
						jump_path_optimization.push_back(all_point_map[path_optimization[path_opt]]);

				}
				else
				{
					jump_num = path_optimization[1];
					//					jump_path_optimization_save = all_point_map[path_optimization[1]];

				}

				close_path = sqrtf(pow((all_point_map[path_optimization[0]].x - all_point_map[path_optimization[path_opt + 1]].x), 2) + pow((all_point_map[path_optimization[0]].y - all_point_map[path_optimization[path_opt + 1]].y), 2));

				cvLine(Draw_Path, cvPoint(all_point_map[path_optimization[path_opt]].x, all_point_map[path_optimization[path_opt]].y), cvPoint(all_point_map[path_optimization[path_opt + 1]].x, all_point_map[path_optimization[path_opt + 1]].y), CV_RGB(0, 255, 0), 1);

				cvCircle(Draw_Path, cvPoint(all_point_map[path_optimization[path_opt]].x, all_point_map[path_optimization[path_opt]].y), 3, CV_RGB(0, 255, 0), 1);

			}

			path_optimization_size_change = path_optimization.size();
			jump_path_optimization.push_back(all_point_map[path_optimization[path_opt]]);

			for (int jump_path_opt = 0; jump_path_opt < jump_path_optimization.size() - 1; jump_path_opt++) //畫出實際要走路線圖
			{

				cvLine(Draw_Path, cvPoint(jump_path_optimization[jump_path_opt].x, jump_path_optimization[jump_path_opt].y), cvPoint(jump_path_optimization[jump_path_opt + 1].x, jump_path_optimization[jump_path_opt + 1].y), CV_RGB(0, 255, 0), 2);

				cvCircle(Draw_Path, cvPoint(all_point_map[path_optimization[path_opt]].x, all_point_map[path_optimization[path_opt]].y), 3, CV_RGB(0, 255, 0), 1);
			}

			jump_path_optimization_copy.assign(jump_path_optimization.begin(), jump_path_optimization.end()); // draw_feature 複制 
			jump_path_optimization.clear();
			cvResize(Draw_Path, Draw_Path_static);

			//			cvSaveImage(write_name, Draw_Path);

			//			m_show2.SetWindowPos(&wndTop, 10, 10, draw_data->width, draw_data->height, SWP_SHOWWINDOW);
			// 			CvvImage show1;
			// 			show1.CopyOf(draw_data);
			// 			show1.Show(*pDC2, 0, 0, draw_data->width, draw_data->height);

			//-------------------------------------------繪圖---------------------------------------

			// 			IplImage * itest2 = NULL;
			// 			IplImage * itest = NULL;
			// 			itest2 = cvCreateImage(cvSize(60, 60), IPL_DEPTH_8U, 1);
			// 			itest = cvCreateImage(cvSize(60, 60), IPL_DEPTH_8U, 3);
			// 			cvResize(Read_LRF_Map_static, itest2, CV_INTER_NN);
			// 			cvCvtColor(itest2, itest, CV_GRAY2RGB);
			// 
			// 			for (int i = 0; i < save_coner.size(); i++)  //角點圖
			// 			{
			// 				cvLine(itest, cvPoint(save_coner[i].x, save_coner[i].y), cvPoint(save_coner[i].x, save_coner[i].y), CV_RGB(0, 250, 250), 1);
			// 			}
			// 			for (int i = 0; i < line_count; i++)   //VD圖
			// 			{
			// 				cvLine(itest, cvPoint(savepoint1[i].x, savepoint1[i].y), cvPoint(savepoint2[i].x, savepoint2[i].y), CV_RGB(250, 200, 100), 1);
			// 			}
			// 			for (int i = 0; i < new_input_index; i++)  //GVD圖
			// 			{
			// 				cvLine(itest, cvPoint(new_savepoint1[i].x, new_savepoint1[i].y), cvPoint(new_savepoint2[i].x, new_savepoint2[i].y), CV_RGB(250, 100, 100), 1);
			// 			}
			// 
			// 			for (int path_index = 0; path_index < show_path.size() - 1; path_index++) //畫出路徑圖
			// 			{
			// 				cvLine(itest, cvPoint(all_point_map[show_path[path_index]].x, all_point_map[show_path[path_index]].y), cvPoint(all_point_map[show_path[path_index + 1]].x, all_point_map[show_path[path_index + 1]].y), CV_RGB(0, 0, 255), 1);
			// 			}
			// 
			// 			for (int path_opt = 0; path_opt < path_optimization.size() - 1; path_opt++) //畫出路徑優化圖
			// 			{
			// 
			// 				cvLine(itest, cvPoint(all_point_map[path_optimization[path_opt]].x, all_point_map[path_optimization[path_opt]].y), cvPoint(all_point_map[path_optimization[path_opt + 1]].x, all_point_map[path_optimization[path_opt + 1]].y), CV_RGB(0, 150, 0), 1);
			// 
			// 				cvCircle(itest, cvPoint(all_point_map[path_optimization[path_opt]].x, all_point_map[path_optimization[path_opt]].y), 7, CV_RGB(0, 150, 0), 1);
			// 
			// 			}

			//			cvSaveImage("輸出.png", itest);
			// 			cvSaveImage("itest.png", itest);
			// 			cvReleaseImage(&itest);

			//-------------------------------------------繪圖---------------------------------------
		}



		memset((unsigned char*)show_data->imageData, 0, show_data->imageSize);
		cvReleaseImage(&Read_LRF_Map_static);



		QueryPerformanceCounter(&tEnd);

		float m_total_time = 1000 / ((tEnd.QuadPart - tStart.QuadPart) * 1000 / (double)(ts.QuadPart));
		str_Value.Format(_T("%0.1f"), m_total_time);
		Static_num->SetWindowText(str_Value);

		//		photo_conunt++;

	}

	*Thread_Info->Continue = false;
	::PostMessage(hWnd->m_hWnd, WNU_THREAD_EXIT, 0, 0);
	return(0);
}

void SHHuangDlg::Control_Methods(bool control_type, double i_rho, double i_alpha, double i_beta, double i_phi, double &o_vr, double &o_vl, int &o_state)
{

	//----------------------第二模式新增之判斷------------------------------------------------------------
	float P2_1 = 1.1737;
	float P2_2 = 1.4317;
	float P2_3 = 0.2422;
	float P2_4 = 0.4223;

	float Vt2 = i_rho * i_rho * P2_1 +
		i_alpha*i_alpha*P2_2 +
		i_alpha*i_phi*P2_4 +
		i_alpha*i_phi*P2_4 +
		i_phi*i_phi*P2_3;

	int rho_gain;
	int times = 0;
	float M1, M2, N1, N2;
	float alpha_gain;
	float beta_gain;
	//-----------------------------------------------------------------------------------------------------


	if (control_type)
	{
		//原始線性控制
		o_vr = 3 * i_rho + 0.15 * (8 * i_alpha - 4 * (i_beta));
		o_vl = 3 * i_rho - 0.15 * (8 * i_alpha - 4 * (i_beta));
		o_vr = o_vr / 10;
		o_vl = o_vl / 10;
	}
	else
	{
		//判斷要切換哪種模式TS-FUZZY
		if (times == 0 || i_alpha > (CV_PI / 10) || i_alpha < (-CV_PI / 10))
		{
			if (o_state == 3)
				o_state = 3;

			else if (o_state == 2)
				o_state = 2;

			else
				o_state = 1;
		}
		else if ((i_rho > 1 || Vt2 > 4))  //gamma = 2
		{
			if (o_state == 3)
				o_state = 3;
			else
				o_state = 2;
		}
		else
			o_state = 3;

		if (i_alpha < CV_PI / 10 && i_alpha>-CV_PI / 10)
			times = 1; //一開始必定Mode1，其餘只看角度來決定是否進入Mode1

					   //切換式TS-Fuzzy控制
		alpha_gain = 1.9161;

		switch (o_state)
		{
		case 1:  //旋轉Mode
				 //		alpha_gain = 10 * (i_alpha / pi);

			o_vr = 0 * i_rho + 0.15 * (alpha_gain*i_alpha - 0 * (i_beta));
			o_vl = 0 * i_rho - 0.15 * (alpha_gain*i_alpha - 0 * (i_beta));
			o_vr = o_vr *1.4;
			o_vl = o_vl *1.4;

			break;

		case 2:  //直線Mode
				 // 			rho_gain = (4 * i_rho / 200) + (1 - (i_rho / 200));
				 // 			if (i_rho > 150) rho_gain = 4;
			rho_gain = 1.0331;
			o_vr = rho_gain *i_rho + 0.15 * (alpha_gain*i_alpha - 0 * (i_beta));
			o_vl = rho_gain *i_rho - 0.15 * (alpha_gain*i_alpha - 0 * (i_beta));
			o_vr = o_vr*0.1;
			o_vl = o_vl*0.1;
			break;

		case 3:  //PDC Mode
				 //			rho_gain = (3 * i_rho / 125) + (1 - (i_rho / 125));
			rho_gain = 1.0331;
			M1 = (cos(i_alpha) - 0.031415926) / (1 - 0.031415926);
			M2 = 1 - M1;
			//		M2 = (1 - cos(i_alpha)) / (1 - 0.031415926);

			if (i_alpha == 0)
				N1 = 1;
			else
				N1 = (0.49*CV_PI * sin(i_alpha) - sin(0.49*CV_PI)*i_alpha) / (i_alpha*(0.49*CV_PI - sin(0.49*CV_PI)));

			N2 = 1 - N1;

			// 		alpha_gain = M1*N1*5.8766 +
			// 			                  M1*N2*5.6385 +
			// 			                  M2*N1*5.8766 +
			// 			                  M2*N2*5.6385;
			// 
			// 		beta_gain = M1*N1*1.1052 +
			// 			                M1*N2*1.0776 +
			// 			                M2*N1*1.1052 +
			// 			                M2*N2*1.0776;

			alpha_gain = M1*N1*1.2833 +
				M1*N2*1.1022 +
				M2*N1* 1.2833 +
				M2*N2*1.1022;

			beta_gain = -M1*N1*0.0487 +
				-M1*N2*0.0517 +
				-M2*N1*0.0487 +
				-M2*N2*0.0517;

			// 			beta_gain = -M1*N1*1.2833 +
			// 				-M1*N2*1.1022 +
			// 				-M2*N1* 1.2833 +
			// 				-M2*N2*1.1022;
			// 
			// 			alpha_gain = M1*N1*0.0487 +
			// 				M1*N2*0.0517 +
			// 				M2*N1*0.0487 +
			// 				M2*N2*0.0517;


			o_vr = rho_gain * i_rho + 0.15 * (alpha_gain * i_alpha - beta_gain * (i_beta));
			o_vl = rho_gain * i_rho - 0.15 * (alpha_gain * i_alpha - beta_gain * (i_beta));
			o_vr = o_vr*0.3;
			o_vl = o_vl*0.3;
			break;

		default:
			break;
		}
	}


}


void SHHuangDlg::DoEvent()
{
	time1 = (double)cvGetTickCount(); //***********************************************************************************

									  //fstream app_Test("Test.txt", ios::app);
									  //app_Test << PhotoCount << endl;

	if (m_LoadImage_Mode_c.GetCheck())
	{
		//		PhotoCount = 500;
		SampleTime = SampleTime_temp[PhotoCount]; // 相片間隔時間

		char path0[100];
		sprintf_s(path0, "photo\\L%d.png", PhotoCount);
		fstream in_image0(path0, ios::in);

		char path1[100];
		sprintf_s(path1, "photo\\R%d.png", PhotoCount);
		fstream in_image1(path1, ios::in);



		if (in_image0 && in_image1)
		{
			time3 = (double)cvGetTickCount(); //***********************************************************************************
			IplImage* l_image = cvLoadImage(path0, CV_LOAD_IMAGE_COLOR);
			IplImage* r_image = cvLoadImage(path1, CV_LOAD_IMAGE_COLOR);
			time4 = (double)cvGetTickCount(); //***********************************************************************************
											  //app_Test << "LoadImage_ok" << endl;
			WORK(l_image, r_image, SampleTime);
			//app_Test << "WORK_ok" << endl;			
			cvReleaseImage(&l_image);
			cvReleaseImage(&r_image);

			CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
			ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);

			PhotoCount++;
		}
		else
		{
			PhotoCount -= 1;

			CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
			ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);

			OnBnClickedStop();

			MessageBox(TEXT("圖片讀取結束!"));
		}

		in_image0.close();
		in_image1.close();


	}
	else if (m_OnLine_Mode_c.GetCheck())
	{
		if (BinocularSLAM.Online_Laser_Localization_enable)
		{
			fstream app_Laser_Data2("Laser_Data.txt", ios::app);
			app_Laser_Data2 << CPSocket::m_pose_data_2.pose_position[0] << " " << CPSocket::m_pose_data_2.pose_position[1] << " " << CPSocket::m_pose_data_2.pose_orientation[2] << " " << CPSocket::m_pose_data_2.pose_cov[0] << " " << CPSocket::m_pose_data_2.pose_cov[7] << " " << CPSocket::m_pose_data_2.pose_cov[35] << endl;
			app_Laser_Data2.close();
		}

		time9 = (double)cvGetTickCount(); //***********************************************************************************

		IplImage* l_image = L_Cam.GetOneImage();
		IplImage* r_image = R_Cam.GetOneImage();

		time5 = (double)cvGetTickCount(); //***********************************************************************************
										  //app_Test << "GetImage_ok" << endl;
		if (m_SaveImage_Mode_c.GetCheck())
		{
			_mkdir("photo"); // 建立資料夾

			char path0[100];
			sprintf_s(path0, "photo\\L%d.png", PhotoCount);
			cvSaveImage(path0, l_image);

			char path1[100];
			sprintf_s(path1, "photo\\R%d.png", PhotoCount);
			cvSaveImage(path1, r_image);

			fstream app_SampleTime("photo\\SampleTime.txt", ios::app);
			app_SampleTime << SampleTime << endl;

			app_SampleTime.close();

			//app_Test << "SaveImage_ok" << endl;
		}

		time6 = (double)cvGetTickCount(); //***********************************************************************************


		WORK(l_image, r_image, SampleTime);
		//app_Test << "WORK_ok" << endl;
		cvReleaseImage(&l_image);
		cvReleaseImage(&r_image);

		CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
		ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);


		PhotoCount++;

	}
	else if (m_LoadImage_Mode_Laser_c.GetCheck())
	{
		SampleTime = SampleTime_temp[PhotoCount]; // 相片間隔時間

		char path0[100];
		sprintf_s(path0, "photo\\L%d.png", PhotoCount);
		fstream in_image0(path0, ios::in);

		char path1[100];
		sprintf_s(path1, "photo\\R%d.png", PhotoCount);
		fstream in_image1(path1, ios::in);

		BinocularSLAM.Offline_Laser_Localization_enable = true;

		if (in_image0 && in_image1)
		{
			time3 = (double)cvGetTickCount(); //***********************************************************************************
			IplImage* l_image = cvLoadImage(path0, CV_LOAD_IMAGE_COLOR);
			IplImage* r_image = cvLoadImage(path1, CV_LOAD_IMAGE_COLOR);
			time4 = (double)cvGetTickCount(); //***********************************************************************************
											  //app_Test << "LoadImage_ok" << endl;
			WORK(l_image, r_image, SampleTime);
			//app_Test << "WORK_ok" << endl;			
			cvReleaseImage(&l_image);
			cvReleaseImage(&r_image);

			CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
			ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);

			PhotoCount++;
		}
		else
		{
			PhotoCount -= 1;

			CWnd* pWnd_Image_PhotoCount = GetDlgItem(IDC_Image_PhotoCount);
			ShowPhotoCount(PhotoCount, pWnd_Image_PhotoCount);

			OnBnClickedStop();

			MessageBox(TEXT("圖片讀取結束!"));
		}

		in_image0.close();
		in_image1.close();
	}









	time2 = (double)cvGetTickCount(); //***********************************************************************************


	m_Time1 = (int)((time2 - time1) / (cvGetTickFrequency()*1000.)); // 系統全部
	m_Frequency2 = cvRound(1. / (((time2 - time1) / (cvGetTickFrequency()*1000.)) / 1000.)); // 系統全部 頻率

	if (m_LoadImage_Mode_c.GetCheck() || m_LoadImage_Mode_Laser_c.GetCheck())
	{
		m_Time2 = 0; // 擷圖
		m_Time3 = (int)((time4 - time3) / (cvGetTickFrequency()*1000.)); // 讀圖
	}
	else if (m_OnLine_Mode_c.GetCheck())
	{
		m_Time2 = (int)((time5 - time9) / (cvGetTickFrequency()*1000.)); // 擷圖
		m_Time3 = 0; // 存讀圖 
		SampleTime = ((time2 - time1) / (cvGetTickFrequency()*1000.)) / 1000.; // 相片間隔時間

		if (m_SaveImage_Mode_c.GetCheck())
		{
			m_Time3 = (int)((time6 - time5) / (cvGetTickFrequency()*1000.)); // 存圖

			fstream app_GetImageTime("photo\\GetImageTime.txt", ios::app);
			app_GetImageTime << ((time5 - time9) / (cvGetTickFrequency()*1000.)) / 1000. << endl;
			app_GetImageTime.close();


			fstream app_SaveImageTime("photo\\SaveImageTime.txt", ios::app);
			app_SaveImageTime << ((time6 - time5) / (cvGetTickFrequency()*1000.)) / 1000. << endl;
			app_SaveImageTime.close();
		}
	}



	m_SampleTime = SampleTime; // 相片間隔時間
	if (SampleTime > 0)   m_Frequency = cvRound(1. / SampleTime); // 相片間隔 頻率

	m_cameraX = BinocularSLAM.p3dx_x.x * 100;
	m_cameraY = BinocularSLAM.p3dx_x.y * 100;
	m_cameraZ = BinocularSLAM.p3dx_x.z * 100;
	m_cameraSitaX = BinocularSLAM.p3dx_x.x_dir * 180 / CV_PI;
	m_cameraSitaY = BinocularSLAM.p3dx_x.y_dir * 180 / CV_PI;
	m_cameraSitaZ = BinocularSLAM.p3dx_x.z_dir * 180 / CV_PI;

	check_updatedata = 0;
	UpdateData(false);
	check_updatedata = 1;

}



void SHHuangDlg::ShowImage(const IplImage* image, CWnd* pWnd)
{

	IplImage* image_show = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);

	if (image->width == 320 && image->height == 240)
	{
		cvCopy(image, image_show); //圖片複製
	}
	else
	{
		cvResize(image, image_show, CV_INTER_LINEAR); //圖片縮放
	}


	CDC* dc = pWnd->GetWindowDC();
	CvvImage show;
	//CImage show;	
	show.CopyOf(image_show);
	show.Show(*dc, 0, 0, image_show->width, image_show->height);
	cvReleaseImage(&image_show);
	ReleaseDC(dc);

}


//****************************************************************************************************************************************SLAM

void SHHuangDlg::WORK(const IplImage* l_image, const IplImage* r_image, const double SampleTime)
{
	//	fstream app_p3dx("isruning.txt", ios::app);

	BinocularSLAM.Run_BinocularSLAM(l_image, r_image, SampleTime);

	Sleep(sleep_time);
	//	InvalidateRect(rect_map); // 小地圖更新  (20160620拿掉，不需要了)
	save_car_pos();  //20160620 改版繪圖

	IplImage* l_image_color = cvCreateImage(cvGetSize(l_image), IPL_DEPTH_8U, 3);
	cvCopy(l_image, l_image_color);

	IplImage* r_image_color = cvCreateImage(cvGetSize(r_image), IPL_DEPTH_8U, 3);
	cvCopy(r_image, r_image_color);

	//----------------------------------------------------------------------------------------------------keyfeature
	for (int i = 0; i < (int)BinocularSLAM.map_feature.size(); i++) // 畫出顯示影像上地圖特徵點
	{
		if (!BinocularSLAM.map_feature[i].match) continue;

		int l_ix = cvRound(BinocularSLAM.map_feature[i].l_ix);
		int l_iy = cvRound(BinocularSLAM.map_feature[i].l_iy);
		int r_ix = cvRound(BinocularSLAM.map_feature[i].r_ix);
		int r_iy = cvRound(BinocularSLAM.map_feature[i].r_iy);
		int r = cvRound(BinocularSLAM.map_feature[i].size*1.2 / 9. * 2);

		if (BinocularSLAM.map_feature[i].theMAP)
		{
			cvRectangle(l_image_color, cvPoint(l_ix - r, l_iy + r), cvPoint(l_ix + r, l_iy - r), CV_RGB(255, 200, 0), 2, CV_AA, 0);
			cvRectangle(r_image_color, cvPoint(r_ix - r, r_iy + r), cvPoint(r_ix + r, r_iy - r), CV_RGB(255, 200, 0), 2, CV_AA, 0);
		}
		else
		{
			cvRectangle(l_image_color, cvPoint(l_ix - r, l_iy + r), cvPoint(l_ix + r, l_iy - r), CV_RGB(0, 0, 255), 2, CV_AA, 0);
			cvRectangle(r_image_color, cvPoint(r_ix - r, r_iy + r), cvPoint(r_ix + r, r_iy - r), CV_RGB(0, 0, 255), 2, CV_AA, 0);
		}
		cvLine(l_image_color, cvPoint(0, 120), cvPoint(320, 120), CV_RGB(0, 255, 125), 1);
		cvLine(l_image_color, cvPoint(160, 0), cvPoint(160, 240), CV_RGB(0, 255, 125), 1);
	}
	//----------------------------------------------------------------------------------------------------
	CWnd* pWnd_L_Image_SURF = GetDlgItem(IDC_L_Image_SURF);
	CWnd* pWnd_R_Image_SURF = GetDlgItem(IDC_R_Image_SURF);
	ShowImage(l_image_color, pWnd_L_Image_SURF);
	ShowImage(r_image_color, pWnd_R_Image_SURF);


	// 	IplImage *big_image = NULL;
	// 	big_image = cvCreateImage(cvSize(1280, 960), l_image_color->depth, 3);
	// 	cvResize(l_image_color, big_image, CV_INTER_LANCZOS4);
	// 
	// 	cvShowImage("big_image", big_image); // 顯示影像於視窗
	// 	cvWaitKey(30); // 停留視窗
	// 
	// 	cvReleaseImage(&big_image);
	//////////////

	_mkdir("photo2"); // 建立資料夾

	char path0[100];
	char path1[100];
	sprintf_s(path0, "photo2\\L%d.jpg", PhotoCount);
	sprintf_s(path1, "photo2\\R%d.jpg", PhotoCount);

	CDC* dc = pWnd_L_Image_SURF->GetWindowDC();
	CDC* dc1 = pWnd_R_Image_SURF->GetWindowDC();

	char num[10];

	CvScalar Color;
	Color = CV_RGB(255, 0, 0);
	CvFont Font1 = cvFont(1.3, 2);

	draw_feature.assign(BinocularSLAM.map_feature.begin(), BinocularSLAM.map_feature.end()); // draw_feature 複制 BinocularSLAM.map_feature
	feature_path.push_back(draw_feature);  //繪製特徵點軌跡

	if (m_FollowMode_c.GetCheck())  //追隨模式
	{
		// 		orgin.x = Path[Path.size()].x  * scale + map_move[0];
		// 		orgin.y = Path[Path.size()].y  * scale + map_move[1];



		// 		map_move[2] = Path[Path.size() - 1].x  * scale;
		// 		map_move[3] = Path[Path.size() - 1].y  * scale;
		// 
		// 		orgin.x = 300 + map_move[2];
		// 		orgin.y = 250 + map_move[3];

	}

	for (int i = 0; i < (int)BinocularSLAM.map_feature.size(); i++) // 顯示影像上地圖特徵點編號
	{
		if (!BinocularSLAM.map_feature[i].match) continue;

		int l_ix = cvRound(BinocularSLAM.map_feature[i].l_ix);
		int l_iy = cvRound(BinocularSLAM.map_feature[i].l_iy);
		int r_ix = cvRound(BinocularSLAM.map_feature[i].r_ix);
		int r_iy = cvRound(BinocularSLAM.map_feature[i].r_iy);
		int r = cvRound(BinocularSLAM.map_feature[i].size*1.2 / 9. * 2);


		CvPoint TextPosition1 = cvPoint(BinocularSLAM.map_feature[i].l_ix - 10, BinocularSLAM.map_feature[i].l_iy - 10);
		CvPoint TextPosition2 = cvPoint(BinocularSLAM.map_feature[i].r_ix - 10, BinocularSLAM.map_feature[i].r_iy - 10);

		sprintf_s(num, "%d", BinocularSLAM.map_feature[i].num);
		cvPutText(l_image_color, num, TextPosition1, &Font1, Color);
		cvPutText(r_image_color, num, TextPosition2, &Font1, Color);

		dc->TextOut(l_ix + 8, l_iy - 8, (CString)num);
		dc1->TextOut(r_ix + 8, r_iy - 8, (CString)num);
	}

	cvSaveImage(path0, l_image_color);
	cvSaveImage(path1, r_image_color);
	/////////////
	cvReleaseImage(&l_image_color);
	cvReleaseImage(&r_image_color);

	CWnd* pWnd_Map = GetDlgItem(IDC_Map);
	CDC* dc2 = pWnd_Map->GetWindowDC();

	dc2->SetBkColor(RGB(0, 0, 0));
	dc2->SetTextColor(RGB(255, 255, 255));

	char finish[10] = "Finish";

	if (move_turn == 3)
	{
		dc2->TextOut(0, 0, (CString)finish);
	}


	////



	ReleaseDC(dc);
	ReleaseDC(dc1);
	ReleaseDC(dc2);


	if (BinocularSLAM.map_feature.size())
	{

		//		fstream app_match("match.txt", ios::app);
		fstream app_world_feature("world_feature.txt", ios::app);
		//		fstream app_print("print.txt", ios::app);
		//fstream app_print_feature_descriptor("FeatureDescriptor12345.txt", ios::app);
		if (PhotoCount > 0)
		{
			int on_image_num1 = 0;
			for (int i = 0; i < BinocularSLAM.map_feature.size(); i++)
			{
				if (BinocularSLAM.map_feature[i].match)
				{
					on_image_num1++;
				}
			}

			//			app_match << setw(8) << PhotoCount << "  "  << setw(8) << BinocularSLAM.map_feature.size() << setw(8) << "  " <<on_image_num1 <<endl;
			//			app_print << setw(8) << PhotoCount << "  " << setw(8) << BinocularSLAM.map_feature.size() << setw(8) << "  " << on_image_num1 << "  " << "0" << "  " << "0" << "  " << "0" << "  " << "0" << "  " << "0" << endl;

			if (PhotoCount % 40 == 0 && !BinocularSLAM.read_map_mode)
			{
				app_world_feature.close();
				remove("world_feature.txt");
				fstream app_world_feature("world_feature.txt", ios::app);
			}

			app_world_feature << "20" << endl;

			for (int i = 0; i < BinocularSLAM.map_feature.size(); i++)
			{

				//-------------------20160608新增儲存地標點資訊--------------------------------------

				if (!BinocularSLAM.read_map_mode)
				{
					app_world_feature << BinocularSLAM.map_feature[i].X << " " << BinocularSLAM.map_feature[i].Y << " " << BinocularSLAM.map_feature[i].Z << " ";
					app_world_feature << BinocularSLAM.map_feature[i].hessian << " ";

					for (int j = 0; j < 16; j++)
						app_world_feature << BinocularSLAM.map_feature[i].average_descriptor[j] << " ";

					app_world_feature << endl;
				}
				//----------------------------------------------------------------------------------------
				// 				app_match<< setw(8)<<BinocularSLAM.map_feature[i].num<< setw(8)<<BinocularSLAM.map_feature[i].match<<endl;
				// 				if(!BinocularSLAM.map_feature[i].match) continue;
				// 				app_print << setw(8) << BinocularSLAM.map_feature[i].num << "  " << setw(8) << BinocularSLAM.map_feature[i].hx << "  " << setw(8) << BinocularSLAM.map_feature[i].hy  << "  " << setw(8) << BinocularSLAM.map_feature[i].hz << "  " ;
				// 				app_print << setw(8) << BinocularSLAM.map_feature[i].l_ix << "  " << setw(8) << BinocularSLAM.map_feature[i].l_iy << "  " << setw(8) << BinocularSLAM.map_feature[i].r_ix << "  " << setw(8) << BinocularSLAM.map_feature[i].r_iy << endl;
				//----------------------------------------------------------------------------------------------------輸出keyfeature
				//app_print_feature_descriptor << setw(4) << BinocularSLAM.map_feature[i].num
				//	<< setw(12) << BinocularSLAM.map_feature[i].hx
				//	<< setw(12) << BinocularSLAM.map_feature[i].hy
				//	<< setw(12) << -BinocularSLAM.map_feature[i].hz
				//	<< setw(3) << BinocularSLAM.map_feature[i].laplacian;
				//		for (int n = 0; n < 16; n++)
				//		{
				//			app_print_feature_descriptor << setw(15) << BinocularSLAM.map_feature[i].original_descriptor[n];
				//		}
				//app_print_feature_descriptor << endl;
				//----------------------------------------------------------------------------------------------------
			}
			// 			app_print << endl << endl;
			// 			app_match << endl << endl;
			// 			app_print.close();
			// 			app_match.close();
		}
	}
}


void SHHuangDlg::ShowPhotoCount(const int PhotoCount, CWnd* pWnd)
{
	CDC* dc = pWnd->GetWindowDC();

	char count[10];
	sprintf_s(count, "%0.4d", PhotoCount);
	dc->TextOut(0, 0, (CString)count);

	ReleaseDC(dc);
}

void SHHuangDlg::OnBnClickedButton1()
{
	IplImage * Draw_Path = NULL;

	//	Read_LRF_Map_static = cvLoadImage("path\\L0.png", 0);
	Draw_Path = cvLoadImage("20160626_090600/map_20160626_090600.pgm", 0);

	cvSaveImage("20160626_090600/map_20160626_090600.png", Draw_Path);


}


void SHHuangDlg::OnBnClickedStart()
{
	//	UpdateData(true);
	m_L_SelectCamera_c.EnableWindow(0);
	m_L_InitialCCD_c.EnableWindow(0);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(0);
	m_R_InitialCCD_c.EnableWindow(0);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(0);
	m_LoadImage_Mode_Laser_c.EnableWindow(0);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);

	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(1);

	//if(m_LoadImage_Mode_c.GetCheck())
	{
		m_Pause_c.EnableWindow(1);
	}
	//else
	//{
	//	m_Pause_c.EnableWindow(0);
	//}



	PhotoCount = 0;
	SampleTime = 0;

	Path.swap(vector<CvPoint2D64f>());
	SampleTime_temp.swap(vector<double>());

	remove("print.txt");
	remove("point.txt");
	remove("FeatureDescriptor12345.txt");
	remove("Measurement.txt");
	remove("Test.txt");
	remove("app_address.txt");
	remove("app_address_2.txt");
	remove("txt\\feature_l.txt");
	remove("txt\\feature_r.txt");
	remove("txt\\feature_p.txt");
	remove("txt\\p3p_time.txt");
	remove("txt\\point.txt");
	remove("txt\\feature_descriptor.txt");
	remove("Comparison.txt");
	remove("txt\\feature_state.txt");
	remove("match.txt");
	remove("isruning.txt");
	remove("position.txt");
	remove("last_map_feature.txt");
	remove("root.txt");
	remove("p3dxpos.txt");
	remove("move.txt");
	remove("RRANSACtime.txt");
	remove("SURFtime.txt");
	remove("P3Ptime.txt");
	remove("TEST.txt");
	if (!BinocularSLAM.read_map_mode)
		remove("world_feature.txt");

	if (BinocularSLAM.Online_Laser_Localization_enable)
	{
		remove("Laser_Data.txt");
		remove("Laser_Data2.txt");
	}
	// 	if (m_read_world_data)
	// 	{
	// 
	// 
	// 
	// 	}


	if (m_LoadImage_Mode_c.GetCheck() || m_LoadImage_Mode_Laser_c.GetCheck())
	{
		DeletePhoto2();
		fstream in_SampleTime("photo\\SampleTime.txt", ios::in);
		if (!in_SampleTime)	exit(1);

		while (!in_SampleTime.eof())
		{
			in_SampleTime >> SampleTime;
			SampleTime_temp.push_back(SampleTime);
		}

		in_SampleTime.close();
	}
	else if (m_OnLine_Mode_c.GetCheck() && m_SaveImage_Mode_c.GetCheck())
	{
		DeletePhoto();
		DeletePhoto2();
		remove("photo\\SampleTime.txt");
	}





	fstream in_Parameter("Parameter.txt", ios::in);
	if (!in_Parameter)	exit(1);

	char str[50];
	double value;

	while (in_Parameter >> str >> value)
	{
		if (!strcmp(str, "scale"))                    scale = scale;     //預設請用50!!!!!注意
		else if (!strcmp(str, "show_image_num"))	        show_image_num = (int)value;
		else if (!strcmp(str, "show_map_num"))	            show_map_num = (int)value;
		else if (!strcmp(str, "show_feature_region"))	    show_feature_region = (int)value;
		else if (!strcmp(str, "show_Search_Window_Size"))	show_Search_Window_Size = (int)value;
		else if (!strcmp(str, "show_all_feature"))	        show_all_feature = (int)value;
		else if (!strcmp(str, "txt"))                      txt = (int)value;
	}

	in_Parameter.close();

	//	initial_scale = scale;


	BinocularSLAM.Initial_SLAM(); //*****************************************************************************************







								  //多媒體計時器參數設定

	UINT uDelay = 1; // 自訂的取樣時間 單位:毫秒
	UINT uResolution = 1;
	DWORD dwUser = (DWORD)this;
	UINT fuEvent = TIME_PERIODIC; //You also choose TIME_ONESHOT;

	timeBeginPeriod(1); //最高取樣精度1ms
	FTimerID = timeSetEvent(uDelay, uResolution, TimeProc, dwUser, fuEvent);
}


void SHHuangDlg::OnBnClickedButtonReadfeature()
{
	fstream ReadFeature;
	double buffer[1000][30];
	string testttt;
	ReadFeature.open("world_feature_20160626.txt", ios::in);
	int size = 0;
	int i = 0;
	//	vector<Keep_Feature> Read_Feature;
	Keep_Feature temp_feature;

	ReadFeature >> size;
	for (i = 0; i < 123456; i++)
	{
		for (int j = 0; j < size; j++)
		{
			ReadFeature >> buffer[i][j];
		}

		temp_feature.X = buffer[i][0];
		temp_feature.Y = buffer[i][1];
		temp_feature.Z = buffer[i][2];
		temp_feature.hessian = buffer[i][3];

		for (int j = 0; j < 16; j++)
			temp_feature.average_descriptor[j] = buffer[i][4 + j];

		temp_feature.match = false;
		temp_feature.theMAP = true;
		temp_feature.num = i;
		temp_feature.laplacian = 1;
		BinocularSLAM.map_feature.push_back(temp_feature);

		if (ReadFeature.eof())
			break;
	}
	BinocularSLAM.feature_num = i;
	BinocularSLAM.read_map_mode = true;
	rotation_AMonte_Carlo = true;

	m_LoadImage_Mode_Laser_c.EnableWindow(0);

	draw_feature.assign(BinocularSLAM.map_feature.begin(), BinocularSLAM.map_feature.end());


}

void SHHuangDlg::OnBnClickedPause()
{
	m_L_SelectCamera_c.EnableWindow(0);
	m_L_InitialCCD_c.EnableWindow(0);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(0);
	m_R_InitialCCD_c.EnableWindow(0);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(0);
	m_LoadImage_Mode_Laser_c.EnableWindow(0);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(1);
	m_Stop_c.EnableWindow(1);


	timeKillEvent(FTimerID);
	timeEndPeriod(1);
}

void SHHuangDlg::OnBnClickedStop()
{
	m_L_SelectCamera_c.EnableWindow(1);
	m_L_InitialCCD_c.EnableWindow(1);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(1);
	m_R_InitialCCD_c.EnableWindow(1);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(1);
	m_LoadImage_Mode_Laser_c.EnableWindow(1);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(1);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);
	m_PathPlanning_c.EnableWindow(1);

	if (L_InitialCCD)
	{
		m_L_OptionCCD_c.EnableWindow(1);
		m_L_CloseCCD_c.EnableWindow(1);
	}

	if (R_InitialCCD)
	{
		m_R_OptionCCD_c.EnableWindow(1);
		m_R_CloseCCD_c.EnableWindow(1);
	}

	if (L_InitialCCD && R_InitialCCD)
	{
		m_OnLine_Mode_c.EnableWindow(1);
		m_SaveImage_Mode_c.EnableWindow(1);
	}
	stop_everthing = false;

	timeKillEvent(FTimerID);
	timeEndPeriod(1);
}

void SHHuangDlg::OnBnClickedContinue()
{
	m_L_SelectCamera_c.EnableWindow(0);
	m_L_InitialCCD_c.EnableWindow(0);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_R_SelectCamera_c.EnableWindow(0);
	m_R_InitialCCD_c.EnableWindow(0);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(0);
	m_LoadImage_Mode_Laser_c.EnableWindow(0);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);
	m_Pause_c.EnableWindow(1);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(1);




	fstream in_Parameter("Parameter.txt", ios::in);
	if (!in_Parameter)	exit(1);

	char str[50];
	double value;

	while (in_Parameter >> str >> value)
	{
		if (!strcmp(str, "show_image_num"))	        show_image_num = (int)value;
		else if (!strcmp(str, "show_map_num"))	            show_map_num = (int)value;
		else if (!strcmp(str, "show_feature_region"))	    show_feature_region = (int)value;
		else if (!strcmp(str, "show_Search_Window_Size"))	show_Search_Window_Size = (int)value;
		else if (!strcmp(str, "show_all_feature"))	        show_all_feature = (int)value;
	}

	in_Parameter.close();




	//多媒體計時器參數設定

	UINT uDelay = 1; // 自訂的取樣時間 單位:毫秒
	UINT uResolution = 1;
	DWORD dwUser = (DWORD)this;
	UINT fuEvent = TIME_PERIODIC; //You also choose TIME_ONESHOT;

	timeBeginPeriod(1); //最高取樣精度1ms
	FTimerID = timeSetEvent(uDelay, uResolution, TimeProc, dwUser, fuEvent);
}



void SHHuangDlg::OnBnClickedLInitialccd()
{
	m_L_SelectCamera_c.EnableWindow(1);
	m_L_InitialCCD_c.EnableWindow(1);
	m_L_OptionCCD_c.EnableWindow(1);
	m_L_CloseCCD_c.EnableWindow(1);
	m_Start_c.EnableWindow(1);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	L_Cam.CcdInitial(m_L_Image_Live_c.m_hWnd, m_L_SelectCamera_c.GetCurSel());
	L_Cam.Capture();
	L_InitialCCD = true;

	if (L_InitialCCD && R_InitialCCD)
	{
		m_LoadImage_Mode_c.SetCheck(0);
		m_OnLine_Mode_c.SetCheck(1);

		m_LoadImage_Mode_c.EnableWindow(1);
		m_OnLine_Mode_c.EnableWindow(1);
		m_SaveImage_Mode_c.EnableWindow(1);
	}
}


void SHHuangDlg::OnBnClickedLOptionccd()
{
	L_Cam.ConfigVideoFilter();
}


void SHHuangDlg::OnBnClickedLCloseccd()
{
	m_LoadImage_Mode_c.SetCheck(1);
	m_OnLine_Mode_c.SetCheck(0);
	m_SaveImage_Mode_c.SetCheck(0);

	m_L_SelectCamera_c.EnableWindow(1);
	m_L_InitialCCD_c.EnableWindow(1);
	m_L_OptionCCD_c.EnableWindow(0);
	m_L_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(1);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	L_Cam.StopCapture();
	L_InitialCCD = false;
}


void SHHuangDlg::OnBnClickedRInitialccd()
{
	m_R_SelectCamera_c.EnableWindow(1);
	m_R_InitialCCD_c.EnableWindow(1);
	m_R_OptionCCD_c.EnableWindow(1);
	m_R_CloseCCD_c.EnableWindow(1);
	m_Start_c.EnableWindow(1);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	R_Cam.CcdInitial(m_R_Image_Live_c.m_hWnd, m_R_SelectCamera_c.GetCurSel());
	R_Cam.Capture();
	R_InitialCCD = true;

	if (L_InitialCCD && R_InitialCCD)
	{
		m_LoadImage_Mode_c.SetCheck(0);
		m_LoadImage_Mode_Laser_c.SetCheck(0);
		m_OnLine_Mode_c.SetCheck(1);

		m_LoadImage_Mode_Laser_c.EnableWindow(1);
		m_LoadImage_Mode_c.EnableWindow(1);
		m_OnLine_Mode_c.EnableWindow(1);
		m_SaveImage_Mode_c.EnableWindow(1);
	}
}

void SHHuangDlg::OnBnClickedROptionccd()
{
	R_Cam.ConfigVideoFilter();
}

void SHHuangDlg::OnBnClickedRCloseccd()
{
	m_LoadImage_Mode_c.SetCheck(1);
	m_OnLine_Mode_c.SetCheck(0);
	m_SaveImage_Mode_c.SetCheck(0);

	m_R_SelectCamera_c.EnableWindow(1);
	m_R_InitialCCD_c.EnableWindow(1);
	m_R_OptionCCD_c.EnableWindow(0);
	m_R_CloseCCD_c.EnableWindow(0);
	m_LoadImage_Mode_c.EnableWindow(1);
	m_OnLine_Mode_c.EnableWindow(0);
	m_SaveImage_Mode_c.EnableWindow(0);
	m_Start_c.EnableWindow(0);
	m_Pause_c.EnableWindow(0);
	m_Continue_c.EnableWindow(0);
	m_Stop_c.EnableWindow(0);

	R_Cam.StopCapture();
	L_InitialCCD = false;
}

void SHHuangDlg::OnBnClickedButtonConnectI90()
{
	Connect_I90.Open(2, 115200);
}

void SHHuangDlg::OnBnClickedButtonSimulation()
{
	Continue_TARGET_control = TRUE;
	Thread_Info_TARGET_control.hWnd = m_hWnd;
	Thread_Info_TARGET_control.Continue = &Continue_TARGET_control;
	m_pThread_TARGET_control = AfxBeginThread(ThreadFun_TARGET_control, (LPVOID)&Thread_Info_TARGET_control);
}

void SHHuangDlg::OnBnClickedButtonPathplanning()
{
	Continue_Path_Planning = TRUE;
	Thread_Info_Path_Planning.hWnd = m_hWnd;
	Thread_Info_Path_Planning.Continue = &Continue_Path_Planning;
	m_pThread_Path_Planning = AfxBeginThread(ThreadFun_Path_Planning, (LPVOID)&Thread_Info_Path_Planning);

	m_PathPlanning_c.EnableWindow(0);
}

void SHHuangDlg::OnBnClickedButtonbig()
{
	scale = scale*Magnification;
}


void SHHuangDlg::OnBnClickedButtonsmall()
{
	scale = scale / Magnification;

}


void SHHuangDlg::OnBnClickedButtonUp()
{
	map_move[1] += Magnification * 2000 / scale + 10;
	map_move[3] += Magnification * 2000 / scale + 10;
}


void SHHuangDlg::OnBnClickedButtonDown()
{
	map_move[1] -= Magnification * 2000 / scale + 10;
	map_move[3] -= Magnification * 2000 / scale + 10;
}


void SHHuangDlg::OnBnClickedButtonLeft()
{
	map_move[0] += Magnification * 2000 / scale + 10;
	map_move[2] += Magnification * 2000 / scale + 10;
}


void SHHuangDlg::OnBnClickedButtonRight()
{
	map_move[0] -= Magnification * 2000 / scale + 10;
	map_move[2] -= Magnification * 2000 / scale + 10;
}


void SHHuangDlg::OnBnClickedButtonReset()
{
	scale = initial_scale;
	map_move[0] = 0;
	map_move[1] = 0;
	map_move[2] = 0;
	map_move[3] = 0;
	orgin.x = (rect_map.right - rect_map.left) / 2;
	orgin.y = (rect_map.bottom - rect_map.top) / 2;

}

void SHHuangDlg::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值

	CDialog::OnKeyDown(nChar, nRepCnt, nFlags);

}

void SHHuangDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值
	Wheel_mouse_pos = point;

	CDialog::OnMouseMove(nFlags, point);
}

BOOL SHHuangDlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值
	if (zDelta > 0)
		scale = scale * Magnification;
	else
		scale = scale / Magnification;

	int x_pos = Wheel_mouse_pos.x - IDC_Map_Rect_Map[0];
	int y_pos = Wheel_mouse_pos.y - IDC_Map_Rect_Map[1];



	double x_pos2 = x_pos - orgin.x;
	double y_pos2 = y_pos - orgin.y;




	if (x_pos > 0 && y_pos > 0 && x_pos < rect_map.right - rect_map.left && y_pos < rect_map.bottom - rect_map.top)
	{
		orgin.x = x_pos;
		orgin.y = y_pos;

		if (zDelta > 0)
		{
			map_move[0] = (map_move[0] - x_pos2) * Magnification;
			map_move[1] = (map_move[1] - y_pos2) * Magnification;
			map_move[2] += ((IDC_Map_Rect_Map[0] + IDC_Map_Rect_Map[2] / 2) - Wheel_mouse_pos.x) * (Magnification - 1)*(initial_scale / scale);
			map_move[3] += ((IDC_Map_Rect_Map[1] + IDC_Map_Rect_Map[3] / 2) - Wheel_mouse_pos.y) * (Magnification - 1)*(initial_scale / scale);
		}
		else
		{
			map_move[0] = (map_move[0] - x_pos2) / Magnification;
			map_move[1] = (map_move[1] - y_pos2) / Magnification;
			map_move[2] -= ((IDC_Map_Rect_Map[0] + IDC_Map_Rect_Map[2] / 2) - Wheel_mouse_pos.x) * (Magnification - 1)*(initial_scale / scale / Magnification);
			map_move[3] -= ((IDC_Map_Rect_Map[1] + IDC_Map_Rect_Map[3] / 2) - Wheel_mouse_pos.y) * (Magnification - 1)*(initial_scale / scale / Magnification);
		}
	}

	return CDialog::OnMouseWheel(nFlags, zDelta, pt);
}

void SHHuangDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值

	LButtonDown_mouse_pos = point;
	Click_Left_Button = true;
	CDialog::OnLButtonDown(nFlags, point);
}

BOOL SHHuangDlg::PreTranslateMessage(MSG * pMsg)
{
	// TODO:  在此加入特定的程式碼和 (或) 呼叫基底類別
	int upupup = 2000;
	int Lpwm1 = 9500 + upupup, Rpwm1 = 9500 + upupup;
	int Lpwm2 = -9500 - upupup, Rpwm2 = -9500 - upupup;
	int Lpwm3 = -9500 - upupup, Rpwm3 = 9500 + upupup;
	int Lpwm4 = 9500 + upupup, Rpwm4 = -9500 - upupup;

	if (pMsg->message == WM_KEYDOWN)
	{
		switch (pMsg->wParam)
		{
		case 0x57:
			I90_PWM_send(Lpwm1, Rpwm1);  break;  //I90直走 w
		case 0x53:
			I90_PWM_send(Lpwm2, Rpwm2); break;  //I90後退 s
		case 0x41:
			I90_PWM_send(Lpwm3, Rpwm3); break;  //I90左轉 a
		case 0x44:
			I90_PWM_send(Lpwm4, Rpwm4); break;  //I90右轉 d
		case 0x54:
			OnBnClickedButtonUp();  break;   //小黑窗上移 t
		case 0x47:
			OnBnClickedButtonDown(); break;   //小黑窗下移 g
		case 0x46:
			OnBnClickedButtonLeft(); break;   //小黑窗左移 f
		case 0x48:
			OnBnClickedButtonRight(); break;   //小黑窗右移 h
		case 0x52:
			OnBnClickedButtonbig(); break;   //小黑窗放大 r
		case 0x59:
			OnBnClickedButtonDown(); break;   //小黑窗縮小 y
		case 0x58:
			sleep_time += 20; break;   //運算速度下降 x
		case 0x5A:
			sleep_time = 0; break;   //運算速度恢復 z
		case VK_F2:
			I90_PWM_send(0, 0); break;   //I90煞車 F2
		default:
			break;
		}
	}

	return CDialog::PreTranslateMessage(pMsg);
}

void SHHuangDlg::OnBnClickedSocketConnect()
{
	if (fg_map_connected || fg_lrf_connected || fg_pose_connected) {
		if (fg_map_connected) DoMapSocketDisconnect();
		if (fg_lrf_connected) DoLRFSocketDisconnect();
		if (fg_pose_connected) DoPoseSocketDisconnect();

	}
	else {

		// If all socket not connect yet
		DoMapSocketConnect();
		DoLRFSocketConnect();
		DoPoseSocketConnect();
	}

	m_socket_connect_c.SetCheck(
		fg_map_connected && fg_lrf_connected && fg_pose_connected);

	BinocularSLAM.Online_Laser_Localization_enable = true;
	//	m_read_map_c.EnableWindow(0);
}


void SHHuangDlg::DeletePhoto()
{

	PhotoCount = 0;

	while (1)
	{
		char path0[100];
		sprintf_s(path0, "photo\\L%d.png", PhotoCount);
		fstream in_image0(path0, ios::in);

		char path1[100];
		sprintf_s(path1, "photo\\R%d.png", PhotoCount);
		fstream in_image1(path1, ios::in);

		if (in_image0 && in_image1)
		{
			in_image0.close();
			in_image1.close();

			remove(path0);
			remove(path1);

			PhotoCount++;
		}
		else
		{
			PhotoCount -= 1;

			in_image0.close();
			in_image1.close();

			break;
		}
	}

	PhotoCount = 0;

}

void SHHuangDlg::DeletePhoto2()
{

	PhotoCount = 0;

	while (1)
	{
		char path0[100], path1[100];
		sprintf_s(path0, "photo2\\L%d.png", PhotoCount);
		sprintf_s(path1, "photo2\\R%d.png", PhotoCount);
		fstream in_image0(path0, ios::in);

		if (in_image0)
		{
			in_image0.close();

			remove(path0);
			remove(path1);

			PhotoCount++;
		}
		else
		{
			PhotoCount -= 1;

			in_image0.close();

			break;
		}
	}

	PhotoCount = 0;

}

void SHHuangDlg::save_car_pos()
{


	if (PhotoCount)
	{
		Camera[0] = BinocularSLAM.p3dx_x.x;
		Camera[1] = BinocularSLAM.p3dx_x.y;
		Camera[2] = BinocularSLAM.p3dx_x.z;
		Camera[3] = BinocularSLAM.p3dx_x.x_dir;
		Camera[4] = BinocularSLAM.p3dx_x.y_dir;
		Camera[5] = BinocularSLAM.p3dx_x.z_dir;
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			Camera[i] = 0;
		}
	}

	CvPoint2D64f Pos;
	Pos.x = Camera[0];
	Pos.y = Camera[1];
	Path.push_back(Pos);

	if (BinocularSLAM.read_map_mode && !BinocularSLAM.do_onetimes_P3P)
	{
		if (fg_cmd_connected) {
			ControlMsg msg;
			memset(&msg, 0, sizeof(ControlMsg));
			msg.lrf_msg = ControlLRFMsg::START_LRF_STREAM;
			msg.map_msg = ControlMapMsg::SET_INIT_POSE;

			msg.init_pose_position[0] = Camera[0];
			msg.init_pose_position[1] = Camera[1];
			msg.init_pose_orientation[2] = Camera[5] + CV_PI / 2;

			// Setup covariance matrix
			msg.init_pose_cov[6 * 0 + 0] = 0.1 * 0.1;
			msg.init_pose_cov[6 * 1 + 1] = 0.1 * 0.1;
			msg.init_pose_cov[6 * 5 + 5] = 0.1 * 0.1;
			m_socket_cmd.Send(&msg, sizeof(ControlMsg));

			BinocularSLAM.read_map_mode = false;
		}
		else {
			AfxMessageBox(CString(_T("請先連接! Cmd tunnel")));
		}

	}

}


template<size_t LENGTH> void SHHuangDlg::SendSocketMessage(char(&data)[LENGTH]) {

	// Send Message
	if (fg_tcp_ip_connected) {
		m_tcp_socket.Send(data, LENGTH);
		ReportTCPStatus(TCPEvent::SEND_MESSAGE_SUCCESSFUL, CString(data));
	}
}


void SHHuangDlg::I90_PWM_send(int L_PWM, int R_PWM)
{
	int L, R, mid = 16384; //0~32767

	if (R_PWM > 0)
		R_PWM = R_PWM + c_Synchronous;

	if (R_PWM < 0)
	{
		L_PWM = L_PWM - c_Synchronous;
		R_PWM = R_PWM;
	}

	//L.R.轉換
	L = mid + L_PWM;
	R = mid - R_PWM;



	I90_PWM_control[7] = R;
	I90_PWM_control[8] = R >> 8;
	I90_PWM_control[10] = L;
	I90_PWM_control[11] = L >> 8;
	I90_PWM_control[(I90_PWM_control[5] + 6)] = checksun(2, (I90_PWM_control[5] + 5));


	//I90rs232.Open(7,115200);
	Connect_I90.SendData(I90_PWM_control, sizeof(I90_PWM_control));
}

unsigned char SHHuangDlg::checksun(int nStar, int nEnd)
{
	unsigned char shift_reg, sr_lsb, data_bit, v, fb_bit;
	int i, j;

	shift_reg = 0;
	for (i = nStar; i <= (nEnd); i++)
	{
		v = (unsigned char)(I90_PWM_control[i] & 0x0000ffff);
		for (j = 0; j < 8; j++)
		{
			data_bit = v & 0x01;  // isolate least sign bit
			sr_lsb = shift_reg & 0x01;
			fb_bit = (data_bit ^ sr_lsb) & 0x01;  //calculate the feed back bit
			shift_reg = shift_reg >> 1;
			if (fb_bit == 1)
			{
				shift_reg = shift_reg ^ 0x8c;
			}
			v = v >> 1;
		}
	}
	return shift_reg;
}


void SHHuangDlg::binarization(IplImage * i_show_data, vector<vector<bool>> &o_sca_image2)
{
	vector<bool> sca_image1;

	for (int y = 0; y < i_show_data->height - 1; y++)
	{
		for (int x = 0; x < i_show_data->width - 1; x++)
		{
			int temp_of_image = cvGetReal2D(i_show_data, y, x);

			if (temp_of_image > 10)
				sca_image1.push_back(1);
			else
				sca_image1.push_back(0);

			if (x == i_show_data->width - 2)
				sca_image1.push_back(0);
		}
		o_sca_image2.push_back(sca_image1);
		sca_image1.clear();
	}

	for (int x = 0; x < i_show_data->width; x++)
	{
		sca_image1.push_back(0);
	}

	o_sca_image2.push_back(sca_image1);
	sca_image1.clear();

}

void SHHuangDlg::find_coner(vector<vector<bool>> i_sca_image, vector <cv::Point> &o_save_coner, int i_Interpolation)
{
	int result_out;
	int Laplacian_mask[3][3] = { { 0, -1, 0 },{ -1, 4, -1 },{ 0, -1, 0 } };
	int Interpolation = 0;

	for (int y = 1; y < i_sca_image.size() - 1; y++)
	{
		for (int x = 1; x < i_sca_image[0].size() - 1; x++)
		{

			result_out =
				Laplacian_mask[0][0] * i_sca_image[y - 1][x - 1] +
				Laplacian_mask[0][1] * i_sca_image[y - 1][x] +
				Laplacian_mask[0][2] * i_sca_image[y - 1][x + 1] +
				Laplacian_mask[1][0] * i_sca_image[y][x - 1] +
				Laplacian_mask[1][1] * i_sca_image[y][x] +
				Laplacian_mask[1][2] * i_sca_image[y][x + 1] +
				Laplacian_mask[2][0] * i_sca_image[y + 1][x - 1] +
				Laplacian_mask[2][1] * i_sca_image[y + 1][x] +
				Laplacian_mask[2][2] * i_sca_image[y + 1][x + 1];


			if (result_out == 1)
			{
				Interpolation++;
				if (Interpolation == i_Interpolation)
				{
					o_save_coner.push_back(cvPoint(x, y));
					Interpolation = 0;
				}
			}
			else
				Interpolation = 0;


			if (Laplacian_mask[0][1] * i_sca_image[y - 1][x] + Laplacian_mask[2][1] * i_sca_image[y + 1][x] == -2)  //濾掉直線
				continue;
			if (Laplacian_mask[1][0] * i_sca_image[y][x - 1] + Laplacian_mask[1][2] * i_sca_image[y][x + 1] == -2)  //濾掉橫線
				continue;

			if (result_out == 2 || result_out == 3)
				o_save_coner.push_back(cvPoint(x, y));
		}
	}


	for (int x = 1; x < i_sca_image[0].size() - 1; x++)
	{
		for (int y = 1; y < i_sca_image.size() - 1; y++)
		{

			result_out =
				Laplacian_mask[0][0] * i_sca_image[y - 1][x - 1] +
				Laplacian_mask[0][1] * i_sca_image[y - 1][x] +
				Laplacian_mask[0][2] * i_sca_image[y - 1][x + 1] +
				Laplacian_mask[1][0] * i_sca_image[y][x - 1] +
				Laplacian_mask[1][1] * i_sca_image[y][x] +
				Laplacian_mask[1][2] * i_sca_image[y][x + 1] +
				Laplacian_mask[2][0] * i_sca_image[y + 1][x - 1] +
				Laplacian_mask[2][1] * i_sca_image[y + 1][x] +
				Laplacian_mask[2][2] * i_sca_image[y + 1][x + 1];


			if (result_out == 1)
			{
				Interpolation++;
				if (Interpolation == i_Interpolation)
				{
					o_save_coner.push_back(cvPoint(x, y));
					Interpolation = 0;
				}
			}
			else
			{
				Interpolation = 0;
			}

		}
	}

}

void SHHuangDlg::trans2Voronoi(vector<vector<bool>> i_sca_image, vector<cv::Point> i_save_coner, double(&o_Data)[8000], int i_Interpolation2)
{
	int input_Data = 0;
	int jump_count = 0;
	jump_count = (i_sca_image.size() + 1) / i_Interpolation2 + 1;

	o_Data[0] = i_save_coner.size();
	for (input_Data = 1; input_Data < i_save_coner.size() + 1; input_Data++)
	{
		//		cvLine(RGB_show_data, cvPoint(save_coner[input_Data - 1].x, save_coner[input_Data - 1].y),cvPoint(save_coner[input_Data - 1].x, save_coner[input_Data - 1].y), CV_RGB(0, 255, 0), 1);
		o_Data[2 * input_Data - 1] = i_save_coner[input_Data - 1].x;
		o_Data[2 * input_Data] = i_save_coner[input_Data - 1].y;
	}



	for (int i = 0; i <= i_sca_image.size() + 1; i = i + i_Interpolation2)
	{
		for (int j = 0; j <= i_sca_image[0].size() + 1; j = j + i_Interpolation2)
		{
			if (i == 0 || i == i_sca_image.size() || j == 0 || j == i_sca_image[0].size())
			{

				o_Data[2 * input_Data - 1] = i;
				o_Data[2 * input_Data] = j;

				input_Data++;
				o_Data[0]++;
				//				cvLine(RGB_show_data, cvPoint(i, j),cvPoint(i, j), CV_RGB(200, 200, 0), 1);
			}
		}
	}
}

void SHHuangDlg::Voronoi_calculate(double i_Data[8000], int x_boundary, int y_boundary, CvPoint2D64f(&o_savepoint1)[3000], CvPoint2D64f(&o_savepoint2)[3000], int &o_line_count)
{
	remove("原始VD座標輸出.txt");
	fstream app_VD_output2("原始VD座標輸出.txt", ios::app);

	CWnd* CW_vo = (CWnd *)GetDlgItem(IDC_Voronoi);
	CDC* pDC = CW_vo->GetWindowDC();
	CVoronoi* vor;
	vor = new CVoronoi();
	POSITION aPos;
	Site** EdgeSite;

	double *x1, *x2, *y1, *y2, let00 = 0, pxmax = x_boundary, pymax = -y_boundary, temp_y1, temp_x1, temp_x2, temp_y2, pymax_invers = y_boundary;
	int i_line_count = 0, line_count2 = 0;
	int pos;

	CPen aPen;
	aPen.CreatePen(PS_SOLID, 2, RGB(255, 0, 0));
	pDC->SelectObject(&aPen);
	vor->SetPoints(i_Data);
	vor->DrawEdges(pDC, 0, 1);
	CList<Edge, Edge&> *VorEdgeList = vor->GetEdges();
	CList<Edge, Edge&> *VorLineList = vor->GetLines();
	if (!VorEdgeList->IsEmpty())
	{
		aPos = VorEdgeList->GetHeadPosition();
		do {
			Edge& aEdge = VorEdgeList->GetNext(aPos);
			pos = aEdge.edgenbr;
			x1 = &aEdge.ep[0]->coord.x;
			y1 = &aEdge.ep[0]->coord.y;
			x2 = &aEdge.ep[1]->coord.x;
			y2 = &aEdge.ep[1]->coord.y;

			if (aEdge.ep[0] && aEdge.ep[1])
			{
				app_VD_output2 << *x1 << ", " << *y1 << " 到 " << *x2 << ", " << *y2 << ",第 " << line_count2 << endl;
				line_count2++;
			}

			if (!aEdge.ep[0] || *x1 < let00 || *y1 < let00)
			{
				if (aEdge.b != 0)
				{
					x1 = &let00;
					temp_y1 = (aEdge.c - aEdge.a*(*x1)) / aEdge.b;
					y1 = &temp_y1;
				}
				else
				{
					temp_x1 = aEdge.c / aEdge.a;
					x1 = &temp_x1;
					y1 = &let00;
				}
			}

			if (!aEdge.ep[1] || *y2 < let00 || *x2 < let00)
			{
				if (aEdge.b != 0)
				{
					x2 = &pxmax;
					temp_y2 = (aEdge.c - aEdge.a*(*x2)) / aEdge.b;
					y2 = &temp_y2;
				}
				else
				{
					temp_x2 = aEdge.c / aEdge.a;
					x2 = &temp_x2;
					y2 = &pymax;
				}
			}

			if (!*y1)
			{
				//				app_VD_output << (int)*x1 << ", " << (int)*y1 << " 到 " << (int)*x2 << ", " << (int)*y2 << "--------y1沒值---------" << endl;
				y1 = &pymax_invers;
			}

			if ((int)*x1 == (int)*x2 && (int)*y1 == (int)*y2)
				continue;

			if (*y2 < 0)
				*y2 = 0;

			pDC->MoveTo((int)*x1, (int)*y1);
			pDC->LineTo((int)*x2, (int)*y2);


			o_savepoint1[i_line_count].x = (*x1);
			o_savepoint1[i_line_count].y = (*y1);
			o_savepoint2[i_line_count].x = (*x2);
			o_savepoint2[i_line_count].y = (*y2);
			i_line_count++;

		} while (aPos);

	}

	o_line_count = i_line_count;

	app_VD_output2.close();
	ReleaseDC(pDC);
	delete vor;

}

void SHHuangDlg::Generalized_Voronoi(vector<vector<bool>> i_sca_image, CvPoint2D64f i_savepoint1[3000], CvPoint2D64f i_savepoint2[3000], int i_line_count, int &o_new_input_index, CvPoint2D64f(&o_new_savepoint1)[3000], CvPoint2D64f(&o_new_savepoint2)[3000])
{
	remove("廣義VD座標輸出.txt");
	fstream app_VD_output("廣義VD座標輸出.txt", ios::app);

	int cheak_point = 0, line_distant, new_input_index = 0;
	CvPoint2D64f cutout_point[2000];

	for (cheak_point = 0; cheak_point < i_line_count; cheak_point++)
	{
		if (i_savepoint1[cheak_point].x == 0 && i_savepoint1[cheak_point].y == 0 && i_savepoint2[cheak_point].x == 0 && i_savepoint2[cheak_point].y == 0)
			continue;

		line_distant = sqrt(pow(i_savepoint1[cheak_point].x - i_savepoint2[cheak_point].x, 2) + pow(i_savepoint1[cheak_point].y - i_savepoint2[cheak_point].y, 2));
		//		line_distant++;

		int cutout = 0;
		for (cutout = 0; cutout < line_distant; cutout++)
		{
			//計算斜線上的點

			double addddd = (sqrt(pow((i_savepoint1[cheak_point].x - i_savepoint2[cheak_point].x), 2)) / (double)line_distant)*(double)cutout;
			double addddd2 = (sqrt(pow((i_savepoint1[cheak_point].y - i_savepoint2[cheak_point].y), 2)) / (double)line_distant)*(double)cutout;

			if (i_savepoint1[cheak_point].x >= i_savepoint2[cheak_point].x && i_savepoint1[cheak_point].y >= i_savepoint2[cheak_point].y)
			{
				cutout_point[cutout].x = i_savepoint2[cheak_point].x + (sqrt(pow((i_savepoint1[cheak_point].x - i_savepoint2[cheak_point].x), 2)) / (double)line_distant)*(double)cutout;
				cutout_point[cutout].y = i_savepoint2[cheak_point].y + (sqrt(pow((i_savepoint1[cheak_point].y - i_savepoint2[cheak_point].y), 2)) / (double)line_distant)*(double)cutout;
			}

			if (i_savepoint2[cheak_point].x >= i_savepoint1[cheak_point].x && i_savepoint1[cheak_point].y >= i_savepoint2[cheak_point].y)
			{
				cutout_point[cutout].x = i_savepoint1[cheak_point].x + (sqrt(pow((i_savepoint1[cheak_point].x - i_savepoint2[cheak_point].x), 2)) / (double)line_distant)*(double)cutout;
				cutout_point[cutout].y = i_savepoint1[cheak_point].y - (sqrt(pow((i_savepoint1[cheak_point].y - i_savepoint2[cheak_point].y), 2)) / (double)line_distant)*(double)cutout;
			}

			if (i_savepoint1[cheak_point].x >= i_savepoint2[cheak_point].x && i_savepoint2[cheak_point].y >= i_savepoint1[cheak_point].y)
			{
				cutout_point[cutout].x = i_savepoint2[cheak_point].x + (sqrt(pow((i_savepoint1[cheak_point].x - i_savepoint2[cheak_point].x), 2)) / (double)line_distant)*(double)cutout;
				cutout_point[cutout].y = i_savepoint1[cheak_point].y - (sqrt(pow((i_savepoint1[cheak_point].y - i_savepoint2[cheak_point].y), 2)) / (double)line_distant)*(double)cutout;
			}

			if (i_savepoint2[cheak_point].x >= i_savepoint1[cheak_point].x && i_savepoint2[cheak_point].y >= i_savepoint1[cheak_point].y)
			{
				cutout_point[cutout].x = i_savepoint1[cheak_point].x + (sqrt(pow((i_savepoint1[cheak_point].x - i_savepoint2[cheak_point].x), 2)) / (double)line_distant)*(double)cutout;
				cutout_point[cutout].y = i_savepoint1[cheak_point].y + (sqrt(pow((i_savepoint1[cheak_point].y - i_savepoint2[cheak_point].y), 2)) / (double)line_distant)*(double)cutout;
			}
		}

		if (line_distant == 0)
		{

			o_new_savepoint1[new_input_index] = i_savepoint1[cheak_point];
			o_new_savepoint2[new_input_index] = i_savepoint2[cheak_point];

			if ((round(o_new_savepoint1[new_input_index].y) < i_sca_image.size() - 1) && (round(o_new_savepoint1[new_input_index].x) < i_sca_image.size() - 1) && (round(o_new_savepoint1[new_input_index].y) > 1 && (round(o_new_savepoint1[new_input_index].x) > 1)))
				if (i_sca_image[round(o_new_savepoint1[new_input_index].y)][round(o_new_savepoint1[new_input_index].x)] == 0 && i_sca_image[round(o_new_savepoint2[new_input_index].y)][round(o_new_savepoint2[new_input_index].x)] == 0)
				{
					app_VD_output << o_new_savepoint1[new_input_index].x * (double)10.0 << ", " << o_new_savepoint1[new_input_index].y * (double)10.0 << " 到 " << o_new_savepoint2[new_input_index].x * (double)10.0 << ", " << o_new_savepoint2[new_input_index].y * (double)10.0 << ", 第 " << new_input_index << endl;
					new_input_index++;
				}
		}

		for (int i = 0; i < cutout - 0; i++)
		{
			if ((round(cutout_point[i].y) > i_sca_image.size() - 1) || (round(cutout_point[i].x) > i_sca_image.size() - 1) || (round(cutout_point[i].y) < 1 || (round(cutout_point[i].x) < 1)))
			{
				o_new_savepoint1[new_input_index] = i_savepoint1[cheak_point];
				o_new_savepoint2[new_input_index] = i_savepoint2[cheak_point];

				app_VD_output << o_new_savepoint1[i].x * (double)10.0 << ", " << o_new_savepoint1[i].y * (double)10.0 << " 到 " << o_new_savepoint2[i].x * (double)10.0 << ", " << o_new_savepoint2[i].y * (double)10.0 << ", 第 " << new_input_index << endl;
				new_input_index++;
				continue;
			}

			if (i_sca_image[round(cutout_point[i].y)][round(cutout_point[i].x)] == 1 /* ||
																					 i_sca_image[round(cutout_point[i].y + 1)][round(cutout_point[i].x)] == 1 ||
																					 i_sca_image[round(cutout_point[i].y - 1)][round(cutout_point[i].x)] == 1||
																					 i_sca_image[round(cutout_point[i].y)][round(cutout_point[i].x - 1)] == 1 ||
																					 i_sca_image[round(cutout_point[i].y)][round(cutout_point[i].x + 1)] == 1*/)
			{
				break;
			}

			if (i == cutout - 1)
			{
				o_new_savepoint1[new_input_index] = i_savepoint1[cheak_point];
				o_new_savepoint2[new_input_index] = i_savepoint2[cheak_point];

				app_VD_output << o_new_savepoint1[i].x * (double)10.0 << ", " << o_new_savepoint1[i].y * (double)10.0 << " 到 " << o_new_savepoint2[i].x * (double)10.0 << ", " << o_new_savepoint2[i].y * (double)10.0 << ", 第 " << new_input_index << endl;
				new_input_index++;
			}
		}
	}
	o_new_input_index = new_input_index;
}

void SHHuangDlg::Match_point(int i_line_count, int i_new_input_index, CvPoint2D64f(&io_new_savepoint1)[3000], CvPoint2D64f(&io_new_savepoint2)[3000], float near_dis)
{
	int small_loop1, small_loop2, small_index = 0;
	double small_dis1, small_dis2, small_dis3;
	CvPoint2D64f temp_point;
	vector <CvPoint2D64f> center_point, center_point2;

	for (small_loop1 = 0; small_loop1 < i_line_count; small_loop1++)
	{
		for (small_loop2 = 0; small_loop2 < i_line_count; small_loop2++)
		{
			if (io_new_savepoint1[small_loop1].x == io_new_savepoint2[small_loop2].x&&
				io_new_savepoint1[small_loop1].y == io_new_savepoint2[small_loop2].y)
				continue;


			small_dis1 = sqrt(pow(io_new_savepoint1[small_loop1].x - io_new_savepoint2[small_loop2].x, 2) + pow(io_new_savepoint1[small_loop1].y - io_new_savepoint2[small_loop2].y, 2));
			if (small_dis1 < near_dis)
			{
				temp_point.x = (io_new_savepoint1[small_loop1].x + io_new_savepoint2[small_loop2].x) / 2;
				temp_point.y = (io_new_savepoint1[small_loop1].y + io_new_savepoint2[small_loop2].y) / 2;
				//io_new_savepoint2[small_loop2] = io_new_savepoint1[small_loop1];
				center_point.push_back(cvPoint2D64f(temp_point.x, temp_point.y));
			}
		}
	}


	for (small_loop1 = 0; small_loop1 < i_line_count; small_loop1++)
	{
		for (small_loop2 = 0; small_loop2 < i_line_count; small_loop2++)
		{
			small_dis1 = sqrt(pow(io_new_savepoint1[small_loop1].x - io_new_savepoint1[small_loop2].x, 2) + pow(io_new_savepoint1[small_loop1].y - io_new_savepoint1[small_loop2].y, 2));
			small_dis2 = sqrt(pow(io_new_savepoint2[small_loop1].x - io_new_savepoint2[small_loop2].x, 2) + pow(io_new_savepoint2[small_loop1].y - io_new_savepoint2[small_loop2].y, 2));
			if (small_dis1 < near_dis)
			{
				temp_point.x = (io_new_savepoint1[small_loop1].x + io_new_savepoint1[small_loop2].x) / 2;
				temp_point.y = (io_new_savepoint1[small_loop1].y + io_new_savepoint1[small_loop2].y) / 2;
			}
			if (small_dis2 < near_dis)
			{
				temp_point.x = (io_new_savepoint2[small_loop1].x + io_new_savepoint2[small_loop2].x) / 2;
				temp_point.y = (io_new_savepoint2[small_loop1].y + io_new_savepoint2[small_loop2].y) / 2;
			}
		}
	}

	center_point2.push_back(center_point[0]);

	for (int i = 1; i < center_point.size(); i++)
	{

		for (int j = 0; j < center_point2.size(); j++)
		{
			small_dis3 = sqrt(pow(center_point[i].x - center_point2[j].x, 2) + pow(center_point[i].y - center_point2[j].y, 2));

			if (i == j)
				continue;

			if (small_dis3 < near_dis)
			{
				//				center_point2.push_back(center_point[i]);
				small_index = 0;
				break;
			}
			small_index = 1;
		}

		if (small_index == 1)
		{
			small_index = 0;
			center_point2.push_back(center_point[i]);
		}

	}

	for (int i = 0; i < i_new_input_index; i++)
	{

		for (int j = 0; j < center_point2.size(); j++)
		{
			small_dis1 = sqrt(pow(io_new_savepoint1[i].x - center_point2[j].x, 2) + pow(io_new_savepoint1[i].y - center_point2[j].y, 2));
			small_dis2 = sqrt(pow(io_new_savepoint2[i].x - center_point2[j].x, 2) + pow(io_new_savepoint2[i].y - center_point2[j].y, 2));

			if (small_dis1 < near_dis)
			{
				io_new_savepoint1[i] = center_point2[j];
			}

			if (small_dis2 < near_dis)
			{
				io_new_savepoint2[i] = center_point2[j];
			}
		}
	}
}

void SHHuangDlg::Dijkstra_path_planning(CvPoint i_robot_start, CvPoint  i_robot_end, CvPoint2D64f i_new_savepoint1[3000], CvPoint2D64f i_new_savepoint2[3000], int i_new_input_index, vector <CPoint> &o_all_point_map, vector <CvPoint2D64f> &o_all_point_map_original)
{
	// 	remove("所有座標編號.txt");
	// 	fstream app_path_num("所有座標編號.txt", ios::app);

	vector <CPoint> CPoint_savepoint1;
	vector <CPoint> CPoint_savepoint2;

	for (int i = 0; i < i_new_input_index; i++)
	{
		CPoint_savepoint1.push_back(CPoint(round(i_new_savepoint1[i].x * 10), round(i_new_savepoint1[i].y * 10)));
		CPoint_savepoint2.push_back(CPoint(round(i_new_savepoint2[i].x * 10), round(i_new_savepoint2[i].y * 10)));
	}

	int loop1, loop2;
	bool onestime;
	int put_index = 0;

	//先丟入第一組，以免檢查時vector時沒東西
	o_all_point_map.push_back(CPoint_savepoint1[0]);
	o_all_point_map_original.push_back(i_new_savepoint1[0]);
	//	app_path_num << o_all_point_map[put_index].x << ", " << o_all_point_map[put_index].y << " 第 " << put_index << endl;
	put_index++;

	for (int i = 0; i < i_new_input_index; i++)
	{
		onestime = 1;
		for (int j = 0; j < o_all_point_map.size(); j++)
		{
			if (o_all_point_map[j] == CPoint_savepoint1[i])
				onestime = 0;
		}
		if (onestime == 1)
		{
			o_all_point_map.push_back(CPoint_savepoint1[i]);
			o_all_point_map_original.push_back(i_new_savepoint1[i]); // 原始的也排列一次
																	 //			app_path_num << o_all_point_map[put_index].x << ", " << o_all_point_map[put_index].y << " 第 " << put_index << endl;
			put_index++;
		}
	}

	for (int i = 0; i < i_new_input_index; i++)
	{
		onestime = 1;
		for (int j = 0; j < o_all_point_map.size(); j++)
		{
			if (o_all_point_map[j] == CPoint_savepoint2[i])
				onestime = 0;
		}
		if (onestime == 1)
		{
			o_all_point_map.push_back(CPoint_savepoint2[i]);
			o_all_point_map_original.push_back(i_new_savepoint2[i]); // 原始的也排列一次
																	 //			app_path_num << o_all_point_map[put_index].x << ", " << o_all_point_map[put_index].y << " 第 " << put_index << endl;
			put_index++;
		}
	}

	int savetemp_index1, savetemp_index2;
	int the_point_index[2];
	int serch_point[4];
	serch_point[2] = 100000;
	serch_point[3] = 100000;
	for (int serch = 0; serch < put_index; serch++)
	{

		serch_point[0] = sqrt(pow((o_all_point_map[serch].x - i_robot_start.x), 2) + pow((o_all_point_map[serch].y - i_robot_start.y), 2));
		serch_point[1] = sqrt(pow((o_all_point_map[serch].x - i_robot_end.x), 2) + pow((o_all_point_map[serch].y - i_robot_end.y), 2));


		if (serch_point[2] > serch_point[0])
		{
			serch_point[2] = serch_point[0];
			the_point_index[0] = serch;
		}
		if (serch_point[3] > serch_point[1])
		{
			serch_point[3] = serch_point[1];
			the_point_index[1] = serch;
		}
	}

	o_all_point_map.push_back(CPoint(i_robot_start.x, i_robot_start.y));
	CPoint_savepoint1.push_back(CPoint(i_robot_start.x, i_robot_start.y));
	o_all_point_map_original.push_back(cvPoint2D64f(i_robot_start.x / 10, i_robot_start.y / 10));
	CPoint_savepoint2.push_back(CPoint(o_all_point_map[the_point_index[0]].x, o_all_point_map[the_point_index[0]].y));
	i_new_input_index++;
	put_index++;

	for (loop1 = 0; loop1 < i_new_input_index; loop1++)
	{
		for (loop2 = 0; loop2 < put_index; loop2++)
		{

			if (CPoint_savepoint1[loop1] == o_all_point_map[loop2])
				savetemp_index1 = loop2;

			if (CPoint_savepoint2[loop1] == o_all_point_map[loop2])
				savetemp_index2 = loop2;

			if (w[loop1][loop2] == 0)
				w[loop1][loop2] = 50000;

		}

		w[savetemp_index1][savetemp_index2] = sqrt(pow(CPoint_savepoint1[loop1].x - CPoint_savepoint2[loop1].x, 2) + pow(CPoint_savepoint1[loop1].y - CPoint_savepoint2[loop1].y, 2));
		w[savetemp_index2][savetemp_index1] = w[savetemp_index1][savetemp_index2];
	}

	//	fstream app_jumppath_output("捷徑輸出.txt", ios::app);



	dijkstra(put_index - 1, o_all_point_map.size());
	find_path(the_point_index[1]);

}

void SHHuangDlg::Path_Optimization(vector<vector<bool>> i_sca_image, vector<CvPoint2D64f> i_all_point_map_original, vector<int>& o_path_optimization)
{

	int path_inside, break_index, line_distant;
	int cheak_x, cheak_y, cheak_index = 0;
	CvPoint2D64f cutout_point[2000];

	o_path_optimization.push_back(show_path[0]);

	for (int i = 0; i < show_path.size() - 1; i++)
	{
		for (int j = i + 1; j < show_path.size() - 1; j++)
		{

			line_distant = sqrt(pow(i_all_point_map_original[show_path[i]].x - i_all_point_map_original[show_path[j]].x, 2) + pow(i_all_point_map_original[show_path[i]].y - i_all_point_map_original[show_path[j]].y, 2));

			int cutout = 0;
			for (cutout = 0; cutout < line_distant; cutout++)
			{
				//計算斜線上的點
				if (i_all_point_map_original[show_path[i]].x >= i_all_point_map_original[show_path[j]].x && i_all_point_map_original[show_path[i]].y >= i_all_point_map_original[show_path[j]].y)
				{
					cutout_point[cutout].x = i_all_point_map_original[show_path[j]].x + (sqrt(pow((i_all_point_map_original[show_path[i]].x - i_all_point_map_original[show_path[j]].x), 2)) / (double)line_distant)*(double)cutout;
					cutout_point[cutout].y = i_all_point_map_original[show_path[j]].y + (sqrt(pow((i_all_point_map_original[show_path[i]].y - i_all_point_map_original[show_path[j]].y), 2)) / (double)line_distant)*(double)cutout;
				}

				if (i_all_point_map_original[show_path[i]].x >= i_all_point_map_original[show_path[j]].x && i_all_point_map_original[show_path[j]].y >= i_all_point_map_original[show_path[i]].y)
				{
					cutout_point[cutout].x = i_all_point_map_original[show_path[j]].x + (sqrt(pow((i_all_point_map_original[show_path[i]].x - i_all_point_map_original[show_path[j]].x), 2)) / (double)line_distant)*(double)cutout;
					cutout_point[cutout].y = i_all_point_map_original[show_path[j]].y - (sqrt(pow((i_all_point_map_original[show_path[i]].y - i_all_point_map_original[show_path[j]].y), 2)) / (double)line_distant)*(double)cutout;
				}

				if (i_all_point_map_original[show_path[j]].x >= i_all_point_map_original[show_path[i]].x && i_all_point_map_original[show_path[i]].y >= i_all_point_map_original[show_path[j]].y)
				{
					cutout_point[cutout].x = i_all_point_map_original[show_path[i]].x + (sqrt(pow((i_all_point_map_original[show_path[i]].x - i_all_point_map_original[show_path[j]].x), 2)) / (double)line_distant)*(double)cutout;
					cutout_point[cutout].y = i_all_point_map_original[show_path[i]].y - (sqrt(pow((i_all_point_map_original[show_path[i]].y - i_all_point_map_original[show_path[j]].y), 2)) / (double)line_distant)*(double)cutout;
				}

				if (i_all_point_map_original[show_path[j]].x >= i_all_point_map_original[show_path[i]].x && i_all_point_map_original[show_path[j]].y >= i_all_point_map_original[show_path[i]].y)
				{
					cutout_point[cutout].x = i_all_point_map_original[show_path[i]].x + (sqrt(pow((i_all_point_map_original[show_path[i]].x - i_all_point_map_original[show_path[j]].x), 2)) / (double)line_distant)*(double)cutout;
					cutout_point[cutout].y = i_all_point_map_original[show_path[i]].y + (sqrt(pow((i_all_point_map_original[show_path[i]].y - i_all_point_map_original[show_path[j]].y), 2)) / (double)line_distant)*(double)cutout;
				}
			}

			for (int cheak_pixel = 0; cheak_pixel < cutout; cheak_pixel++)
			{
				cheak_x = round(cutout_point[cheak_pixel].x);
				cheak_y = round(cutout_point[cheak_pixel].y);

				if ((cheak_y > i_sca_image.size() - 2) || (cheak_x > i_sca_image.size() - 2) || (cheak_y < 1 || (cheak_x < 1)))
					continue;

				if (i_sca_image[cheak_y][cheak_x] == 1 ||
					i_sca_image[cheak_y + 1][cheak_x] == 1 ||
					i_sca_image[cheak_y - 1][cheak_x] == 1 ||
					i_sca_image[cheak_y][cheak_x + 1] == 1 ||
					i_sca_image[cheak_y][cheak_x - 1] == 1 ||
					i_sca_image[cheak_y - 1][cheak_x - 1] == 1 ||
					i_sca_image[cheak_y + 1][cheak_x + 1] == 1 ||
					i_sca_image[cheak_y - 1][cheak_x + 1] == 1 ||
					i_sca_image[cheak_y + 1][cheak_x - 1] == 1/**/
					)
				{
					cheak_index = j - 1;  // 紀錄碰撞的上一點
					break;
				}

			}

			if (cheak_index)
			{
				o_path_optimization.push_back(show_path[cheak_index]);
				break;
			}

		}
		if (cheak_index == 0)
		{
			i = show_path.size();
			o_path_optimization.push_back(show_path[show_path.size() - 1]);
		}
		else
		{
			i = cheak_index;
			cheak_index = 0;
		}

	}
}

void SHHuangDlg::find_path(int x)   // 印出由起點到x點的最短路徑
{
	//	fstream app_truepath_output("路徑輸出.txt", ios::app);

	if (x != parent[x]) // 先把之前的路徑都印出來
		find_path(parent[x]);



	//	app_truepath_output << "x = " << x << " parent[x] = " << parent[x] << endl;
	//	app_truepath_output.close();
	show_path.push_back(x);
}

void SHHuangDlg::dijkstra(int source, int node_num)
{
	for (int i = 0; i < node_num; i++) visit[i] = false;   // initialize
	for (int i = 0; i < node_num; i++) d[i] = 100000000;

	d[source] = 0;   //令d[a]是起點到a點的最短路徑長度，起點設為零，其他點都是空的
	parent[source] = source;  // 紀錄各個點在最短路徑樹上的父親是誰

	for (int k = 0; k < node_num; k++)
	{
		int a = -1, b = -1, min = 100000000;
		for (int i = 0; i < node_num; i++)
			if (!visit[i] && d[i] < min)
			{
				a = i;  // 記錄這一條邊
				min = d[i];
			}

		if (a == -1) break;     // 起點有連通的最短路徑都已找完
		if (min == 1e9) break;  // 不連通即是最短路徑長度無限長
		visit[a] = true;

		// 以邊ab進行relaxation
		for (b = 0; b < node_num; b++)
			if (!visit[b] && d[a] + w[a][b] < d[b])
			{
				d[b] = d[a] + w[a][b];
				parent[b] = a;
			}
	}
}

void SHHuangDlg::DoMapSocketConnect() {

	// Sync the Panel data to the model
	UpdateData(true);

	// Read the TCP/IP address setting from User
	byte aIpAddressUnit[4];
	m_socket_ip_c.GetAddress(
		aIpAddressUnit[0], aIpAddressUnit[1], aIpAddressUnit[2], aIpAddressUnit[3]);
	CString aStrIpAddress;
	aStrIpAddress.Format(_T("%d.%d.%d.%d"),
		aIpAddressUnit[0], aIpAddressUnit[1], aIpAddressUnit[2], aIpAddressUnit[3]);

	m_socket_cmd.Close();
	if (!m_socket_cmd.Create()) {

		// If Socket Create Fail Report Message
		TCHAR szMsg[1024] = { 0 };
		wsprintf(szMsg, _T("Map socket create faild: %d"), m_socket_cmd.GetLastError());
		ReportSocketStatus(TCPEvent::CREATE_SOCKET_FAIL, CString("CMD Socket"));
		AfxMessageBox(szMsg);
	}
	else {

		ReportSocketStatus(TCPEvent::CREATE_SOCKET_SUCCESSFUL, CString("CMD Socket"));
		// Connect to the Server ( Raspberry Pi Server )
		fg_cmd_connected = m_socket_cmd.Connect(aStrIpAddress, m_socket_map_port);

	}

	// Create a TCP Socket for transfer Camera data
	if (!m_socket_map.Create()) {

		// If Socket Create Fail Report Message
		TCHAR szMsg[1024] = { 0 };
		wsprintf(szMsg, _T("Map socket create faild: %d"), m_socket_map.GetLastError());
		ReportSocketStatus(TCPEvent::CREATE_SOCKET_FAIL, CString("Map Socket"));
		AfxMessageBox(szMsg);
	}
	else {

		ReportSocketStatus(TCPEvent::CREATE_SOCKET_SUCCESSFUL, CString("Map Socket"));
		// Connect to the Server ( Raspberry Pi Server )
		fg_map_connected = m_socket_map.Connect(aStrIpAddress, m_socket_map_port);

		//fg_tcp_ip_read = true;
		//m_tcp_ip_IOHandle_thread = AfxBeginThread(TcpIODataHandler, LPVOID(this));

		//For Test
		m_socket_map.Send("Test from here", 14);
	}


	CString report_map(aStrIpAddress);
	report_map.Append(CString(" Map Socket"));

	if (fg_map_connected)
		ReportSocketStatus(TCPEvent::CONNECT_SUCCESSFUL, report_map);
	else {
		ReportSocketStatus(TCPEvent::CONNECT_FAIL, report_map);
		m_socket_map.Close();
	}

	CString report_cmd(aStrIpAddress);
	report_cmd.Append(CString(" Cmd Socket"));

	if (fg_cmd_connected)
		ReportSocketStatus(TCPEvent::CONNECT_SUCCESSFUL, report_cmd);
	else {
		ReportSocketStatus(TCPEvent::CONNECT_FAIL, report_cmd);
		m_socket_cmd.Close();
	}



}



void SHHuangDlg::DoLRFSocketConnect() {

	// Sync the Panel data to the model
	UpdateData(true);

	// Read the TCP/IP address setting from User
	byte aIpAddressUnit[4];
	m_socket_ip_c.GetAddress(
		aIpAddressUnit[0], aIpAddressUnit[1], aIpAddressUnit[2], aIpAddressUnit[3]);
	CString aStrIpAddress;
	aStrIpAddress.Format(_T("%d.%d.%d.%d"),
		aIpAddressUnit[0], aIpAddressUnit[1], aIpAddressUnit[2], aIpAddressUnit[3]);

	// Create a TCP Socket for transfer Camera data
	// m_tcp_socket.Create(m_tcp_ip_port, 1, aStrIpAddress);
	if (!m_socket_lrf.Create()) {

		// If Socket Create Fail Report Message
		TCHAR szMsg[1024] = { 0 };
		wsprintf(szMsg, _T("LRF socket create faild: %d"), m_socket_lrf.GetLastError());
		ReportSocketStatus(TCPEvent::CREATE_SOCKET_FAIL, CString("LRF Socket"));
		AfxMessageBox(szMsg);
	}
	else {

		ReportSocketStatus(TCPEvent::CREATE_SOCKET_SUCCESSFUL, CString("LRF Socket"));
		// Connect to the Server ( Raspberry Pi Server )
		fg_lrf_connected = m_socket_lrf.Connect(aStrIpAddress, m_socket_lrf_port);

		//fg_tcp_ip_read = true;
		//m_tcp_ip_IOHandle_thread = AfxBeginThread(TcpIODataHandler, LPVOID(this));

		//For Test
		m_socket_lrf.Send("Test from here", 14);
	}

	if (fg_lrf_connected)
		ReportSocketStatus(TCPEvent::CONNECT_SUCCESSFUL, aStrIpAddress);
	else {
		ReportSocketStatus(TCPEvent::CONNECT_FAIL, aStrIpAddress);
		m_socket_lrf.Close();
	}



}



void SHHuangDlg::DoPoseSocketConnect() {
	// Sync the Panel data to the model
	UpdateData(true);

	// Read the TCP/IP address setting from User
	byte aIpAddressUnit[4];
	m_socket_ip_c.GetAddress(
		aIpAddressUnit[0], aIpAddressUnit[1], aIpAddressUnit[2], aIpAddressUnit[3]);
	CString aStrIpAddress;
	aStrIpAddress.Format(_T("%d.%d.%d.%d"),
		aIpAddressUnit[0], aIpAddressUnit[1], aIpAddressUnit[2], aIpAddressUnit[3]);

	// Create a TCP Socket for transfer Camera data
	// m_tcp_socket.Create(m_tcp_ip_port, 1, aStrIpAddress);
	if (!m_socket_pose.Create()) {

		// If Socket Create Fail Report Message
		TCHAR szMsg[1024] = { 0 };
		wsprintf(szMsg, _T("Pose socket create faild: %d"), m_socket_pose.GetLastError());
		ReportSocketStatus(TCPEvent::CREATE_SOCKET_FAIL, CString("Pose Socket"));
		AfxMessageBox(szMsg);
	}
	else {

		ReportSocketStatus(TCPEvent::CREATE_SOCKET_SUCCESSFUL, CString("Pose Socket"));
		// Connect to the Server ( Raspberry Pi Server )
		fg_pose_connected = m_socket_pose.Connect(aStrIpAddress, m_socket_pose_port);

		//For Test
		m_socket_pose.Send("Test from here", 14);
	}

	if (fg_pose_connected)
		ReportSocketStatus(TCPEvent::CONNECT_SUCCESSFUL, aStrIpAddress);
	else
	{
		ReportSocketStatus(TCPEvent::CONNECT_FAIL, aStrIpAddress);
		m_socket_pose.Close();
	}
}



void SHHuangDlg::DoMapSocketDisconnect() {
	// Setup Connect-staus flag
	fg_map_connected = false;
	//fg_tcp_ip_read = false;

	// Close the TCP/IP Socket
	m_socket_map.Close();

	// Report TCP/IP connect status
	CString tmp_log; tmp_log.Format(_T("I/O event: %s"), _T("Close map Socket"));
	m_socket_log_c.AddString(tmp_log);
}



void SHHuangDlg::DoLRFSocketDisconnect() {
	// Setup Connect-staus flag
	fg_lrf_connected = false;
	//fg_tcp_ip_read = false;

	// Close the TCP/IP Socket
	m_socket_lrf.Close();

	// Report TCP/IP connect status
	CString tmp_log; tmp_log.Format(_T("I/O event: %s"), _T("Close LRF Socket"));
	m_socket_log_c.AddString(tmp_log);
}



void SHHuangDlg::DoPoseSocketDisconnect() {
	// Setup Connect-staus flag
	fg_pose_connected = false;
	//fg_tcp_ip_read = false;

	// Close the TCP/IP Socket
	m_socket_pose.Close();

	// Report TCP/IP connect status
	CString tmp_log; tmp_log.Format(_T("I/O event: %s"), _T("Close Pose Socket"));
	m_socket_log_c.AddString(tmp_log);
}



void SHHuangDlg::ReportSocketStatus(TCPEvent event_, CString &msg) {
	CString tmp_log;
	CTime current_time = CTime::GetCurrentTime();
	tmp_log.Format(_T("[ %02d點%02d分%02d秒: ] "),
		current_time.GetHour(), current_time.GetMinute(), current_time.GetSecond());
	switch (event_) {
	case CREATE_SOCKET_SUCCESSFUL:
		tmp_log.AppendFormat(_T("I/O event: %s <%s>"),
			_T("Create Socket Successful"), msg);
		break;
	case CREATE_SOCKET_FAIL:
		tmp_log.AppendFormat(_T("I/O event: %s"),
			_T("Create Socket Fail <%s>"), msg);
		break;
	case CONNECT_SUCCESSFUL:
		tmp_log.AppendFormat(_T("I/O event: %s%s%s"),
			_T("Connect "), msg, _T(" Successful"));
		break;
	case CONNECT_FAIL:
		tmp_log.AppendFormat(_T("I/O event: %s%s%s"),
			_T("Connect "), msg, _T(" Fail"));
		break;
	case DISCONNECT:
		tmp_log.AppendFormat(_T("I/O event: %s"),
			_T("Disconnect"));
		break;
	case SEND_MESSAGE_SUCCESSFUL:
		tmp_log.AppendFormat(_T("I/O event: %s%s%s"),
			_T("Sent Message"), msg, _T("Successful"));
		break;
	case SENT_MESSAGE_FAIL:
		tmp_log.AppendFormat(_T("I/O event: %s%s%s"),
			_T("Sent Message"), msg, _T("Fail"));
		break;
	}
	m_socket_log_c.AddString(tmp_log);
}


void CPSocket::OnReceive(int nErrorCode) {
	static int counter(0);
	counter++;

	if (0 == nErrorCode) {

		static int i = 0;
		i++;

		int nRead(0);

		volatile int cbLeft(sizeof(PoseData)); // 4 Byte
		volatile int cbDataReceived(0);
		int cTimesRead(0);
		do {


			// Determine Socket State
			nRead = Receive(&m_pose_data + cbDataReceived, cbLeft);

			if (nRead == SOCKET_ERROR) {
				if (GetLastError() != WSAEWOULDBLOCK) {
					//AfxMessageBox(_T("Error occurred"));
					Close();
					// Trying to reconnect
					((SHHuangDlg*)m_parent)->DoPoseSocketConnect();
					return;
				}
				break;
			}


			cbLeft -= nRead;
			cbDataReceived += nRead;
			cTimesRead++;
		} while (cbLeft > 0 && cTimesRead < 50);
	}

	m_pose_data_2 = m_pose_data;
	//rotation_AMonte_Carlo
	if (rotation_AMonte_Carlo)
	{
		m_pose_data_2.pose_orientation[2] = m_pose_data_2.pose_orientation[2] - CV_PI / 2;
	}

	CSocket::OnReceive(nErrorCode);
}

void CPSocket::registerParent(CWnd* _parent) {
	m_parent = _parent;
}

void CCRLSocket::OnReceive(int nErrorCode) {
	static int counter(0);
	counter++;

	if (0 == nErrorCode) {

		static int i = 0;
		i++;

		int nRead(0);

		volatile int cbLeft(sizeof(ControlMsg)); // 4 Byte
		volatile int cbDataReceived(0);
		int cTimesRead(0);
		do {


			// Determine Socket State
			nRead = Receive(&m_cmd_msg + cbDataReceived, cbLeft);


#ifdef _SHOW_SOCKET_DEBUG

			/****** Print Socket status inorder to debug ****/
			// 			CString msg;
			// 			msg.Format(_T("Control Msg Recivced: %d"), counter);
			// 			m_parent->SetDlgItemTextW(IDC_RECE_STATUS2, msg);
			// 			m_parent->UpdateData(false);
			/**********************************************/

			if (cbLeft < 0) AfxMessageBox(_T("WTF occurred"));
#endif // _SHOW_SOCKET_DEBUG


			if (nRead == SOCKET_ERROR) {
				if (GetLastError() != WSAEWOULDBLOCK) {
					//AfxMessageBox(_T("Error occurred"));
					Close();
					// Trying to reconnect
					((SHHuangDlg*)m_parent)->DoMapSocketConnect();
					return;
				}
				break;
			}


			cbLeft -= nRead;
			cbDataReceived += nRead;
			cTimesRead++;
		} while (cbLeft > 0 && cTimesRead < 50);
	}

	CSocket::OnReceive(nErrorCode);
}



void CCRLSocket::registerParent(CWnd* _parent) {
	m_parent = _parent;
}

void CMSocket::OnReceive(int nErrorCode) {

	static unsigned int counter(0);
	MapDataPartPtr ptrMap(new MapDataPart);

	if (0 == nErrorCode) {

		static int i = 0;
		i++;

		int nRead(0);

		volatile int cbLeft(sizeof(MapDataPart)); // 4 Byte
		volatile int cbDataReceived(0);
		int cTimesRead(0);
		do {


			// Determine Socket State
			nRead = Receive(ptrMap.get() + cbDataReceived, cbLeft);

			m_map_parts.push_back(ptrMap);

#ifdef _SHOW_SOCKET_DEBUG

			/****** Print Socket status inorder to debug ****/
			// 			CString msg;
			// 			msg.Format(_T("Stamp: %d Rec count: %d, Left count: %d"), ptrMap->info.part_stamp, nRead, cbLeft);
			// 			m_parent->SetDlgItemTextW(IDC_RECE_STATUS, msg);
			// 			m_parent->UpdateData(false);
			/**********************************************/

			if (cbLeft < 0) AfxMessageBox(_T("WTF occurred"));
#endif // _SHOW_SOCKET_DEBUG


			if (nRead == SOCKET_ERROR) {
				if (GetLastError() != WSAEWOULDBLOCK) {
					//AfxMessageBox(_T("Error occurred"));
					Close();
					((SHHuangDlg*)m_parent)->DoMapSocketConnect();
					return;
				}
				break;
			}


			cbLeft -= nRead;
			cbDataReceived += nRead;
			cTimesRead++;
		} while (cbLeft > 0 && cTimesRead < 50);



	}

	if (ptrMap->info.part_stamp == (MAP_PART_COUNT - 1)*MAP_PER_CUT_SIZE) {
		ptrMap.reset();
		assembleData();
		// Show the data to View module
	}

	//if (m_map_parts.size() == MAP_PART_COUNT) {
	//	assembleData();
	//	// Show the data to View module
	//	showData();
	//}



	CSocket::OnReceive(nErrorCode);
}

void CMSocket::assembleData() {

	// Copy the Meta data
	m_map_data->info.map_stamp = m_map_parts.at(0)->info.map_stamp;
	m_map_data->info.res = m_map_parts.at(0)->info.res;
	m_map_data->info.height = m_map_parts.at(0)->info.height;
	m_map_data->info.width = m_map_parts.at(0)->info.width;
	m_map_data->info.origin_x = m_map_parts.at(0)->info.origin_x;
	m_map_data->info.origin_y = m_map_parts.at(0)->info.origin_y;
	m_map_data->info.origin_yaw = m_map_parts.at(0)->info.origin_yaw;

	// Copy the map data
	int i(0);
	//if (m_map_parts.size() != MAP_PART_COUNT) {
	//	m_map_parts.clear();
	//	return;
	//}
	for (MapDataPartArray::iterator it = m_map_parts.begin();
		it != m_map_parts.end();
		it++, i++) {

		// Get each map part data pointer
		uint8_t* ptrData = it->get()->data;
		size_t offset = it->get()->info.part_stamp;
		if (!(offset > (MAP_PART_COUNT - 1)*MAP_PER_CUT_SIZE)) {
			memcpy(m_map_data->data + offset, ptrData, MAP_PER_CUT_SIZE);
		}
	}

	// Clear all data
	m_map_parts.clear();

}

void CMSocket::registerParent(CWnd* _parent) {
	m_parent = _parent;
}

void CRSocket::OnReceive(int nErrorCode)
{
	return;
	if (0 == nErrorCode) {
		static int i = 0;
		i++;

		int nRead(0);

		volatile int cbLeft(SIZE_LRF_DATA * 4); // 4 Byte
		volatile int cbDataReceived(0);
		int cTimesRead(0);
		do {


			// Determine Socket State
			nRead = Receive((byte*)&m_lrf_data + cbDataReceived, cbLeft);

			//nRead = Receive(this->image_buffer, 1460);
			//memcpy_s(m_remote_image.data + cbDataReceived, nRead, image_buffer, nRead);
			//nRead = Receive(m_remote_image.data + cbDataReceived, 1460);

			if (nRead == SOCKET_ERROR) {
				if (GetLastError() != WSAEWOULDBLOCK) {
					//AfxMessageBox(_T("Error occurred"));
					Close();
					// Trying to reconnect
					((SHHuangDlg*)m_parent)->DoLRFSocketConnect();
				}
				break;
			}

			//memcpy(m_remote_image.data + cbDataReceived, image_buffer, nRead);

			cbLeft -= nRead;
			cbDataReceived += nRead;
			cTimesRead++;
		} while (cbLeft > 0 && cTimesRead < 50);

		//ClearTCPBuffer();
		//ShowRemoteImage();
	}
	//ShowRemoteImage();

	memcpy(m_lrf_data_2, m_lrf_data, SIZE_LRF_DATA);
	CSocket::OnReceive(nErrorCode);
}

void CRSocket::registerParent(CWnd* _parent) {
	m_parent = _parent;
}

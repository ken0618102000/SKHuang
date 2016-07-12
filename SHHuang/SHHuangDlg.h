// SHHuangDlg.h : ���Y��
//

#pragma once


#include "..//RoboticsPlatform//VideoCapture//DxCapture.h"
#include "..//SLAM//SLAM.h"
#include "afxwin.h"
#include <direct.h>
#include <fstream>
#include <iomanip>
#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <vector>
#include <highgui.h> //CImage �K��
#include <direct.h> //_getcwd ���|
#include "RSocket.h"
//*****************************************************************************************�h�C��p�ɾ�
#include <windows.h>
#include <mmsystem.h>
#include "afxcmn.h"
#pragma comment( lib, "winmm.lib" )
//*****************************************************************************************

using namespace std;

struct draw_car
{
	CvPoint car[6];
	vector <CvPoint>sim_path;
};
// CSHHuangDlg ��ܤ��
class SHHuangDlg : public CDialog
{
// �غc
public:
	SHHuangDlg(CWnd* pParent = NULL);	// �зǫغc�禡

// ��ܤ�����
	enum { IDD = IDD_SHHuang_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �䴩


// �{���X��@
protected:
	HICON m_hIcon;

	// ���ͪ��T�������禡
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

	enum TCPEvent {
		CREATE_SOCKET_SUCCESSFUL,
		CREATE_SOCKET_FAIL,
		CONNECT_SUCCESSFUL,
		CONNECT_FAIL,
		DISCONNECT,
		SEND_MESSAGE_SUCCESSFUL,
		SENT_MESSAGE_FAIL
	};

public:
	void DoMapSocketConnect();
	void DoLRFSocketConnect();
	void DoPoseSocketConnect();
	void DoMapSocketDisconnect();
	void DoLRFSocketDisconnect();
	void DoPoseSocketDisconnect();
	void ReportSocketStatus(TCPEvent event_, CString &msg = CString(""));
	bool fg_map_connected;
	bool fg_lrf_connected;
	bool fg_cmd_connected;
	bool fg_pose_connected;
	CMSocket m_socket_map;
	CRSocket m_socket_lrf;
	CCRLSocket m_socket_cmd;
	CPSocket m_socket_pose;
	UINT m_socket_map_port;
	UINT m_socket_lrf_port;
	UINT m_socket_pose_port;


	static bool check_updatedata;
public:
	CComboBox m_L_SelectCamera_c;
	CComboBox m_R_SelectCamera_c;
	afx_msg void OnBnClickedLInitialccd();
	afx_msg void OnBnClickedLOptionccd();
	afx_msg void OnBnClickedLCloseccd();
	afx_msg void OnBnClickedRInitialccd();
	afx_msg void OnBnClickedROptionccd();
	afx_msg void OnBnClickedRCloseccd();
	CStatic m_L_Image_Live_c;
	CStatic m_R_Image_Live_c;
	CStatic m_L_Image_SURF_c;
	CStatic m_R_Image_SURF_c;
	CButton m_SaveImage_Mode_c;
	CButton m_OnLine_Mode_c;
	CButton m_Start_c;
	CButton m_Pause_c;
	CButton m_Stop_c;
	CButton m_Continue_c;
	afx_msg void OnBnClickedStart();
	CButton m_LoadImage_Mode_c;
	CStatic m_Image_PhotoCount_c;
	CStatic m_Map_c;

	double m_SampleTime;
	int m_Frequency;
	int m_Frequency2;
	int m_Time1;
	int m_Time2;
	int m_Time3;
	int m_Time4;
	double m_cameraSitaX;
	double m_cameraSitaY;
	double m_cameraSitaZ;
	double m_cameraX;
	double m_cameraY;
	double m_cameraZ;

	double time1;
	double time2;
	double time3;
	double time4;
	double time5;
	double time6;
	double time7;
	double time8;
	double time9;


	void DoEvent();
	void DeletePhoto();
    void DeletePhoto2();
	void WORK( const IplImage* l_image, const IplImage* r_image, const double SampleTime );
	void ShowImage( const IplImage* image, CWnd* pWnd );
	void save_car_pos();
	void find_path(int x);
	void dijkstra(int source, int node_num);
	void binarization(IplImage *i_show_data, vector<vector<bool>>  &o_sca_image2);
	void find_coner(vector<vector<bool>> i_sca_image, vector <cv::Point> &o_save_coner, int i_Interpolation);
	void trans2Voronoi(vector<vector<bool>> i_sca_image, vector <cv::Point> i_save_coner, double(&o_Data)[8000], int i_Interpolation2);
	void Voronoi_calculate(double i_Data[8000], int x_boundary, int y_boundary, CvPoint2D64f(&o_savepoint1)[3000], CvPoint2D64f(&o_savepoint2)[3000], int &o_line_count);
	void Generalized_Voronoi(vector<vector<bool>> i_sca_image, CvPoint2D64f i_savepoint1[3000], CvPoint2D64f i_savepoint2[3000], int i_line_count, int &o_new_input_index, CvPoint2D64f(&o_new_savepoint1)[3000], CvPoint2D64f(&o_new_savepoint2)[3000]);
	void Match_point(int i_line_count, int i_new_input_index, CvPoint2D64f(&io_new_savepoint1)[3000], CvPoint2D64f(&io_new_savepoint2)[3000], float near_dis);
	void Dijkstra_path_planning(CvPoint i_robot_start, CvPoint  i_robot_end, CvPoint2D64f i_new_savepoint1[3000], CvPoint2D64f i_new_savepoint2[3000], int i_new_input_index, vector <CPoint> &o_all_point_map, vector <CvPoint2D64f> &o_all_point_map_original);
	void Path_Optimization(vector<vector<bool>> i_sca_image, vector <CvPoint2D64f> i_all_point_map_original, vector <int> &o_path_optimization);
	void Control_Methods(bool control_type, double i_rho, double i_alpha, double i_beta, double i_phi, double &o_vr, double &o_vl, int &o_state);
	static double scale;
	int show_image_num;
    int show_map_num;
	int show_feature_region;
	int show_Search_Window_Size;
	int show_all_feature;
	int txt;


	void ShowPhotoCount( const int PhotoCount, CWnd* pWnd );
	afx_msg void OnBnClickedPause();
	afx_msg void OnBnClickedStop();
	afx_msg void OnBnClickedContinue();

	
	CRect rect_map;
	CWnd* pWnd_map;

	int PhotoCount;

	int move_time; //�~
	int delay;
	int active_times;//��
	int move_active[8];
	int move_turn;
	int p3dxdis;

	double SampleTime;
	vector<double> SampleTime_temp;
	static vector<Keep_Feature> draw_feature;
	static vector<CvPoint2D64f> Path;
	static vector <vector<Keep_Feature>> feature_path;
	static bool carFLAG;
	static bool Click_Left_Button;
	static void I90_PWM_send(int L_PWM, int R_PWM);
	static unsigned char checksun(int nStar, int nEnd);
	static CvPoint2D64f orgin;
	static vector <CPoint> jump_path_optimization;
	static vector <CPoint> jump_path_optimization_copy;
	static CPoint jump_path_optimization_save;
	static vector <draw_car> local_draw_car;
	static vector <draw_car> static_draw_car;
	SLAM BinocularSLAM;


	
	MMRESULT FTimerID; // �h�C��p�ɾ�

	bool L_InitialCCD;
	bool R_InitialCCD;


	CButton m_L_InitialCCD_c;
	CButton m_R_InitialCCD_c;
	CButton m_L_OptionCCD_c;
	CButton m_R_OptionCCD_c;
	CButton m_L_CloseCCD_c;
	CButton m_R_CloseCCD_c;
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButtonbig();
	afx_msg void OnBnClickedButtonsmall();
	afx_msg void OnBnClickedButtonUp();
	afx_msg void OnBnClickedButtonDown();
	afx_msg void OnBnClickedButtonLeft();
	afx_msg void OnBnClickedButtonRight();
	afx_msg void OnBnClickedButtonReset();

	//20160131
	int m_FeatureNum;
	double m_X_shift;
	double m_Y_shift;
	double m_Z_shift;
	afx_msg void OnBnClickedButtonReadfeature();
	static double Camera[6];
	static double map_move[6]; //���ʤp�a�ϥ�0,1���ܭy��P�S�x�I�ΡA2,3,4,5���e���|�W���ϥΡA4,5�ΨӲ��ʹϹ�
	static double initial_scale;
	static double vr, vl;    //�n����I90��@�w��ɩҥ�
	static double vr_draw, vl_draw;
	static double car_x, car_y, car_zdir;
	static double target_pos[3];
	static int sampleTime;
	static int state;
	static double IDC_Map_Rect_Map[4];  //��ʵ��wMap��m�P�j�p�A�쥻rect������
	static CPoint Wheel_mouse_pos;
	static CPoint LButtonDown_mouse_pos;
	static bool stop_everthing;
	static int jump_num;
	static int path_optimization_size_change;

	static IplImage * Read_LRF_Map_static;
	static IplImage * Draw_Path_static;

	static int d[2000];       // �����_�I��U���I���̵u���|����
	static int parent[2000];  // �����U���I�b�̵u���|��W�����ˬO��
	static bool visit[2000];  // �����U���I�O���O�w�b�̵u���|�𤧤�
	static int w[2000][2000];    // �@�i���v������
	static vector <int> show_path;
	//---------thread car_draw------------
private:
	CWinThread * m_pThread_car_draw;
	bool Continue_car_draw;
	static UINT ThreadFun_car_draw(LPVOID lParam);
	//---------thread TARGET_control------------
private:
	CWinThread * m_pThread_TARGET_control;
	bool Continue_TARGET_control;
	static UINT ThreadFun_TARGET_control(LPVOID lParam);
	//---------thread Path_Planning------------
private:
	CWinThread * m_pThread_Path_Planning;
	bool Continue_Path_Planning;
	static UINT ThreadFun_Path_Planning(LPVOID lParam);


public:
	afx_msg void OnBnClickedSocketConnect();
	afx_msg void OnBnClickedButtonConnectI90();
	BOOL PreTranslateMessage(MSG* pMsg);
	CIPAddressCtrl m_socket_ip_c;
	CListBox m_socket_log_c;
	CButton m_socket_connect_c;
	CButton m_read_map_c;
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	CButton m_FollowMode_c;
	CButton m_LoadImage_Mode_Laser_c;
	afx_msg void OnBnClickedButtonSimulation();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnBnClickedButtonPathplanning();
	CButton m_PathPlanning_c;
	
};



class float_point
{
	float x;
	float y;
};

class CElement : public CObject
{
	DECLARE_SERIAL(CElement)

protected:
	COLORREF m_Color;                       // Color of an element
	CRect m_EnclosingRect;                  // Rectangle enclosing an element
	int m_Pen;                              // Pen width

public:
	virtual ~CElement() {}                   // Virtual destructor

											 // Virtual draw operation
	virtual void Draw(CDC* pDC, const CElement* pElement = 0) const {}
	virtual void Move(const CSize& Size) {} // Move an element
	CRect GetBoundRect() const;             // Get the bounding rectangle for an element

	virtual void Serialize(CArchive& ar);      // Serialize function for CElement

protected:
	CElement() {}                            // Default constructor
};


// Class defining a vor_point object
class CVor_Point : public CElement
{
	DECLARE_SERIAL(CVor_Point)

public:
	int GetPositionY();
	int GetPositionX();
	// Function to display a line
	virtual void Draw(CDC* pDC, const CElement* pElement = 0) const;
	virtual void Move(const CSize& aSize);       // Function to move an element

												 // Constructor for a line object
	CVor_Point(const CPoint& Start, const COLORREF& Color, const int& PenWidth);

	virtual void Serialize(CArchive& ar);      // Serialize function for CVor_Point

protected:
	CPoint m_StartPoint;          // Start point of line

	CVor_Point() {}             // Default constructor - should not be used
};

class SK_point
{
public:
	int x;
	int y;
	float distant;
};

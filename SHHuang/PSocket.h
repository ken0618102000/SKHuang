#pragma once
#include "afxsock.h"
#include "../server/include/PoseDataStruct.h"

class CPSocket :
	public CSocket {
public:
	CPSocket();
	virtual ~CPSocket();
	virtual void OnReceive(int nErrorCode);
	void registerParent(CWnd* _parent);
	PoseData m_pose_data;
	static PoseData m_pose_data_2;
	CWnd* m_parent;
};
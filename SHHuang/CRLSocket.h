#pragma once
#include "afxsock.h"
#include "../server/include/ControlDataStruct.h"

#include "E:\boost_1_62_0\boost\shared_ptr.hpp"

#include <vector>

class CCRLSocket :
	public CSocket {
public:
	CCRLSocket();
	~CCRLSocket();
	virtual void OnReceive(int nErrorCode);
	void registerParent(CWnd* _parent);
	CWnd* m_parent;
	struct ControlMsg m_cmd_msg;
};


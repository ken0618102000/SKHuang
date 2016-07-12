#pragma once
#include "afxsock.h"


#define SIZE_LRF_DATA 510
class CRSocket :
	public CSocket {
public:
	CRSocket();
	virtual ~CRSocket();
	virtual void OnReceive(int nErrorCode);
	float m_lrf_data[SIZE_LRF_DATA];
	static float m_lrf_data_2[SIZE_LRF_DATA];
	void registerParent(CWnd* _parent);

	CWnd* m_parent;
};


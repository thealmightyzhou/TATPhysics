#pragma once
#include <iostream>
struct TATBVNode;

class TATBvhCollideCallBack
{
public:
	virtual ~TATBvhCollideCallBack() {}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2) {}
};
#pragma once
#include <iostream>
class TATBVNode;

class TATBvhCollideCallBack
{
public:
	virtual ~TATBvhCollideCallBack() {}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2) {}
};
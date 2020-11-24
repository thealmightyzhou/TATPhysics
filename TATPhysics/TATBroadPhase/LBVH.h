#pragma once
#include "../TATCommon/TATVector3.h"
#include <iostream>
#include <algorithm>
#include <map>
#include <stack>
#include <queue>

struct LBVNode
{
public:
	LBVNode(const TATVector3& min, const TATVector3& max) :m_Min(min), m_Max(max)
	{
		m_Center = (min + max) / 2;
		m_IsInternal = false;
		m_Children[0] = m_Children[1] = m_Parent = 0;
		m_UserData = 0;
	}

	LBVNode()
	{
		m_IsInternal = false;
		m_Children[0] = m_Children[1] = m_Parent = 0;
		m_UserData = 0;
	}

	bool operator<(const LBVNode& c)
	{
		return m_MortonCode < c.m_MortonCode;
	}

	unsigned int ExpandBits(unsigned int v)
	{
		v = (v * 0x00010001u) & 0xFF0000FFu;
		v = (v * 0x00000101u) & 0x0F00F00Fu;
		v = (v * 0x00000011u) & 0xC30C30C3u;
		v = (v * 0x00000005u) & 0x49249249u;
		return v;
	}

	// Calculates a 30-bit Morton code for the
	// given 3D point located within the unit cube [0,1].
	void GenMorton()
	{
		float x = m_UnitCenter.X;
		float y = m_UnitCenter.Y;
		float z = m_UnitCenter.Z;

		x = _Min(_Max(x * 1024.0f, 0.0f), 1023.0f);
		y = _Min(_Max(y * 1024.0f, 0.0f), 1023.0f);
		z = _Min(_Max(z * 1024.0f, 0.0f), 1023.0f);
		unsigned int xx = ExpandBits((unsigned int)x);
		unsigned int yy = ExpandBits((unsigned int)y);
		unsigned int zz = ExpandBits((unsigned int)z);
		m_MortonCode = xx * 4 + yy * 2 + zz;
	}

	bool m_IsInternal;
	TATVector3 m_Min, m_Max;
	TATVector3 m_Center;
	TATVector3 m_UnitCenter;
	LBVNode* m_Children[2];
	LBVNode* m_Parent;
	void* m_UserData;
	UINT m_MortonCode;
};

class LBVH
{
public:
	LBVH()
	{
		m_Root = 0;
	}

	LBVNode* InsertAABB(const TATVector3& min, const TATVector3& max)
	{
		m_StoreNodes.push_back(LBVNode(min, max));
		m_Positions.push_back((min + max) / 2);
		return &m_StoreNodes[m_StoreNodes.size() - 1];
	}

	void Build();

	void TraverseTreeB2T()
	{
		if (!m_Root)
			return;

		std::stack<LBVNode*> stk;
		std::queue<LBVNode*> que;
		que.push(m_Root);

		LBVNode* node;
		while (que.size() > 0)
		{
			node = que.front();
			que.pop();
			stk.push(node);

			if (node->m_Children[0] && node->m_Children[0]->m_IsInternal)
				que.push(node->m_Children[0]);
			if (node->m_Children[1] && node->m_Children[1]->m_IsInternal)
				que.push(node->m_Children[1]);
		}

		while (stk.size() > 0)
		{
			node = stk.top();
			stk.pop();

			node->m_Min = TATVector3::MakeMin(node->m_Children[0]->m_Min, node->m_Children[1]->m_Min);
			node->m_Max = TATVector3::MakeMax(node->m_Children[0]->m_Max, node->m_Children[1]->m_Max);
		}
	}

	std::vector<LBVNode> m_InternalNodes;
	std::vector<LBVNode> m_StoreNodes;
	std::vector<TATVector3> m_Positions;
	std::map<UINT, LBVNode*> m_MapNodes;

	LBVNode* m_Root;
};

#pragma once
#include "../TATCommon/TATVector3.h"
#include <iostream>
#include <algorithm>
#include <map>
#include <stack>
#include <queue>

struct LBVPrim
{
public:
	LBVPrim(const TATVector3& min, const TATVector3& max) :m_Min(min), m_Max(max)
	{
		m_Center = (min + max) / 2;
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

	bool IsOverlapped(LBVPrim* other)
	{
		if (!other)
			return false;
		bool overlapped = true;

		if (other->m_Max.X < this->m_Min.X || other->m_Min.X > this->m_Max.X)
			overlapped = false;
		if (other->m_Max.Y < this->m_Min.Y || other->m_Min.Y > this->m_Max.Y)
			overlapped = false;
		if (other->m_Max.Z < this->m_Min.Z || other->m_Min.Z > this->m_Max.Z)
			overlapped = false;

		return overlapped;
	}

	bool operator<(const LBVPrim& c)
	{
		return m_MortonCode < c.m_MortonCode;
	}

	TATVector3 m_Min, m_Max;
	TATVector3 m_Center;
	TATVector3 m_UnitCenter;
	UINT m_MortonCode;
	void* m_UserData;

};

struct LBVNode
{
public:
	LBVNode(const TATVector3& min, const TATVector3& max) :m_Min(min), m_Max(max)
	{
		m_IsInternal = false;
		m_Children[0] = m_Children[1] = m_Parent = 0;
		m_Ptr = this;
	}

	LBVNode()
	{
		m_IsInternal = true;
		m_Children[0] = m_Children[1] = m_Parent = 0;
		m_Ptr = this;
	}

	LBVNode(const LBVNode& c)
	{
		m_IsInternal = c.m_IsInternal;
		m_Min = c.m_Min;
		m_Max = c.m_Max;

		m_Children[0] = c.m_Children[0];
		m_Children[1] = c.m_Children[1];
		m_Parent = c.m_Parent;
		m_MortonCode = c.m_MortonCode;

		m_Ptr = this;
		m_Prims = c.m_Prims;
	}

	bool IsOverlapped(LBVNode* other)
	{
		if (!other)
			return false;
		bool overlapped = true;

		if (other->m_Max.X < this->m_Min.X || other->m_Min.X > this->m_Max.X)
			overlapped = false;
		if (other->m_Max.Y < this->m_Min.Y || other->m_Min.Y > this->m_Max.Y)
			overlapped = false;
		if (other->m_Max.Z < this->m_Min.Z || other->m_Min.Z > this->m_Max.Z)
			overlapped = false;

		return overlapped;
	}

	void Update()
	{
		m_Min = TAT_MAXVECTOR3;
		m_Max = -TAT_MAXVECTOR3;

		for (int i = 0; i < m_Prims.size(); ++i) 
		{
			m_Min.SetMin(m_Prims[i].m_Min);
			m_Max.SetMax(m_Prims[i].m_Max);
		}

		m_MortonCode = m_Prims[0].m_MortonCode;
	}

	bool m_IsInternal;
	TATVector3 m_Min, m_Max;

	LBVNode* m_Children[2];
	LBVNode* m_Parent;
	UINT m_MortonCode;

	LBVNode * m_Ptr;
	std::vector<LBVPrim> m_Prims;
};

class LBVHCollideCallBack
{
public:

	virtual ~LBVHCollideCallBack() {}
	virtual void HandleNodeOverlap(LBVPrim* node1, LBVPrim* node2) {}
};

class LBVH
{
public:
	struct OverlapPair
	{
	public:
		OverlapPair(LBVNode* n1, LBVNode* n2) :m_Node1(n1), m_Node2(n2)
		{}

		OverlapPair() {}

		LBVNode* m_Node1;
		LBVNode* m_Node2;
	};

	LBVH()
	{
		m_Root = 0;
	}

	LBVPrim* InsertAABB(const TATVector3& min, const TATVector3& max)
	{
		m_StorePrims.push_back(LBVPrim(min, max));
		m_Positions.push_back((min + max) / 2);
		return &m_StorePrims[m_StorePrims.size() - 1];
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
			{
				que.push(node->m_Children[0]);
			}

			if (node->m_Children[1] && node->m_Children[1]->m_IsInternal)
			{
				que.push(node->m_Children[1]);
			}
		}

		while (stk.size() > 0)
		{
			node = stk.top();
			stk.pop();

			node->m_Min = TATVector3::MakeMin(node->m_Children[0]->m_Min, node->m_Children[1]->m_Min);
			node->m_Max = TATVector3::MakeMax(node->m_Children[0]->m_Max, node->m_Children[1]->m_Max);
		}
	}

	void CollideWithBVH(const LBVH& bvh, LBVHCollideCallBack* cb)
	{
		CollideWithNode(bvh.m_Root, cb);
	}

	void CollideWithNode(LBVNode* node, LBVHCollideCallBack* cb)
	{
		std::stack<OverlapPair> stk;

		stk.push(OverlapPair(m_Root, node));
		LBVPrim* p1, * p2;
		OverlapPair pair;
		while (stk.size() > 0)
		{
			pair = stk.top();
			stk.pop();

			if (pair.m_Node1 == pair.m_Node2)
				continue;

			if (pair.m_Node1->IsOverlapped(pair.m_Node2))
			{
				if (pair.m_Node1->m_IsInternal && pair.m_Node2->m_IsInternal)
				{
					stk.push(OverlapPair(pair.m_Node1->m_Children[0], pair.m_Node2->m_Children[0]));
					stk.push(OverlapPair(pair.m_Node1->m_Children[0], pair.m_Node2->m_Children[1]));
					stk.push(OverlapPair(pair.m_Node1->m_Children[1], pair.m_Node2->m_Children[0]));
					stk.push(OverlapPair(pair.m_Node1->m_Children[1], pair.m_Node2->m_Children[1]));
				}
				else if (pair.m_Node1->m_IsInternal && !pair.m_Node2->m_IsInternal)
				{
					stk.push(OverlapPair(pair.m_Node1->m_Children[0], pair.m_Node2));
					stk.push(OverlapPair(pair.m_Node1->m_Children[1], pair.m_Node2));
				}
				else if (!pair.m_Node1->m_IsInternal && pair.m_Node2->m_IsInternal)
				{
					stk.push(OverlapPair(pair.m_Node1, pair.m_Node2->m_Children[0]));
					stk.push(OverlapPair(pair.m_Node1, pair.m_Node2->m_Children[1]));
				}
				else if (!pair.m_Node1->m_IsInternal && !pair.m_Node2->m_IsInternal)
				{
					for (int i = 0; i < pair.m_Node1->m_Prims.size(); ++i)
					{
						p1 = &pair.m_Node1->m_Prims[i];
						for (int j = 0; j < pair.m_Node2->m_Prims.size(); ++j)
						{
							p2 = &pair.m_Node2->m_Prims[j];
							if (p1->IsOverlapped(p2))
							{
								cb->HandleNodeOverlap(p1, p2);
							}
						}
					}

				}
			}
		}
	}

	std::vector<LBVNode> m_InternalNodes;
	std::vector<LBVNode> m_LeafNodes;
	std::vector<LBVPrim> m_StorePrims;
	std::vector<TATVector3> m_Positions;

	LBVNode* m_Root;
};

#pragma once
#include "../TATCommon/TATVector3.h"
#include "../TATBasis/TATHasher.h"
#include <vector>
#include <set>

class TATBvhCollideCallBack;

struct TATBVNode
{
public:
	TATBVNode(const TATVector3& aabbMin, const TATVector3& aabbMax) :m_AabbMin(aabbMin), m_AabbMax(aabbMax)
	{
		m_IsLeaf = false;
		m_Data = 0;
	}

	TATVector3 m_AabbMin;
	TATVector3 m_AabbMax;
	void* m_Data;
	TAT_REGISTER_ATTRIBUTE(bool, IsLeaf);

	bool IsOverlapped(TATBVNode* other)
	{
		bool overlapped = true;

		if (other->m_AabbMax.X < this->m_AabbMin.X || other->m_AabbMin.X > this->m_AabbMax.X)
			overlapped = false;
		if (other->m_AabbMax.Y < this->m_AabbMin.Y || other->m_AabbMin.Y > this->m_AabbMax.Y)
			overlapped = false;
		if (other->m_AabbMax.Z < this->m_AabbMin.Z || other->m_AabbMin.Z > this->m_AabbMax.Z)
			overlapped = false;

		return overlapped;
	}

	TATBVNode* Merge(TATBVNode* other)
	{
		return new TATBVNode(TATVector3::MakeMin(m_AabbMin, other->m_AabbMin),
							 TATVector3::MakeMax(m_AabbMax, other->m_AabbMax));
	}

	inline void SetAabb(const TATVector3& aabbMin,const TATVector3& aabbMax)
	{
		m_AabbMin = aabbMin; m_AabbMax = aabbMax;
	}
};

struct TATBVCollisonPair
{
public:
	TATBVCollisonPair(TATBVNode* node1, TATBVNode* node2) :m_node1(node1), m_node2(node2)
	{}
	TATBVNode* m_node1;
	TATBVNode* m_node2;
};

class TATBvh
{
protected:

	int m_NodeCount;

	std::vector<TATBVNode*> m_BVTree;

	bool m_CanCollideSelf;

public:

	TATBvh()
	{
		TATBVNode* head = new TATBVNode(TAT_MAXVECTOR3, -TAT_MAXVECTOR3);
		head->SetIsLeaf(false);
		m_BVTree.push_back(head);

		TATBVNode* root = new TATBVNode(TAT_MAXVECTOR3, -TAT_MAXVECTOR3);
		root->SetIsLeaf(false);
		m_BVTree.push_back(root);

		m_NodeCount = 0;
		m_CollideWithTree = false;
	}
	~TATBvh()
	{
		for (int i = 0; i < (int)m_BVTree.size(); i++)
		{
			delete m_BVTree[i];
		}
	}

	void SetCollideSelf(bool b)
	{
		m_CanCollideSelf = b;
	}

	void Clear()
	{
		for (int i = 0; i < (int)m_BVTree.size(); ++i)
		{
			delete m_BVTree[i];
			m_BVTree[i] = 0;
		}
		m_BVTree.clear();
		TATBVNode* head = new TATBVNode(TAT_MAXVECTOR3, -TAT_MAXVECTOR3);
		head->SetIsLeaf(false);
		m_BVTree.push_back(head);

		TATBVNode* root = new TATBVNode(TAT_MAXVECTOR3, -TAT_MAXVECTOR3);
		root->SetIsLeaf(false);
		m_BVTree.push_back(root);

		m_NodeCount = 0;
		m_OverlapSet.clear();
	}

	void FinishBuild();

	void PrintTree()
	{
		for (int i = 0; i < (int)m_BVTree.size(); i++)
		{
			std::cout << m_BVTree[i]->m_Data << " " << m_BVTree[i]->GetIsLeaf() << std::endl;
		}
	}

	TATBVNode* InsertAabbNode(const TATVector3& aabbMin, const TATVector3& aabbMax);

	bool CollideWithAabbNode(TATBVNode* node, TATBvhCollideCallBack* cb);

	bool CollideWithBVTree(TATBvh* other, TATBvhCollideCallBack* cb);

	void GetBound(TATVector3& min, TATVector3& max)
	{
		min = m_BVTree[0]->m_AabbMin;
		max = m_BVTree[0]->m_AabbMax;
	}

protected:
	bool InternalCollideWithAabbNode(TATBVNode* node, TATBvhCollideCallBack* cb);

	bool ProcessTraverse(int id, TATBVNode* node, TATBvhCollideCallBack* cb);

	bool ProcessTraverseTree(int id, TATBvh* tree, TATBvhCollideCallBack* cb);

	std::set<int> m_OverlapSet;

	bool m_CollideWithTree;
};
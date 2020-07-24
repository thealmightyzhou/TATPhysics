#include "TATBvh.h"
#include "TATBvhCollideCallBack.h"

TATBVNode* TATBvh::InsertAabbNode(const TATVector3& aabbMin, const TATVector3& aabbMax)
{
	TATBVNode* node = new TATBVNode(aabbMin, aabbMax);
	node->SetIsLeaf(true);

	m_BVTree.push_back(node);
	m_NodeCount++;
	int nodeId = m_BVTree.size() - 1;

	if (!m_BVTree[nodeId / 2]->GetIsLeaf())
	{
		return node;
	}
	else
	{
		TATBVNode* leafNode = m_BVTree[nodeId / 2];
		TATBVNode* tempNode = node->Merge(leafNode);
		tempNode->SetIsLeaf(false);
		m_BVTree[nodeId / 2] = tempNode;
		m_BVTree.push_back(leafNode);
		return node;
	}
}

void TATBvh::FinishBuild()
{
	for (int i = m_BVTree.size() / 2; i > 0; i--)
	{
		int lsonId = 2 * i;
		int rsonId = 2 * i + 1;

		if (lsonId < m_BVTree.size() && rsonId < m_BVTree.size())
		{
			m_BVTree[i]->SetAabb(TATVector3::MakeMin(m_BVTree[lsonId]->m_AabbMin, m_BVTree[rsonId]->m_AabbMin),
				TATVector3::MakeMax(m_BVTree[lsonId]->m_AabbMax, m_BVTree[rsonId]->m_AabbMax));
		}
		else
		{
			if (lsonId < m_BVTree.size())
				m_BVTree[i]->SetAabb(m_BVTree[lsonId]->m_AabbMin, m_BVTree[lsonId]->m_AabbMax);
		}
	}

	if (m_NodeCount > 0)
	{
		m_BVTree[0]->SetAabb(m_BVTree[1]->m_AabbMin, m_BVTree[1]->m_AabbMax);
	}
}

bool TATBvh::CollideWithAabbNode(TATBVNode* node, TATBvhCollideCallBack* cb)
{
	if (!node->IsOverlapped(m_BVTree[0]))
		return false;
	return ProcessTraverse(1, node, cb);
}

bool TATBvh::CollideWithBVTree(TATBvh* other, TATBvhCollideCallBack* cb)
{
	if (other == this && !m_CanCollideSelf)
		return false;

	if (!m_BVTree[0]->IsOverlapped(other->m_BVTree[0]))
		return false;

	return ProcessTraverseTree(1, other, cb);
}

bool TATBvh::ProcessTraverse(int id, TATBVNode* node, TATBvhCollideCallBack* cb)
{
	if (id >= m_BVTree.size())
		return false;

	if (m_BVTree[id]->IsOverlapped(node))
	{
		if (m_BVTree[id]->GetIsLeaf() && node->GetIsLeaf())
		{
			if ((m_BVTree[id] == node && m_CanCollideSelf) || m_BVTree[id] != node)
			{
				cb->NodeOverlapped(m_BVTree[id], node);
				return true;
			}
			else
				return false;
		}
		else if (m_BVTree[id]->GetIsLeaf())
		{
			return true;
		}
		else
		{
			bool success = false;
			if (ProcessTraverse(id * 2, node, cb))
				success = true;
			if (ProcessTraverse(id * 2 + 1, node, cb))
				success = true;;
			return success;
		}
	}
}

bool TATBvh::ProcessTraverseTree(int id, TATBvh* tree, TATBvhCollideCallBack* cb)
{
	if (id >= m_BVTree.size())
		return false;

	bool success = false;
	if (tree->CollideWithAabbNode(m_BVTree[id], cb))
	{
		success = true;
		ProcessTraverseTree(id * 2, tree, cb);
		ProcessTraverseTree(id * 2 + 1, tree, cb);
	}
	return success;
}
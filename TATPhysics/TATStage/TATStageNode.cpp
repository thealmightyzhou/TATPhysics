#include "TATStageNode.h"
#include "../TATBasis/TATWorld.h"
#include "TATActor.h"

//fail if has mounted actor before
//if has a actor will add to stage 
bool TATStageNode::MountActor(TATActor* actor)
{
	if (m_Actor)
		return false;

	m_Actor = actor;

	TATWorld::Instance()->AddToStage(m_Name, this);
}

bool TATStageNode::DumpActor()
{
	m_Actor = 0;
	TATWorld::Instance()->RemoveFromStage(this);
	return true;
}

bool TATStageNode::CreateChild(const TString& name)
{
	TString node_name = "node_" + name;
	if (TATWorld::Instance()->GetNodeByName(node_name))
	{
		return false;
	}

	TATStageNode* node = new TATStageNode(node_name);
	node->m_ParentNode = this;
	this->m_ChildNodes.push_back(node);
}

bool TATStageNode::DestroyChildByName(const TString& name)
{
	std::vector<TATStageNode*>::iterator it = m_ChildNodes.begin();
	bool deleted = false;
	while (it != m_ChildNodes.end())
	{
		if ((*it)->GetName() == name)
		{
			TATWorld::Instance()->RemoveFromStage(*it);

			delete *it;
			*it = 0;
			it = m_ChildNodes.erase(it);

			deleted = true;
		}

		it++;
	}

	return deleted;
}

void TATStageNode::DestroySelf()
{
	if (m_ParentNode)
	{
		m_ParentNode->DestroyChildByName(m_Name);
	}
	else
		this->~TATStageNode();

}

bool TATStageNode::DestroyChildByIndex(int index)
{
	if (index < 0 || m_ChildNodes.size() - 1 < index)
	{
		return false;
	}

	TATStageNode* node = m_ChildNodes[index];

	TATWorld::Instance()->RemoveFromStage(node);

	delete node;
	node = 0;
	m_ChildNodes.erase(m_ChildNodes.begin() + index);
}

TATStageNode::~TATStageNode()
{
	for (int i = 0; i < m_ChildNodes.size(); i++)
	{
		delete m_ChildNodes[i];
		m_ChildNodes[i] = 0;
	}

	m_ChildNodes.clear();
	m_ParentNode = 0;
	m_Actor = 0;
}
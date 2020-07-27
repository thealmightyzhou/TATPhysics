#pragma once

#include "../TATCommon/TATransform.h"
#include "../TATBasis/TString.h"

class TATActor;

//TODO aim to build stage bvh to do tailoring test
class TATStageNode
{
public:
	//replaced by CreateChild(name); 
	TATStageNode(const TString& name) :m_Name(name)
	{
		m_ParentNode = 0;
		m_Actor = 0;
		m_Visible = true;
	}

	virtual ~TATStageNode();

	TATStageNode* m_ParentNode;

	std::vector<TATStageNode*> m_ChildNodes;

	TATActor* m_Actor;

	TAT_REGISTER_ATTRIBUTE(bool, Visible);

	TAT_REGISTER_ATTRIBUTE_GET(TString, Name);

	TATransform m_RelativeTransform;

	TATransform GetWorldTransform()
	{
		TATransform tr = m_RelativeTransform;
		TATStageNode* node = m_ParentNode;
		while (node)
		{
			tr = node->m_RelativeTransform * tr;
			node = node->m_ParentNode;
		}
		return tr;
	}

	bool CreateChild(const TString& name);

	bool DestroyChildByName(const TString& name);

	bool DestroyChildByIndex(int index);

	void DestroySelf();

	void SetPosition(const TATVector3& pos)
	{
		m_RelativeTransform.SetOrigin(pos);
	}

	void SetRotation(const TATQuaternion& rot)
	{
		m_RelativeTransform.SetRotation(rot);
	}

	void SetTransform(const TATransform& tr)
	{
		m_RelativeTransform = tr;
	}

	bool MountActor(TATActor* actor);

	bool DumpActor();

};
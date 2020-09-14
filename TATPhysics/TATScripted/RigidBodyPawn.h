#pragma once
#include "../TATStage/TATPawn.h"
#include "../TATDynamics/TATRigidBody.h"
#include "../TATDynamics/TATDynamicWorld.h"

class RigidBodyPawn :public TATPawn
{
public:
	RigidBodyPawn(TATMesh* mesh, TATRigidBody* rb) :TATPawn(mesh), m_Rigid(rb)
	{
		m_Rigid->m_InvMass = 0.0f;
	}

	virtual void OnKeyPressed(int key) {}
	virtual void OnKeyReleased(int key)
	{
		m_Rigid->m_LinVel.SetZero();
	
	}
	virtual void OnKeyHold(int key) 
	{
		float dis = 10;
		TATVector3 pre_pos = m_Rigid->m_Pos;
		if (key == GLFW_KEY_UP)
		{
			m_Rigid->m_Pos -= dis * TATVector3::UnitZ();
		}
		else if (key == GLFW_KEY_DOWN)
		{
			m_Rigid->m_Pos += dis * TATVector3::UnitZ();
		}
		else if (key == GLFW_KEY_LEFT)
		{
			m_Rigid->m_Pos -= dis * TATVector3::UnitX();
		}
		else if (key == GLFW_KEY_RIGHT)
		{
			m_Rigid->m_Pos += dis * TATVector3::UnitX();
		}
		else if (key == GLFW_KEY_PAGE_UP)
		{
			m_Rigid->m_Pos += dis * TATVector3::UnitY();
		}
		else if (key == GLFW_KEY_PAGE_DOWN)
		{
			m_Rigid->m_Pos -= dis * TATVector3::UnitY();
		}

		m_Rigid->m_WorldTransform.SetOrigin(m_Rigid->m_Pos);
		m_Rigid->m_LinVel = (m_Rigid->m_Pos - pre_pos) * 100;
	}
	virtual void OnCursorMove(float dx, float dy) {}
	virtual void OnCursorPressed(int key) {}
	virtual void OnCursorReleased(int key) {}
	virtual void OnCursorHold(int key) {}
	virtual void OnCursorScroll(float offset) {}

	TATRigidBody* m_Rigid;
};
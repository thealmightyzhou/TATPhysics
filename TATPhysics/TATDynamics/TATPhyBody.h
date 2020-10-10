#pragma once

class TATPhyBody
{
public:
	enum BodyType
	{
		RigidBody,
		SoftBody

	};

	TATPhyBody(BodyType type) :m_Type(type)
	{
		m_CollisionMask = 0;
		m_InitValue = (unsigned long)this;
	}

	bool Legal()
	{
		return m_InitValue == (unsigned long)this;
	}

	BodyType m_Type;
	unsigned long m_CollisionMask;
	unsigned long m_InitValue;
};
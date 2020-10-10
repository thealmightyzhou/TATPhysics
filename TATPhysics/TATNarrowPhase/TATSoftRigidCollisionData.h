#pragma once

//#include "../TATDynamics/TATRigidBody.h"
//#include "../TATPositionBasedDynamics/TATPBDBody.h"
#include "../TATCommon/TATVector3.h"

class TATRigidBody;
class TATPBDBody;
class TATPBDParticle;

struct TATSoftRigidCollideData
{
	TATRigidBody* m_Rigid;
	TATPBDBody* m_Soft;
	TATPBDParticle* m_Particle;
	int m_SoftFaceIndex;
	float m_Penetration;
	float m_HitFraction;
	float m_SoftFaceWeight[3];
	TATVector3 m_CollideNormal; // point form rigid to soft
	TATVector3 m_SoftPt;
	TATVector3 m_RigidPt;
};

class TATSoftRigidCollisionAlgoPrimitive
{
public:
	virtual ~TATSoftRigidCollisionAlgoPrimitive() {}

	virtual bool ProcessSoftRigidCollision(TATPBDBody* soft, TATRigidBody* rb, float dt)
	{
		return false;
	}

	virtual void SolveRSContacts(TATSoftRigidCollideData& c,float dt) {}

	std::vector<TATSoftRigidCollideData> m_CollisionDatas;
};
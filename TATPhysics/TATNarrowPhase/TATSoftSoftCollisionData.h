#pragma once

#include "../TATCommon/TATVector3.h"

class TATPBDBody;
class TATPBDParticle;

struct TATSoftSoftCollideData
{
	enum SoftSoftCollisionType
	{
		PtCollideFace,
		FaceCollideFace,
		SegCollideFace,
	};

	SoftSoftCollisionType m_CollisionType;
	TATPBDBody* m_Soft0;
	TATPBDBody* m_Soft1;

	TATPBDParticle* m_Particle0;

	//pt face
	int m_FaceIndices[3];
	float m_FaceWeights[3];

	//seg face
	int m_SegIndices[2];
	float m_SegWeights[2];

	float m_Penetration;
	float m_HitFraction;
	TATVector3 m_CollideNormal; // point form soft1 to soft0

	TATVector3 m_SoftPt0;
	TATVector3 m_SoftPt1;
};

class TATSoftSoftCollisionAlgoPrimitive
{
public:
	virtual ~TATSoftSoftCollisionAlgoPrimitive() {}

	virtual bool ProcessSoftSoftCollision(TATPBDBody* soft0, TATPBDBody* soft1)
	{
		return false;
	}

	std::vector<TATSoftSoftCollideData> m_CollisionDatas;
};
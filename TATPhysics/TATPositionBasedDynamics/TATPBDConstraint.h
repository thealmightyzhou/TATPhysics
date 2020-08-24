#pragma once
#include "../TATBasis/TATHasher.h"
#include "../TATGeometry/TATMeshInfo.h"

class TATRigidBody;
class TATPBDParticle;

class TATPBDConstraint
{
public:
	TATPBDConstraint() {}
	virtual ~TATPBDConstraint() {}

	virtual bool SolveConstraint() 
	{
		return false;
	}

	int GetHashValue() const
	{
		return m_HashValue;
	}

	float SetRestValue(float val)
	{
		m_RestValue = val;
	}
protected:
	int m_HashValue;

	float m_RestValue;
};

class TATPBDVertexDistConstraint :public TATPBDConstraint
{
public:
	TATPBDVertexDistConstraint(TATPBDParticle* v0, TATPBDParticle* v1, float compressStiff, float strechStiff);

	bool SolveConstraint();

	TATPBDParticle* m_Vertex0;
	TATPBDParticle* m_Vertex1;

	float m_CompressionStiffness;
	float m_StrechStiffness;
};

class TATPBDVertexSpacePtDistConstraint :public TATPBDConstraint
{
public:
	TATPBDVertexSpacePtDistConstraint(TATPBDParticle* v, const TATVector3& pt, float comp, float strech);

	bool SolveConstraint();

	TATPBDParticle* m_Vertex;
	TATVector3 m_Position;

	float m_CompressionStiffness;
	float m_StrechStiffness;
};

class TATPBDVertexRigidPtDistConstraint :public TATPBDConstraint
{
public:
	TATPBDVertexRigidPtDistConstraint(TATPBDParticle* v, const TATVector3& ptInLocal, TATRigidBody* rigid, float comp, float strech);

	bool SolveConstraint();

	TATPBDParticle* m_Vertex;
	TATVector3 m_Position;
	TATRigidBody* m_Rigid;

	float m_CompressionStiffness;
	float m_StrechStiffness;
};

class TATPBDVolumeConstraint :public TATPBDConstraint
{
public:
	TATPBDVolumeConstraint(TATPBDParticle* p0, TATPBDParticle* p1, TATPBDParticle* p2, TATPBDParticle* p3, float negStiff, float posStiff);

	bool SolveConstraint();

	TATPBDParticle* m_Vertex0;
	TATPBDParticle* m_Vertex1;
	TATPBDParticle* m_Vertex2;
	TATPBDParticle* m_Vertex3;

	float m_NegtiveStiffness;
	float m_PositiveStiffness;
};

class TATPBDDihedralConstraint : public TATPBDConstraint
{
public:
	TATPBDDihedralConstraint(TATPBDParticle* p0, TATPBDParticle* p1, TATPBDParticle* p2, TATPBDParticle* p3, float stiff);

	bool SolveConstraint();

	TATPBDParticle* m_Particle0;
	TATPBDParticle* m_Particle1;
	TATPBDParticle* m_Particle2;
	TATPBDParticle* m_Particle3;

	float m_Stiffness;
};
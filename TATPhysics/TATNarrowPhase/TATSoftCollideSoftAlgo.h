#pragma once

#include "TATSoftSoftCollisionData.h"
#include "../TATBroadPhase/TATBvh.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"
#include "../TATNarrowPhase/TATCollisionUtil.h"
#include "../TATPositionBasedDynamics/TATPBDBody.h"

class TATSoftFaceSoftParticleCollisionCB :public TATBvhCollideCallBack
{
public:
	TATSoftFaceSoftParticleCollisionCB(TATPBDBody* soft0, TATPBDBody* soft1, std::vector<TATSoftSoftCollideData>& datas) :
		m_Soft0(soft0), m_Soft1(soft1), m_CollideDatas(datas)
	{}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
	{
		int face_id = (int)node1->m_Data;
		TATPBDParticle* particle = (TATPBDParticle*)node2->m_Data;
		std::vector<TATPhyFace>& faces = m_Soft1->m_PhyBody.m_Faces;
		TATVector3 face[3]
		{
			m_Soft1->GetParticleAt(faces[face_id].m_VertexIndices[0])->m_LastPos,
			m_Soft1->GetParticleAt(faces[face_id].m_VertexIndices[1])->m_LastPos,
			m_Soft1->GetParticleAt(faces[face_id].m_VertexIndices[2])->m_LastPos
		};
		TATVector3 vel[3]
		{
			m_Soft1->GetParticleAt(faces[face_id].m_VertexIndices[0])->m_CurrPos - face[0],
			m_Soft1->GetParticleAt(faces[face_id].m_VertexIndices[1])->m_CurrPos - face[1],
			m_Soft1->GetParticleAt(faces[face_id].m_VertexIndices[2])->m_CurrPos - face[2],
		};

		float t;
		float margin = 0.2;
		float collide_radius = 0.02;
		if (TATCollisionUtil::CalcTimeOfImpact(particle->m_LastPos, particle->m_CurrPos - particle->m_LastPos, face, vel, t, 64, margin))
		{
			TATVector3 pt = particle->m_LastPos + t * (particle->m_CurrPos - particle->m_LastPos);
			face[0] = face[0] + t * vel[0];
			face[1] = face[1] + t * vel[1];
			face[2] = face[2] + t * vel[2];

			TATSoftSoftCollideData data;
			data.m_Soft0 = m_Soft0;
			data.m_Soft1 = m_Soft1;
			data.m_CollisionType = TATSoftSoftCollideData::SoftSoftCollisionType::PtCollideFace;
			data.m_Particle0 = particle;
			
			data.m_FaceIndices[0] = faces[face_id].m_VertexIndices[0];
			data.m_FaceIndices[1] = faces[face_id].m_VertexIndices[1];
			data.m_FaceIndices[2] = faces[face_id].m_VertexIndices[2];

			float weight[3]{0.3333,0.3333,0.3334}; //temp
			TATGeometryUtil::ClosetPtOnTri(pt, face[0], face[1], face[2], weight);

			data.m_SoftPt0 = pt;
			data.m_SoftPt1 = face[0] * weight[0] + face[1] * weight[1] + face[2] * weight[2];

			data.m_CollideNormal = (face[1] - face[0]).Cross(face[2] - face[0]).Normalized();
			data.m_Penetration = margin + collide_radius - (pt - face[0]).Dot(data.m_CollideNormal);

			m_CollideDatas.push_back(data);
		}
	}

	TATPBDBody* m_Soft0, * m_Soft1;
	std::vector<TATSoftSoftCollideData>& m_CollideDatas;
};

class TATSoftCollideSoftAlgo : public TATSoftSoftCollisionAlgoPrimitive
{
public:
	virtual bool ProcessSoftSoftCollision(TATPBDBody* soft0, TATPBDBody* soft1)
	{
		TATBvh softFaceTree;

		TATVector3 min, max;

		std::vector<TATPhyFace>& faces = soft1->m_PhyBody.m_Faces;

		TATVector3 pos0, pos1, pos2;
		TATVector3 vel0, vel1, vel2;

		for (int i = 0; i < faces.size(); ++i)
		{
			pos0 = soft1->GetParticleAt(faces[i].m_VertexIndices[0])->m_LastPos;
			pos1 = soft1->GetParticleAt(faces[i].m_VertexIndices[1])->m_LastPos;
			pos2 = soft1->GetParticleAt(faces[i].m_VertexIndices[2])->m_LastPos;

			vel0 = soft1->GetParticleAt(faces[i].m_VertexIndices[0])->m_CurrPos - pos0;
			vel1 = soft1->GetParticleAt(faces[i].m_VertexIndices[1])->m_CurrPos - pos1;
			vel2 = soft1->GetParticleAt(faces[i].m_VertexIndices[2])->m_CurrPos - pos2;

			min = pos0;
			min.SetMin(pos1);
			min.SetMin(pos2);
			min.SetMin(pos0 + vel0); min.SetMin(pos1 + vel1); min.SetMin(pos2 + vel2);

			max = pos0;
			max.SetMax(pos1);
			max.SetMax(pos2);
			max.SetMax(pos0 + vel0); max.SetMax(pos1 + vel1); max.SetMax(pos2 + vel2);

			softFaceTree.InsertAabbNode(min, max)->m_Data = (void*)(i);
		}

		softFaceTree.FinishBuild();

		TATSoftFaceSoftParticleCollisionCB cb(soft0, soft1, m_CollisionDatas);
		soft0->m_ParticleBVH.CollideWithBVTree(&softFaceTree, &cb);

		if (m_CollisionDatas.size() > 0)
			return true;
		else
			return false;
	}
};


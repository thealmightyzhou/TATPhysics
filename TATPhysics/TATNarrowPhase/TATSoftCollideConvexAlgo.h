#pragma once
#include "TATSoftRigidCollisionData.h"
#include "../TATBroadPhase/TATBvh.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"
#include "../TATNarrowPhase/TATCollisionUtil.h"
#include "../TATDynamics/TATRigidBody.h"
#include "../TATPositionBasedDynamics/TATPBDBody.h"
#include "../TATApplication/TATRenderTask.h"
#include "../TATApplication/TATApplication.h"
#include "../TATApplication/TAThread.h"

#define FACE_EXTENT 0.1

struct ConvexFacePack
{
	int m_FaceIndex;
	TATVector3 m_Pos[3];
	TATVector3 m_CurrPos[3];
	TATVector3 m_Vel[3];
};

class SoftConvexFaceOverlapCallBack :public TATBvhCollideCallBack
{
public:
	SoftConvexFaceOverlapCallBack(TATRigidBody* rb, std::vector<TATSoftRigidCollideData>& collideDatas,float dt) :
		m_Rigid(rb),m_CollideDatas(collideDatas), m_DeltaTime(dt)
	{

	}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
	{
		TATPBDParticle* particle = (TATPBDParticle*)(node1->m_Data);
		ConvexFacePack* facePack = (ConvexFacePack*)(node2->m_Data);

		TATVector3 vel = particle->m_CurrPos - particle->m_LastPos;

		float t;
		float margin = 0.2;
		float collideRadius = 0.02f;

		if (TATCollisionUtil::CalcTimeOfImpact
		(
			particle->m_LastPos,
			vel,
			m_Rigid,
			m_Rigid->GetLinearVelocity(),
			m_Rigid->GetAngularVelocity(),
			facePack->m_FaceIndex,
			m_DeltaTime,
			margin,
			collideRadius,
			64,
			t
		))
		{
			float time = t * m_DeltaTime;
			TATransform predictTr;
			TATransformUtil::IntegrateTransform
			(
				m_Rigid->GetWorldTransform(),
				m_Rigid->GetLinearVelocity(),
				m_Rigid->GetAngularVelocity(),
				time,
				predictTr
			);
			TATCollideShapeConvex* convex = m_Rigid->m_CollideShape->Cast<TATCollideShapeConvex>();
			const TATPhyFace& face = convex->m_CollideMeshData.m_Faces[facePack->m_FaceIndex];
			TATVector3 points[3]
			{
				predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[0]].m_Position,
				predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[1]].m_Position,
				predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[2]].m_Position
			};
			TATVector3 norm = (points[1] - points[0]).Cross(points[2] - points[0]).Normalized();
			TATVector3 p = vel * t + particle->m_LastPos + norm * collideRadius;
			float dist = (p - points[0]).Dot(norm) - margin;

			float pen = collideRadius - dist;

			TATSoftRigidCollideData data;
			data.m_Soft = particle->m_HostBody;
			data.m_Rigid = m_Rigid;
			data.m_Particle = particle;
			data.m_CollideNormal = norm;
			data.m_Penetration = pen;
			data.m_RigidPt = p - norm * (dist + margin);
			data.m_SoftPt = p;
			data.m_HitFraction = t;

			m_CollideDatas.push_back(data);
		}
	}

	TATRigidBody* m_Rigid;
	std::vector<TATSoftRigidCollideData>& m_CollideDatas;
	float m_DeltaTime;
};

class TATSoftCollideConvexAlgo : public TATSoftRigidCollisionAlgoPrimitive
{
public:
	virtual bool ProcessSoftRigidCollision(TATPBDBody* soft, TATRigidBody* rb, float dt)
	{
		TATCollideShapeConvex* convex = dynamic_cast<TATCollideShapeConvex*>(rb->m_CollideShape);

		TATBvh faceTree;
		TATVector3 min, max;

		const TATransform& tr = rb->GetWorldTransform();

		std::vector<TATPhyFace>& faces = convex->m_CollideMeshData.m_Faces;
		std::vector<TATPhyVertex>& vertices = convex->m_CollideMeshData.m_Vertices;

		TATVector3 pos0, pos1, pos2;
		TATVector3 vel0, vel1, vel2;
		TATVector3 predict0, predict1, predict2;

		std::vector<ConvexFacePack> facePack;
		facePack.resize(faces.size());
		for (int i = 0; i < faces.size(); ++i)
		{
			const TATPhyVertex& vertex0 = vertices[faces[i].m_VertexIndices[0]];
			const TATPhyVertex& vertex1 = vertices[faces[i].m_VertexIndices[1]];
			const TATPhyVertex& vertex2 = vertices[faces[i].m_VertexIndices[2]];

			pos0 = tr * vertex0.m_Position;
			pos1 = tr * vertex1.m_Position;
			pos2 = tr * vertex2.m_Position;

			vel0 = rb->GetVelocityAtWCS(pos0);
			vel1 = rb->GetVelocityAtWCS(pos1);
			vel2 = rb->GetVelocityAtWCS(pos2);

			predict0 = pos0 + vel0 * dt;
			predict1 = pos1 + vel1 * dt;
			predict2 = pos2 + vel2 * dt;

			min = pos0;
			min.SetMin(pos1);
			min.SetMin(pos2);
			min.SetMin(predict0); min.SetMin(predict1); min.SetMin(predict2);

			max = pos0;
			max.SetMax(pos1);
			max.SetMax(pos2);
			max.SetMax(predict0); max.SetMax(predict1); max.SetMax(predict2);

			facePack[i].m_FaceIndex = i;
			facePack[i].m_Pos[0] = pos0; facePack[i].m_Pos[1] = pos1; facePack[i].m_Pos[2] = pos2;
			facePack[i].m_CurrPos[0] = predict0;
			facePack[i].m_CurrPos[1] = predict1;
			facePack[i].m_CurrPos[2] = predict2;
			facePack[i].m_Vel[0] = predict0 - facePack[i].m_Pos[0];
			facePack[i].m_Vel[1] = predict1 - facePack[i].m_Pos[1];
			facePack[i].m_Vel[2] = predict2 - facePack[i].m_Pos[2];

			faceTree.InsertAabbNode(min - TATVector3::One() * FACE_EXTENT, max + TATVector3::One() * FACE_EXTENT)->m_Data = (void*)(&facePack[i]);
		}

		faceTree.FinishBuild();

		SoftConvexFaceOverlapCallBack cb(rb, m_CollisionDatas, dt);
		faceTree.CollideWithBVTree(&soft->m_ParticleBVH, &cb);

		if (m_CollisionDatas.size() > 0)
			return true;
		else
			return false;
	}

	virtual void SolveRSContacts(TATSoftRigidCollideData& data, float dt)
	{
		TATPBDParticle* particle = data.m_Particle;
		TATRigidBody* rigid = data.m_Rigid;

		if (rigid->m_InvMass != 0)
		{
			TATransform predictTr;
			TATransformUtil::IntegrateTransform(rigid->GetWorldTransform(), rigid->GetLinearVelocity(), rigid->GetAngularVelocity(), data.m_HitFraction * dt, predictTr);
			rigid->SetWorldTransform(predictTr);
			rigid->UpdataInverseInertiaWorld();
		}

		const TATMatrix3& iwi = rigid->m_InvInertiaWorld;
		//TATVector3 r = data.m_RigidPt - rigid->GetMassCenter();
		TATVector3 r = data.m_SoftPt - rigid->GetMassCenter();

		TATMatrix3 impulseMatrix = TATCollisionUtil::ImpulseMatrix
		(
			dt,
			data.m_Particle->m_InvMass,
			rigid->m_InvMass,
			iwi,
			r
		);

		//TATVector3 va = rigid->GetVelocityAtWCS(data.m_RigidPt) * dt;
		TATVector3 va = rigid->GetVelocityAtWCS(data.m_SoftPt) * dt;
		TATVector3 vb = particle->m_CurrPos - particle->m_LastPos;
		const TATVector3 rel_vel = vb - va;
		const float rel_vel_normal = rel_vel.Dot(data.m_CollideNormal);
		if (rel_vel_normal > 0)
			return;

		const TATVector3 rel_frict_vel = rel_vel - rel_vel_normal * data.m_CollideNormal;
		float frict = particle->m_HostBody->m_FrictionCoeffcient * rigid->m_FrictionCoeff;
		float fricCoeff = rel_frict_vel.Length() < rel_vel_normal * particle->m_HostBody->m_FrictionCoeffcient ? 0 : 1 - frict;

		float kst = 1.0f;

		if (data.m_Penetration > 0)
			return;

		const TATVector3 impulse = impulseMatrix *
			(rel_vel - rel_frict_vel * fricCoeff - data.m_Penetration * rigid->m_ContactHardness * data.m_CollideNormal) * kst;

		particle->m_CurrPos = data.m_SoftPt;
		particle->m_CurrPos -= impulse * particle->m_InvMass * dt;
		//particle->m_Velocity -= impulse * particle->m_InvMass;

		rigid->ApplyImpulse(impulse, r);

	}
};

//================================
class SoftFaceConvexFaceOverlapCallBack :public TATBvhCollideCallBack
{
public:
	SoftFaceConvexFaceOverlapCallBack(TATRigidBody* rb, TATPBDBody* soft, std::vector<TATSoftRigidCollideData>& collideDatas, float dt, const TATransform& tr) :
		m_Rigid(rb), m_Soft(soft), m_CollideDatas(collideDatas), m_DeltaTime(dt), m_Transform(tr)
	{

	}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
	{
		int soft_face_index = (int)(node1->m_Data);
		TATCollideShapeConvex* convex = (TATCollideShapeConvex*)(node2->m_Data);
		
		const TATPhyFace& softface = m_Soft->m_PhyBody.m_Faces[soft_face_index];

		TATPBDParticle* particles[3]
		{
			m_Soft->GetParticleAt(softface.m_VertexIndices[0]),
			m_Soft->GetParticleAt(softface.m_VertexIndices[1]),
			m_Soft->GetParticleAt(softface.m_VertexIndices[2])
		};

		TATVector3 soft_pt[3]
		{
			particles[0]->m_CurrPos,
			particles[1]->m_CurrPos,
			particles[2]->m_CurrPos
		};

		float margin = 0.2;
		float collideRadius = 0.02f;

		TATSATDistPack dp;
		float dist = TATSATDistSolver::SolveTriConvexDistance(soft_pt, convex, m_Transform, dp);
		if (dist > margin + collideRadius)
			return;

		float w[3];
		TATGeometryUtil::ClosetPtOnTri(dp.m_ClostPtA, soft_pt[0], soft_pt[1], soft_pt[2], w);
		
		{
			TATSoftRigidCollideData c;
			c.m_Soft = m_Soft;
			c.m_Rigid = m_Rigid;
			c.m_CollideNormal = dp.m_Normal;
			memcpy(c.m_SoftFaceWeight, w, 3 * sizeof(float));
			c.m_SoftFaceIndex = soft_face_index;
			c.m_RigidPt = dp.m_ClostPtB;
			c.m_SoftPt = dp.m_ClostPtA;
			c.m_Penetration = margin - dist;

			m_CollideDatas.push_back(c);
		}
	}

	TATRigidBody* m_Rigid;
	TATPBDBody* m_Soft;
	std::vector<TATSoftRigidCollideData>& m_CollideDatas;
	float m_DeltaTime;
	TATransform m_Transform;
};

//softbody face based
class TATSoftCollideConvexAlgoV2 : public TATSoftRigidCollisionAlgoPrimitive
{
public:
	virtual bool ProcessSoftRigidCollision(TATPBDBody* soft, TATRigidBody* rb, float dt)
	{
		TATCollideShapeConvex* convex = dynamic_cast<TATCollideShapeConvex*>(rb->m_CollideShape);

		TATBvh faceTree;
		TATBvh softFaceTree;

		TATVector3 min, max;
		rb->GetWorldAabb(min, max);
		faceTree.InsertAabbNode(min, max)->m_Data = (void*)convex;
		faceTree.FinishBuild();

		const TATransform& tr = rb->GetWorldTransform();

		for (int i = 0; i < soft->m_PhyBody.m_Faces.size(); ++i)
		{
			TATPhyFace& face = soft->m_PhyBody.m_Faces[i];
			TATPBDParticle* p[3]
			{
				soft->GetParticleAt(face.m_VertexIndices[0]),
				soft->GetParticleAt(face.m_VertexIndices[1]),
				soft->GetParticleAt(face.m_VertexIndices[2])
			};

			min = p[0]->m_CurrPos;
			max = p[0]->m_CurrPos;
			for (int j = 1; j < 3; ++j)
			{
				min.SetMin(p[j]->m_CurrPos);
				max.SetMax(p[j]->m_CurrPos);
			}

			softFaceTree.InsertAabbNode(min, max)->m_Data = (void*)i;
		}
		softFaceTree.FinishBuild();


		SoftFaceConvexFaceOverlapCallBack cb(rb, soft, m_CollisionDatas, dt, rb->GetWorldTransform());
		faceTree.CollideWithBVTree(&softFaceTree, &cb);

		if (m_CollisionDatas.size() > 0)
			return true;
		else
			return false;
	}

	virtual void SolveRSContacts(TATSoftRigidCollideData& data, float dt)
	{
		TATPhyFace& soft_face = data.m_Soft->m_PhyBody.m_Faces[data.m_SoftFaceIndex];
		TATPBDParticle* particles[3]
		{
			& data.m_Soft->m_Particles[soft_face.m_VertexIndices[0]],
			& data.m_Soft->m_Particles[soft_face.m_VertexIndices[1]],
			& data.m_Soft->m_Particles[soft_face.m_VertexIndices[2]]
		};

		TATVector3 soft_vx[3]
		{
			particles[0]->m_CurrPos,
			particles[1]->m_CurrPos,
			particles[2]->m_CurrPos
		};

		TATCollideShapeConvex* convex = data.m_Rigid->m_CollideShape->Cast<TATCollideShapeConvex>();

		float invm = 1 / (1 / particles[0]->m_InvMass * data.m_SoftFaceWeight[0]
						+ 1 / particles[1]->m_InvMass * data.m_SoftFaceWeight[1]
						+ 1 / particles[2]->m_InvMass * data.m_SoftFaceWeight[2]);

		TATVector3 r = data.m_RigidPt - data.m_Rigid->GetMassCenter();
		TATMatrix3 imp = TATCollisionUtil::ImpulseMatrix(invm, data.m_Rigid->m_InvMass, data.m_Rigid->m_InvInertiaWorld, r);

		TATVector3 vel = particles[0]->m_Velocity * data.m_SoftFaceWeight[0] +
						 particles[1]->m_Velocity * data.m_SoftFaceWeight[1] +
						 particles[2]->m_Velocity * data.m_SoftFaceWeight[2];

		TATVector3 rel_vel = data.m_Rigid->GetLinearVelocity() + data.m_Rigid->GetAngularVelocity().Cross(r);
		rel_vel -= vel;

		//float va = rel_vel.Dot(data.m_CollideNormal);
		float va = rel_vel.Dot(data.m_CollideNormal) + data.m_Penetration * 0.8;
		TATVector3 frict = rel_vel - rel_vel.Dot(data.m_CollideNormal);

		if (va < 0)
			return;

		float j = (va * data.m_CollideNormal * imp).Dot(data.m_CollideNormal);
		TATVector3 J = j * data.m_CollideNormal;

		particles[0]->m_CurrPos += J * particles[0]->m_InvMass * dt * data.m_SoftFaceWeight[0];
		particles[1]->m_CurrPos += J * particles[1]->m_InvMass * dt * data.m_SoftFaceWeight[1];
		particles[2]->m_CurrPos += J * particles[2]->m_InvMass * dt * data.m_SoftFaceWeight[2];

		data.m_Rigid->ApplyImpulse(-J, r);

		TATVector3 softPt = soft_vx[0] * data.m_SoftFaceWeight[0] + soft_vx[1] * data.m_SoftFaceWeight[1] + soft_vx[2] * data.m_SoftFaceWeight[2];
		
		TAT_RENDER_TASK_LIST->PushTask([](const TATVector3& p0, const TATVector3& p1, const TATVector3& col)->void
		{
			if (TAT_RENDER_THREAD->m_LinePainter)
			{
				TAT_RENDER_THREAD->m_LinePainter->Clear();
				TAT_RENDER_THREAD->m_LinePainter->PaintLine(p0, p1, col);
			}

		}, softPt, softPt + J, TATVector3(1, 0, 0));
	}
};


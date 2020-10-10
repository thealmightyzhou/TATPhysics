#pragma once
#include "../TATCommon/TATransform.h"
#include "../TATDynamics/TATRigidBody.h"
#include "../TATCommon/TATRange.h"
#include "../TATGeometry/TATGeometryComputer.h"

struct TATSATCollideData
{
public:
	TATSATCollideData()
	{
		m_Penetration = TAT_MAX;
		m_Dist = -TAT_MAX;
		m_ResFaceA = 0;
		m_ResFaceB = 0;
		m_ResVertexA = 0;
		m_ResVertexB = 0;
		m_ResEdges[0] = m_ResEdges[1] = 0;
		m_UserData = 0;
		m_RbA = 0;
		m_RbB = 0;
	}

	//always point from B to A
	TATVector3 m_CollideNormal;
	TATVector3 m_CollidePtA;
	TATVector3 m_CollidePtB;

	float m_Penetration;
	float m_Dist;

	TATransform m_TrA;
	TATransform m_TrB;

	TATPhyVertex* m_ResVertexA, * m_ResVertexB;
	TATPhyFace* m_ResFaceA, * m_ResFaceB;
	TATPhyEdge* m_ResEdges[2];

	void* m_UserData;

	int m_StateFlag; //0:ptA & faceB; 1:ptB & faceA; 2:edgeA & edgeB;

	int m_ResFlag; //0,1,2

	TATRigidBody* m_RbA;
	TATRigidBody* m_RbB;
};

//only for convex
class TATSAT
{
public:

	static bool SeparateAxisTest(TATRigidBody* rb0, TATRigidBody* rb1, TATSATCollideData& cd)
	{
		TATransform tr0 = rb0->GetWorldTransform();
		TATransform tr1 = rb1->GetWorldTransform();

		cd.m_TrA = tr0;
		cd.m_TrB = tr1;
		cd.m_RbA = rb0;
		cd.m_RbB = rb1;

		TATVector3 dir = tr0 * rb1->m_CollideShape->m_LocalMassCenter -
			tr1 * rb0->m_CollideShape->m_LocalMassCenter;

		TATCollideShapeConvex* convex0 = dynamic_cast<TATCollideShapeConvex*>(rb0->m_CollideShape);
		TATCollideShapeConvex* convex1 = dynamic_cast<TATCollideShapeConvex*>(rb1->m_CollideShape);

		std::vector<TATPhyFace>& faces0 = convex0->m_CollideMeshData.m_Faces;
		std::vector<TATPhyFace>& faces1 = convex1->m_CollideMeshData.m_Faces;
		std::vector<TATPhyEdge>& edges0 = convex0->m_CollideMeshData.m_Edges;
		std::vector<TATPhyEdge>& edges1 = convex1->m_CollideMeshData.m_Edges;

		TATVector3 testDir;
		int res;

		cd.m_StateFlag = 0;
		for (int i = 0; i < (int)faces1.size(); i++)
		{
			testDir = tr1.GetRotation() * faces1[i].GetNormal();

			if (testDir.Dot(dir) < 0)
			{
				testDir = -testDir;
			}

			res = TestOneDir(rb0, rb1, testDir, cd);
			if (res == -1)
				return false;
			if (res == 1)
			{
				cd.m_ResVertexA = (TATPhyVertex*)cd.m_UserData;
				cd.m_ResFaceB = &faces1[i];
				cd.m_ResFlag = cd.m_StateFlag;
				cd.m_CollideNormal = testDir;
			}
		}

		cd.m_StateFlag = 1;
		for (int i = 0; i < (int)faces0.size(); i++)
		{
			testDir = tr0.GetRotation() * faces0[i].GetNormal();

			//dir always from rb0 to rb1
			if (testDir.Dot(dir) < 0)
			{
				testDir = -testDir;
			}

			res = TestOneDir(rb1, rb0, testDir, cd);
			if (res == -1)
				return false;
			if (res == 1)
			{
				cd.m_ResVertexB = (TATPhyVertex*)cd.m_UserData;
				cd.m_ResFaceA = &faces0[i];
				cd.m_ResFlag = cd.m_StateFlag;
				cd.m_CollideNormal = testDir;
			}
		}

		cd.m_StateFlag = 2;
		for (int i = 0; i < (int)edges0.size(); i++)
		{
			const TATPhyEdge& e0 = edges0[i];
			for (int j = 0; j < (int)edges1.size(); j++)
			{
				const TATPhyEdge& e1 = edges1[j];
				if (CanBuildMinkowskiFace(e0, e1, tr0, tr1))
				{
					testDir = (tr0 * e0.GetDirection()).Cross(tr1 * e1.GetDirection()).Normalized();

					if (testDir.Dot(dir) < 0)
					{
						testDir = -testDir;
					}

					res = TestOneDir(rb0, rb1, testDir, cd);
					if (res == 1)
					{
						cd.m_ResEdges[0] = &edges0[i];
						cd.m_ResEdges[1] = &edges1[j];
						cd.m_ResFlag = cd.m_StateFlag;
						cd.m_CollideNormal = testDir;
					}
					else if (res == -1)
						return false;
				}
			}
		}

		FindClosetPt(cd);
		return true;
	}

	//Get min index and max index on v0set and v1set
	static void Support(const std::vector<TATPhyVertex>& vertices0, const std::vector<TATPhyVertex>& vertices1,
		const TATransform& tr0, const TATransform& tr1, const TATVector3& dir, int* indices, float* projects)
	{
		int i = 0, j = 0;
		float proj;
		indices[0] = indices[2] = -1;
		indices[1] = indices[3] = -1;
		projects[0] = projects[2] = TAT_MAX;
		projects[1] = projects[3] = -TAT_MAX;
		while (i < (int)vertices0.size() || j < (int)vertices1.size())
		{
			if (i < (int)vertices0.size())
			{
				proj = (tr0 * vertices0[i].m_Position).Dot(dir);
				if (proj < projects[0])
				{
					projects[0] = proj;
					indices[0] = i;
				}
				if (proj > projects[1])
				{
					projects[1] = proj;
					indices[1] = i;
				}
			}
			i++;
			if (j < (int)vertices1.size())
			{
				proj = (tr1 * vertices1[j].m_Position).Dot(dir);
				if (proj < projects[2])
				{
					projects[2] = proj;
					indices[2] = j;
				}
				if (proj > projects[3])
				{
					projects[3] = proj;
					indices[3] = j;
				}

			}
			j++;
		}

	}

	//test on wcs
	static int TestOneDir(TATRigidBody* rb0, TATRigidBody* rb1, const TATVector3& dir, TATSATCollideData& cd)
	{
		TATCollideShapeConvex* convex0 = dynamic_cast<TATCollideShapeConvex*>(rb0->m_CollideShape);
		TATCollideShapeConvex* convex1 = dynamic_cast<TATCollideShapeConvex*>(rb1->m_CollideShape);

		std::vector<TATPhyVertex>& vertices0 = convex0->m_CollideMeshData.m_Vertices;
		std::vector<TATPhyVertex>& vertices1 = convex1->m_CollideMeshData.m_Vertices;

		TATransform tr0 = rb0->GetWorldTransform();
		TATransform tr1 = rb1->GetWorldTransform();

		int indices[4];
		float projs[4];
		Support(vertices0, vertices1, tr0, tr1, dir, indices, projs);

		TATRange range1(projs[0], projs[1]);
		TATRange range2(projs[2], projs[3]);

		range1.SetUserData((void*)(indices[0]), (void*)(indices[1]));
		range2.SetUserData((void*)(indices[2]), (void*)(indices[3]));

		if (!range1.Coincide(range2))
		{
			return -1;
		}
		else
		{
			void* dataA;
			void* dataB;
			float pen;
			if (!range1.Penetration(range2, dataA, dataB, pen))
				return 0;
			int ptA = (int)(dataA);
			int ptB = (int)(dataB);

			if (pen < cd.m_Penetration)
			{
				cd.m_Penetration = pen;
				cd.m_CollideNormal = dir;
				if (cd.m_StateFlag == 0)
					cd.m_UserData = &vertices0[ptA];
				else if (cd.m_StateFlag == 1)
					cd.m_UserData = &vertices1[ptB];
				return 1;
			}
			return 0;
		}
	}

	static bool CanBuildMinkowskiFace(const TATPhyEdge& e1, const TATPhyEdge& e2, const TATransform& tr0, const TATransform& tr1)
	{
		TATVector3 a = tr0.GetRotation() * e1.m_Faces[0]->GetNormal();

		TATVector3 b = tr0.GetRotation() * e1.m_Faces[1]->GetNormal();

		TATVector3 c = tr1.GetRotation() * e2.m_Faces[0]->GetNormal();

		TATVector3 d = tr1.GetRotation() * e2.m_Faces[1]->GetNormal();

		return IsMinkowskiFace(a, b, -c, -d);
	}

	static bool IsMinkowskiFace(const TATVector3& a, const TATVector3& b, const TATVector3& c, const TATVector3& d)
	{
		if (a.Normalized().Distance(b.Normalized()) < TAT_EPSILON2 || c.Normalized().Distance(d.Normalized()) < TAT_EPSILON2)
			return false;

		TATVector3 b_x_a = b.Cross(a);
		TATVector3 d_x_c = d.Cross(c);

		float cba = c.Dot(b_x_a);
		float dba = d.Dot(b_x_a);
		float adc = a.Dot(d_x_c);
		float bdc = b.Dot(d_x_c);

		return (cba * dba < 0 && adc * bdc < 0 && cba * bdc > 0);
	}

	static void FindClosetPt(TATSATCollideData& cd)
	{
		if (cd.m_ResFlag == 0)
		{
			cd.m_CollidePtA = cd.m_TrA * cd.m_ResVertexA->GetPosition();
			cd.m_CollidePtB = cd.m_CollidePtA + cd.m_CollideNormal * cd.m_Penetration;
		}
		else if (cd.m_ResFlag == 1)
		{
			cd.m_CollidePtB = cd.m_TrB * cd.m_ResVertexB->GetPosition();
			cd.m_CollidePtA = cd.m_CollidePtB - cd.m_CollideNormal * cd.m_Penetration;
		}
		else if (cd.m_ResFlag == 2)
		{
			TATVector3 a = cd.m_TrA * cd.m_ResEdges[0]->GetHeadPosition();
			TATVector3 b = cd.m_TrA * cd.m_ResEdges[0]->GetTailPosition();
			TATVector3 c = cd.m_TrB * cd.m_ResEdges[1]->GetHeadPosition();
			TATVector3 d = cd.m_TrB * cd.m_ResEdges[1]->GetTailPosition();

			float w1, w2;
			TATGeometryUtil::GetSegmentsClosetPt(a, b, c, d, w1, w2);
			cd.m_CollidePtA = a * w1 + b * (1 - w1);
			cd.m_CollidePtB = c * w2 + d * (1 - w2);
			//TATGeometryUtil::ClosetPtBetweenIntersectSegments(a, b, c, d, cd.m_CollidePtA, cd.m_CollidePtB);
		}
	}
};

struct TATSATDistPack
{
public:
	TATSATDistPack()
	{
		m_Dist = -TAT_MAX;
		m_ResVertexA = 0;
		m_ResVertexB = 0;
		m_ResFaceA = 0;
		m_ResFaceB = 0;
		m_ResEdges[0] = m_ResEdges[1] = 0;
		m_UserData = 0;
		m_StateFlag = -1;
	}

	struct VertexGroup
	{
	public:
		VertexGroup(TATPhyVertex* a, TATPhyVertex* b, float dist) :m_VertexA(a), m_VertexB(b), m_Dist(dist)
		{
			m_Use = true;
			m_VertexIndexA = -1;
			m_VertexIndexB = -1;
		}

		VertexGroup(int a, int b, float dist) :m_VertexIndexA(a), m_VertexIndexB(b), m_Dist(dist)
		{
			m_Use = true;
			m_VertexA = 0;
			m_VertexB = 0;
		}

		bool Equals(const VertexGroup& g)
		{
			if (m_VertexIndexA == -1)
			{
				return ((m_VertexA == g.m_VertexA && m_VertexB == g.m_VertexB) ||
					(m_VertexA == g.m_VertexB && m_VertexB == g.m_VertexA));
			}

			else if (m_VertexA == 0)
			{
				return ((m_VertexIndexA == g.m_VertexIndexA && m_VertexIndexB == g.m_VertexIndexB) ||
					(m_VertexIndexA == g.m_VertexIndexB && m_VertexIndexB == g.m_VertexIndexA));
			}

			return false;
		}

		TATPhyVertex* m_VertexA;
		TATPhyVertex* m_VertexB;

		int m_VertexIndexA;
		int m_VertexIndexB;

		float m_Dist;

		bool m_Use;
	};

	//always point from B to A
	TATVector3 m_Normal;
	TATVector3 m_ClostPtA;
	TATVector3 m_ClostPtB;

	float m_Dist;

	int m_ResVertexA, m_ResVertexB;
	int m_ResFaceA, m_ResFaceB;
	int m_ResEdges[2];

	void* m_UserData;

	int m_StateFlag; //0:ptA & faceB; 1:ptB & faceA; 2:edgeA & edgeB;

	int m_ResFlag; //0,1,2

	std::vector<TATPhyVertex*> m_ResVerticesA;
	std::vector<TATPhyVertex*> m_ResVerticesB;

	std::vector<VertexGroup> m_VertexGroups;

	TATransform m_TrA;
	TATransform m_TrB;
};

//only for convex
class TATSATDistSolver
{
public:

	//if seperate return positive value other wise return negative value
	static float SolveConvexDistance(TATCollideShapeConvex* convex0, TATCollideShapeConvex* convex1, const TATransform& tr0, const TATransform& tr1, TATSATDistPack& cd)
	{
		cd.m_TrA = tr0;
		cd.m_TrB = tr1;

		TATVector3 dir = tr0 * convex0->m_LocalMassCenter -
						 tr1 * convex1->m_LocalMassCenter;

		std::vector<TATPhyFace>& faces0 = convex0->m_CollideMeshData.m_Faces;
		std::vector<TATPhyFace>& faces1 = convex1->m_CollideMeshData.m_Faces;
		std::vector<TATPhyEdge>& edges0 = convex0->m_CollideMeshData.m_Edges;
		std::vector<TATPhyEdge>& edges1 = convex1->m_CollideMeshData.m_Edges;

		TATVector3 testDir;
		int res;

		cd.m_StateFlag = 0;
		for (int i = 0; i < (int)faces1.size(); i++)
		{
			testDir = tr1.GetRotation() * faces1[i].GetNormal();

			if (testDir.Dot(dir) < 0)
			{
				testDir = -testDir;
			}

			res = TestOneDir(convex0, convex1, testDir, cd);
			if (res == 1)
			{
				cd.m_ResFlag = cd.m_StateFlag;
			}
		}

		cd.m_StateFlag = 1;
		for (int i = 0; i < (int)faces0.size(); i++)
		{
			testDir = tr0.GetRotation() * faces0[i].GetNormal();

			//dir always from rb0 to rb1
			if (testDir.Dot(dir) < 0)
			{
				testDir = -testDir;
			}

			res = TestOneDir(convex0, convex1, testDir, cd);
			if (res == 1)
			{
				cd.m_ResFlag = cd.m_StateFlag;
			}
		}

		cd.m_StateFlag = 2;
		for (int i = 0; i < (int)edges0.size(); i++)
		{
			const TATPhyEdge& e0 = edges0[i];
			for (int j = 0; j < (int)edges1.size(); j++)
			{
				const TATPhyEdge& e1 = edges1[j];

				if (TATSAT::CanBuildMinkowskiFace(e0, e1, tr0, tr1))
				{
					testDir = (tr0.GetRotation() * e0.GetDirection()).Cross(tr1.GetRotation() * e1.GetDirection());
					testDir.SafeNormalize();

					if (testDir.Dot(dir) < 0)
					{
						testDir = -testDir;
					}

					res = TestOneDir(convex0, convex1, testDir, cd);
					if (res == 1)
					{
						cd.m_ResFlag = cd.m_StateFlag;
					}
				}
			}
		}

		TATVector3 va, vb;
		float min_dist = TAT_MAX;
		float dist;

		float max_sep = -TAT_MAX;
		int max_index = -1;

		for (int i = 0; i < cd.m_VertexGroups.size(); ++i)
		{
			if (cd.m_VertexGroups[i].m_Dist > max_sep)
			{
				max_sep = cd.m_VertexGroups[i].m_Dist;
				max_index = i;
			}
		}

		int usecnt = cd.m_VertexGroups.size();
		for (int i = 0; i < cd.m_VertexGroups.size(); ++i)
		{
			if (cd.m_VertexGroups[i].m_Dist < cd.m_VertexGroups[max_index].m_Dist - TAT_EPSILON2)
			{
				cd.m_VertexGroups[i].m_Use = false;
				usecnt--;
			}

		}

		for (int i = 0; i < (int)cd.m_VertexGroups.size() - 1; ++i)
		{
			for (int j = i + 1; j < cd.m_VertexGroups.size(); ++j)
			{
				if (cd.m_VertexGroups[i].Equals(cd.m_VertexGroups[j]) && usecnt > 1)
				{
					usecnt--;
					cd.m_VertexGroups[j].m_Use = false;
				}
			}
		}

		for (int i = 0; i < cd.m_VertexGroups.size(); ++i)
		{
			if (cd.m_VertexGroups[i].m_Use)
			{
				FindClosetFeature(cd.m_VertexGroups[i].m_VertexA, cd.m_VertexGroups[i].m_VertexB, cd.m_TrA, cd.m_TrB, va, vb);
				dist = va.Distance(vb);
				if (dist < min_dist)
				{
					min_dist = dist;
					cd.m_ClostPtA = va;
					cd.m_ClostPtB = vb;
				}
			}
		}


		if (max_sep > 0)
		{
			cd.m_Normal = (cd.m_ClostPtA - cd.m_ClostPtB);
			cd.m_Normal.SafeNormalize();
			return min_dist;
		}

		else
		{
			cd.m_Normal = (cd.m_ClostPtB - cd.m_ClostPtA);
			cd.m_Normal.SafeNormalize();
			return -min_dist;
		}

	}

	static void FindClosetFeature(TATPhyVertex* p0, TATPhyVertex* p1,const TATransform& tr0,const TATransform& tr1,TATVector3& va,TATVector3& vb)
	{
		std::vector<TATPhyFace*>& faces0 = p0->m_Faces;
		std::vector<TATPhyFace*>& faces1 = p1->m_Faces;
		std::vector<TATPhyEdge*>& edges0 = p0->m_Edges;
		std::vector<TATPhyEdge*>& edges1 = p1->m_Edges;
		TATVector3 pa = tr0 * p0->m_Position;
		TATVector3 pb = tr1 * p1->m_Position;

		float min_dist = TAT_MAX;
		float dist;

		for (int i = 0; i < faces1.size(); ++i)
		{
			TATVector3 face[3]
			{
				tr1 * faces1[i]->m_Vertices[0]->m_Position,
				tr1 * faces1[i]->m_Vertices[1]->m_Position,
				tr1 * faces1[i]->m_Vertices[2]->m_Position
			};
			float w[3];
			dist = pa.Distance(TATGeometryUtil::ClosetPtOnTri(pa, face[0], face[1], face[2], w));
			if (dist < min_dist)
			{
				min_dist = dist;
				va = pa;
				vb = face[0] * w[0] + face[1] * w[1] + face[2] * w[2];
			}
		}

		for (int i = 0; i < faces0.size(); ++i)
		{
			TATVector3 face[3]
			{
				tr0* faces0[i]->m_Vertices[0]->m_Position,
				tr0* faces0[i]->m_Vertices[1]->m_Position,
				tr0* faces0[i]->m_Vertices[2]->m_Position
			};
			float w[3];
			dist = pb.Distance(TATGeometryUtil::ClosetPtOnTri(pb, face[0], face[1], face[2], w));
			if (dist < min_dist)
			{
				min_dist = dist;
				va = face[0] * w[0] + face[1] * w[1] + face[2] * w[2];
				vb = pb;
			}
		}

		for (int i = 0; i < edges0.size(); ++i)
		{
			TATPhyEdge* ea = edges0[i];
			for (int j = 0; j < edges1.size(); ++j)
			{
				TATPhyEdge* eb = edges1[j];

				TATVector3 a = tr0 * ea->GetHeadPosition();
				TATVector3 b = tr0 * ea->GetTailPosition();
				TATVector3 c = tr1 * eb->GetHeadPosition();
				TATVector3 d = tr1 * eb->GetTailPosition();

				float w1, w2;
				dist = TATGeometryUtil::GetSegmentsClosetPt(a, b, c, d, w1, w2);
				if (dist < min_dist)
				{
					min_dist = dist;
					va = a * w1 + b * (1 - w1);
					vb = c * w2 + d * (1 - w2);
				}
			}
		}
	}

	static void FindClosetPt(TATSATCollideData& cd)
	{
		if (cd.m_ResFlag == 0)
		{
			std::vector<TATPhyFace*>& faces = cd.m_ResVertexB->m_Faces;
			cd.m_CollidePtA = cd.m_TrA * cd.m_ResVertexA->GetPosition();

			float min_dist = TAT_MAX;
			float dist;
			for (int i = 0; i < faces.size(); ++i)
			{
				TATVector3 face[3]
				{
					cd.m_TrB * faces[i]->m_Vertices[0]->m_Position,
					cd.m_TrB * faces[i]->m_Vertices[1]->m_Position,
					cd.m_TrB * faces[i]->m_Vertices[2]->m_Position
				};
				float w[3];
				dist = cd.m_CollidePtA.Distance(TATGeometryUtil::ClosetPtOnTri(cd.m_CollidePtA, face[0], face[1], face[2], w));
				if (dist < min_dist)
				{
					min_dist = dist;
					cd.m_CollidePtB = face[0] * w[0] + face[1] * w[1] + face[2] * w[2];
				}
			}
			//cd.m_CollidePtB = cd.m_CollidePtA - cd.m_CollideNormal * cd.m_Dist;
		}
		else if (cd.m_ResFlag == 1)
		{
			std::vector<TATPhyFace*>& faces = cd.m_ResVertexA->m_Faces;

			cd.m_CollidePtB = cd.m_TrB * cd.m_ResVertexB->GetPosition();
			float min_dist = TAT_MAX;
			float dist;
			for (int i = 0; i < faces.size(); ++i)
			{
				TATVector3 face[3]
				{
					cd.m_TrA * faces[i]->m_Vertices[0]->m_Position,
					cd.m_TrA * faces[i]->m_Vertices[1]->m_Position,
					cd.m_TrA * faces[i]->m_Vertices[2]->m_Position
				};
				float w[3];
				dist = cd.m_CollidePtB.Distance(TATGeometryUtil::ClosetPtOnTri(cd.m_CollidePtB, face[0], face[1], face[2], w));
				if (dist < min_dist)
				{
					min_dist = dist;
					cd.m_CollidePtA = face[0] * w[0] + face[1] * w[1] + face[2] * w[2];
				}
			}

			//cd.m_CollidePtA = cd.m_CollidePtB + cd.m_CollideNormal * cd.m_Dist;
		}
		else if (cd.m_ResFlag == 2)
		{
			std::vector<TATPhyEdge*>& e0 = cd.m_ResVertexA->m_Edges;
			std::vector<TATPhyEdge*>& e1 = cd.m_ResVertexB->m_Edges;
			float dist;
			float min_dist = TAT_MAX;
			for (int i = 0; i < e0.size(); ++i)
			{
				TATPhyEdge* ea = e0[i];
				for (int j = 0; j < e1.size(); ++j)
				{
					TATPhyEdge* eb = e1[j];

					TATVector3 a = cd.m_TrA * ea->GetHeadPosition();
					TATVector3 b = cd.m_TrA * ea->GetTailPosition();
					TATVector3 c = cd.m_TrB * eb->GetHeadPosition();
					TATVector3 d = cd.m_TrB * eb->GetTailPosition();

					float w1, w2;
					dist = TATGeometryUtil::GetSegmentsClosetPt(a, b, c, d, w1, w2);
					if (dist < min_dist)
					{
						min_dist = dist;
						cd.m_CollidePtA = a * w1 + b * (1 - w1);
						cd.m_CollidePtB = c * w2 + d * (1 - w2);
					}
				}
			}
		}
	}

	static float SolveTriangleDistance(TATVector3* face0, TATVector3* face1, TATSATDistPack& dp)
	{
		TATVector3 ct0 = (face0[0] + face0[1] + face0[2]) / 3;
		TATVector3 ct1 = (face1[0] + face1[1] + face1[2]) / 3;

		TATVector3 normal0 = ((face0[1] - face0[0]).Cross(face0[2] - face0[0])).Normalized();
		TATVector3 normal1 = ((face1[1] - face1[0]).Cross(face1[2] - face1[0])).Normalized();

		TATVector3 dir = ct0 - ct1;

		TATVector3 edge0[3]{ face0[1] - face0[0],face0[2] - face0[1],face0[0] - face0[2] };
		TATVector3 edge1[3]{ face1[1] - face1[0],face1[2] - face1[1],face1[0] - face1[2] };
		int edgeIndex[6]{ 0,1,1,2,2,0 };

		TATVector3 testDir;
		int res;

		dp.m_StateFlag = 0;
		{
			testDir = normal1;

			if (testDir.Dot(dir) < 0)
			{
				testDir = -testDir;
			}

			res = TestOneDir(face0, face1, testDir, dp);
			if (res == 1)
			{
				dp.m_ResFlag = dp.m_StateFlag;
			}
		}

		dp.m_StateFlag = 1;
		{
			testDir = normal0;

			if (testDir.Dot(dir) < 0)
			{
				testDir = -testDir;
			}

			res = TestOneDir(face0, face1, testDir, dp);
			if (res == 1)
			{
				dp.m_ResFlag = dp.m_StateFlag;
			}
		}

		dp.m_StateFlag = 2;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				//if (TATSAT::CanBuildMinkowskiFace(e0, e1, tr0, tr1))
				{
					testDir = (edge0[i]).Cross(edge1[j]).Normalized();

					if (testDir.Dot(dir) < 0)
					{
						testDir = -testDir;
					}

					res = TestOneDir(face0, face1, testDir, dp);
					if (res == 1)
					{
						dp.m_ResFlag = dp.m_StateFlag;
					}
				}
			}
		}

		TATVector3 va, vb;
		float min_dist = TAT_MAX;
		float dist;

		float max_sep = -TAT_MAX;
		int max_index = -1;

		//find max seperate distance
		for (int i = 0; i < dp.m_VertexGroups.size(); ++i)
		{
			if (dp.m_VertexGroups[i].m_Dist > max_sep)
			{
				max_sep = dp.m_VertexGroups[i].m_Dist;
				max_index = i;
			}
		}

		//eliminate the wrong pair
		for (int i = 0; i < dp.m_VertexGroups.size(); ++i)
		{
			if (dp.m_VertexGroups[i].m_Dist < dp.m_VertexGroups[max_index].m_Dist - TAT_EPSILON2)
				dp.m_VertexGroups[i].m_Use = false;
		}

		//eliminate same pair
		for (int i = 0; i < (int)dp.m_VertexGroups.size() - 1; ++i)
		{
			for (int j = i + 1; j < (int)dp.m_VertexGroups.size(); ++j)
			{
				if (dp.m_VertexGroups[i].Equals(dp.m_VertexGroups[j]))
				{
					dp.m_VertexGroups[j].m_Use = false;
				}
			}
		}

		for (int i = 0; i < dp.m_VertexGroups.size(); ++i)
		{
			if (dp.m_VertexGroups[i].m_Use)
			{
				int v0 = dp.m_VertexGroups[i].m_VertexIndexA;
				int v1 = dp.m_VertexGroups[i].m_VertexIndexB;

				float weight[3];
				TATVector3 pt = TATGeometryUtil::ClosetPtOnTri(face0[v0], face1[0], face1[1], face1[2], weight);
				dist = pt.Distance(face0[v0]);
				if (dist < min_dist)
				{
					min_dist = dist;
					dp.m_ClostPtA = face0[v0];
					dp.m_ClostPtB = pt;
				}

				pt = TATGeometryUtil::ClosetPtOnTri(face1[v1], face0[0], face0[1], face0[2], weight);
				dist = pt.Distance(face1[v1]);
				if (dist < min_dist)
				{
					min_dist = dist;
					dp.m_ClostPtA = pt;
					dp.m_ClostPtB = face1[v1];
				}

				TATVector3 e0[2]{ face0[v0], face0[(v0 + 1) % 3] };
				TATVector3 e1[2]{ face0[v0], face0[(v0 + 2) % 3] };
				TATVector3 e2[2]{ face1[v1], face1[(v1 + 1) % 3] };
				TATVector3 e3[2]{ face1[v1], face1[(v1 + 2) % 3] };

				float w1, w2;
				dist = TATGeometryUtil::GetSegmentsClosetPt(e0[0], e0[1], e2[0], e2[1], w1, w2);
				if (dist < min_dist)
				{
					min_dist = dist;
					dp.m_ClostPtA = w1 * e0[0] + (1 - w1) * e0[1];
					dp.m_ClostPtB = w2 * e2[0] + (1 - w2) * e2[1];
				}

				dist = TATGeometryUtil::GetSegmentsClosetPt(e0[0], e0[1], e3[0], e3[1], w1, w2);
				if (dist < min_dist)
				{
					min_dist = dist;
					dp.m_ClostPtA = w1 * e0[0] + (1 - w1) * e0[1];
					dp.m_ClostPtB = w2 * e3[0] + (1 - w2) * e3[1];
				}

				dist = TATGeometryUtil::GetSegmentsClosetPt(e1[0], e1[1], e2[0], e2[1], w1, w2);
				if (dist < min_dist)
				{
					min_dist = dist;
					dp.m_ClostPtA = w1 * e1[0] + (1 - w1) * e1[1];
					dp.m_ClostPtB = w2 * e2[0] + (1 - w2) * e2[1];
				}

				dist = TATGeometryUtil::GetSegmentsClosetPt(e1[0], e1[1], e3[0], e3[1], w1, w2);
				if (dist < min_dist)
				{
					min_dist = dist;
					dp.m_ClostPtA = w1 * e1[0] + (1 - w1) * e1[1];
					dp.m_ClostPtB = w2 * e3[0] + (1 - w2) * e3[1];
				}
			}
		}

		return max_sep > 0 ? dp.m_ClostPtA.Distance(dp.m_ClostPtB) : -dp.m_ClostPtA.Distance(dp.m_ClostPtB);
	}

	static float SolveTriConvexDistance(TATVector3* tri, TATCollideShapeConvex* convex, const TATransform& tr, TATSATDistPack& dp)
	{
		TATCollideShapeConvex* con = CreateTriConvex(tri);
		TATransform tr0;

		float dist = SolveConvexDistance(con, convex, tr0, tr, dp);

		delete con;
		return dist;
	}

	//test on wcs
	//edge data must be recalculate
	static int TestOneDir(TATCollideShapeConvex* convex0, TATCollideShapeConvex* convex1, const TATVector3& dir, TATSATDistPack& cd)
	{

		std::vector<TATPhyVertex>& vertices0 = convex0->m_CollideMeshData.m_Vertices;
		std::vector<TATPhyVertex>& vertices1 = convex1->m_CollideMeshData.m_Vertices;

		TATransform tr0 = cd.m_TrA;
		TATransform tr1 = cd.m_TrB;

		int indices[4];
		float projs[4];
		TATSAT::Support(vertices0, vertices1, tr0, tr1, dir, indices, projs);

		TATRange range1(projs[0], projs[1]);
		TATRange range2(projs[2], projs[3]);

		{
			void* dataA;
			void* dataB;
			float dist;
			int item[2];

			range1.Distance(range2, dist, item);

			int ptA = indices[item[0]];
			int ptB = indices[item[1]];

			if (dist >= cd.m_Dist - TAT_EPSILON2)
			{
				cd.m_Dist = dist;

				cd.m_ResVerticesA.push_back(&vertices0[ptA]);
				cd.m_ResVerticesB.push_back(&vertices1[ptB]);

				cd.m_VertexGroups.push_back(TATSATDistPack::VertexGroup(&vertices0[ptA], &vertices1[ptB], dist));

				return 1;
			}
			return 0;
		}
	}

	static int TestOneDir(TATVector3* face0, TATVector3* face1, const TATVector3& dir, TATSATDistPack& dp)
	{
		int indices[4];
		float projs[4];
		Support(face0, 3, face1, 3, dir, indices, projs);

		TATRange range1(projs[0], projs[1]);
		TATRange range2(projs[2], projs[3]);

		range1.SetUserData((void*)(indices[0]), (void*)(indices[1]));
		range2.SetUserData((void*)(indices[2]), (void*)(indices[3]));

		{
			void* dataA;
			void* dataB;
			float dist;
			int item[2];

			range1.Distance(range2, dist, item);

			int ptA = indices[item[0]];
			int ptB = indices[item[1]];

			if (dist >= dp.m_Dist - TAT_EPSILON2)
			{
				dp.m_Dist = dist;

				dp.m_VertexGroups.push_back(TATSATDistPack::VertexGroup(ptA, ptB, dist));

				return 1;
			}
			return 0;
		}
	}

	static int TestOneDir(TATVector3* v0, int n0, TATVector3* v1, int n1, const TATVector3& dir, TATSATDistPack& dp)
	{
		int indices[4];
		float projs[4];
		Support(v0, n0, v1, n1, dir, indices, projs);

		TATRange range1(projs[0], projs[1]);
		TATRange range2(projs[2], projs[3]);

		{
			void* dataA;
			void* dataB;
			float dist;
			int item[2];

			range1.Distance(range2, dist, item);

			int ptA = indices[item[0]];
			int ptB = indices[item[1]];

			if (dist >= dp.m_Dist - TAT_EPSILON2)
			{
				dp.m_Dist = dist;

				dp.m_VertexGroups.push_back(TATSATDistPack::VertexGroup(ptA, ptB, dist));

				return 1;
			}
			return 0;
		}
	}

	static void Support(TATVector3* v0, int n0, TATVector3* v1, int n1, const TATVector3& dir, int* indices, float* projects)
	{
		int i = 0, j = 0;
		float proj;
		indices[0] = indices[2] = -1;
		indices[1] = indices[3] = -1;
		projects[0] = projects[2] = TAT_MAX;
		projects[1] = projects[3] = -TAT_MAX;
		while (i < n0 || j < n1)
		{
			if (i < n0)
			{
				proj = v0[i].Dot(dir);
				if (proj < projects[0])
				{
					projects[0] = proj;
					indices[0] = i;
				}
				if (proj > projects[1])
				{
					projects[1] = proj;
					indices[1] = i;
				}
			}
			i++;
			if (j < n1)
			{
				proj = v1[j].Dot(dir);
				if (proj < projects[2])
				{
					projects[2] = proj;
					indices[2] = j;
				}
				if (proj > projects[3])
				{
					projects[3] = proj;
					indices[3] = j;
				}

			}
			j++;
		}

	}

	static TATCollideShapeConvex* CreateTriConvex(TATVector3* tri)
	{
		TATPhyMeshData cmesh;
		cmesh.m_Vertices.push_back(TATPhyVertex(tri[0]));
		cmesh.m_Vertices.push_back(TATPhyVertex(tri[1]));
		cmesh.m_Vertices.push_back(TATPhyVertex(tri[2]));

		TATCollideShapeConvex * convex = new TATCollideShapeConvex(cmesh, 1, false);
		TATPhyMeshData & mesh = convex->m_CollideMeshData;

		mesh.m_Faces.push_back(TATPhyFace(0, 1, 2));
		mesh.m_Faces.push_back(TATPhyFace(0, 2, 1));
		mesh.m_Faces[0].m_Normal = (tri[1] - tri[0]).Cross(tri[2] - tri[0]).Normalized();
		mesh.m_Faces[1].m_Normal = -mesh.m_Faces[0].m_Normal;

		mesh.m_Edges.push_back(TATPhyEdge(0, 1));
		mesh.m_Edges.push_back(TATPhyEdge(1, 2));
		mesh.m_Edges.push_back(TATPhyEdge(2, 0));

		for (int i = 0; i < 3; ++i)
		{
			mesh.m_Vertices[i].m_FaceIndices.push_back(0);
			mesh.m_Vertices[i].m_Faces.push_back(&mesh.m_Faces[0]);
			mesh.m_Vertices[i].m_FaceIndices.push_back(1);
			mesh.m_Vertices[i].m_Faces.push_back(&mesh.m_Faces[1]);
		}

		mesh.m_Faces[0].m_Normal = (tri[1] - tri[0]).Cross(tri[2] - tri[0]).Normalized();
		mesh.m_Faces[0].m_Vertices[0] = &mesh.m_Vertices[0];
		mesh.m_Faces[0].m_Vertices[1] = &mesh.m_Vertices[1];
		mesh.m_Faces[0].m_Vertices[2] = &mesh.m_Vertices[2];
		mesh.m_Faces[1].m_Vertices[0] = &mesh.m_Vertices[0];
		mesh.m_Faces[1].m_Vertices[1] = &mesh.m_Vertices[2];
		mesh.m_Faces[1].m_Vertices[2] = &mesh.m_Vertices[1];

		mesh.m_Edges[0].m_Vertices[0] = &mesh.m_Vertices[0];
		mesh.m_Edges[0].m_Vertices[1] = &mesh.m_Vertices[1];
		mesh.m_Edges[1].m_Vertices[0] = &mesh.m_Vertices[1];
		mesh.m_Edges[1].m_Vertices[1] = &mesh.m_Vertices[2];
		mesh.m_Edges[2].m_Vertices[0] = &mesh.m_Vertices[2];
		mesh.m_Edges[2].m_Vertices[1] = &mesh.m_Vertices[0];

		for (int i = 0; i < 3; ++i)
		{
			mesh.m_Edges[i].m_Direction = (mesh.m_Edges[i].m_Vertices[1]->m_Position - mesh.m_Edges[i].m_Vertices[0]->m_Position).Normalized();
			mesh.m_Edges[i].m_FaceIndices[0] = 0;
			mesh.m_Edges[i].m_FaceIndices[1] = 1;
			mesh.m_Edges[i].m_Faces[0] = &mesh.m_Faces[0];
			mesh.m_Edges[i].m_Faces[1] = &mesh.m_Faces[1];
		}

		mesh.m_Vertices[0].m_EdgeIndices.push_back(0);
		mesh.m_Vertices[0].m_EdgeIndices.push_back(2);
		mesh.m_Vertices[1].m_EdgeIndices.push_back(0);
		mesh.m_Vertices[1].m_EdgeIndices.push_back(1);
		mesh.m_Vertices[2].m_EdgeIndices.push_back(1);
		mesh.m_Vertices[2].m_EdgeIndices.push_back(2);

		mesh.m_Vertices[0].m_Edges.push_back(&mesh.m_Edges[0]);
		mesh.m_Vertices[0].m_Edges.push_back(&mesh.m_Edges[2]);
		mesh.m_Vertices[1].m_Edges.push_back(&mesh.m_Edges[0]);
		mesh.m_Vertices[1].m_Edges.push_back(&mesh.m_Edges[1]);
		mesh.m_Vertices[2].m_Edges.push_back(&mesh.m_Edges[1]);
		mesh.m_Vertices[2].m_Edges.push_back(&mesh.m_Edges[2]);

		return convex;
	}
};
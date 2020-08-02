#pragma once
#include "../TATCommon/TATransform.h"
#include "../TATDynamics/TATRigidBody.h"
#include "../TATCommon/TATRange.h"
#include "../TATGeometry/TATGeometryComputer.h"

struct TATRigidBodyGroup
{
public:
	TATRigidBodyGroup(TATRigidBody* rba, TATRigidBody* rbb) :m_RbA(rba), m_RbB(rbb)
	{

	}
	int m_RbIndexA;
	int m_RbIndexB;
	TATRigidBody* m_RbA;
	TATRigidBody* m_RbB;
};

struct TATSATCollideData
{
public:
	TATSATCollideData()
	{
		m_Penetration = TAT_MAX;
		m_ResFaceA = 0;
		m_ResFaceB = 0;
		m_ResVertexA = 0;
		m_ResVertexB = 0;
		m_ResEdges[0] = m_ResEdges[1] = 0;
		m_UserData = 0;
		m_RbA = 0;
		m_RbB = 0;
	}

	TATVector3 m_CollideNormal;
	TATVector3 m_CollidePtA;
	TATVector3 m_CollidePtB;

	float m_Penetration;

	TATransform m_TrA;
	TATransform m_TrB;

	TATPhyVertex* m_ResVertexA, *m_ResVertexB;
	TATPhyFace* m_ResFaceA, *m_ResFaceB;
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

	static bool SeparateAxisTest(const TATRigidBodyGroup& rbGroup, TATSATCollideData& cd)
	{
		TATRigidBody* rb0 = rbGroup.m_RbA;
		TATRigidBody* rb1 = rbGroup.m_RbB;

		TATransform tr0 = rb0->GetWorldTransform();
		TATransform tr1 = rb1->GetWorldTransform();

		cd.m_TrA = tr0;
		cd.m_TrB = tr1;

		TATVector3 dir = tr1 * rb1->m_CollideShape->m_LocalMassCenter -
						 tr0 * rb0->m_CollideShape->m_LocalMassCenter;

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
		const TATransform& tr0, const TATransform& tr1, const TATVector3& dir, int* indices,float* projects)
	{
		int i = 0, j = 0;
		float proj;
		indices[0] = indices[2] = -1;
		indices[1] = indices[3] = -1;
		projects[1] = projects[2] =  TAT_MAX;
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

		range1.SetUserData((void*)(&indices[0]), (void*)(&indices[1]));
		range2.SetUserData((void*)(&indices[2]), (void*)(&indices[3]));

		if (!range1.Coincide(range2))
		{
			return -1;
		}
		else
		{
			void* dataA;
			void* dataB;
			float pen = range1.Penetration(range2, dataA, dataB);
			int ptA = (int)(dataA);
			int ptB = (int)(dataB);

			if (pen < cd.m_Penetration)
			{
				cd.m_Penetration = pen;
				cd.m_CollideNormal = dir;
				if(cd.m_StateFlag == 0)
					cd.m_UserData = &vertices0[ptA];
				else if(cd.m_StateFlag == 1)
					cd.m_UserData = &vertices1[ptB];
				return 1;
			}
			return 0;
		}
	}

	static bool CanBuildMinkowskiFace(const TATPhyEdge& e1, const TATPhyEdge& e2,const TATransform& tr0,const TATransform& tr1)
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
			cd.m_CollidePtB = cd.m_TrB * cd.m_ResVertexB->GetPosition();
			cd.m_CollidePtA = cd.m_CollidePtB + cd.m_CollideNormal * cd.m_Penetration;
		}
		else if (cd.m_ResFlag == 1)
		{
			cd.m_CollidePtA = cd.m_TrA * cd.m_ResVertexA->GetPosition();
			cd.m_CollidePtB = cd.m_CollidePtA - cd.m_CollideNormal * cd.m_Penetration;
		}
		else if (cd.m_ResFlag == 2)
		{
			TATVector3 a = cd.m_TrA * cd.m_ResEdges[0]->GetHeadPosition();
			TATVector3 b = cd.m_TrA * cd.m_ResEdges[0]->GetTailPosition();
			TATVector3 c = cd.m_TrB * cd.m_ResEdges[1]->GetHeadPosition();
			TATVector3 d = cd.m_TrB * cd.m_ResEdges[1]->GetTailPosition();

			TATGeometryUtil::ClosetPtBetweenIntersectSegments(a, b, c, d, cd.m_CollidePtA, cd.m_CollidePtB);
		}
	}
};

//only for convex
class TATSATDist
{
public:

	static float ConvexDistance(TATCollideShapeConvex* convex0,TATCollideShapeConvex* convex1,const TATransform& tr0, const TATransform& tr1,TATSATCollideData& cd)
	{
		cd.m_TrA = tr0;
		cd.m_TrB = tr1;

		TATVector3 dir = tr1 * convex0->m_LocalMassCenter -
						 tr0 * convex0->m_LocalMassCenter;

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
				cd.m_ResVertexA = (TATPhyVertex*)cd.m_UserData;
				cd.m_ResFaceB = &faces1[i];
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
				cd.m_ResVertexB = (TATPhyVertex*)cd.m_UserData;
				cd.m_ResFaceA = &faces0[i];
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
					testDir = (tr0 * e0.GetDirection()).Cross(tr1 * e1.GetDirection()).Normalized();

					if (testDir.Dot(dir) < 0)
					{
						testDir = -testDir;
					}

					res = TestOneDir(convex0, convex1, testDir, cd);
					if (res == 1)
					{
						cd.m_ResEdges[0] = &edges0[i];
						cd.m_ResEdges[1] = &edges1[j];
						cd.m_ResFlag = cd.m_StateFlag;
					}
				}
			}
		}

		TATSAT::FindClosetPt(cd);
		return -cd.m_Penetration;
	}

	//test on wcs
	//edge data must be recalculate
	static int TestOneDir(TATCollideShapeConvex* convex0, TATCollideShapeConvex* convex1, const TATVector3& dir, TATSATCollideData& cd)
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

		range1.SetUserData((void*)(&indices[0]), (void*)(&indices[1]));
		range2.SetUserData((void*)(&indices[2]), (void*)(&indices[3]));

		{
			void* dataA;
			void* dataB;
			float pen = range1.Penetration(range2, dataA, dataB);
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
};
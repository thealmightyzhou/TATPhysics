#pragma once
#include "../../TATPhysics/TATResources/TATetLoader.h"
#include "../../TATPhysics/TATResources/TATObjLoader.h"
#include "../../TATPhysics/TATResources/TATFileStream.h"
//#include "../../TATPhysics/TATBroadPhase/TATBvh.h"
#include "../../TATPhysics/TATBroadPhase/TATBvh.cpp"
#include "../../TATPhysics/TATBroadPhase/TATBvhCollideCallBack.h"
#include <map>

/******************************
|tmodel format:
|1.vertex count n
|2.n lines of vertex | each line contains <x y z> <nx ny nz> <u v> <tx ty tz> <1/0 (rend vert or not)>
|3.face count n
|4.n lines of face (each line contains <v0 v1 v2>)
|5.tet count n
|6.n lines of tet (each line contains <v0 v1 v2 v3>)
******************************/

class TATetModelConverter
{
public:

	static void Convert()
	{
		std::vector<TString> allObjs;
		TATFileStream::FetchAllFilesBelow("", "obj", allObjs);
		for (int i = 0; i < allObjs.size(); ++i)
		{
			TString tetName = allObjs[i].FetchInnerName().Append(".ttet");
			if (!TATFileStream::FileExists(tetName))
				continue;

			TATObjLoader objLoader(allObjs[i]);
			TATetLoader tetLoader(tetName);

			TATModelBuffer result;
			CombineObjTet(objLoader.m_Buffer, tetLoader.m_Buffer, result);

			std::vector<TString> tModelLines;

			int offset = 0;
			int vertCnt = (int)result.vertexBuffer.size();
			int faceCnt = (int)result.faceBuffer.size();
			int tetCnt = (int)result.tetraBuffer.size();
			tModelLines.resize(vertCnt + faceCnt + tetCnt + 3);
			tModelLines[offset++] = TString::ConvertInt(vertCnt);
			for (int i = 0; i < vertCnt; ++i)
			{
				const TATVertexBuffer& v = result.vertexBuffer[i];
				tModelLines[offset++] =
					  TString::ConvertFloat(v.m_Position[0]) + " "
					+ TString::ConvertFloat(v.m_Position[1]) + " "
					+ TString::ConvertFloat(v.m_Position[2]) + " "
					+ TString::ConvertFloat(v.m_Normal[0]) + " " 
					+ TString::ConvertFloat(v.m_Normal[1]) + " " 
					+ TString::ConvertFloat(v.m_Normal[2]) + " "
					+ TString::ConvertFloat(v.m_TexCoordinates[0][0]) + " "
					+ TString::ConvertFloat(v.m_TexCoordinates[0][1]) + " "
					+ TString::ConvertFloat(v.m_Tangent[0]) + " "
					+ TString::ConvertFloat(v.m_Tangent[1]) + " "
					+ TString::ConvertFloat(v.m_Tangent[2]) + " "
					+ TString::ConvertInt((int)v.m_Data);
			}
			tModelLines[offset++] = TString::ConvertInt(faceCnt);
			for (int i = 0; i < faceCnt; ++i)
			{
				const TATFaceBuffer& f = result.faceBuffer[i];
				tModelLines[offset++] =	TString::ConvertInt(f.v1) + " "
									  + TString::ConvertInt(f.v2) + " "
									  + TString::ConvertInt(f.v3);
			}
			tModelLines[offset++] = TString::ConvertInt(tetCnt);
			for (int i = 0; i < tetCnt; ++i)
			{
				const TATetraBuffer& t = result.tetraBuffer[i];
				tModelLines[offset++] =	TString::ConvertInt(t.v1) + " "
									  + TString::ConvertInt(t.v2) + " "
									  + TString::ConvertInt(t.v3) + " "
									  + TString::ConvertInt(t.v4);

			}

			TString tModelName = tetName.FetchInnerName().Append(".tmodel");
			TATFileStream::WriteFile(tModelName, tModelLines);
		}
	}

	class ObjTetCombineCallBack :public TATBvhCollideCallBack
	{
	public:
		ObjTetCombineCallBack(TATModelBuffer& objBuffer, TATModelBuffer& tetBuffer, std::map<int, int>& vertexMapping)
			:m_ObjBuffer(objBuffer), m_TetBuffer(tetBuffer), m_VertexMapping(vertexMapping)
		{}

		virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
		{
			float eps = 10.0f;

			int vert0 = (int)(node1->m_Data);
			int vert1 = (int)(node2->m_Data);

			TATVector3 pos0 = m_ObjBuffer.vertexBuffer[vert0].m_Position;
			TATVector3 pos1 = m_TetBuffer.vertexBuffer[vert1].m_Position;
			float dist0 = pos0.Distance(pos1);
			if (dist0 > eps)
				return;

			m_TetBuffer.vertexBuffer[vert1].m_Data = (void*)true;

			std::map<int, int>::iterator ite = m_VertexMapping.find(vert1);
			if (ite != m_VertexMapping.end())
			{
				int objVert = m_VertexMapping[vert1];
				TATVector3 pos = m_ObjBuffer.vertexBuffer[objVert].m_Position;
				float dist1 = pos.Distance(pos1);
				if (dist0 < dist1)
					m_VertexMapping[vert1] = vert0;
			}
			else
				m_VertexMapping[vert1] = vert0;
		}

		TATModelBuffer& m_ObjBuffer;
		TATModelBuffer& m_TetBuffer;
		std::map<int, int>& m_VertexMapping;
	};

	static void CombineObjTet(TATModelBuffer& objBuffer, TATModelBuffer& tetBuffer,TATModelBuffer& result)
	{
		float extent = 10.0f;
		TATBvh objTree;
		TATBvh tetTree;

		std::map<int, int> vertexMap;

		for (int i = 0; i < tetBuffer.vertexBuffer.size(); ++i)
		{
			TATVector3 pos(tetBuffer.vertexBuffer[i].m_Position);
			TATBVNode* node = tetTree.InsertAabbNode(pos - TATVector3::One() * extent, pos + TATVector3::One() * extent);
			node->m_Data = (void*)(i);
		}
		tetTree.FinishBuild();

		for (int i = 0; i < objBuffer.vertexBuffer.size(); ++i)
		{
			TATVector3 pos(objBuffer.vertexBuffer[i].m_Position);
			TATBVNode* node = objTree.InsertAabbNode(pos - TATVector3::One() * extent, pos + TATVector3::One() * extent);
			node->m_Data = (void*)(i);
		}
		objTree.FinishBuild();

		ObjTetCombineCallBack cb(objBuffer, tetBuffer, vertexMap);
		tetTree.CollideWithBVTree(&objTree, &cb);

		result.vertexBuffer.resize(tetBuffer.vertexBuffer.size());
		result.faceBuffer.resize(tetBuffer.faceBuffer.size());
		result.tetraBuffer.resize(tetBuffer.tetraBuffer.size());

		for (int i = 0; i < result.vertexBuffer.size(); ++i)
		{
			TATVertexBuffer& vertex = result.vertexBuffer[i];
			vertex.m_Position[0] = tetBuffer.vertexBuffer[i].m_Position[0];
			vertex.m_Position[1] = tetBuffer.vertexBuffer[i].m_Position[1];
			vertex.m_Position[2] = tetBuffer.vertexBuffer[i].m_Position[2];

			int objVert = vertexMap[i];
			TATVertexBuffer& objVertex = objBuffer.vertexBuffer[objVert];
			vertex.m_Normal[0] = objVertex.m_Normal[0];
			vertex.m_Normal[1] = objVertex.m_Normal[1];
			vertex.m_Normal[2] = objVertex.m_Normal[2];

			vertex.m_TexCoordinates[0][0] = objVertex.m_TexCoordinates[0][0];
			vertex.m_TexCoordinates[0][1] = objVertex.m_TexCoordinates[0][1];

			vertex.m_Tangent[0] = objVertex.m_Tangent[0];
			vertex.m_Tangent[1] = objVertex.m_Tangent[1];
			vertex.m_Tangent[2] = objVertex.m_Tangent[2];
		}

		for (int i = 0; i < result.faceBuffer.size(); ++i)
		{
			TATFaceBuffer& face = result.faceBuffer[i];
			face.v1 = tetBuffer.faceBuffer[i].v1;
			face.v2 = tetBuffer.faceBuffer[i].v2;
			face.v3 = tetBuffer.faceBuffer[i].v3;
			result.vertexBuffer[face.v1 - 1].m_Data = (void*)true; //mark as render vertex
			result.vertexBuffer[face.v2 - 1].m_Data = (void*)true;
			result.vertexBuffer[face.v3 - 1].m_Data = (void*)true;
		}

		for (int i = 0; i < result.tetraBuffer.size(); ++i)
		{
			TATetraBuffer& tet = result.tetraBuffer[i];
			tet.v1 = tetBuffer.tetraBuffer[i].v1;
			tet.v2 = tetBuffer.tetraBuffer[i].v2;
			tet.v3 = tetBuffer.tetraBuffer[i].v3;
			tet.v4 = tetBuffer.tetraBuffer[i].v4;
		}
	}
};
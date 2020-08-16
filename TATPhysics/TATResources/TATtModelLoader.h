#pragma once
#include "TATModelLoader.h"
#include "TATFileStream.h"

class TATtModelLoader :public TATModelLoader
{
public:
	TATtModelLoader(const TString& name) :TATModelLoader(name)
	{
		Load(name, m_Buffer);
	}

	virtual void Load(const TString& name, TATModelBuffer& result)
	{
		std::vector<TString> tModelLines;
		std::vector<TString> datas;
		TATFileStream::ReadFileToLines(name, tModelLines);

		int offset = 0;
		int vertCnt = tModelLines[offset++].ToInt();
		result.vertexBuffer.resize(vertCnt);
		for (int i = 0; i < vertCnt; ++i)
		{
			TATVertexBuffer& v = result.vertexBuffer[i];
			tModelLines[offset++].Split(" ", datas);
			v.m_Position[0] = datas[0].ToFloat();
			v.m_Position[1] = datas[1].ToFloat();
			v.m_Position[2] = datas[2].ToFloat();
			v.m_Normal[0] = datas[3].ToFloat();
			v.m_Normal[1] = datas[4].ToFloat();
			v.m_Normal[2] = datas[5].ToFloat();
			v.m_TexCoordinates[0][0] = datas[6].ToFloat();
			v.m_TexCoordinates[0][1] = datas[7].ToFloat();
			v.m_Tangent[0] = datas[8].ToFloat();
			v.m_Tangent[1] = datas[9].ToFloat();
			v.m_Tangent[2] = datas[10].ToFloat();
			v.m_IsRendVert = (datas[11].ToInt() == 1);

			datas.clear();
		}
		
		int faceCnt = tModelLines[offset++].ToInt();
		result.faceBuffer.resize(faceCnt);
		for (int i = 0; i < faceCnt; ++i)
		{
			tModelLines[offset++].Split(" ", datas);
			TATFaceBuffer& f = result.faceBuffer[i];
			f.v1 = datas[0].ToInt() - 1;
			f.v2 = datas[1].ToInt() - 1;
			f.v3 = datas[2].ToInt() - 1;

			datas.clear();
		}

		int tetCnt = tModelLines[offset++].ToInt();
		result.tetraBuffer.resize(tetCnt);
		for (int i = 0; i < tetCnt; ++i)
		{
			tModelLines[offset++].Split(" ", datas);
			TATetraBuffer& t = result.tetraBuffer[i];
			t.v1 = datas[0].ToInt() - 1;
			t.v2 = datas[1].ToInt() - 1;
			t.v3 = datas[2].ToInt() - 1;
			t.v4 = datas[3].ToInt() - 1;

			datas.clear();
		}

		m_ModelElementMask.UseFace();
		m_ModelElementMask.UseNormal();
		m_ModelElementMask.UseTexCoordinate();
		m_ModelElementMask.m_TexCount = 1;
	}
};
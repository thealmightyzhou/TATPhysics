#pragma once
#include "TATModelLoader.h"
#include "TATFileStream.h"

class TATetLoader :public TATModelLoader
{
public:
	TATetLoader(const TString& fileName) :TATModelLoader(fileName)
	{
		Load(fileName, m_Buffer);
	}

	virtual void Load(const TString& path, TATModelBuffer& buffer)
	{
		std::vector<TString> lines;
		TATFileStream::ReadFileToLines(path, lines);
		int offset = 0;
		std::vector<TString> datas;
		int vertexCount = lines[offset++].ToInt();
		buffer.vertexBuffer.resize(vertexCount);
		for (int i = 0; i < vertexCount; ++i)
		{
			lines[offset++].Split(" ", datas);
			buffer.vertexBuffer[i].m_Position[0] = datas[0].ToFloat();
			buffer.vertexBuffer[i].m_Position[1] = datas[1].ToFloat();
			buffer.vertexBuffer[i].m_Position[2] = datas[2].ToFloat();
			datas.clear();
		}

		int tetCount = lines[offset++].ToInt();
		buffer.tetraBuffer.resize(tetCount);
		for (int i = 0; i < tetCount; ++i)
		{
			lines[offset++].Split(" ", datas);
			buffer.tetraBuffer[i].v1 = datas[1].ToInt();
			buffer.tetraBuffer[i].v2 = datas[2].ToInt();
			buffer.tetraBuffer[i].v3 = datas[3].ToInt();
			buffer.tetraBuffer[i].v4 = datas[4].ToInt();
			datas.clear();
		}

		int faceCount = lines[offset++].ToInt();
		buffer.faceBuffer.resize(faceCount);
		for (int i = 0; i < faceCount; ++i)
		{
			lines[offset++].Split(" ", datas);
			buffer.faceBuffer[i].v1 = datas[1].ToInt();
			buffer.faceBuffer[i].v2 = datas[2].ToInt();
			buffer.faceBuffer[i].v3 = datas[3].ToInt();
			datas.clear();
		}
	}
};
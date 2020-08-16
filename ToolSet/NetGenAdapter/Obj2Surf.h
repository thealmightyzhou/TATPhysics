#pragma once
#include "../../TATPhysics/TATResources/TATObjLoader.h"
#include "../../TATPhysics/TATResources/TATFileStream.h"
#include <io.h>
#include <Windows.h>
#include <stdio.h>
#include <string>
#include <iostream> 
#include <direct.h>
#include <vector>
#include <atlstr.h>

using namespace std;

/***************************
|.surf file formation:
|1.meshName
|2.vertex count n
|3.x y z (n lines)
|4.face count f
|5.v0 v1 v2 (f lines) !!attention index start at 1 while this objloader's index start at 0
|6.blank end (no need)
***************************/

class TATObj2SurfConverter
{
public:
	static void Obj2Surf()
	{
		vector<std::string> files;
		FindFile(files);

		for (int i = 0; i < files.size(); ++i)
		{
			ConvertObj(files[i]);
		}
	}

	static void ConvertObj(const TString& file)
	{
		TATObjLoader loader(file);
		const TATModelBuffer& buffer = loader.m_Buffer;

		TString surfName = file.FetchInnerName() + TString(".surf");

		std::vector<TString> content;

		int vertCount = (int)buffer.vertexBuffer.size();
		int faceCount = (int)buffer.faceBuffer.size();

		content.resize(vertCount + faceCount + 3);
		int offset = 0;
		content[offset++] = "surfacemesh";
		content[offset++] = TString::ConvertInt(vertCount);

		for (int i = 0; i < vertCount; ++i)
		{

			const TATVertexBuffer& vertex = buffer.vertexBuffer[i];
			content[offset++] = TString(" ") +
								TString::ConvertFloat(vertex.m_Position[0]) + " " +
								TString::ConvertFloat(vertex.m_Position[1]) + " " +
								TString::ConvertFloat(vertex.m_Position[2]) + " ";

			TString str = TString::ConvertFloat(vertex.m_Position[0]);
		}

		content[offset++] = TString::ConvertInt(faceCount);
		for (int i = 0; i < faceCount; ++i)
		{
			const TATFaceBuffer& face = buffer.faceBuffer[i];
			content[offset++] = TString(" ") +
								TString::ConvertInt(face.v1 + 1) + " " +
								TString::ConvertInt(face.v2 + 1) + " " +
								TString::ConvertInt(face.v3 + 1);
		}

		TATFileStream::WriteFile(surfName, content);
	}

	static void FindFile(vector<std::string>& files)
	{
		WIN32_FIND_DATAA wfd;
		CString sPath = "*.obj";
		HANDLE hFile = FindFirstFile(sPath.GetBuffer(), &wfd);
		if (INVALID_HANDLE_VALUE == hFile)
			return;
		do 
		{
			files.push_back(std::string((LPCTSTR)wfd.cFileName));
		}
		while (FindNextFile(hFile, &wfd));

	}

};
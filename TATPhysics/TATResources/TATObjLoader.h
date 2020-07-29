#ifndef THEALMIGHTY_OBJLOADER
#define THEALMIGHTY_OBJLOADER

#include "TATModelLoader.h"
#include "../TATBasis/TString.h"

using namespace std;

class TATObjLoader:public TATModelLoader
{
public:
	TATObjLoader(const TString& filePath) :TATModelLoader(filePath)
	{
		Load(filePath,m_Buffer);
	}
	
	virtual void Load(const TString& name, TATModelBuffer& buffer)
	{
		TString app = TAT_APPNAME;
		TString filePath = TATPaths::PathOfModel(app, name);

		const int len = 100;
		ifstream objFile;
		objFile.open(filePath.m_Str, ios::in);
		if (!objFile.is_open())
		{
			std::cout << "file not exist!" << endl;
			return;
		}

		char* lineStrBuffer = new char[len];
		while (objFile.getline(lineStrBuffer, len).good())
		{
			TString lineStr(lineStrBuffer);
			std::vector<TString> splitResult;
			lineStr.Split(" ", splitResult);
			if (splitResult.size() == 0)
				continue;
			if (splitResult[0] == "v")
			{
				float x = splitResult[1].ToFloat();
				float y = splitResult[2].ToFloat();
				float z = splitResult[3].ToFloat();
				TATVertexBuffer vb(x, y, z);
				vb.isRendVert = true;
				buffer.vertexBuffer.push_back(vb);
			}
			else if (splitResult[0] == "vn")
			{
				float x = splitResult[1].ToFloat();
				float y = splitResult[2].ToFloat();
				float z = splitResult[3].ToFloat();
				buffer.normalBuffer.push_back(TATNormalBuffer(x, y, z));
			}
			else if (splitResult[0] == "vt")
			{
				float x = splitResult[1].ToFloat();
				float y = splitResult[2].ToFloat();
				float z = splitResult[3].ToFloat();
				buffer.texCoordinateBuffer.push_back(TATexCoordinateBuffer(x, y, z));
			}
			else if (splitResult[0] == "f")
			{
				std::vector<int> indexs;
				int size = 0;
				for (int i = 1; i < (int)splitResult.size(); i++)
				{
					std::vector<TString> datas;
					splitResult[i].Split("/", datas);
					size = datas.size();
					if (size > 0)
					{
						int index = datas[0].ToInt();
						indexs.push_back(index - 1);
					}
					if (size > 1)
					{
						int uvId = datas[1].ToInt();
						indexs.push_back(uvId - 1);
					}
					if (size > 2)
					{
						int nid = datas[2].ToInt();
						indexs.push_back(nid - 1);
					}

				}
				if ((int)indexs.size() > 3 * size)
				{
					for (int i = 0; i < size; i++)
					{
						indexs.push_back(indexs[0 * size + i]);
					}
					for (int i = 0; i < size; i++)
					{
						indexs.push_back(indexs[2 * size + i]);
					}
				}
				for (int i = 0; i < (int)indexs.size() / (3*size); i++)
				{
					if (size == 1)
						buffer.faceBuffer.push_back(TATFaceBuffer(indexs[i * 3 * size + 0 * size], indexs[i * 3 * size + 1 * size], indexs[i * 3 * size + 2 * size]));
					if (size == 3)
						buffer.faceBuffer.push_back(TATFaceBuffer(indexs[i * 3 * size + 0 * size],
						indexs[i * 3 * size + 1 * size],
						indexs[i * 3 * size + 2 * size],
						indexs[i * 3 * size + 0 * size + 1],
						indexs[i * 3 * size + 1 * size + 1],
						indexs[i * 3 * size + 2 * size + 1],
						indexs[i * 3 * size + 0 * size + 2],
						indexs[i * 3 * size + 1 * size + 2],
						indexs[i * 3 * size + 2 * size + 2]
						));
				}
			}
			else if (splitResult[0] == "t")
			{
				std::vector<int> indexs;
				for (int i = 1; i < (int)splitResult.size(); i++)
				{
					std::vector<TString> datas;
					splitResult[i].Split("/", datas);
					int index = datas[0].ToInt();
					indexs.push_back(index - 1);
				}
				for (int i = 0; i < (int)indexs.size() / 4; i++)
				{
					buffer.tetraBuffer.push_back(TATetraBuffer(indexs[i + 0], indexs[i + 1], indexs[i + 2], indexs[i + 3]));
				}
			}
		}

		if (buffer.normalBuffer.size() > 0)
		{
			for (int i = 0; i < (int)buffer.faceBuffer.size(); i++)
			{
				const TATFaceBuffer& face = buffer.faceBuffer[i];
				for (int c = 0; c < 3; c++)
				{
					buffer.vertexBuffer[face.v1].m_Normal[c] = buffer.normalBuffer[face.n1].m_Normal[c];
					buffer.vertexBuffer[face.v2].m_Normal[c] = buffer.normalBuffer[face.n2].m_Normal[c];
					buffer.vertexBuffer[face.v3].m_Normal[c] = buffer.normalBuffer[face.n3].m_Normal[c];
				}

			}
		}

		if (buffer.texCoordinateBuffer.size() > 0)
		{
			for (int i = 0; i < (int)buffer.faceBuffer.size(); i++)
			{
				const TATFaceBuffer& face = buffer.faceBuffer[i];
				for (int c = 0; c < 2; c++)
				{
					buffer.vertexBuffer[face.v1].m_TexCoordinates[0][c] = buffer.texCoordinateBuffer[face.uv1].m_TexCoordinate[c];
					buffer.vertexBuffer[face.v2].m_TexCoordinates[0][c] = buffer.texCoordinateBuffer[face.uv2].m_TexCoordinate[c];
					buffer.vertexBuffer[face.v3].m_TexCoordinates[0][c] = buffer.texCoordinateBuffer[face.uv3].m_TexCoordinate[c];
				}

			}
		}

		if (buffer.faceBuffer.size() > 0)
			m_ModelElementMask.UseFace();
		if (buffer.normalBuffer.size() > 0)
			m_ModelElementMask.UseNormal();
		if (buffer.texCoordinateBuffer.size() > 0)
			m_ModelElementMask.UseTexCoordinate();

		m_ModelElementMask.SetTexNum(1);
		//temply no texcoordinate and tangent
	}
};


#endif
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
		TString filePath = TATPaths::PathOfModel(TATApplication::Instance()->GetAppName(), name);

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
			std::vector<TString> splitResult = lineStr.Split(" ");
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
			else if (splitResult[0]=="vn")
			{
				float x = splitResult[1].ToFloat();
				float y = splitResult[2].ToFloat();
				float z = splitResult[3].ToFloat();
				buffer.normalBuffer.push_back(TATNormalBuffer(x, y, z));

			}
			else if (splitResult[0] == "f")
			{
				std::vector<int> indexs;
				int size = 0;
				for (int i = 1; i < splitResult.size(); i++)
				{
					std::vector<TString> datas = splitResult[i].Split("/");
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
				if (indexs.size() > 3 * size)
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
				for (int i = 0; i < indexs.size() / (3*size); i++)
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
				for (int i = 1; i < splitResult.size(); i++)
				{
					std::vector<TString> datas = splitResult[i].Split("/");
					int index = datas[0].ToInt();
					indexs.push_back(index - 1);
				}
				for (int i = 0; i < indexs.size() / 4; i++)
				{
					buffer.tetraBuffer.push_back(TATetraBuffer(indexs[i + 0], indexs[i + 1], indexs[i + 2], indexs[i + 3]));
				}
			}
		}

		if (buffer.normalBuffer.size()>0)
		{
			for (int i = 0; i < buffer.faceBuffer.size(); i++)
			{
				buffer.vertexBuffer[buffer.faceBuffer[i].v1].nx = buffer.normalBuffer[buffer.faceBuffer[i].n1].nx;
				buffer.vertexBuffer[buffer.faceBuffer[i].v2].nx = buffer.normalBuffer[buffer.faceBuffer[i].n2].nx;
				buffer.vertexBuffer[buffer.faceBuffer[i].v3].nx = buffer.normalBuffer[buffer.faceBuffer[i].n3].nx;
				buffer.vertexBuffer[buffer.faceBuffer[i].v1].ny = buffer.normalBuffer[buffer.faceBuffer[i].n1].ny;
				buffer.vertexBuffer[buffer.faceBuffer[i].v2].ny = buffer.normalBuffer[buffer.faceBuffer[i].n2].ny;
				buffer.vertexBuffer[buffer.faceBuffer[i].v3].ny = buffer.normalBuffer[buffer.faceBuffer[i].n3].ny;
				buffer.vertexBuffer[buffer.faceBuffer[i].v1].nz = buffer.normalBuffer[buffer.faceBuffer[i].n1].nz;
				buffer.vertexBuffer[buffer.faceBuffer[i].v2].nz = buffer.normalBuffer[buffer.faceBuffer[i].n2].nz;
				buffer.vertexBuffer[buffer.faceBuffer[i].v3].nz = buffer.normalBuffer[buffer.faceBuffer[i].n3].nz;
			}
		}
	}


};


#endif
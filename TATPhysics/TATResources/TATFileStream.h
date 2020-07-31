#pragma once
#include <iostream>
#include <fstream>
#include "../TATBasis/TString.h"

class TATFileStream
{
public:
	TATFileStream()
	{

	}

	static bool FileExists(const TString& path)
	{
		errno_t err;
		FILE* file;
		err = fopen_s(&file, path.ToChar(), "r");
		if (file)
			return true;
		else
			return false;
	}

	static void ReadFileToLines(const TString& path, std::vector<TString>& res)
	{
		const int len = 500;
		ifstream file;
		file.open(path.m_Str, ios::in);
		if (!file.is_open())
		{
			std::cout << "file not exist!" << endl;
			return;
		}

		char* lineStrBuffer = new char[len];
		while (file.getline(lineStrBuffer, len).good())
		{
			res.push_back(lineStrBuffer);
		}
	}

	static void WriteFile(const TString& path, const std::vector<TString>& res)
	{

	}

	static void AppendLine(const TString& path, const TString& line)
	{
		//TODO
	}
};
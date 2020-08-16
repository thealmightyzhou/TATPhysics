#pragma once
#include <iostream>
#include <fstream>
#include <io.h>
#include <Windows.h>
#include <stdio.h>
#include <direct.h>
#include <atlstr.h>
#include "../TATBasis/TString.h"

class TATFileStream
{
public:
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
		ofstream ofresult(path.ToChar(), ios::ate);
		for (int i = 0; i < res.size(); i++)
		{
			ofresult << res[i].ToChar() << endl;
		}
	}

	static void AppendLine(const TString& path, const TString& line)
	{
		//TODO
	}

	//extend: "obj","stl" or other extend name
	static void FetchAllFilesBelow(const TString& path,const TString& extend, vector<TString>& files)
	{
		WIN32_FIND_DATAA wfd;
		CString sPath = (TString("*.") + extend).ToChar();
		HANDLE hFile = FindFirstFile(sPath.GetBuffer(), &wfd);
		if (INVALID_HANDLE_VALUE == hFile)
			return;
		do
		{
			files.push_back(std::string((LPCTSTR)wfd.cFileName));
		} while (FindNextFile(hFile, &wfd));

	}
};
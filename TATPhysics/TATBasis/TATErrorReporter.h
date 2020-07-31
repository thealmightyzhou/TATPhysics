#pragma once
#include <iostream>
#include "../TATCommon/TATSingleton.h"
#include "../TATResources/TATPaths.h"
#include "../TATResources/TATFileStream.h"

class TATErrorReporter:public Singleton<TATErrorReporter>
{
public:
	TATErrorReporter() {}

	void ReportErr(const TString& msg)
	{
		std::cout << std::endl << msg.m_Str << std::endl;
	}

	void ReportFileMissed(const TString& fileName)
	{
		ReportErr(fileName + " not exists!");
	}

	void LogErr(const TString& msg)
	{
		TATFileStream::AppendLine(TATPaths::PathOfLog(), "err: " + msg);
	}

	void LogInfo(const TString& msg)
	{
		TATFileStream::AppendLine(TATPaths::PathOfLog(), "info: " + msg);
	}
};
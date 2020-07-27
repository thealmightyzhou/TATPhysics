#include "TString.h"
#include "../TATCommon/TATVector3.h"

void TString::ToVector3(TATVector3& v)
{
	std::vector<TString> arr = Split(TString(","));
	assert(arr.size() >= 3);
	v.X = arr[0].ToFloat();
	v.Y = arr[1].ToFloat();
	v.Z = arr[2].ToFloat();
}

TATVector3 TString::ToVector3()
{
	TATVector3 res;
	ToVector3(res);
	return res;
}

TString TString::operator+(const TString& str) const
{
	return TString(m_Str + str.m_Str);
}

TString TString::operator+(const std::string& str) const
{
	return TString(m_Str + str);
}

TString TString::operator+(const char* str) const
{
	return TString(m_Str + str);
}

TString operator+(const char* str0, const TString& str1)
{
	return TString(str0 + str1.m_Str);
}

TString operator+(const std::string& str0, const TString& str1)
{
	return TString(str0 + str1.m_Str);
}
#ifndef THEALMIGHTY_TSTRING
#define THEALMIGHTY_TSTRING

#include <string>
#include <vector>

class TATVector3;

using namespace std;
class TString
{
public:
	TString() {}

	TString(const char* str) :m_Str(str)
	{}

	TString(const TString& str) :m_Str(str.m_Str)
	{}

	TString(const std::string& str) :m_Str(str)
	{}

	inline TString Append(const TString& str)
	{
		return *this + str;
	}

	inline TString Append(const char* str)
	{
		return *this + str;
	}

	//combine filename with folder path like "d:/doc" into "d:/doc/a.obj"
	inline TString Visit(const TString& str)
	{
		return Append("/").Append(str);
	}

	//fetch filename from str like "d:/doc/a.obj" into a.obj
	inline TString FetchFileName() const
	{
		std::vector<TString> strs;
		Split("/", strs);
		if (strs.size() == 0)
		{
			return m_Str;
		}

		return strs[strs.size() - 1];
	}

	//input a.obj out a
	inline TString FetchInnerName() const
	{
		TString wholeName = FetchFileName();
		std::vector<TString> strs;
		Split(".", strs);
		return strs[0];
	}

	//in d:/a/b/c/d.obj return d:/a/b/c
	inline TString UpperDirect()
	{
		std::vector<TString> strs;
		Split("/", strs);
		
		for (int i = 1; i < (int)strs.size() - 1; i++)
		{
			strs[0] = strs[0].Visit(strs[i]);
		}

		return strs[0];
	}

	TString operator+(const TString& str) const
	{
		return TString(m_Str + str.m_Str);
	}

	TString operator+(const std::string& str) const
	{
		return TString(m_Str + str);
	}

	TString operator+(const char* str) const
	{
		return TString(m_Str + str);
	}

	inline bool operator<(const TString& str) const
	{
		return m_Str < str.m_Str;
	}

	void Split(const TString& delim, std::vector<TString>& strings) const
	{
		//char* context = new char[100];
		//std::vector<TString> res;
		//if ("" == m_Str) return res;

		//char* strs = new char[m_Str.length() + 1];
		//strcpy_s(strs, strlen(strs) + 1, m_Str.c_str());

		//char* d = new char[m_Str.length() + 1];
		//strcpy_s(d, strlen(d) + 2, delim.m_Str.c_str());
		//char* p = NULL;
		//p = strtok_s(strs, d, &context);
		//while (p) {
		//	string s = p;
		//	res.push_back(s);
		//	p = strtok_s(NULL, d, &context);
		//}

		//return res;

		if (m_Str == "")
			return;

		const char* del = delim.m_Str.c_str();
		const char* target = m_Str.c_str();

		int preTail = -1;
		int head = 0;
		int tail = delim.m_Str.length() - 1;
		int size = delim.m_Str.length();
		bool flag = false;
		while (tail < m_Str.length())
		{
			if (target[head] == del[0])
			{
				flag = true;
				for (int i = 0; i < size; i++)
				{
					if (target[head + i] != del[i])
						flag = false;
				}

				if (flag)
				{
					TString str = Take(target, preTail + 1, head - 1);
					if (str != delim && str != "")
						strings.push_back(str);
					preTail = tail;
				}
			}

			head++;
			tail++;
		}

		TString str = Take(target, preTail + 1, m_Str.length() - 1);
		if (str != "")
		{
			strings.push_back(str);
		}

		return;
	}

	const char* ToChar() const
	{
		return m_Str.c_str();
	}

	float ToFloat()
	{
		return (float)atof(ToChar());
	}

	int ToInt()
	{
		return atoi(ToChar());
	}

	void FromFloat(float f)
	{
		m_Str = std::to_string(f);
	}

	void FromInt(int i)
	{
		m_Str = std::to_string(i);
	}

	static TString ConvertInt(int i)
	{
		TString str;
		str.FromInt(i);
		return str;
	}

	static TString ConvertFloat(float f)
	{
		TString str;
		str.FromFloat(f);
		return str;
	}

	static TString Make(int i)
	{
		TString str;
		str.FromInt(i);
		return str;
	}

	static TString Make(float i)
	{
		TString str;
		str.FromFloat(i);
		return str;
	}

	TString Take(const char* c, int head, int tail) const
	{
		char* str = new char[tail - head + 2];
		for (int i = head; i <= tail; i++)
		{
			str[i - head] = c[i];
		}
		str[tail - head + 1] = '\0';
		return TString(str);
	}

	void ToVector3(TATVector3& v);

	TATVector3 ToVector3();

	bool operator==(const char* str) const
	{
		return m_Str == str;
	}

	bool operator==(const TString& str) const
	{
		return m_Str == str.m_Str;
	}

	bool operator!=(const TString& str) const
	{
		return !(*this == str);
	}

	bool IsEmpty() const
	{
		return m_Str == "";
	}

	bool Contains(const TString& str) const
	{
		if (m_Str.length() < str.m_Str.length())
			return;

		const char* del = str.m_Str.c_str();
		const char* target = m_Str.c_str();

		int preTail = -1;
		int head = 0;
		int tail = str.m_Str.length() - 1;
		int size = str.m_Str.length();
		bool flag = false;
		while (tail < m_Str.length())
		{
			if (target[head] == del[0])
			{
				flag = true;
				for (int i = 0; i < size; i++)
				{
					if (target[head + i] != del[i])
						flag = false;
				}

				if (flag)
				{
					return true;
				}
			}

			head++;
			tail++;
		}

		return false;
	}

	std::string m_Str;
};

TString operator+(const char* str0, const TString& str1);

TString operator+(const std::string& str0, const TString& str1);

#endif // !THEALMIGHTY_TSTRING
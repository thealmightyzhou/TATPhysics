#pragma once
#include <iostream>
#include <cassert>
using namespace std;

class TATMatrix
{
public:
	float* m_Data;
	int m_Row;
	int m_Col;

	TATMatrix(int r, int c)
	{
		m_Data = new float[r * c];
		m_Row = r;
		m_Col = c;
		int num = r * c;
		memset(m_Data, 0, num * sizeof(float));
	}

	TATMatrix(const TATMatrix& m)
	{
		m_Row = m.m_Row;
		m_Col = m.m_Col;
		memcpy(m_Data, m.m_Data, sizeof(float) * m_Row * m_Col);
	}

	float Get(int row,int col) const
	{
		assert(0 <= row && row < m_Row);
		assert(0 <= col && col < m_Col);

		return m_Data[row * m_Col + col];
	}

	float& operator()(int row, int col)
	{
		assert(0 <= row && row < m_Row);
		assert(0 <= col && col < m_Col);

		return m_Data[row * m_Col + col];
	}

	TATMatrix Row(int r) const
	{
		assert(0 <= r && r < m_Row);
		TATMatrix rmat(1, m_Col);
		for (int i = 0; i < m_Col; ++i)
		{
			rmat(0, i) = Get(r, i);
		}
		return rmat;
	}

	TATMatrix Col(int c) const
	{
		assert(0 <= c && c < m_Col);
		TATMatrix cmat(m_Row, 1);
		for (int i = 0; i < m_Row; ++i)
		{
			cmat(i, 0) = Get(i, c);
		}
		return cmat;
	}

	TATMatrix operator*(const TATMatrix& m) const
	{
		assert(m_Col == m.m_Row);

		int cnt = m_Col;

		TATMatrix res(m_Row, m.m_Col);
		
		for (int r = 0; r < m_Row; ++r)
		{
			for (int c = 0; c < m.m_Col; ++c)
			{
				for (int i = 0; i < cnt; ++i)
				{
					res(r, c) += Get(r, i) * m.Get(i, c);
				}
			}
		}

		return res;
	}

	TATMatrix Inverse() const
	{
		assert(m_Row == m_Col);

	}

};
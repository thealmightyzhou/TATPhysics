#pragma once
#include <iostream>
#include <cassert>
#include "../TATBasis/TATErrorReporter.h"
#include "../TATBasis/TString.h"
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

	TATMatrix(int r, int c, float* m)
	{
		m_Data = new float[r * c];
		m_Row = r;
		m_Col = c;
		memcpy(m_Data, m, sizeof(float) * r * c);
	}

	TATMatrix(const TATMatrix& m)
	{
		m_Row = m.m_Row;
		m_Col = m.m_Col;
		m_Data = new float[m_Row * m_Col];
		memcpy(m_Data, m.m_Data, sizeof(float) * m_Row * m_Col);
	}

	~TATMatrix()
	{
		delete m_Data;
	}

	TATMatrix Zero() const
	{
		static TATMatrix zero(1, 1);
		zero.Set(0, 0, 0);
		return zero;
	}

	float Get(int row, int col) const
	{
		assert(0 <= row && row < m_Row);
		assert(0 <= col && col < m_Col);

		return m_Data[row * m_Col + col];
	}

	void Set(int row, int col, float val)
	{
		assert(0 <= row && row < m_Row);
		assert(0 <= col && col < m_Col);

		m_Data[row * m_Col + col] = val;
	}

	float& operator()(int row, int col)
	{
		assert(0 <= row && row < m_Row);
		assert(0 <= col && col < m_Col);

		return m_Data[row * m_Col + col];
	}

	float operator[](int i)
	{
		assert(0 <= i && i < m_Row* m_Col);
		return m_Data[i];
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

	TATMatrix operator * (float v) const
	{
		TATMatrix res(m_Row, m_Col);
		for (int i = 0; i < m_Row; ++i)
		{
			for (int j = 0; j < m_Col; ++j)
			{
				res(i, j) = v * Get(i, j);
			}
		}
		return res;
	}

	TATMatrix operator / (float v) const
	{
		assert(v != 0);

		float inv = 1.0f / v;
		return this->operator*(inv);
	}

	TATMatrix Inverse() const
	{
		assert(m_Row == m_Col);

		TATMatrix adj = Adjoint();
		float det = Determinant();
		if (det == 0)
		{
			TATErrorReporter::Instance()->ReportErr("divide by zero!! this matrix can not inverse");
			return Zero();
		}
		return adj / det;
	}

	float Determinant() const
	{
		assert(m_Row == m_Col);

		return InternalDeterminant(m_Row, this);
	}

	TATMatrix Adjoint() const
	{
		assert(m_Row == m_Col);

		TATMatrix adj(m_Row, m_Col);

		InternalAdjoint(*this, m_Row, adj);
		return adj;
	}

	TATMatrix Transpose() const
	{
		TATMatrix m(m_Col, m_Row);
		for (int i = 0; i < m_Row; ++i)
		{
			for (int j = 0; j < m_Col; ++j)
			{
				m(j, i) = Get(i, j);
			}
		}
		return m;
	}

	void OutPut() const
	{
		for (int i = 0; i < m_Row; ++i)
		{
			TString str("");
			for (int j = 0; j < m_Col; ++j)
			{
				str = str.Append(TString::ConvertFloat(Get(i, j)) + " ");
			}
			TATErrorReporter::Instance()->ReportErr(str);
		}
	}
private:

	static float InternalDeterminant(int n, const TATMatrix* m)
	{
		if (n == 1)
			return m->Get(0, 0);
		if (n == 2)
		{
			return (m->Get(0, 0) * m->Get(1, 1) - m->Get(0, 1) * m->Get(1, 0));
		}

		TATMatrix* sub_mat = new TATMatrix(n - 1, n - 1);
		int mov = 0;
		float sum = 0.0f;

		for (int row = 0; row < n; ++row)
		{
			for (int subMatrow = 0; subMatrow < n - 1; ++subMatrow)//把Mat阵第一列各元素的代数余子式存到subMat  
			{
				mov = subMatrow < row ? 0 : 1; //subMat中小于Matrow的行，同行赋值，等于的错过，大于的加一  
				for (int j = 0; j < n - 1; j++)  //从Mat的第二列赋值到第n列  
				{
					sub_mat->Set(subMatrow, j, m->Get(subMatrow + mov, j + 1));
				}
			}
			int flag = (row % 2 == 0 ? 1 : -1);//因为列数为0，所以行数是偶数时候，代数余子式为1.  
			sum += flag * m->Get(row, 0) * InternalDeterminant(n - 1, sub_mat);//Mat第一列各元素与其代数余子式积的和即为行列式
		}

		delete sub_mat;

		return sum;
	}

	static void InternalAdjoint(const TATMatrix& arcs, int n, TATMatrix& ans)
	{
		if (n == 1)
		{
			ans(0, 0) = 1;
			return;
		}
		int i, j, k, t;
		TATMatrix temp(n - 1, n - 1);
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				for (k = 0; k < n - 1; k++)
				{
					for (t = 0; t < n - 1; t++)
					{
						temp(k, t) = arcs.Get(k >= i ? k + 1 : k, t >= j ? t + 1 : t);
					}
				}


				ans(j, i) = temp.Determinant();
				if ((i + j) % 2 == 1)
				{
					ans(j, i) = -ans.Get(j, i);
				}
			}
		}
	}
};
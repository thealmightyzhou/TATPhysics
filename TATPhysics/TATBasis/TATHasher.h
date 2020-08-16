#pragma once
#include <iostream>

using namespace std;

class TATHasher
{
public:

	//make sure hash(x,y) is same as hash(y,x)
	static int HashTwo(int x, int y)
	{
		int xy = x * y;
		int x_y = x + y;

		std::hash<int> hasher;
		size_t seed = 0;

		seed ^= hasher(xy) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hasher(x_y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

		return seed;
	}

	static int HashThree(int x, int y, int z)
	{
		return HashTwo(x + y + z, x * y * z);
	}
};
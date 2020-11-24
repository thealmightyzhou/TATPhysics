#pragma once
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "cuda_runtime.h"  
#include "device_launch_parameters.h"  
#include "../TATBroadPhase/LBVH.h"

class CUDADemo
{
public:
	int CUDAHelloWorld();
	void CUDAVectorAdd();
	void CUDAVectorDot();
	void DimNum();

	void CUDAGetMinMax();

	void LBVHTest();
};